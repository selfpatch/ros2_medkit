#!/usr/bin/env python3
# Copyright 2026 ahmedsleem109
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
End-to-end tests for ros2_medkit_policy_bridge through a real fault manager.

The tests run in order and each starts a new episode. Every check that
something was NOT raised reads the fault in every status and its occurrence
count, after a sentinel (slow readings in the same episode) proves the bridge
has processed the messages before it.

Worst case at scale 1, every wait running out (a wait_for overruns by one
ListFaults call of 2 s and a 0.2 s pause): service 10 s; test_01 30 s (bridge
15, sentinel 12.4, one read 2); test_02 39 s (three waits of 12.2 or 12.4, one
read); test_03 12.4 s; test_04 14.5 s; test_05 35 s (sentinel, one read, two
output waits of 10); parameter checks 40 s (four output waits); stop 10 s.
Sum 191 s; TIMEOUT 240. At MEDKIT_TEST_TIME_SCALE 3 every budget triples
(573 s), and so does TIMEOUT in the sanitizer jobs (720 s).
"""

import math
import os
import sys
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_policy_msgs.msg import PolicyIntrospection

# The helpers sit next to this file; launch_test does not put its folder on the path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_test_utils import (  # noqa: E402, I100
    FaultReader, HEALING_FAULT_MANAGER, LOW_CODE, message, SLOW_CODE, stamp, time_scale, TOPIC,
)

WAIT = 10.0 * time_scale()


def generate_test_description():
    """Launch description with fault_manager and policy_bridge nodes."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[HEALING_FAULT_MANAGER],
        sigterm_timeout='5',
        sigkill_timeout='5',
    )

    # The packaged config first, so the test also guards the shipped YAML;
    # then the thresholds a real deployment would measure on its policy.
    config_file = os.path.join(
        get_package_share_directory('ros2_medkit_policy_bridge'),
        'config', 'policy_bridge.yaml')
    policy_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_policy_bridge',
        executable='policy_bridge_node',
        name='policy_bridge',
        output='screen',
        parameters=[config_file, {
            'spread_threshold': 0.05,
            'max_inference_ms': 50,  # an integer, as people write it
            'raise_after': 3,
        }],
        sigterm_timeout='5',
        sigkill_timeout='5',
    )

    # Bad values are named in a warning; the node keeps running.
    bad_params_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_policy_bridge',
        executable='policy_bridge_node',
        name='policy_bridge_bad_params',
        output='screen',
        parameters=[{
            'introspection_topic': '/unused/introspection',
            'raise_after': 4294967298,
            'chunk_disagreement_severity': 7,
            'joint_scale': [1.0, -2.0],
            'spread_threshold': 'high',
        }],
        sigterm_timeout='5',
        sigkill_timeout='5',
    )

    return (
        LaunchDescription([
            fault_manager_node,
            policy_bridge_node,
            bad_params_bridge_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'policy_bridge_node': policy_bridge_node,
            'bad_params_bridge_node': bad_params_bridge_node,
        },
    )


class TestPolicyBridgeIntegration(unittest.TestCase):
    """Policy introspection -> faults through the real fault manager."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_policy')
        cls.pub = cls.node.create_publisher(PolicyIntrospection, TOPIC, 10)
        cls.faults = FaultReader(cls.node)
        assert cls.faults.wait_for_service(WAIT), 'ListFaults service not available'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def wait_for_bridge(self):
        """
        Wait until the bridge's subscription is matched.

        The bridge names the publisher's node as the fault source, so the test
        node's name must have reached it before any fault.
        """
        deadline = time.monotonic() + 15.0 * time_scale()
        while time.monotonic() < deadline:
            infos = self.node.get_subscriptions_info_by_topic(TOPIC)
            if any(info.node_name == 'policy_bridge' for info in infos) and \
                    self.pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def publish(self, messages):
        for msg in messages:
            self.pub.publish(msg)
            # Paced so a best-effort subscriber keeps every message.
            time.sleep(0.05)

    def sentinel(self, episode, first_step):
        """Publish slow readings; once SLOW confirms, the messages before them were processed."""
        since = time.time()
        self.publish([message(episode, first_step + 5 * k, 0.0, inference_ms=120.0)
                      for k in range(3)])
        fault = self.faults.wait_for(SLOW_CODE, Fault.STATUS_CONFIRMED, WAIT, since)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED, 'sentinel not processed')
        self.assertGreaterEqual(stamp(fault.last_occurred), since)

    def test_01_calm_policy_raises_nothing(self):
        """Agreeing predictions at normal speed leave no CHUNK_DISAGREEMENT."""
        self.assertTrue(self.wait_for_bridge(), 'bridge did not subscribe in time')
        self.publish([message(0, 5 * k, 0.0, inference_ms=10.0) for k in range(10)])
        self.sentinel(0, 50)
        self.assertIsNone(self.faults.get(LOW_CODE))

    def test_02_unsure_policy_raises_chunk_disagreement_once(self):
        """Predictions that keep changing raise one WARN fault on the publisher."""
        self.publish([message(1, 5 * k, 0.2 if k % 2 == 0 else -0.2, inference_ms=10.0)
                      for k in range(8)])
        fault = self.faults.wait_for(LOW_CODE, Fault.STATUS_CONFIRMED, WAIT)
        self.assertIsNotNone(fault, f'expected a {LOW_CODE} fault')
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(fault.severity, Fault.SEVERITY_WARN)
        self.assertIn('/test_policy', list(fault.reporting_sources))
        # Episode 1 passed the sentinel's SLOW_INFERENCE from episode 0.
        slow = self.faults.wait_for(SLOW_CODE, Fault.STATUS_HEALED, WAIT)
        self.assertIsNotNone(slow)
        self.assertEqual(slow.status, Fault.STATUS_HEALED)
        # Eight unsure readings, one report: the raise at the third spread
        # (step 15). A re-report replaces the description, so a bridge that
        # sent FAILED on every reading above the limit would show step 35.
        # The sentinel proves every reading before it was handled.
        self.sentinel(1, 40)
        fault = self.faults.get(LOW_CODE)
        self.assertIn('for 3 readings in a row (episode 1, step 15)', fault.description)

    def test_03_new_episode_heals_it(self):
        """The next episode passes the fault."""
        self.publish([message(2, 5 * k, 0.0, inference_ms=10.0) for k in range(3)])
        fault = self.faults.wait_for(LOW_CODE, Fault.STATUS_HEALED, WAIT)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_HEALED, f'{LOW_CODE} not HEALED in time')

    def test_04_slow_inference_raises_its_own_fault(self):
        """Inference over max_inference_ms raises SLOW_INFERENCE, not CHUNK_DISAGREEMENT."""
        since = time.time()
        self.publish([message(3, 5 * k, 0.0, inference_ms=120.0) for k in range(5)])
        fault = self.faults.wait_for(SLOW_CODE, Fault.STATUS_CONFIRMED, WAIT, since)
        self.assertIsNotNone(fault)
        self.assertGreaterEqual(stamp(fault.last_occurred), since)
        self.assertIn('120.0 ms', fault.description)
        low = self.faults.get(LOW_CODE)
        self.assertEqual((low.status, low.occurrence_count), (Fault.STATUS_HEALED, 1))

    def test_05_bad_messages_change_nothing(self, proc_output, policy_bridge_node):
        """
        Wrong-size and NaN chunks are ignored, not counted, and said once per kind.

        Each kind comes in its own run, 2 steps apart with alternating noise:
        used, either run would raise CHUNK_DISAGREEMENT.
        """
        since = time.time()
        short = []
        for k in range(6):
            msg = message(4, 2 * k, 0.2 if k % 2 == 0 else -0.2)
            msg.chunk = msg.chunk[:-1]
            short.append(msg)
        nan = []
        for k in range(6):
            msg = message(4, 12 + 2 * k, 0.2 if k % 2 == 0 else -0.2)
            msg.chunk[0] = math.nan
            nan.append(msg)
        self.publish(short + nan)
        # Episode 4 passed SLOW_INFERENCE; the sentinel raises it again.
        self.sentinel(4, 30)
        low = self.faults.get(LOW_CODE)
        self.assertIsNotNone(low)
        self.assertEqual(low.status, Fault.STATUS_HEALED)
        self.assertLess(stamp(low.last_occurred), since, 'a bad message raised CHUNK_DISAGREEMENT')
        for fragment in ('values, expected chunk_length x action_dim = 20',
                         'chunk holds NaN or inf'):
            with self.subTest(fragment=fragment):
                proc_output.assertWaitFor(fragment, process=policy_bridge_node, timeout=WAIT)


class TestPolicyBridgeParameters(unittest.TestCase):
    """Bad parameters are named in a warning and the node keeps running."""

    def test_bad_parameters_are_reported(self, proc_output, bad_params_bridge_node):
        for fragment in (
            'raise_after 4294967298 is outside',
            'chunk_disagreement_severity 7 is outside [0, 3]',
            'joint_scale holds a value that is not a positive number',
            'spread_threshold is string, expected a number: using 0',
        ):
            with self.subTest(fragment=fragment):
                proc_output.assertWaitFor(fragment, process=bad_params_bridge_node, timeout=WAIT)


@launch_testing.post_shutdown_test()
class TestPolicyBridgeShutdown(unittest.TestCase):
    """Test clean shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
