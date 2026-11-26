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
The bridge and the fault manager starting in either order.

Only the bridge is launched. The test starts and stops the fault manager
and a second bridge itself, so each order is real processes:

1. A raise made while no fault manager runs is CONFIRMED once one starts
   (held, not dropped).
2. A fault left CONFIRMED by a bridge that is gone heals when a new bridge
   starts: it passes every code once at start-up (with the healing settings
   of HEALING_FAULT_MANAGER).
3. A fault manager killed (SIGKILL) and started again at once gets the raise
   that is still active, although the dead one's endpoints keep the service
   looking ready.

Worst case at scale 1, every wait running out (a wait_for overruns by one
ListFaults call of 2 s and a 0.2 s pause): test_1 44 s (bridge 15, 1 s pause,
fault manager 15, confirm 12.2); test_2 49.4 s (service 10, call 10, stale
fault 12.2, heal 17.2); test_3 32.2 s (back within 30 + overrun); stops
40 s (three processes at 10 s, the launched bridge 10 s). Sum 166 s; TIMEOUT
200. At MEDKIT_TEST_TIME_SCALE 3 every budget triples (498 s), and so does
TIMEOUT in the sanitizer jobs (600 s).
"""

import os
import signal
import subprocess
import sys
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault
from ros2_medkit_policy_msgs.msg import PolicyIntrospection

# The helpers sit next to this file; launch_test does not put its folder on the path.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from policy_test_utils import (  # noqa: E402, I100
    FaultReader, HEALING_FAULT_MANAGER, LOW_CODE, message, SLOW_CODE, time_scale, TOPIC,
)

WAIT = 10.0 * time_scale()


def _executable(package, name):
    return os.path.join(get_package_prefix(package), 'lib', package, name)


def generate_test_description():
    bridge = launch_ros.actions.Node(
        package='ros2_medkit_policy_bridge',
        executable='policy_bridge_node',
        name='policy_bridge',
        output='screen',
        parameters=[{'spread_threshold': 0.05, 'raise_after': 3}],
        sigterm_timeout='5',
        sigkill_timeout='5',
    )
    return LaunchDescription([bridge, launch_testing.actions.ReadyToTest()])


class TestFaultManagerLifecycle(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_policy')
        cls.pub = cls.node.create_publisher(PolicyIntrospection, TOPIC, 10)
        cls.faults = FaultReader(cls.node)
        cls.report = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.procs = []

    @classmethod
    def tearDownClass(cls):
        for proc in cls.procs:
            cls._stop(proc)
        cls.node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _stop(proc):
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()

    def _start(self, cmd):
        proc = subprocess.Popen(cmd)
        self.procs.append(proc)
        return proc

    def _wait_for_bridge(self, count):
        deadline = time.monotonic() + 15.0 * time_scale()
        while time.monotonic() < deadline:
            if self.pub.get_subscription_count() >= count:
                return True
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return False

    def test_1_raise_before_the_fault_manager_starts(self):
        self.assertTrue(self._wait_for_bridge(1))
        self.assertFalse(self.faults.wait_for_service(0.5), 'no fault manager may run yet')
        for k in range(8):
            self.pub.publish(message(0, 5 * k, 0.2 if k % 2 == 0 else -0.2))
            time.sleep(0.05)
        # The raise is made and waits; nothing can receive it.
        time.sleep(1.0 * time_scale())

        self._start([_executable('ros2_medkit_fault_manager', 'fault_manager_node'), '--ros-args',
                     '-r', '__node:=fault_manager',
                     *[a for k, v in HEALING_FAULT_MANAGER.items() for a in ('-p', f'{k}:={v}')]])
        self.assertTrue(self.faults.wait_for_service(15.0 * time_scale()),
                        'fault manager did not start')
        fault = self.faults.wait_for(LOW_CODE, Fault.STATUS_CONFIRMED, WAIT)
        self.assertIsNotNone(fault, 'the raise made before the fault manager started was lost')
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        self.assertIn('(episode 0, step 15)', fault.description)

    def test_3_fault_manager_killed_and_restarted(self):
        """
        The fault manager dies (SIGKILL, memory storage) and a new one starts at once.

        The raise that is still active must reach the new one.
        """
        fm = self.procs[0]
        fm.kill()
        fm.wait()
        start = time.monotonic()
        self._start([_executable('ros2_medkit_fault_manager', 'fault_manager_node'), '--ros-args',
                     '-r', '__node:=fault_manager',
                     *[a for k, v in HEALING_FAULT_MANAGER.items() for a in ('-p', f'{k}:={v}')]])
        fault = self.faults.wait_for(LOW_CODE, Fault.STATUS_CONFIRMED, 30.0 * time_scale())
        print(f'raise back after {time.monotonic() - start:.1f} s')
        self.assertIsNotNone(fault, 'the active raise did not reach the restarted fault manager')
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)

    def test_2_new_bridge_heals_a_fault_left_by_the_last_one(self):
        # A fault as an earlier bridge left it, on another topic so the
        # launched bridge does not see this test's messages.
        request = ReportFault.Request()
        request.fault_code = 'OLD_' + SLOW_CODE
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_WARN
        request.description = 'left by a bridge that is gone'
        request.source_id = '/old_policy'
        self.assertTrue(self.report.wait_for_service(timeout_sec=WAIT))
        future = self.report.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=WAIT)
        stale = self.faults.wait_for('OLD_' + SLOW_CODE, Fault.STATUS_CONFIRMED, WAIT)
        self.assertIsNotNone(stale)
        self.assertEqual(stale.status, Fault.STATUS_CONFIRMED)

        self._start([_executable('ros2_medkit_policy_bridge', 'policy_bridge_node'), '--ros-args',
                     '-r', '__node:=policy_bridge_old', '-p', 'code_prefix:=OLD_POLICY',
                     '-p', 'introspection_topic:=/old_policy/introspection',
                     '-p', 'max_inference_ms:=50.0'])
        # No publisher on its topic: it reports under the topic name after its
        # 5 s grace, so the heal can take that long.
        healed = self.faults.wait_for('OLD_' + SLOW_CODE, Fault.STATUS_HEALED, WAIT + 5.0)
        self.assertIsNotNone(healed)
        self.assertEqual(healed.status, Fault.STATUS_HEALED,
                         'a stale fault stayed CONFIRMED after the bridge restarted')
