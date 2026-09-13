# Copyright 2026 bburda
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
Integration tests for a node that leaves the ROS graph while discovery runs.

A discovery pass lists node names first and then asks the graph about each
name. Anything can happen between those two steps, and what the second step
answers for a name that is no longer a node is either an rcl error or an empty
set of endpoints. Both mean the same thing and both are ordinary, so the
gateway has to keep serving across them.

Two scenarios:

1. Churn. A demo node is started and signalled repeatedly at an interval short
   enough to land the signal inside the node's own start-up, which is the
   interval that puts departures inside discovery passes. Afterwards the
   gateway still answers and the node is gone from ``/apps``; its exit code is
   checked by the shared shutdown case.

2. A minimal live node is still an App. ``silent_node`` runs with parameter
   services and the parameter-event publisher switched off, keeping only its
   ``/rosout`` publisher - the one endpoint rclcpp creates the same way on
   every supported distro. The case names that endpoint through an rclpy probe
   before asking the gateway, so a pass means "the graph attributes an endpoint
   to this node AND the gateway lists it" rather than either half alone. This
   is the control on the rule that removes names the graph attributes no
   endpoint to: it may only remove names that are gone, never a node that is
   merely quiet.
"""

import os
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import rclpy

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DEFAULT_DOMAIN_ID,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
    DEMO_NODE_REGISTRY,
    get_coverage_env,
)

# The node that is started and signalled over and over. It carries a service,
# so a pass that catches it half-way up or half-way down has per-node queries
# to run against it rather than skipping it on the name alone.
CHURN_NODE_KEY = 'calibration'

# Cycles, and how long each one lets the node live. A ROS node needs about a
# second to finish announcing itself, so a fraction of that puts the signal
# inside start-up: the departure then lands while the gateway is mid-pass over
# a graph that still names the node. Thirty cycles at that interval is what
# opens the window often enough to be worth running; the count is not a
# statistical claim.
CHURN_CYCLES = 30
CHURN_NODE_LIFETIME_SEC = 0.1

# A churned process is signalled during start-up, so it may have to finish
# coming up before it can act on the signal.
CHURN_NODE_EXIT_TIMEOUT_SEC = 30.0 * get_time_scale()

# What a departure costs once the process is gone: the DDS participant lease
# plus one gateway refresh. Stated in the gateway's own configuration
# reference; repeated here only as a budget.
DEPARTURE_TIMEOUT_SEC = 25.0 * get_time_scale()

# Start-up budget for the quiet node, measured from the gateway side.
SILENT_NODE_TIMEOUT_SEC = 30.0 * get_time_scale()

SILENT_NODE_NAME = 'silent_node'

# The endpoint the quiet node is guaranteed to own. rclcpp creates the /rosout
# publisher for every node on every supported distro unless enable_rosout is
# turned off, and this fixture leaves it on precisely so the control has an
# endpoint that does not depend on the distro.
SILENT_NODE_ENDPOINT = '/rosout'

# The probe reads the graph directly, so it is bounded by DDS discovery rather
# than by a gateway refresh.
PROBE_TIMEOUT_SEC = 30.0 * get_time_scale()
PROBE_INTERVAL_SEC = 0.5


def generate_test_description():
    gateway_node = create_gateway_node(
        extra_params={
            # Short backstop and short debounce: a pass then starts often
            # enough that the churn below runs concurrently with one.
            'refresh_interval_ms': 500,
            'discovery.refresh_debounce_ms': 100,
        },
    )

    delayed = TimerAction(
        period=2.0,
        actions=create_demo_nodes(nodes=['silent'], lidar_faulty=False),
    )

    return (
        LaunchDescription([
            gateway_node,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


def _resolve_demo_executable(name):
    pkg = 'ros2_medkit_integration_tests'
    candidate = os.path.join(get_package_prefix(pkg), 'lib', pkg, name)
    if not os.path.isfile(candidate):
        raise FileNotFoundError(f'demo executable not found: {candidate}')
    return candidate


class TestDepartedNodeDiscovery(GatewayTestCase):
    """A node leaving mid-pass must not cost the gateway anything."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls._churn_procs = []

    @classmethod
    def tearDownClass(cls):
        for proc in cls._churn_procs:
            if proc.poll() is None:
                proc.kill()
                proc.wait(timeout=10)
        cls._churn_procs = []
        super().tearDownClass()

    @classmethod
    def _spawn(cls, key):
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[key]
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
        env.update(get_coverage_env())
        proc = subprocess.Popen(
            [
                _resolve_demo_executable(executable),
                '--ros-args',
                '-r', f'__ns:={namespace}',
                '-r', f'__node:={ros_name}',
            ],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        cls._churn_procs.append(proc)
        return proc

    def _app_ids(self):
        data = self.get_json('/apps')
        return [app['id'] for app in data.get('items', [])]

    def _poll_probe_publishers(self, node_name):
        """Topics the graph attributes to `node_name`, read with rclpy.

        Asked of the graph rather than of the gateway on purpose: the gateway is
        the thing under test, so the anti-vacuity check for this case cannot
        come from it.
        """
        rclpy.init()
        try:
            probe = rclpy.create_node('departed_node_discovery_probe')
            try:
                deadline = time.monotonic() + PROBE_TIMEOUT_SEC
                topics = []
                while time.monotonic() < deadline:
                    for name, namespace in probe.get_node_names_and_namespaces():
                        if name != node_name:
                            continue
                        topics = [
                            topic for topic, _ in
                            probe.get_publisher_names_and_types_by_node(name, namespace)
                        ]
                        if topics:
                            return topics
                    time.sleep(PROBE_INTERVAL_SEC)
                return topics
            finally:
                probe.destroy_node()
        finally:
            rclpy.shutdown()

    def test_01_gateway_serves_through_node_churn(self):
        """Repeated departures mid-pass leave the gateway answering."""
        _, ros_name, _ = DEMO_NODE_REGISTRY[CHURN_NODE_KEY]

        for cycle in range(CHURN_CYCLES):
            proc = self._spawn(CHURN_NODE_KEY)
            time.sleep(CHURN_NODE_LIFETIME_SEC)
            proc.send_signal(signal.SIGTERM)
            try:
                proc.wait(timeout=CHURN_NODE_EXIT_TIMEOUT_SEC)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=10)
                self.fail(
                    f'churn cycle {cycle}: demo node did not act on SIGTERM within '
                    f'{CHURN_NODE_EXIT_TIMEOUT_SEC}s'
                )

        health = self.get_json('/health')
        self.assertEqual(
            health.get('status'), 'healthy', f'gateway unhealthy after churn: {health}')

        departed = self.poll_endpoint_until(
            '/apps',
            lambda data: data if ros_name not in [
                app['id'] for app in data.get('items', [])
            ] else None,
            timeout=DEPARTURE_TIMEOUT_SEC,
        )
        self.assertIsNotNone(
            departed,
            f"'{ros_name}' still listed {DEPARTURE_TIMEOUT_SEC}s after the last cycle exited",
        )

    def test_02_a_node_with_no_parameter_services_is_still_an_app(self):
        """The endpoint rule removes names that are gone, not quiet nodes."""
        publishers = self._poll_probe_publishers(SILENT_NODE_NAME)
        self.assertIn(
            SILENT_NODE_ENDPOINT, publishers,
            f"the graph attributes no '{SILENT_NODE_ENDPOINT}' publisher to "
            f"'{SILENT_NODE_NAME}' ({publishers}), so this case would pass on a "
            'gateway that lists nothing at all',
        )

        listed = self.poll_endpoint_until(
            '/apps',
            lambda data: data if SILENT_NODE_NAME in [
                app['id'] for app in data.get('items', [])
            ] else None,
            timeout=SILENT_NODE_TIMEOUT_SEC,
        )
        self.assertIsNotNone(
            listed,
            'a node with parameter services and the parameter-event publisher '
            f'off must still be an App; /apps held {self._app_ids()}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """The gateway exits cleanly after a run full of departing nodes."""

    def test_exit_codes(self, proc_info, gateway_node):
        exit_code = proc_info[gateway_node].returncode
        self.assertIn(
            exit_code,
            ALLOWED_EXIT_CODES,
            f'Process gateway_node exited with {exit_code}',
        )
