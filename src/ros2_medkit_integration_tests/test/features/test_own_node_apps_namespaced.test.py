#!/usr/bin/env python3
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

"""The helper-node filter holds when the gateway alone is moved to a namespace.

``test_own_node_apps.test.py`` covers the case where all four of the gateway's
nodes share one namespace, where the three helper names are simply the
gateway's plus a suffix. That is not the only shape they take. The subscription
node is created with the gateway's namespace, but the fault-client and
lifecycle-reader nodes are created from the gateway's node NAME alone, so they
take the process default namespace - and a remap naming the gateway alone,
``-r ros2_medkit_gateway:__ns:=/subsystem_b``, moves the gateway and the
subscription node while leaving those two in the root namespace.

The gateway must still recognise all three. This launch is the split
invocation, with one gateway on the graph so nothing else can supply a node by
those names.
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.coverage import get_coverage_env

GATEWAY_PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{GATEWAY_PORT}{API_BASE_PATH}'

GATEWAY_NAME = 'ros2_medkit_gateway'
GATEWAY_NS = '/subsystem_b'
GATEWAY_FQN = f'{GATEWAY_NS}/{GATEWAY_NAME}'
# The split: one helper follows the gateway, two stay at the root.
NAMESPACED_HELPER_FQN = f'{GATEWAY_NS}/{GATEWAY_NAME}_sub'
ROOT_HELPER_FQNS = (
    f'/{GATEWAY_NAME}_fault_clients',
    f'/{GATEWAY_NAME}_lifecycle_state_reader',
)
ALL_HELPER_FQNS = (NAMESPACED_HELPER_FQN,) + ROOT_HELPER_FQNS
WITNESS_NODE = 'own_node_apps_ns_witness'
PROBE_NODE = 'own_node_apps_ns_probe'
# This file's own two nodes are ordinary ROS nodes, so the gateway lists them
# as apps like any other. Naming them keeps the assertion an exact set rather
# than a membership test that would not notice a helper slipping back in.
TEST_OWN_NODES = {WITNESS_NODE, PROBE_NODE}

HEALTH_BUDGET = 30.0
HELPERS_ON_GRAPH_BUDGET = 20.0
REFRESH_WITNESS_BUDGET = 25.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch one gateway with only its own node moved into a namespace.

    Built here because the remap has to name the gateway node, which
    ``create_gateway_node`` cannot express: a namespace on the launch action
    moves all four nodes together, and that case is the sibling file's.
    """
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name=None,
        output='screen',
        parameters=[{
            'server.host': '127.0.0.1',
            'server.port': GATEWAY_PORT,
            'refresh_interval_ms': 1000,
        }],
        ros_arguments=['-r', f'{GATEWAY_NAME}:__ns:={GATEWAY_NS}'],
        additional_env=dict(get_coverage_env()),
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


class TestOwnNodeAppsNamespaced(unittest.TestCase):
    """A gateway moved on its own still filters all three of its helpers."""

    @classmethod
    def setUpClass(cls):
        """Wait for the gateway, then settle /apps against the helper nodes."""
        cls.session = requests.Session()
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.probe = Node(PROBE_NODE, context=cls.context)
        cls.witness = None
        cls.apps = set()
        cls.graph_fqns = set()
        cls.witness_seen = False

        cls._wait_for_health()
        cls.graph_fqns = cls._graph_fqns_until(set(ALL_HELPER_FQNS), HELPERS_ON_GRAPH_BUDGET)
        if not set(ALL_HELPER_FQNS) <= cls.graph_fqns:
            return

        cls.witness = Node(WITNESS_NODE, context=cls.context)
        cls.witness_seen, cls.apps = cls._apps_until_contains(
            WITNESS_NODE, REFRESH_WITNESS_BUDGET)

    @classmethod
    def tearDownClass(cls):
        if cls.witness is not None:
            cls.witness.destroy_node()
        cls.probe.destroy_node()
        rclpy.shutdown(context=cls.context)
        cls.session.close()

    @classmethod
    def _wait_for_health(cls):
        deadline = time.monotonic() + HEALTH_BUDGET * get_time_scale()
        last = None
        while time.monotonic() < deadline:
            try:
                response = cls.session.get(f'{BASE_URL}/health', timeout=5)
                if response.status_code == 200:
                    return
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(
            f'gateway not ready within {HEALTH_BUDGET}s (last: {last})')

    @classmethod
    def _graph_fqns_until(cls, awaited, budget):
        """Poll the graph until *awaited* is a subset of the node FQNs."""
        deadline = time.monotonic() + budget * get_time_scale()
        while True:
            fqns = {
                (namespace.rstrip('/') + '/' + name)
                for name, namespace in cls.probe.get_node_names_and_namespaces()
            }
            if awaited <= fqns or time.monotonic() >= deadline:
                return fqns
            time.sleep(0.2)

    @classmethod
    def _apps_until_contains(cls, app_id, budget):
        """Poll /apps until *app_id* is listed. Returns (seen, last snapshot)."""
        deadline = time.monotonic() + budget * get_time_scale()
        apps = set()
        while True:
            body = cls.session.get(f'{BASE_URL}/apps', timeout=10).json()
            apps = {item['id'] for item in body.get('items', [])}
            if app_id in apps:
                return True, apps
            if time.monotonic() >= deadline:
                return False, apps
            time.sleep(0.5)

    def test_the_split_helper_nodes_are_all_on_the_graph(self):
        """The remap really does split them, so the absences below mean something.

        Without this the next test would pass on a launch where the namespace
        remap silently did nothing and the helpers never existed under these
        names.
        """
        self.assertTrue(
            set(ALL_HELPER_FQNS) <= self.graph_fqns,
            f'expected the gateway and its subscription node in {GATEWAY_NS} and the '
            f'other two helpers at the root, missing: '
            f'{sorted(set(ALL_HELPER_FQNS) - self.graph_fqns)}. '
            f'Nodes seen: {sorted(self.graph_fqns)}')
        self.assertIn(
            GATEWAY_FQN, self.graph_fqns,
            f'the gateway node is not in {GATEWAY_NS}, so the remap did not apply')

    def test_apps_lists_the_gateway_and_none_of_its_helpers(self):
        """A namespaced gateway serves itself and none of its plumbing.

        @verifies REQ_INTEROP_003
        """
        self.assertTrue(
            self.witness_seen,
            f'/apps never listed "{WITNESS_NODE}", so no snapshot is known to '
            f'post-date the helper nodes. Listed: {sorted(self.apps)}')

        self.assertEqual(
            self.apps, {GATEWAY_NAME} | TEST_OWN_NODES,
            f'a gateway alone on the graph must serve its own node and nothing '
            f"of its plumbing, beside this test's own two nodes. "
            f'Listed: {sorted(self.apps)}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
