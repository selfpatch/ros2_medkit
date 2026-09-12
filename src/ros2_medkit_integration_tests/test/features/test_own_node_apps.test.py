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

"""What of the gateway's own process shows up in ``/apps``.

The gateway runs four ROS nodes in one process: itself, the subscription
executor's ``<gateway>_sub``, the fault-service transport's
``<gateway>_fault_clients`` and the lifecycle reader's
``<gateway>_lifecycle_state_reader``. None of the three helper names begins
with ``_``, so the ROS 2 hidden-node convention leaves them in the graph and
runtime discovery would turn each into an App - the gateway advertising its own
plumbing as something an operator can diagnose.

The gateway's own node is the opposite case and stays an App: its ROS
parameters are what SOVD serves as that App's configurations, so
``/apps/<gateway>/configurations`` is the only place a caller can read or write
``aggregation.peer_auth_header`` and its neighbours. Two gateways watching one
graph also have to agree on what is on it, which they cannot do if each hides
a different node.

This fixture launches the gateway with no ``__node`` remap, the way ``ros2 run``
and the container images start it. ``launch_ros``' ``name=`` applies
``-r __node:=<name>`` to the whole process, which renames all four nodes to the
same string - so under the suite's usual launch the helper names do not exist
and nothing here could be observed.
"""

import time
import unittest

import launch
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
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

GATEWAY_PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{GATEWAY_PORT}{API_BASE_PATH}'

GATEWAY_NODE = 'ros2_medkit_gateway'
HELPER_NODES = (
    f'{GATEWAY_NODE}_sub',
    f'{GATEWAY_NODE}_fault_clients',
    f'{GATEWAY_NODE}_lifecycle_state_reader',
)


@pytest.mark.launch_test
def generate_test_description():
    """Launch one bare gateway, keeping its process's real node names."""
    gateway_node = create_gateway_node(
        port=GATEWAY_PORT,
        name=None,
        extra_params={'server.host': '127.0.0.1', 'refresh_interval_ms': 1000},
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


def _graph_node_fqns(awaited, timeout=30.0):
    """Fully qualified node names on the graph, waiting for *awaited*.

    Runs on its own rclpy context so it cannot disturb anything else in the
    process, and polls: a graph query reads the discovery database directly and
    needs no executor.
    """
    context = Context()
    rclpy.init(context=context)
    probe = Node('own_node_apps_graph_probe', context=context)
    try:
        deadline = time.monotonic() + timeout * get_time_scale()
        while True:
            fqns = {
                (namespace.rstrip('/') + '/' + name)
                for name, namespace in probe.get_node_names_and_namespaces()
            }
            if awaited <= fqns or time.monotonic() >= deadline:
                return fqns
            time.sleep(0.2)
    finally:
        probe.destroy_node()
        rclpy.shutdown(context=context)


class TestOwnNodeApps(unittest.TestCase):
    """The gateway is a diagnosable App; its in-process helpers are not."""

    @classmethod
    def setUpClass(cls):
        """Wait for the gateway to answer, then read its app list once."""
        cls.session = requests.Session()
        deadline = time.monotonic() + 60.0 * get_time_scale()
        last = None
        while time.monotonic() < deadline:
            try:
                response = cls.session.get(f'{BASE_URL}/health', timeout=5)
                if response.status_code == 200:
                    break
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        else:
            raise AssertionError(f'gateway not ready within 60s (last: {last})')

        # The app list is served from the discovery cache, which the first
        # refresh fills; poll until the gateway's own node is in it or the
        # budget is out, so the absence assertions below read a settled list.
        cls.apps = set()
        app_deadline = time.monotonic() + 30.0 * get_time_scale()
        while time.monotonic() < app_deadline:
            body = cls.session.get(f'{BASE_URL}/apps', timeout=10).json()
            cls.apps = {item['id'] for item in body.get('items', [])}
            if GATEWAY_NODE in cls.apps:
                break
            time.sleep(0.5)

    @classmethod
    def tearDownClass(cls):
        cls.session.close()

    def test_the_gateways_own_node_is_an_app(self):
        """The gateway's own ROS node is listed, addressable and configurable.

        @verifies REQ_INTEROP_003
        """
        self.assertIn(
            GATEWAY_NODE, self.apps,
            f'the gateway node must be a diagnosable App. Listed: {sorted(self.apps)}')

        detail = self.session.get(f'{BASE_URL}/apps/{GATEWAY_NODE}', timeout=10)
        self.assertEqual(detail.status_code, 200, detail.text)
        self.assertEqual(detail.json()['id'], GATEWAY_NODE)

        configurations = self.session.get(
            f'{BASE_URL}/apps/{GATEWAY_NODE}/configurations', timeout=10)
        self.assertEqual(configurations.status_code, 200, configurations.text)
        items = configurations.json().get('items', [])
        self.assertTrue(
            items,
            'the gateway App must serve its own ROS parameters as configurations; '
            'they are reachable nowhere else')

    def test_the_in_process_helper_nodes_are_not_apps(self):
        """The gateway's own helper nodes stay out of the app list.

        @verifies REQ_INTEROP_003
        """
        for helper in HELPER_NODES:
            self.assertNotIn(
                helper, self.apps,
                f'in-process helper "{helper}" must not be a diagnosable App. '
                f'Listed: {sorted(self.apps)}')

        # Anti-vacuity: each helper really is a node on this graph, so the
        # assertions above are about the filter and not about names that were
        # never there. Without this the test passes on a gateway that creates
        # no helpers at all.
        awaited = {f'/{helper}' for helper in HELPER_NODES}
        graph_fqns = _graph_node_fqns(awaited)
        self.assertTrue(
            awaited <= graph_fqns,
            f'helper nodes missing from the graph: {sorted(awaited - graph_fqns)}. '
            f'Nodes seen: {sorted(graph_fqns)}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
