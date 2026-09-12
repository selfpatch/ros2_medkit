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

An absence assertion is only worth reading if the thing could have been there.
The gateway's helper nodes are created at three different points of start-up,
two of them after the first ``refresh_cache()`` and after ``/health`` starts
answering, so a naive "wait until /apps is non-empty" can read a list built
from a graph that did not yet contain them - and then the absences below hold
whatever the filter does. The settle sequence therefore proves the order it
needs: all three helpers on the graph FIRST, then a witness node created after
them, then an ``/apps`` snapshot that contains the witness. Such a snapshot was
built from a graph that held the helpers too.
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
WITNESS_NODE = 'own_node_apps_refresh_witness'

# The three stages of the settle sequence. They run one after another, so their
# sum plus launch and teardown has to stay inside this file's ctest TIMEOUT
# (the feature default, 120 s): a run killed by ctest reports a timeout and no
# test name, which hides whichever assertion actually failed.
HEALTH_BUDGET = 30.0
HELPERS_ON_GRAPH_BUDGET = 20.0
REFRESH_WITNESS_BUDGET = 25.0


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


class TestOwnNodeApps(unittest.TestCase):
    """The gateway is a diagnosable App; its in-process helpers are not."""

    @classmethod
    def setUpClass(cls):
        """Wait for the gateway, then settle /apps against the helper nodes."""
        cls.session = requests.Session()
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.probe = Node('own_node_apps_graph_probe', context=cls.context)
        cls.witness = None
        cls.apps = set()
        cls.graph_fqns = set()
        cls.witness_seen = False

        cls._wait_for_health()
        # Stage 1: every helper the absences below are about must be on the
        # graph before anything reads /apps.
        cls.graph_fqns = cls._graph_fqns_until(
            {f'/{helper}' for helper in HELPER_NODES}, HELPERS_ON_GRAPH_BUDGET)
        if not {f'/{helper}' for helper in HELPER_NODES} <= cls.graph_fqns:
            return

        # Stage 2: a node created strictly after the helpers appeared. Its own
        # arrival in /apps dates the snapshot: the gateway cannot have seen the
        # witness without having seen the helpers.
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
        """Node FQNs on the graph, polled until *awaited* is a subset.

        The graph query reads the discovery database directly, so this polls
        rather than spinning an executor. Returns the last set seen even on
        timeout - the caller asserts on it, so a timeout cannot pass silently.
        """
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
        # Anti-vacuity, first half: each helper really is a node on this graph,
        # so the absences below are about the filter and not about names that
        # were never there.
        awaited = {f'/{helper}' for helper in HELPER_NODES}
        self.assertTrue(
            awaited <= self.graph_fqns,
            f'helper nodes missing from the graph: {sorted(awaited - self.graph_fqns)}. '
            f'Nodes seen: {sorted(self.graph_fqns)}')

        # Anti-vacuity, second half: the snapshot was rebuilt after they
        # appeared. Without this the assertions below can be read from a list
        # the gateway built before it could have listed a helper at all.
        self.assertTrue(
            self.witness_seen,
            f'/apps never listed "{WITNESS_NODE}", so no snapshot is known to '
            f'post-date the helper nodes and these absences prove nothing. '
            f'Listed: {sorted(self.apps)}')

        for helper in HELPER_NODES:
            self.assertNotIn(
                helper, self.apps,
                f'in-process helper "{helper}" must not be a diagnosable App. '
                f'Listed: {sorted(self.apps)}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
