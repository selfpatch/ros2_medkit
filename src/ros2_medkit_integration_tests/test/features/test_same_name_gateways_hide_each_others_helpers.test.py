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

"""Gateways that share a node name do not serve each other's helper nodes.

Three gateways keep the node name ``ros2_medkit_gateway`` on one graph: one in
the root namespace, one moved by a remap that names the gateway
(``-r ros2_medkit_gateway:__ns:=/subsystem_a``) and one moved by a process-wide
namespace (``/subsystem_b``). Their helper nodes share bare names across
namespaces, so runtime discovery gives the namespaced ones ids such as
``subsystem_a__ros2_medkit_gateway_sub``, which do not start with ``_``.

Each gateway must serve the three gateway nodes and none of the nine helper
nodes, and all three must list the same apps. A visible node in a namespace
that starts with ``_`` is served even when a name collision gives it an id
that starts with ``_``. Functions ``subsystem_a`` and ``_ops`` host exactly
the served apps in their namespace, and a namespace that holds only a hidden
node has no Function on either Function endpoint.
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

GATEWAY_NAME = 'ros2_medkit_gateway'
HELPER_SUFFIXES = ('_sub', '_fault_clients', '_lifecycle_state_reader')
# (namespace, port) of each gateway.
GATEWAYS = (
    ('/', get_test_port(0)),
    ('/subsystem_a', get_test_port(1)),
    ('/subsystem_b', get_test_port(2)),
)
GATEWAY_FQNS = {f'{ns.rstrip("/")}/{GATEWAY_NAME}' for ns, _ in GATEWAYS}
HELPER_FQNS = {
    f'{ns.rstrip("/")}/_{GATEWAY_NAME}{suffix}'
    for ns, _ in GATEWAYS for suffix in HELPER_SUFFIXES
}
# Runtime discovery qualifies a bare name shared across namespaces, except in
# the root namespace.
GATEWAY_APP_IDS = {
    GATEWAY_NAME,
    f'subsystem_a_{GATEWAY_NAME}',
    f'subsystem_b_{GATEWAY_NAME}',
}
WITNESS_NODE = 'same_name_gateways_witness'
# The witness name also runs here, so its id is qualified to start with '_'.
OPS_NAMESPACE = '/_ops'
OPS_WITNESS_APP_ID = f'_ops_{WITNESS_NODE}'
# Function id -> its hosts, written out: the served Apps in that namespace.
FUNCTION_HOSTS = {
    'subsystem_a': {f'subsystem_a_{GATEWAY_NAME}'},
    '_ops': {OPS_WITNESS_APP_ID},
}
# Hidden and alone in its namespace, so that namespace gets no served Function.
HIDDEN_NAMESPACE = '/hidden_ns'
HIDDEN_NODE = '_hidden_ns_probe'
HIDDEN_FUNCTION_ID = 'hidden_ns'
# Hidden, so it is no app of any gateway.
PROBE_NODE = '_same_name_gateways_probe'

HEALTH_BUDGET = 30.0
GRAPH_BUDGET = 20.0
REFRESH_WITNESS_BUDGET = 25.0


def _base_url(port):
    return f'http://127.0.0.1:{port}{API_BASE_PATH}'


@pytest.mark.launch_test
def generate_test_description():
    """Launch three gateways that keep one node name."""
    common = {'server.host': '127.0.0.1', 'refresh_interval_ms': 1000}
    root_gateway = create_gateway_node(
        port=GATEWAYS[0][1], name=GATEWAY_NAME, extra_params=common)
    remapped_gateway = create_gateway_node(
        port=GATEWAYS[1][1], name=GATEWAY_NAME, extra_params=common,
        remappings=[(f'{GATEWAY_NAME}:__ns', '/subsystem_a')])
    namespaced_gateway = create_gateway_node(
        port=GATEWAYS[2][1], name=GATEWAY_NAME, extra_params=common,
        namespace='/subsystem_b')

    return launch.LaunchDescription([
        root_gateway,
        remapped_gateway,
        namespaced_gateway,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': root_gateway}


class TestSameNameGatewaysHideEachOthersHelpers(unittest.TestCase):
    """No gateway serves a helper node, its own or another gateway's."""

    @classmethod
    def setUpClass(cls):
        """Wait for all gateways, then date each /apps snapshot with a witness."""
        cls.session = requests.Session()
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.probe = Node(PROBE_NODE, context=cls.context)
        cls.hidden = None
        cls.ops_witness = None
        cls.witness = None
        cls.graph_fqns = set()
        cls.apps = {}
        cls.witness_seen = {}
        cls.functions = {}
        cls.hidden_function = {}

        for _, port in GATEWAYS:
            cls._wait_for_health(port)
        cls.graph_fqns = cls._graph_fqns_until(GATEWAY_FQNS | HELPER_FQNS, GRAPH_BUDGET)
        if not (GATEWAY_FQNS | HELPER_FQNS) <= cls.graph_fqns:
            return

        # Created after the probe saw every helper node, so a snapshot that lists
        # the witnesses was built after that moment. This does not show that
        # each gateway had discovered the helpers by then.
        # The wait is for OPS_WITNESS_APP_ID: that id exists only while both
        # witnesses are on the graph. The hidden node is created first on the
        # same context, so a graph update that lists both witnesses lists it.
        cls.hidden = Node(HIDDEN_NODE, namespace=HIDDEN_NAMESPACE, context=cls.context)
        cls.ops_witness = Node(WITNESS_NODE, namespace=OPS_NAMESPACE, context=cls.context)
        cls.witness = Node(WITNESS_NODE, context=cls.context)
        for ns, port in GATEWAYS:
            cls.witness_seen[ns], cls.apps[ns] = cls._apps_until_contains(
                port, OPS_WITNESS_APP_ID, REFRESH_WITNESS_BUDGET)
            cls.functions[ns] = {
                function_id: (
                    cls.session.get(f'{_base_url(port)}/functions/{function_id}', timeout=10),
                    cls.session.get(
                        f'{_base_url(port)}/functions/{function_id}/hosts', timeout=10),
                )
                for function_id in FUNCTION_HOSTS
            }
            cls.hidden_function[ns] = {
                path: cls.session.get(f'{_base_url(port)}{path}', timeout=10)
                for path in (f'/functions/{HIDDEN_FUNCTION_ID}',
                             f'/functions/{HIDDEN_FUNCTION_ID}/hosts')
            }

    @classmethod
    def tearDownClass(cls):
        if cls.witness is not None:
            cls.witness.destroy_node()
        if cls.ops_witness is not None:
            cls.ops_witness.destroy_node()
        if cls.hidden is not None:
            cls.hidden.destroy_node()
        cls.probe.destroy_node()
        rclpy.shutdown(context=cls.context)
        cls.session.close()

    @classmethod
    def _wait_for_health(cls, port):
        deadline = time.monotonic() + HEALTH_BUDGET * get_time_scale()
        last = None
        while time.monotonic() < deadline:
            try:
                response = cls.session.get(f'{_base_url(port)}/health', timeout=5)
                if response.status_code == 200:
                    return
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(
            f'gateway on port {port} not ready within {HEALTH_BUDGET}s (last: {last})')

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
    def _apps_until_contains(cls, port, app_id, budget):
        """Poll /apps until *app_id* is listed. Returns (seen, last items by id)."""
        deadline = time.monotonic() + budget * get_time_scale()
        apps = {}
        while True:
            body = cls.session.get(f'{_base_url(port)}/apps', timeout=10).json()
            apps = {item['id']: item for item in body.get('items', [])}
            if app_id in apps:
                return True, apps
            if time.monotonic() >= deadline:
                return False, apps
            time.sleep(0.5)

    def test_every_gateway_and_helper_node_is_on_the_graph(self):
        """The three gateways and their nine helpers exist under these names."""
        expected = GATEWAY_FQNS | HELPER_FQNS
        self.assertTrue(
            expected <= self.graph_fqns,
            f'missing from the graph: {sorted(expected - self.graph_fqns)}. '
            f'Nodes seen: {sorted(self.graph_fqns)}')

    def test_no_gateway_serves_a_helper_node(self):
        """Each gateway serves the gateway nodes and the witnesses, no helper node.

        @verifies REQ_INTEROP_003
        """
        expected = GATEWAY_APP_IDS | {WITNESS_NODE, OPS_WITNESS_APP_ID}
        for ns, port in GATEWAYS:
            with self.subTest(gateway=ns):
                self.assertTrue(
                    self.witness_seen.get(ns),
                    f'gateway {ns} (port {port}) never listed "{OPS_WITNESS_APP_ID}", so '
                    f'no snapshot is known to post-date the helper nodes. '
                    f'Listed: {sorted(self.apps.get(ns, {}))}')
                served = set(self.apps[ns])
                self.assertEqual(
                    served, expected,
                    f'gateway {ns} (port {port}) serves {sorted(served - expected)} '
                    f'beyond the expected apps and misses {sorted(expected - served)}')

    def test_a_function_hosts_only_served_apps(self):
        """Both Function endpoints list exactly the served apps in its namespace."""
        for ns, port in GATEWAYS:
            for function_id, expected in FUNCTION_HOSTS.items():
                detail, hosts = self.functions.get(ns, {}).get(function_id, (None, None))
                with self.subTest(gateway=ns, function=function_id):
                    self.assertIsNotNone(detail, f'gateway {ns} (port {port}) was not queried')
                    self.assertEqual(
                        detail.status_code, 200,
                        f'gateway {ns} (port {port}): GET /functions/{function_id} '
                        f'returned {detail.status_code}: {detail.text}')
                    self.assertEqual(
                        set(detail.json().get('x-medkit', {}).get('hosts', [])), expected,
                        f'gateway {ns} (port {port}): x-medkit.hosts of {function_id}')
                    self.assertEqual(
                        hosts.status_code, 200,
                        f'gateway {ns} (port {port}): GET /functions/{function_id}/hosts '
                        f'returned {hosts.status_code}: {hosts.text}')
                    self.assertEqual(
                        {item['id'] for item in hosts.json().get('items', [])}, expected,
                        f'gateway {ns} (port {port}): /functions/{function_id}/hosts')

    def test_a_function_of_hidden_nodes_only_is_not_served(self):
        """Both Function endpoints answer 404 entity-not-found for that namespace."""
        for ns, port in GATEWAYS:
            for path, response in self.hidden_function.get(ns, {}).items():
                with self.subTest(gateway=ns, path=path):
                    self.assertEqual(
                        response.status_code, 404,
                        f'gateway {ns} (port {port}): GET {path} returned '
                        f'{response.status_code}: {response.text}')
                    body = response.json()
                    self.assertEqual(body.get('error_code'), 'entity-not-found', body)
                    self.assertIn(
                        HIDDEN_FUNCTION_ID, body.get('parameters', {}).values(), body)
            with self.subTest(gateway=ns):
                self.assertEqual(len(self.hidden_function.get(ns, {})), 2,
                                 f'gateway {ns} (port {port}) was not queried')

    def test_the_gateways_agree(self):
        """All three gateways list the same apps."""
        lists = {ns: frozenset(self.apps.get(ns, {})) for ns, _ in GATEWAYS}
        self.assertEqual(
            len(set(lists.values())), 1,
            f'the gateways disagree: {({ns: sorted(apps) for ns, apps in lists.items()})}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
