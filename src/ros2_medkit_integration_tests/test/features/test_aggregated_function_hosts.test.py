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

"""An aggregated Function hosts only Apps the aggregator serves.

A peer in one DDS domain and two aggregators in another. The peer runs with
``filter_internal_nodes`` false, so it serves its hidden nodes and lists them
as Function hosts. Each gateway filters only its own nodes, so both aggregators
serve every peer App. One aggregator runs the default filter and hides its own
hidden nodes, the other runs with the filter off.

Namespace ``line_a``: the peer runs a visible and a hidden node, the
aggregator one hidden node. Function ``line_a`` is merged from both, so the
aggregator answers it itself, and both Function routes must list exactly the
peer's two Apps.

Namespace ``cell_b``: each side runs one hidden node only. The merged Function
hosts only the peer's App.

Namespace ``peer_only_ns``: only the peer has it, so the aggregator forwards
``GET /functions/peer_only_ns`` and the answer is the peer's view.
``GET /functions/peer_only_ns/hosts`` is answered by the aggregator.

Namespace ``dup_ns``: only the peer has it, with one hidden node ``_dup_node``.
The aggregator side runs a node of the same name in ``local_ns``, which the
unfiltered aggregator serves as App ``_dup_node``. The peer's App is renamed
``peer_gateway___dup_node`` there, and the peer's host ``_dup_node`` must
follow it to that id instead of binding to the local App.
"""

import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
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
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

AGGREGATOR_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
UNFILTERED_PORT = get_test_port(2)
AGGREGATOR_URL = f'http://127.0.0.1:{AGGREGATOR_PORT}{API_BASE_PATH}'
PEER_URL = f'http://127.0.0.1:{PEER_PORT}{API_BASE_PATH}'
UNFILTERED_URL = f'http://127.0.0.1:{UNFILTERED_PORT}{API_BASE_PATH}'
AGGREGATOR_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)
PEER_NAME = 'peer_gateway'

# (namespace, node name) on each side. A name that starts with '_' is hidden.
PEER_NODES = (
    ('/line_a', 'line_a_sensor'),
    ('/line_a', '_line_a_peer_internal'),
    ('/cell_b', '_cell_b_peer_internal'),
    ('/peer_only_ns', 'peer_only_sensor'),
    ('/peer_only_ns', '_peer_only_internal'),
    ('/dup_ns', '_dup_node'),
)
AGGREGATOR_NODES = (
    ('/line_a', '_line_a_local_internal'),
    ('/cell_b', '_cell_b_local_internal'),
    ('/local_ns', '_dup_node'),
)
# Hidden peer Apps every aggregator lists, whatever its own filter setting.
PEER_HIDDEN_ONLY = {'_line_a_peer_internal', '_cell_b_peer_internal', '_peer_only_internal'}
# The aggregator side's own hidden nodes. Only the unfiltered aggregator lists them.
LOCAL_HIDDEN = {'_line_a_local_internal', '_cell_b_local_internal'}
# The id the unfiltered aggregator gives the peer's _dup_node next to its own.
RENAMED_PEER_DUP = f'{PEER_NAME}___dup_node'
# Created last on each side, so a snapshot that lists one post-dates the nodes
# above on that side.
PEER_WITNESS = 'aggregated_hosts_peer_witness'
AGGREGATOR_WITNESS = 'aggregated_hosts_local_witness'

# Function id -> its hosts on the aggregator, written out.
EXPECTED_HOSTS = {
    'line_a': {'line_a_sensor', '_line_a_peer_internal'},
    'cell_b': {'_cell_b_peer_internal'},
}

HEALTH_BUDGET = 30.0
SNAPSHOT_BUDGET = 40.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch two aggregators and a peer that serves its hidden nodes."""
    common = {'server.host': '127.0.0.1', 'refresh_interval_ms': 1000}
    aggregation = {
        'aggregation.enabled': True,
        'aggregation.timeout_ms': 5000,
        'aggregation.announce': False,
        'aggregation.discover': False,
        'aggregation.peer_urls': [f'http://127.0.0.1:{PEER_PORT}'],
        'aggregation.peer_names': [PEER_NAME],
    }
    aggregator = create_gateway_node(
        port=AGGREGATOR_PORT, extra_params={**common, **aggregation})
    unfiltered = create_gateway_node(
        name='unfiltered_aggregator_node',
        port=UNFILTERED_PORT,
        extra_params={
            **common, **aggregation, 'discovery.runtime.filter_internal_nodes': False},
    )
    peer = create_gateway_node(
        name='peer_gateway_node',
        port=PEER_PORT,
        extra_params={**common, 'discovery.runtime.filter_internal_nodes': False},
        extra_env={'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)},
    )
    return LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(AGGREGATOR_DOMAIN_ID)),
        aggregator,
        unfiltered,
        peer,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': aggregator, 'peer_gateway': peer}


def _get(url):
    return requests.get(url, timeout=10)


class TestAggregatedFunctionHosts(unittest.TestCase):
    """Function hosts on the aggregator name only Apps it serves."""

    @classmethod
    def setUpClass(cls):
        """Start the nodes on both domains, then wait for a dated snapshot."""
        cls.contexts = []
        cls.nodes = []
        for url in (AGGREGATOR_URL, PEER_URL, UNFILTERED_URL):
            cls._wait_for_health(url)

        for domain_id, nodes, witness in (
            (PEER_DOMAIN_ID, PEER_NODES, PEER_WITNESS),
            (AGGREGATOR_DOMAIN_ID, AGGREGATOR_NODES, AGGREGATOR_WITNESS),
        ):
            context = Context()
            rclpy.init(context=context, domain_id=domain_id)
            cls.contexts.append(context)
            for namespace, name in nodes:
                cls.nodes.append(Node(name, namespace=namespace, context=context))
            cls.nodes.append(Node(witness, context=context))

        # The peer lists its witness and its hidden node: its filter is off.
        cls.peer_apps = cls._apps_until(
            PEER_URL, {PEER_WITNESS, '_line_a_peer_internal'})
        # The aggregator lists both witnesses. The peer witness arrives only in
        # a peer fetch made after the peer saw every peer node.
        cls.aggregator_apps = cls._apps_until(
            AGGREGATOR_URL, {AGGREGATOR_WITNESS, PEER_WITNESS})
        cls.unfiltered_apps = cls._apps_until(
            UNFILTERED_URL, {AGGREGATOR_WITNESS, PEER_WITNESS, '_dup_node', RENAMED_PEER_DUP},
            items=True)
        cls.unfiltered_dup_hosts = _get(f'{UNFILTERED_URL}/functions/dup_ns/hosts')
        cls.peer_line_a = _get(f'{PEER_URL}/functions/line_a')
        cls.functions = {
            function_id: (
                _get(f'{AGGREGATOR_URL}/functions/{function_id}'),
                _get(f'{AGGREGATOR_URL}/functions/{function_id}/hosts'),
            )
            for function_id in (*EXPECTED_HOSTS, 'peer_only_ns')
        }

    @classmethod
    def tearDownClass(cls):
        for node in cls.nodes:
            node.destroy_node()
        for context in cls.contexts:
            rclpy.shutdown(context=context)

    @classmethod
    def _wait_for_health(cls, url):
        deadline = time.monotonic() + HEALTH_BUDGET * get_time_scale()
        last = None
        while time.monotonic() < deadline:
            try:
                response = _get(f'{url}/health')
                if response.status_code == 200:
                    return
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(f'{url} not ready within {HEALTH_BUDGET}s (last: {last})')

    @classmethod
    def _apps_until(cls, url, awaited, items=False):
        """Poll /apps until every id in *awaited* is listed.

        Returns the ids, or the items by id when *items* is true.
        """
        deadline = time.monotonic() + SNAPSHOT_BUDGET * get_time_scale()
        apps = set()
        while time.monotonic() < deadline:
            try:
                body = _get(f'{url}/apps').json()
                by_id = {item['id']: item for item in body.get('items', [])}
                apps = set(by_id)
                if awaited <= apps:
                    return by_id if items else apps
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(
            f'{url} never listed {sorted(awaited - apps)}. Listed: {sorted(apps)}')

    def test_the_peer_lists_its_hidden_node_as_a_host(self):
        """Precondition: the peer serves its hidden App and hosts it in line_a."""
        self.assertIn('_line_a_peer_internal', self.peer_apps)
        self.assertEqual(self.peer_line_a.status_code, 200, self.peer_line_a.text)
        self.assertEqual(
            set(self.peer_line_a.json().get('x-medkit', {}).get('hosts', [])),
            {'line_a_sensor', '_line_a_peer_internal'})

    def test_the_aggregator_serves_the_peer_apps_and_hides_its_own(self):
        """The peer's hidden Apps are served, the aggregator's own hidden nodes are not."""
        self.assertLessEqual(PEER_HIDDEN_ONLY, self.aggregator_apps, sorted(self.aggregator_apps))
        self.assertFalse(LOCAL_HIDDEN & self.aggregator_apps, sorted(self.aggregator_apps))
        self.assertIn('line_a_sensor', self.aggregator_apps)

    def test_a_merged_function_hosts_only_served_apps(self):
        """Both Function routes list exactly the Apps the aggregator serves."""
        for function_id, expected in EXPECTED_HOSTS.items():
            detail, hosts = self.functions[function_id]
            with self.subTest(function=function_id):
                self.assertEqual(detail.status_code, 200, detail.text)
                x_medkit = detail.json().get('x-medkit', {})
                # Answered by the aggregator itself: both sides contribute.
                self.assertEqual(
                    set(x_medkit.get('contributors', [])), {'local', f'peer:{PEER_NAME}'},
                    x_medkit)
                self.assertEqual(set(x_medkit.get('hosts', [])), expected, x_medkit)
                self.assertEqual(hosts.status_code, 200, hosts.text)
                self.assertEqual(
                    {item['id'] for item in hosts.json().get('items', [])}, expected,
                    hosts.text)

    def test_a_forwarded_function_answers_with_the_peer_view(self):
        """Detail is forwarded to the peer, /hosts is answered here."""
        detail, hosts = self.functions['peer_only_ns']
        self.assertEqual(detail.status_code, 200, detail.text)
        x_medkit = detail.json().get('x-medkit', {})
        # The peer's own view: its filter is off, so its hidden App is a host,
        # and no aggregation provenance is attached.
        self.assertEqual(
            set(x_medkit.get('hosts', [])), {'peer_only_sensor', '_peer_only_internal'}, x_medkit)
        self.assertNotIn('contributors', x_medkit)
        self.assertEqual(hosts.status_code, 200, hosts.text)
        self.assertEqual(
            {item['id'] for item in hosts.json().get('items', [])},
            {'peer_only_sensor', '_peer_only_internal'}, hosts.text)

    def test_the_unfiltered_aggregator_serves_both_sides(self):
        """Its own hidden nodes and the peer's, with the peer's _dup_node renamed."""
        self.assertLessEqual(
            PEER_HIDDEN_ONLY | LOCAL_HIDDEN, set(self.unfiltered_apps),
            sorted(self.unfiltered_apps))
        dup = self.unfiltered_apps['_dup_node'].get('x-medkit', {})
        self.assertEqual(dup.get('ros2', {}).get('node'), '/local_ns/_dup_node', dup)

    def test_a_renamed_peer_app_is_the_host_under_its_new_id(self):
        """The peer's host _dup_node names the renamed peer App, not the local _dup_node."""
        response = self.unfiltered_dup_hosts
        self.assertEqual(response.status_code, 200, response.text)
        self.assertEqual(
            [item['id'] for item in response.json().get('items', [])], [RENAMED_PEER_DUP],
            response.text)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
