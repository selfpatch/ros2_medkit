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

Two gateways in separate DDS domains. The peer runs with
``filter_internal_nodes`` false, so it serves its hidden nodes and lists them
as Function hosts. The aggregator runs the default and drops a peer's hidden
Apps.

Namespace ``line_a``: the peer runs a visible and a hidden node, the
aggregator one hidden node. Function ``line_a`` is merged from both, so the
aggregator answers it itself, and both Function routes must list exactly the
peer's visible App.
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
AGGREGATOR_URL = f'http://127.0.0.1:{AGGREGATOR_PORT}{API_BASE_PATH}'
PEER_URL = f'http://127.0.0.1:{PEER_PORT}{API_BASE_PATH}'
AGGREGATOR_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)
PEER_NAME = 'peer_gateway'

# (namespace, node name) on each side. A name that starts with '_' is hidden.
PEER_NODES = (
    ('/line_a', 'line_a_sensor'),
    ('/line_a', '_line_a_peer_internal'),
)
AGGREGATOR_NODES = (
    ('/line_a', '_line_a_local_internal'),
)
# Created last on each side, so a snapshot that lists one post-dates the nodes
# above on that side.
PEER_WITNESS = 'aggregated_hosts_peer_witness'
AGGREGATOR_WITNESS = 'aggregated_hosts_local_witness'

# Function id -> its hosts on the aggregator, written out.
EXPECTED_HOSTS = {
    'line_a': {'line_a_sensor'},
}

HEALTH_BUDGET = 30.0
SNAPSHOT_BUDGET = 40.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch an aggregator and a peer that serves its hidden nodes."""
    common = {'server.host': '127.0.0.1', 'refresh_interval_ms': 1000}
    aggregator = create_gateway_node(
        port=AGGREGATOR_PORT,
        extra_params={
            **common,
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://127.0.0.1:{PEER_PORT}'],
            'aggregation.peer_names': [PEER_NAME],
        },
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
        for url in (AGGREGATOR_URL, PEER_URL):
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
        cls.peer_line_a = _get(f'{PEER_URL}/functions/line_a')
        cls.functions = {
            function_id: (
                _get(f'{AGGREGATOR_URL}/functions/{function_id}'),
                _get(f'{AGGREGATOR_URL}/functions/{function_id}/hosts'),
            )
            for function_id in EXPECTED_HOSTS
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
    def _apps_until(cls, url, awaited):
        """Poll /apps until every id in *awaited* is listed."""
        deadline = time.monotonic() + SNAPSHOT_BUDGET * get_time_scale()
        apps = set()
        while time.monotonic() < deadline:
            try:
                body = _get(f'{url}/apps').json()
                apps = {item['id'] for item in body.get('items', [])}
                if awaited <= apps:
                    return apps
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

    def test_the_aggregator_serves_no_hidden_app(self):
        """The hidden nodes of both sides are no Apps of the aggregator."""
        hidden = {name for _, name in PEER_NODES + AGGREGATOR_NODES if name.startswith('_')}
        self.assertFalse(hidden & self.aggregator_apps, sorted(self.aggregator_apps))
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


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
