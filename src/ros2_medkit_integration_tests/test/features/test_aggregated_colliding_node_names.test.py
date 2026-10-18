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

"""An aggregator serves every App its peer serves, whatever the id looks like.

The peer runs ``/camera`` and ``/_ops/camera``. The shared node name gives the
second App a namespace-qualified id that starts with ``_``, and the peer serves
it because its node name, ``camera``, is visible. The aggregator, in another DDS
domain, must list that App and route requests for it to the peer.
"""

import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions
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
AGGREGATOR_URL = f'http://localhost:{AGGREGATOR_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'
AGGREGATOR_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

VISIBLE_NODE = '/camera'
HIDDEN_NAMESPACE_NODE = '/_ops/camera'
TIMEOUT_SEC = 60.0 * get_time_scale()


def _camera(namespace, env):
    return launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_engine_temp_sensor',
        name='camera',
        namespace=namespace,
        additional_env=env,
    )


def generate_test_description():
    peer_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}
    aggregator = create_gateway_node(
        port=AGGREGATOR_PORT,
        extra_params={
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': ['peer_gateway'],
        },
    )
    peer = create_gateway_node(
        name='peer_gateway_node', port=PEER_PORT,
        extra_env={'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)})
    nodes = TimerAction(period=2.0, actions=[_camera('/', peer_env), _camera('/_ops', peer_env)])
    return (
        LaunchDescription([
            SetEnvironmentVariable('ROS_DOMAIN_ID', str(AGGREGATOR_DOMAIN_ID)),
            aggregator,
            peer,
            nodes,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': aggregator, 'peer_gateway': peer},
    )


def _apps(base_url):
    try:
        response = requests.get(f'{base_url}/apps', timeout=10)
    except requests.exceptions.RequestException:
        return None
    if response.status_code != 200:
        return None
    return response.json().get('items', [])


def _node(app):
    return app.get('x-medkit', {}).get('ros2', {}).get('node')


class TestAggregatedCollidingNodeNames(unittest.TestCase):

    def test_aggregator_serves_the_peer_app_with_an_underscore_id(self):
        # The id is qualified only once both nodes are on the peer's graph.
        by_node = {}
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline and not (
                VISIBLE_NODE in by_node
                and by_node.get(HIDDEN_NAMESPACE_NODE, '').startswith('_')):
            by_node = {_node(a): a['id'] for a in _apps(PEER_URL) or []}
            time.sleep(0.5)
        self.assertIn(VISIBLE_NODE, by_node, f'the peer never served {VISIBLE_NODE}')
        app_id = by_node.get(HIDDEN_NAMESPACE_NODE, '')
        self.assertTrue(
            app_id.startswith('_'),
            f'the peer serves {HIDDEN_NAMESPACE_NODE} as {app_id!r}, so this test does not '
            'cover an id that starts with an underscore')

        ids = []
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline and app_id not in ids:
            ids = [a['id'] for a in _apps(AGGREGATOR_URL) or []]
            time.sleep(0.5)
        self.assertIn(app_id, ids, f'the aggregator does not list the peer App {app_id}')

        response = requests.get(f'{AGGREGATOR_URL}/apps/{app_id}', timeout=10)
        self.assertEqual(response.status_code, 200, response.text)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
