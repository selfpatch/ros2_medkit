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

"""The gateway's in-process helper nodes keep their own names under launch_ros.

launch_ros ``Node(name=...)`` passes ``-r __node:=<name>`` to the whole
process. The gateway creates helper nodes of its own (fault service clients,
lifecycle state reads, topic subscriptions). Each of them must stay distinct
from the gateway node, stay hidden from discovery, and keep working.

The namespaced, remapped and sim-time launch is covered by
``test_gateway_helper_nodes_namespaced.test.py``.
"""

import unittest

import launch_testing
import rclpy
from rclpy.node import Node

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch
from ros2_medkit_test_utils.ros_graph import (
    advertised_services,
    duplicate_fqns,
    leaf_name,
    ros2_param_list,
    wait_for_graph,
)

GATEWAY_NAME = 'ros2_medkit_gateway'
GATEWAY_FQN = '/' + GATEWAY_NAME
LAUNCHED_APPS = {
    GATEWAY_NAME, 'fault_manager', 'temp_sensor', 'managed_lifecycle',
    'managed_lifecycle_active',
}


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor', 'managed_lifecycle', 'managed_lifecycle_active'],
        fault_manager=True,
    )


class TestGatewayHelperNodes(GatewayTestCase):
    """One gateway launched through launch_ros is one node in the graph."""

    REQUIRED_APPS = LAUNCHED_APPS

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        # Hidden, so the probe itself never becomes an App.
        cls._probe = Node('_helper_node_probe')

    @classmethod
    def tearDownClass(cls):
        cls._probe.destroy_node()
        rclpy.shutdown()
        super().tearDownClass()

    def _settled_graph(self):
        # The fault manager is launched last, so its presence means every
        # launched node had time to join the graph.
        return wait_for_graph(
            self._probe,
            lambda fqns: GATEWAY_FQN in fqns and '/fault_manager' in fqns,
        )

    def test_gateway_name_is_unique_in_graph(self):
        fqns = self._settled_graph()
        self.assertEqual(
            fqns.count(GATEWAY_FQN), 1,
            f'{GATEWAY_FQN} appears {fqns.count(GATEWAY_FQN)} times in {sorted(fqns)}',
        )
        self.assertEqual(duplicate_fqns(fqns), [], f'duplicate node names in {sorted(fqns)}')

    def test_helper_nodes_are_hidden(self):
        fqns = self._settled_graph()
        own = [f for f in fqns if GATEWAY_NAME in leaf_name(f) and f != GATEWAY_FQN]
        self.assertTrue(own, f'no helper node of the gateway found in {sorted(fqns)}')
        visible = [f for f in own if not leaf_name(f).startswith('_')]
        self.assertEqual(visible, [], f'helper nodes not hidden: {visible}')

    def test_helper_nodes_advertise_no_parameter_services(self):
        # A helper spun only during its own requests would never answer a
        # parameter service.
        fqns = self._settled_graph()
        own = [f for f in fqns if GATEWAY_NAME in leaf_name(f) and f != GATEWAY_FQN]
        self.assertTrue(own, f'no helper node of the gateway found in {sorted(fqns)}')
        for fqn in own:
            params = [s for s in advertised_services(self._probe, fqn) if 'parameter' in s]
            self.assertEqual(params, [], f'{fqn} advertises {params}')

    def test_ros2_param_list_lists_each_parameter_once(self):
        self._settled_graph()
        names, output = ros2_param_list(GATEWAY_FQN)
        self.assertIn('server.port', names, output)
        duplicates = sorted({n for n in names if names.count(n) > 1})
        self.assertEqual(duplicates, [], f'parameters listed more than once:\n{output}')

    def test_helper_nodes_are_not_apps(self):
        self._settled_graph()
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if LAUNCHED_APPS <= {a['id'] for a in d.get('items', [])} else None,
            timeout=15.0,
        )
        ids = {a['id'] for a in data['items']}
        own = {i for i in ids if GATEWAY_NAME in i}
        self.assertEqual(own, {GATEWAY_NAME}, f'gateway-derived apps: {sorted(ids)}')

    def test_fault_client_node_serves_requests(self):
        data = self.poll_endpoint('/faults', timeout=15.0)
        self.assertIn('items', data)

    def test_subscription_node_samples_topics(self):
        topic = self.encode_topic_path('/powertrain/engine/temperature')
        data = self.poll_endpoint_until(
            f'/apps/temp_sensor/data/{topic}',
            lambda d: d if d.get('x-medkit', {}).get('status') == 'data' else None,
            timeout=15.0,
        )
        self.assertEqual(data['x-medkit']['status'], 'data')

    def test_lifecycle_reader_node_reads_states(self):
        # Graph presence alone reads "ready"; notReady for the unconfigured
        # node proves a real GetState round trip.
        unconfigured = self.poll_endpoint_until(
            '/apps/managed_lifecycle/status',
            lambda d: d if d.get('status') == 'notReady' else None,
            timeout=20.0,
        )
        self.assertEqual(unconfigured['status'], 'notReady')
        active = self.poll_endpoint_until(
            '/apps/managed_lifecycle_active/status',
            lambda d: d if d.get('status') == 'ready' else None,
            timeout=20.0,
        )
        self.assertEqual(active['status'], 'ready')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
