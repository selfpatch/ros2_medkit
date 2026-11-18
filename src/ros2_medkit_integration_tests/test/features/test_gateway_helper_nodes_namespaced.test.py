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

"""Gateway helper nodes keep the process namespace, remaps and sim time.

The gateway runs in a namespace, with ``use_sim_time`` and with remap rules
that are the only route to a fault manager in that namespace
(``fault_manager.namespace`` is left unset on purpose). The gateway's helper
nodes must keep their own names and still get all of these. With a namespace
set, launch_ros writes the parameters under the gateway's FQN, so a helper
with another name does not read them from that file.
"""

import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_fault_manager_node,
    create_gateway_node,
)
from ros2_medkit_test_utils.ros_graph import (
    duplicate_fqns,
    get_bool_parameter,
    leaf_name,
    ros2_param_list,
    subscribed_topics,
    wait_for_graph,
)

NAMESPACE = '/robot1'
GATEWAY_NAME = 'ros2_medkit_gateway'
GATEWAY_FQN = f'{NAMESPACE}/{GATEWAY_NAME}'
FAULT_MANAGER_FQN = f'{NAMESPACE}/fault_manager'
FAULT_SERVICES = [
    'report_fault', 'get_fault', 'list_faults', 'clear_fault', 'get_snapshots',
    'get_rosbag', 'list_rosbags',
]
LAUNCHED_APPS = {
    GATEWAY_NAME, 'fault_manager', 'temp_sensor', 'managed_lifecycle',
    'managed_lifecycle_active',
}


def generate_test_description():
    gateway = create_gateway_node(
        namespace=NAMESPACE,
        remappings=[
            (f'/fault_manager/{s}', f'{FAULT_MANAGER_FQN}/{s}') for s in FAULT_SERVICES
        ],
        extra_params={'use_sim_time': True},
    )
    delayed = TimerAction(
        period=2.0,
        actions=create_demo_nodes(
            ['temp_sensor', 'managed_lifecycle', 'managed_lifecycle_active'],
        ) + [create_fault_manager_node(namespace=NAMESPACE)],
    )
    return (
        LaunchDescription([gateway, delayed, launch_testing.actions.ReadyToTest()]),
        {'gateway_node': gateway},
    )


class TestNamespacedGatewayHelperNodes(GatewayTestCase):
    """Helper nodes follow the gateway's namespace, remaps and sim time."""

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
        return wait_for_graph(
            self._probe,
            lambda fqns: GATEWAY_FQN in fqns and FAULT_MANAGER_FQN in fqns,
        )

    def _helper_fqns(self, fqns):
        return [f for f in fqns if GATEWAY_NAME in leaf_name(f) and f != GATEWAY_FQN]

    def test_gateway_name_is_unique_in_graph(self):
        fqns = self._settled_graph()
        self.assertEqual(
            fqns.count(GATEWAY_FQN), 1,
            f'{GATEWAY_FQN} appears {fqns.count(GATEWAY_FQN)} times in {sorted(fqns)}',
        )
        self.assertEqual(duplicate_fqns(fqns), [], f'duplicate node names in {sorted(fqns)}')

    def test_helper_nodes_live_in_gateway_namespace(self):
        fqns = self._settled_graph()
        helpers = self._helper_fqns(fqns)
        self.assertTrue(helpers, f'no helper node of the gateway found in {sorted(fqns)}')
        outside = [f for f in helpers if not f.startswith(NAMESPACE + '/')]
        self.assertEqual(outside, [], f'helper nodes outside {NAMESPACE}: {outside}')
        visible = [f for f in helpers if not leaf_name(f).startswith('_')]
        self.assertEqual(visible, [], f'helper nodes not hidden: {visible}')

    def test_helper_nodes_use_sim_time(self):
        # A node with use_sim_time subscribes to /clock.
        fqns = self._settled_graph()
        self.assertIs(get_bool_parameter(self._probe, GATEWAY_FQN, 'use_sim_time'), True)
        helpers = self._helper_fqns(fqns)
        self.assertTrue(helpers, f'no helper node of the gateway found in {sorted(fqns)}')
        for fqn in helpers:
            topics = subscribed_topics(self._probe, fqn)
            self.assertIn('/clock', topics, f'{fqn} does not use sim time')

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

    def test_fault_client_node_follows_remaps(self):
        # Without the remap the fault clients call /fault_manager/*, which
        # nothing serves, and the gateway answers 503.
        data = self.poll_endpoint('/faults', timeout=20.0)
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
