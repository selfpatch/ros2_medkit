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

"""Gateway helper nodes follow a namespace set for the gateway node only.

The gateway gets its namespace from a node-specific rule, and a global rule
names another namespace. The helpers must land in the gateway's namespace, where
is_own_gateway_helper_node looks for them.
"""

import unittest

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node
from ros2_medkit_test_utils.ros_graph import duplicate_fqns, leaf_name, wait_for_graph

NAMESPACE = '/robot1'
GATEWAY_NAME = 'ros2_medkit_gateway'
GATEWAY_FQN = f'{NAMESPACE}/{GATEWAY_NAME}'
HELPER_SUFFIXES = ('_sub', '_fault_clients', '_lifecycle_state_reader')


def generate_test_description():
    gateway = create_gateway_node(
        ros_arguments=['-r', f'{GATEWAY_NAME}:__ns:={NAMESPACE}', '-r', '__ns:=/'],
    )
    return (
        LaunchDescription([gateway, launch_testing.actions.ReadyToTest()]),
        {'gateway_node': gateway},
    )


class TestNodeNamespacedGatewayHelperNodes(GatewayTestCase):
    """Helper nodes take the namespace the gateway node got."""

    REQUIRED_APPS = {GATEWAY_NAME}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._probe = Node('_helper_node_probe')

    @classmethod
    def tearDownClass(cls):
        cls._probe.destroy_node()
        rclpy.shutdown()
        super().tearDownClass()

    def _helper_fqns(self):
        names = {f'_{GATEWAY_NAME}{suffix}' for suffix in HELPER_SUFFIXES}
        fqns = wait_for_graph(
            self._probe,
            lambda fqns: GATEWAY_FQN in fqns and names <= {leaf_name(f) for f in fqns},
        )
        return fqns, [f for f in fqns if leaf_name(f) in names]

    def test_helper_nodes_live_in_gateway_namespace(self):
        fqns, helpers = self._helper_fqns()
        self.assertEqual(len(helpers), len(HELPER_SUFFIXES), f'helpers in {sorted(fqns)}')
        outside = [f for f in helpers if not f.startswith(NAMESPACE + '/')]
        self.assertEqual(outside, [], f'helper nodes outside {NAMESPACE}: {outside}')
        self.assertEqual(duplicate_fqns(fqns), [], f'duplicate node names in {sorted(fqns)}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
