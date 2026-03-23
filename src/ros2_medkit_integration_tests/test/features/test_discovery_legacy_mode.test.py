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

"""
Integration tests for legacy discovery mode (create_synthetic_components: false).

When synthetic components are disabled, each node becomes its own Component
in a 1:1 mapping (no namespace-based grouping).
"""

import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


def generate_test_description():
    gateway_node = create_gateway_node(
        extra_params={
            'discovery.runtime.create_synthetic_components': False,
        },
    )

    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'pressure_sensor'],
    )
    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway_node,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


# @verifies REQ_INTEROP_003
class TestLegacyDiscoveryMode(GatewayTestCase):
    """Test create_synthetic_components=false (legacy 1:1 node-to-component mode)."""

    def test_each_node_has_own_component(self):
        """Each node should become its own Component (no synthetic grouping)."""
        expected = ['temp_sensor', 'rpm_sensor', 'pressure_sensor']
        data = self.poll_endpoint_until(
            '/components',
            lambda d: d if all(
                any(name in c['id'] for c in d.get('items', []))
                for name in expected
            ) else None,
            timeout=60.0,
        )
        component_ids = [c['id'] for c in data['items']]

        # Each demo node should appear as a component
        for name in expected:
            self.assertTrue(
                any(name in cid for cid in component_ids),
                f"{name} not found in components: {component_ids}",
            )

    def test_no_synthetic_namespace_components(self):
        """No synthetic components from namespace grouping should exist."""
        expected = ['temp_sensor', 'rpm_sensor', 'pressure_sensor']
        data = self.poll_endpoint_until(
            '/components',
            lambda d: d if all(
                any(name in c['id'] for c in d.get('items', []))
                for name in expected
            ) else None,
            timeout=60.0,
        )

        # With synthetic off, components should NOT have source="synthetic"
        for comp in data['items']:
            x_medkit = comp.get('x-medkit', {})
            source = x_medkit.get('source', '')
            self.assertNotEqual(
                source, 'synthetic',
                f"Component {comp['id']} has source=synthetic in legacy mode",
            )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
