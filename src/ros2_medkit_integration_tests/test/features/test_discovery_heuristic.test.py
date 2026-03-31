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
Integration tests for heuristic discovery - runtime Apps, Functions.

This test file validates the runtime discovery heuristics with SOVD-aligned model:
- Nodes are exposed as Apps with source="heuristic"
- Functions are created from namespace grouping (default)
- Areas are always empty (come from manifest only)
- Components come from HostInfoProvider (single host-level component)

Tests verify:
- Apps have correct source field
- Functions are created from namespaces
- Areas are empty in runtime mode
- HostInfoProvider component exists

"""

import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.coverage import get_coverage_env
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node


def generate_test_description():
    """Generate launch description with gateway in runtime_only mode."""
    coverage_env = get_coverage_env()

    # Gateway node with runtime_only discovery mode (default)
    # Functions from namespaces is enabled by default
    # HostInfoProvider creates the single host-level Component
    gateway_node = create_gateway_node()

    # Launch demo nodes to test heuristic discovery
    demo_nodes = [
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_engine_temp_sensor',
            name='temp_sensor',
            namespace='/powertrain/engine',
            output='screen',
            additional_env=coverage_env,
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_rpm_sensor',
            name='rpm_sensor',
            namespace='/powertrain/engine',
            output='screen',
            additional_env=coverage_env,
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_brake_pressure_sensor',
            name='pressure_sensor',
            namespace='/chassis/brakes',
            output='screen',
            additional_env=coverage_env,
        ),
        # Node in root namespace
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_engine_temp_sensor',
            name='root_ns_demo',
            namespace='/',
            output='screen',
            additional_env=coverage_env,
        ),
    ]

    return LaunchDescription(
        [
            gateway_node,
            *demo_nodes,
            TimerAction(
                period=5.0,
                actions=[
                    launch_testing.actions.ReadyToTest(),
                ],
            ),
        ]
    ), {
        'gateway_node': gateway_node,
    }


class TestHeuristicAppsDiscovery(GatewayTestCase):
    """Integration tests for heuristic runtime discovery of Apps."""

    MIN_EXPECTED_APPS = 3
    REQUIRED_FUNCTIONS = {'powertrain', 'chassis'}

    def test_apps_have_heuristic_source(self):
        """Test that runtime-discovered apps have source='heuristic'."""
        data = self.get_json('/apps')
        self.assertIn('items', data)
        apps = data['items']
        self.assertGreaterEqual(len(apps), self.MIN_EXPECTED_APPS)

        # Get detailed info for each app and verify source
        for app in apps:
            app_id = app.get('id')
            if not app_id:
                continue
            # Check source in list response's x-medkit
            x_medkit = app.get('x-medkit', {})
            self.assertIn(
                'source', x_medkit,
                f"App {app_id} missing 'source' field in x-medkit"
            )
            self.assertEqual(
                x_medkit['source'], 'heuristic',
                f"App {app_id} has source={x_medkit['source']}, expected 'heuristic'"
            )

    def test_host_component_created(self):
        """Test that a single host-level Component exists from HostInfoProvider."""
        data = self.get_json('/components')
        self.assertIn('items', data)
        components = data['items']

        # Should have exactly one host-derived component
        self.assertEqual(
            len(components), 1,
            f"Expected exactly 1 host component, found: {[c.get('id') for c in components]}"
        )

    def test_functions_created_from_namespaces(self):
        """Test that Functions are created from top-level namespaces."""
        data = self.get_json('/functions')
        self.assertIn('items', data)
        functions = data['items']

        # Should have functions for powertrain, chassis
        func_ids = [f.get('id') for f in functions]
        self.assertIn('powertrain', func_ids, f"Missing 'powertrain' function, found: {func_ids}")
        self.assertIn('chassis', func_ids, f"Missing 'chassis' function, found: {func_ids}")

    def test_areas_always_empty(self):
        """Test that areas are always empty in runtime mode - Areas come from manifest only."""
        data = self.get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']
        self.assertEqual(
            len(areas), 0,
            f'Expected empty areas in runtime mode, '
            f'got: {[a.get("id") for a in areas]}'
        )

    def test_no_duplicate_component_ids(self):
        """Test that component IDs are unique."""
        data = self.get_json('/components')
        self.assertIn('items', data)

        component_ids = [c['id'] for c in data['items']]

        # Count occurrences of each component ID
        id_counts = {}
        for comp_id in component_ids:
            id_counts[comp_id] = id_counts.get(comp_id, 0) + 1

        # Assert all counts are exactly 1 (no duplicates)
        duplicates = {cid: count for cid, count in id_counts.items() if count > 1}
        self.assertEqual(
            len(duplicates), 0,
            f'Found duplicate component IDs: {duplicates}'
        )

    def test_root_namespace_node_exists_as_app(self):
        """Test that root namespace node exists as an app."""
        # Verify root_ns_demo IS an app
        apps = self.get_json('/apps').get('items', [])
        app_ids = [a.get('id') for a in apps]
        self.assertIn(
            'root_ns_demo', app_ids,
            "'root_ns_demo' should exist as an app"
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown tests for heuristic apps discovery tests."""

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
