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

"""Feature tests for entity listing endpoints (areas, components, apps).

Validates discovery of entities, area components, and entity ID validation.

Migrated from:
- test_02_list_areas
- test_03_list_components
- test_04_automotive_areas_discovery
- test_05_area_components_success
- test_06_area_components_nonexistent_error
- test_13_invalid_app_id_special_chars
- test_14_invalid_area_id_special_chars
- test_15_valid_ids_with_underscores
- test_16_invalid_ids_with_special_chars
"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import ALL_DEMO_NODES, create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=ALL_DEMO_NODES,
        fault_manager=False,
    )


class TestEntityListing(GatewayTestCase):
    """Entity listing, area components, and ID validation tests."""

    MIN_EXPECTED_APPS = 8
    REQUIRED_AREAS = {'powertrain', 'chassis', 'body'}
    REQUIRED_APPS = {'temp_sensor', 'long_calibration', 'lidar_sensor', 'actuator'}

    # ------------------------------------------------------------------
    # Area listing
    # ------------------------------------------------------------------

    def test_list_areas(self):
        """GET /areas returns all discovered areas.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']
        self.assertIsInstance(areas, list)
        self.assertGreaterEqual(len(areas), 1)
        area_ids = [area['id'] for area in areas]
        self.assertIn('root', area_ids)

    def test_automotive_areas_discovery(self):
        """GET /areas returns expected automotive areas (powertrain, chassis, body).

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/areas')
        areas = data['items']
        area_ids = [area['id'] for area in areas]

        expected_areas = ['powertrain', 'chassis', 'body']
        for expected in expected_areas:
            self.assertIn(expected, area_ids)

    # ------------------------------------------------------------------
    # Component listing
    # ------------------------------------------------------------------

    def test_list_components(self):
        """GET /components returns all discovered synthetic components.

        With heuristic discovery (default), components are synthetic groups
        created by namespace aggregation. ROS 2 nodes are exposed as Apps.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertIsInstance(components, list)
        # With synthetic components, we have fewer components (grouped by namespace)
        # Expected: powertrain, chassis, body, perception, root (at minimum)
        self.assertGreaterEqual(len(components), 4)

        # Verify response structure - all components should have required fields
        for component in components:
            self.assertIn('id', component)
            self.assertIn('name', component)
            self.assertIn('href', component)
            # x-medkit contains ROS2-specific fields
            self.assertIn('x-medkit', component)
            x_medkit = component['x-medkit']
            # namespace may be in x-medkit.ros2.namespace
            self.assertTrue(
                'ros2' in x_medkit and 'namespace' in x_medkit.get('ros2', {}),
                f"Component {component['id']} should have namespace in x-medkit.ros2"
            )

        # Verify expected synthetic component IDs are present
        component_ids = [comp['id'] for comp in components]
        self.assertIn('powertrain', component_ids)
        self.assertIn('chassis', component_ids)
        self.assertIn('body', component_ids)

    # ------------------------------------------------------------------
    # Area components
    # ------------------------------------------------------------------

    def test_area_components_success(self):
        """GET /areas/{area_id}/components returns components for valid area.

        With synthetic components, the powertrain area contains the 'powertrain'
        synthetic component which aggregates all ROS 2 nodes in that namespace.

        @verifies REQ_INTEROP_006
        """
        # Test powertrain area
        data = self.get_json('/areas/powertrain/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertIsInstance(components, list)
        self.assertGreater(len(components), 0)

        # All components should have EntityReference format with x-medkit
        for component in components:
            self.assertIn('id', component)
            self.assertIn('name', component)
            self.assertIn('href', component)
            self.assertIn('x-medkit', component)
            # Verify namespace is in x-medkit.ros2
            x_medkit = component['x-medkit']
            self.assertTrue(
                'ros2' in x_medkit and 'namespace' in x_medkit.get('ros2', {}),
                'Component should have namespace in x-medkit.ros2'
            )

        # Verify the synthetic 'powertrain' component exists
        component_ids = [comp['id'] for comp in components]
        self.assertIn('powertrain', component_ids)

    def test_area_components_nonexistent_error(self):
        """GET /areas/{area_id}/components returns 404 for nonexistent area.

        @verifies REQ_INTEROP_006
        """
        response = self.get_raw('/areas/nonexistent/components', expected_status=404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Area not found')
        self.assertIn('parameters', data)
        self.assertIn('area_id', data['parameters'])
        self.assertEqual(data['parameters'].get('area_id'), 'nonexistent')

    # ------------------------------------------------------------------
    # Entity ID validation
    # ------------------------------------------------------------------

    def test_invalid_app_id_special_chars(self):
        """GET /apps/{app_id}/data rejects special characters.

        @verifies REQ_INTEROP_018
        """
        # Test various invalid characters
        invalid_ids = [
            'app;drop',  # SQL injection attempt
            'app<script>',  # XSS attempt
            'app"test',  # Quote
            'app`test',  # Backtick
            'app$test',  # Dollar sign
            'app|test',  # Pipe
            'app&test',  # Ampersand
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/apps/{invalid_id}/data', timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for app_id: {invalid_id}',
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid entity ID')
            self.assertIn('parameters', data)
            self.assertIn('details', data['parameters'])

    def test_invalid_area_id_special_chars(self):
        """GET /areas/{area_id}/components rejects special characters.

        @verifies REQ_INTEROP_006
        """
        # Test various invalid characters
        # Note: Forward slash is handled by URL routing, not validation
        invalid_ids = [
            'area;drop',  # SQL injection attempt
            'area<script>',  # XSS attempt
            'area"test',  # Quote
            'area|test',  # Pipe
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/areas/{invalid_id}/components', timeout=5
            )
            self.assertEqual(
                response.status_code, 400, f'Expected 400 for area_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid area ID')
            self.assertIn('parameters', data)
            self.assertIn('details', data['parameters'])

    def test_valid_ids_with_underscores(self):
        """Valid IDs with underscores are accepted (ROS 2 naming).

        @verifies REQ_INTEROP_018
        """
        # While these IDs don't exist in the test environment,
        # they should pass validation and return 404 (not 400)
        valid_ids = [
            'app_name',  # Underscore
            'app_name_123',  # Underscore and numbers
            'AppName',  # CamelCase
            'app123',  # Alphanumeric
        ]

        for valid_id in valid_ids:
            response = requests.get(
                f'{self.BASE_URL}/apps/{valid_id}/data', timeout=5
            )
            # Should return 404 (not found) not 400 (invalid)
            self.assertEqual(
                response.status_code,
                404,
                f'Expected 404 for valid but nonexistent ID: {valid_id}',
            )

    def test_invalid_ids_with_special_chars(self):
        """Reject IDs with special chars (except underscore/hyphen).

        @verifies REQ_INTEROP_018
        """
        invalid_ids = [
            'app@name',
            'app name',
            'app!name',
            'app$name',
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/apps/{invalid_id}/data', timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for invalid ID: {invalid_id}',
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid entity ID')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            allowed = {0, -2, -15}  # OK, SIGINT, SIGTERM
            self.assertIn(
                info.returncode, allowed,
                f'Process {info.process_name} exited with code {info.returncode}'
            )
