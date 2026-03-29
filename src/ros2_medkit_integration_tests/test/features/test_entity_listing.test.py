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

"""Feature tests for entity listing endpoints (areas, components, apps, functions).

Validates discovery of entities with the SOVD-aligned entity model:
- Areas are empty in runtime_only mode (no synthetic areas)
- Components returns a single host-derived default Component
- Functions are created from namespace grouping
- Apps are ROS 2 nodes (unchanged)

"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import ALL_DEMO_NODES, create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=ALL_DEMO_NODES,
        fault_manager=False,
    )


class TestEntityListing(GatewayTestCase):
    """Entity listing, functions, components, and ID validation tests."""

    MIN_EXPECTED_APPS = 8
    REQUIRED_FUNCTIONS = {'powertrain', 'chassis', 'body'}
    REQUIRED_APPS = {'temp_sensor', 'long_calibration', 'lidar_sensor', 'actuator'}

    # ------------------------------------------------------------------
    # Area listing (empty in runtime_only mode)
    # ------------------------------------------------------------------

    def test_list_areas_empty_in_runtime_mode(self):
        """GET /areas returns empty items in runtime_only mode.

        With the SOVD-aligned entity model, areas are not created from
        namespaces in runtime_only mode. Namespace grouping creates
        Functions instead.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']
        self.assertIsInstance(areas, list)
        self.assertEqual(len(areas), 0, 'Areas should be empty in runtime_only mode')

    # ------------------------------------------------------------------
    # Component listing (single host component)
    # ------------------------------------------------------------------

    def test_list_components(self):
        """GET /components returns the single host-derived default Component.

        With the SOVD-aligned entity model, runtime_only mode exposes a
        single Component derived from the host system info (hostname, OS,
        architecture) rather than synthetic namespace-based components.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertIsInstance(components, list)
        # Exactly one default host component
        self.assertEqual(
            len(components), 1,
            f'Expected exactly 1 host component, got {len(components)}: '
            f'{[c.get("id") for c in components]}'
        )

        # Verify response structure
        component = components[0]
        self.assertIn('id', component)
        self.assertIn('name', component)
        self.assertIn('href', component)
        # x-medkit contains source info
        self.assertIn('x-medkit', component)
        x_medkit = component['x-medkit']
        self.assertEqual(
            x_medkit.get('source'), 'runtime',
            'Host component should have source=runtime'
        )

    # ------------------------------------------------------------------
    # Function listing (namespace-derived)
    # ------------------------------------------------------------------

    def test_list_functions(self):
        """GET /functions returns namespace-derived Functions.

        With the SOVD-aligned entity model, namespace grouping creates
        Function entities instead of Areas/Components.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/functions')
        self.assertIn('items', data)
        functions = data['items']
        self.assertIsInstance(functions, list)
        self.assertGreaterEqual(
            len(functions), 3,
            'Expected at least 3 functions from namespace grouping'
        )

        # Verify expected function IDs from demo node namespaces
        func_ids = [f['id'] for f in functions]
        self.assertIn('powertrain', func_ids)
        self.assertIn('chassis', func_ids)
        self.assertIn('body', func_ids)

        # Verify response structure
        for func in functions:
            self.assertIn('id', func)
            self.assertIn('name', func)
            self.assertIn('href', func)

    def test_function_detail_accessible(self):
        """GET /functions/{id} returns function detail for namespace-derived function.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/functions/powertrain')
        self.assertIn('id', data)
        self.assertEqual(data['id'], 'powertrain')

    # ------------------------------------------------------------------
    # Area components (404 in runtime_only mode)
    # ------------------------------------------------------------------

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
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
