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

"""Feature tests for entity type routing validation.

Validates that component routes reject app IDs and vice versa,
ensuring proper entity type enforcement across all resource endpoints.

Migrated from:
- test_116_component_route_rejects_app_id
- test_117_component_route_rejects_app_id_operations
- test_118_component_route_rejects_app_id_configurations
- test_119_component_route_rejects_app_id_faults
- test_120_app_routes_work_with_app_id
"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor', 'calibration_service'],
        fault_manager=False,
    )


class TestEntityRouting(GatewayTestCase):
    """Entity type routing validation tests."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'calibration'}

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_app_only_id(self):
        """Find an app ID that is not also a component ID."""
        apps_response = requests.get(f'{self.BASE_URL}/apps', timeout=10)
        self.assertEqual(apps_response.status_code, 200)

        apps = apps_response.json().get('items', [])
        if len(apps) == 0:
            self.skipTest('No apps available for testing')

        components_response = requests.get(f'{self.BASE_URL}/components', timeout=10)
        self.assertEqual(components_response.status_code, 200)

        components = components_response.json().get('items', [])
        component_ids = {c.get('id') for c in components if isinstance(c, dict) and 'id' in c}

        for app in apps:
            if not isinstance(app, dict) or 'id' not in app:
                continue
            if app['id'] not in component_ids:
                return app['id']

        self.skipTest('No app ID available that is distinct from component IDs')

    # ------------------------------------------------------------------
    # Component route rejects app IDs (test_116-119)
    # ------------------------------------------------------------------

    def test_component_route_rejects_app_id_data(self):
        """Component /data rejects app IDs.

        In runtime-only discovery mode, /components/{id} should only accept
        synthetic component IDs, not individual ROS 2 node (app) IDs.

        @verifies REQ_INTEROP_003
        """
        app_id = self._find_app_only_id()

        # Verify this ID is recognized as an app via /apps/{id}
        app_response = requests.get(f'{self.BASE_URL}/apps/{app_id}', timeout=10)
        self.assertEqual(app_response.status_code, 200)

        # Now try to use this app ID with /components/{id}/data - should fail
        response = requests.get(
            f'{self.BASE_URL}/components/{app_id}/data',
            timeout=10
        )
        self.assertEqual(
            response.status_code,
            400,
            f'Expected 400 when using app ID "{app_id}" with /components route'
        )

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['error_code'], 'invalid-parameter')
        self.assertIn('Invalid entity type for route', data['message'])
        self.assertIn('expected_type', data.get('parameters', {}))
        self.assertIn('actual_type', data.get('parameters', {}))
        self.assertEqual(data['parameters']['expected_type'], 'Component')
        self.assertEqual(data['parameters']['actual_type'], 'App')

    def test_component_route_rejects_app_id_operations(self):
        """Component /operations rejects app IDs.

        @verifies REQ_INTEROP_003
        """
        app_id = self._find_app_only_id()

        response = requests.get(
            f'{self.BASE_URL}/components/{app_id}/operations',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertEqual(data['error_code'], 'invalid-parameter')
        self.assertIn('Invalid entity type for route', data['message'])

    def test_component_route_rejects_app_id_configurations(self):
        """Component /configurations rejects app IDs.

        @verifies REQ_INTEROP_003
        """
        app_id = self._find_app_only_id()

        response = requests.get(
            f'{self.BASE_URL}/components/{app_id}/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertEqual(data['error_code'], 'invalid-parameter')
        self.assertIn('Invalid entity type for route', data['message'])

    def test_component_route_rejects_app_id_faults(self):
        """Component /faults rejects app IDs.

        @verifies REQ_INTEROP_003
        """
        app_id = self._find_app_only_id()

        response = requests.get(
            f'{self.BASE_URL}/components/{app_id}/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertEqual(data['error_code'], 'invalid-parameter')
        self.assertIn('Invalid entity type for route', data['message'])

    # ------------------------------------------------------------------
    # App route works with app IDs (test_120)
    # ------------------------------------------------------------------

    def test_app_routes_work_with_app_id(self):
        """App /data works correctly with app IDs.

        Verify that while /components rejects app IDs, /apps accepts them.

        @verifies REQ_INTEROP_003
        """
        apps_response = requests.get(f'{self.BASE_URL}/apps', timeout=10)
        self.assertEqual(apps_response.status_code, 200)

        apps = apps_response.json().get('items', [])
        if len(apps) == 0:
            self.skipTest('No apps available for testing')

        app_id = apps[0]['id']

        # /apps/{id}/data should work
        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)


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
