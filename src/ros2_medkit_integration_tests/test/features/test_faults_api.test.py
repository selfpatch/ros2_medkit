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

"""Feature tests for faults API.

Validates fault listing, fault response structure, status filters,
error handling for nonexistent entities and invalid parameters.

Migrated from:
- test_55_root_endpoint_includes_faults
- test_56_list_faults_response_structure
- test_57_faults_nonexistent_component
- test_58_get_nonexistent_fault
- test_59_list_all_faults_globally
- test_60_list_all_faults_with_status_filter
- test_61_list_faults_invalid_status_returns_400
- test_62_component_faults_invalid_status_returns_400
"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        lidar_faulty=True,
    )


class TestFaultsApi(GatewayTestCase):
    """Faults API tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    def test_root_endpoint_includes_faults(self):
        """Root endpoint lists faults endpoints and capability.

        @verifies REQ_INTEROP_012
        """
        data = self.get_json('/')

        # Verify faults endpoints are listed
        self.assertIn('endpoints', data)
        self.assertIn(
            'GET /api/v1/faults',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults/{fault_code}',
            data['endpoints']
        )
        self.assertIn(
            'DELETE /api/v1/components/{component_id}/faults/{fault_code}',
            data['endpoints']
        )

        # Verify faults capability is listed
        self.assertIn('capabilities', data)
        self.assertIn('faults', data['capabilities'])
        self.assertTrue(data['capabilities']['faults'])

    def test_list_faults_response_structure(self):
        """GET /apps/{app_id}/faults returns valid response structure.

        In the heuristic discovery model, ROS nodes are Apps.

        @verifies REQ_INTEROP_012
        """
        # Use lidar_sensor which is more likely to have faults
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'lidar_sensor')
        self.assertIn('source_id', x_medkit)
        self.assertIn('count', x_medkit)

    def test_faults_nonexistent_component(self):
        """GET /components/{component_id}/faults returns 404 for unknown entity.

        @verifies REQ_INTEROP_012
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        # SOVD error format: parameters in parameters field
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_component')

    def test_get_nonexistent_fault(self):
        """GET /apps/{app_id}/faults/{fault_code} returns 404.

        @verifies REQ_INTEROP_013
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults/NONEXISTENT_FAULT',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        # SOVD error format: parameters in parameters field
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('fault_code'), 'NONEXISTENT_FAULT')

    def test_list_all_faults_globally(self):
        """GET /faults returns all faults across the system.

        This is a convenience API for dashboards and monitoring tools.
        """
        response = requests.get(
            f'{self.BASE_URL}/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('count', x_medkit)
        self.assertIsInstance(x_medkit['count'], int)
        self.assertEqual(x_medkit['count'], len(data['items']))

    def test_list_all_faults_with_status_filter(self):
        """GET /faults?status={status} filters faults by status."""
        # Test with status=all
        response = requests.get(
            f'{self.BASE_URL}/faults?status=all',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)
        self.assertIn('count', data['x-medkit'])

        # Test other valid status values (including healed)
        for status in ['pending', 'confirmed', 'cleared', 'healed']:
            response = requests.get(
                f'{self.BASE_URL}/faults?status={status}',
                timeout=10
            )
            self.assertEqual(response.status_code, 200)

    def test_list_faults_invalid_status_returns_400(self):
        """GET /faults?status=invalid returns 400 Bad Request."""
        response = requests.get(
            f'{self.BASE_URL}/faults?status=invalid_status',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Invalid status parameter value')
        # Check parameters in parameters field
        self.assertIn('parameters', data)
        params = data['parameters']
        self.assertIn('allowed_values', params)
        self.assertIn('pending', params['allowed_values'])
        self.assertIn('healed', params['allowed_values'])
        self.assertIn('parameter', params)
        self.assertEqual(params.get('parameter'), 'status')
        self.assertIn('value', params)
        self.assertEqual(params['value'], 'invalid_status')

    def test_component_faults_invalid_status_returns_400(self):
        """GET /apps/{id}/faults?status=invalid returns 400."""
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults?status=bogus',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Invalid status parameter value')
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('app_id'), 'lidar_sensor')


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
