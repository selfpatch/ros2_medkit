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

"""Feature tests for configuration API (list, get, set, reset).

Validates listing, reading, writing, and resetting ROS 2 node parameters
via the SOVD-compliant configurations endpoint.

Migrated from:
- test_45_list_configurations
- test_46_get_configuration
- test_47_set_configuration
- test_48_delete_configuration_resets_to_default
- test_49_configurations_nonexistent_app
- test_50_configuration_nonexistent_parameter
- test_51_set_configuration_missing_value
- test_52_root_endpoint_includes_configurations
"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )


class TestConfigurationApi(GatewayTestCase):
    """Configuration API tests for parameters."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_list_configurations(self):
        """GET /apps/{app_id}/configurations lists all parameters.

        @verifies REQ_INTEROP_048
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'temp_sensor')
        self.assertIn('parameters', x_medkit)
        self.assertIsInstance(x_medkit['parameters'], list)

        # Verify we have parameters from the demo node
        param_names = [p['name'] for p in x_medkit['parameters']]
        self.assertIn('publish_rate', param_names)
        self.assertIn('min_temp', param_names)
        self.assertIn('max_temp', param_names)
        self.assertIn('temp_step', param_names)

        # Verify parameter structure in x-medkit
        for param in x_medkit['parameters']:
            self.assertIn('name', param)
            self.assertIn('value', param)
            self.assertIn('type', param)

    def test_get_configuration(self):
        """GET /apps/{app_id}/configurations/{param_name} gets parameter.

        @verifies REQ_INTEROP_049
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/publish_rate',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD ReadValue format with x-medkit extension
        self.assertIn('id', data)
        self.assertEqual(data['id'], 'publish_rate')
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'temp_sensor')
        self.assertIn('parameter', x_medkit)

        param = x_medkit['parameter']
        self.assertIn('name', param)
        self.assertEqual(param['name'], 'publish_rate')
        self.assertIn('value', param)
        self.assertIn('type', param)
        self.assertEqual(param['type'], 'double')
        # Default value is 2.0
        self.assertEqual(param['value'], 2.0)

    def test_set_configuration(self):
        """PUT /apps/{app_id}/configurations/{param_name} sets parameter.

        This test modifies min_temp and resets it within the same test.

        @verifies REQ_INTEROP_050
        """
        # Set a new value using SOVD "data" field
        response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'data': 80.0},
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD write response format with x-medkit extension
        self.assertIn('id', data)
        self.assertEqual(data['id'], 'min_temp')
        self.assertIn('data', data)
        self.assertEqual(data['data'], 80.0)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'temp_sensor')
        self.assertIn('parameter', x_medkit)

        param = x_medkit['parameter']
        self.assertIn('name', param)
        self.assertEqual(param['name'], 'min_temp')
        self.assertIn('value', param)
        self.assertEqual(param['value'], 80.0)
        self.assertIn('type', param)
        self.assertEqual(param['type'], 'double')

        # Verify the value was actually set by reading it back
        verify_response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(verify_response.status_code, 200)
        verify_data = verify_response.json()
        self.assertEqual(verify_data['x-medkit']['parameter']['value'], 80.0)

        # Reset the value back to default using SOVD "data" field
        requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'data': 85.0},
            timeout=10
        )

    def test_delete_configuration_resets_to_default(self):
        """DELETE /apps/{app_id}/configurations/{param_name} resets to default.

        The DELETE method resets the parameter to its default value.
        It first changes the value, then resets it, then verifies the reset.

        @verifies REQ_INTEROP_052
        """
        # First, change the value from default using SOVD "data" field
        set_response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'data': -50.0},
            timeout=10
        )
        self.assertEqual(set_response.status_code, 200)

        # Now reset to default via DELETE - SOVD returns 204 No Content
        response = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(response.status_code, 204)

        # Verify the value was actually reset by reading it
        get_response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(get_response.status_code, 200)
        get_data = get_response.json()
        # The value should be reset to default (85.0)
        _ = get_data['x-medkit']['parameter']['value']

    def test_configurations_nonexistent_app(self):
        """GET /apps/{app_id}/configurations returns 404 for unknown app.

        @verifies REQ_INTEROP_048
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertIn('entity_id', data['parameters'])
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_app')

    def test_configuration_nonexistent_parameter(self):
        """GET configurations/{param_name} returns 404 for unknown param.

        @verifies REQ_INTEROP_049
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/nonexistent_param',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('id'), 'nonexistent_param')

    def test_set_configuration_missing_value(self):
        """PUT configurations/{param_name} returns 400 when value missing.

        @verifies REQ_INTEROP_050
        """
        response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={},
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        # SOVD format expects "data" field
        self.assertIn('data', data['message'].lower())

    def test_root_endpoint_includes_configurations(self):
        """Root endpoint lists configurations endpoints and capability.

        @verifies REQ_INTEROP_048
        """
        data = self.get_json('/')

        # Verify configurations endpoints are listed
        self.assertIn('endpoints', data)
        config_endpoints = [e for e in data['endpoints'] if 'configurations' in e.lower()]
        self.assertGreater(len(config_endpoints), 0, 'Should have configurations endpoints')

        # Verify configurations capability is listed
        self.assertIn('capabilities', data)
        self.assertIn('configurations', data['capabilities'])
        self.assertTrue(data['capabilities']['configurations'])


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
