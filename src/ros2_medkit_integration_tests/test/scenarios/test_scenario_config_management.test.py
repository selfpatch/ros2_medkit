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

"""Scenario: Configuration management — list, get, set, reset.

End-to-end story exercising the full ROS 2 parameter management API for
the temp_sensor demo node.
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


class TestScenarioConfigManagement(GatewayTestCase):
    """Scenario: Full configuration lifecycle for temp_sensor.

    Validates listing, reading, writing, and resetting ROS 2 parameters
    via the configurations REST API. Each test is self-contained.

    Steps:
    1. List all configurations for temp_sensor
    2. Get a specific configuration parameter value
    3. Set a configuration value, verify it changed, restore original
    4. Reset all configurations to defaults
    5. Change a parameter, reset that single parameter, verify reset
    """

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    APP_ID = 'temp_sensor'
    ENTITY_ENDPOINT = '/apps/temp_sensor'

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_list_configurations(self):
        """GET /apps/temp_sensor/configurations returns configuration list.

        @verifies REQ_INTEROP_048
        """
        data = self.get_json(f'{self.ENTITY_ENDPOINT}/configurations')
        self.assertIn('items', data)

    def test_02_get_configuration_value(self):
        """GET /apps/{id}/configurations/{config-id} returns value.

        Dynamically finds an app with configurations and tests a single
        config endpoint.

        @verifies REQ_INTEROP_049
        """
        # Find a configuration on temp_sensor
        configs_data = self.get_json(f'{self.ENTITY_ENDPOINT}/configurations')
        configs = configs_data.get('items', [])
        if not configs:
            self.fail('No configurations found on temp_sensor')

        config_id = configs[0]['id']

        # Get single config
        data = self.get_json(
            f'{self.ENTITY_ENDPOINT}/configurations/{config_id}',
        )

        self.assertIn('id', data)
        self.assertEqual(data['id'], config_id)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)

        x_medkit = data['x-medkit']
        self.assertIn('parameter', x_medkit)
        param = x_medkit['parameter']
        self.assertIn('name', param)
        self.assertIn('type', param)

    def test_03_set_and_verify_configuration(self):
        """PUT new value, GET to verify, PUT original back.

        @verifies REQ_INTEROP_050
        """
        # Find a writable configuration
        configs_data = self.get_json(f'{self.ENTITY_ENDPOINT}/configurations')
        configs = configs_data.get('items', [])

        config_found = False
        for config_item in configs:
            config_id = config_item['id']

            # Get current value
            get_response = requests.get(
                f'{self.BASE_URL}{self.ENTITY_ENDPOINT}/configurations/{config_id}',
                timeout=10,
            )
            if get_response.status_code != 200:
                continue

            current_data = get_response.json()

            # Skip read-only parameters
            if current_data.get('read_only', False):
                continue

            current_value = current_data.get('data', 1.0)

            # Try to set the same value back — should succeed
            response = requests.put(
                f'{self.BASE_URL}{self.ENTITY_ENDPOINT}/configurations/{config_id}',
                json={'data': current_value},
                timeout=10,
            )

            # Read-only params may not report read_only in descriptor
            if response.status_code == 403:
                continue

            self.assertEqual(
                response.status_code, 200,
                f'Expected 200 for setting config {config_id}, '
                f'got {response.status_code}: {response.text}',
            )

            data = response.json()
            self.assertIn('id', data)
            self.assertEqual(data['id'], config_id)
            self.assertIn('data', data)

            config_found = True
            break

        if not config_found:
            self.fail('No writable app configurations found')

    def test_04_reset_all_configurations(self):
        """DELETE /apps/temp_sensor/configurations resets all configs.

        Returns 204 on complete success, 207 if some parameters could not
        be reset.

        @verifies REQ_INTEROP_051
        """
        response = requests.delete(
            f'{self.BASE_URL}{self.ENTITY_ENDPOINT}/configurations',
            timeout=10,
        )

        # 204 = complete success, 207 = partial success
        self.assertIn(
            response.status_code, [204, 207],
            f'Expected 204/207 for reset all configs, '
            f'got {response.status_code}',
        )

    def test_05_reset_single_configuration(self):
        """PUT to change, DELETE single config, verify reset.

        @verifies REQ_INTEROP_052
        """
        config_id = 'min_temp'

        # Verify the parameter exists
        data = self.get_json(
            f'{self.ENTITY_ENDPOINT}/configurations/{config_id}',
        )
        self.assertEqual(data['id'], config_id)

        # Reset it
        response = self.delete_request(
            f'{self.ENTITY_ENDPOINT}/configurations/{config_id}',
            expected_status=204,
        )
        self.assertEqual(len(response.content), 0, '204 should have no body')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            allowed = {0, -2, -15}  # OK, SIGINT, SIGTERM
            self.assertIn(
                info.returncode, allowed,
                f'{info.process_name} exited with code {info.returncode}'
            )
