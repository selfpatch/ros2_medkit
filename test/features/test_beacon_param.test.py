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

"""Integration tests for parameter-based beacon discovery plugin.

Validates the pull-based beacon pipeline: demo_param_beacon_node declares
ros2_medkit.discovery.* parameters, the gateway's ParameterBeaconPlugin
polls them, stores hints, and serves enriched metadata at the vendor
extension endpoint GET /{entity_type}/{id}/x-medkit-param-beacon.

The gateway runs in hybrid mode with the parameter_beacon plugin loaded
and allow_new_entities=False.
"""

import os
import unittest

import launch_testing
import launch_testing.asserts
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def _get_param_beacon_plugin_path():
    """Get path to param_beacon_plugin.so."""
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_param_beacon')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_param_beacon', 'libparam_beacon_plugin.so'
    )


def generate_test_description():
    """Launch gateway with parameter_beacon plugin + param beacon demo node.

    Uses temp_sensor as the runtime-discovered entity. The param beacon node
    is launched with its entity_id parameter pointing at temp_sensor.
    """
    plugin_path = _get_param_beacon_plugin_path()
    launch_description, context = create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        gateway_params={
            'discovery.mode': 'hybrid',
            'plugins': ['parameter_beacon'],
            'plugins.parameter_beacon.path': plugin_path,
            'plugins.parameter_beacon.poll_interval_sec': 1.0,
            'plugins.parameter_beacon.beacon_ttl_sec': 5.0,
            'plugins.parameter_beacon.beacon_expiry_sec': 30.0,
            'plugins.parameter_beacon.allow_new_entities': False,
        },
    )

    from launch_ros.actions import Node as LaunchNode

    param_node = LaunchNode(
        package='ros2_medkit_integration_tests',
        executable='demo_param_beacon_node',
        name='param_beacon_node',
        namespace='/powertrain/engine',
        parameters=[{
            'ros2_medkit.discovery.entity_id': 'temp_sensor',
            'ros2_medkit.discovery.transport_type': 'shared_memory',
            'ros2_medkit.discovery.process_name': 'param_beacon_test',
            'ros2_medkit.discovery.hostname': 'test-host',
            'ros2_medkit.discovery.process_id': 9999,
            'ros2_medkit.discovery.stable_id': 'stable-temp-sensor',
        }],
    )
    launch_description.add_action(param_node)
    return launch_description, context


class TestBeaconParam(GatewayTestCase):
    """Parameter beacon discovery plugin integration tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_01_param_beacon_enrichment(self):
        """Verify entity metadata appears via parameter beacon vendor endpoint."""
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/x-medkit-param-beacon',
            lambda d: d if d.get('status') == 'active' else None,
            timeout=30,
        )
        self.assertEqual(data['entity_id'], 'temp_sensor')
        self.assertEqual(data['transport_type'], 'shared_memory')
        self.assertEqual(data['process_name'], 'param_beacon_test')
        self.assertEqual(data['hostname'], 'test-host')
        self.assertEqual(data['stable_id'], 'stable-temp-sensor')
        self.assertIn('age_sec', data)

    def test_02_param_beacon_capabilities_registered(self):
        """Verify app capabilities include x-medkit-param-beacon."""
        data = self.get_json('/apps/temp_sensor')
        capabilities = data.get('capabilities', [])
        cap_names = [c['name'] for c in capabilities]
        self.assertIn(
            'x-medkit-param-beacon',
            cap_names,
            f'x-medkit-param-beacon not in capabilities: {cap_names}',
        )
        beacon_cap = next(
            c for c in capabilities if c['name'] == 'x-medkit-param-beacon'
        )
        self.assertIn('/apps/temp_sensor/x-medkit-param-beacon', beacon_cap['href'])

    def test_03_param_beacon_not_found(self):
        """Verify 404 for entity without param beacon data."""
        r = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_entity/x-medkit-param-beacon',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify gateway exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
