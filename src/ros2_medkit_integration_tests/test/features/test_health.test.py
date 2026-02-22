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

"""Feature tests for health, version, root, and docs endpoints.

Validates the gateway's informational endpoints that do not depend on
discovered entities. Only a single lightweight demo node (temp_sensor)
is launched so the test starts quickly.

"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )


class TestHealth(GatewayTestCase):
    """Health, version, root endpoint, and docs feature tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_health_endpoint_returns_200(self):
        """GET /health returns 200 with status field.

        @verifies REQ_INTEROP_010
        """
        data = self.get_json('/health')
        self.assertIn('status', data)

    def test_root_endpoint_returns_api_info(self):
        """GET / returns server capabilities and entry points.

        @verifies REQ_INTEROP_010
        """
        data = self.get_json('/')
        self.assertIn('name', data)
        self.assertIn('version', data)
        self.assertIn('endpoints', data)
        self.assertIn('capabilities', data)

        self.assertEqual(data['name'], 'ROS 2 Medkit Gateway')
        self.assertRegex(
            data['version'],
            r'^\d+\.\d+\.\d+$',
            f'Version should be semver format, got: {data["version"]}'
        )

        # Verify endpoints list
        self.assertIsInstance(data['endpoints'], list)
        self.assertIn('GET /api/v1/health', data['endpoints'])
        self.assertIn('GET /api/v1/version-info', data['endpoints'])
        self.assertIn('GET /api/v1/areas', data['endpoints'])
        self.assertIn('GET /api/v1/components', data['endpoints'])
        self.assertIn(
            'PUT /api/v1/components/{component_id}/data/{data_id}', data['endpoints']
        )

        # Verify api_base field
        self.assertIn('api_base', data)
        self.assertEqual(data['api_base'], API_BASE_PATH)

        # Verify capabilities
        self.assertIn('discovery', data['capabilities'])
        self.assertIn('data_access', data['capabilities'])
        self.assertTrue(data['capabilities']['discovery'])
        self.assertTrue(data['capabilities']['data_access'])

    def test_version_endpoint(self):
        """GET /version-info returns valid format and data.

        @verifies REQ_INTEROP_001
        """
        data = self.get_json('/version-info')

        # Check sovd_info array
        self.assertIn('sovd_info', data)
        self.assertIsInstance(data['sovd_info'], list)
        self.assertGreaterEqual(len(data['sovd_info']), 1)

        # Check first sovd_info entry
        info = data['sovd_info'][0]
        self.assertIn('version', info)
        self.assertIn('base_uri', info)
        self.assertIn('vendor_info', info)
        self.assertIn('version', info['vendor_info'])
        self.assertIn('name', info['vendor_info'])
        self.assertEqual(info['vendor_info']['name'], 'ros2_medkit')

    def test_root_includes_apps_endpoints(self):
        """GET / includes apps endpoints in the endpoints list.

        @verifies REQ_INTEROP_010
        """
        data = self.get_json('/')
        self.assertIn('endpoints', data)

        endpoints = data['endpoints']
        self.assertIn('GET /api/v1/apps', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/depends-on', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/data', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/operations', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/configurations', endpoints)

    def test_docs_endpoint(self):
        """GET /components/docs returns 404 (docs not yet implemented).

        TODO(#135): Change to 200 when docs endpoint is implemented.
        """
        response = requests.get(f'{self.BASE_URL}/components/docs', timeout=10)

        # Currently not implemented - 'docs' is treated as component ID
        self.assertEqual(
            response.status_code, 404,
            'Docs endpoint not implemented - returns 404 (component "docs" not found)'
        )


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
