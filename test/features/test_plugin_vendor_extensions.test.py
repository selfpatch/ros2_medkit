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

"""Integration tests for plugin vendor extension endpoints.

Validates that plugins can register per-entity-type capabilities via
register_capability() and serve entity-scoped vendor extension resources
via register_routes().

Uses test_gateway_plugin which registers an "x-medkit-diagnostics"
capability for all Components and serves diagnostic data at
GET /components/{id}/x-medkit-diagnostics.
"""

import os
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def _get_test_plugin_path():
    """Get path to test_gateway_plugin.so."""
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_gateway', 'libtest_gateway_plugin.so'
    )


def generate_test_description():
    """Launch gateway with test_gateway_plugin and a few demo nodes."""
    plugin_path = _get_test_plugin_path()
    return create_test_launch(
        demo_nodes=['temp_sensor', 'rpm_sensor'],
        fault_manager=False,
        gateway_params={
            'plugins': ['test_plugin'],
            'plugins.test_plugin.path': plugin_path,
        },
    )


class TestPluginVendorExtensions(GatewayTestCase):
    """Vendor extension endpoint tests via test_gateway_plugin.

    @verifies REQ_INTEROP_003
    """

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor'}

    def _get_any_component_id(self):
        """Get the first discovered component ID."""
        data = self.get_json('/components')
        items = data.get('items', [])
        self.assertGreater(len(items), 0, 'No components discovered')
        return items[0]['id']

    def test_01_vendor_extension_endpoint_returns_data(self):
        """GET /components/{id}/x-medkit-diagnostics returns plugin data."""
        comp_id = self._get_any_component_id()
        data = self.get_json(f'/components/{comp_id}/x-medkit-diagnostics')
        self.assertEqual(data['entity_id'], comp_id)
        self.assertEqual(data['plugin'], 'test_plugin')
        self.assertIn('cpu_usage', data)
        self.assertIn('memory_mb', data)
        self.assertIn('uptime_seconds', data)

    def test_02_capabilities_include_vendor_extension(self):
        """Entity capabilities include the plugin-registered capability."""
        comp_id = self._get_any_component_id()
        data = self.get_json(f'/components/{comp_id}')
        capabilities = data.get('capabilities', [])
        cap_names = [c['name'] for c in capabilities]
        self.assertIn(
            'x-medkit-diagnostics',
            cap_names,
            f'x-medkit-diagnostics not in capabilities: {cap_names}',
        )
        # Verify href points to the correct path
        diag_cap = next(
            c for c in capabilities if c['name'] == 'x-medkit-diagnostics'
        )
        self.assertIn(f'/components/{comp_id}/x-medkit-diagnostics', diag_cap['href'])

    def test_03_vendor_extension_nonexistent_entity_returns_404(self):
        """GET /components/nonexistent/x-medkit-diagnostics returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/components/nonexistent-entity/x-medkit-diagnostics',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_04_global_vendor_endpoint_still_works(self):
        """GET /x-test/ping global endpoint still responds."""
        r = requests.get(f'{self.BASE_URL}/x-test/ping', timeout=5)
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.text, 'pong')


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
