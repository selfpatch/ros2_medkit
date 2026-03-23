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

"""Integration tests for the procfs introspection plugin.

Validates that the procfs plugin correctly exposes per-app and per-component
process info (PID, RSS, thread count, etc.) via vendor extension endpoints.
Exercises the PID cache lookup, /proc reading, and Component-level
aggregation across child apps.
"""

import os
import time
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def _get_plugin_path(plugin_so_name):
    """Get path to a plugin .so file installed by ros2_medkit_linux_introspection."""
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_linux_introspection')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_linux_introspection', plugin_so_name
    )


def generate_test_description():
    """Launch gateway with the procfs plugin and demo nodes."""
    plugin_path = _get_plugin_path('libprocfs_introspection.so')
    return create_test_launch(
        demo_nodes=['temp_sensor', 'rpm_sensor'],
        fault_manager=False,
        gateway_params={
            'plugins': ['procfs'],
            'plugins.procfs.path': plugin_path,
        },
    )


class TestProcfsIntrospection(GatewayTestCase):
    """Test procfs plugin vendor extension endpoints.

    @verifies REQ_INTEROP_003
    """

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor'}

    def _get_any_app_id(self):
        """Get the first discovered app ID."""
        data = self.get_json('/apps')
        items = data.get('items', [])
        self.assertGreater(len(items), 0, 'No apps discovered')
        return items[0]['id']

    def _get_any_component_id(self):
        """Get the first discovered component ID."""
        data = self.get_json('/components')
        items = data.get('items', [])
        self.assertGreater(len(items), 0, 'No components discovered')
        return items[0]['id']

    def _poll_procfs_app(self, app_id, timeout=20.0):
        """Poll procfs endpoint until it returns valid data.

        The PID cache may not be populated on the first request (503),
        so we retry until data arrives.
        """
        start = time.monotonic()
        last_status = None
        while time.monotonic() - start < timeout:
            try:
                r = requests.get(
                    f'{self.BASE_URL}/apps/{app_id}/x-medkit-procfs',
                    timeout=5,
                )
                last_status = r.status_code
                if r.status_code == 200:
                    return r.json()
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        self.fail(
            f'Procfs data not available for app {app_id} after {timeout}s '
            f'(last status: {last_status})'
        )

    def _poll_procfs_component(self, comp_id, timeout=20.0):
        """Poll procfs component endpoint until it returns processes."""
        start = time.monotonic()
        last_status = None
        while time.monotonic() - start < timeout:
            try:
                r = requests.get(
                    f'{self.BASE_URL}/components/{comp_id}/x-medkit-procfs',
                    timeout=5,
                )
                last_status = r.status_code
                if r.status_code == 200:
                    data = r.json()
                    if data.get('processes'):
                        return data
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        self.fail(
            f'Procfs data not available for component {comp_id} after {timeout}s '
            f'(last status: {last_status})'
        )

    def test_01_app_procfs_returns_process_info(self):
        """GET /apps/{id}/x-medkit-procfs returns process info with valid fields."""
        app_id = self._get_any_app_id()
        data = self._poll_procfs_app(app_id)

        self.assertGreater(data['pid'], 0, 'PID should be positive')
        self.assertGreater(data['rss_bytes'], 0, 'RSS should be positive')
        self.assertIsInstance(data['exe'], str)
        self.assertTrue(len(data['exe']) > 0, 'exe_path should be non-empty')
        self.assertGreaterEqual(data['threads'], 1, 'Thread count should be >= 1')

    def test_02_component_procfs_returns_aggregation(self):
        """GET /components/{id}/x-medkit-procfs returns aggregated processes."""
        comp_id = self._get_any_component_id()
        data = self._poll_procfs_component(comp_id)

        processes = data['processes']
        self.assertGreater(len(processes), 0, 'Should have at least one process')

        for proc in processes:
            self.assertGreater(proc['pid'], 0, 'PID should be positive')
            self.assertGreater(proc['rss_bytes'], 0, 'RSS should be positive')
            self.assertIn('node_ids', proc)
            self.assertIsInstance(proc['node_ids'], list)
            self.assertGreater(
                len(proc['node_ids']), 0, 'node_ids should be non-empty'
            )

    def test_03_nonexistent_app_returns_404(self):
        """GET /apps/nonexistent-entity/x-medkit-procfs returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/apps/nonexistent-entity/x-medkit-procfs',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_04_capabilities_include_procfs(self):
        """Entity capabilities include x-medkit-procfs with correct href."""
        app_id = self._get_any_app_id()
        data = self.get_json(f'/apps/{app_id}')
        capabilities = data.get('capabilities', [])
        cap_names = [c['name'] for c in capabilities]
        self.assertIn(
            'x-medkit-procfs',
            cap_names,
            f'x-medkit-procfs not in capabilities: {cap_names}',
        )
        procfs_cap = next(
            c for c in capabilities if c['name'] == 'x-medkit-procfs'
        )
        self.assertIn(
            f'/apps/{app_id}/x-medkit-procfs', procfs_cap['href']
        )


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
