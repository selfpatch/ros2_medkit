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

"""Feature tests for lifecycle status endpoints (GET /status, PUT /status/{action}).

Validates the default (no-provider) lifecycle status path:
- GET /{entity}/status returns 200 with status == "ready" for online entities.
- PUT /{entity}/status/{action} returns 501 when no lifecycle provider is registered.

"""

import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )


# @verifies REQ_INTEROP_076
class TestLifecycleStatus(GatewayTestCase):
    """Lifecycle status endpoint tests for apps and components."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    # Cache for the dynamically-discovered host component ID.
    _host_component_id = None

    def _get_host_component_id(self):
        """Get the host component ID (cached after first lookup)."""
        if TestLifecycleStatus._host_component_id is not None:
            return TestLifecycleStatus._host_component_id

        data = self.get_json('/components')
        components = data.get('items', [])
        self.assertGreater(
            len(components), 0,
            'Expected at least 1 host component, got none'
        )
        TestLifecycleStatus._host_component_id = components[0]['id']
        return TestLifecycleStatus._host_component_id

    def test_app_status_ready(self):
        """GET /apps/{app_id}/status returns 200 with status == "ready" for an online app.

        @verifies REQ_INTEROP_076
        """
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/status',
            condition=lambda d: d if d.get('status') == 'ready' else None,
            timeout=15.0,
        )
        self.assertEqual(data.get('status'), 'ready')

    def test_component_status_ready(self):
        """GET /components/{component_id}/status returns 200 with status == "ready".

        @verifies REQ_INTEROP_076
        """
        host_id = self._get_host_component_id()
        data = self.poll_endpoint_until(
            f'/components/{host_id}/status',
            condition=lambda d: d if d.get('status') == 'ready' else None,
            timeout=15.0,
        )
        self.assertEqual(data.get('status'), 'ready')

    def test_app_status_restart_returns_501(self):
        """PUT /apps/{app_id}/status/restart returns 501 when no lifecycle provider is registered.

        @verifies REQ_INTEROP_076
        """
        response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/status/restart',
            json={},
            timeout=10,
        )
        self.assertEqual(response.status_code, 501)
        data = response.json()
        self.assertIn('error_code', data)

    def test_app_status_structure(self):
        """GET /apps/{app_id}/status response has required 'status' field.

        @verifies REQ_INTEROP_076
        """
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/status',
            condition=lambda d: d if 'status' in d else None,
            timeout=15.0,
        )
        self.assertIn('status', data)
        self.assertIn(data['status'], ['ready', 'notReady'])

    def test_app_status_nonexistent_returns_404(self):
        """GET /apps/{app_id}/status returns 404 for a nonexistent app.

        @verifies REQ_INTEROP_076
        """
        response = self.get_raw('/apps/nonexistent_app/status', expected_status=404)
        data = response.json()
        self.assertIn('error_code', data)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
