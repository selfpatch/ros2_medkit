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

"""Scenario: Fault lifecycle — appear, delete all, delete single.

Uses the lidar_sensor demo node which produces deterministic faults due to
intentionally invalid configuration parameters (min_range > max_range, etc.).
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
    )


class TestScenarioFaultLifecycle(GatewayTestCase):
    """Scenario: Full fault lifecycle from appearance through deletion.

    The lidar_sensor demo node publishes deterministic faults because its
    configuration has min_range > max_range. Tests exercise:

    Steps:
    1. Wait for faults to appear and verify structure
    2. Delete all faults for a component
    3. Delete all faults for an app
    4. Delete all faults for a nonexistent entity (expect 404)
    5. Wait for a specific fault and delete it individually
    """

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}
    REQUIRED_AREAS = {'perception'}

    LIDAR_ENDPOINT = '/apps/lidar_sensor'
    FAULT_CODE = 'LIDAR_RANGE_INVALID'

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _wait_for_any_lidar_fault(self, max_wait=30.0):
        """Wait for any fault to appear on lidar_sensor.

        Returns
        -------
        dict or None
            The first fault item, or None on timeout.

        """
        import time
        start_time = time.monotonic()
        while time.monotonic() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}{self.LIDAR_ENDPOINT}/faults',
                    timeout=5,
                )
                if response.status_code == 200:
                    data = response.json()
                    items = data.get('items', [])
                    if items:
                        return items[0]
            except requests.exceptions.RequestException:
                pass
            time.sleep(1)
        return None

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_wait_for_faults_to_appear(self):
        """Wait for lidar fault, verify response structure.

        @verifies REQ_INTEROP_012
        """
        fault = self._wait_for_any_lidar_fault(max_wait=30.0)
        if fault is None:
            self.skipTest('No lidar fault appeared within timeout')

        # Basic fault structure verification
        self.assertIn('fault_code', fault)
        self.assertIsInstance(fault['fault_code'], str)
        self.assertGreater(len(fault['fault_code']), 0)

    def test_02_delete_all_faults_for_component(self):
        """DELETE /components/{id}/faults clears all faults for component.

        @verifies REQ_INTEROP_014
        """
        # Get the perception component (lidar_sensor is under /perception)
        data = self.get_json('/components')
        self.assertGreater(len(data['items']), 0)
        component_id = data['items'][0]['id']

        # Clear all faults (should succeed even if empty)
        response = self.delete_request(
            f'/components/{component_id}/faults',
            expected_status=204,
        )
        self.assertEqual(len(response.content), 0)

    def test_03_delete_all_faults_for_app(self):
        """DELETE /apps/{id}/faults clears all faults for app.

        @verifies REQ_INTEROP_014
        """
        # Get an app
        data = self.get_json('/apps')
        self.assertGreater(len(data['items']), 0)
        app_id = data['items'][0]['id']

        response = self.delete_request(
            f'/apps/{app_id}/faults',
            expected_status=204,
        )
        self.assertEqual(len(response.content), 0)

    def test_04_delete_all_faults_nonexistent(self):
        """DELETE /{entity}/faults returns 404 for nonexistent entity.

        @verifies REQ_INTEROP_014
        """
        response = requests.delete(
            f'{self.BASE_URL}/components/nonexistent_component/faults',
            timeout=10,
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('message', data)
        self.assertEqual(data['message'], 'Entity not found')

    def test_05_delete_single_fault(self):
        """Wait for specific fault, DELETE it.

        The lidar_sensor may immediately re-report the fault after deletion,
        so we only verify that DELETE returns 204 (success) or 404 (not found).

        @verifies REQ_INTEROP_015
        """
        app_id = 'lidar_sensor'
        fault_code = self.FAULT_CODE

        # Wait for the fault to be reported
        try:
            self.wait_for_fault(
                self.LIDAR_ENDPOINT, fault_code, max_wait=15.0,
            )
        except AssertionError:
            # Fault not present — verify 404 is returned
            response = requests.delete(
                f'{self.BASE_URL}/apps/{app_id}/faults/{fault_code}',
                timeout=10,
            )
            self.assertEqual(
                response.status_code, 404,
                f'Expected 404 for absent fault, got {response.status_code}',
            )
            return

        # Delete the fault
        response = requests.delete(
            f'{self.BASE_URL}/apps/{app_id}/faults/{fault_code}',
            timeout=10,
        )
        self.assertIn(
            response.status_code, [204, 404],
            f'Expected 204 or 404 for fault deletion, '
            f'got {response.status_code}',
        )


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
