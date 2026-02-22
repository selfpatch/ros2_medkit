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
        # threshold=-2 spaces out confirmations (needs 2 FAILED events).
        fault_manager_params={'confirmation_threshold': -2},
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

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_wait_for_faults_to_appear(self):
        """Wait for lidar fault, verify response structure.

        @verifies REQ_INTEROP_012
        """
        fault = self.poll_endpoint_until(
            f'{self.LIDAR_ENDPOINT}/faults',
            lambda d: next(iter(d.get('items', [])), None),
            timeout=30.0,
            interval=1.0,
        )

        # Basic fault structure verification
        self.assertIn('fault_code', fault)
        self.assertIsInstance(fault['fault_code'], str)
        self.assertGreater(len(fault['fault_code']), 0)

    def test_02_delete_all_faults_for_component(self):
        """DELETE /components/{id}/faults clears all faults for component.

        @verifies REQ_INTEROP_014
        """
        # The lidar_sensor is under /perception namespace
        component_id = 'perception'

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
        # Use lidar_sensor directly - only app with faults in this test
        app_id = 'lidar_sensor'

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

        # Delete the fault (confirmed present by wait_for_fault above)
        response = requests.delete(
            f'{self.BASE_URL}/apps/{app_id}/faults/{fault_code}',
            timeout=10,
        )
        self.assertEqual(
            response.status_code, 204,
            f'Expected 204 for deletion of confirmed fault, '
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
                f'{info.process_name} exited with code {info.returncode}'
            )
