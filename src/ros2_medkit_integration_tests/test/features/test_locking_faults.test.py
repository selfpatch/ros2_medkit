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

"""Integration test: global DELETE /faults respects entity locks.

Verifies that faults on locked entities are skipped during global clear,
while faults on unlocked entities are cleared normally.
"""

import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    """Launch gateway with locking, fault manager, and faulty lidar."""
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        gateway_params={
            'locking.enabled': True,
            'locking.default_max_expiration': 3600,
        },
        fault_manager=True,
        fault_manager_params={'confirmation_threshold': -2},
        lidar_faulty=True,
    )


class TestLockingFaults(GatewayTestCase):
    """Global fault clear respects entity locks."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    # @verifies REQ_INTEROP_100
    def test_global_clear_skips_locked_entity_faults(self):
        """Lock an app, global DELETE /faults skips its faults, unlock and clear works."""
        # Wait for lidar to produce a fault
        self.wait_for_fault('/apps/lidar_sensor', 'LIDAR_RANGE_INVALID')

        # Lock lidar_sensor as client_a
        resp = requests.post(
            f'{self.BASE_URL}/apps/lidar_sensor/locks',
            json={'lock_expiration': 300},
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']

        # Global clear as client_b - should skip locked faults
        resp = requests.delete(
            f'{self.BASE_URL}/faults',
            headers={'X-Client-Id': 'client_b'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 204, resp.text)

        # Fault should still exist (lock protected it)
        resp = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults',
            timeout=10,
        )
        self.assertEqual(resp.status_code, 200, resp.text)
        faults = resp.json().get('items', [])
        fault_codes = [f['fault_code'] for f in faults]
        self.assertIn(
            'LIDAR_RANGE_INVALID', fault_codes,
            'Locked fault should survive global clear',
        )

        # Release lock
        requests.delete(
            f'{self.BASE_URL}/apps/lidar_sensor/locks/{lock_id}',
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )

        # Global clear without lock - should now succeed
        resp = requests.delete(
            f'{self.BASE_URL}/faults',
            timeout=10,
        )
        self.assertEqual(resp.status_code, 204, resp.text)

        # Fault should be gone
        resp = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults',
            timeout=10,
        )
        self.assertEqual(resp.status_code, 200, resp.text)
        faults = resp.json().get('items', [])
        fault_codes = [f['fault_code'] for f in faults]
        self.assertNotIn('LIDAR_RANGE_INVALID', fault_codes, 'Unlocked fault should be cleared')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
