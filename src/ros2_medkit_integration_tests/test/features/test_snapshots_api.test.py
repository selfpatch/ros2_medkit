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

"""Feature tests for legacy snapshot endpoints.

Validates that legacy snapshot endpoints have been removed and return 404.
The new snapshot data is available inline in fault responses and via
bulk-data endpoints.

Migrated from:
- test_65_root_endpoint_includes_snapshots
- test_66_get_snapshots_nonexistent_fault
- test_67_get_component_snapshots_nonexistent_fault
- test_68_get_snapshots_nonexistent_component
- test_69_get_snapshots_invalid_component_id
- test_107_get_rosbag_nonexistent_fault
- test_108_get_rosbag_invalid_fault_code
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
        lidar_faulty=True,
    )


class TestSnapshotsApi(GatewayTestCase):
    """Legacy snapshot endpoint tests (all return 404 since endpoints removed)."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    def test_root_endpoint_includes_snapshots(self):
        """Root endpoint lists snapshots endpoints.

        @verifies REQ_INTEROP_088
        """
        data = self.get_json('/')

        # Verify snapshots endpoints are listed
        self.assertIn('endpoints', data)
        self.assertIn(
            'GET /api/v1/faults/{fault_code}/snapshots',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults/{fault_code}/snapshots',
            data['endpoints']
        )

    def test_get_snapshots_nonexistent_fault(self):
        """GET /faults/{fault_code}/snapshots returns 404 (endpoint removed).

        Legacy snapshot endpoints have been removed in favor of:
        - Inline snapshots in fault response (environment_data.snapshots)
        - Bulk-data endpoint for rosbag downloads
        """
        response = requests.get(
            f'{self.BASE_URL}/faults/NONEXISTENT_FAULT_CODE/snapshots',
            timeout=10
        )
        # Endpoint was removed, should return 404
        self.assertEqual(response.status_code, 404)

    def test_get_component_snapshots_nonexistent_fault(self):
        """GET /apps/{id}/faults/{code}/snapshots returns 404 (endpoint removed).

        Legacy snapshot endpoints have been removed.
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/faults/NONEXISTENT_FAULT/snapshots',
            timeout=10
        )
        # Endpoint was removed, should return 404
        self.assertEqual(response.status_code, 404)

    def test_get_snapshots_nonexistent_component(self):
        """GET /components/{id}/faults/{code}/snapshots returns 404 (endpoint removed).

        Legacy snapshot endpoints have been removed.
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/faults/ANY_FAULT/snapshots',
            timeout=10
        )
        # Endpoint was removed, should return 404
        self.assertEqual(response.status_code, 404)

    def test_get_snapshots_invalid_component_id(self):
        """GET /components/{id}/faults/{code}/snapshots returns 404 (endpoint removed).

        Legacy snapshot endpoints have been removed.
        """
        # Note: endpoint removed, so any ID should return 404
        invalid_ids = [
            'component;drop',
            'component<script>',
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/components/{invalid_id}/faults/ANY_FAULT/snapshots',
                timeout=10
            )
            # Endpoint was removed, should return 404
            self.assertEqual(
                response.status_code,
                404,
                f'Expected 404 for entity_id: {invalid_id} (endpoint removed)'
            )

    def test_get_rosbag_nonexistent_fault(self):
        """Legacy /faults/{code}/snapshots/bag returns 404 (endpoint removed).

        Legacy rosbag download endpoint removed. Use bulk-data endpoint instead.
        """
        response = requests.get(
            f'{self.BASE_URL}/faults/NONEXISTENT_ROSBAG_FAULT/snapshots/bag',
            timeout=10
        )
        # Endpoint was removed, should return 404
        self.assertEqual(response.status_code, 404)

    def test_get_rosbag_invalid_fault_code(self):
        """Legacy /faults/{code}/snapshots/bag returns 404 (endpoint removed).

        Legacy rosbag download endpoint removed. Use bulk-data endpoint instead.
        """
        invalid_codes = [
            '../../../etc/passwd',  # Path traversal attempt
            'fault"injection',      # Quote injection attempt
        ]

        for invalid_code in invalid_codes:
            response = requests.get(
                f'{self.BASE_URL}/faults/{invalid_code}/snapshots/bag',
                timeout=10
            )
            # Endpoint was removed, should return 404
            self.assertEqual(
                response.status_code,
                404,
                f'Expected 404 for fault_code: {invalid_code} (endpoint removed)'
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
