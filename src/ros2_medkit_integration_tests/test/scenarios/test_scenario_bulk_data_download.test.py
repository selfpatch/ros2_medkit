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

"""Scenario: Bulk data download — rosbag download, cross-entity check, content verification.

Requires the fault manager with rosbag capture enabled. The lidar_sensor
demo node produces deterministic faults that trigger rosbag snapshots.
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


class TestScenarioBulkDataDownload(GatewayTestCase):
    """Scenario: Download rosbag snapshots via the bulk-data REST API.

    Each test independently waits for a rosbag to become available,
    then validates a specific aspect of the download.

    Steps:
    1. Download rosbag successfully, verify headers and non-empty body
    2. Try downloading from wrong entity, expect 404
    3. Download rosbag and verify it is a valid file format
    """

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    LIDAR_ENDPOINT = '/apps/lidar_sensor'

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_download_rosbag_success(self):
        """Wait for fault with rosbag, download it, verify non-empty.

        @verifies REQ_INTEROP_073
        """
        rosbag_id = self.wait_for_fault_with_rosbag(
            self.LIDAR_ENDPOINT, max_wait=30.0,
        )
        if rosbag_id is None:
            self.skipTest('No rosbag available for download test')

        # Download the rosbag
        response = self.get_raw(
            f'{self.LIDAR_ENDPOINT}/bulk-data/rosbags/{rosbag_id}',
            timeout=30,
            stream=True,
        )

        # Verify Content-Type header
        content_type = response.headers.get('Content-Type', '')
        valid_types = [
            'application/x-sqlite3',
            'application/x-mcap',
            'application/octet-stream',
        ]
        self.assertTrue(
            any(t in content_type for t in valid_types),
            f'Expected valid rosbag Content-Type, got: {content_type}',
        )

        # Verify Content-Disposition header
        content_disposition = response.headers.get('Content-Disposition', '')
        self.assertIn('attachment', content_disposition)
        self.assertIn('filename=', content_disposition)

        # Verify we got actual data
        content = response.content
        self.assertGreater(len(content), 0, 'Downloaded rosbag should have content')

    def test_02_download_wrong_entity_returns_404(self):
        """Wait for rosbag, try downloading from wrong entity — expect 404.

        Security check: rosbag belonging to one entity should not be
        accessible via another entity's bulk-data endpoint.

        @verifies REQ_INTEROP_073
        """
        rosbag_id = self.wait_for_fault_with_rosbag(
            self.LIDAR_ENDPOINT, max_wait=15.0,
        )
        if rosbag_id is None:
            self.skipTest('No rosbag available for cross-entity test')

        # Try to access it via a nonexistent entity (temp_sensor not launched)
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/bulk-data/rosbags/{rosbag_id}',
            timeout=10,
        )
        self.assertEqual(response.status_code, 404)

    def test_03_verify_complete_rosbag_content(self):
        """Download rosbag and verify it is a valid file.

        @verifies REQ_INTEROP_073
        """
        rosbag_id = self.wait_for_fault_with_rosbag(
            self.LIDAR_ENDPOINT, max_wait=30.0,
        )
        if rosbag_id is None:
            self.skipTest('No rosbag available for complete download test')

        response = self.get_raw(
            f'{self.LIDAR_ENDPOINT}/bulk-data/rosbags/{rosbag_id}',
            timeout=30,
            stream=True,
        )

        content = response.content
        content_type = response.headers.get('Content-Type', '')

        if 'mcap' in content_type:
            # MCAP files should have content
            self.assertGreater(len(content), 0, 'MCAP file should have content')
        elif 'sqlite3' in content_type:
            # SQLite3 database files start with "SQLite format 3\x00"
            self.assertTrue(
                content[:16] == b'SQLite format 3\x00',
                f'Expected SQLite header, got: {content[:16]!r}',
            )
        else:
            # Other formats — just verify we have content
            self.assertGreater(len(content), 0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
