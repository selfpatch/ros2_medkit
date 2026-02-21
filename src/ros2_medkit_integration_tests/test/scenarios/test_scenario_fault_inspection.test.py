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

"""Scenario: Fault inspection — SOVD-compliant response structure validation.

Deep-inspects every part of the fault response returned by the gateway:
item, status, environment_data, freeze_frame snapshot, rosbag snapshot,
and x-medkit extensions.
"""

import unittest

import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        # Use threshold -2 so faults need 2 FAILED events before confirmation.
        # This gives the rosbag ring buffer time to subscribe to topics and
        # collect data before the first fault triggers rosbag capture.
        fault_manager_params={'confirmation_threshold': -2},
    )


class TestScenarioFaultInspection(GatewayTestCase):
    """Scenario: Inspect every field of a SOVD-compliant fault response.

    Each test independently waits for a lidar fault and then validates
    a specific sub-structure of the response. No test depends on the
    state left by a previous test.

    Steps:
    1. Validate full SOVD response structure (item + environment_data)
    2. Validate status sub-object (testFailed, presentStatus, confirmedStatus)
    3. Validate environment_data (extended_data_records, snapshots)
    4. Validate freeze_frame snapshot structure
    5. Validate rosbag snapshot has bulk_data_uri
    6. Validate x-medkit extensions (occurrence_count, severity_label, etc.)
    """

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    LIDAR_ENDPOINT = '/apps/lidar_sensor'

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _get_fault_detail(self, snapshot_types=None):
        """Wait for a lidar fault, then GET the full detail response.

        Parameters
        ----------
        snapshot_types : set or list of str, optional
            Snapshot types to wait for (e.g. ``{'freeze_frame', 'rosbag'}``).
            Delegates to :meth:`GatewayTestCase.wait_for_fault_detail`.

        Returns the parsed JSON body.
        """
        return self.wait_for_fault_detail(
            self.LIDAR_ENDPOINT,
            snapshot_types=snapshot_types,
        )

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_fault_response_structure(self):
        """Validate full SOVD response structure (item + environment_data).

        @verifies REQ_INTEROP_013
        """
        data = self._get_fault_detail()

        self.assertIn('item', data)
        self.assertIn('environment_data', data)

        item = data['item']
        self.assertIn('code', item)
        self.assertIn('status', item)

    def test_02_fault_status_object_structure(self):
        """Validate status sub-object fields.

        The gateway returns AUTOSAR DEM-style status fields:
        - aggregatedStatus: "active", "passive", or "cleared"
        - testFailed, confirmedDTC, pendingDTC: "0" or "1"

        @verifies REQ_INTEROP_013
        """
        data = self._get_fault_detail()
        status = data['item']['status']

        self.assertIn('aggregatedStatus', status)
        self.assertIn(status['aggregatedStatus'], ('active', 'passive', 'cleared'))
        self.assertIn('testFailed', status)
        self.assertIn('confirmedDTC', status)
        self.assertIn('pendingDTC', status)
        for field in ('testFailed', 'confirmedDTC', 'pendingDTC'):
            self.assertIn(status[field], ('0', '1'), f'{field} must be "0" or "1"')

    def test_03_fault_environment_data_structure(self):
        """Validate environment_data has extended_data_records and snapshots.

        @verifies REQ_INTEROP_013
        """
        data = self._get_fault_detail()
        env_data = data['environment_data']

        self.assertIn('extended_data_records', env_data)
        self.assertIn('snapshots', env_data)
        self.assertIsInstance(env_data['extended_data_records'], dict)
        self.assertIn('first_occurrence', env_data['extended_data_records'])
        self.assertIn('last_occurrence', env_data['extended_data_records'])
        self.assertIsInstance(env_data['snapshots'], list)

    def test_04_fault_snapshot_freeze_frame(self):
        """Validate freeze_frame snapshot structure.

        Snapshots are captured asynchronously by the fault manager after
        a fault is confirmed, so we poll until the freeze_frame is present.

        @verifies REQ_INTEROP_013
        """
        data = self._get_fault_detail(snapshot_types={'freeze_frame'})
        snapshots = data['environment_data']['snapshots']

        freeze_frame = next(
            s for s in snapshots if s.get('type') == 'freeze_frame'
        )
        self.assertIn('name', freeze_frame)
        self.assertEqual(freeze_frame['type'], 'freeze_frame')
        # freeze_frame has inline data
        self.assertIn('data', freeze_frame)

    def test_05_fault_snapshot_rosbag_has_bulk_data_uri(self):
        """Validate rosbag snapshot has correct bulk_data_uri format.

        Rosbag snapshots are flushed asynchronously after fault confirmation,
        so we poll until the rosbag snapshot is present.

        @verifies REQ_INTEROP_013
        """
        data = self._get_fault_detail(snapshot_types={'rosbag'})
        snapshots = data['environment_data']['snapshots']

        rosbag = next(s for s in snapshots if s.get('type') == 'rosbag')
        self.assertIn('name', rosbag)
        self.assertEqual(rosbag['type'], 'rosbag')
        self.assertIn('bulk_data_uri', rosbag)
        self.assertIn('/bulk-data/rosbags/', rosbag['bulk_data_uri'])

    def test_06_fault_x_medkit_extensions(self):
        """Validate x-medkit extensions on the fault response.

        @verifies REQ_INTEROP_013
        """
        data = self._get_fault_detail()

        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('occurrence_count', x_medkit)
        self.assertIn('severity_label', x_medkit)
        self.assertIn('reporting_sources', x_medkit)


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
