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

"""A recording downloads for the owner of the record it is attached to.

Two external apps under one component report the same fault code, each under
its own entity id, so the fault manager keeps two records of that code and
captures a recording for each. A recording belongs to the (fault code, owner)
records it is attached to, so over the recording-id URL:

* each app downloads its own recording and gets ``404`` on the other's, in both
  directions, although both apps own a record of the same code
* the component hosting both apps downloads both recordings
* after one app's record is cleared, that app still gets ``404`` on the other
  app's recording, and still downloads its own

The unit layer pins the same rule against a stub fault manager
(``PluginClearOwnerTest.EachOwnerDownloadsOnlyItsOwnRecordingById``). This runs
it against the real fault manager and its real rosbag rows.
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch_testing
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


OWNER_A = 'owner-a'
OWNER_B = 'owner-b'
HOST_COMPONENT = 'shared-code-hub'
SHARED_CODE = 'SHARED_RECORDED'
RECORDING_TIMEOUT = 30.0


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'fault_owner_identity_manifest.yaml',
    )
    return create_test_launch(
        # The lidar publishes the scan topic the fault manager records, so each
        # confirmation leaves a real bag behind.
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,  # a single report confirms
            # Keep a cleared record's recording, so the app that cleared can
            # still be shown downloading its own bag after the clear.
            'snapshots.rosbag.auto_cleanup': False,
        },
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
        },
    )


class TestRosbagOwnerDownload(GatewayTestCase):
    """Each owner of one fault code downloads only its own recording by id."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {OWNER_A, OWNER_B}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._reporter = Node('rosbag_owner_download_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        super().setUpClass()
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report(self, source_id):
        """Report SHARED_RECORDED under one owner, fire and forget."""
        req = ReportFault.Request()
        req.fault_code = SHARED_CODE
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = f'reported by {source_id}'
        req.source_id = source_id
        self._report_client.call_async(req)
        rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def _recordings_listed_for(self, app_id):
        listing = self.get_json(f'/apps/{app_id}/bulk-data/rosbags')
        return sorted(
            item['id'] for item in listing.get('items', [])
            if SHARED_CODE in item.get('x-medkit', {}).get('fault_codes', [])
        )

    def _record_for(self, app_id):
        """Raise the app's own record and wait for its one finished recording."""
        deadline = time.monotonic() + RECORDING_TIMEOUT
        ids = []
        while time.monotonic() < deadline:
            # Re-sent until the recording is listed: the report is fire and
            # forget, and a repeat inside one occurrence changes nothing.
            self._report(app_id)
            ids = self._recordings_listed_for(app_id)
            if ids:
                break
            time.sleep(0.5)
        self.assertEqual(len(ids), 1, f'{app_id} should hold exactly one recording, got {ids}')
        return ids[0]

    def test_01_each_owner_downloads_only_its_own_recording(self):
        """Two owners of one code, a recording each, over the recording-id URL.

        @verifies REQ_INTEROP_072
        """
        # One after the other, so the second confirmation falls outside the
        # first recording's post-roll and gets a recording of its own instead
        # of attaching to the first one as a burst would.
        rec_a = self._record_for(OWNER_A)
        rec_b = self._record_for(OWNER_B)
        self.assertNotEqual(rec_a, rec_b, 'the two records share one recording')

        self._assert_downloads({
            (f'/apps/{OWNER_A}', rec_a): 200,
            (f'/apps/{OWNER_A}', rec_b): 404,
            (f'/apps/{OWNER_B}', rec_b): 200,
            (f'/apps/{OWNER_B}', rec_a): 404,
            (f'/components/{HOST_COMPONENT}', rec_a): 200,
            (f'/components/{HOST_COMPONENT}', rec_b): 200,
        })

        # Clearing owner-a's record keeps owner-a in its own scope with a
        # CLEARED record of the code. That must not make owner-b's recording
        # owner-a's, and owner-a's own recording stays served.
        self.delete_request(f'/apps/{OWNER_A}/faults/{SHARED_CODE}', expected_status=204)
        self._assert_downloads({
            (f'/apps/{OWNER_A}', rec_b): 404,
            (f'/apps/{OWNER_A}', rec_a): 200,
            (f'/apps/{OWNER_B}', rec_a): 404,
            (f'/apps/{OWNER_B}', rec_b): 200,
        })

    def _assert_downloads(self, expected):
        """Download each (entity path, recording id) and report every wrong status at once."""
        wrong = []
        for (entity_path, recording_id), want in expected.items():
            response = requests.get(
                f'{self.BASE_URL}{entity_path}/bulk-data/rosbags/{recording_id}',
                timeout=15,
            )
            if response.status_code != want:
                wrong.append(
                    f'{entity_path} downloading {recording_id} answered '
                    f'{response.status_code}, expected {want}'
                )
        self.assertEqual(wrong, [], '\n'.join(wrong))


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """All processes exited cleanly."""

    def test_exit_codes(self, proc_info):
        for process_name in proc_info.process_names():
            self.assertIn(
                proc_info[process_name].returncode, ALLOWED_EXIT_CODES,
                f'{process_name} exited with {proc_info[process_name].returncode}',
            )
