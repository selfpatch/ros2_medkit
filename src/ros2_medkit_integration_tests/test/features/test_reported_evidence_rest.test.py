#!/usr/bin/env python3
# Copyright 2026 selfpatch
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

"""Reporter evidence reaches the REST fault detail.

A FAILED report can carry key-value evidence. The fault manager serves it in
the freeze frame under ``"x-medkit": {"reported": ...}``, and the gateway
passes it through in ``GET /apps/{app}/faults/{code}``. This checks the shape
the docs show, end to end.
"""

import re
import time
import unittest

from diagnostic_msgs.msg import KeyValue
import launch_testing
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


APP = 'temp_sensor'
SOURCE_ID = '/powertrain/engine/temp_sensor'
FAULT_CODE = 'EVIDENCE_REST_FAULT'


def generate_test_description():
    return create_test_launch(
        demo_nodes=[APP],
        fault_manager=True,
        lidar_faulty=False,
    )


class TestReportedEvidenceRest(GatewayTestCase):
    """Evidence sent with a report is served in the REST fault detail."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {APP}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._reporter = Node('reported_evidence_rest_reporter')
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

    def _report(self):
        """Report the fault with evidence, fire-and-forget like the fault_reporter."""
        req = ReportFault.Request()
        req.fault_code = FAULT_CODE
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'fusion diverged'
        req.source_id = SOURCE_ID
        req.evidence = [
            KeyValue(key='rejected_fixes', value='37'),
            KeyValue(key='nis', value='0.03'),
        ]
        self._report_client.call_async(req)
        rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def _reported(self, detail):
        frames = [
            s for s in detail.get('environment_data', {}).get('snapshots', [])
            if s.get('type') == 'freeze_frame' and s.get('name') == 'freeze_frame'
        ]
        if not frames:
            return None
        data = frames[0].get('data')
        if not isinstance(data, dict):
            return None
        return data.get('x-medkit', {}).get('reported')

    def test_evidence_is_served_in_the_fault_detail(self):
        """GET /apps/{app}/faults/{code} carries the evidence by source and key."""
        # The report is fire-and-forget, so send it again until the detail shows it.
        url = f'{self.BASE_URL}/apps/{APP}/faults/{FAULT_CODE}'
        reported = None
        last = None
        deadline = time.monotonic() + 30.0
        while reported is None and time.monotonic() < deadline:
            self._report()
            try:
                response = requests.get(url, timeout=2)
                last = response.status_code
                if response.status_code == 200:
                    reported = self._reported(response.json())
            except requests.exceptions.RequestException as e:
                last = str(e)
            if reported is None:
                time.sleep(0.5)
        self.assertIsNotNone(reported, f'no evidence in {url} after 30s (last: {last})')

        self.assertEqual(set(reported), {SOURCE_ID})
        by_key = reported[SOURCE_ID]
        self.assertEqual(by_key['rejected_fixes']['value'], '37')
        self.assertEqual(by_key['nis']['value'], '0.03')
        self.assertRegex(
            by_key['nis']['reported_at'],
            re.compile(r'^\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}\.\d{3}Z$'))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
