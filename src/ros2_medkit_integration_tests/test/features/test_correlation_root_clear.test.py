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

"""Integration test: clearing a root cause through its app route unmutes its symptoms.

The gateway's per-entity DELETE sends skip_correlation_auto_clear, so it clears the root cause
alone. The symptom the root cause muted must then show on its own app again, and stay a
CONFIRMED fault.
"""

import atexit
import os
import tempfile
import time
import unittest

import launch_testing
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults, ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

ROOT_CODE = 'LIDAR_RANGE_INVALID'
SYMPTOM_CODE = 'ROOT_CLEAR_SYMPTOM'
SYMPTOM_SOURCE = '/powertrain/engine/temp_sensor'

CORRELATION = """\
correlation:
  enabled: true
  rules:
    - id: lidar_cascade
      name: "Lidar cascade"
      mode: hierarchical
      root_cause:
        codes: ["LIDAR_RANGE_INVALID"]
      symptoms:
        - codes: ["ROOT_CLEAR_SYMPTOM"]
      window_ms: 60000
      mute_symptoms: true
      auto_clear_with_root: true
"""


def generate_test_description():
    """Launch gateway, fault manager with a cascade rule, a faulty lidar and a temp sensor."""
    rules = tempfile.NamedTemporaryFile(
        'w', suffix='.yaml', prefix='root_clear_rules_', delete=False)
    rules.write(CORRELATION)
    rules.close()
    atexit.register(os.unlink, rules.name)
    return create_test_launch(
        demo_nodes=['lidar_sensor', 'temp_sensor'],
        gateway_params={},
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,
            'correlation.config_file': rules.name,
        },
        lidar_faulty=True,
    )


class TestCorrelationRootClear(GatewayTestCase):
    """DELETE of a root cause on its app route ends its relations."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'lidar_sensor', 'temp_sensor'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._node = Node('root_clear_reporter')
        cls._report = cls._node.create_client(ReportFault, '/fault_manager/report_fault')
        cls._list = cls._node.create_client(ListFaults, '/fault_manager/list_faults')
        assert cls._report.wait_for_service(timeout_sec=30.0)
        assert cls._list.wait_for_service(timeout_sec=30.0)

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), 'service call timed out')
        return future.result()

    def _muted_codes(self):
        request = ListFaults.Request()
        request.include_muted = True
        return {m.fault_code for m in self._call(self._list, request).muted_faults}

    def test_deleting_the_root_cause_unmutes_its_symptom(self):
        self.wait_for_fault('/apps/lidar_sensor', ROOT_CODE)

        request = ReportFault.Request()
        request.fault_code = SYMPTOM_CODE
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'symptom of the lidar'
        request.source_id = SYMPTOM_SOURCE
        self.assertTrue(self._call(self._report, request).accepted)

        deadline = time.monotonic() + 30.0
        while SYMPTOM_CODE not in self._muted_codes():
            self.assertLess(time.monotonic(), deadline, 'the symptom was never muted')
            rclpy.spin_once(self._node, timeout_sec=0.2)
        resp = requests.get(f'{self.BASE_URL}/apps/temp_sensor/faults', timeout=10)
        self.assertNotIn(SYMPTOM_CODE, [f['fault_code'] for f in resp.json().get('items', [])])

        resp = requests.delete(
            f'{self.BASE_URL}/apps/lidar_sensor/faults/{ROOT_CODE}', timeout=10)
        self.assertEqual(resp.status_code, 204, resp.text)

        fault = self.wait_for_fault('/apps/temp_sensor', SYMPTOM_CODE, max_wait=30.0)
        self.assertEqual(fault['status'], 'CONFIRMED')
        self.assertNotIn(SYMPTOM_CODE, self._muted_codes())


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
