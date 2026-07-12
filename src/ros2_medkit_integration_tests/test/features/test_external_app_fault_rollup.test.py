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

"""End-to-end regression for #517: external-app faults across every rollup.

An external app (``external: true`` - e.g. a PLC bridged into SOVD by a protocol
plugin) reports faults to the fault_manager under its own entity id, not a ROS
FQN. Fault-scope resolution must recognise that bare id so the app's faults
surface on the app route AND on every aggregate route that rolls it up
(component, function, area). #517 was the fault scope resolving to a live FQN
instead of the bare id, which silently emptied those rollups.

The fixture app deliberately also carries a ``ros_binding`` (the #517 "neighbor"
case): ``effective_fqn()`` derives a live FQN, so a fault scope that used it
would drop the app's bare-id faults. This test therefore fails against the
pre-fix fault scope and passes only when the external classification wins.

The merge-level preservation is unit-covered
(``MergePipelineTest.PluginExternalClassificationSurvivesManifestMetadataMerge``)
and the per-route scope resolution is unit-covered
(``ResolveEntitySourceFqnsTest.*OwnsItsFaults``). This test pins the full
HTTP-stack behaviour those two disjoint unit layers never exercise together:
report a fault under the bare id, then observe it on every rollup route.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch_testing
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


EXTERNAL_APP = 'plc-process'
HOST_COMPONENT = 's7-plc'
HOST_FUNCTION = 'level-control'
HOST_AREA = 'plc-cell'
FAULT_CODE = 'PLC_LEVEL_OVERFLOW'


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'external_app_fault_manifest.yaml',
    )
    return create_test_launch(
        demo_nodes=[],
        fault_manager=True,
        # Negative threshold: the fault confirms after a couple of FAILED
        # events (see _report_fault, which reports several times).
        fault_manager_params={'confirmation_threshold': -2},
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
        },
    )


class TestExternalAppFaultRollup(GatewayTestCase):
    """An external app owns its faults on the app route and every rollup."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {EXTERNAL_APP}
    REQUIRED_AREAS = {HOST_AREA}
    REQUIRED_FUNCTIONS = {HOST_FUNCTION}

    @classmethod
    def setUpClass(cls):
        # Wait for the gateway + discovery (the external app, area and function
        # must exist before we exercise the rollup routes).
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('external_app_fault_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report_fault(self, times=4):
        """Report the external app's fault under its bare entity id.

        Reported several times so the negative ``confirmation_threshold``
        latches the fault to CONFIRMED regardless of debounce timing.
        """
        for _ in range(times):
            req = ReportFault.Request()
            req.fault_code = FAULT_CODE
            req.event_type = ReportFault.Request.EVENT_FAILED
            req.severity = Fault.SEVERITY_ERROR
            req.description = 'PLC tank level exceeded high limit'
            req.source_id = EXTERNAL_APP
            future = self._report_client.call_async(req)
            rclpy.spin_until_future_complete(
                self._reporter, future, timeout_sec=5.0
            )
            self.assertIsNotNone(
                future.result(), 'report_fault call timed out'
            )
            self.assertTrue(
                future.result().accepted, 'fault_manager rejected the report'
            )

    def test_external_app_fault_appears_on_every_rollup(self):
        """The external app's fault surfaces on app, component, function, area.

        @verifies REQ_INTEROP_012
        """
        self._report_fault()

        # Precondition: the owning app sees the fault. If this fails the setup
        # is wrong (not the rollup), so assert it first for a clear signal.
        self.wait_for_fault(f'/apps/{EXTERNAL_APP}', FAULT_CODE)

        # #517: the same fault must roll up to every aggregate route that
        # includes the external app. Before the fix these rollups were empty
        # because the app's bare-id fault scope was dropped.
        for endpoint in (
            f'/components/{HOST_COMPONENT}',
            f'/functions/{HOST_FUNCTION}',
            f'/areas/{HOST_AREA}',
        ):
            with self.subTest(rollup=endpoint):
                fault = self.wait_for_fault(endpoint, FAULT_CODE)
                self.assertEqual(fault.get('fault_code'), FAULT_CODE)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
