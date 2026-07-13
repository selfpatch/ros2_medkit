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

"""End-to-end regression for #516: external-Component faults across every rollup.

An external Component (``external: true`` - e.g. a PLC bridged into SOVD by a
protocol plugin) can report faults to the fault_manager under its own entity id
with NO child App located on it. Fault-scope resolution must fall back to that
bare id when no app contributes a scope, so the component's faults surface on
the component route AND on every aggregate route that rolls it up (function,
area). This mirrors the #517 external-App rule
(``ResolveEntitySourceFqnsTest.ExternalAppWithStrayRosBindingOwnsFaultsByBareId``)
one level up the entity hierarchy
(``ResolveEntitySourceFqnsTest.ExternalComponentWithNoAppsOwnsFaultsUnderItsOwnId``).

The fixture also carries a separate external App (``plc-diag``) that is
deliberately NOT located on the PLC Component: locating it there would give the
Component a contributing child app, and the fault scope would resolve to the
app's id instead of falling back to the component's own bare id
(``ResolveEntitySourceFqnsTest.ExternalComponentWithContributingAppKeepsAppScope``),
silently breaking the rollup this test exists to pin. The App instead pins
``external`` wire parity on the App route.

The fixture's second Component (``nav-controller``) is internal (no
``external`` key) and has no apps located on it either - the guardrail case
(``ResolveEntitySourceFqnsTest.NonExternalComponentWithNoAppsStaysEmpty``): an
unbound ROS component must not claim faults it never reported.

The merge-level preservation is unit-covered
(``MergePipelineTest.PluginExternalClassificationSurvivesManifestComponentMetadataMerge``)
and the per-route scope resolution is unit-covered
(``ResolveEntitySourceFqnsTest.*``). This test pins the full HTTP-stack
behaviour those two disjoint unit layers never exercise together: report a
fault under the Component's bare id, then observe it on every rollup route.
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


EXTERNAL_COMPONENT = 's7-plc'
EXTERNAL_APP = 'plc-diag'
INTERNAL_COMPONENT = 'nav-controller'
HOST_FUNCTION = 'material_flow'
HOST_AREA = 'plc-cell'
PLC_FAULT = 'PLC_JAM_INFEED'
NAV_FAULT = 'NAV_STALL'


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'external_component_fault_manifest.yaml',
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


class TestExternalComponentFaultRollup(GatewayTestCase):
    """An external Component owns its faults with no child app (#516)."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {EXTERNAL_APP}
    REQUIRED_AREAS = {HOST_AREA}
    REQUIRED_FUNCTIONS = {HOST_FUNCTION}

    @classmethod
    def setUpClass(cls):
        # Wait for the gateway + discovery (the external component, app, area
        # and function must exist before we exercise the rollup routes).
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('external_component_fault_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report_fault(self, source_id, fault_code, times=4):
        """Fire-and-forget fault reports.

        Mirror the #517 helper: drop the reply future to avoid
        sanitizer-timing flakes; report several times so the negative
        confirmation_threshold latches CONFIRMED.
        """
        for _ in range(times):
            req = ReportFault.Request()
            req.fault_code = fault_code
            req.event_type = ReportFault.Request.EVENT_FAILED
            req.severity = Fault.SEVERITY_ERROR
            req.source_id = source_id
            # Fire-and-forget: the request is sent synchronously by
            # call_async; spin briefly to flush and pace the reports for the
            # debounce, and intentionally drop the reply future.
            self._report_client.call_async(req)
            rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def test_external_component_owns_faults_on_every_rollup(self):
        """External Component fault surfaces on component, function, area.

        @verifies REQ_INTEROP_012
        """
        self._report_fault(EXTERNAL_COMPONENT, PLC_FAULT)

        # #516: the component's bare-id fault must roll up to every aggregate
        # route that includes it. Before the fix these rollups were empty
        # because a non-external component (or one with no contributing app)
        # never resolved to its own bare id.
        for endpoint in (
            f'/components/{EXTERNAL_COMPONENT}',
            f'/functions/{HOST_FUNCTION}',
            f'/areas/{HOST_AREA}',
        ):
            with self.subTest(rollup=endpoint):
                fault = self.wait_for_fault(endpoint, PLC_FAULT)
                self.assertEqual(fault.get('fault_code'), PLC_FAULT)

    def test_internal_component_does_not_own_bare_id_faults(self):
        """Guardrail: a non-external Component must not claim bare-id faults.

        Race-free: confirm the fault is recorded on the server-wide /faults
        list first, then assert it is filtered out of the component route.
        """
        self._report_fault(INTERNAL_COMPONENT, NAV_FAULT)
        # wait_for_fault('', NAV_FAULT) polls the server-wide '/faults' list
        # (it appends '/faults' to the empty entity endpoint), proving the
        # fault_manager recorded it regardless of scope resolution.
        self.wait_for_fault('', NAV_FAULT)
        comp_faults = self.get_json(f'/components/{INTERNAL_COMPONENT}/faults')
        codes = [f.get('fault_code') for f in comp_faults.get('items', [])]
        self.assertNotIn(NAV_FAULT, codes)

    def test_external_flag_and_identity_on_the_wire(self):
        """External flag present on external Component + App, absent internal.

        AAS identity is present on both (orthogonal to external).
        """
        comp = self.get_json(f'/components/{EXTERNAL_COMPONENT}')
        self.assertTrue(comp['x-medkit'].get('external'))
        self.assertIn('identity', comp['x-medkit'])

        app = self.get_json(f'/apps/{EXTERNAL_APP}')
        self.assertTrue(app['x-medkit'].get('external'))

        internal = self.get_json(f'/components/{INTERNAL_COMPONENT}')
        self.assertNotIn('external', internal['x-medkit'])
        self.assertIn('identity', internal['x-medkit'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
