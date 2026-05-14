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

"""Regression tests for cross-entity fault scope leak (#395).

Per-entity fault routes (`GET` / `DELETE
/components/{id}/faults/{fault_code}`) must filter by the apps the entity
owns. Without this, components whose `namespace_path` is empty (synthetic,
host-derived, or manifest components without an explicit ``namespace``
field) would short-circuit the transport-level prefix filter and expose
faults reported by apps that belong to entirely different components.

These tests use the demo manifest in hybrid mode. Manifest components
in that file declare ``area`` but not ``namespace``, so every component
sees the bug pre-fix.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


LIDAR_FAULT_CODE = 'LIDAR_RANGE_INVALID'
LIDAR_OWNER_COMPONENT = 'lidar-unit'
OTHER_COMPONENT = 'temp-sensor-hw'


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )
    return create_test_launch(
        demo_nodes=['lidar_sensor', 'temp_sensor'],
        fault_manager=True,
        lidar_faulty=True,
        fault_manager_params={'confirmation_threshold': -2},
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': True,
        },
    )


class TestFaultsScopeIsolation(GatewayTestCase):
    """Per-entity fault routes must not leak faults across components."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'lidar-sensor'}

    def setUp(self):
        super().setUp()
        # The lidar app reports LIDAR_RANGE_INVALID deterministically when
        # launched with faulty params. Wait until the gateway sees it on the
        # owning app before exercising the per-component routes.
        self.wait_for_fault('/apps/lidar-sensor', LIDAR_FAULT_CODE)

    def test_get_fault_returns_404_on_unrelated_component(self):
        """GET on a different component must not return another entity's fault.

        @verifies REQ_INTEROP_013
        """
        response = requests.get(
            f'{self.BASE_URL}/components/{OTHER_COMPONENT}/faults/{LIDAR_FAULT_CODE}',
            timeout=10,
        )
        self.assertEqual(
            response.status_code, 404,
            f'Expected 404, got {response.status_code} body={response.text}',
        )

    def test_get_fault_returns_200_on_owning_component(self):
        """GET on the component that hosts the reporting app must return the fault.

        @verifies REQ_INTEROP_013
        """
        data = self.get_json(
            f'/components/{LIDAR_OWNER_COMPONENT}/faults/{LIDAR_FAULT_CODE}'
        )
        self.assertEqual(data.get('item', {}).get('code'), LIDAR_FAULT_CODE)

    def test_clear_fault_returns_404_on_unrelated_component(self):
        """DELETE on a different component must not clear another entity's fault.

        @verifies REQ_INTEROP_015
        """
        response = requests.delete(
            f'{self.BASE_URL}/components/{OTHER_COMPONENT}/faults/{LIDAR_FAULT_CODE}',
            timeout=10,
        )
        self.assertEqual(
            response.status_code, 404,
            f'Expected 404, got {response.status_code} body={response.text}',
        )

        # Verify the fault is still present on the owning app afterwards.
        listing = self.get_json('/apps/lidar-sensor/faults')
        codes = {item.get('fault_code') for item in listing.get('items', [])}
        self.assertIn(
            LIDAR_FAULT_CODE, codes,
            'Unrelated DELETE must not clear faults outside its scope',
        )

    def test_zzz_clear_fault_returns_204_on_owning_component(self):
        """DELETE on the owning component clears the fault.

        Named `_zzz_` so unittest's alphabetic ordering runs this last - it is
        destructive (clears the lidar fault from the global FaultManager). The
        lidar_sensor demo re-reports LIDAR_RANGE_INVALID continuously, so this
        does not break other test runs, but in-suite siblings rely on the fault
        being present via the setUp `wait_for_fault` poll.

        @verifies REQ_INTEROP_015
        """
        response = requests.delete(
            f'{self.BASE_URL}/components/{LIDAR_OWNER_COMPONENT}/faults/{LIDAR_FAULT_CODE}',
            timeout=10,
        )
        self.assertEqual(
            response.status_code, 204,
            f'Expected 204, got {response.status_code} body={response.text}',
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
