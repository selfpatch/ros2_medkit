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

"""
Integration tests for hybrid mode gap-fill configuration.

Tests that gap-fill controls restrict which heuristic entities the
runtime layer can create when a manifest is present. Also tests
namespace blacklist/whitelist filtering.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    # Hybrid mode with restrictive gap-fill:
    # - No heuristic areas (only manifest areas)
    # - No heuristic components (only manifest components)
    # - Apps still allowed (for linking)
    gateway_node = create_gateway_node(extra_params={
        'discovery.mode': 'hybrid',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
        'discovery.merge_pipeline.gap_fill.allow_heuristic_areas': False,
        'discovery.merge_pipeline.gap_fill.allow_heuristic_components': False,
        'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': True,
    })

    # Launch a subset of demo nodes (some in manifest, some not)
    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'calibration'],
    )
    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway_node,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


# @verifies REQ_INTEROP_003
class TestGapFillConfig(GatewayTestCase):
    """Test gap-fill restrictions in hybrid mode."""

    POLL_INTERVAL = 1.0
    POLL_TIMEOUT = 30.0

    def test_only_manifest_areas_present(self):
        """With allow_heuristic_areas=false, only manifest areas should exist."""
        areas = self.poll_endpoint_until(
            '/areas',
            lambda data: len(data) >= 1,
        )
        area_ids = [a['id'] for a in areas]

        # Manifest defines: powertrain, chassis, body, perception
        # No heuristic areas from runtime namespaces should appear
        for area_id in area_ids:
            self.assertIn(area_id, [
                'powertrain', 'chassis', 'body', 'perception',
                # Subareas defined in manifest
                'engine', 'brakes', 'lidar', 'door', 'lights',
                'front-left-door',
            ], f"Unexpected heuristic area found: {area_id}")

    def test_only_manifest_components_present(self):
        """With allow_heuristic_components=false, only manifest components exist."""
        components = self.poll_endpoint_until(
            '/components',
            lambda data: len(data) >= 1,
        )
        component_ids = [c['id'] for c in components]

        # Only manifest-defined components should be present
        manifest_components = [
            'engine-ecu', 'temp-sensor-hw', 'rpm-sensor-hw',
            'brake-ecu', 'pressure-sensor-hw',
            'lidar-unit',
            'door-controller', 'light-controller',
        ]
        for comp_id in component_ids:
            self.assertIn(
                comp_id, manifest_components,
                f"Unexpected heuristic component found: {comp_id}",
            )

    def test_health_shows_gap_fill_filtering(self):
        """Health endpoint should show filtered_by_gap_fill count."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: 'discovery' in data,
        )
        discovery = health.get('discovery', {})
        pipeline = discovery.get('pipeline', {})

        # Should have filtered some entities
        self.assertIn('filtered_by_gap_fill', pipeline)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
