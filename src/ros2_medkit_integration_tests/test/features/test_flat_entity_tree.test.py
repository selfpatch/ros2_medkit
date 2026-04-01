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

"""Integration tests for flat entity tree (no areas).

Validates that the gateway works correctly in manifest_only mode with a
manifest that defines no areas. Only top-level components appear in
GET /components; subcomponents are accessible via the /subcomponents endpoint.

Uses flat_robot_manifest.yaml which defines:
  - 0 areas
  - 1 top-level component (turtlebot3) + 3 subcomponents
  - 4 apps (lidar-driver, turtlebot3-node, nav2-controller, robot-state-publisher)
  - 2 functions (autonomous-navigation, teleoperation)
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import launch_testing.asserts

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'flat_robot_manifest.yaml'
    )

    gateway_node = create_gateway_node(extra_params={
        'discovery.mode': 'manifest_only',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
    })

    return (
        LaunchDescription([
            gateway_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


# @verifies REQ_INTEROP_003
class TestFlatEntityTree(GatewayTestCase):
    """Test entity listing with a flat manifest (no areas)."""

    def test_areas_empty(self):
        """GET /areas returns empty list when no areas are defined.

        @verifies REQ_INTEROP_003
        """
        data = self.poll_endpoint_until(
            '/areas',
            lambda d: d if 'items' in d else None,
            timeout=30.0,
        )
        self.assertEqual(len(data['items']), 0)

    def test_components_count(self):
        """GET /components returns only top-level components.

        Expected: turtlebot3 (root) only. Subcomponents (raspberry-pi,
        opencr-board, lds-sensor) are filtered from the top-level listing
        and accessible via GET /components/turtlebot3/subcomponents.

        @verifies REQ_INTEROP_003
        """
        data = self.poll_endpoint_until(
            '/components',
            lambda d: d if len(d.get('items', [])) >= 1 else None,
            timeout=30.0,
        )
        components = data['items']
        self.assertEqual(len(components), 1)

        component_ids = sorted([c['id'] for c in components])
        self.assertEqual(component_ids, ['turtlebot3'])

    def test_subcomponents_count(self):
        """GET /components/turtlebot3/subcomponents returns exactly 3.

        Expected: raspberry-pi, opencr-board, lds-sensor.

        @verifies REQ_INTEROP_003
        """
        data = self.poll_endpoint_until(
            '/components/turtlebot3/subcomponents',
            lambda d: d if len(d.get('items', [])) >= 3 else None,
            timeout=30.0,
        )
        subcomponents = data['items']
        self.assertEqual(len(subcomponents), 3)

        sub_ids = sorted([s['id'] for s in subcomponents])
        self.assertEqual(
            sub_ids,
            sorted(['raspberry-pi', 'opencr-board', 'lds-sensor']),
        )

    def test_apps_count(self):
        """GET /apps returns exactly 4 apps.

        Expected: lidar-driver, turtlebot3-node, nav2-controller,
        robot-state-publisher.

        @verifies REQ_INTEROP_003
        """
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if len(d.get('items', [])) >= 4 else None,
            timeout=30.0,
        )
        apps = data['items']
        self.assertEqual(len(apps), 4)

        app_ids = sorted([a['id'] for a in apps])
        self.assertEqual(
            app_ids,
            sorted([
                'lidar-driver', 'turtlebot3-node',
                'nav2-controller', 'robot-state-publisher',
            ]),
        )

    def test_functions_count(self):
        """GET /functions returns exactly 2 functions.

        Expected: autonomous-navigation, teleoperation.

        @verifies REQ_INTEROP_003
        """
        data = self.poll_endpoint_until(
            '/functions',
            lambda d: d if len(d.get('items', [])) >= 2 else None,
            timeout=30.0,
        )
        functions = data['items']
        self.assertEqual(len(functions), 2)

        func_ids = sorted([f['id'] for f in functions])
        self.assertEqual(
            func_ids,
            sorted(['autonomous-navigation', 'teleoperation']),
        )

    def test_app_accessible_without_areas(self):
        """GET /apps/lidar-driver returns 200 even without areas.

        Verifies that apps are accessible in a flat entity tree where no
        areas are defined.

        @verifies REQ_INTEROP_003
        """
        data = self.poll_endpoint_until(
            '/apps/lidar-driver',
            lambda d: d if d.get('id') == 'lidar-driver' else None,
            timeout=30.0,
        )
        self.assertEqual(data['id'], 'lidar-driver')
        self.assertIn('name', data)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
