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
Integration tests for hybrid discovery duplicate suppression (#307).

Launches the gateway in hybrid mode with the demo_nodes_manifest.yaml and
all demo nodes that match manifest ros_binding entries. Asserts with exact
counts that entity totals match the manifest - no synthetic "root" areas,
no underscored duplicates of components or apps that were already linked.

Manifest (demo_nodes_manifest.yaml) defines:
  - 4 top-level areas (powertrain, chassis, body, perception)
  - 6 subareas  (engine, brakes, door, front-left-door, lights, lidar)
    (only accessible via /areas/{id}/subareas, not in GET /areas)
  - 9 components (engine-ecu, temp-sensor-hw, rpm-sensor-hw, brake-ecu,
                   brake-pressure-sensor-hw, brake-actuator-hw,
                   door-sensor-hw, light-module, lidar-unit)
  - 9 apps     (engine-temp-sensor, engine-rpm-sensor,
                 engine-calibration-service, engine-long-calibration,
                 brake-pressure-sensor, brake-actuator, door-status-sensor,
                 light-controller, lidar-sensor)
  - 5 functions (engine-monitoring, engine-calibration, brake-system,
                  body-electronics, perception-system)
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
from ros2_medkit_test_utils.launch_helpers import (
    ALL_DEMO_NODES,
    create_demo_nodes,
    create_gateway_node,
)

# Expected manifest entity counts and IDs
# All areas defined in the manifest (top-level + subareas)
MANIFEST_ALL_AREAS = {
    'powertrain', 'engine', 'chassis', 'brakes', 'body', 'door',
    'front-left-door', 'lights', 'perception', 'lidar',
}
# Only top-level areas appear in GET /areas; subareas are filtered
MANIFEST_TOP_LEVEL_AREAS = {
    'powertrain', 'chassis', 'body', 'perception',
}
MANIFEST_SUBAREAS = MANIFEST_ALL_AREAS - MANIFEST_TOP_LEVEL_AREAS
MANIFEST_COMPONENTS = {
    'engine-ecu', 'temp-sensor-hw', 'rpm-sensor-hw', 'brake-ecu',
    'brake-pressure-sensor-hw', 'brake-actuator-hw', 'door-sensor-hw',
    'light-module', 'lidar-unit',
}
MANIFEST_APPS = {
    'engine-temp-sensor', 'engine-rpm-sensor', 'engine-calibration-service',
    'engine-long-calibration', 'brake-pressure-sensor', 'brake-actuator',
    'door-status-sensor', 'light-controller', 'lidar-sensor',
}
MANIFEST_FUNCTIONS = {
    'engine-monitoring', 'engine-calibration', 'brake-system',
    'body-electronics', 'perception-system',
}


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    # Hybrid mode with gap-fill restrictions: only manifest entities allowed.
    # All demo nodes match manifest ros_bindings, so the runtime layer
    # should link them to manifest apps (not create duplicates).
    # Gap-fill blocks heuristic apps for non-manifest nodes (e.g. the
    # gateway node itself, param client) so we can assert exact counts.
    # Note: Areas and Components are never created by runtime discovery,
    # so no gap-fill control is needed for them.
    gateway_node = create_gateway_node(extra_params={
        'discovery.mode': 'hybrid',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
        'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': False,
    })

    # Launch ALL demo nodes so every manifest app gets a runtime match.
    demo_nodes = create_demo_nodes(ALL_DEMO_NODES)
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
class TestHybridSuppression(GatewayTestCase):
    """Verify hybrid mode suppresses duplicate entities after linking."""

    # Wait for all manifest apps to be discovered before running tests.
    # REQUIRED_AREAS only checks top-level areas (subareas are filtered).
    MIN_EXPECTED_APPS = len(MANIFEST_APPS)
    REQUIRED_AREAS = MANIFEST_TOP_LEVEL_AREAS
    REQUIRED_APPS = MANIFEST_APPS

    def test_exact_area_count(self):
        """Top-level area count must match manifest - no synthetic extras.

        Subareas are filtered from GET /areas and only accessible via
        GET /areas/{id}/subareas.
        """
        # @verifies REQ_INTEROP_003
        data = self.get_json('/areas')
        area_ids = {a['id'] for a in data['items']}
        self.assertEqual(
            area_ids, MANIFEST_TOP_LEVEL_AREAS,
            f'Area mismatch. Extra: {area_ids - MANIFEST_TOP_LEVEL_AREAS}, '
            f'Missing: {MANIFEST_TOP_LEVEL_AREAS - area_ids}',
        )
        self.assertEqual(
            len(data['items']), len(MANIFEST_TOP_LEVEL_AREAS),
            f'Expected {len(MANIFEST_TOP_LEVEL_AREAS)} top-level areas, '
            f'got {len(data["items"])}: '
            f'{[a["id"] for a in data["items"]]}',
        )

    def test_exact_component_count(self):
        """Component count must match manifest exactly - no underscored duplicates."""
        # @verifies REQ_INTEROP_003
        data = self.poll_endpoint_until(
            '/components',
            lambda d: d if len(d.get('items', [])) == len(MANIFEST_COMPONENTS) else None,
            timeout=30.0,
        )
        component_ids = {c['id'] for c in data['items']}
        self.assertEqual(
            component_ids, MANIFEST_COMPONENTS,
            f'Component mismatch. Extra: {component_ids - MANIFEST_COMPONENTS}, '
            f'Missing: {MANIFEST_COMPONENTS - component_ids}',
        )
        self.assertEqual(
            len(data['items']), len(MANIFEST_COMPONENTS),
            f'Expected {len(MANIFEST_COMPONENTS)} components, '
            f'got {len(data["items"])}: '
            f'{[c["id"] for c in data["items"]]}',
        )

    def test_exact_app_count(self):
        """App count must match manifest exactly - no underscored duplicates."""
        # @verifies REQ_INTEROP_003
        data = self.get_json('/apps')
        app_ids = {a['id'] for a in data['items']}
        self.assertEqual(
            app_ids, MANIFEST_APPS,
            f'App mismatch. Extra: {app_ids - MANIFEST_APPS}, '
            f'Missing: {MANIFEST_APPS - app_ids}',
        )
        self.assertEqual(
            len(data['items']), len(MANIFEST_APPS),
            f'Expected {len(MANIFEST_APPS)} apps, got {len(data["items"])}: '
            f'{[a["id"] for a in data["items"]]}',
        )

    def test_exact_function_count(self):
        """Function count must match manifest exactly."""
        # @verifies REQ_INTEROP_003
        data = self.poll_endpoint_until(
            '/functions',
            lambda d: d if len(d.get('items', [])) == len(MANIFEST_FUNCTIONS) else None,
            timeout=30.0,
        )
        function_ids = {f['id'] for f in data['items']}
        self.assertEqual(
            function_ids, MANIFEST_FUNCTIONS,
            f'Function mismatch. Extra: {function_ids - MANIFEST_FUNCTIONS}, '
            f'Missing: {MANIFEST_FUNCTIONS - function_ids}',
        )
        self.assertEqual(
            len(data['items']), len(MANIFEST_FUNCTIONS),
            f'Expected {len(MANIFEST_FUNCTIONS)} functions, '
            f'got {len(data["items"])}: '
            f'{[f["id"] for f in data["items"]]}',
        )

    def test_no_underscored_app_duplicates(self):
        """No apps should have underscore-style IDs from runtime discovery.

        In hybrid mode, runtime nodes matching manifest ros_bindings
        should be linked to the manifest app, not create separate
        entities like 'powertrain_engine_temp_sensor'.
        """
        # @verifies REQ_INTEROP_003
        data = self.get_json('/apps')
        app_ids = [a['id'] for a in data['items']]
        underscored = [
            aid for aid in app_ids
            if '_' in aid and aid not in MANIFEST_APPS
        ]
        self.assertEqual(
            underscored, [],
            f'Found underscored runtime duplicate apps: {underscored}',
        )

    def test_no_underscored_component_duplicates(self):
        """No components should have underscore-style IDs from runtime heuristics.

        Synthetic components like 'powertrain_engine' should be suppressed
        when manifest already defines components for those namespaces.
        """
        # @verifies REQ_INTEROP_003
        data = self.get_json('/components')
        component_ids = [c['id'] for c in data['items']]
        underscored = [
            cid for cid in component_ids
            if '_' in cid and cid not in MANIFEST_COMPONENTS
        ]
        self.assertEqual(
            underscored, [],
            f'Found underscored runtime duplicate components: {underscored}',
        )

    def test_no_root_or_synthetic_areas(self):
        """No 'root' or underscored synthetic areas should exist in top-level."""
        # @verifies REQ_INTEROP_003
        data = self.get_json('/areas')
        area_ids = [a['id'] for a in data['items']]
        synthetic = [
            aid for aid in area_ids
            if aid == 'root' or (aid not in MANIFEST_TOP_LEVEL_AREAS)
        ]
        self.assertEqual(
            synthetic, [],
            f'Found synthetic/unexpected areas: {synthetic}',
        )

    def test_manifest_apps_are_online(self):
        """Linked manifest apps should be online after runtime linking.

        Even with allow_heuristic_apps=false (gap-fill blocking runtime
        apps from the entity tree), the linker must still receive the
        unfiltered runtime apps so it can bind manifest apps to live nodes.
        """
        # @verifies REQ_INTEROP_003
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if all(
                a.get('x-medkit', {}).get('is_online', False)
                for a in d.get('items', [])
            ) and len(d.get('items', [])) == len(MANIFEST_APPS) else None,
            timeout=30.0,
        )
        for app in data['items']:
            x_medkit = app.get('x-medkit', {})
            is_online = x_medkit.get('is_online', False)
            self.assertTrue(
                is_online,
                f"App {app['id']} should be online but is not",
            )

    def test_health_shows_hybrid_mode(self):
        """Health endpoint should confirm hybrid discovery mode."""
        # @verifies REQ_INTEROP_003
        health = self.get_json('/health')
        discovery = health.get('discovery', {})
        self.assertEqual(discovery.get('mode'), 'hybrid')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
