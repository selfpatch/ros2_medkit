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

"""Integration tests for the Phase 1 SOVD entity model in runtime_only mode.

Validates the SOVD-aligned entity model where:
- Namespaces create Function entities (not Areas)
- Areas are empty in runtime_only mode
- A single host-derived default Component is created from system info
- Apps are linked to the default Component via is-located-on

Uses standard demo nodes that span multiple namespaces:
  /powertrain/engine, /chassis/brakes, /body/door/front_left,
  /body/lights, /perception/lidar
"""

import unittest

import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import ALL_DEMO_NODES, create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=ALL_DEMO_NODES,
        fault_manager=False,
    )


class TestEntityModelRuntime(GatewayTestCase):
    """Verify the SOVD entity model in runtime_only discovery mode."""

    MIN_EXPECTED_APPS = 8
    REQUIRED_FUNCTIONS = {'powertrain', 'chassis', 'body', 'perception'}
    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor', 'actuator', 'lidar_sensor'}

    # ------------------------------------------------------------------
    # Namespaces create Functions
    # ------------------------------------------------------------------

    def test_namespaces_create_functions(self):
        """Namespace grouping creates Function entities from first-level segments.

        Demo nodes in /powertrain/engine, /chassis/brakes, /body/..., /perception/lidar
        should produce Functions: powertrain, chassis, body, perception.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/functions')
        self.assertIn('items', data)
        functions = data['items']
        func_ids = {f['id'] for f in functions}

        for expected_func in ('powertrain', 'chassis', 'body', 'perception'):
            self.assertIn(
                expected_func, func_ids,
                f'Expected Function "{expected_func}" from namespace grouping. '
                f'Found: {func_ids}'
            )

        # Each Function should have proper structure
        for func in functions:
            self.assertIn('id', func)
            self.assertIn('name', func)
            self.assertIn('href', func)

    def test_function_detail_shows_namespace_source(self):
        """GET /functions/{id} returns function with source=runtime.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/functions/powertrain')
        self.assertEqual(data['id'], 'powertrain')
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(
            x_medkit.get('source'), 'runtime',
            'Function should have source=runtime'
        )

    # ------------------------------------------------------------------
    # Areas empty in runtime_only mode
    # ------------------------------------------------------------------

    def test_areas_empty_in_runtime_mode(self):
        """GET /areas returns empty items in runtime_only mode.

        With the SOVD-aligned entity model, namespaces create Functions,
        not Areas. Areas require explicit manifest definition.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']
        self.assertEqual(
            len(areas), 0,
            f'Areas should be empty in runtime_only mode, got: '
            f'{[a.get("id") for a in areas]}'
        )

    # ------------------------------------------------------------------
    # Default Component from host
    # ------------------------------------------------------------------

    def test_default_component_from_host(self):
        """GET /components returns a single host-derived default Component.

        The default Component is created from HostInfoProvider with
        hostname, OS, and architecture metadata.

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertEqual(
            len(components), 1,
            f'Expected exactly 1 host component in runtime_only mode, '
            f'got {len(components)}: {[c.get("id") for c in components]}'
        )

        component = components[0]
        self.assertIn('id', component)
        self.assertIn('name', component)
        self.assertIn('href', component)

        # Verify x-medkit metadata
        self.assertIn('x-medkit', component)
        x_medkit = component['x-medkit']
        self.assertEqual(
            x_medkit.get('source'), 'runtime',
            'Host component should have source=runtime'
        )

    def test_default_component_detail(self):
        """GET /components/{id} returns component detail for the host component.

        @verifies REQ_INTEROP_003
        """
        # First get the component ID from listing
        listing = self.get_json('/components')
        component_id = listing['items'][0]['id']

        detail = self.get_json(f'/components/{component_id}')
        self.assertEqual(detail['id'], component_id)
        self.assertIn('name', detail)

    # ------------------------------------------------------------------
    # Apps linked to default Component
    # ------------------------------------------------------------------

    def test_apps_linked_to_default_component(self):
        """Apps have is-located-on link pointing to the host component.

        Every discovered App should reference the single default Component
        via the SOVD is-located-on relationship.

        @verifies REQ_INTEROP_003
        """
        # Get the default component ID
        comp_data = self.get_json('/components')
        self.assertEqual(len(comp_data['items']), 1)
        component_id = comp_data['items'][0]['id']

        # Get app detail and check is-located-on
        app_data = self.get_json('/apps/temp_sensor')
        self.assertIn('_links', app_data)
        links = app_data['_links']
        self.assertIn(
            'is-located-on', links,
            f'App should have is-located-on link. Links: {links}'
        )

        # The link should reference the host component
        expected_path = f'/api/v1/components/{component_id}'
        self.assertEqual(
            links['is-located-on'], expected_path,
            f'is-located-on should point to host component {component_id}'
        )

    def test_app_is_located_on_endpoint(self):
        """GET /apps/{id}/is-located-on returns the host component.

        The is-located-on endpoint returns a collection response with
        items containing the host component.

        @verifies REQ_INTEROP_003
        """
        # Get the default component ID
        comp_data = self.get_json('/components')
        component_id = comp_data['items'][0]['id']

        # Call the is-located-on endpoint (returns collection format)
        data = self.get_json('/apps/temp_sensor/is-located-on')
        self.assertIn('items', data)
        self.assertGreaterEqual(len(data['items']), 1)
        self.assertEqual(
            data['items'][0]['id'], component_id,
            'is-located-on should return the host component'
        )

    def test_multiple_apps_same_component(self):
        """All Apps reference the same default Component.

        In runtime_only mode, there is only one Component, so every App
        must be located on it. We verify a subset of known demo apps
        rather than all apps (the gateway node itself is also an app).

        @verifies REQ_INTEROP_003
        """
        comp_data = self.get_json('/components')
        component_id = comp_data['items'][0]['id']
        expected_path = f'/api/v1/components/{component_id}'

        # Check a sample of demo apps from different namespaces
        demo_apps = ['temp_sensor', 'actuator', 'lidar_sensor', 'controller']
        for app_id in demo_apps:
            app_detail = self.get_json(f'/apps/{app_id}')
            links = app_detail.get('_links', {})
            located_on = links.get('is-located-on', '')
            self.assertEqual(
                located_on, expected_path,
                f'App {app_id} should be located on {component_id}, '
                f'but is-located-on is: {located_on}'
            )

    # ------------------------------------------------------------------
    # Cross-entity relationships
    # ------------------------------------------------------------------

    def test_function_hosts_apps(self):
        """Functions group the Apps from their namespace.

        The powertrain Function (from /powertrain namespace) should host
        apps like temp_sensor, rpm_sensor, calibration, long_calibration.
        We verify this by checking the x-medkit.ros2.node field on app detail
        which contains the FQN (e.g. /powertrain/engine/temp_sensor).

        @verifies REQ_INTEROP_003
        """
        data = self.get_json('/functions/powertrain')
        self.assertEqual(data['id'], 'powertrain')

        # Check that powertrain apps are accessible via FQN
        # Apps in /powertrain/engine: temp_sensor, rpm_sensor,
        # calibration, long_calibration
        apps_data = self.get_json('/apps')
        powertrain_app_ids = set()
        for app in apps_data['items']:
            app_detail = self.get_json(f'/apps/{app["id"]}')
            x_medkit = app_detail.get('x-medkit', {})
            ros2 = x_medkit.get('ros2', {})
            # ros2.node contains the FQN like /powertrain/engine/temp_sensor
            node_fqn = ros2.get('node', '')
            if node_fqn.startswith('/powertrain'):
                powertrain_app_ids.add(app['id'])

        self.assertIn('temp_sensor', powertrain_app_ids)
        self.assertIn('rpm_sensor', powertrain_app_ids)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
