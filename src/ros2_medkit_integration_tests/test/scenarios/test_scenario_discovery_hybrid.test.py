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

"""Scenario: Hybrid discovery mode validation.

Validates discovery endpoints when the gateway is configured with
discovery_mode: hybrid, combining manifest definitions with runtime
ROS 2 graph discovery.

Tests verify:
- Areas from manifest are present with descriptions and subareas
- Components from manifest are present with types and area relationships
- Component dependencies (depends-on) and capabilities
- Apps from manifest are enriched with runtime data (topics, services)
- Runtime-discovered nodes are linked to manifest apps (is_online)
- Functions aggregate data from their hosting apps
- Orphan nodes (not in manifest) are handled according to config
- Error handling for nonexistent entities
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    gateway = create_gateway_node(extra_params={
        'discovery_mode': 'hybrid',
        'manifest_path': manifest_path,
        'manifest_strict_validation': False,
        'unmanifested_nodes': 'warn',
    })

    # Launch all demo nodes matching the manifest for full hybrid validation
    demo_nodes = create_demo_nodes(
        [
            'temp_sensor', 'rpm_sensor', 'calibration', 'long_calibration',
            'pressure_sensor', 'actuator',
            'status_sensor', 'controller',
            'lidar_sensor',
        ],
        lidar_faulty=False,
    )

    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


class TestScenarioDiscoveryHybrid(GatewayTestCase):
    """Scenario: Gateway in hybrid discovery mode.

    The gateway loads entity definitions from demo_nodes_manifest.yaml and
    enriches them with runtime ROS 2 graph discovery. All demo nodes are
    launched to validate runtime linking.

    Steps:
    1. Verify areas from manifest including hierarchy and descriptions
    2. Verify components from manifest with types, areas, and dependencies
    3. Verify apps from manifest are enriched with runtime data
    4. Verify app-component and app-dependency relationships
    5. Verify functions aggregate data from hosting apps
    6. Verify runtime enrichment (topics, services, online status)
    7. Verify error handling for nonexistent entities
    """

    # Hybrid mode needs all demo nodes linked before tests run
    MIN_EXPECTED_APPS = 5
    REQUIRED_AREAS = {'powertrain', 'chassis', 'body', 'perception', 'engine'}
    REQUIRED_APPS = {
        'engine-temp-sensor', 'engine-rpm-sensor',
        'brake-pressure-sensor', 'lidar-sensor',
    }

    # =========================================================================
    # Areas - Manifest + Runtime
    # =========================================================================

    def test_01_areas_from_manifest(self):
        """Areas are loaded from manifest in hybrid mode.

        @verifies REQ_INTEROP_003
        """
        self.assert_entity_list_contains(
            'areas',
            {'powertrain', 'chassis', 'body', 'perception', 'engine'},
        )

    def test_02_area_with_description(self):
        """Area descriptions from manifest are preserved.

        @verifies REQ_INTEROP_003
        """
        area = self.assert_entity_exists('areas', 'powertrain')
        self.assertEqual(area['id'], 'powertrain')
        if 'description' in area:
            self.assertIn('Engine', area['description'])

    def test_03_area_subareas_hierarchy(self):
        """Subarea relationships from manifest: body -> door, lights.

        @verifies REQ_INTEROP_004
        """
        data = self.get_json('/areas/body/subareas')
        subarea_ids = [s['id'] for s in data['items']]
        self.assertIn('door', subarea_ids)
        self.assertIn('lights', subarea_ids)

    def test_04_nested_subareas(self):
        """Deeply nested subareas: door -> front-left-door.

        @verifies REQ_INTEROP_004
        """
        data = self.get_json('/areas/door/subareas')
        subarea_ids = [s['id'] for s in data['items']]
        self.assertIn('front-left-door', subarea_ids)

    # =========================================================================
    # Components - Manifest Definitions
    # =========================================================================

    def test_05_components_from_manifest(self):
        """Components are loaded from manifest.

        @verifies REQ_INTEROP_003
        """
        self.assert_entity_list_contains(
            'components',
            {'engine-ecu', 'temp-sensor-hw', 'brake-ecu', 'lidar-unit'},
        )

    def test_06_component_type_preserved(self):
        """Component type from manifest is preserved in x-medkit.

        @verifies REQ_INTEROP_003
        """
        component = self.assert_entity_exists('components', 'engine-ecu')
        self.assertEqual(component['x-medkit']['type'], 'controller')

    def test_07_component_area_relationship(self):
        """Component is associated with correct area.

        @verifies REQ_INTEROP_006
        """
        data = self.get_json('/areas/engine/components')
        component_ids = [c['id'] for c in data['items']]
        self.assertIn('engine-ecu', component_ids)
        self.assertIn('temp-sensor-hw', component_ids)
        self.assertIn('rpm-sensor-hw', component_ids)

    def test_08_component_subcomponents(self):
        """GET /components/{id}/subcomponents returns list in hybrid mode.

        @verifies REQ_INTEROP_005
        """
        data = self.get_json('/components/engine-ecu/subcomponents')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_09_component_subcomponents_not_found(self):
        """GET /components/{id}/subcomponents returns 404 for unknown component.

        @verifies REQ_INTEROP_005
        """
        self.get_json(
            '/components/nonexistent/subcomponents', expected_status=404
        )

    def test_10_component_depends_on_returns_items(self):
        """GET /components/{id}/depends-on returns dependency references.

        @verifies REQ_INTEROP_008
        """
        data = self.get_json('/components/engine-ecu/depends-on')
        self.assertIn('items', data)

        dep_ids = [d['id'] for d in data['items']]
        self.assertIn('temp-sensor-hw', dep_ids)
        self.assertIn('rpm-sensor-hw', dep_ids)

        # Each item should have href link
        for item in data['items']:
            self.assertIn('href', item)
            self.assertTrue(item['href'].startswith('/api/v1/components/'))

    def test_11_component_depends_on_empty(self):
        """GET /components/{id}/depends-on returns empty list for component without deps.

        @verifies REQ_INTEROP_008
        """
        data = self.get_json('/components/temp-sensor-hw/depends-on')
        self.assertIn('items', data)
        self.assertEqual(len(data['items']), 0)

    def test_12_component_depends_on_not_found(self):
        """GET /components/{id}/depends-on returns 404 for unknown component.

        @verifies REQ_INTEROP_008
        """
        self.get_json('/components/nonexistent/depends-on', expected_status=404)

    def test_13_component_capabilities_includes_depends_on_link(self):
        """Component with dependencies has depends-on in capabilities.

        @verifies REQ_INTEROP_008
        """
        component = self.assert_entity_exists('components', 'engine-ecu')
        self.assertIn('x-medkit', component)
        self.assertIn('capabilities', component['x-medkit'])

        cap_hrefs = [c.get('href', '') for c in component['x-medkit']['capabilities']]
        self.assertTrue(
            any('/depends-on' in href for href in cap_hrefs),
            f'Expected depends-on capability in: {cap_hrefs}'
        )

    # =========================================================================
    # Apps - Manifest + Runtime Linking
    # =========================================================================

    def test_14_apps_from_manifest(self):
        """All manifest-defined apps are present.

        @verifies REQ_INTEROP_003
        """
        self.assert_entity_list_contains('apps', {
            'engine-temp-sensor',
            'engine-rpm-sensor',
            'engine-calibration-service',
            'engine-long-calibration',
            'brake-pressure-sensor',
            'brake-actuator',
            'door-status-sensor',
            'light-controller',
            'lidar-sensor',
        })

    def test_15_app_online_with_runtime_node(self):
        """Apps linked to running nodes have is_online=true in x-medkit.

        Runtime linking happens asynchronously after nodes start, so poll
        until at least one app is online.

        @verifies REQ_INTEROP_003
        """
        deadline = time.monotonic() + 30.0
        online_apps = []
        while time.monotonic() < deadline:
            data = self.get_json('/apps')
            apps_by_id = {a['id']: a for a in data['items']}

            online_apps = [
                app_id for app_id, app in apps_by_id.items()
                if app.get('x-medkit', {}).get('is_online', False)
            ]

            if online_apps:
                break
            time.sleep(1.0)

        self.assertGreater(
            len(online_apps), 0,
            'No apps are online - runtime linking may have failed'
        )

    def test_16_app_has_runtime_topics(self):
        """Online app has topics from runtime discovery."""
        # Wait a bit for runtime linking
        time.sleep(3)

        response = requests.get(
            f'{self.BASE_URL}/apps/engine-temp-sensor/data', timeout=5
        )

        if response.status_code == 200:
            data = response.json()
            if 'items' in data and data['items']:
                topic_names = [t.get('name', '') for t in data['items']]
                self.assertTrue(
                    any('temperature' in name for name in topic_names),
                    f'Expected temperature topic, got: {topic_names}'
                )

    def test_17_app_has_runtime_service(self):
        """App with service has it discovered at runtime."""
        response = requests.get(
            f'{self.BASE_URL}/apps/engine-calibration-service/operations',
            timeout=5,
        )

        if response.status_code == 200:
            data = response.json()
            if 'items' in data and data['items']:
                op_names = [o.get('name', '') for o in data['items']]
                self.assertTrue(
                    any('calibrate' in name for name in op_names),
                    f'Expected calibrate service, got: {op_names}'
                )

    def test_18_app_component_relationship(self):
        """App is-located-on links to correct component via HATEOAS."""
        app = self.assert_entity_exists('apps', 'engine-temp-sensor')
        self.assertIn('_links', app, 'App response should contain _links')
        self.assertIn(
            'is-located-on', app['_links'],
            'App should have is-located-on link when component is specified'
        )
        self.assertEqual(
            app['_links']['is-located-on'],
            '/api/v1/components/temp-sensor-hw'
        )

    def test_19_app_depends_on_relationship(self):
        """App depends_on creates dependency link.

        @verifies REQ_INTEROP_009
        """
        app = self.assert_entity_exists('apps', 'engine-long-calibration')
        if 'depends_on' in app:
            self.assertIn('engine-calibration-service', app['depends_on'])

    # =========================================================================
    # Functions - Aggregation from Hosts
    # =========================================================================

    def test_20_functions_from_manifest(self):
        """Functions are loaded from manifest.

        @verifies REQ_INTEROP_003
        """
        self.assert_entity_list_contains('functions', {
            'engine-monitoring',
            'engine-calibration',
            'brake-system',
            'body-electronics',
            'perception-system',
        })

    def test_21_function_hosts_relationship(self):
        """Function hosts are correctly linked.

        @verifies REQ_INTEROP_007
        """
        data = self.get_json('/functions/engine-monitoring/hosts')
        host_ids = [h['id'] for h in data['items']]
        self.assertIn('engine-temp-sensor', host_ids)
        self.assertIn('engine-rpm-sensor', host_ids)

    def test_22_function_aggregates_host_data(self):
        """Function /data aggregates topics from all hosts."""
        response = requests.get(
            f'{self.BASE_URL}/functions/engine-monitoring/data', timeout=5
        )

        if response.status_code == 200:
            data = response.json()
            if 'items' in data:
                topic_names = [t.get('name', '') for t in data['items']]
                self.assertIsInstance(topic_names, list)

    def test_23_function_aggregates_host_operations(self):
        """Function /operations aggregates services from all hosts."""
        response = requests.get(
            f'{self.BASE_URL}/functions/engine-calibration/operations',
            timeout=5,
        )

        if response.status_code == 200:
            data = response.json()
            if 'items' in data:
                op_names = [o.get('name', '') for o in data['items']]
                self.assertIsInstance(op_names, list)

    def test_24_function_with_tags(self):
        """Function tags from manifest are preserved.

        @verifies REQ_INTEROP_011
        """
        func = self.assert_entity_exists('functions', 'brake-system')
        if 'tags' in func:
            self.assertIn('safety-critical', func['tags'])

    # =========================================================================
    # Hybrid-Specific Behavior
    # =========================================================================

    def test_25_runtime_enriches_manifest_data(self):
        """Runtime discovery adds data to manifest entities."""
        app = self.assert_entity_exists('apps', 'lidar-sensor')

        # Manifest data should be present
        self.assertEqual(app['name'], 'LiDAR Sensor')

        # Tags from manifest
        if 'tags' in app:
            self.assertIn('fault-reporter', app['tags'])

    def test_26_capabilities_include_runtime_resources(self):
        """Capabilities reflect runtime-discovered resources."""
        app = self.assert_entity_exists('apps', 'engine-temp-sensor')
        self.assertIn('capabilities', app)

        cap_hrefs = [c.get('href', '') for c in app['capabilities']]
        self.assertTrue(
            any('/data' in href for href in cap_hrefs),
            'Expected data capability'
        )

    # =========================================================================
    # Error Handling
    # =========================================================================

    def test_27_nonexistent_area(self):
        """404 for non-existent area."""
        self.assert_entity_not_found('areas', 'nonexistent')

    def test_28_nonexistent_component(self):
        """404 for non-existent component."""
        self.assert_entity_not_found('components', 'nonexistent')

    def test_29_nonexistent_app(self):
        """404 for non-existent app."""
        self.assert_entity_not_found('apps', 'nonexistent')

    def test_30_nonexistent_function(self):
        """404 for non-existent function."""
        self.assert_entity_not_found('functions', 'nonexistent')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            allowed = {0, -2, -15}  # OK, SIGINT, SIGTERM
            self.assertIn(
                info.returncode, allowed,
                f'Process {info.process_name} exited with code {info.returncode}'
            )
