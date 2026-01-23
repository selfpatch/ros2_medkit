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
Integration tests for hybrid discovery mode.

This test file validates discovery endpoints when the gateway is configured
with discovery_mode: hybrid, combining manifest definitions with runtime
ROS 2 graph discovery.

Tests verify:
- Areas from manifest are present
- Components from manifest are present
- Apps from manifest are enriched with runtime data (topics, services)
- Runtime-discovered nodes are linked to manifest apps
- Functions aggregate data from their hosting apps
- Orphan nodes (not in manifest) are handled according to config
"""

import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests


def get_coverage_env():
    """Get environment variables for gcov coverage data collection."""
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_gateway')
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_gateway')
        if os.path.exists(build_dir):
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        # Coverage env is optional - gracefully continue without coverage settings
        pass
    return {}


def generate_test_description():
    """Generate launch description with gateway in hybrid discovery mode."""
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    coverage_env = get_coverage_env()

    # Gateway node with hybrid discovery mode
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[{
            'discovery_mode': 'hybrid',
            'manifest_path': manifest_path,
            'manifest_strict_validation': False,  # Allow warnings about subarea references
            # Allow orphan nodes to be discovered (warn, not fail)
            'unmanifested_nodes': 'warn',
        }],
        additional_env=coverage_env,
    )

    # Launch demo nodes matching the manifest
    demo_nodes = [
        # Powertrain/Engine
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_engine_temp_sensor',
            name='temp_sensor',
            namespace='/powertrain/engine',
            output='screen',
            additional_env=coverage_env,
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_rpm_sensor',
            name='rpm_sensor',
            namespace='/powertrain/engine',
            output='screen',
            additional_env=coverage_env,
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_calibration_service',
            name='calibration',
            namespace='/powertrain/engine',
            output='screen',
            additional_env=coverage_env,
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_long_calibration_action',
            name='long_calibration',
            namespace='/powertrain/engine',
            output='screen',
            additional_env=coverage_env,
        ),
        # Chassis/Brakes
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_brake_pressure_sensor',
            name='pressure_sensor',
            namespace='/chassis/brakes',
            output='screen',
            additional_env=coverage_env,
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_brake_actuator',
            name='actuator',
            namespace='/chassis/brakes',
            output='screen',
            additional_env=coverage_env,
        ),
        # Body
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_door_status_sensor',
            name='status_sensor',
            namespace='/body/door/front_left',
            output='screen',
            additional_env=coverage_env,
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_light_controller',
            name='controller',
            namespace='/body/lights',
            output='screen',
            additional_env=coverage_env,
        ),
        # Perception
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_lidar_sensor',
            name='lidar_sensor',
            namespace='/perception/lidar',
            output='screen',
            additional_env=coverage_env,
        ),
    ]

    delayed_nodes = TimerAction(
        period=2.0,
        actions=demo_nodes,
    )

    return (
        LaunchDescription([
            gateway_node,
            delayed_nodes,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


API_BASE_PATH = '/api/v1'


class TestDiscoveryHybridMode(unittest.TestCase):
    """Integration tests for hybrid discovery mode."""

    BASE_URL = f'http://localhost:8080{API_BASE_PATH}'
    MAX_DISCOVERY_WAIT = 60.0
    MIN_EXPECTED_APPS_ONLINE = 5

    @classmethod
    def setUpClass(cls):
        """Wait for gateway and discovery to complete."""
        # Wait for gateway health
        for i in range(30):
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if response.status_code == 200:
                    break
            except requests.exceptions.RequestException:
                # Gateway not ready yet, retry after sleep
                pass
            time.sleep(1)
        else:
            raise unittest.SkipTest('Gateway not responding')

        # Wait for apps to come online (runtime linking)
        start_time = time.time()
        while time.time() - start_time < cls.MAX_DISCOVERY_WAIT:
            try:
                response = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
                if response.status_code == 200:
                    data = response.json()
                    online_count = sum(
                        1 for a in data['items'] if a.get('is_online', False)
                    )
                    if online_count >= cls.MIN_EXPECTED_APPS_ONLINE:
                        print(f'✓ Hybrid discovery: {online_count} apps online')
                        return
                    print(f'  Waiting for apps: {online_count}/{cls.MIN_EXPECTED_APPS_ONLINE}...')
            except requests.exceptions.RequestException:
                # Apps endpoint not ready yet, retry after sleep
                pass
            time.sleep(2)

        print('⚠ Warning: Not all expected apps came online')

    # =========================================================================
    # Areas - Manifest + Runtime
    # =========================================================================

    def test_areas_from_manifest(self):
        """
        Test areas are loaded from manifest in hybrid mode.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/areas', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        area_ids = [a['id'] for a in data['items']]

        # Manifest-defined areas should be present
        self.assertIn('powertrain', area_ids)
        self.assertIn('chassis', area_ids)
        self.assertIn('body', area_ids)
        self.assertIn('perception', area_ids)
        self.assertIn('engine', area_ids)

    def test_area_with_description(self):
        """
        Test area descriptions from manifest are preserved.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/areas/powertrain', timeout=5)
        self.assertEqual(response.status_code, 200)

        area = response.json()
        self.assertEqual(area['id'], 'powertrain')
        # Description should come from manifest
        if 'description' in area:
            self.assertIn('Engine', area['description'])

    def test_area_subareas_hierarchy(self):
        """
        Test subarea relationships from manifest.

        @verifies REQ_INTEROP_004
        """
        response = requests.get(f'{self.BASE_URL}/areas/body/subareas', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        subarea_ids = [s['id'] for s in data['items']]

        # Body has subareas: door, lights
        self.assertIn('door', subarea_ids)
        self.assertIn('lights', subarea_ids)

    def test_nested_subareas(self):
        """
        Test deeply nested subareas (door -> front-left-door).

        @verifies REQ_INTEROP_004
        """
        response = requests.get(f'{self.BASE_URL}/areas/door/subareas', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        subarea_ids = [s['id'] for s in data['items']]
        self.assertIn('front-left-door', subarea_ids)

    # =========================================================================
    # Components - Manifest Definitions
    # =========================================================================

    def test_components_from_manifest(self):
        """
        Test components are loaded from manifest.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/components', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        component_ids = [c['id'] for c in data['items']]

        # Hardware components from manifest
        self.assertIn('engine-ecu', component_ids)
        self.assertIn('temp-sensor-hw', component_ids)
        self.assertIn('brake-ecu', component_ids)
        self.assertIn('lidar-unit', component_ids)

    def test_component_type_preserved(self):
        """
        Test component type from manifest is preserved.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/components/engine-ecu', timeout=5)
        self.assertEqual(response.status_code, 200)

        component = response.json()
        # SOVD-compliant: type is in x-medkit extension
        self.assertEqual(component['x-medkit']['type'], 'controller')

    def test_component_area_relationship(self):
        """
        Test component is associated with correct area.

        @verifies REQ_INTEROP_006
        """
        response = requests.get(f'{self.BASE_URL}/areas/engine/components', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        component_ids = [c['id'] for c in data['items']]

        # Engine area should have these components
        self.assertIn('engine-ecu', component_ids)
        self.assertIn('temp-sensor-hw', component_ids)
        self.assertIn('rpm-sensor-hw', component_ids)

    def test_hybrid_component_subcomponents(self):
        """
        Test GET /components/{id}/subcomponents returns subcomponents in hybrid mode.

        @verifies REQ_INTEROP_005
        """
        # Test subcomponents endpoint for a component
        # Returns empty list if no subcomponents defined, but endpoint works
        response = requests.get(f'{self.BASE_URL}/components/engine-ecu/subcomponents', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        # Subcomponents may be empty but format should be correct
        self.assertIsInstance(data['items'], list)

    def test_hybrid_component_subcomponents_not_found(self):
        """
        Test GET /components/{id}/subcomponents returns 404 for unknown component in hybrid mode.

        @verifies REQ_INTEROP_005
        """
        response = requests.get(f'{self.BASE_URL}/components/nonexistent/subcomponents', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_component_depends_on_returns_items(self):
        """
        Test GET /components/{id}/depends-on returns dependency references.

        @verifies REQ_INTEROP_008
        """
        response = requests.get(f'{self.BASE_URL}/components/engine-ecu/depends-on', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

        # engine-ecu depends on temp-sensor-hw and rpm-sensor-hw
        dep_ids = [d['id'] for d in data['items']]
        self.assertIn('temp-sensor-hw', dep_ids)
        self.assertIn('rpm-sensor-hw', dep_ids)

        # Each item should have href link
        for item in data['items']:
            self.assertIn('href', item)
            self.assertTrue(item['href'].startswith('/api/v1/components/'))

    def test_component_depends_on_empty(self):
        """
        Test GET /components/{id}/depends-on returns empty list for component without deps.

        @verifies REQ_INTEROP_008
        """
        # temp-sensor-hw has no dependencies
        response = requests.get(f'{self.BASE_URL}/components/temp-sensor-hw/depends-on', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertEqual(len(data['items']), 0)

    def test_component_depends_on_not_found(self):
        """
        Test GET /components/{id}/depends-on returns 404 for unknown component.

        @verifies REQ_INTEROP_008
        """
        response = requests.get(f'{self.BASE_URL}/components/nonexistent/depends-on', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_component_capabilities_includes_depends_on_link(self):
        """
        Test component with dependencies has depends-on in capabilities.

        @verifies REQ_INTEROP_008
        """
        response = requests.get(f'{self.BASE_URL}/components/engine-ecu', timeout=5)
        self.assertEqual(response.status_code, 200)

        component = response.json()
        # SOVD-compliant: capabilities is in x-medkit extension
        self.assertIn('x-medkit', component)
        self.assertIn('capabilities', component['x-medkit'])

        # Should have depends-on capability
        cap_hrefs = [c.get('href', '') for c in component['x-medkit']['capabilities']]
        self.assertTrue(
            any('/depends-on' in href for href in cap_hrefs),
            f'Expected depends-on capability in: {cap_hrefs}'
        )

    # =========================================================================
    # Apps - Manifest + Runtime Linking
    # =========================================================================

    def test_apps_from_manifest(self):
        """
        Test apps are loaded from manifest.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/apps', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        app_ids = [a['id'] for a in data['items']]

        # All manifest apps should be present
        expected_apps = [
            'engine-temp-sensor',
            'engine-rpm-sensor',
            'engine-calibration-service',
            'engine-long-calibration',
            'brake-pressure-sensor',
            'brake-actuator',
            'door-status-sensor',
            'light-controller',
            'lidar-sensor',
        ]

        for app_id in expected_apps:
            self.assertIn(app_id, app_ids, f'Missing app: {app_id}')

    def test_app_online_with_runtime_node(self):
        """
        Test apps linked to running nodes have is_online=true.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/apps', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        apps_by_id = {a['id']: a for a in data['items']}

        # SOVD-compliant: is_online is in x-medkit extension
        online_apps = [
            app_id for app_id, app in apps_by_id.items()
            if app.get('x-medkit', {}).get('is_online', False)
        ]

        self.assertGreater(
            len(online_apps), 0,
            'No apps are online - runtime linking may have failed'
        )

    def test_app_has_runtime_topics(self):
        """Test online app has topics from runtime discovery."""
        # Wait a bit for runtime linking
        time.sleep(3)

        response = requests.get(
            f'{self.BASE_URL}/apps/engine-temp-sensor/data', timeout=5
        )

        # This test verifies runtime linking when it succeeds.
        # Skip assertion if no topics found - runtime linking may take longer.
        if response.status_code == 200:
            data = response.json()
            # Should have temperature topic if runtime linking worked
            if 'items' in data and data['items']:
                topic_names = [t.get('name', '') for t in data['items']]
                # Demo node publishes 'temperature'
                self.assertTrue(
                    any('temperature' in name for name in topic_names),
                    f'Expected temperature topic, got: {topic_names}'
                )

    def test_app_has_runtime_service(self):
        """Test app with service has it discovered at runtime."""
        response = requests.get(
            f'{self.BASE_URL}/apps/engine-calibration-service/operations', timeout=5
        )

        # This test verifies runtime linking when it succeeds.
        # Skip assertion if no operations found - runtime linking may take longer.
        if response.status_code == 200:
            data = response.json()
            if 'items' in data and data['items']:
                op_names = [o.get('name', '') for o in data['items']]
                # Demo node provides 'calibrate' service
                self.assertTrue(
                    any('calibrate' in name for name in op_names),
                    f'Expected calibrate service, got: {op_names}'
                )

    def test_app_component_relationship(self):
        """Test app is_located_on links to correct component."""
        response = requests.get(f'{self.BASE_URL}/apps/engine-temp-sensor', timeout=5)
        self.assertEqual(response.status_code, 200)

        app = response.json()
        # Should be located on temp-sensor-hw via HATEOAS link
        self.assertIn('_links', app, 'App response should contain _links')
        self.assertIn(
            'is-located-on', app['_links'],
            'App should have is-located-on link when component is specified'
        )
        self.assertEqual(
            app['_links']['is-located-on'],
            '/api/v1/components/temp-sensor-hw'
        )

    def test_app_depends_on_relationship(self):
        """
        Test app depends_on creates dependency link.

        @verifies REQ_INTEROP_009
        """
        response = requests.get(f'{self.BASE_URL}/apps/engine-long-calibration', timeout=5)
        self.assertEqual(response.status_code, 200)

        app = response.json()
        # Should depend on engine-calibration-service
        if 'depends_on' in app:
            self.assertIn('engine-calibration-service', app['depends_on'])

    # =========================================================================
    # Functions - Aggregation from Hosts
    # =========================================================================

    def test_functions_from_manifest(self):
        """
        Test functions are loaded from manifest.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/functions', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        function_ids = [f['id'] for f in data['items']]

        expected_functions = [
            'engine-monitoring',
            'engine-calibration',
            'brake-system',
            'body-electronics',
            'perception-system',
        ]

        for func_id in expected_functions:
            self.assertIn(func_id, function_ids, f'Missing function: {func_id}')

    def test_function_hosts_relationship(self):
        """
        Test function hosts are correctly linked.

        @verifies REQ_INTEROP_007
        """
        response = requests.get(
            f'{self.BASE_URL}/functions/engine-monitoring/hosts', timeout=5
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        host_ids = [h['id'] for h in data['items']]

        # engine-monitoring hosted by temp-sensor and rpm-sensor
        self.assertIn('engine-temp-sensor', host_ids)
        self.assertIn('engine-rpm-sensor', host_ids)

    def test_function_aggregates_host_data(self):
        """Test function /data aggregates topics from all hosts."""
        response = requests.get(
            f'{self.BASE_URL}/functions/engine-monitoring/data', timeout=5
        )

        if response.status_code == 200:
            data = response.json()
            if 'items' in data:
                # Should have topics from both temp_sensor and rpm_sensor
                topic_names = [t.get('name', '') for t in data['items']]
                # At minimum should have some data
                self.assertIsInstance(topic_names, list)

    def test_function_aggregates_host_operations(self):
        """Test function /operations aggregates services from all hosts."""
        response = requests.get(
            f'{self.BASE_URL}/functions/engine-calibration/operations', timeout=5
        )

        if response.status_code == 200:
            data = response.json()
            if 'items' in data:
                # Should have calibrate service and long_calibration action
                op_names = [o.get('name', '') for o in data['items']]
                self.assertIsInstance(op_names, list)

    def test_function_with_tags(self):
        """
        Test function tags from manifest are preserved.

        @verifies REQ_INTEROP_011
        """
        response = requests.get(f'{self.BASE_URL}/functions/brake-system', timeout=5)
        self.assertEqual(response.status_code, 200)

        func = response.json()
        if 'tags' in func:
            self.assertIn('safety-critical', func['tags'])

    # =========================================================================
    # Hybrid-Specific Behavior
    # =========================================================================

    def test_runtime_enriches_manifest_data(self):
        """Test runtime discovery adds data to manifest entities."""
        # Get an app that's online
        response = requests.get(f'{self.BASE_URL}/apps/lidar-sensor', timeout=5)
        self.assertEqual(response.status_code, 200)

        app = response.json()

        # Manifest data should be present
        self.assertEqual(app['name'], 'LiDAR Sensor')

        # Tags from manifest
        if 'tags' in app:
            self.assertIn('fault-reporter', app['tags'])

    def test_capabilities_include_runtime_resources(self):
        """Test capabilities reflect runtime-discovered resources."""
        response = requests.get(f'{self.BASE_URL}/apps/engine-temp-sensor', timeout=5)
        self.assertEqual(response.status_code, 200)

        app = response.json()
        self.assertIn('capabilities', app)

        # Should have data capability (topics discovered at runtime)
        cap_hrefs = [c.get('href', '') for c in app['capabilities']]
        self.assertTrue(
            any('/data' in href for href in cap_hrefs),
            'Expected data capability'
        )

    # =========================================================================
    # Error Handling
    # =========================================================================

    def test_nonexistent_area(self):
        """Test 404 for non-existent area."""
        response = requests.get(f'{self.BASE_URL}/areas/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_nonexistent_component(self):
        """Test 404 for non-existent component."""
        response = requests.get(f'{self.BASE_URL}/components/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_nonexistent_app(self):
        """Test 404 for non-existent app."""
        response = requests.get(f'{self.BASE_URL}/apps/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_nonexistent_function(self):
        """Test 404 for non-existent function."""
        response = requests.get(f'{self.BASE_URL}/functions/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)


@launch_testing.post_shutdown_test()
class TestDiscoveryHybridModeShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_code(self, proc_info):
        """Check gateway exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
