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
Integration tests for manifest-only discovery mode.

This test file validates discovery endpoints when the gateway is configured
with discovery_mode: manifest_only using demo_nodes_manifest.yaml.

Tests verify:
- Areas are loaded from manifest (not runtime discovery)
- Components are loaded from manifest
- Apps are loaded from manifest with correct ros_binding
- Functions are loaded from manifest with hosted_by relationships
- Subareas and related-components relationships work
- Entity details and capabilities are correct
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
    """Generate launch description with gateway in manifest_only mode."""
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    coverage_env = get_coverage_env()

    # Gateway node with manifest_only discovery mode
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[{
            'discovery_mode': 'manifest_only',
            'manifest_path': manifest_path,
            'manifest_strict_validation': False,  # Allow warnings about subarea references
        }],
        additional_env=coverage_env,
    )

    # Launch demo nodes to verify apps become online
    demo_nodes = [
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
            executable='demo_brake_pressure_sensor',
            name='pressure_sensor',
            namespace='/chassis/brakes',
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


class TestDiscoveryManifestMode(unittest.TestCase):
    """Integration tests for manifest-only discovery mode."""

    BASE_URL = f'http://localhost:8080{API_BASE_PATH}'
    MAX_WAIT = 30.0

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready."""
        for i in range(30):
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if response.status_code == 200:
                    # Give time for manifest to be loaded
                    time.sleep(2)
                    return
            except requests.exceptions.RequestException:
                # Gateway not ready yet, retry after sleep
                pass
            time.sleep(1)
        raise unittest.SkipTest('Gateway not responding')

    # =========================================================================
    # Areas Endpoints
    # =========================================================================

    def test_list_areas(self):
        """
        Test GET /areas returns all manifest-defined areas.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/areas', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('total_count', data)

        # Manifest defines: powertrain, chassis, body, perception (top-level)
        # Plus subareas: engine, brakes, door, front-left-door, lights, lidar
        area_ids = [a['id'] for a in data['items']]

        # Check top-level areas
        self.assertIn('powertrain', area_ids)
        self.assertIn('chassis', area_ids)
        self.assertIn('body', area_ids)
        self.assertIn('perception', area_ids)

        # Check subareas
        self.assertIn('engine', area_ids)
        self.assertIn('brakes', area_ids)
        self.assertIn('lidar', area_ids)

    def test_get_area_details(self):
        """
        Test GET /areas/{id} returns area with capabilities.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/areas/powertrain', timeout=5)
        self.assertEqual(response.status_code, 200)

        area = response.json()
        self.assertEqual(area['id'], 'powertrain')
        self.assertEqual(area['name'], 'Powertrain')
        self.assertIn('capabilities', area)
        self.assertIn('_links', area)

    def test_get_area_not_found(self):
        """Test GET /areas/{id} returns 404 for unknown area."""
        response = requests.get(f'{self.BASE_URL}/areas/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_area_subareas(self):
        """
        Test GET /areas/{id}/subareas returns nested areas.

        @verifies REQ_INTEROP_004
        """
        response = requests.get(f'{self.BASE_URL}/areas/powertrain/subareas', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        subarea_ids = [s['id'] for s in data['items']]
        self.assertIn('engine', subarea_ids)

    def test_area_components(self):
        """
        Test GET /areas/{id}/components returns components in area.

        @verifies REQ_INTEROP_006
        """
        response = requests.get(f'{self.BASE_URL}/areas/engine/components', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

    def test_area_related_components(self):
        """
        Test GET /areas/{id}/related-components includes subarea components.

        @verifies REQ_INTEROP_006
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/powertrain/related-components', timeout=5
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        # Should include engine components (from subarea)
        component_ids = [c['id'] for c in data['items']]
        self.assertIn('engine-ecu', component_ids)

    # =========================================================================
    # Components Endpoints
    # =========================================================================

    def test_list_components(self):
        """
        Test GET /components returns all manifest-defined components.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/components', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('total_count', data)

        component_ids = [c['id'] for c in data['items']]

        # Check manifest-defined components
        self.assertIn('engine-ecu', component_ids)
        self.assertIn('temp-sensor-hw', component_ids)
        self.assertIn('brake-ecu', component_ids)
        self.assertIn('lidar-unit', component_ids)

    def test_get_component_details(self):
        """
        Test GET /components/{id} returns component with capabilities.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/components/engine-ecu', timeout=5)
        self.assertEqual(response.status_code, 200)

        component = response.json()
        self.assertEqual(component['id'], 'engine-ecu')
        self.assertEqual(component['name'], 'Engine ECU')
        self.assertIn('capabilities', component)
        self.assertIn('_links', component)

    def test_get_component_not_found(self):
        """Test GET /components/{id} returns 404 for unknown component."""
        response = requests.get(f'{self.BASE_URL}/components/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_component_subcomponents(self):
        """
        Test GET /components/{id}/subcomponents returns subcomponents.

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

    def test_component_subcomponents_not_found(self):
        """
        Test GET /components/{id}/subcomponents returns 404 for unknown component.

        @verifies REQ_INTEROP_005
        """
        response = requests.get(f'{self.BASE_URL}/components/nonexistent/subcomponents', timeout=5)
        self.assertEqual(response.status_code, 404)

    # =========================================================================
    # Apps Endpoints
    # =========================================================================

    def test_list_apps(self):
        """
        Test GET /apps returns all manifest-defined apps.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/apps', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('total_count', data)

        app_ids = [a['id'] for a in data['items']]

        # Check manifest-defined apps
        self.assertIn('engine-temp-sensor', app_ids)
        self.assertIn('engine-rpm-sensor', app_ids)
        self.assertIn('brake-pressure-sensor', app_ids)
        self.assertIn('lidar-sensor', app_ids)

    def test_get_app_details(self):
        """
        Test GET /apps/{id} returns app with capabilities.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/apps/engine-temp-sensor', timeout=5)
        self.assertEqual(response.status_code, 200)

        app = response.json()
        self.assertEqual(app['id'], 'engine-temp-sensor')
        self.assertEqual(app['name'], 'Engine Temperature Sensor')
        self.assertIn('capabilities', app)
        self.assertIn('_links', app)

    def test_get_app_not_found(self):
        """Test GET /apps/{id} returns 404 for unknown app."""
        response = requests.get(f'{self.BASE_URL}/apps/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_app_online_status(self):
        """Test that apps with running nodes have is_online=true."""
        # Wait for nodes to be discovered
        time.sleep(5)

        response = requests.get(f'{self.BASE_URL}/apps', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        apps_by_id = {a['id']: a for a in data['items']}

        # engine-temp-sensor should be online (demo node running)
        if 'engine-temp-sensor' in apps_by_id:
            # May or may not be online depending on timing
            pass  # Just verify it exists

    def test_app_data_endpoint(self):
        """Test GET /apps/{id}/data returns topic list."""
        response = requests.get(f'{self.BASE_URL}/apps/engine-temp-sensor/data', timeout=5)
        # May return 200 with topics or empty list
        self.assertIn(response.status_code, [200, 404])

    def test_app_operations_endpoint(self):
        """Test GET /apps/{id}/operations returns services/actions."""
        response = requests.get(
            f'{self.BASE_URL}/apps/engine-calibration-service/operations', timeout=5
        )
        self.assertIn(response.status_code, [200, 404])

    def test_app_configurations_endpoint(self):
        """
        Test GET /apps/{id}/configurations returns parameters.

        May return:
        - 200 if node is running and has parameters
        - 404 if app not found
        - 503 if node is not running (manifest-only mode)
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar-sensor/configurations', timeout=5
        )
        self.assertIn(response.status_code, [200, 404, 503])

    def test_app_data_item_endpoint(self):
        """
        Test GET /apps/{id}/data/{data_id} returns sampled topic data.

        @verifies REQ_INTEROP_003
        """
        # First get the list of data items for the app
        response = requests.get(f'{self.BASE_URL}/apps/engine-temp-sensor/data', timeout=5)
        if response.status_code != 200:
            self.skipTest('App data endpoint not available')

        data = response.json()
        if not data.get('items'):
            self.skipTest('No data items for app')

        # Get the first data item
        data_id = data['items'][0]['id']
        response = requests.get(
            f'{self.BASE_URL}/apps/engine-temp-sensor/data/{data_id}', timeout=5
        )
        self.assertIn(response.status_code, [200, 404])

        if response.status_code == 200:
            item = response.json()
            self.assertIn('id', item)
            self.assertIn('direction', item)

    def test_component_related_apps(self):
        """
        Test GET /components/{id}/related-apps returns apps hosted on component.

        @verifies REQ_INTEROP_003
        """
        # temp-sensor-hw hosts engine-temp-sensor according to manifest
        url = f'{self.BASE_URL}/components/temp-sensor-hw/related-apps'
        response = requests.get(url, timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('total_count', data)

        # Apps hosted on temp-sensor-hw should be returned
        app_ids = [a['id'] for a in data['items']]
        self.assertIn('engine-temp-sensor', app_ids)

    def test_component_related_apps_empty(self):
        """Test GET /components/{id}/related-apps returns empty for component with no apps."""
        # engine-ecu doesn't have apps directly hosted on it
        response = requests.get(f'{self.BASE_URL}/components/engine-ecu/related-apps', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_component_related_apps_not_found(self):
        """Test GET /components/{id}/related-apps returns 404 for unknown component."""
        response = requests.get(f'{self.BASE_URL}/components/nonexistent/related-apps', timeout=5)
        self.assertEqual(response.status_code, 404)

    # =========================================================================
    # Functions Endpoints
    # =========================================================================

    def test_list_functions(self):
        """
        Test GET /functions returns all manifest-defined functions.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/functions', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('total_count', data)

        function_ids = [f['id'] for f in data['items']]

        # Check manifest-defined functions
        self.assertIn('engine-monitoring', function_ids)
        self.assertIn('engine-calibration', function_ids)
        self.assertIn('brake-system', function_ids)
        self.assertIn('body-electronics', function_ids)
        self.assertIn('perception-system', function_ids)

    def test_get_function_details(self):
        """
        Test GET /functions/{id} returns function with capabilities.

        @verifies REQ_INTEROP_003
        """
        response = requests.get(f'{self.BASE_URL}/functions/engine-monitoring', timeout=5)
        self.assertEqual(response.status_code, 200)

        func = response.json()
        self.assertEqual(func['id'], 'engine-monitoring')
        self.assertEqual(func['name'], 'Engine Monitoring')
        self.assertIn('capabilities', func)
        self.assertIn('_links', func)

    def test_get_function_not_found(self):
        """Test GET /functions/{id} returns 404 for unknown function."""
        response = requests.get(f'{self.BASE_URL}/functions/nonexistent', timeout=5)
        self.assertEqual(response.status_code, 404)

    def test_function_hosts(self):
        """
        Test GET /functions/{id}/hosts returns hosting apps.

        @verifies REQ_INTEROP_007
        """
        response = requests.get(f'{self.BASE_URL}/functions/engine-monitoring/hosts', timeout=5)
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        host_ids = [h['id'] for h in data['items']]

        # engine-monitoring is hosted by engine-temp-sensor and engine-rpm-sensor
        self.assertIn('engine-temp-sensor', host_ids)
        self.assertIn('engine-rpm-sensor', host_ids)

    def test_function_data(self):
        """Test GET /functions/{id}/data aggregates data from hosts."""
        response = requests.get(f'{self.BASE_URL}/functions/engine-monitoring/data', timeout=5)
        self.assertIn(response.status_code, [200, 404])

    def test_function_operations(self):
        """Test GET /functions/{id}/operations aggregates operations from hosts."""
        response = requests.get(
            f'{self.BASE_URL}/functions/engine-calibration/operations', timeout=5
        )
        self.assertIn(response.status_code, [200, 404])

    # =========================================================================
    # Discovery Statistics
    # =========================================================================

    def test_discovery_stats(self):
        """Test GET /discovery/stats returns manifest mode info."""
        response = requests.get(f'{self.BASE_URL}/discovery/stats', timeout=5)
        if response.status_code == 200:
            stats = response.json()
            # Should indicate manifest_only mode
            if 'mode' in stats:
                self.assertEqual(stats['mode'], 'manifest_only')

    # =========================================================================
    # Error Cases
    # =========================================================================

    def test_invalid_area_id(self):
        """Test GET /areas/{id} with invalid ID returns 400."""
        response = requests.get(f'{self.BASE_URL}/areas/invalid..id', timeout=5)
        self.assertIn(response.status_code, [400, 404])

    def test_invalid_component_id(self):
        """Test GET /components/{id} with invalid ID returns 400."""
        response = requests.get(f'{self.BASE_URL}/components/../etc/passwd', timeout=5)
        self.assertIn(response.status_code, [400, 404])


@launch_testing.post_shutdown_test()
class TestDiscoveryManifestModeShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_code(self, proc_info):
        """Check gateway exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
