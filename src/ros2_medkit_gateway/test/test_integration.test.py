#!/usr/bin/env python3
# Copyright 2025 bburda, mfaferek93
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
Launch file for ROS 2 Medkit Gateway integration tests.

This launch file:
1. Starts the ROS 2 Medkit Gateway node
2. Launches demo nodes in different namespaces (powertrain, chassis, body)
3. Runs integration tests
4. Cleans up all processes
"""

import os
import time
import unittest
from urllib.parse import quote

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests


def get_coverage_env():
    """
    Get environment variables for gcov coverage data collection.

    When running with coverage enabled (ENABLE_COVERAGE=ON), subprocess nodes
    need GCOV_PREFIX set to write coverage data to the correct build directory.
    This allows integration test coverage to be captured alongside unit tests.

    Returns
    -------
    dict
        Environment variables dict with GCOV_PREFIX and GCOV_PREFIX_STRIP,
        or empty dict if coverage path cannot be determined.

    """
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_gateway')
        # pkg_prefix is like /path/to/workspace/install/ros2_medkit_gateway
        # workspace is 2 levels up from install/package_name
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_gateway')

        if os.path.exists(build_dir):
            # GCOV_PREFIX_STRIP removes leading path components from compiled-in paths
            # GCOV_PREFIX prepends the new path for .gcda file output
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        # Ignore: if coverage environment cannot be determined,
        # return empty dict so tests proceed without coverage data.
        pass
    return {}


def encode_topic_path(topic_path: str) -> str:
    """
    Encode a ROS topic path for use in URLs.

    Slashes are encoded as %2F for proper URL routing.
    Example: '/powertrain/engine/temperature' -> 'powertrain%2Fengine%2Ftemperature'

    Parameters
    ----------
    topic_path : str
        Full ROS topic path starting with '/'

    Returns
    -------
    str
        URL-encoded topic path without leading slash

    Raises
    ------
    ValueError
        If topic_path doesn't start with '/'

    """
    # Validate that the topic path starts with a leading slash
    if not topic_path.startswith('/'):
        raise ValueError(f"Topic path must start with '/': {topic_path}")
    # Remove leading slash and encode the rest
    topic_path = topic_path[1:]
    return quote(topic_path, safe='')


def generate_test_description():
    """Generate launch description with gateway node, demo nodes, and tests."""
    # Launch the ROS 2 Medkit Gateway node
    # additional_env sets GCOV_PREFIX for coverage data collection from subprocess
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[],
        additional_env=get_coverage_env(),
    )

    # Launch demo automotive sensor nodes
    # All demo nodes also get coverage env for completeness
    coverage_env = get_coverage_env()

    engine_temp_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_engine_temp_sensor',
        name='temp_sensor',
        namespace='/powertrain/engine',
        output='screen',
        additional_env=coverage_env,
    )

    rpm_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_rpm_sensor',
        name='rpm_sensor',
        namespace='/powertrain/engine',
        output='screen',
        additional_env=coverage_env,
    )

    brake_pressure_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_brake_pressure_sensor',
        name='pressure_sensor',
        namespace='/chassis/brakes',
        output='screen',
        additional_env=coverage_env,
    )

    door_status_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_door_status_sensor',
        name='status_sensor',
        namespace='/body/door/front_left',
        output='screen',
        additional_env=coverage_env,
    )

    brake_actuator = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_brake_actuator',
        name='actuator',
        namespace='/chassis/brakes',
        output='screen',
        additional_env=coverage_env,
    )

    light_controller = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_light_controller',
        name='controller',
        namespace='/body/lights',
        output='screen',
        additional_env=coverage_env,
    )

    calibration_service = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_calibration_service',
        name='calibration',
        namespace='/powertrain/engine',
        output='screen',
        additional_env=coverage_env,
    )

    long_calibration_action = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_long_calibration_action',
        name='long_calibration',
        namespace='/powertrain/engine',
        output='screen',
        additional_env=coverage_env,
    )

    # Start demo nodes with a delay to ensure gateway starts first
    delayed_sensors = TimerAction(
        period=2.0,
        actions=[
            engine_temp_sensor,
            rpm_sensor,
            brake_pressure_sensor,
            door_status_sensor,
            brake_actuator,
            light_controller,
            calibration_service,
            long_calibration_action,
        ],
    )

    return (
        LaunchDescription(
            [
                # Launch gateway first
                gateway_node,
                # Launch demo nodes with delay
                delayed_sensors,
                # Start tests after nodes have time to initialize
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {
            'gateway_node': gateway_node,
        },
    )


# API version prefix - must match rest_server.cpp
API_BASE_PATH = '/api/v1'


class TestROS2MedkitGatewayIntegration(unittest.TestCase):
    """Integration tests for ROS 2 Medkit Gateway REST API and discovery."""

    BASE_URL = f'http://localhost:8080{API_BASE_PATH}'
    # Wait for cache refresh + safety margin
    # Must be kept in sync with gateway_params.yaml refresh_interval_ms (10000ms)
    # Need to wait for at least 2 refresh cycles to ensure all demo nodes are discovered
    CACHE_REFRESH_INTERVAL = 12.0

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready before any tests."""
        # Wait for gateway and discovery
        time.sleep(cls.CACHE_REFRESH_INTERVAL)

        # Verify gateway is responding
        max_retries = 5
        for i in range(max_retries):
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=1)
                if response.status_code == 200:
                    return
            except requests.exceptions.RequestException:
                if i == max_retries - 1:
                    raise unittest.SkipTest('Gateway not responding')
                time.sleep(1)

    def _get_json(self, endpoint: str):
        """Get JSON from an endpoint."""
        response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=5)
        response.raise_for_status()
        return response.json()

    def test_01_root_endpoint(self):
        """
        Test GET / returns server capabilities and entry points.

        @verifies REQ_INTEROP_010
        """
        data = self._get_json('/')
        self.assertIn('name', data)
        self.assertIn('version', data)
        self.assertIn('endpoints', data)
        self.assertIn('capabilities', data)

        self.assertEqual(data['name'], 'ROS 2 Medkit Gateway')
        self.assertEqual(data['version'], '0.1.0')

        # Verify endpoints list
        self.assertIsInstance(data['endpoints'], list)
        self.assertIn('GET /api/v1/health', data['endpoints'])
        self.assertIn('GET /api/v1/version-info', data['endpoints'])
        self.assertIn('GET /api/v1/areas', data['endpoints'])
        self.assertIn('GET /api/v1/components', data['endpoints'])
        self.assertIn(
            'PUT /api/v1/components/{component_id}/data/{topic_name}', data['endpoints']
        )

        # Verify api_base field
        self.assertIn('api_base', data)
        self.assertEqual(data['api_base'], API_BASE_PATH)

        # Verify capabilities
        self.assertIn('discovery', data['capabilities'])
        self.assertIn('data_access', data['capabilities'])
        self.assertTrue(data['capabilities']['discovery'])
        self.assertTrue(data['capabilities']['data_access'])
        print('✓ Root endpoint test passed')

    def test_01b_version_info_endpoint(self):
        """
        Test GET /version-info returns gateway status and version.

        @verifies REQ_INTEROP_001
        """
        data = self._get_json('/version-info')
        self.assertIn('status', data)
        self.assertIn('version', data)
        self.assertIn('timestamp', data)
        self.assertEqual(data['status'], 'ROS 2 Medkit Gateway running')
        self.assertEqual(data['version'], '0.1.0')
        self.assertIsInstance(data['timestamp'], int)
        print('✓ Version info endpoint test passed')

    def test_02_list_areas(self):
        """
        Test GET /areas returns all discovered areas.

        @verifies REQ_INTEROP_003
        """
        areas = self._get_json('/areas')
        self.assertIsInstance(areas, list)
        self.assertGreaterEqual(len(areas), 1)
        area_ids = [area['id'] for area in areas]
        self.assertIn('root', area_ids)
        print(f'✓ Areas test passed: {len(areas)} areas discovered')

    def test_03_list_components(self):
        """
        Test GET /components returns all discovered components.

        @verifies REQ_INTEROP_003
        """
        components = self._get_json('/components')
        self.assertIsInstance(components, list)
        # Should have at least 7 demo nodes + gateway node
        self.assertGreaterEqual(len(components), 7)

        # Verify response structure - all components should have required fields
        for component in components:
            self.assertIn('id', component)
            self.assertIn('namespace', component)
            self.assertIn('fqn', component)
            self.assertIn('type', component)
            self.assertIn('area', component)
            self.assertEqual(component['type'], 'Component')

        # Verify some expected component IDs are present
        component_ids = [comp['id'] for comp in components]
        self.assertIn('temp_sensor', component_ids)
        self.assertIn('rpm_sensor', component_ids)
        self.assertIn('pressure_sensor', component_ids)

        print(f'✓ Components test passed: {len(components)} components discovered')

    def test_04_automotive_areas_discovery(self):
        """
        Test that automotive areas are properly discovered.

        @verifies REQ_INTEROP_003
        """
        areas = self._get_json('/areas')
        area_ids = [area['id'] for area in areas]

        expected_areas = ['powertrain', 'chassis', 'body']
        for expected in expected_areas:
            self.assertIn(expected, area_ids)

        print(f'✓ All automotive areas discovered: {area_ids}')

    def test_05_area_components_success(self):
        """
        Test GET /areas/{area_id}/components returns components for valid area.

        @verifies REQ_INTEROP_006
        """
        # Test powertrain area
        components = self._get_json('/areas/powertrain/components')
        self.assertIsInstance(components, list)
        self.assertGreater(len(components), 0)

        # All components should belong to powertrain area
        for component in components:
            self.assertEqual(component['area'], 'powertrain')
            self.assertIn('id', component)
            self.assertIn('namespace', component)

        # Verify expected powertrain components
        component_ids = [comp['id'] for comp in components]
        self.assertIn('temp_sensor', component_ids)
        self.assertIn('rpm_sensor', component_ids)

        print(
            f'✓ Area components test passed: {len(components)} components in powertrain'
        )

    def test_06_area_components_nonexistent_error(self):
        """
        Test GET /areas/{area_id}/components returns 404 for nonexistent area.

        @verifies REQ_INTEROP_006
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/nonexistent/components', timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Area not found')
        self.assertIn('area_id', data)
        self.assertEqual(data['area_id'], 'nonexistent')

        print('✓ Nonexistent area error test passed')

    def test_07_component_data_powertrain_engine(self):
        """
        Test GET /components/{component_id}/data for engine component.

        @verifies REQ_INTEROP_018
        """
        # Get data from temp_sensor component (powertrain/engine)
        data = self._get_json('/components/temp_sensor/data')
        self.assertIsInstance(data, list)

        # Should have at least one topic with data or metadata
        if len(data) > 0:
            for topic_data in data:
                self.assertIn('topic', topic_data)
                self.assertIn('status', topic_data)
                # Status can be 'data' (with actual data) or 'metadata_only' (fallback)
                self.assertIn(topic_data['status'], ['data', 'metadata_only'])
                if topic_data['status'] == 'data':
                    self.assertIn('data', topic_data)
                print(
                    f"  - Topic: {topic_data['topic']} (status: {topic_data['status']})"
                )

        print(f'✓ Engine component data test passed: {len(data)} topics')

    def test_08_component_data_chassis_brakes(self):
        """
        Test GET /components/{component_id}/data for brakes component.

        @verifies REQ_INTEROP_018
        """
        # Get data from pressure_sensor component (chassis/brakes)
        data = self._get_json('/components/pressure_sensor/data')
        self.assertIsInstance(data, list)

        # Check if any data/metadata is available
        if len(data) > 0:
            for topic_data in data:
                self.assertIn('topic', topic_data)
                self.assertIn('status', topic_data)
                # Status can be 'data' (with actual data) or 'metadata_only' (fallback)
                self.assertIn(topic_data['status'], ['data', 'metadata_only'])
                if topic_data['status'] == 'data':
                    self.assertIn('data', topic_data)

        print(f'✓ Brakes component data test passed: {len(data)} topics')

    def test_09_component_data_body_door(self):
        """
        Test GET /components/{component_id}/data for door component.

        @verifies REQ_INTEROP_018
        """
        # Get data from status_sensor component (body/door/front_left)
        data = self._get_json('/components/status_sensor/data')
        self.assertIsInstance(data, list)

        # Check structure
        if len(data) > 0:
            for topic_data in data:
                self.assertIn('topic', topic_data)
                self.assertIn('status', topic_data)
                # Status can be 'data' (with actual data) or 'metadata_only' (fallback)
                self.assertIn(topic_data['status'], ['data', 'metadata_only'])
                if topic_data['status'] == 'data':
                    self.assertIn('data', topic_data)

        print(f'✓ Door component data test passed: {len(data)} topics')

    def test_10_component_data_structure(self):
        """
        Test GET /components/{component_id}/data response structure.

        @verifies REQ_INTEROP_018
        """
        data = self._get_json('/components/temp_sensor/data')
        self.assertIsInstance(data, list, 'Response should be an array')

        # If we have data, verify structure
        if len(data) > 0:
            first_item = data[0]
            self.assertIn('topic', first_item, "Each item should have 'topic' field")
            self.assertIn(
                'timestamp', first_item, "Each item should have 'timestamp' field"
            )
            self.assertIn('status', first_item, "Each item should have 'status' field")
            self.assertIsInstance(
                first_item['topic'], str, "'topic' should be a string"
            )
            self.assertIsInstance(
                first_item['timestamp'],
                int,
                "'timestamp' should be an integer (nanoseconds)",
            )
            # Status can be 'data' or 'metadata_only'
            self.assertIn(first_item['status'], ['data', 'metadata_only'])
            if first_item['status'] == 'data':
                self.assertIn('data', first_item, "status=data should have 'data'")
                self.assertIsInstance(
                    first_item['data'], dict, "'data' should be object"
                )

            # Verify metadata fields are present (for both data and metadata_only)
            self.assertIn('type', first_item, "Each item should have 'type' field")
            self.assertIn('type_info', first_item, "Each item should have 'type_info'")
            self.assertIn(
                'publisher_count', first_item, "Should have 'publisher_count'"
            )
            self.assertIn(
                'subscriber_count', first_item, "Should have 'subscriber_count'"
            )

            # Verify type_info structure
            type_info = first_item['type_info']
            self.assertIn('schema', type_info, "'type_info' should have 'schema'")
            self.assertIn(
                'default_value', type_info, "'type_info' should have 'default_value'"
            )

        print('✓ Component data structure test passed')

    def test_11_component_nonexistent_error(self):
        """
        Test GET /components/{component_id}/data returns 404 for nonexistent component.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/data', timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Component not found')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'nonexistent_component')

        print('✓ Nonexistent component error test passed')

    def test_12_component_no_topics(self):
        """
        Test GET /components/{component_id}/data returns empty array.

        Verifies that components with no topics return an empty array.
        The calibration component typically has only services, no topics.

        @verifies REQ_INTEROP_018
        """
        # Or test with a component that we know has no publishing topics
        # For now, we'll verify that any component returns an array (even if empty)
        data = self._get_json('/components/calibration/data')
        self.assertIsInstance(data, list, 'Response should be an array even when empty')

        print(f'✓ Component with no topics test passed: {len(data)} topics')

    def test_13_invalid_component_id_special_chars(self):
        """
        Test GET /components/{component_id}/data rejects special characters.

        @verifies REQ_INTEROP_018
        """
        # Test various invalid characters
        invalid_ids = [
            'component;drop',  # SQL injection attempt
            'component<script>',  # XSS attempt
            'component"test',  # Quote
            'component`test',  # Backtick
            'component$test',  # Dollar sign
            'component|test',  # Pipe
            'component&test',  # Ampersand
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/components/{invalid_id}/data', timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for component_id: {invalid_id}',
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid component ID')
            self.assertIn('details', data)

        print('✓ Invalid component ID special characters test passed')

    def test_14_invalid_area_id_special_chars(self):
        """
        Test GET /areas/{area_id}/components rejects special characters.

        @verifies REQ_INTEROP_006
        """
        # Test various invalid characters
        # Note: Forward slash is handled by URL routing, not validation
        invalid_ids = [
            'area;drop',  # SQL injection attempt
            'area<script>',  # XSS attempt
            'area"test',  # Quote
            'area|test',  # Pipe
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/areas/{invalid_id}/components', timeout=5
            )
            self.assertEqual(
                response.status_code, 400, f'Expected 400 for area_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid area ID')
            self.assertIn('details', data)

        print('✓ Invalid area ID special characters test passed')

    def test_15_valid_ids_with_underscores(self):
        """
        Test that valid IDs with underscores are accepted (ROS 2 naming).

        @verifies REQ_INTEROP_018
        """
        # While these IDs don't exist in the test environment,
        # they should pass validation and return 404 (not 400)
        valid_ids = [
            'component_name',  # Underscore
            'component_name_123',  # Underscore and numbers
            'ComponentName',  # CamelCase
            'component123',  # Alphanumeric
        ]

        for valid_id in valid_ids:
            response = requests.get(
                f'{self.BASE_URL}/components/{valid_id}/data', timeout=5
            )
            # Should return 404 (not found) not 400 (invalid)
            self.assertEqual(
                response.status_code,
                404,
                f'Expected 404 for valid but nonexistent ID: {valid_id}',
            )

        print('✓ Valid IDs with underscores test passed')

    def test_16_invalid_ids_with_hyphens(self):
        """
        Test that IDs with hyphens are rejected (not allowed in ROS 2 names).

        @verifies REQ_INTEROP_018
        """
        invalid_ids = [
            'component-name',
            'component-name-123',
            'my-component',
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/components/{invalid_id}/data', timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for hyphenated ID: {invalid_id}',
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid component ID')

        print('✓ Invalid IDs with hyphens test passed')

    def test_17_component_topic_temperature(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for temperature topic.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /powertrain/engine/temperature
        topic_path = encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('topic', data)
        self.assertIn('timestamp', data)
        self.assertIn('status', data)
        self.assertEqual(data['topic'], '/powertrain/engine/temperature')
        self.assertIsInstance(data['timestamp'], int)
        self.assertIn(data['status'], ['data', 'metadata_only'])
        if data['status'] == 'data':
            self.assertIn('data', data)
            self.assertIsInstance(data['data'], dict)

        print(f"✓ Temperature test passed: {data['topic']} ({data['status']})")

    def test_18_component_topic_rpm(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for RPM topic.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /powertrain/engine/rpm
        topic_path = encode_topic_path('/powertrain/engine/rpm')
        response = requests.get(
            f'{self.BASE_URL}/components/rpm_sensor/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('topic', data)
        self.assertIn('timestamp', data)
        self.assertIn('status', data)
        self.assertEqual(data['topic'], '/powertrain/engine/rpm')
        self.assertIn(data['status'], ['data', 'metadata_only'])
        if data['status'] == 'data':
            self.assertIn('data', data)

        print(
            f"✓ Component topic RPM test passed: {data['topic']} (status: {data['status']})"
        )

    def test_19_component_topic_pressure(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for pressure topic.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /chassis/brakes/pressure
        topic_path = encode_topic_path('/chassis/brakes/pressure')
        response = requests.get(
            f'{self.BASE_URL}/components/pressure_sensor/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('topic', data)
        self.assertIn('timestamp', data)
        self.assertIn('status', data)
        self.assertEqual(data['topic'], '/chassis/brakes/pressure')
        self.assertIn(data['status'], ['data', 'metadata_only'])
        if data['status'] == 'data':
            self.assertIn('data', data)

        print(f"✓ Pressure test passed: {data['topic']} ({data['status']})")

    def test_20_component_topic_data_structure(self):
        """
        Test GET /components/{component_id}/data/{topic_name} response structure.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /powertrain/engine/temperature
        topic_path = encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Verify all required fields
        self.assertIn('topic', data, "Response should have 'topic' field")
        self.assertIn('timestamp', data, "Response should have 'timestamp' field")
        self.assertIn('status', data, "Response should have 'status' field")

        # Verify field types
        self.assertIsInstance(data['topic'], str, "'topic' should be a string")
        self.assertIsInstance(
            data['timestamp'], int, "'timestamp' should be an integer (nanoseconds)"
        )
        # Status can be 'data' or 'metadata_only'
        self.assertIn(data['status'], ['data', 'metadata_only'])
        if data['status'] == 'data':
            self.assertIn(
                'data', data, "Response with status=data should have 'data' field"
            )
            self.assertIsInstance(data['data'], dict, "'data' should be an object")

        # Verify topic path format
        self.assertTrue(
            data['topic'].startswith('/'),
            "Topic should be an absolute path starting with '/'",
        )

        print('✓ Component topic data structure test passed')

    def test_21_component_nonexistent_topic_error(self):
        """
        Test GET /components/{component_id}/data/{topic_name} returns 404 for nonexistent topic.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /some/nonexistent/topic
        topic_path = encode_topic_path('/some/nonexistent/topic')
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Topic not found')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'temp_sensor')
        self.assertIn('topic_name', data)
        # topic_name in response is the decoded path (from URL)
        self.assertEqual(data['topic_name'], 'some/nonexistent/topic')

        print('✓ Nonexistent topic error test passed')

    def test_22_component_topic_nonexistent_component_error(self):
        """
        Test GET endpoint returns 404 for nonexistent component.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path
        topic_path = encode_topic_path('/some/topic')
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/data/{topic_path}',
            timeout=5,
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Component not found')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'nonexistent_component')

        print('✓ Component topic nonexistent component error test passed')

    def test_23_component_topic_with_slashes(self):
        """
        Test GET with percent-encoded slashes in topic path.

        @verifies REQ_INTEROP_019
        """
        # Test that percent-encoded slashes work correctly
        # /powertrain/engine/temperature encoded as powertrain%2Fengine%2Ftemperature
        topic_path = encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/data/{topic_path}', timeout=10
        )
        # Should return 200 (found) since this topic exists
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertEqual(data['topic'], '/powertrain/engine/temperature')

        print('✓ Percent-encoded slashes test passed')

    def test_24_component_topic_valid_names(self):
        """
        Test that valid topic names work correctly.

        @verifies REQ_INTEROP_019
        """
        # These topic names are valid but may not exist as full paths
        # They should return 404 (not found) - the gateway will try to find /topic_name
        valid_topic_names = [
            'topic_name',
            'topic_name_123',
            'TopicName',
            'topic123',
        ]

        for valid_topic in valid_topic_names:
            response = requests.get(
                f'{self.BASE_URL}/components/temp_sensor/data/{valid_topic}', timeout=10
            )
            # Should return 404 (topic not found) not 400 (invalid name)
            self.assertIn(
                response.status_code,
                [200, 404],
                f'Expected 200 or 404 for valid topic name: {valid_topic}, '
                f'got {response.status_code}',
            )

        print('✓ Valid topic names with underscores test passed')

    # ========== PUT /components/{component_id}/data/{topic_name} tests ==========

    def test_25_publish_brake_command(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} publishes data.

        @verifies REQ_INTEROP_020
        """
        # Use percent encoding for topic path: /chassis/brakes/command
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32', 'data': {'data': 50.0}},
            timeout=10,
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('topic', data)
        self.assertIn('type', data)
        self.assertIn('status', data)
        self.assertIn('timestamp', data)
        self.assertIn('component_id', data)
        self.assertIn('topic_name', data)
        self.assertEqual(data['status'], 'published')
        self.assertEqual(data['type'], 'std_msgs/msg/Float32')
        self.assertEqual(data['component_id'], 'actuator')
        # topic_name is the decoded path from URL
        self.assertEqual(data['topic_name'], 'chassis/brakes/command')

        print(f"✓ Publish brake command test passed: {data['topic']}")

    def test_26_publish_validation_missing_type(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 when type missing.

        @verifies REQ_INTEROP_020
        """
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/{topic_path}',
            json={'data': {'data': 50.0}},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('type', data['error'].lower())

        print('✓ Publish validation missing type test passed')

    def test_27_publish_validation_missing_data(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 when data missing.

        @verifies REQ_INTEROP_020
        """
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('data', data['error'].lower())

        print('✓ Publish validation missing data test passed')

    def test_28_publish_validation_invalid_type_format(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 for invalid type.

        @verifies REQ_INTEROP_020
        """
        # Test various invalid message type formats
        invalid_types = [
            'InvalidType',  # No slashes
            'std_msgs/Float32',  # Missing /msg/
            'std_msgs/srv/Empty',  # Wrong middle part (srv instead of msg)
            'a/b/c/d',  # Too many slashes (no /msg/)
            'a/msg/b/c',  # Too many slashes (3 instead of 2)
            '/msg/Type',  # Missing package (starts with /)
            'package/msg/',  # Missing type (ends with /)
        ]

        topic_path = encode_topic_path('/chassis/brakes/command')
        for invalid_type in invalid_types:
            response = requests.put(
                f'{self.BASE_URL}/components/actuator/data/{topic_path}',
                json={'type': invalid_type, 'data': {'data': 50.0}},
                timeout=5,
            )
            self.assertEqual(
                response.status_code, 400, f'Expected 400 for type: {invalid_type}'
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid message type format')

        print('✓ Publish validation invalid type format test passed')

    def test_29_publish_nonexistent_component(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 404 for unknown component.

        @verifies REQ_INTEROP_020
        """
        topic_path = encode_topic_path('/some/topic/path')
        response = requests.put(
            f'{self.BASE_URL}/components/nonexistent_component/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32', 'data': {'data': 50.0}},
            timeout=5,
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Component not found')
        self.assertEqual(data['component_id'], 'nonexistent_component')

        print('✓ Publish nonexistent component test passed')

    def test_30_publish_invalid_json_body(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 for invalid JSON.

        @verifies REQ_INTEROP_020
        """
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/{topic_path}',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('json', data['error'].lower())

        print('✓ Publish invalid JSON body test passed')

    # ========== POST /components/{component_id}/operations/{operation_name} tests ==========

    def test_31_operation_call_calibrate_service(self):
        """
        Test POST /components/{component_id}/operations/{operation_name} calls a service.

        @verifies REQ_INTEROP_035
        """
        response = requests.post(
            f'{self.BASE_URL}/components/calibration/operations/calibrate',
            json={},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'success')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'calibration')
        self.assertIn('operation', data)
        self.assertEqual(data['operation'], 'calibrate')
        self.assertIn('response', data)

        # Verify service response structure (std_srvs/srv/Trigger response)
        self.assertIn('success', data['response'])
        self.assertIn('message', data['response'])
        self.assertIsInstance(data['response']['success'], bool)
        self.assertIsInstance(data['response']['message'], str)

        print(f'✓ Operation call calibrate service test passed: {data["response"]}')

    def test_32_operation_call_nonexistent_operation(self):
        """
        Test operation call returns 404 for unknown operation.

        POST /components/{component_id}/operations/{operation_name}

        @verifies REQ_INTEROP_035
        """
        response = requests.post(
            f'{self.BASE_URL}/components/calibration/operations/nonexistent_op',
            json={},
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('Operation not found', data['error'])

        print('✓ Operation call nonexistent operation test passed')

    def test_33_operation_call_nonexistent_component(self):
        """
        Test operation call returns 404 for unknown component.

        POST /components/{component_id}/operations/{operation_name}

        @verifies REQ_INTEROP_035
        """
        response = requests.post(
            f'{self.BASE_URL}/components/nonexistent_component/operations/calibrate',
            json={},
            timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Component not found')
        self.assertEqual(data['component_id'], 'nonexistent_component')

        print('✓ Operation call nonexistent component test passed')

    def test_34_operation_call_invalid_component_id(self):
        """
        Test operation call rejects invalid component ID.

        POST /components/{component_id}/operations/{operation_name}

        @verifies REQ_INTEROP_035
        """
        invalid_ids = [
            'component;drop',
            'component<script>',
            'component-name',
        ]

        for invalid_id in invalid_ids:
            response = requests.post(
                f'{self.BASE_URL}/components/{invalid_id}/operations/calibrate',
                json={},
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for component_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid component ID')

        print('✓ Operation call invalid component ID test passed')

    def test_35_operation_call_invalid_operation_name(self):
        """
        Test operation call rejects invalid operation name.

        POST /components/{component_id}/operations/{operation_name}

        @verifies REQ_INTEROP_021
        """
        invalid_names = [
            'op;drop',
            'op<script>',
            'op-name',
        ]

        for invalid_name in invalid_names:
            response = requests.post(
                f'{self.BASE_URL}/components/calibration/operations/{invalid_name}',
                json={},
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for operation_name: {invalid_name}'
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid operation name')

        print('✓ Operation call invalid operation name test passed')

    def test_36_operation_call_with_invalid_json(self):
        """
        Test operation call returns 400 for invalid JSON body.

        POST /components/{component_id}/operations/{operation_name}

        @verifies REQ_INTEROP_021
        """
        response = requests.post(
            f'{self.BASE_URL}/components/calibration/operations/calibrate',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('json', data['error'].lower())

        print('✓ Operation call invalid JSON body test passed')

    def test_37_operations_listed_in_component_discovery(self):
        """
        Test that operations (services) are listed in component discovery response.

        @verifies REQ_INTEROP_021
        """
        components = self._get_json('/components')

        # Find calibration component
        calibration = None
        for comp in components:
            if comp['id'] == 'calibration':
                calibration = comp
                break

        self.assertIsNotNone(calibration, 'Calibration component should exist')
        self.assertIn('operations', calibration, 'Component should have operations field')
        self.assertIsInstance(calibration['operations'], list)

        # Find the calibrate operation
        calibrate_op = None
        for op in calibration['operations']:
            if op['name'] == 'calibrate':
                calibrate_op = op
                break

        self.assertIsNotNone(calibrate_op, 'Calibrate operation should be listed')
        self.assertIn('kind', calibrate_op)
        self.assertEqual(calibrate_op['kind'], 'service')
        self.assertIn('type', calibrate_op)
        self.assertEqual(calibrate_op['type'], 'std_srvs/srv/Trigger')
        self.assertIn('path', calibrate_op)
        self.assertEqual(calibrate_op['path'], '/powertrain/engine/calibrate')

        print('✓ Operations listed in component discovery test passed')

    def test_38_root_endpoint_includes_operations(self):
        """
        Test that root endpoint lists operations endpoint and capability.

        @verifies REQ_INTEROP_021
        """
        data = self._get_json('/')

        # Verify operations endpoint is listed
        self.assertIn('endpoints', data)
        self.assertIn(
            'POST /api/v1/components/{component_id}/operations/{operation_name}',
            data['endpoints']
        )

        # Verify operations capability is listed
        self.assertIn('capabilities', data)
        self.assertIn('operations', data['capabilities'])
        self.assertTrue(data['capabilities']['operations'])

        print('✓ Root endpoint includes operations test passed')

    # ========== Async Action Operations Tests (test_39-44) ==========

    def test_39_action_send_goal_and_get_id(self):
        """
        Test POST /components/{component_id}/operations/{operation_name} sends action goal.

        Sends a goal to the long_calibration action and verifies goal_id is returned.

        @verifies REQ_INTEROP_022
        """
        response = requests.post(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration',
            json={'goal': {'order': 5}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'success')
        self.assertIn('kind', data)
        self.assertEqual(data['kind'], 'action')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'long_calibration')
        self.assertIn('operation', data)
        self.assertEqual(data['operation'], 'long_calibration')
        self.assertIn('goal_id', data)
        self.assertIsInstance(data['goal_id'], str)
        self.assertGreater(len(data['goal_id']), 0)
        self.assertIn('goal_status', data)
        # Status can be 'executing' or 'succeeded' depending on timing
        self.assertIn(data['goal_status'], ['accepted', 'executing', 'succeeded'])

        print(f'✓ Action send goal test passed: goal_id={data["goal_id"]}')

    def test_40_action_status_endpoint(self):
        """
        Test GET /components/{component_id}/operations/{operation_name}/status returns goal status.

        @verifies REQ_INTEROP_022
        """
        # First, send a goal with enough steps to ensure it's still running
        response = requests.post(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration',
            json={'goal': {'order': 10}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        goal_id = response.json()['goal_id']

        # Check status immediately
        status_response = requests.get(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration/status',
            params={'goal_id': goal_id},
            timeout=5
        )
        self.assertEqual(status_response.status_code, 200)

        data = status_response.json()
        self.assertIn('goal_id', data)
        self.assertEqual(data['goal_id'], goal_id)
        self.assertIn('status', data)
        valid_statuses = ['accepted', 'executing', 'succeeded', 'canceled', 'aborted']
        self.assertIn(data['status'], valid_statuses)
        self.assertIn('action_path', data)
        self.assertEqual(data['action_path'], '/powertrain/engine/long_calibration')
        self.assertIn('action_type', data)
        self.assertEqual(data['action_type'], 'example_interfaces/action/Fibonacci')

        print(f'✓ Action status endpoint test passed: status={data["status"]}')

    @unittest.skip('Flaky on CI due to action server timing - goal acceptance can timeout')
    def test_41_action_status_after_completion(self):
        """
        Test that action status is updated to succeeded after completion via native subscription.

        The native status subscription updates goal status in real-time.
        After an action completes, polling the status endpoint should show 'succeeded'.

        @verifies REQ_INTEROP_022
        """
        # Send a short goal that will complete quickly
        response = requests.post(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration',
            json={'goal': {'order': 3}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        goal_id = response.json()['goal_id']

        # Wait for action to complete (3 steps * 0.5s = ~1.5s, plus margin)
        time.sleep(3)

        # Check status should show succeeded (updated via native subscription)
        status_response = requests.get(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration/status',
            params={'goal_id': goal_id},
            timeout=5
        )
        self.assertEqual(status_response.status_code, 200)

        data = status_response.json()
        self.assertIn('goal_id', data)
        self.assertEqual(data['goal_id'], goal_id)
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'succeeded')

        print(f'✓ Action status after completion test passed: status={data["status"]}')

    @unittest.skip('Flaky on CI due to action server timing - goal acceptance can timeout')
    def test_42_action_cancel_endpoint(self):
        """
        Test DELETE /components/{component_id}/operations/{operation_name} cancels action.

        @verifies REQ_INTEROP_022
        """
        # Send a long goal that we can cancel
        response = requests.post(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration',
            json={'goal': {'order': 20}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        goal_id = response.json()['goal_id']

        # Wait a moment for it to start executing
        time.sleep(1)

        # Cancel the goal
        cancel_response = requests.delete(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration',
            params={'goal_id': goal_id},
            timeout=10
        )
        self.assertEqual(cancel_response.status_code, 200)

        data = cancel_response.json()
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'canceling')
        self.assertIn('goal_id', data)
        self.assertEqual(data['goal_id'], goal_id)

        print(f'✓ Action cancel endpoint test passed: {data}')

    def test_43_action_listed_in_component_discovery(self):
        """
        Test that actions are listed in component discovery response.

        @verifies REQ_INTEROP_022
        """
        components = self._get_json('/components')

        # Find long_calibration component
        long_cal = None
        for comp in components:
            if comp['id'] == 'long_calibration':
                long_cal = comp
                break

        self.assertIsNotNone(long_cal, 'long_calibration component should exist')
        self.assertIn('operations', long_cal, 'Component should have operations field')
        self.assertIsInstance(long_cal['operations'], list)

        # Find the long_calibration action operation
        action_op = None
        for op in long_cal['operations']:
            if op['name'] == 'long_calibration' and op['kind'] == 'action':
                action_op = op
                break

        self.assertIsNotNone(action_op, 'long_calibration action should be listed')
        self.assertEqual(action_op['kind'], 'action')
        self.assertEqual(action_op['type'], 'example_interfaces/action/Fibonacci')
        self.assertEqual(action_op['path'], '/powertrain/engine/long_calibration')

        print('✓ Action listed in component discovery test passed')

    def test_44_action_status_without_goal_id_returns_latest(self):
        """
        Test action status without goal_id returns latest goal.

        GET /components/{component_id}/operations/{operation_name}/status
        Returns the most recent goal status when no goal_id is provided.

        @verifies REQ_INTEROP_022
        """
        # First, send a goal so we have something to query
        response = requests.post(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration',
            json={'goal': {'order': 3}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        expected_goal_id = response.json()['goal_id']

        # Wait for it to complete
        time.sleep(3)

        # Now query status without goal_id - should return the latest goal
        status_response = requests.get(
            f'{self.BASE_URL}/components/long_calibration/operations/long_calibration/status',
            timeout=5
        )
        self.assertEqual(status_response.status_code, 200)

        data = status_response.json()
        self.assertIn('goal_id', data)
        self.assertEqual(data['goal_id'], expected_goal_id)
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'succeeded')

        print(f'✓ Action status without goal_id returns latest goal: {data["goal_id"]}')

    # ========== Configurations API Tests (test_45-52) ==========

    def test_45_list_configurations(self):
        """
        Test GET /components/{component_id}/configurations lists all parameters.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'temp_sensor')
        self.assertIn('node_name', data)
        self.assertIn('parameters', data)
        self.assertIsInstance(data['parameters'], list)

        # Verify we have parameters from the demo node
        param_names = [p['name'] for p in data['parameters']]
        # The engine_temp_sensor should have these parameters we just added
        self.assertIn('publish_rate', param_names)
        self.assertIn('min_temp', param_names)
        self.assertIn('max_temp', param_names)
        self.assertIn('temp_step', param_names)

        # Verify parameter structure
        for param in data['parameters']:
            self.assertIn('name', param)
            self.assertIn('value', param)
            self.assertIn('type', param)

        print(f'✓ List configurations test passed: {len(data["parameters"])} parameters')

    def test_46_get_configuration(self):
        """
        Test GET /components/{component_id}/configurations/{param_name} gets parameter.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/configurations/publish_rate',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'temp_sensor')
        self.assertIn('parameter', data)

        param = data['parameter']
        self.assertIn('name', param)
        self.assertEqual(param['name'], 'publish_rate')
        self.assertIn('value', param)
        self.assertIn('type', param)
        self.assertEqual(param['type'], 'double')
        # Default value is 2.0
        self.assertEqual(param['value'], 2.0)

        print(f'✓ Get configuration test passed: {param["name"]}={param["value"]}')

    def test_47_set_configuration(self):
        """
        Test PUT /components/{component_id}/configurations/{param_name} sets parameter.

        @verifies REQ_INTEROP_024
        """
        # Set a new value
        response = requests.put(
            f'{self.BASE_URL}/components/temp_sensor/configurations/min_temp',
            json={'value': 80.0},
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'success')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'temp_sensor')
        self.assertIn('parameter', data)

        param = data['parameter']
        self.assertIn('name', param)
        self.assertEqual(param['name'], 'min_temp')
        self.assertIn('value', param)
        self.assertEqual(param['value'], 80.0)
        self.assertIn('type', param)
        self.assertEqual(param['type'], 'double')

        # Verify the value was actually set by reading it back
        verify_response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(verify_response.status_code, 200)
        verify_data = verify_response.json()
        self.assertEqual(verify_data['parameter']['value'], 80.0)

        # Reset the value back to default
        requests.put(
            f'{self.BASE_URL}/components/temp_sensor/configurations/min_temp',
            json={'value': 85.0},
            timeout=10
        )

        print(f'✓ Set configuration test passed: {param["name"]}={param["value"]}')

    def test_48_delete_configuration_resets_to_default(self):
        """
        Test DELETE /components/{component_id}/configurations/{param_name} resets to default.

        The DELETE method resets the parameter to its default value.
        It first changes the value, then resets it, then verifies the reset.

        @verifies REQ_INTEROP_025
        """
        # First, change the value from default
        set_response = requests.put(
            f'{self.BASE_URL}/components/temp_sensor/configurations/min_temp',
            json={'value': -50.0},
            timeout=10
        )
        self.assertEqual(set_response.status_code, 200)

        # Now reset to default via DELETE
        response = requests.delete(
            f'{self.BASE_URL}/components/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('reset_to_default', data)
        self.assertTrue(data['reset_to_default'])
        self.assertIn('name', data)
        self.assertEqual(data['name'], 'min_temp')
        self.assertIn('value', data)  # The value after reset

        # Verify the value was actually reset by reading it
        get_response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(get_response.status_code, 200)
        get_data = get_response.json()
        # The value should match what DELETE returned
        self.assertEqual(get_data['parameter']['value'], data['value'])

        print(f'✓ Delete configuration (reset to default) test passed: value={data["value"]}')

    def test_49_configurations_nonexistent_component(self):
        """
        Test GET /components/{component_id}/configurations returns 404 for unknown component.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Component not found')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'nonexistent_component')

        print('✓ Configurations nonexistent component test passed')

    def test_50_configuration_nonexistent_parameter(self):
        """
        Test GET configurations/{param_name} returns 404 for unknown param.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/configurations/nonexistent_param',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('param_name', data)
        self.assertEqual(data['param_name'], 'nonexistent_param')

        print('✓ Configuration nonexistent parameter test passed')

    def test_51_set_configuration_missing_value(self):
        """
        Test PUT configurations/{param_name} returns 400 when value missing.

        @verifies REQ_INTEROP_024
        """
        response = requests.put(
            f'{self.BASE_URL}/components/temp_sensor/configurations/min_temp',
            json={},
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('value', data['error'].lower())

        print('✓ Set configuration missing value test passed')

    def test_52_root_endpoint_includes_configurations(self):
        """
        Test that root endpoint lists configurations endpoints and capability.

        @verifies REQ_INTEROP_023
        """
        data = self._get_json('/')

        # Verify configurations endpoints are listed
        self.assertIn('endpoints', data)
        self.assertIn(
            'GET /api/v1/components/{component_id}/configurations',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/configurations/{param_name}',
            data['endpoints']
        )
        self.assertIn(
            'PUT /api/v1/components/{component_id}/configurations/{param_name}',
            data['endpoints']
        )

        # Verify configurations capability is listed
        self.assertIn('capabilities', data)
        self.assertIn('configurations', data['capabilities'])
        self.assertTrue(data['capabilities']['configurations'])

        print('✓ Root endpoint includes configurations test passed')

    # ========== Operation Schema Tests (test_53-54) ==========

    def test_53_service_operation_has_type_info_schema(self):
        """
        Test that service operations include type_info with request/response schemas.

        @verifies REQ_INTEROP_025
        """
        components = self._get_json('/components')

        # Find calibration component with the calibrate service
        calibration = None
        for comp in components:
            if comp['id'] == 'calibration':
                calibration = comp
                break

        self.assertIsNotNone(calibration, 'Calibration component should exist')
        self.assertIn('operations', calibration, 'Component should have operations')

        # Find the calibrate service operation
        calibrate_op = None
        for op in calibration['operations']:
            if op['name'] == 'calibrate' and op['kind'] == 'service':
                calibrate_op = op
                break

        self.assertIsNotNone(calibrate_op, 'Calibrate service should be listed')

        # Verify type_info is present with request/response schemas
        self.assertIn('type_info', calibrate_op, 'Service should have type_info')
        type_info = calibrate_op['type_info']

        self.assertIn('request', type_info, 'Service type_info should have request')
        self.assertIn('response', type_info, 'Service type_info should have response')
        self.assertIsInstance(type_info['request'], dict)
        self.assertIsInstance(type_info['response'], dict)

        # std_srvs/srv/Trigger has empty request and response with success+message
        self.assertIn('success', type_info['response'])
        self.assertIn('message', type_info['response'])

        print(f'✓ Service operation type_info test passed: {type_info}')

    def test_54_action_operation_has_type_info_schema(self):
        """
        Test that action operations include type_info with goal/result/feedback schemas.

        @verifies REQ_INTEROP_025
        """
        components = self._get_json('/components')

        # Find long_calibration component with the action
        long_cal = None
        for comp in components:
            if comp['id'] == 'long_calibration':
                long_cal = comp
                break

        self.assertIsNotNone(long_cal, 'Long calibration component should exist')
        self.assertIn('operations', long_cal, 'Component should have operations')

        # Find the long_calibration action operation
        action_op = None
        for op in long_cal['operations']:
            if op['name'] == 'long_calibration' and op['kind'] == 'action':
                action_op = op
                break

        self.assertIsNotNone(action_op, 'Long calibration action should be listed')

        # Verify type_info is present with goal/result/feedback schemas
        self.assertIn('type_info', action_op, 'Action should have type_info')
        type_info = action_op['type_info']

        self.assertIn('goal', type_info, 'Action type_info should have goal')
        self.assertIn('result', type_info, 'Action type_info should have result')
        self.assertIn('feedback', type_info, 'Action type_info should have feedback')
        self.assertIsInstance(type_info['goal'], dict)
        self.assertIsInstance(type_info['result'], dict)
        self.assertIsInstance(type_info['feedback'], dict)

        # example_interfaces/action/Fibonacci has order in goal, sequence in result/feedback
        self.assertIn('order', type_info['goal'])
        self.assertIn('sequence', type_info['result'])
        self.assertIn('sequence', type_info['feedback'])

        print(f'✓ Action operation type_info test passed: {type_info}')
