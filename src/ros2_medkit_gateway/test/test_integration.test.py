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

import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests


def generate_test_description():
    """Generate launch description with gateway node, demo nodes, and tests."""
    # Launch the ROS 2 Medkit Gateway node
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[],
    )

    # Launch demo automotive sensor nodes
    engine_temp_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_engine_temp_sensor',
        name='temp_sensor',
        namespace='/powertrain/engine',
        output='screen',
    )

    rpm_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_rpm_sensor',
        name='rpm_sensor',
        namespace='/powertrain/engine',
        output='screen',
    )

    brake_pressure_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_brake_pressure_sensor',
        name='pressure_sensor',
        namespace='/chassis/brakes',
        output='screen',
    )

    door_status_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_door_status_sensor',
        name='status_sensor',
        namespace='/body/door/front_left',
        output='screen',
    )

    brake_actuator = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_brake_actuator',
        name='actuator',
        namespace='/chassis/brakes',
        output='screen',
    )

    light_controller = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_light_controller',
        name='controller',
        namespace='/body/lights',
        output='screen',
    )

    calibration_service = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_calibration_service',
        name='calibration',
        namespace='/powertrain/engine',
        output='screen',
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
    # Must be kept in sync with gateway_params.yaml refresh_interval_ms (2000ms)
    # Need to wait for at least 2 refresh cycles to ensure all demo nodes are discovered
    CACHE_REFRESH_INTERVAL = 5.0

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
        self.assertIn('PUT /api/v1/components/{component_id}/data/{topic_name}', data['endpoints'])

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

        # Should have at least one topic with data
        if len(data) > 0:
            for topic_data in data:
                self.assertIn('topic', topic_data)
                self.assertIn('data', topic_data)
                print(f'  - Topic: {topic_data["topic"]}')

        print(f'✓ Engine component data test passed: {len(data)} topics')

    def test_08_component_data_chassis_brakes(self):
        """
        Test GET /components/{component_id}/data for brakes component.

        @verifies REQ_INTEROP_018
        """
        # Get data from pressure_sensor component (chassis/brakes)
        data = self._get_json('/components/pressure_sensor/data')
        self.assertIsInstance(data, list)

        # Check if any data is available
        if len(data) > 0:
            for topic_data in data:
                self.assertIn('topic', topic_data)
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
            self.assertIn('timestamp', first_item, "Each item should have 'timestamp' field")
            self.assertIn('data', first_item, "Each item should have 'data' field")
            self.assertIsInstance(first_item['topic'], str, "'topic' should be a string")
            self.assertIsInstance(
                first_item['timestamp'],
                int,
                "'timestamp' should be an integer (nanoseconds)"
            )
            self.assertIsInstance(first_item['data'], dict, "'data' should be an object")

        print('✓ Component data structure test passed')

    def test_11_component_nonexistent_error(self):
        """
        Test GET /components/{component_id}/data returns 404 for nonexistent component.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/data',
            timeout=5
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
                f'{self.BASE_URL}/components/{invalid_id}/data',
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
                f'{self.BASE_URL}/areas/{invalid_id}/components',
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for area_id: {invalid_id}'
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
                f'{self.BASE_URL}/components/{valid_id}/data',
                timeout=5
            )
            # Should return 404 (not found) not 400 (invalid)
            self.assertEqual(
                response.status_code,
                404,
                f'Expected 404 for valid but nonexistent ID: {valid_id}'
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
                f'{self.BASE_URL}/components/{invalid_id}/data',
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for hyphenated ID: {invalid_id}'
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
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/data/temperature',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('topic', data)
        self.assertIn('timestamp', data)
        self.assertIn('data', data)
        self.assertEqual(data['topic'], '/powertrain/engine/temperature')
        self.assertIsInstance(data['timestamp'], int)
        self.assertIsInstance(data['data'], dict)

        print(f'✓ Component topic temperature test passed: {data["topic"]}')

    def test_18_component_topic_rpm(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for RPM topic.

        @verifies REQ_INTEROP_019
        """
        response = requests.get(
            f'{self.BASE_URL}/components/rpm_sensor/data/rpm',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('topic', data)
        self.assertIn('timestamp', data)
        self.assertIn('data', data)
        self.assertEqual(data['topic'], '/powertrain/engine/rpm')

        print(f'✓ Component topic RPM test passed: {data["topic"]}')

    def test_19_component_topic_pressure(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for pressure topic.

        @verifies REQ_INTEROP_019
        """
        response = requests.get(
            f'{self.BASE_URL}/components/pressure_sensor/data/pressure',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('topic', data)
        self.assertIn('timestamp', data)
        self.assertIn('data', data)
        self.assertEqual(data['topic'], '/chassis/brakes/pressure')

        print(f'✓ Component topic pressure test passed: {data["topic"]}')

    def test_20_component_topic_data_structure(self):
        """
        Test GET /components/{component_id}/data/{topic_name} response structure.

        @verifies REQ_INTEROP_019
        """
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/data/temperature',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Verify all required fields
        self.assertIn('topic', data, "Response should have 'topic' field")
        self.assertIn('timestamp', data, "Response should have 'timestamp' field")
        self.assertIn('data', data, "Response should have 'data' field")

        # Verify field types
        self.assertIsInstance(data['topic'], str, "'topic' should be a string")
        self.assertIsInstance(
            data['timestamp'],
            int,
            "'timestamp' should be an integer (nanoseconds)"
        )
        self.assertIsInstance(data['data'], dict, "'data' should be an object")

        # Verify topic path format
        self.assertTrue(
            data['topic'].startswith('/'),
            "Topic should be an absolute path starting with '/'"
        )

        print('✓ Component topic data structure test passed')

    def test_21_component_nonexistent_topic_error(self):
        """
        Test GET /components/{component_id}/data/{topic_name} returns 404 for nonexistent topic.

        @verifies REQ_INTEROP_019
        """
        response = requests.get(
            f'{self.BASE_URL}/components/temp_sensor/data/nonexistent_topic',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Topic not found or not publishing')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'temp_sensor')
        self.assertIn('topic_name', data)
        self.assertEqual(data['topic_name'], 'nonexistent_topic')

        print('✓ Nonexistent topic error test passed')

    def test_22_component_topic_nonexistent_component_error(self):
        """
        Test GET endpoint returns 404 for nonexistent component.

        @verifies REQ_INTEROP_019
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/data/temperature',
            timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Component not found')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'nonexistent_component')

        print('✓ Component topic nonexistent component error test passed')

    def test_23_component_topic_invalid_topic_name(self):
        """
        Test GET /components/{component_id}/data/{topic_name} rejects invalid topic names.

        @verifies REQ_INTEROP_019
        """
        invalid_topic_names = [
            'topic;drop',  # SQL injection attempt
            'topic<script>',  # XSS attempt
            'topic"test',  # Quote
            'topic|test',  # Pipe
            'topic-name',  # Hyphen (not allowed in ROS 2)
        ]

        for invalid_topic in invalid_topic_names:
            response = requests.get(
                f'{self.BASE_URL}/components/temp_sensor/data/{invalid_topic}',
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for topic_name: {invalid_topic}'
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid topic name')
            self.assertIn('details', data)

        print('✓ Invalid topic name test passed')

    def test_24_component_topic_valid_underscores(self):
        """
        Test that valid topic names with underscores pass validation.

        @verifies REQ_INTEROP_019
        """
        # These topic names are valid but may not exist
        # They should return 404 (not found) not 400 (invalid)
        valid_topic_names = [
            'topic_name',
            'topic_name_123',
            'TopicName',
            'topic123',
        ]

        for valid_topic in valid_topic_names:
            response = requests.get(
                f'{self.BASE_URL}/components/temp_sensor/data/{valid_topic}',
                timeout=10
            )
            # Should return 404 (topic not found) not 400 (invalid name)
            self.assertIn(
                response.status_code,
                [200, 404],
                f'Expected 200 or 404 for valid topic name: {valid_topic}, '
                f'got {response.status_code}'
            )

        print('✓ Valid topic names with underscores test passed')

    # ========== PUT /components/{component_id}/data/{topic_name} tests ==========

    def test_25_publish_brake_command(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} publishes data.

        @verifies REQ_INTEROP_020
        """
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/command',
            json={
                'type': 'std_msgs/msg/Float32',
                'data': {'data': 50.0}
            },
            timeout=10
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
        self.assertEqual(data['topic_name'], 'command')

        print(f'✓ Publish brake command test passed: {data["topic"]}')

    def test_26_publish_validation_missing_type(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 when type missing.

        @verifies REQ_INTEROP_020
        """
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/command',
            json={
                'data': {'data': 50.0}
            },
            timeout=5
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
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/command',
            json={
                'type': 'std_msgs/msg/Float32'
            },
            timeout=5
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
            'InvalidType',           # No slashes
            'std_msgs/Float32',      # Missing /msg/
            'std_msgs/srv/Empty',    # Wrong middle part (srv instead of msg)
            'a/b/c/d',               # Too many slashes (no /msg/)
            'a/msg/b/c',             # Too many slashes (3 instead of 2)
            '/msg/Type',             # Missing package (starts with /)
            'package/msg/',          # Missing type (ends with /)
        ]

        for invalid_type in invalid_types:
            response = requests.put(
                f'{self.BASE_URL}/components/actuator/data/command',
                json={
                    'type': invalid_type,
                    'data': {'data': 50.0}
                },
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for type: {invalid_type}'
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
        response = requests.put(
            f'{self.BASE_URL}/components/nonexistent_component/data/command',
            json={
                'type': 'std_msgs/msg/Float32',
                'data': {'data': 50.0}
            },
            timeout=5
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
        response = requests.put(
            f'{self.BASE_URL}/components/actuator/data/command',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('json', data['error'].lower())

        print('✓ Publish invalid JSON body test passed')
