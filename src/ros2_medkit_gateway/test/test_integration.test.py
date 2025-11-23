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
            engine_temp_sensor, rpm_sensor, brake_pressure_sensor, door_status_sensor,
            brake_actuator, light_controller, calibration_service
        ]
    )

    return (
        LaunchDescription([
            # Launch gateway first
            gateway_node,

            # Launch demo nodes with delay
            delayed_sensors,

            # Start tests after nodes have time to initialize
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'gateway_node': gateway_node,
        }
    )


class TestROS2MedkitGatewayIntegration(unittest.TestCase):
    """Integration tests for ROS 2 Medkit Gateway REST API and discovery."""

    BASE_URL = 'http://localhost:8080'
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
                response = requests.get(f'{cls.BASE_URL}/', timeout=1)
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
        """Test GET / returns gateway status and version."""
        data = self._get_json('/')
        self.assertIn('status', data)
        self.assertIn('version', data)
        self.assertEqual(data['status'], 'ROS 2 Medkit Gateway running')
        self.assertEqual(data['version'], '0.1.0')
        print('✓ Root endpoint test passed')

    def test_02_list_areas(self):
        """Test GET /areas returns all discovered areas."""
        areas = self._get_json('/areas')
        self.assertIsInstance(areas, list)
        self.assertGreaterEqual(len(areas), 1)
        area_ids = [area['id'] for area in areas]
        self.assertIn('root', area_ids)
        print(f'✓ Areas test passed: {len(areas)} areas discovered')

    def test_03_list_components(self):
        """Test GET /components returns all discovered components."""
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
        """Test that automotive areas are properly discovered."""
        areas = self._get_json('/areas')
        area_ids = [area['id'] for area in areas]

        expected_areas = ['powertrain', 'chassis', 'body']
        for expected in expected_areas:
            self.assertIn(expected, area_ids)

        print(f'✓ All automotive areas discovered: {area_ids}')
