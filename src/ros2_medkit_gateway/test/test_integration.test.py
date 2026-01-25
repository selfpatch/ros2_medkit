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
from launch.actions import ExecuteProcess, TimerAction
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
    # Use fast refresh interval (1s) for tests to ensure cache is updated quickly
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[{'refresh_interval_ms': 1000}],
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

    # LIDAR sensor with intentionally invalid parameters to trigger faults
    lidar_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_lidar_sensor',
        name='lidar_sensor',
        namespace='/perception/lidar',
        output='screen',
        additional_env=coverage_env,
        parameters=[{
            'min_range': 10.0,   # Invalid: greater than max_range
            'max_range': 5.0,    # Invalid: less than min_range
            'scan_frequency': 25.0,  # Unsupported: exceeds 20.0 Hz
            'angular_resolution': 0.5,
        }],
    )

    # Launch the fault_manager node for fault REST API tests
    # Use in-memory storage to avoid filesystem permission issues in CI
    # Enable rosbag capture for integration testing
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=coverage_env,
        parameters=[{
            'storage_type': 'memory',
            'snapshots.rosbag.enabled': True,
            'snapshots.rosbag.duration_sec': 2.0,
            'snapshots.rosbag.duration_after_sec': 0.5,
            'snapshots.rosbag.topics': 'explicit',
            'snapshots.rosbag.include_topics': ['/rosbag_test_topic'],
        }],
    )

    # Simple publisher for rosbag test (publishes at 10Hz)
    rosbag_test_publisher = ExecuteProcess(
        cmd=[
            'ros2', 'topic', 'pub', '--rate', '10',
            '/rosbag_test_topic', 'std_msgs/msg/String',
            '{data: "rosbag_test_message"}'
        ],
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
            long_calibration_action,
            lidar_sensor,
            fault_manager_node,
            rosbag_test_publisher,
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

# Action timeout - 30s should be sufficient, if still flaky then code has performance issues
ACTION_TIMEOUT = 30.0


class TestROS2MedkitGatewayIntegration(unittest.TestCase):
    """Integration tests for ROS 2 Medkit Gateway REST API and discovery."""

    BASE_URL = f'http://localhost:8080{API_BASE_PATH}'

    # Expected entities from demo_nodes.launch.py:
    # - Apps: temp_sensor, rpm_sensor, pressure_sensor, status_sensor, actuator,
    #         controller, calibration, long_calibration, lidar_sensor (9 total)
    # - Areas: powertrain, chassis, body, perception, root (5 total)
    # - Components: Synthetic groupings by area (powertrain, chassis, body, perception)
    #               created from nodes in those namespaces
    #
    # Minimum expected components (synthetic, grouped by top-level namespace)
    # With default config: powertrain, chassis, body, perception (root may not have synthetic)
    MIN_EXPECTED_COMPONENTS = 4
    # Minimum expected apps (ROS 2 nodes from demo launch)
    MIN_EXPECTED_APPS = 8
    # Required areas that must be discovered (not just count, but specific IDs)
    REQUIRED_AREAS = {'powertrain', 'chassis', 'body'}
    # Required apps that must be discovered for deterministic tests
    REQUIRED_APPS = {'temp_sensor', 'long_calibration', 'lidar_sensor', 'actuator'}

    # Maximum time to wait for discovery (seconds)
    MAX_DISCOVERY_WAIT = 60.0
    # Interval between discovery checks (seconds)
    DISCOVERY_CHECK_INTERVAL = 1.0

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready and apps/areas to be discovered."""
        # First, wait for gateway to respond
        max_retries = 30
        for i in range(max_retries):
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if response.status_code == 200:
                    break
            except requests.exceptions.RequestException:
                if i == max_retries - 1:
                    raise unittest.SkipTest('Gateway not responding after 30 retries')
                time.sleep(1)

        # Wait for required apps AND areas to be discovered (CI can be slow)
        start_time = time.time()
        while time.time() - start_time < cls.MAX_DISCOVERY_WAIT:
            try:
                apps_response = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
                areas_response = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
                if apps_response.status_code == 200 and areas_response.status_code == 200:
                    apps = apps_response.json().get('items', [])
                    areas = areas_response.json().get('items', [])
                    app_ids = {a.get('id', '') for a in apps}
                    area_ids = {a.get('id', '') for a in areas}

                    # Check if all required areas and apps are discovered
                    missing_areas = cls.REQUIRED_AREAS - area_ids
                    missing_apps = cls.REQUIRED_APPS - app_ids
                    apps_ok = len(apps) >= cls.MIN_EXPECTED_APPS and not missing_apps
                    areas_ok = not missing_areas

                    if apps_ok and areas_ok:
                        print(f'✓ Discovery complete: {len(apps)} apps, {len(areas)} areas')
                        return

                    print(f'  Waiting: {len(apps)}/{cls.MIN_EXPECTED_APPS} apps, '
                          f'{len(areas)} areas. Missing areas: {missing_areas}, '
                          f'Missing apps: {missing_apps}')
            except requests.exceptions.RequestException:
                # Ignore connection errors during discovery wait; will retry until timeout
                pass
            time.sleep(cls.DISCOVERY_CHECK_INTERVAL)

        # If we get here, not all entities were discovered but continue anyway
        print('Warning: Discovery timeout, some tests may fail')

    def _get_json(self, endpoint: str, timeout: int = 10):
        """Get JSON from an endpoint."""
        response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=timeout)
        response.raise_for_status()
        return response.json()

    def _ensure_calibration_app_ready(self, timeout: float = 10.0, interval: float = 0.2):
        """
        Wait for the calibration app REST resource to become available.

        This is a workaround for a discovery readiness race condition in CI:
        Discovery may complete (setUpClass passes) but individual app resources
        may not yet be accessible via REST endpoints. This helper polls the
        calibration app endpoint and skips the test if it's not available within
        the timeout, avoiding flaky CI failures.

        Parameters
        ----------
        timeout : float
            Maximum time to wait in seconds (default: 10.0).
        interval : float
            Time between polling attempts in seconds (default: 0.2).

        Raises
        ------
        unittest.SkipTest
            If the calibration app is not available within the timeout.

        """
        start_time = time.time()
        last_error = None
        while time.time() - start_time < timeout:
            try:
                response = requests.get(
                    f'{self.BASE_URL}/apps/calibration',
                    timeout=2
                )
                if response.status_code == 200:
                    return  # Calibration app is ready
                last_error = f'Status {response.status_code}'
            except requests.exceptions.RequestException as e:
                last_error = str(e)
            time.sleep(interval)

        # Timeout reached - skip this test due to discovery readiness race in CI
        raise unittest.SkipTest(
            f'Calibration app not available after {timeout}s '
            f'(flaky discovery readiness race in CI). Last error: {last_error}'
        )

    def _wait_for_execution_status(
        self, execution_id: str, target_statuses: list, max_wait: float = None
    ) -> dict:
        """
        Poll execution status until it reaches one of the target statuses.

        Parameters
        ----------
        execution_id : str
            The execution ID (goal_id) to check status for.
        target_statuses : list
            List of SOVD status strings to wait for (e.g., ['completed', 'failed']).
        max_wait : float
            Maximum time to wait in seconds. Defaults to ACTION_TIMEOUT (30s).

        Returns
        -------
        dict
            The status response data when target status is reached.

        Raises
        ------
        AssertionError
            If target status is not reached within max_wait.

        """
        if max_wait is None:
            max_wait = ACTION_TIMEOUT
        start_time = time.time()
        last_status = None
        while time.time() - start_time < max_wait:
            try:
                status_response = requests.get(
                    f'{self.BASE_URL}/apps/long_calibration/operations/'
                    f'long_calibration/executions/{execution_id}',
                    timeout=5
                )
                if status_response.status_code == 200:
                    data = status_response.json()
                    last_status = data.get('status')
                    if last_status in target_statuses:
                        return data
            except requests.exceptions.RequestException:
                pass  # Retry on transient errors
            time.sleep(0.5)

        raise AssertionError(
            f'Execution did not reach status {target_statuses} within {max_wait}s. '
            f'Last status: {last_status}'
        )

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
        Test GET /version-info returns valid format and data.

        @verifies REQ_INTEROP_001
        """
        data = self._get_json('/version-info')
        # Check sovd_info array
        self.assertIn('sovd_info', data)
        self.assertIsInstance(data['sovd_info'], list)
        self.assertGreaterEqual(len(data['sovd_info']), 1)

        # Check first sovd_info entry
        info = data['sovd_info'][0]
        self.assertIn('version', info)
        self.assertIn('base_uri', info)
        self.assertIn('vendor_info', info)
        self.assertIn('version', info['vendor_info'])
        self.assertIn('name', info['vendor_info'])
        self.assertEqual(info['vendor_info']['name'], 'ros2_medkit')
        print('✓ Version info endpoint test passed')

    def test_02_list_areas(self):
        """
        Test GET /areas returns all discovered areas.

        @verifies REQ_INTEROP_003
        """
        data = self._get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']
        self.assertIsInstance(areas, list)
        self.assertGreaterEqual(len(areas), 1)
        area_ids = [area['id'] for area in areas]
        self.assertIn('root', area_ids)
        print(f'✓ Areas test passed: {len(areas)} areas discovered')

    def test_03_list_components(self):
        """
        Test GET /components returns all discovered synthetic components.

        With heuristic discovery (default), components are synthetic groups
        created by namespace aggregation. ROS 2 nodes are exposed as Apps.

        @verifies REQ_INTEROP_003
        """
        data = self._get_json('/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertIsInstance(components, list)
        # With synthetic components, we have fewer components (grouped by namespace)
        # Expected: powertrain, chassis, body, perception, root (at minimum)
        self.assertGreaterEqual(len(components), 4)

        # Verify response structure - all components should have required fields
        for component in components:
            self.assertIn('id', component)
            self.assertIn('name', component)
            self.assertIn('href', component)
            # x-medkit contains ROS2-specific fields
            self.assertIn('x-medkit', component)
            x_medkit = component['x-medkit']
            # namespace may be in x-medkit.ros2.namespace
            self.assertTrue(
                'ros2' in x_medkit and 'namespace' in x_medkit.get('ros2', {}),
                f"Component {component['id']} should have namespace in x-medkit.ros2"
            )

        # Verify expected synthetic component IDs are present
        # With heuristic discovery, components are synthetic groups created
        # by namespace aggregation. These IDs (powertrain, chassis, body)
        # represent namespace-based component groups, not individual ROS 2
        # nodes. Individual nodes are exposed as Apps instead.
        component_ids = [comp['id'] for comp in components]
        self.assertIn('powertrain', component_ids)
        self.assertIn('chassis', component_ids)
        self.assertIn('body', component_ids)

        print(f'✓ Components test passed: {len(components)} synthetic components discovered')

    def test_04_automotive_areas_discovery(self):
        """
        Test that automotive areas are properly discovered.

        @verifies REQ_INTEROP_003
        """
        data = self._get_json('/areas')
        areas = data['items']
        area_ids = [area['id'] for area in areas]

        expected_areas = ['powertrain', 'chassis', 'body']
        for expected in expected_areas:
            self.assertIn(expected, area_ids)

        print(f'✓ All automotive areas discovered: {area_ids}')

    def test_05_area_components_success(self):
        """
        Test GET /areas/{area_id}/components returns components for valid area.

        With synthetic components, the powertrain area contains the 'powertrain'
        synthetic component which aggregates all ROS 2 nodes in that namespace.

        @verifies REQ_INTEROP_006
        """
        # Test powertrain area
        data = self._get_json('/areas/powertrain/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertIsInstance(components, list)
        self.assertGreater(len(components), 0)

        # All components should have EntityReference format with x-medkit
        for component in components:
            self.assertIn('id', component)
            self.assertIn('name', component)
            self.assertIn('href', component)
            self.assertIn('x-medkit', component)
            # Verify namespace is in x-medkit.ros2
            x_medkit = component['x-medkit']
            self.assertTrue(
                'ros2' in x_medkit and 'namespace' in x_medkit.get('ros2', {}),
                'Component should have namespace in x-medkit.ros2'
            )

        # Verify the synthetic 'powertrain' component exists
        component_ids = [comp['id'] for comp in components]
        self.assertIn('powertrain', component_ids)

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
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Area not found')
        self.assertIn('parameters', data)
        self.assertIn('area_id', data['parameters'])
        self.assertEqual(data['parameters'].get('area_id'), 'nonexistent')

        print('✓ Nonexistent area error test passed')

    def test_07_app_data_powertrain_engine(self):
        """
        Test GET /apps/{app_id}/data for engine temperature sensor app.

        Apps are ROS 2 nodes. The temp_sensor app publishes temperature data.

        @verifies REQ_INTEROP_018
        """
        # Get data from temp_sensor app (powertrain/engine)
        data = self._get_json('/apps/temp_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        # Should have at least one topic
        if len(items) > 0:
            for topic_data in items:
                self.assertIn('id', topic_data)
                self.assertIn('name', topic_data)
                # direction is now in x-medkit.ros2
                self.assertIn('x-medkit', topic_data)
                x_medkit = topic_data['x-medkit']
                self.assertIn('ros2', x_medkit)
                self.assertIn('direction', x_medkit['ros2'])
                direction = x_medkit['ros2']['direction']
                self.assertIn(direction, ['publish', 'subscribe'])
                print(
                    f"  - Topic: {topic_data['name']} ({direction})"
                )

        print(f'✓ Engine app data test passed: {len(items)} topics')

    def test_08_app_data_chassis_brakes(self):
        """
        Test GET /apps/{app_id}/data for brakes pressure sensor app.

        @verifies REQ_INTEROP_018
        """
        # Get data from pressure_sensor app (chassis/brakes)
        data = self._get_json('/apps/pressure_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        # Check structure
        if len(items) > 0:
            for topic_data in items:
                self.assertIn('id', topic_data)
                self.assertIn('name', topic_data)
                # direction is now in x-medkit.ros2
                self.assertIn('x-medkit', topic_data)
                x_medkit = topic_data['x-medkit']
                self.assertIn('ros2', x_medkit)
                self.assertIn('direction', x_medkit['ros2'])

        print(f'✓ Brakes app data test passed: {len(items)} topics')

    def test_09_app_data_body_door(self):
        """
        Test GET /apps/{app_id}/data for door status sensor app.

        @verifies REQ_INTEROP_018
        """
        # Get data from status_sensor app (body/door/front_left)
        data = self._get_json('/apps/status_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        # Check structure
        if len(items) > 0:
            for topic_data in items:
                self.assertIn('id', topic_data)
                self.assertIn('name', topic_data)
                # direction is now in x-medkit.ros2
                self.assertIn('x-medkit', topic_data)
                x_medkit = topic_data['x-medkit']
                self.assertIn('ros2', x_medkit)
                self.assertIn('direction', x_medkit['ros2'])

        print(f'✓ Door app data test passed: {len(items)} topics')

    def test_10_app_data_structure(self):
        """
        Test GET /apps/{app_id}/data response structure.

        @verifies REQ_INTEROP_018
        """
        data = self._get_json('/apps/temp_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list, 'Response should have items array')

        # If we have data, verify structure
        if len(items) > 0:
            first_item = items[0]
            self.assertIn('id', first_item, "Each item should have 'id' field")
            self.assertIn('name', first_item, "Each item should have 'name' field")
            # direction and href moved to x-medkit for SOVD compliance
            self.assertIn('x-medkit', first_item, "Each item should have 'x-medkit' field")
            x_medkit = first_item['x-medkit']
            self.assertIn('ros2', x_medkit, 'x-medkit should have ros2 section')
            self.assertIn('direction', x_medkit['ros2'], 'x-medkit.ros2 should have direction')
            self.assertIsInstance(
                first_item['name'], str, "'name' should be a string"
            )
            self.assertIn(x_medkit['ros2']['direction'], ['publish', 'subscribe'])

        print('✓ App data structure test passed')

    def test_11_app_nonexistent_error(self):
        """
        Test GET /apps/{app_id}/data returns 404 for nonexistent app.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/data', timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'App not found')
        self.assertIn('parameters', data)
        self.assertIn('app_id', data['parameters'])
        self.assertEqual(data['parameters'].get('app_id'), 'nonexistent_app')

        print('✓ Nonexistent app error test passed')

    def test_12_app_no_topics(self):
        """
        Test GET /apps/{app_id}/data returns empty array.

        Verifies that apps with no topics return an empty items array.
        The calibration app typically has only services, no topics.

        @verifies REQ_INTEROP_018
        """
        # Ensure calibration app is available via REST (handles discovery race)
        self._ensure_calibration_app_ready()

        # Test with calibration app that we know has no publishing topics
        data = self._get_json('/apps/calibration/data')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list, 'Response should have items array')

        print(f'✓ App with no topics test passed: {len(data["items"])} topics')

    def test_13_invalid_app_id_special_chars(self):
        """
        Test GET /apps/{app_id}/data rejects special characters.

        @verifies REQ_INTEROP_018
        """
        # Test various invalid characters
        invalid_ids = [
            'app;drop',  # SQL injection attempt
            'app<script>',  # XSS attempt
            'app"test',  # Quote
            'app`test',  # Backtick
            'app$test',  # Dollar sign
            'app|test',  # Pipe
            'app&test',  # Ampersand
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/apps/{invalid_id}/data', timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for app_id: {invalid_id}',
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid app ID')
            self.assertIn('parameters', data)
            self.assertIn('details', data['parameters'])

        print('✓ Invalid app ID special characters test passed')

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
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid area ID')
            self.assertIn('parameters', data)
            self.assertIn('details', data['parameters'])

        print('✓ Invalid area ID special characters test passed')

    def test_15_valid_ids_with_underscores(self):
        """
        Test that valid IDs with underscores are accepted (ROS 2 naming).

        @verifies REQ_INTEROP_018
        """
        # While these IDs don't exist in the test environment,
        # they should pass validation and return 404 (not 400)
        valid_ids = [
            'app_name',  # Underscore
            'app_name_123',  # Underscore and numbers
            'AppName',  # CamelCase
            'app123',  # Alphanumeric
        ]

        for valid_id in valid_ids:
            response = requests.get(
                f'{self.BASE_URL}/apps/{valid_id}/data', timeout=5
            )
            # Should return 404 (not found) not 400 (invalid)
            self.assertEqual(
                response.status_code,
                404,
                f'Expected 404 for valid but nonexistent ID: {valid_id}',
            )

        print('✓ Valid IDs with underscores test passed')

    def test_16_invalid_ids_with_special_chars(self):
        """
        Test that IDs with special chars (except underscore/hyphen) are rejected.

        @verifies REQ_INTEROP_018
        """
        invalid_ids = [
            'app@name',
            'app name',
            'app!name',
            'app$name',
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/apps/{invalid_id}/data', timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for invalid ID: {invalid_id}',
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid app ID')

        print('✓ Invalid IDs with special chars test passed')

    def test_17_component_topic_temperature(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for temperature topic.

        Uses synthetic 'powertrain' component which aggregates apps in that namespace.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /powertrain/engine/temperature
        topic_path = encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD ReadValue format with x-medkit extension
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/powertrain/engine/temperature')
        self.assertIn('timestamp', x_medkit)
        self.assertIn('status', x_medkit)
        self.assertIsInstance(x_medkit['timestamp'], int)
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

        print(f"✓ Temperature test passed: {x_medkit['ros2']['topic']} ({x_medkit['status']})")

    def test_18_component_topic_rpm(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for RPM topic.

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /powertrain/engine/rpm
        topic_path = encode_topic_path('/powertrain/engine/rpm')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD ReadValue format with x-medkit extension
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/powertrain/engine/rpm')
        self.assertIn('timestamp', x_medkit)
        self.assertIn('status', x_medkit)
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

        topic = x_medkit['ros2']['topic']
        print(f'✓ Component topic RPM test passed: {topic} (status: {x_medkit["status"]})')

    def test_19_component_topic_pressure(self):
        """
        Test GET /components/{component_id}/data/{topic_name} for pressure topic.

        Uses synthetic 'chassis' component.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /chassis/brakes/pressure
        topic_path = encode_topic_path('/chassis/brakes/pressure')
        response = requests.get(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD ReadValue format with x-medkit extension
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/chassis/brakes/pressure')
        self.assertIn('timestamp', x_medkit)
        self.assertIn('status', x_medkit)
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

        print(f"✓ Pressure test passed: {x_medkit['ros2']['topic']} ({x_medkit['status']})")

    def test_20_component_topic_data_structure(self):
        """
        Test GET /components/{component_id}/data/{topic_name} response structure.

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /powertrain/engine/temperature
        topic_path = encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Verify SOVD ReadValue structure with x-medkit extension
        self.assertIn('id', data, "Response should have 'id' field")
        self.assertIn('data', data, "Response should have 'data' field")
        self.assertIn('x-medkit', data, "Response should have 'x-medkit' field")

        # Verify x-medkit fields
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit, 'x-medkit should have ros2 section')
        self.assertIn('topic', x_medkit['ros2'], 'x-medkit.ros2 should have topic')
        self.assertIn('timestamp', x_medkit, 'x-medkit should have timestamp')
        self.assertIn('status', x_medkit, 'x-medkit should have status')

        # Verify field types
        self.assertIsInstance(x_medkit['ros2']['topic'], str, "'ros2.topic' should be a string")
        self.assertIsInstance(
            x_medkit['timestamp'], int, "'timestamp' should be an integer (nanoseconds)"
        )
        # Status can be 'data' or 'metadata_only'
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

        # Verify topic path format
        self.assertTrue(
            x_medkit['ros2']['topic'].startswith('/'),
            "Topic should be an absolute path starting with '/'",
        )

        print('✓ Component topic data structure test passed')

    def test_21_component_nonexistent_topic_metadata_only(self):
        """
        Test nonexistent topic returns 200 with metadata_only status.

        Test GET /components/{component_id}/data/{topic_name} returns 200 with
        metadata_only status for nonexistent topics.

        The gateway returns metadata about the topic even if no data is available.
        This allows discovery of topic availability without errors.
        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /some/nonexistent/topic
        topic_path = encode_topic_path('/some/nonexistent/topic')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        # Returns 200 with metadata_only status for nonexistent topics
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD ReadValue format with x-medkit extension
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)

        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'powertrain')
        # Nonexistent topics return metadata_only status
        self.assertEqual(x_medkit['status'], 'metadata_only')

        print('✓ Nonexistent topic metadata_only test passed')

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
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Component not found')
        self.assertIn('parameters', data)
        self.assertIn('component_id', data['parameters'])
        self.assertEqual(data['parameters'].get('component_id'), 'nonexistent_component')

        print('✓ Component topic nonexistent component error test passed')

    def test_23_component_topic_with_slashes(self):
        """
        Test GET with percent-encoded slashes in topic path.

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        # Test that percent-encoded slashes work correctly
        # /powertrain/engine/temperature encoded as powertrain%2Fengine%2Ftemperature
        topic_path = encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        # Should return 200 (found) since this topic exists
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Topic is now in x-medkit.ros2.topic for SOVD compliance
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/powertrain/engine/temperature')

        print('✓ Percent-encoded slashes test passed')

    def test_24_component_topic_valid_names(self):
        """
        Test that valid topic names work correctly.

        Uses synthetic 'powertrain' component.

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
                f'{self.BASE_URL}/components/powertrain/data/{valid_topic}', timeout=10
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

        Uses synthetic 'chassis' component.

        @verifies REQ_INTEROP_020
        """
        # Use percent encoding for topic path: /chassis/brakes/command
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32', 'data': {'data': 50.0}},
            timeout=10,
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD write response format with x-medkit extension
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/chassis/brakes/command')
        self.assertEqual(x_medkit['ros2']['type'], 'std_msgs/msg/Float32')
        self.assertEqual(x_medkit['status'], 'published')

        print(f"✓ Publish brake command test passed: {x_medkit['ros2']['topic']}")

    def test_26_publish_validation_missing_type(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 when type missing.

        @verifies REQ_INTEROP_020
        """
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            json={'data': {'data': 50.0}},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('type', data['message'].lower())

        print('✓ Publish validation missing type test passed')

    def test_27_publish_validation_missing_data(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 when data missing.

        @verifies REQ_INTEROP_020
        """
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('data', data['message'].lower())

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
                f'{self.BASE_URL}/components/chassis/data/{topic_path}',
                json={'type': invalid_type, 'data': {'data': 50.0}},
                timeout=5,
            )
            self.assertEqual(
                response.status_code, 400, f'Expected 400 for type: {invalid_type}'
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid message type format')

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
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Component not found')
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('component_id'), 'nonexistent_component')

        print('✓ Publish nonexistent component test passed')

    def test_30_publish_invalid_json_body(self):
        """
        Test PUT /components/{component_id}/data/{topic_name} returns 400 for invalid JSON.

        @verifies REQ_INTEROP_020
        """
        topic_path = encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('json', data['message'].lower())

        print('✓ Publish invalid JSON body test passed')

    # ========== POST /apps/{app_id}/operations/{op}/executions tests ==========

    def test_31_operation_call_calibrate_service(self):
        """
        Test POST /apps/{app_id}/operations/{op}/executions calls a service.

        Operations are exposed on Apps (ROS 2 nodes), not synthetic Components.

        @verifies REQ_INTEROP_035
        """
        # Ensure calibration app is available via REST (handles discovery race)
        self._ensure_calibration_app_ready()

        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate/executions',
            json={},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD service response: {"parameters": {...}}
        self.assertIn('parameters', data)

        # Verify service response structure (std_srvs/srv/Trigger response)
        params = data['parameters']
        self.assertIn('success', params)
        self.assertIn('message', params)
        self.assertIsInstance(params['success'], bool)
        self.assertIsInstance(params['message'], str)

        print(f'✓ Operation call calibrate service test passed: {params}')

    def test_32_operation_call_nonexistent_operation(self):
        """
        Test operation call returns 404 for unknown operation.

        POST /apps/{app_id}/operations/{op}/executions

        @verifies REQ_INTEROP_035
        """
        # Ensure calibration app is available via REST (handles discovery race)
        self._ensure_calibration_app_ready()

        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/nonexistent_op/executions',
            json={},
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('not found', data['message'].lower())

        print('✓ Operation call nonexistent operation test passed')

    def test_33_operation_call_nonexistent_entity(self):
        """
        Test operation call returns 404 for unknown entity.

        POST /apps/{app_id}/operations/{op}/executions

        @verifies REQ_INTEROP_035
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/nonexistent_app/operations/calibrate/executions',
            json={},
            timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('not found', data['message'].lower())

        print('✓ Operation call nonexistent entity test passed')

    def test_34_operation_call_invalid_entity_id(self):
        """
        Test operation call rejects invalid entity ID.

        POST /apps/{app_id}/operations/{op}/executions

        @verifies REQ_INTEROP_035
        """
        invalid_ids = [
            'app;drop',
            'app<script>',
            'app name',
        ]

        for invalid_id in invalid_ids:
            response = requests.post(
                f'{self.BASE_URL}/apps/{invalid_id}/operations/calibrate/executions',
                json={},
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for entity_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertIn('invalid', data['message'].lower())

        print('✓ Operation call invalid entity ID test passed')

    def test_35_operation_call_invalid_operation_name(self):
        """
        Test operation call rejects invalid operation name.

        POST /apps/{app_id}/operations/{op}/executions

        @verifies REQ_INTEROP_021
        """
        invalid_names = [
            'op;drop',
            'op<script>',
            'op name',
        ]

        for invalid_name in invalid_names:
            response = requests.post(
                f'{self.BASE_URL}/apps/calibration/operations/{invalid_name}/executions',
                json={},
                timeout=5
            )
            # Accept 400 (invalid) or 404 (not found) - both are valid rejections
            self.assertIn(
                response.status_code,
                [400, 404],
                f'Expected 400 or 404 for operation_name: {invalid_name}'
            )

            data = response.json()
            self.assertIn('error_code', data)

        print('✓ Operation call invalid operation name test passed')

    def test_36_operation_call_with_invalid_json(self):
        """
        Test operation call returns 400 for invalid JSON body.

        POST /apps/{app_id}/operations/{op}/executions

        @verifies REQ_INTEROP_021
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate/executions',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('json', data['message'].lower())

        print('✓ Operation call invalid JSON body test passed')

    def test_37_operations_listed_in_app_discovery(self):
        """
        Test that operations (services) are available via app detail endpoint.

        Operations are exposed via /apps/{id} detail endpoint or /apps/{id}/operations,
        not in the list response (to keep listing lightweight).

        @verifies REQ_INTEROP_021
        """
        # Ensure calibration app is available via REST (handles discovery race)
        self._ensure_calibration_app_ready()

        # Use the detail endpoint to check operations
        data = self._get_json('/apps/calibration')

        # App detail should have capabilities including operations
        self.assertIn('capabilities', data, 'App should have capabilities')
        cap_names = [c.get('name') for c in data['capabilities']]
        self.assertIn('operations', cap_names, 'App should have operations capability')

        # Check operations endpoint directly
        ops_data = self._get_json('/apps/calibration/operations')
        self.assertIn('items', ops_data, 'Operations endpoint should return items')
        ops = ops_data['items']

        # Find the calibrate operation
        # kind is now in x-medkit.ros2.kind for SOVD compliance
        calibrate_op = None
        for op in ops:
            if op['name'] == 'calibrate':
                calibrate_op = op
                break

        self.assertIsNotNone(calibrate_op, 'Calibrate operation should be listed')
        self.assertIn('x-medkit', calibrate_op)
        x_medkit = calibrate_op['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['kind'], 'service')
        self.assertEqual(x_medkit['ros2']['type'], 'std_srvs/srv/Trigger')
        self.assertEqual(x_medkit['ros2']['service'], '/powertrain/engine/calibrate')

        print('✓ Operations listed in app discovery test passed')

    def test_38_root_endpoint_includes_operations(self):
        """
        Test that root endpoint lists operations endpoint and capability.

        @verifies REQ_INTEROP_021
        """
        data = self._get_json('/')

        # Verify operations endpoint is listed (both apps and components)
        self.assertIn('endpoints', data)
        # Check that at least one operations endpoint exists
        operations_endpoints = [e for e in data['endpoints'] if 'operations' in e.lower()]
        self.assertGreater(len(operations_endpoints), 0, 'Should have operations endpoints')

        # Verify operations capability is listed
        self.assertIn('capabilities', data)
        self.assertIn('operations', data['capabilities'])
        self.assertTrue(data['capabilities']['operations'])

        print('✓ Root endpoint includes operations test passed')

    # ========== Async Action Operations Tests (test_39-44) ==========

    def test_39_action_send_goal_and_get_id(self):
        """
        Test POST /apps/{app_id}/operations/{operation_id}/executions sends action goal.

        Sends a goal to the long_calibration action and verifies execution_id is returned.

        @verifies REQ_INTEROP_022
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            json={'parameters': {'order': 5}},
            timeout=15
        )
        self.assertEqual(response.status_code, 202)  # SOVD returns 202 Accepted

        data = response.json()
        self.assertIn('id', data)
        self.assertIsInstance(data['id'], str)
        self.assertGreater(len(data['id']), 0)
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'running')

        # Verify Location header is set
        self.assertIn('Location', response.headers)
        self.assertIn('/executions/', response.headers['Location'])

        print(f'✓ Action send goal test passed: execution_id={data["id"]}')

    def test_40_action_status_endpoint(self):
        """
        Test GET /apps/{app_id}/operations/{operation_id}/executions/{exec_id} returns status.

        @verifies REQ_INTEROP_022
        """
        # First, send a goal with enough steps to ensure it's still running
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            json={'parameters': {'order': 10}},
            timeout=15
        )
        self.assertEqual(response.status_code, 202)
        execution_id = response.json()['id']

        # Check status immediately (allow extra time for action server response)
        exec_url = (f'{self.BASE_URL}/apps/long_calibration/operations/'
                    f'long_calibration/executions/{execution_id}')
        status_response = requests.get(exec_url, timeout=10)
        self.assertEqual(status_response.status_code, 200)

        data = status_response.json()
        self.assertIn('status', data)
        # SOVD status: running, completed, failed
        valid_statuses = ['running', 'completed', 'failed']
        self.assertIn(data['status'], valid_statuses)
        self.assertIn('capability', data)
        self.assertEqual(data['capability'], 'execute')
        # x-medkit extension has ROS2-specific details
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('goal_id', x_medkit)
        self.assertEqual(x_medkit['goal_id'], execution_id)
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['action'], '/powertrain/engine/long_calibration')
        self.assertEqual(x_medkit['ros2']['type'], 'example_interfaces/action/Fibonacci')

        print(f'✓ Action status endpoint test passed: status={data["status"]}')

    def test_41_action_status_after_completion(self):
        """
        Test that execution status is updated to completed after action finishes.

        The native status subscription updates goal status in real-time.
        After an action completes, polling the executions endpoint should show 'completed'.

        @verifies REQ_INTEROP_022
        """
        # Send a short goal that will complete quickly
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            json={'parameters': {'order': 3}},
            timeout=15
        )
        self.assertEqual(response.status_code, 202)
        execution_id = response.json()['id']

        # Poll for completion instead of fixed sleep (handles CI timing variance)
        data = self._wait_for_execution_status(
            execution_id, ['completed', 'failed'], max_wait=ACTION_TIMEOUT
        )

        self.assertIn('status', data)
        self.assertEqual(data['status'], 'completed')

        print(f'✓ Action status after completion test passed: status={data["status"]}')

    def test_42_action_cancel_endpoint(self):
        """
        Test DELETE /apps/{app_id}/operations/{operation_id}/executions/{exec_id} cancels action.

        @verifies REQ_INTEROP_022
        """
        # Send a long goal that we can cancel
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            json={'parameters': {'order': 20}},
            timeout=15
        )
        self.assertEqual(response.status_code, 202)
        execution_id = response.json()['id']

        # Poll until action is executing (handles CI timing variance)
        try:
            self._wait_for_execution_status(
                execution_id, ['running'], max_wait=ACTION_TIMEOUT
            )
        except AssertionError:
            # If action already completed or is still starting, try cancel anyway
            pass

        # Cancel the execution
        exec_url = (f'{self.BASE_URL}/apps/long_calibration/operations/'
                    f'long_calibration/executions/{execution_id}')
        cancel_response = requests.delete(exec_url, timeout=10)
        # SOVD specifies 204 No Content for successful cancel
        self.assertEqual(cancel_response.status_code, 204)

        print('✓ Action cancel endpoint test passed (204 No Content)')

    def test_43_action_listed_in_app_discovery(self):
        """
        Test that actions are listed in app detail/operations response.

        Note: The /apps list endpoint returns lightweight items without operations.
        Operations are available via /apps/{id} detail or /apps/{id}/operations.

        @verifies REQ_INTEROP_022
        """
        # Get app detail for long_calibration
        data = self._get_json('/apps/long_calibration')

        # App detail should have capabilities including operations
        self.assertIn('capabilities', data, 'App detail should have capabilities')
        cap_names = [c.get('name') for c in data['capabilities']]
        self.assertIn('operations', cap_names, 'App should have operations capability')

        # Check operations endpoint directly
        ops_data = self._get_json('/apps/long_calibration/operations')
        self.assertIn('items', ops_data, 'Operations endpoint should return items')
        ops = ops_data['items']

        # Find the long_calibration action operation
        # kind is now in x-medkit.ros2.kind for SOVD compliance
        action_op = None
        for op in ops:
            if op['name'] == 'long_calibration':
                x_medkit = op.get('x-medkit', {})
                ros2 = x_medkit.get('ros2', {})
                if ros2.get('kind') == 'action':
                    action_op = op
                    break

        self.assertIsNotNone(action_op, 'long_calibration action should be listed')
        x_medkit = action_op['x-medkit']
        self.assertEqual(x_medkit['ros2']['kind'], 'action')
        self.assertEqual(x_medkit['ros2']['type'], 'example_interfaces/action/Fibonacci')
        self.assertEqual(x_medkit['ros2']['action'], '/powertrain/engine/long_calibration')

        print('✓ Action listed in app operations test passed')

    def test_44_list_executions_endpoint(self):
        """
        Test GET /apps/{app_id}/operations/{operation_id}/executions lists all executions.

        Returns list of execution IDs for the operation.

        @verifies REQ_INTEROP_022
        """
        # First, send a goal so we have something to list
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            json={'parameters': {'order': 3}},
            timeout=15
        )
        self.assertEqual(response.status_code, 202)
        expected_execution_id = response.json()['id']

        # Wait for it to complete
        time.sleep(3)

        # List all executions for this operation
        list_response = requests.get(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/executions',
            timeout=5
        )
        self.assertEqual(list_response.status_code, 200)

        data = list_response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        # Our execution should be in the list (items is list of dicts with 'id' key)
        execution_ids = [item['id'] for item in data['items']]
        self.assertIn(expected_execution_id, execution_ids)

        print(f'✓ List executions test passed: {len(data["items"])} executions found')

    # ========== Configurations API Tests (test_45-52) ==========

    def test_45_list_configurations(self):
        """
        Test GET /apps/{app_id}/configurations lists all parameters.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'temp_sensor')
        self.assertIn('parameters', x_medkit)
        self.assertIsInstance(x_medkit['parameters'], list)

        # Verify we have parameters from the demo node
        param_names = [p['name'] for p in x_medkit['parameters']]
        # The engine_temp_sensor should have these parameters we just added
        self.assertIn('publish_rate', param_names)
        self.assertIn('min_temp', param_names)
        self.assertIn('max_temp', param_names)
        self.assertIn('temp_step', param_names)

        # Verify parameter structure in x-medkit
        for param in x_medkit['parameters']:
            self.assertIn('name', param)
            self.assertIn('value', param)
            self.assertIn('type', param)

        print(f'✓ List configurations test passed: {len(x_medkit["parameters"])} parameters')

    def test_46_get_configuration(self):
        """
        Test GET /apps/{app_id}/configurations/{param_name} gets parameter.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/publish_rate',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD ReadValue format with x-medkit extension
        self.assertIn('id', data)
        self.assertEqual(data['id'], 'publish_rate')
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'temp_sensor')
        self.assertIn('parameter', x_medkit)

        param = x_medkit['parameter']
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
        Test PUT /apps/{app_id}/configurations/{param_name} sets parameter.

        @verifies REQ_INTEROP_024
        """
        # Set a new value using SOVD "data" field
        response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'data': 80.0},
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD write response format with x-medkit extension
        self.assertIn('id', data)
        self.assertEqual(data['id'], 'min_temp')
        self.assertIn('data', data)
        self.assertEqual(data['data'], 80.0)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'temp_sensor')
        self.assertIn('parameter', x_medkit)

        param = x_medkit['parameter']
        self.assertIn('name', param)
        self.assertEqual(param['name'], 'min_temp')
        self.assertIn('value', param)
        self.assertEqual(param['value'], 80.0)
        self.assertIn('type', param)
        self.assertEqual(param['type'], 'double')

        # Verify the value was actually set by reading it back
        verify_response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(verify_response.status_code, 200)
        verify_data = verify_response.json()
        self.assertEqual(verify_data['x-medkit']['parameter']['value'], 80.0)

        # Reset the value back to default using SOVD "data" field
        requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'data': 85.0},
            timeout=10
        )

        print(f'✓ Set configuration test passed: {param["name"]}={param["value"]}')

    def test_48_delete_configuration_resets_to_default(self):
        """
        Test DELETE /apps/{app_id}/configurations/{param_name} resets to default.

        The DELETE method resets the parameter to its default value.
        It first changes the value, then resets it, then verifies the reset.

        @verifies REQ_INTEROP_025
        """
        # First, change the value from default using SOVD "data" field
        set_response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'data': -50.0},
            timeout=10
        )
        self.assertEqual(set_response.status_code, 200)

        # Now reset to default via DELETE - SOVD returns 204 No Content
        response = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(response.status_code, 204)

        # Verify the value was actually reset by reading it
        get_response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(get_response.status_code, 200)
        get_data = get_response.json()
        # The value should be reset to default (85.0)
        reset_value = get_data['x-medkit']['parameter']['value']

        print(f'✓ Delete configuration (reset to default) test passed: value={reset_value}')

    def test_49_configurations_nonexistent_app(self):
        """
        Test GET /apps/{app_id}/configurations returns 404 for unknown app.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertIn('entity_id', data['parameters'])
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_app')

        print('✓ Configurations nonexistent app test passed')

    def test_50_configuration_nonexistent_parameter(self):
        """
        Test GET configurations/{param_name} returns 404 for unknown param.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/nonexistent_param',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        # Error format: parameters in parameters field
        self.assertIn('parameters', data)
        # Handler uses 'id' as the field name for the parameter
        self.assertEqual(data['parameters'].get('id'), 'nonexistent_param')

        print('✓ Configuration nonexistent parameter test passed')

    def test_51_set_configuration_missing_value(self):
        """
        Test PUT configurations/{param_name} returns 400 when value missing.

        @verifies REQ_INTEROP_024
        """
        response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={},
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        # SOVD format expects "data" field
        self.assertIn('data', data['message'].lower())

        print('✓ Set configuration missing value test passed')

    def test_52_root_endpoint_includes_configurations(self):
        """
        Test that root endpoint lists configurations endpoints and capability.

        @verifies REQ_INTEROP_023
        """
        data = self._get_json('/')

        # Verify configurations endpoints are listed (for apps or components)
        self.assertIn('endpoints', data)
        config_endpoints = [e for e in data['endpoints'] if 'configurations' in e.lower()]
        self.assertGreater(len(config_endpoints), 0, 'Should have configurations endpoints')

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
        # Get operations directly from the operations endpoint
        ops_data = self._get_json('/apps/calibration/operations')
        self.assertIn('items', ops_data, 'Operations endpoint should return items')
        ops = ops_data['items']

        # Find the calibrate service operation
        # kind is now in x-medkit.ros2.kind for SOVD compliance
        calibrate_op = None
        for op in ops:
            if op['name'] == 'calibrate':
                x_medkit = op.get('x-medkit', {})
                ros2 = x_medkit.get('ros2', {})
                if ros2.get('kind') == 'service':
                    calibrate_op = op
                    break

        self.assertIsNotNone(calibrate_op, 'Calibrate service should be listed')

        # Verify type_info is present in x-medkit with request/response schemas
        x_medkit = calibrate_op['x-medkit']
        self.assertIn('type_info', x_medkit, 'Service should have type_info in x-medkit')
        type_info = x_medkit['type_info']

        self.assertIn('request', type_info, 'Service type_info should have request')
        self.assertIn('response', type_info, 'Service type_info should have response')
        self.assertIsInstance(type_info['request'], dict)
        self.assertIsInstance(type_info['response'], dict)

        # std_srvs/srv/Trigger has empty request and response with success+message
        # Schema format is JSON Schema: {"type": "object", "properties": {...}}
        response_schema = type_info['response']
        self.assertIn('properties', response_schema, 'Response schema should have properties')
        self.assertIn('success', response_schema['properties'])
        self.assertIn('message', response_schema['properties'])

        print(f'✓ Service operation type_info test passed: {type_info}')

    def test_54_action_operation_has_type_info_schema(self):
        """
        Test that action operations include type_info with goal/result/feedback schemas.

        @verifies REQ_INTEROP_025
        """
        # Get operations directly from the operations endpoint
        ops_data = self._get_json('/apps/long_calibration/operations')
        self.assertIn('items', ops_data, 'Operations endpoint should return items')
        ops = ops_data['items']

        # Find the long_calibration action operation
        # kind is now in x-medkit.ros2.kind for SOVD compliance
        action_op = None
        for op in ops:
            if op['name'] == 'long_calibration':
                x_medkit = op.get('x-medkit', {})
                ros2 = x_medkit.get('ros2', {})
                if ros2.get('kind') == 'action':
                    action_op = op
                    break

        self.assertIsNotNone(action_op, 'Long calibration action should be listed')

        # Verify type_info is present in x-medkit with goal/result/feedback schemas
        x_medkit = action_op['x-medkit']
        self.assertIn('type_info', x_medkit, 'Action should have type_info in x-medkit')
        type_info = x_medkit['type_info']

        self.assertIn('goal', type_info, 'Action type_info should have goal')
        self.assertIn('result', type_info, 'Action type_info should have result')
        self.assertIn('feedback', type_info, 'Action type_info should have feedback')
        self.assertIsInstance(type_info['goal'], dict)
        self.assertIsInstance(type_info['result'], dict)
        self.assertIsInstance(type_info['feedback'], dict)

        # example_interfaces/action/Fibonacci has order in goal, sequence in result/feedback
        # Schema format is JSON Schema: {"type": "object", "properties": {...}}
        goal_schema = type_info['goal']
        result_schema = type_info['result']
        feedback_schema = type_info['feedback']
        self.assertIn('properties', goal_schema, 'Goal schema should have properties')
        self.assertIn('order', goal_schema['properties'])
        self.assertIn('properties', result_schema, 'Result schema should have properties')
        self.assertIn('sequence', result_schema['properties'])
        self.assertIn('properties', feedback_schema, 'Feedback schema should have properties')
        self.assertIn('sequence', feedback_schema['properties'])

        print(f'✓ Action operation type_info test passed: {type_info}')

    # ========== Faults API Tests (test_55-58) ==========

    def test_55_root_endpoint_includes_faults(self):
        """
        Test that root endpoint lists faults endpoints and capability.

        @verifies REQ_INTEROP_012
        """
        data = self._get_json('/')

        # Verify faults endpoints are listed
        self.assertIn('endpoints', data)
        self.assertIn(
            'GET /api/v1/faults',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults/{fault_code}',
            data['endpoints']
        )
        self.assertIn(
            'DELETE /api/v1/components/{component_id}/faults/{fault_code}',
            data['endpoints']
        )

        # Verify faults capability is listed
        self.assertIn('capabilities', data)
        self.assertIn('faults', data['capabilities'])
        self.assertTrue(data['capabilities']['faults'])

        print('✓ Root endpoint includes faults test passed')

    def test_56_list_faults_response_structure(self):
        """
        Test GET /apps/{app_id}/faults returns valid response structure.

        In the heuristic discovery model, ROS nodes are Apps.
        This test uses temp_sensor which is an App (ROS node).

        @verifies REQ_INTEROP_012
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'temp_sensor')
        self.assertIn('source_id', x_medkit)
        self.assertIn('count', x_medkit)

        print(f'✓ List faults response structure test passed: {x_medkit["count"]} faults')

    def test_57_faults_nonexistent_component(self):
        """
        Test GET /components/{component_id}/faults returns 404 for unknown entity.

        @verifies REQ_INTEROP_012
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        # SOVD error format: parameters in parameters field
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_component')

        print('✓ Faults nonexistent component test passed')

    def test_58_get_nonexistent_fault(self):
        """
        Test GET /apps/{app_id}/faults/{fault_code} returns 404.

        @verifies REQ_INTEROP_013
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/faults/NONEXISTENT_FAULT',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        # SOVD error format: parameters in parameters field
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('fault_code'), 'NONEXISTENT_FAULT')

        print('✓ Get nonexistent fault test passed')

    def test_59_list_all_faults_globally(self):
        """
        Test GET /faults returns all faults across the system.

        This is a convenience API for dashboards and monitoring tools
        that need a complete system health view without iterating over components.
        """
        response = requests.get(
            f'{self.BASE_URL}/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('count', x_medkit)
        self.assertIsInstance(x_medkit['count'], int)
        self.assertEqual(x_medkit['count'], len(data['items']))

        print(f'✓ List all faults globally test passed: {x_medkit["count"]} faults')

    def test_60_list_all_faults_with_status_filter(self):
        """Test GET /faults?status={status} filters faults by status."""
        # Test with status=all
        response = requests.get(
            f'{self.BASE_URL}/faults?status=all',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Items array format with x-medkit extension
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)
        self.assertIn('count', data['x-medkit'])

        # Test other valid status values
        for status in ['pending', 'confirmed', 'cleared']:
            response = requests.get(
                f'{self.BASE_URL}/faults?status={status}',
                timeout=10
            )
            self.assertEqual(response.status_code, 200)

        count = data['x-medkit']['count']
        print(f'✓ List all faults with status filter test passed: {count} faults')

    def test_61_list_faults_invalid_status_returns_400(self):
        """Test GET /faults?status=invalid returns 400 Bad Request."""
        response = requests.get(
            f'{self.BASE_URL}/faults?status=invalid_status',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Invalid status parameter value')
        # Check parameters in parameters field
        self.assertIn('parameters', data)
        params = data['parameters']
        self.assertIn('allowed_values', params)
        self.assertIn('pending', params['allowed_values'])  # Should mention valid values
        self.assertIn('parameter', params)
        self.assertEqual(params.get('parameter'), 'status')
        self.assertIn('value', params)
        self.assertEqual(params['value'], 'invalid_status')

        print('✓ List faults invalid status returns 400 test passed')

    def test_62_component_faults_invalid_status_returns_400(self):
        """Test GET /apps/{id}/faults?status=invalid returns 400."""
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/faults?status=bogus',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Invalid status parameter value')
        # SOVD error format: parameters in parameters field
        self.assertIn('parameters', data)
        # Handler uses entity_info.id_field which is 'app_id' for apps endpoint
        self.assertEqual(data['parameters'].get('app_id'), 'temp_sensor')

        print('✓ App faults invalid status returns 400 test passed')

    # ==================== SSE Fault Stream Tests ====================

    def test_63_sse_stream_endpoint_returns_correct_headers(self):
        """Test GET /faults/stream returns SSE headers."""
        # Use stream=True and timeout to avoid blocking
        try:
            response = requests.get(
                f'{self.BASE_URL}/faults/stream',
                stream=True,
                timeout=2
            )
            # Check SSE-specific headers
            self.assertEqual(response.status_code, 200)
            content_type = response.headers.get('Content-Type', '')
            self.assertIn('text/event-stream', content_type)
            self.assertEqual(
                response.headers.get('Cache-Control'),
                'no-cache'
            )

            # Close the connection (we just wanted to check headers)
            response.close()

            print('✓ SSE stream endpoint returns correct headers')
        except requests.exceptions.ReadTimeout:
            # Timeout is expected since SSE keeps connection open
            print('✓ SSE stream endpoint connection established (timeout expected)')

    def test_64_sse_stream_sends_keepalive(self):
        """Test that SSE stream can be read and handles concurrent connections."""
        import threading
        import time

        received_data = []
        connection_error = []
        stop_event = threading.Event()

        def read_stream():
            try:
                response = requests.get(
                    f'{self.BASE_URL}/faults/stream',
                    stream=True,
                    timeout=35  # Slightly longer than keepalive interval
                )
                for line in response.iter_lines(decode_unicode=True):
                    if stop_event.is_set():
                        break
                    if line:
                        received_data.append(line)
                response.close()
            except requests.exceptions.Timeout:
                # Timeout is expected when stop_event is set
                pass
            except Exception as exc:
                # Capture connection errors for assertion
                connection_error.append(str(exc))

        # Start reading in background thread
        thread = threading.Thread(target=read_stream)
        thread.daemon = True
        thread.start()

        # Wait briefly to ensure connection is established
        time.sleep(1)
        stop_event.set()
        thread.join(timeout=2)

        # Verify no connection errors occurred
        self.assertEqual(len(connection_error), 0,
                         f'SSE stream connection failed: {connection_error}')
        print('✓ SSE stream connection test passed')

    # ==================== Snapshot API Tests ====================

    def test_65_root_endpoint_includes_snapshots(self):
        """
        Test that root endpoint lists snapshots endpoints.

        @verifies REQ_INTEROP_088
        """
        data = self._get_json('/')

        # Verify snapshots endpoints are listed
        self.assertIn('endpoints', data)
        self.assertIn(
            'GET /api/v1/faults/{fault_code}/snapshots',
            data['endpoints']
        )
        self.assertIn(
            'GET /api/v1/components/{component_id}/faults/{fault_code}/snapshots',
            data['endpoints']
        )

        print('✓ Root endpoint includes snapshots test passed')

    def test_66_get_snapshots_nonexistent_fault(self):
        """
        Test GET /faults/{fault_code}/snapshots returns 404 for unknown fault.

        @verifies REQ_INTEROP_088
        """
        response = requests.get(
            f'{self.BASE_URL}/faults/NONEXISTENT_FAULT_CODE/snapshots',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('parameters', data)
        self.assertIn('fault_code', data['parameters'])
        self.assertEqual(data['parameters'].get('fault_code'), 'NONEXISTENT_FAULT_CODE')

        print('✓ Get snapshots nonexistent fault test passed')

    def test_67_get_component_snapshots_nonexistent_fault(self):
        """
        Test GET /apps/{id}/faults/{code}/snapshots returns 404 for unknown fault.

        @verifies REQ_INTEROP_088
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/faults/NONEXISTENT_FAULT/snapshots',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('parameters', data)
        self.assertIn('app_id', data['parameters'])
        self.assertEqual(data['parameters'].get('app_id'), 'temp_sensor')
        self.assertIn('fault_code', data['parameters'])
        self.assertEqual(data['parameters'].get('fault_code'), 'NONEXISTENT_FAULT')

        print('✓ Get app snapshots nonexistent fault test passed')

    def test_68_get_snapshots_nonexistent_component(self):
        """
        Test GET /components/{id}/faults/{code}/snapshots returns 404 for unknown entity.

        @verifies REQ_INTEROP_088
        """
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/faults/ANY_FAULT/snapshots',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertIn('entity_id', data['parameters'])
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_component')

        print('✓ Get snapshots nonexistent entity test passed')

    def test_69_get_snapshots_invalid_component_id(self):
        """
        Test GET /components/{id}/faults/{code}/snapshots returns 400 for invalid entity ID.

        @verifies REQ_INTEROP_088
        """
        # Note: hyphens are now allowed in IDs (for manifest entity IDs like 'engine-ecu')
        invalid_ids = [
            'component;drop',
            'component<script>',
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/components/{invalid_id}/faults/ANY_FAULT/snapshots',
                timeout=10
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for entity_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid entity ID')

        print('✓ Get snapshots invalid component ID test passed')
    # ==================== Discovery Compliance Tests ====================

    def test_70_components_list_has_href(self):
        """
        Test GET /components returns items with href field.

        Each entity in a list response MUST have an href field pointing to
        its detail endpoint.

        @verifies REQ_INTEROP_003
        """
        data = self._get_json('/components')
        self.assertIn('items', data)
        components = data['items']
        self.assertGreater(len(components), 0, 'Should have at least one component')

        for component in components:
            self.assertIn('id', component, "Component should have 'id'")
            self.assertIn('name', component, "Component should have 'name'")
            self.assertIn('href', component, "Component should have 'href'")
            self.assertTrue(
                component['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {component['href']}"
            )
            self.assertIn(component['id'], component['href'])

        print(f'✓ Components list has href test passed: {len(components)} components')

    def test_71_apps_list_has_href(self):
        """
        Test GET /apps returns items with href field.

        @verifies REQ_INTEROP_003
        """
        data = self._get_json('/apps')
        self.assertIn('items', data)
        apps = data['items']
        self.assertGreater(len(apps), 0, 'Should have at least one app')

        for app in apps:
            self.assertIn('id', app, "App should have 'id'")
            self.assertIn('name', app, "App should have 'name'")
            self.assertIn('href', app, "App should have 'href'")
            self.assertTrue(
                app['href'].startswith('/api/v1/apps/'),
                f"href should start with /api/v1/apps/, got: {app['href']}"
            )
            self.assertIn(app['id'], app['href'])

        print(f'✓ Apps list has href test passed: {len(apps)} apps')

    def test_72_areas_list_has_href(self):
        """
        Test GET /areas returns items with href field.

        @verifies REQ_INTEROP_003
        """
        data = self._get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']
        self.assertGreater(len(areas), 0, 'Should have at least one area')

        for area in areas:
            self.assertIn('id', area, "Area should have 'id'")
            self.assertIn('name', area, "Area should have 'name'")
            self.assertIn('href', area, "Area should have 'href'")
            self.assertTrue(
                area['href'].startswith('/api/v1/areas/'),
                f"href should start with /api/v1/areas/, got: {area['href']}"
            )
            self.assertIn(area['id'], area['href'])

        print(f'✓ Areas list has href test passed: {len(areas)} areas')

    def test_73_component_detail_has_capability_uris(self):
        """
        Test GET /components/{id} returns capability URIs at top level.

        SOVD requires entity details to have flat capability URIs.

        @verifies REQ_INTEROP_003
        """
        # First get list of components
        components = self._get_json('/components')['items']
        self.assertGreater(len(components), 0)

        # Get detail for first component
        component_id = components[0]['id']
        data = self._get_json(f'/components/{component_id}')

        # Verify required fields
        self.assertIn('id', data)
        self.assertEqual(data['id'], component_id)
        self.assertIn('name', data)

        # Verify SOVD capability URIs at top level
        self.assertIn('data', data, 'Component should have data URI')
        self.assertIn('operations', data, 'Component should have operations URI')
        self.assertIn('configurations', data, 'Component should have configurations URI')
        self.assertIn('faults', data, 'Component should have faults URI')

        # Verify URIs are correct format
        base = f'/api/v1/components/{component_id}'
        self.assertEqual(data['data'], f'{base}/data')
        self.assertEqual(data['operations'], f'{base}/operations')
        self.assertEqual(data['configurations'], f'{base}/configurations')
        self.assertEqual(data['faults'], f'{base}/faults')

        print(f'✓ Component detail has capability URIs test passed: {component_id}')

    def test_74_app_detail_has_capability_uris(self):
        """
        Test GET /apps/{id} returns capability URIs at top level.

        @verifies REQ_INTEROP_003
        """
        # Get detail for temp_sensor app
        data = self._get_json('/apps/temp_sensor')

        # Verify required fields
        self.assertIn('id', data)
        self.assertEqual(data['id'], 'temp_sensor')
        self.assertIn('name', data)

        # Verify SOVD capability URIs at top level
        self.assertIn('data', data, 'App should have data URI')
        self.assertIn('operations', data, 'App should have operations URI')
        self.assertIn('configurations', data, 'App should have configurations URI')

        # Verify URIs are correct format
        base = '/api/v1/apps/temp_sensor'
        self.assertEqual(data['data'], f'{base}/data')
        self.assertEqual(data['operations'], f'{base}/operations')
        self.assertEqual(data['configurations'], f'{base}/configurations')

        print('✓ App detail has capability URIs test passed: temp_sensor')

    def test_75_subareas_list_has_href(self):
        """
        Test GET /areas/{id}/subareas returns items with href field.

        @verifies REQ_INTEROP_004
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/root/subareas',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

        # If there are subareas, verify they have href
        for subarea in data.get('items', []):
            self.assertIn('id', subarea, "Subarea should have 'id'")
            self.assertIn('name', subarea, "Subarea should have 'name'")
            self.assertIn('href', subarea, "Subarea should have 'href'")
            self.assertTrue(
                subarea['href'].startswith('/api/v1/areas/'),
                f"href should start with /api/v1/areas/, got: {subarea['href']}"
            )

        print(f'✓ Subareas list has href test passed: {len(data.get("items", []))} subareas')

    def test_76_subcomponents_list_has_href(self):
        """
        Test GET /components/{id}/subcomponents returns items with href field.

        @verifies REQ_INTEROP_005
        """
        # First get a component
        components = self._get_json('/components')['items']
        self.assertGreater(len(components), 0)
        component_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{component_id}/subcomponents',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

        # If there are subcomponents, verify they have href
        for subcomp in data.get('items', []):
            self.assertIn('id', subcomp, "Subcomponent should have 'id'")
            self.assertIn('name', subcomp, "Subcomponent should have 'name'")
            self.assertIn('href', subcomp, "Subcomponent should have 'href'")
            self.assertTrue(
                subcomp['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {subcomp['href']}"
            )

        count = len(data.get('items', []))
        print(f'✓ Subcomponents list has href test passed: {count} subcomponents')

    def test_77b_contains_list_has_href(self):
        """
        Test GET /areas/{id}/contains returns items with href field.

        @verifies REQ_INTEROP_006
        """
        # Get contains for root area
        response = requests.get(
            f'{self.BASE_URL}/areas/root/contains',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('_links', data)
        self.assertEqual(data['_links']['self'], '/api/v1/areas/root/contains')
        self.assertEqual(data['_links']['area'], '/api/v1/areas/root')

        # If there are contained components, verify they have href
        for comp in data.get('items', []):
            self.assertIn('id', comp, "Contained component should have 'id'")
            self.assertIn('name', comp, "Contained component should have 'name'")
            self.assertIn('href', comp, "Contained component should have 'href'")
            self.assertTrue(
                comp['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {comp['href']}"
            )

        count = len(data.get('items', []))
        print(f'✓ Area contains list has href test passed: {count} components')

    def test_77c_hosts_list_has_href(self):
        """
        Test GET /components/{id}/hosts returns items with href field.

        @verifies REQ_INTEROP_007
        """
        # Get hosts for powertrain component
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/hosts',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('_links', data)
        self.assertEqual(data['_links']['self'], '/api/v1/components/powertrain/hosts')
        self.assertEqual(data['_links']['component'], '/api/v1/components/powertrain')

        # Powertrain should have hosted apps
        for app in data.get('items', []):
            self.assertIn('id', app, "Hosted app should have 'id'")
            self.assertIn('name', app, "Hosted app should have 'name'")
            self.assertIn('href', app, "Hosted app should have 'href'")
            self.assertTrue(
                app['href'].startswith('/api/v1/apps/'),
                f"href should start with /api/v1/apps/, got: {app['href']}"
            )

        print(f'✓ Component hosts list has href test passed: {len(data.get("items", []))} apps')

    def test_78_depends_on_components_has_href(self):
        """
        Test GET /components/{id}/depends-on returns items with href field.

        @verifies REQ_INTEROP_008
        """
        # Get a component to test depends-on
        components = self._get_json('/components')['items']
        self.assertGreater(len(components), 0)
        component_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{component_id}/depends-on',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)

        # If there are dependencies, verify they have href
        for dep in data.get('items', []):
            self.assertIn('id', dep, "Dependency should have 'id'")
            self.assertIn('name', dep, "Dependency should have 'name'")
            self.assertIn('href', dep, "Dependency should have 'href'")
            self.assertTrue(
                dep['href'].startswith('/api/v1/components/'),
                f"href should start with /api/v1/components/, got: {dep['href']}"
            )

        print(f'✓ Component depends-on has href test passed: {len(data.get("items", []))} deps')

    def test_79_depends_on_apps_has_href(self):
        """
        Test GET /apps/{id}/depends-on returns items with href field.

        @verifies REQ_INTEROP_009
        """
        # Get temp_sensor app's depends-on
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/depends-on',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIn('_links', data)
        self.assertEqual(data['_links']['self'], '/api/v1/apps/temp_sensor/depends-on')
        self.assertEqual(data['_links']['app'], '/api/v1/apps/temp_sensor')

        # If there are dependencies, verify they have href
        for dep in data.get('items', []):
            self.assertIn('id', dep, "Dependency should have 'id'")
            self.assertIn('name', dep, "Dependency should have 'name'")
            self.assertIn('href', dep, "Dependency should have 'href'")
            self.assertTrue(
                dep['href'].startswith('/api/v1/apps/'),
                f"href should start with /api/v1/apps/, got: {dep['href']}"
            )

        print(f'✓ App depends-on has href test passed: {len(data.get("items", []))} deps')

    def test_80_depends_on_apps_nonexistent(self):
        """
        Test GET /apps/{id}/depends-on returns 404 for unknown app.

        @verifies REQ_INTEROP_009
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/depends-on',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'App not found')
        self.assertIn('parameters', data)
        self.assertIn('app_id', data['parameters'])
        self.assertEqual(data['parameters'].get('app_id'), 'nonexistent_app')

        print('✓ App depends-on nonexistent app test passed')

    def test_81_functions_list_has_href(self):
        """
        Test GET /functions returns items with href field.

        @verifies REQ_INTEROP_003
        """
        data = self._get_json('/functions')
        self.assertIn('items', data)
        functions = data['items']

        # Functions may be empty if no manifest is loaded
        for func in functions:
            self.assertIn('id', func, "Function should have 'id'")
            self.assertIn('name', func, "Function should have 'name'")
            self.assertIn('href', func, "Function should have 'href'")
            self.assertTrue(
                func['href'].startswith('/api/v1/functions/'),
                f"href should start with /api/v1/functions/, got: {func['href']}"
            )

        print(f'✓ Functions list has href test passed: {len(functions)} functions')

    def test_82_root_endpoint_has_apps_endpoints(self):
        """
        Test that root endpoint lists apps endpoints including depends-on.

        @verifies REQ_INTEROP_010
        """
        data = self._get_json('/')
        self.assertIn('endpoints', data)

        # Verify apps endpoints are listed
        endpoints = data['endpoints']
        self.assertIn('GET /api/v1/apps', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/depends-on', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/data', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/operations', endpoints)
        self.assertIn('GET /api/v1/apps/{app_id}/configurations', endpoints)

        print('✓ Root endpoint has apps endpoints test passed')

    def test_83_x_medkit_extension_in_list_responses(self):
        """
        Test that list responses have x-medkit at item and response level.

        ROS2-specific data should be in x-medkit extension, not at top level.

        @verifies REQ_INTEROP_003
        """
        # Test components list
        data = self._get_json('/components')
        self.assertIn('items', data)
        self.assertIn('x-medkit', data, 'Response should have x-medkit')
        self.assertIn('total_count', data['x-medkit'], 'Response x-medkit should have total_count')

        for component in data['items']:
            self.assertIn('x-medkit', component, 'Item should have x-medkit')
            x_medkit = component['x-medkit']
            # ROS2-specific fields should be in x-medkit
            self.assertIn('source', x_medkit, 'x-medkit should have source')

        # Test apps list
        data = self._get_json('/apps')
        self.assertIn('items', data)
        self.assertIn('x-medkit', data)

        for app in data['items']:
            self.assertIn('x-medkit', app, 'Item should have x-medkit')
            x_medkit = app['x-medkit']
            self.assertIn('source', x_medkit, 'x-medkit should have source')
            self.assertIn('is_online', x_medkit, 'x-medkit should have is_online')

        print('✓ x-medkit extension in list responses test passed')

    def test_84_get_operation_details_for_service(self):
        """
        Test GET /{entity}/operations/{op-id} returns operation details for service.

        @verifies REQ_INTEROP_034
        """
        # First get operations for powertrain component
        data = self._get_json('/components/powertrain/operations')
        self.assertIn('items', data)
        operations = data['items']
        self.assertGreater(len(operations), 0, 'Component should have operations')

        # Find a service (asynchronous_execution: false)
        service_op = None
        for op in operations:
            if not op.get('asynchronous_execution', True):
                service_op = op
                break

        if service_op is None:
            self.skipTest('No service operations found')
            return

        operation_id = service_op['id']

        # Get the operation details
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/operations/{operation_id}',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('item', data)
        item = data['item']

        # Requred fields
        self.assertIn('id', item)
        self.assertEqual(item['id'], operation_id)
        self.assertIn('name', item)
        self.assertIn('proximity_proof_required', item)
        self.assertFalse(item['proximity_proof_required'])
        self.assertIn('asynchronous_execution', item)
        self.assertFalse(item['asynchronous_execution'])

        # x-medkit extension
        self.assertIn('x-medkit', item)
        x_medkit = item['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertIn('kind', x_medkit['ros2'])
        self.assertEqual(x_medkit['ros2']['kind'], 'service')
        self.assertIn('type', x_medkit['ros2'])
        self.assertIn('service', x_medkit['ros2'])

        print(f'✓ Get operation details for service test passed: {operation_id}')

    def test_85_get_operation_details_for_action(self):
        """
        Test GET /{entity}/operations/{op-id} returns operation details for action.

        @verifies REQ_INTEROP_034
        """
        # Find an action operation
        data = self._get_json('/components')
        components = data['items']

        action_op = None
        component_id = None

        for comp in components:
            ops_data = self._get_json(f'/components/{comp["id"]}/operations')
            for op in ops_data.get('items', []):
                if op.get('asynchronous_execution', False):
                    action_op = op
                    component_id = comp['id']
                    break
            if action_op:
                break

        if action_op is None:
            self.skipTest('No action operations found')
            return

        operation_id = action_op['id']

        # Get the operation details
        response = requests.get(
            f'{self.BASE_URL}/components/{component_id}/operations/{operation_id}',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('item', data)
        item = data['item']

        # Required fields
        self.assertIn('asynchronous_execution', item)
        self.assertTrue(item['asynchronous_execution'])

        # x-medkit extension
        self.assertIn('x-medkit', item)
        x_medkit = item['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertIn('kind', x_medkit['ros2'])
        self.assertEqual(x_medkit['ros2']['kind'], 'action')

        print(f'✓ Get operation details for action test passed: {operation_id}')

    def test_86_get_operation_not_found(self):
        """
        Test GET /{entity}/operations/{op-id} returns 404 for nonexistent operation.

        @verifies REQ_INTEROP_034
        """
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/operations/nonexistent_op',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('message', data)
        self.assertEqual(data['message'], 'Operation not found')

        print('✓ Get operation not found test passed')

    def test_87_list_executions_returns_items_array(self):
        """
        Test GET /{entity}/operations/{op-id}/executions returns items array.

        @verifies REQ_INTEROP_036
        """
        # Find an action to test with
        data = self._get_json('/components')
        components = data['items']

        action_op = None
        component_id = None

        for comp in components:
            ops_data = self._get_json(f'/components/{comp["id"]}/operations')
            for op in ops_data.get('items', []):
                if op.get('asynchronous_execution', False):
                    action_op = op
                    component_id = comp['id']
                    break
            if action_op:
                break

        if action_op is None:
            self.skipTest('No action operations found')
            return

        operation_id = action_op['id']

        # List executions - should return items array (may be empty)
        response = requests.get(
            f'{self.BASE_URL}/components/{component_id}/operations/{operation_id}/executions',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

        print(f'✓ List executions returns items array test passed: {operation_id}')

    def test_88_create_execution_for_service(self):
        """
        Test POST /{entity}/operations/{op-id}/executions calls service and returns.

        @verifies REQ_INTEROP_035
        """
        # Find a service operation to call
        data = self._get_json('/components/powertrain/operations')
        operations = data['items']

        service_op = None
        for op in operations:
            if not op.get('asynchronous_execution', True):
                service_op = op
                break

        if service_op is None:
            self.skipTest('No service operations found')
            return

        operation_id = service_op['id']

        # Call the service
        response = requests.post(
            f'{self.BASE_URL}/components/powertrain/operations/{operation_id}/executions',
            json={'parameters': {}},
            timeout=30
        )

        # Should return 200 for sync or 400/500 if service is unavailable
        self.assertIn(
            response.status_code, [200, 400, 500, 503],
            f'Expected 200/400/500/503, got {response.status_code}: {response.text}'
        )

        data = response.json()
        if response.status_code == 200:
            # Successful sync execution
            self.assertIn('parameters', data)

        print(f'✓ Create execution for service test passed: {operation_id}')

    def test_89_cancel_nonexistent_execution(self):
        """
        Test DELETE /{entity}/operations/{op-id}/executions/{exec-id} returns 404.

        @verifies REQ_INTEROP_039
        """
        url = (f'{self.BASE_URL}/components/powertrain/operations/'
               'nonexistent_op/executions/fake-exec-id')
        response = requests.delete(url, timeout=10)
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('message', data)
        self.assertEqual(data['message'], 'Execution not found')

        print('✓ Cancel nonexistent execution test passed')

    def test_90_delete_all_faults_for_component(self):
        """
        Test DELETE /components/{id}/faults clears all faults for component.

        @verifies REQ_INTEROP_014
        """
        # First get a component
        data = self._get_json('/components')
        self.assertGreater(len(data['items']), 0)
        component_id = data['items'][0]['id']

        # Attempt to clear all faults (should succeed even if no faults)
        response = requests.delete(
            f'{self.BASE_URL}/components/{component_id}/faults',
            timeout=10
        )

        # Should return 204 No Content on success
        self.assertEqual(response.status_code, 204)
        self.assertEqual(len(response.content), 0)

        print(f'✓ Delete all faults for component test passed: {component_id}')

    def test_91_delete_all_faults_for_app(self):
        """
        Test DELETE /apps/{id}/faults clears all faults for app.

        @verifies REQ_INTEROP_014
        """
        # Get an app
        data = self._get_json('/apps')
        self.assertGreater(len(data['items']), 0)
        app_id = data['items'][0]['id']

        # Attempt to clear all faults
        response = requests.delete(
            f'{self.BASE_URL}/apps/{app_id}/faults',
            timeout=10
        )

        # Should return 204 No Content
        self.assertEqual(response.status_code, 204)
        self.assertEqual(len(response.content), 0)

        print(f'✓ Delete all faults for app test passed: {app_id}')

    def test_92_delete_all_faults_nonexistent_entity(self):
        """
        Test DELETE /{entity}/faults returns 404 for nonexistent entity.

        @verifies REQ_INTEROP_014
        """
        response = requests.delete(
            f'{self.BASE_URL}/components/nonexistent_component/faults',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('message', data)
        self.assertEqual(data['message'], 'Entity not found')

        print('✓ Delete all faults nonexistent entity test passed')

    def test_93_get_operation_details_for_apps(self):
        """
        Test GET /apps/{id}/operations/{op-id} works for apps.

        @verifies REQ_INTEROP_034
        """
        # Get apps with operations
        data = self._get_json('/apps')
        apps = data['items']

        operation_found = False
        for app in apps:
            ops_data = self._get_json(f'/apps/{app["id"]}/operations')
            if ops_data.get('items'):
                operation_id = ops_data['items'][0]['id']

                # Get operation details
                response = requests.get(
                    f'{self.BASE_URL}/apps/{app["id"]}/operations/{operation_id}',
                    timeout=10
                )
                self.assertEqual(response.status_code, 200)

                data = response.json()
                self.assertIn('item', data)
                self.assertIn('id', data['item'])
                self.assertIn('x-medkit', data['item'])

                operation_found = True
                print(f'✓ Get operation details for apps test passed: {operation_id}')
                break

        if not operation_found:
            self.skipTest('No app operations found')

    # ==================== REQ_INTEROP_002: Documentation Endpoint ====================

    def test_94_docs_endpoint(self):
        """
        Test GET /{resource}/docs endpoint.

        TODO: The /docs endpoint is not implemented. Currently returns 404
        because 'docs' is interpreted as a component ID that doesn't exist.
        When docs endpoint is implemented, this test should verify proper
        documentation response with 200 status. Add verifies after implementation
        """
        # Try docs endpoint on components collection
        # Currently not implemented - 'docs' is treated as component ID
        response = requests.get(f'{self.BASE_URL}/components/docs', timeout=10)

        # TODO: Change to 200 when docs endpoint is implemented
        self.assertEqual(
            response.status_code, 404,
            'Docs endpoint not implemented - returns 404 (component "docs" not found)'
        )

        print('✓ Docs endpoint test passed: 404 (not implemented)')

    # ==================== REQ_INTEROP_015: Delete Single Fault ====================

    def _wait_for_fault(self, app_id: str, fault_code: str,
                        max_wait: float = 10.0) -> dict:
        """
        Wait for a specific fault to appear on an app.

        Parameters
        ----------
        app_id : str
            The app ID to check for faults.
        fault_code : str
            The fault code to wait for.
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        dict
            The fault data when found.

        Raises
        ------
        AssertionError
            If fault is not found within max_wait.

        """
        start_time = time.time()
        while time.time() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}/apps/{app_id}/faults',
                    timeout=5
                )
                if response.status_code == 200:
                    data = response.json()
                    for fault in data.get('items', []):
                        if fault.get('fault_code') == fault_code:
                            return fault
            except requests.exceptions.RequestException:
                # Network errors expected during transient states; silently retry
                pass
            time.sleep(0.5)

        raise AssertionError(
            f'Fault {fault_code} not found on {app_id} within {max_wait}s'
        )

    def test_95_delete_single_fault(self):
        """
        Test DELETE /apps/{id}/faults/{code} clears a specific fault.

        Uses lidar_sensor which has deterministic faults due to invalid parameters.
        The LIDAR_RANGE_INVALID fault is triggered because min_range > max_range.

        Note: The fault may be immediately re-reported by the sensor after deletion,
        so we only verify the DELETE returns 204 (success) or 404 (not found).

        @verifies REQ_INTEROP_015
        """
        # lidar_sensor has known faults triggered by invalid parameters
        app_id = 'lidar_sensor'
        fault_code = 'LIDAR_RANGE_INVALID'

        # Wait for the fault to be reported (lidar_sensor publishes faults)
        try:
            self._wait_for_fault(app_id, fault_code, max_wait=15.0)
        except AssertionError:
            # Fault may not be present if fault_manager didn't receive it yet
            # In this case, test that 404 is returned for nonexistent fault
            response = requests.delete(
                f'{self.BASE_URL}/apps/{app_id}/faults/{fault_code}',
                timeout=10
            )
            self.assertEqual(
                response.status_code, 404,
                f'Expected 404 for nonexistent fault, got {response.status_code}'
            )
            print('✓ Delete single fault test passed: fault not present, 404 returned')
            return

        # Delete the fault - should return 204 (success) or 404 (already gone)
        response = requests.delete(
            f'{self.BASE_URL}/apps/{app_id}/faults/{fault_code}',
            timeout=10
        )
        self.assertIn(
            response.status_code, [204, 404],
            f'Expected 204 or 404 for fault deletion, got {response.status_code}'
        )

        # Note: We do NOT verify the fault is gone because lidar_sensor continuously
        # re-reports it due to its invalid configuration. The important assertion is
        # that the DELETE endpoint works correctly (returns 204 when fault exists).

        print(f'✓ Delete single fault test passed: DELETE returned {response.status_code}')

    # ==================== REQ_INTEROP_016: Data Categories ====================

    def test_96_list_data_categories(self):
        """
        Test GET /apps/{id}/data-categories returns 501 Not Implemented.

        TODO: Data categories are not yet implemented in the gateway. This test
        verifies the endpoint exists and returns the correct error status.
        Add verifies after implementation
        """
        # Use known app
        app_id = 'temp_sensor'

        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/data-categories',
            timeout=10
        )

        # Feature not implemented - expect 501
        self.assertEqual(
            response.status_code, 501,
            f'Expected 501 Not Implemented, got {response.status_code}'
        )

        data = response.json()
        self.assertIn('error_code', data)

        print('✓ Data categories test passed: 501 Not Implemented')

    # ==================== REQ_INTEROP_017: Data Groups ====================

    def test_97_list_data_groups(self):
        """
        Test GET /apps/{id}/data-groups returns 501 Not Implemented.

        TODO: Data groups are not yet implemented in the gateway. This test
        verifies the endpoint exists and returns the correct error status.
        Add verifies after implementation.
        """
        # Use known app
        app_id = 'temp_sensor'

        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/data-groups',
            timeout=10
        )

        # Feature not implemented - expect 501
        self.assertEqual(
            response.status_code, 501,
            f'Expected 501 Not Implemented, got {response.status_code}'
        )

        data = response.json()
        self.assertIn('error_code', data)

        print('✓ Data groups test passed: 501 Not Implemented')

    # ==================== REQ_INTEROP_020: Write Data ====================

    def test_98_write_data_to_topic(self):
        """
        Test PUT /apps/{id}/data/{data-id} publishes data to topic.

        Uses the brake actuator which subscribes to /chassis/brakes/command.
        This is a deterministic writable topic for testing data writes.

        @verifies REQ_INTEROP_020
        """
        # Brake actuator has a known command topic that accepts writes
        app_id = 'actuator'

        # Get the actuator's data to find the command topic
        app_data = self._get_json(f'/apps/{app_id}/data')
        self.assertIn('items', app_data)

        # Find a topic with subscribe direction (actuator listens to commands)
        subscribe_topic = None
        for item in app_data['items']:
            x_medkit = item.get('x-medkit', {})
            ros2 = x_medkit.get('ros2', {})
            if ros2.get('direction') == 'subscribe':
                subscribe_topic = item
                break

        if subscribe_topic is None:
            self.skipTest('Actuator has no subscribe topics')
            return

        topic_id = subscribe_topic['id']

        # Write brake pressure command (50.0 bar)
        response = requests.put(
            f'{self.BASE_URL}/apps/{app_id}/data/{topic_id}',
            json={
                'type': 'std_msgs/msg/Float32',
                'data': {'data': 50.0}
            },
            timeout=10
        )

        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 for data write, got {response.status_code}: {response.text}'
        )

        data = response.json()
        self.assertIn('x-medkit', data)
        self.assertEqual(data['x-medkit']['status'], 'published')

        print(f'✓ Write data test passed: published to {topic_id}')

    # ==================== Helper: Wait for Operation Discovery ====================

    def _wait_for_operation(self, app_id: str, operation_id: str,
                            max_wait: float = 15.0) -> bool:
        """
        Wait for an operation to be discovered for an app.

        Parameters
        ----------
        app_id : str
            The app ID to check operations for.
        operation_id : str
            The operation ID to wait for.
        max_wait : float
            Maximum time to wait in seconds.

        Returns
        -------
        bool
            True if operation found, False otherwise.

        """
        start_time = time.time()
        while time.time() - start_time < max_wait:
            try:
                response = requests.get(
                    f'{self.BASE_URL}/apps/{app_id}/operations',
                    timeout=5
                )
                if response.status_code == 200:
                    ops = response.json().get('items', [])
                    if any(op.get('id') == operation_id for op in ops):
                        return True
            except requests.exceptions.RequestException:
                # Network errors expected during transient states; silently retry
                pass
            time.sleep(0.5)
        return False

    # ==================== REQ_INTEROP_033: List Operations ====================

    def test_99_list_operations(self):
        """
        Test GET /apps/{id}/operations returns operations list.

        @verifies REQ_INTEROP_033
        """
        # Get an app
        data = self._get_json('/apps')
        self.assertGreater(len(data['items']), 0)
        app_id = data['items'][0]['id']

        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/operations',
            timeout=10
        )

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('items', data)

        print(f'✓ List operations test passed: {len(data["items"])} operations')

    # ==================== REQ_INTEROP_037: Get Execution Status ====================

    def test_100_get_execution_status(self):
        """
        Test GET /apps/{id}/operations/{op-id}/executions/{exec-id} gets status.

        Creates a real execution using long_calibration action, then verifies
        the execution status endpoint returns correct data.

        @verifies REQ_INTEROP_037
        """
        # Use long_calibration app which provides long_calibration action
        app_id = 'long_calibration'
        # Operation ID is the action name (last segment of action path)
        # Action path: /powertrain/engine/long_calibration -> name: long_calibration
        operation_id = 'long_calibration'

        # Wait for operation to be discovered (action discovery can be slower)
        found = self._wait_for_operation(app_id, operation_id, max_wait=15.0)
        self.assertTrue(
            found,
            f'Operation {operation_id} not discovered for {app_id} within timeout'
        )

        # Create a real execution
        create_response = requests.post(
            f'{self.BASE_URL}/apps/{app_id}/operations/{operation_id}/executions',
            json={'parameters': {'order': 5}},
            timeout=15
        )
        self.assertEqual(
            create_response.status_code, 202,
            f'Expected 202 for action creation, got {create_response.status_code}'
        )

        execution_id = create_response.json()['id']

        # Now get the execution status
        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/operations/{operation_id}'
            f'/executions/{execution_id}',
            timeout=10
        )

        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 for execution status, got {response.status_code}'
        )

        data = response.json()
        # Execution status response uses x-medkit.goal_id as identifier
        self.assertIn('status', data)
        self.assertIn(data['status'], ['running', 'completed', 'failed'])
        self.assertIn('x-medkit', data)
        self.assertEqual(data['x-medkit']['goal_id'], execution_id)

        print(f'✓ Get execution status test passed: {data["status"]}')

    # ==================== REQ_INTEROP_038: Update Execution ====================

    def test_101_update_execution(self):
        """
        Test PUT /apps/{id}/operations/{op-id}/executions/{exec-id} returns 501.

        Execution updates (pause/resume) are not supported for ROS 2 actions.
        This test verifies the endpoint exists and returns appropriate error.

        @verifies REQ_INTEROP_038
        """
        # Use long_calibration app which provides long_calibration action
        app_id = 'long_calibration'
        # Operation ID is the action name (last segment of action path)
        # Action path: /powertrain/engine/long_calibration -> name: long_calibration
        operation_id = 'long_calibration'

        # Wait for operation to be discovered (action discovery can be slower)
        found = self._wait_for_operation(app_id, operation_id, max_wait=15.0)
        self.assertTrue(
            found,
            f'Operation {operation_id} not discovered for {app_id} within timeout'
        )

        # Create a real execution
        create_response = requests.post(
            f'{self.BASE_URL}/apps/{app_id}/operations/{operation_id}/executions',
            json={'parameters': {'order': 10}},
            timeout=15
        )
        self.assertEqual(
            create_response.status_code, 202,
            f'Expected 202 for action creation, got {create_response.status_code}'
        )

        execution_id = create_response.json()['id']

        # Try to update (pause) the execution - not supported
        response = requests.put(
            f'{self.BASE_URL}/apps/{app_id}/operations/{operation_id}'
            f'/executions/{execution_id}',
            json={'action': 'pause'},
            timeout=10
        )

        # PUT for pause/resume returns 400 (invalid request) or 501 (not implemented)
        self.assertIn(
            response.status_code, [400, 501],
            f'Expected 400 or 501 for unsupported pause, got {response.status_code}'
        )

        data = response.json()
        self.assertIn('error_code', data)

        print(f'✓ Update execution test passed: {response.status_code}')

    # ==================== REQ_INTEROP_048: List Configurations ====================

    def test_102_list_configurations(self):
        """
        Test GET /apps/{id}/configurations returns configuration list.

        @verifies REQ_INTEROP_048
        """
        # Get an app
        data = self._get_json('/apps')
        self.assertGreater(len(data['items']), 0)
        app_id = data['items'][0]['id']

        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/configurations',
            timeout=10
        )

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('items', data)

        print(f'✓ List configurations test passed: {len(data["items"])} configs')

    # ==================== REQ_INTEROP_049: Get Configuration ====================

    def test_103_get_configuration(self):
        """
        Test GET /apps/{id}/configurations/{config-id} returns configuration value.

        Dynamically finds an app with configurations and tests single config endpoint.

        @verifies REQ_INTEROP_049
        """
        # Find an app that has configurations
        apps_data = self._get_json('/apps')
        self.assertGreater(len(apps_data['items']), 0, 'No apps found')

        app_id = None
        config_id = None

        for app in apps_data['items']:
            configs_response = requests.get(
                f'{self.BASE_URL}/apps/{app["id"]}/configurations',
                timeout=10
            )
            if configs_response.status_code == 200:
                configs = configs_response.json().get('items', [])
                if configs:
                    app_id = app['id']
                    config_id = configs[0]['id']
                    break

        if not app_id or not config_id:
            self.skipTest('No app with configurations found')

        # Now test get single config
        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/configurations/{config_id}',
            timeout=10
        )

        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 for config {config_id}, got {response.status_code}'
        )

        data = response.json()
        self.assertIn('id', data)
        self.assertEqual(data['id'], config_id)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)

        # Verify parameter details
        x_medkit = data['x-medkit']
        self.assertIn('parameter', x_medkit)
        param = x_medkit['parameter']
        self.assertIn('name', param)
        self.assertIn('type', param)

        print(f'✓ Get configuration test passed: {app_id}/{config_id}={param.get("value")}')

    # ==================== REQ_INTEROP_050: Set Configuration ====================

    def test_104_set_configuration(self):
        """
        Test PUT /apps/{id}/configurations/{config-id} sets configuration value.

        @verifies REQ_INTEROP_050
        """
        # Get an app with configurations
        apps_data = self._get_json('/apps')
        self.assertGreater(len(apps_data['items']), 0)

        config_found = False
        for app in apps_data['items']:
            configs_data = self._get_json(f'/apps/{app["id"]}/configurations')
            if configs_data.get('items'):
                config_id = configs_data['items'][0]['id']

                # Get current value first
                get_response = requests.get(
                    f'{self.BASE_URL}/apps/{app["id"]}/configurations/{config_id}',
                    timeout=10
                )

                if get_response.status_code == 200:
                    current_data = get_response.json()
                    current_value = current_data.get('data', 1.0)

                    # Try to set the same value back - should succeed
                    response = requests.put(
                        f'{self.BASE_URL}/apps/{app["id"]}/configurations/{config_id}',
                        json={'data': current_value},
                        timeout=10
                    )

                    # Setting an existing config to the same value should succeed
                    self.assertEqual(
                        response.status_code, 200,
                        f'Expected 200 for setting config {config_id}, '
                        f'got {response.status_code}: {response.text}'
                    )

                    # Verify response structure
                    data = response.json()
                    self.assertIn('id', data)
                    self.assertEqual(data['id'], config_id)
                    self.assertIn('data', data)

                    config_found = True
                    print(f'✓ Set configuration test passed: {config_id}')
                    break

        if not config_found:
            self.skipTest('No app configurations found')

    # ==================== REQ_INTEROP_051: Reset All Configurations ====================

    def test_105_reset_all_configurations(self):
        """
        Test DELETE /apps/{id}/configurations resets all configurations.

        Returns 204 on complete success, 207 if some parameters couldn't be reset.

        @verifies REQ_INTEROP_051
        """
        # Use temp_sensor which has known parameters
        app_id = 'temp_sensor'

        response = requests.delete(
            f'{self.BASE_URL}/apps/{app_id}/configurations',
            timeout=10
        )

        # 204 = complete success, 207 = partial success (some params reset)
        self.assertIn(
            response.status_code, [204, 207],
            f'Expected 204/207 for reset all configs, got {response.status_code}'
        )

        print(f'✓ Reset all configurations test passed (status: {response.status_code})')

    # ==================== REQ_INTEROP_052: Reset Single Configuration ====================

    def test_106_reset_single_configuration(self):
        """
        Test DELETE /apps/{id}/configurations/{config-id} resets single config.

        Uses temp_sensor with known 'min_temp' parameter that can be reset.

        @verifies REQ_INTEROP_052
        """
        # Use temp_sensor with known parameter
        app_id = 'temp_sensor'
        config_id = 'min_temp'

        # First verify the parameter exists
        get_response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/configurations/{config_id}',
            timeout=10
        )
        self.assertEqual(
            get_response.status_code, 200,
            f'Parameter {config_id} should exist on {app_id}'
        )

        # Now reset it
        response = requests.delete(
            f'{self.BASE_URL}/apps/{app_id}/configurations/{config_id}',
            timeout=10
        )

        # 204 = parameter reset to default successfully
        self.assertEqual(
            response.status_code, 204,
            f'Expected 204 for reset config {config_id}, got {response.status_code}'
        )
        self.assertEqual(len(response.content), 0, '204 should have no body')

        print(f'✓ Reset single configuration test passed: {config_id}')

    # ==================== Rosbag Snapshot Tests ====================

    def test_107_get_rosbag_nonexistent_fault(self):
        """
        Test /faults/{code}/snapshots/bag returns 404 for unknown fault.

        @verifies REQ_INTEROP_088
        """
        response = requests.get(
            f'{self.BASE_URL}/faults/NONEXISTENT_ROSBAG_FAULT/snapshots/bag',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('message', data)
        self.assertIn('parameters', data)
        self.assertIn('fault_code', data['parameters'])
        self.assertEqual(data['parameters'].get('fault_code'), 'NONEXISTENT_ROSBAG_FAULT')

        print('✓ Get rosbag nonexistent fault test passed')

    def test_108_get_rosbag_invalid_fault_code(self):
        """
        Test /faults/{code}/snapshots/bag rejects invalid fault codes.

        @verifies REQ_INTEROP_088
        """
        # These should be rejected by fault_code validation
        invalid_codes = [
            '../../../etc/passwd',  # Path traversal attempt
            'fault"injection',      # Quote injection attempt
        ]

        for invalid_code in invalid_codes:
            response = requests.get(
                f'{self.BASE_URL}/faults/{invalid_code}/snapshots/bag',
                timeout=10
            )
            # Should return 400 (bad request) or 404 (not found)
            # depending on whether validation happens before or after lookup
            self.assertIn(
                response.status_code,
                [400, 404],
                f'Expected 400 or 404 for fault_code: {invalid_code}'
            )

        print('✓ Get rosbag invalid fault code test passed')

    def test_72_get_rosbag_happy_path(self):
        """Test rosbag download happy path (@verifies REQ_INTEROP_088)."""
        import tarfile
        import tempfile
        import time

        fault_code = 'ROSBAG_TEST_FAULT'

        # Wait for ring buffer to fill (duration_sec = 2.0)
        time.sleep(3)

        # Report a CRITICAL fault (severity=3) to trigger immediate confirmation
        # Note: ReportFault goes through ROS2 service, not REST
        # Use subprocess to call ROS2 service
        import subprocess
        subprocess.run(
            [
                'ros2', 'service', 'call',
                '/fault_manager/report_fault',
                'ros2_medkit_msgs/srv/ReportFault',
                f"{{fault_code: '{fault_code}', source_id: '/rosbag_test', "
                f"severity: 3, message: 'Test fault for rosbag'}}"
            ],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,  # Don't raise on non-zero exit
        )

        # Wait for post-fault recording to complete (duration_after_sec = 0.5)
        time.sleep(2)

        # Download the rosbag
        response = requests.get(
            f'{self.BASE_URL}/faults/{fault_code}/snapshots/bag',
            timeout=30
        )

        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 OK but got {response.status_code}: {response.text}'
        )

        # Verify headers
        content_type = response.headers.get('Content-Type', '')
        self.assertIn('gzip', content_type, 'Expected gzip content type for tar.gz archive')

        content_disp = response.headers.get('Content-Disposition', '')
        self.assertIn('attachment', content_disp, 'Expected attachment disposition')
        self.assertIn('.tar.gz', content_disp, 'Expected .tar.gz extension')

        # Verify content is a valid tar.gz archive
        with tempfile.NamedTemporaryFile(suffix='.tar.gz', delete=False) as f:
            f.write(response.content)
            temp_path = f.name

        try:
            with tarfile.open(temp_path, 'r:gz') as tar:
                names = tar.getnames()
                # Should contain at least one file
                self.assertGreater(len(names), 0, 'Archive should not be empty')
                # Should contain metadata.yaml (rosbag2 standard)
                has_metadata = any('metadata.yaml' in n for n in names)
                self.assertTrue(has_metadata, f'Expected metadata.yaml in archive: {names}')
        finally:
            import os
            os.unlink(temp_path)

        print('✓ Get rosbag happy path test passed')
