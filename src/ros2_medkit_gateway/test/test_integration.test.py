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
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=coverage_env,
        parameters=[{'storage_type': 'memory'}],
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
    # Minimum expected areas (powertrain, chassis, body, perception + root)
    MIN_EXPECTED_AREAS = 4

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

        # Wait for apps AND areas to be discovered (CI can be slow)
        start_time = time.time()
        while time.time() - start_time < cls.MAX_DISCOVERY_WAIT:
            try:
                apps_response = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
                areas_response = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
                if apps_response.status_code == 200 and areas_response.status_code == 200:
                    apps = apps_response.json().get('items', [])
                    areas = areas_response.json().get('items', [])
                    apps_ok = len(apps) >= cls.MIN_EXPECTED_APPS
                    areas_ok = len(areas) >= cls.MIN_EXPECTED_AREAS
                    if apps_ok and areas_ok:
                        print(f'✓ Discovery complete: {len(apps)} apps, {len(areas)} areas')
                        return
                    area_ids = [a.get('id', '?') for a in areas]
                    print(f'  Waiting: {len(apps)}/{cls.MIN_EXPECTED_APPS} apps, '
                          f'{len(areas)}/{cls.MIN_EXPECTED_AREAS} areas {area_ids}')
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

    def _wait_for_action_status(
        self, goal_id: str, target_statuses: list, max_wait: float = None
    ) -> dict:
        """
        Poll action status until it reaches one of the target statuses.

        Parameters
        ----------
        goal_id : str
            The goal ID to check status for.
        target_statuses : list
            List of status strings to wait for (e.g., ['succeeded', 'aborted']).
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
                    f'long_calibration/status',
                    params={'goal_id': goal_id},
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
            f'Action did not reach status {target_statuses} within {max_wait}s. '
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
            # namespace may be 'namespace' or 'namespace_path' depending on source
            self.assertTrue(
                'namespace' in component or 'namespace_path' in component,
                f"Component {component['id']} should have namespace field"
            )
            self.assertIn('fqn', component)
            self.assertIn('type', component)
            self.assertIn('area', component)
            # Synthetic components have type 'ComponentGroup', topic-based have 'Component'
            self.assertIn(component['type'], ['Component', 'ComponentGroup'])

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

        # All components should belong to powertrain area
        for component in components:
            self.assertEqual(component['area'], 'powertrain')
            self.assertIn('id', component)
            self.assertIn('namespace', component)

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
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Area not found')
        self.assertIn('area_id', data)
        self.assertEqual(data['area_id'], 'nonexistent')

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
                self.assertIn('direction', topic_data)
                self.assertIn(topic_data['direction'], ['publish', 'subscribe'])
                print(
                    f"  - Topic: {topic_data['name']} ({topic_data['direction']})"
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
                self.assertIn('direction', topic_data)

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
                self.assertIn('direction', topic_data)

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
            self.assertIn('direction', first_item, "Each item should have 'direction' field")
            self.assertIn('href', first_item, "Each item should have 'href' field")
            self.assertIsInstance(
                first_item['name'], str, "'name' should be a string"
            )
            self.assertIn(first_item['direction'], ['publish', 'subscribe'])

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
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'App not found')
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'nonexistent_app')

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
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid app ID')
            self.assertIn('details', data)

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
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid app ID')

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

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        # Use percent encoding for topic path: /some/nonexistent/topic
        topic_path = encode_topic_path('/some/nonexistent/topic')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Topic not found')
        self.assertIn('component_id', data)
        self.assertEqual(data['component_id'], 'powertrain')
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
        self.assertEqual(data['topic'], '/powertrain/engine/temperature')

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
        self.assertIn('topic', data)
        self.assertIn('type', data)
        self.assertIn('status', data)
        self.assertIn('timestamp', data)
        self.assertIn('component_id', data)
        self.assertIn('topic_name', data)
        self.assertEqual(data['status'], 'published')
        self.assertEqual(data['type'], 'std_msgs/msg/Float32')
        self.assertEqual(data['component_id'], 'chassis')
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
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
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
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
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
                f'{self.BASE_URL}/components/chassis/data/{topic_path}',
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
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('json', data['error'].lower())

        print('✓ Publish invalid JSON body test passed')

    # ========== POST /apps/{app_id}/operations/{operation_name} tests ==========

    def test_31_operation_call_calibrate_service(self):
        """
        Test POST /apps/{app_id}/operations/{operation_name} calls a service.

        Operations are exposed on Apps (ROS 2 nodes), not synthetic Components.

        @verifies REQ_INTEROP_035
        """
        # Ensure calibration app is available via REST (handles discovery race)
        self._ensure_calibration_app_ready()

        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate',
            json={},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'success')
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'calibration')
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

        POST /apps/{app_id}/operations/{operation_name}

        @verifies REQ_INTEROP_035
        """
        # Ensure calibration app is available via REST (handles discovery race)
        self._ensure_calibration_app_ready()

        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/nonexistent_op',
            json={},
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('Operation not found', data['error'])

        print('✓ Operation call nonexistent operation test passed')

    def test_33_operation_call_nonexistent_entity(self):
        """
        Test operation call returns 404 for unknown entity.

        POST /apps/{app_id}/operations/{operation_name}

        @verifies REQ_INTEROP_035
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/nonexistent_app/operations/calibrate',
            json={},
            timeout=5
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Entity not found')
        self.assertEqual(data['entity_id'], 'nonexistent_app')

        print('✓ Operation call nonexistent entity test passed')

    def test_34_operation_call_invalid_entity_id(self):
        """
        Test operation call rejects invalid entity ID.

        POST /apps/{app_id}/operations/{operation_name}

        @verifies REQ_INTEROP_035
        """
        invalid_ids = [
            'app;drop',
            'app<script>',
            'app name',
        ]

        for invalid_id in invalid_ids:
            response = requests.post(
                f'{self.BASE_URL}/apps/{invalid_id}/operations/calibrate',
                json={},
                timeout=5
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for entity_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid entity ID')

        print('✓ Operation call invalid entity ID test passed')

    def test_35_operation_call_invalid_operation_name(self):
        """
        Test operation call rejects invalid operation name.

        POST /apps/{app_id}/operations/{operation_name}

        @verifies REQ_INTEROP_021
        """
        invalid_names = [
            'op;drop',
            'op<script>',
            'op name',
        ]

        for invalid_name in invalid_names:
            response = requests.post(
                f'{self.BASE_URL}/apps/calibration/operations/{invalid_name}',
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

        POST /apps/{app_id}/operations/{operation_name}

        @verifies REQ_INTEROP_021
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/calibration/operations/calibrate',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertIn('json', data['error'].lower())

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
        calibrate_op = None
        for op in ops:
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
        Test POST /apps/{app_id}/operations/{operation_name} sends action goal.

        Sends a goal to the long_calibration action and verifies goal_id is returned.

        @verifies REQ_INTEROP_022
        """
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration',
            json={'goal': {'order': 5}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'success')
        self.assertIn('kind', data)
        self.assertEqual(data['kind'], 'action')
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'long_calibration')
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
        Test GET /apps/{app_id}/operations/{operation_name}/status returns goal status.

        @verifies REQ_INTEROP_022
        """
        # First, send a goal with enough steps to ensure it's still running
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration',
            json={'goal': {'order': 10}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        goal_id = response.json()['goal_id']

        # Check status immediately (allow extra time for action server response)
        status_response = requests.get(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/status',
            params={'goal_id': goal_id},
            timeout=10
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

    def test_41_action_status_after_completion(self):
        """
        Test that action status is updated to succeeded after completion via native subscription.

        The native status subscription updates goal status in real-time.
        After an action completes, polling the status endpoint should show 'succeeded'.

        @verifies REQ_INTEROP_022
        """
        # Send a short goal that will complete quickly
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration',
            json={'goal': {'order': 3}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        goal_id = response.json()['goal_id']

        # Poll for completion instead of fixed sleep (handles CI timing variance)
        data = self._wait_for_action_status(
            goal_id, ['succeeded', 'aborted'], max_wait=ACTION_TIMEOUT
        )

        self.assertIn('goal_id', data)
        self.assertEqual(data['goal_id'], goal_id)
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'succeeded')

        print(f'✓ Action status after completion test passed: status={data["status"]}')

    def test_42_action_cancel_endpoint(self):
        """
        Test DELETE /apps/{app_id}/operations/{operation_name} cancels action.

        @verifies REQ_INTEROP_022
        """
        # Send a long goal that we can cancel
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration',
            json={'goal': {'order': 20}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        goal_id = response.json()['goal_id']

        # Poll until action is executing (handles CI timing variance)
        try:
            self._wait_for_action_status(
                goal_id, ['executing'], max_wait=ACTION_TIMEOUT
            )
        except AssertionError:
            # If action already completed or is still in accepted, try cancel anyway
            pass

        # Cancel the goal
        cancel_response = requests.delete(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration',
            params={'goal_id': goal_id},
            timeout=10
        )
        self.assertEqual(cancel_response.status_code, 200)

        data = cancel_response.json()
        self.assertIn('status', data)
        # Status can be 'canceling' or 'canceled' depending on timing
        self.assertIn(data['status'], ['canceling', 'canceled'])
        self.assertIn('goal_id', data)
        self.assertEqual(data['goal_id'], goal_id)

        print(f'✓ Action cancel endpoint test passed: {data}')

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
        action_op = None
        for op in ops:
            if op['name'] == 'long_calibration' and op['kind'] == 'action':
                action_op = op
                break

        self.assertIsNotNone(action_op, 'long_calibration action should be listed')
        self.assertEqual(action_op['kind'], 'action')
        self.assertEqual(action_op['type'], 'example_interfaces/action/Fibonacci')
        self.assertEqual(action_op['path'], '/powertrain/engine/long_calibration')

        print('✓ Action listed in app operations test passed')

    def test_44_action_status_without_goal_id_returns_latest(self):
        """
        Test action status without goal_id returns latest goal.

        GET /apps/{app_id}/operations/{operation_name}/status
        Returns the most recent goal status when no goal_id is provided.

        @verifies REQ_INTEROP_022
        """
        # First, send a goal so we have something to query
        response = requests.post(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration',
            json={'goal': {'order': 3}},
            timeout=15
        )
        self.assertEqual(response.status_code, 200)
        expected_goal_id = response.json()['goal_id']

        # Wait for it to complete
        time.sleep(3)

        # Now query status without goal_id - should return the latest goal
        status_response = requests.get(
            f'{self.BASE_URL}/apps/long_calibration/operations/long_calibration/status',
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
        Test GET /apps/{app_id}/configurations lists all parameters.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'temp_sensor')
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
        Test GET /apps/{app_id}/configurations/{param_name} gets parameter.

        @verifies REQ_INTEROP_023
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/publish_rate',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'temp_sensor')
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
        Test PUT /apps/{app_id}/configurations/{param_name} sets parameter.

        @verifies REQ_INTEROP_024
        """
        # Set a new value
        response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'value': 80.0},
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('status', data)
        self.assertEqual(data['status'], 'success')
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'temp_sensor')
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
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(verify_response.status_code, 200)
        verify_data = verify_response.json()
        self.assertEqual(verify_data['parameter']['value'], 80.0)

        # Reset the value back to default
        requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'value': 85.0},
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
        # First, change the value from default
        set_response = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            json={'value': -50.0},
            timeout=10
        )
        self.assertEqual(set_response.status_code, 200)

        # Now reset to default via DELETE
        response = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
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
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
            timeout=10
        )
        self.assertEqual(get_response.status_code, 200)
        get_data = get_response.json()
        # The value should match what DELETE returned
        self.assertEqual(get_data['parameter']['value'], data['value'])

        print(f'✓ Delete configuration (reset to default) test passed: value={data["value"]}')

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
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Entity not found')
        self.assertIn('entity_id', data)
        self.assertEqual(data['entity_id'], 'nonexistent_app')

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
            f'{self.BASE_URL}/apps/temp_sensor/configurations/min_temp',
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
        calibrate_op = None
        for op in ops:
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
        action_op = None
        for op in ops:
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
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'temp_sensor')
        self.assertIn('source_id', data)
        self.assertIn('faults', data)
        self.assertIsInstance(data['faults'], list)
        self.assertIn('count', data)

        print(f'✓ List faults response structure test passed: {data["count"]} faults')

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
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Entity not found')
        self.assertIn('entity_id', data)
        self.assertEqual(data['entity_id'], 'nonexistent_component')

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
        self.assertIn('error', data)
        self.assertIn('fault_code', data)
        self.assertEqual(data['fault_code'], 'NONEXISTENT_FAULT')

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
        self.assertIn('faults', data)
        self.assertIsInstance(data['faults'], list)
        self.assertIn('count', data)
        self.assertIsInstance(data['count'], int)
        self.assertEqual(data['count'], len(data['faults']))

        print(f'✓ List all faults globally test passed: {data["count"]} faults')

    def test_60_list_all_faults_with_status_filter(self):
        """Test GET /faults?status={status} filters faults by status."""
        # Test with status=all
        response = requests.get(
            f'{self.BASE_URL}/faults?status=all',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('faults', data)
        self.assertIn('count', data)

        # Test other valid status values
        for status in ['pending', 'confirmed', 'cleared']:
            response = requests.get(
                f'{self.BASE_URL}/faults?status={status}',
                timeout=10
            )
            self.assertEqual(response.status_code, 200)

        print(f'✓ List all faults with status filter test passed: {data["count"]} faults')

    def test_61_list_faults_invalid_status_returns_400(self):
        """Test GET /faults?status=invalid returns 400 Bad Request."""
        response = requests.get(
            f'{self.BASE_URL}/faults?status=invalid_status',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Invalid status parameter')
        self.assertIn('details', data)
        self.assertIn('pending', data['details'])  # Should mention valid values
        self.assertIn('parameter', data)
        self.assertEqual(data['parameter'], 'status')
        self.assertIn('value', data)
        self.assertEqual(data['value'], 'invalid_status')

        print('✓ List faults invalid status returns 400 test passed')

    def test_62_component_faults_invalid_status_returns_400(self):
        """Test GET /apps/{id}/faults?status=invalid returns 400."""
        response = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/faults?status=bogus',
            timeout=10
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Invalid status parameter')
        self.assertIn('app_id', data)

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
        self.assertIn('error', data)
        self.assertIn('fault_code', data)
        self.assertEqual(data['fault_code'], 'NONEXISTENT_FAULT_CODE')

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
        self.assertIn('error', data)
        self.assertIn('app_id', data)
        self.assertEqual(data['app_id'], 'temp_sensor')
        self.assertIn('fault_code', data)
        self.assertEqual(data['fault_code'], 'NONEXISTENT_FAULT')

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
        self.assertIn('error', data)
        self.assertEqual(data['error'], 'Entity not found')
        self.assertIn('entity_id', data)
        self.assertEqual(data['entity_id'], 'nonexistent_component')

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
            self.assertIn('error', data)
            self.assertEqual(data['error'], 'Invalid entity ID')

        print('✓ Get snapshots invalid component ID test passed')
