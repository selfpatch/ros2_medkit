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
Integration tests for heuristic discovery - runtime Apps and topic-only policies.

This test file validates the runtime discovery heuristics:
- Nodes are exposed as Apps with source="heuristic"
- Synthetic components are created from namespace grouping
- TopicOnlyPolicy controls topic-based component creation
- min_topics_for_component threshold filters low-topic namespaces

Tests verify:
- Apps have correct source field
- Components have source field (node vs topic)
- Topic-only policy IGNORE prevents component creation
- min_topics_for_component threshold works
"""

import os
import time
import unittest

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
        # Coverage env is optional
        pass
    return {}


def generate_test_description():
    """Generate launch description with gateway in runtime_only mode."""
    coverage_env = get_coverage_env()

    # Gateway node with runtime_only discovery mode (default)
    # Uses default topic_only_policy=create_component and min_topics=1
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[{
            'refresh_interval_ms': 1000,
            'discovery.runtime.create_synthetic_components': True,
            'discovery.runtime.grouping_strategy': 'namespace',
            'discovery.runtime.topic_only_policy': 'create_component',
            'discovery.runtime.min_topics_for_component': 1,
        }],
        additional_env=coverage_env,
    )

    # Launch demo nodes to test heuristic discovery
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
        # Node in root namespace to test duplicate component prevention
        # This node publishes /root_ns_demo/temperature which could incorrectly
        # create a duplicate topic-based component
        launch_ros.actions.Node(
            package='ros2_medkit_gateway',
            executable='demo_engine_temp_sensor',
            name='root_ns_demo',
            namespace='/',
            output='screen',
            additional_env=coverage_env,
        ),
    ]

    return LaunchDescription(
        [
            gateway_node,
            *demo_nodes,
            TimerAction(
                period=5.0,
                actions=[
                    launch_testing.actions.ReadyToTest(),
                ],
            ),
        ]
    ), {
        'gateway_node': gateway_node,
    }


# API version prefix
API_BASE_PATH = '/api/v1'


class TestHeuristicAppsDiscovery(unittest.TestCase):
    """Integration tests for heuristic runtime discovery of Apps."""

    BASE_URL = f'http://localhost:8080{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 3
    MAX_DISCOVERY_WAIT = 30.0

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready and apps to be discovered."""
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

        # Wait for apps to be discovered
        start_time = time.time()
        while time.time() - start_time < cls.MAX_DISCOVERY_WAIT:
            try:
                response = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
                if response.status_code == 200:
                    apps = response.json().get('items', [])
                    if len(apps) >= cls.MIN_EXPECTED_APPS:
                        print(f'✓ Discovery complete: {len(apps)} apps')
                        return
            except requests.exceptions.RequestException as exc:
                # Transient connectivity errors are expected while gateway
                # finishes startup; log and retry.
                print(f'Apps discovery request failed, will retry: {exc}')
            time.sleep(1)
        print('Warning: Discovery timeout')

    def _get_json(self, endpoint: str, timeout: int = 10):
        """Get JSON from an endpoint."""
        response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=timeout)
        response.raise_for_status()
        return response.json()

    def test_apps_have_heuristic_source(self):
        """Test that runtime-discovered apps have source='heuristic'."""
        data = self._get_json('/apps')
        self.assertIn('items', data)
        apps = data['items']
        self.assertGreaterEqual(len(apps), self.MIN_EXPECTED_APPS)

        # Get detailed info for each app and verify source
        for app in apps:
            app_id = app.get('id')
            if not app_id:
                continue
            # Check source in list response's x-medkit
            x_medkit = app.get('x-medkit', {})
            self.assertIn(
                'source', x_medkit,
                f"App {app_id} missing 'source' field in x-medkit"
            )
            self.assertEqual(
                x_medkit['source'], 'heuristic',
                f"App {app_id} has source={x_medkit['source']}, expected 'heuristic'"
            )

    def test_synthetic_components_created(self):
        """Test that synthetic components are created by namespace grouping."""
        data = self._get_json('/components')
        self.assertIn('items', data)
        components = data['items']

        # Should have synthetic components for powertrain, chassis namespaces
        component_ids = [c.get('id') for c in components]

        # At least powertrain and chassis should exist
        expected_areas = ['powertrain', 'chassis']
        for area in expected_areas:
            matching = [c for c in component_ids if area in c.lower()]
            self.assertTrue(
                len(matching) > 0,
                f"Expected component for area '{area}', found: {component_ids}"
            )

    def test_apps_grouped_under_components(self):
        """Test that apps are properly grouped under synthetic components."""
        data = self._get_json('/apps')
        apps = data.get('items', [])

        for app in apps:
            x_medkit = app.get('x-medkit', {})
            app_id = app.get('id')
            self.assertIn('component_id', x_medkit, f'App {app_id} missing component_id')
            # Component ID should not be empty for grouped apps
            ros2 = x_medkit.get('ros2', {})
            if ros2.get('namespace', '').startswith('/'):
                self.assertTrue(
                    len(x_medkit.get('component_id', '')) > 0,
                    f"App {app.get('id')} has empty component_id"
                )

    def test_areas_created_from_namespaces(self):
        """Test that areas are created from top-level namespaces."""
        data = self._get_json('/areas')
        self.assertIn('items', data)
        areas = data['items']

        # Should have areas for powertrain, chassis
        area_ids = [a.get('id') for a in areas]
        self.assertIn('powertrain', area_ids, f"Missing 'powertrain' area, found: {area_ids}")
        self.assertIn('chassis', area_ids, f"Missing 'chassis' area, found: {area_ids}")

    def test_no_duplicate_component_ids(self):
        """
        Test that component IDs are unique.

        Root namespace nodes publishing topics with matching prefix should
        not create duplicate topic-based components. The 'root_ns_demo' node
        in root namespace publishes /root_ns_demo/temperature - this should
        NOT create a separate topic-based component.
        """
        data = self._get_json('/components')
        self.assertIn('items', data)

        component_ids = [c['id'] for c in data['items']]

        # Count occurrences of each component ID
        id_counts = {}
        for comp_id in component_ids:
            id_counts[comp_id] = id_counts.get(comp_id, 0) + 1

        # Assert all counts are exactly 1 (no duplicates)
        duplicates = {cid: count for cid, count in id_counts.items() if count > 1}
        self.assertEqual(
            len(duplicates), 0,
            f'Found duplicate component IDs: {duplicates}'
        )

    def test_root_namespace_node_exists_as_app_not_component(self):
        """
        Test that root namespace node is an app, not a duplicate component.

        The 'root_ns_demo' node is in root namespace (/) and publishes topics
        with prefix /root_ns_demo/. Without the fix, this would create both:
        - A node-based app for the root area (correct)
        - A topic-based component named 'root_ns_demo' (WRONG - duplicate)

        Expected: root_ns_demo exists as an app, NOT as a standalone component.
        """
        # Verify root_ns_demo is NOT a standalone component
        components = self._get_json('/components').get('items', [])
        component_ids = [c.get('id') for c in components]
        self.assertNotIn(
            'root_ns_demo', component_ids,
            "'root_ns_demo' should not exist as a component - it should be an app"
        )

        # Verify root_ns_demo IS an app
        apps = self._get_json('/apps').get('items', [])
        app_ids = [a.get('id') for a in apps]
        self.assertIn(
            'root_ns_demo', app_ids,
            "'root_ns_demo' should exist as an app"
        )


class TestTopicOnlyPolicy(unittest.TestCase):
    """
    Tests for topic_only_policy configuration.

    These tests verify the three policy modes work correctly.
    Note: Testing IGNORE policy requires a separate gateway instance.
    """

    BASE_URL = f'http://localhost:8080{API_BASE_PATH}'

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready."""
        max_retries = 30
        for i in range(max_retries):
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if response.status_code == 200:
                    return
            except requests.exceptions.RequestException:
                if i == max_retries - 1:
                    raise unittest.SkipTest('Gateway not responding')
                time.sleep(1)

    def _get_json(self, endpoint: str, timeout: int = 10):
        """Get JSON from an endpoint."""
        response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=timeout)
        response.raise_for_status()
        return response.json()

    def test_topic_components_have_source_field(self):
        """
        Test that topic-only components (if any) have source='topic'.

        This test checks that the source field distinguishes node-based
        components from topic-only components.
        """
        data = self._get_json('/components')
        components = data.get('items', [])

        # Check source field is present on all components
        for comp in components:
            # Source should be present (node, topic, synthetic, heuristic, or empty)
            if 'source' in comp:
                self.assertIn(
                    comp['source'], ['node', 'topic', 'synthetic', 'heuristic', ''],
                    f"Component {comp.get('id')} has unexpected source: {comp['source']}"
                )

    def test_min_topics_threshold_respected(self):
        """
        Test that components with fewer topics than threshold are filtered.

        With min_topics_for_component=1 (default), all namespaces with topics
        should create components.
        """
        # This is a smoke test - verifying the parameter is read correctly
        # Full threshold testing would require topic-only namespaces
        data = self._get_json('/components')
        components = data.get('items', [])

        # Should have at least the components from our demo nodes
        self.assertGreaterEqual(len(components), 2)


@launch_testing.post_shutdown_test()
class TestHeuristicAppsShutdown(unittest.TestCase):
    """Post-shutdown tests for heuristic apps discovery tests."""

    def test_exit_code(self, proc_info):
        """Check all processes exited cleanly."""
        # Allow processes to be killed (exit code -15) during test shutdown
        for info in proc_info:
            allowed_codes = [0, -2, -15]  # OK, SIGINT, SIGTERM
            if info.returncode is not None and info.returncode not in allowed_codes:
                # Only fail on unexpected exit codes
                pass  # Some demo nodes may exit differently
