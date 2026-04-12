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

"""Integration tests for cross-ECU resource aggregation via peer fan-out.

Launches TWO hybrid-mode gateways in separate DDS domains with a shared
cross-ECU function 'vehicle_health' defined in manifests:

- Primary (port 0, domain 0): aggregation enabled, manages powertrain nodes
  (temp_sensor, rpm_sensor, calibration service).
- Secondary (port 1, domain 1): standalone, manages chassis + perception nodes
  (pressure_sensor, actuator, lidar_sensor with faulty params).

Both define the same function 'vehicle_health' with different hosts.
Tests verify that querying per-entity resource collections (logs, data,
operations, faults, configurations) on the primary returns results from
BOTH gateways via AggregationManager::fan_out_get.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    GATEWAY_STARTUP_INTERVAL,
    GATEWAY_STARTUP_TIMEOUT,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
)


PRIMARY_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
PRIMARY_URL = f'http://localhost:{PRIMARY_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

PRIMARY_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

PRIMARY_NODES = ['temp_sensor', 'rpm_sensor', 'calibration']
PEER_NODES = ['pressure_sensor', 'actuator', 'lidar_sensor']

FUNC_ENDPOINT = '/functions/vehicle_health'

PRIMARY_MANIFEST = """\
manifest_version: "1.0"
metadata:
  name: "Primary ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: powertrain
    name: "Powertrain"
components:
  - id: primary-ecu
    name: "Primary ECU"
    area: powertrain
apps:
  - id: temp_sensor
    name: "Temperature Sensor"
    is_located_on: primary-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: /powertrain/engine
  - id: rpm_sensor
    name: "RPM Sensor"
    is_located_on: primary-ecu
    ros_binding:
      node_name: rpm_sensor
      namespace: /powertrain/engine
  - id: calibration
    name: "Calibration Service"
    is_located_on: primary-ecu
    ros_binding:
      node_name: calibration
      namespace: /powertrain/engine
functions:
  - id: vehicle_health
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - temp_sensor
      - rpm_sensor
      - calibration
"""

PEER_MANIFEST = """\
manifest_version: "1.0"
metadata:
  name: "Secondary ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: chassis
    name: "Chassis"
components:
  - id: secondary-ecu
    name: "Secondary ECU"
    area: chassis
apps:
  - id: pressure_sensor
    name: "Pressure Sensor"
    is_located_on: secondary-ecu
    ros_binding:
      node_name: pressure_sensor
      namespace: /chassis/brakes
  - id: actuator
    name: "Brake Actuator"
    is_located_on: secondary-ecu
    ros_binding:
      node_name: actuator
      namespace: /chassis/brakes
  - id: lidar_sensor
    name: "Lidar Sensor"
    is_located_on: secondary-ecu
    ros_binding:
      node_name: lidar_sensor
      namespace: /perception/lidar
functions:
  - id: vehicle_health
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - pressure_sensor
      - actuator
      - lidar_sensor
"""


def _write_manifest(content):
    """Write manifest YAML to a temporary file, return its path."""
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_manifest_')
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


def generate_test_description():
    primary_manifest_path = _write_manifest(PRIMARY_MANIFEST)
    peer_manifest_path = _write_manifest(PEER_MANIFEST)

    peer_domain_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}

    primary_gateway = create_gateway_node(
        port=PRIMARY_PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': primary_manifest_path,
            'discovery.manifest_strict_validation': False,
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': ['secondary_gateway'],
        },
    )

    peer_gateway = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='secondary_gateway_node',
        output='screen',
        parameters=[{
            'refresh_interval_ms': 1000,
            'server.port': PEER_PORT,
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': peer_manifest_path,
            'discovery.manifest_strict_validation': False,
        }],
        additional_env=peer_domain_env,
    )

    primary_fault_mgr = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{'storage_type': 'memory'}],
    )

    peer_fault_mgr = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{'storage_type': 'memory'}],
        additional_env=peer_domain_env,
    )

    primary_demo_nodes = create_demo_nodes(PRIMARY_NODES, lidar_faulty=False)
    peer_demo_nodes = create_demo_nodes(
        PEER_NODES, lidar_faulty=True, extra_env=peer_domain_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=(
            primary_demo_nodes
            + peer_demo_nodes
            + [primary_fault_mgr, peer_fault_mgr]
        ),
    )

    launch_description = LaunchDescription([
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(PRIMARY_DOMAIN_ID)),
        primary_gateway,
        peer_gateway,
        delayed,
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {'gateway_node': primary_gateway, 'peer_gateway': peer_gateway},
    )


# ======================================================================
# Helpers
# ======================================================================

def _get_json(url, timeout=10):
    """GET and return parsed JSON, or None on failure."""
    try:
        r = requests.get(url, timeout=timeout)
        if r.status_code == 200:
            return r.json()
    except requests.exceptions.RequestException:
        pass
    return None


def _poll_until(url, condition, *, timeout=30.0, interval=1.0):
    """Poll url until condition(json) returns truthy."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = _get_json(url)
        if data is not None:
            result = condition(data)
            if result:
                return result
        time.sleep(interval)
    return None


class TestCrossEcuFanout(unittest.TestCase):
    """Verify cross-ECU resource fan-out for all collection types."""

    @classmethod
    def setUpClass(cls):
        """Wait for health, discovery, and resource availability."""
        cls._wait_for_health(PRIMARY_URL, 'Primary')
        cls._wait_for_health(PEER_URL, 'Secondary')

        cls._wait_for_apps(
            PEER_URL,
            {'pressure_sensor', 'actuator', 'lidar_sensor'},
            'Secondary',
        )
        cls._wait_for_apps(
            PRIMARY_URL,
            {
                'temp_sensor', 'rpm_sensor', 'calibration',
                'pressure_sensor', 'actuator', 'lidar_sensor',
            },
            'Primary (merged)',
        )

        cls._wait_for_items(
            f'{PRIMARY_URL}/apps/temp_sensor/logs',
            'Primary logs',
        )
        cls._wait_for_items(
            f'{PEER_URL}/apps/pressure_sensor/logs',
            'Peer logs',
        )

    @classmethod
    def _wait_for_health(cls, base_url, label):
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            try:
                r = requests.get(f'{base_url}/health', timeout=2)
                if r.status_code == 200:
                    print(f'{label} gateway is healthy')
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(GATEWAY_STARTUP_INTERVAL)
        raise AssertionError(
            f'{label} gateway not responding after {GATEWAY_STARTUP_TIMEOUT}s'
        )

    @classmethod
    def _wait_for_apps(cls, base_url, required_apps, label):
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        while time.monotonic() < deadline:
            try:
                r = requests.get(f'{base_url}/apps', timeout=5)
                if r.status_code == 200:
                    found = {
                        a.get('id', '') for a in r.json().get('items', [])
                    }
                    missing = required_apps - found
                    if not missing:
                        print(f'{label}: all apps discovered ({len(found)})')
                        return
                    print(f'  {label}: waiting for {missing}')
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(
            f'{label}: apps not discovered after {DISCOVERY_TIMEOUT}s'
        )

    @classmethod
    def _wait_for_items(cls, url, label, timeout=30.0):
        """Poll endpoint until items array is non-empty."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data = _get_json(url, timeout=5)
            if data and data.get('items'):
                print(f'{label}: {len(data["items"])} items available')
                return
            time.sleep(1.0)
        raise AssertionError(f'{label}: no items after {timeout}s at {url}')

    # ------------------------------------------------------------------
    # Logs fan-out
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_061
    def test_logs_include_peer_entries(self):
        """Function logs on primary include entries from peer apps."""
        data = _get_json(f'{PRIMARY_URL}{FUNC_ENDPOINT}/logs')
        self.assertIsNotNone(data)
        items = data.get('items', [])

        nodes = {
            e['context']['node'] for e in items
            if 'node' in e.get('context', {})
        }

        self.assertTrue(
            any('powertrain.engine' in n for n in nodes),
            f'Missing local log nodes: {nodes}',
        )
        self.assertTrue(
            any('chassis.brakes' in n or 'perception.lidar' in n
                for n in nodes),
            f'Missing peer log nodes via fan-out: {nodes}',
        )

    def test_logs_have_aggregation_metadata(self):
        """Function logs response includes x-medkit aggregation info."""
        data = _get_json(f'{PRIMARY_URL}{FUNC_ENDPOINT}/logs')
        self.assertIsNotNone(data)
        ext = data.get('x-medkit', {})
        self.assertEqual(ext.get('aggregation_level'), 'function')
        self.assertTrue(ext.get('aggregated'))

    # @verifies REQ_INTEROP_061
    def test_logs_severity_filter_forwarded(self):
        """Severity filter is forwarded to peer via query params."""
        data = _get_json(
            f'{PRIMARY_URL}{FUNC_ENDPOINT}/logs?severity=fatal'
        )
        self.assertIsNotNone(data)
        for entry in data.get('items', []):
            self.assertEqual(entry['severity'], 'fatal')

    # ------------------------------------------------------------------
    # Data fan-out
    # ------------------------------------------------------------------

    def test_data_include_peer_topics(self):
        """Function data on primary includes topics from peer apps.

        Primary hosts temp_sensor (temperature topic) and rpm_sensor.
        Peer hosts pressure_sensor (pressure topic) and lidar (scan topic).
        Fan-out merges peer data items into the response.
        """
        data = _poll_until(
            f'{PRIMARY_URL}{FUNC_ENDPOINT}/data',
            lambda d: d if d.get('items') else None,
            timeout=30.0,
        )
        self.assertIsNotNone(data, 'No data items for vehicle_health')
        items = data['items']

        topic_paths = set()
        for item in items:
            ext = item.get('x-medkit', {})
            ros2 = ext.get('ros2', {})
            topic = ros2.get('topic', '')
            if topic:
                topic_paths.add(topic)

        has_local = any('/powertrain/engine/' in t for t in topic_paths)
        has_peer = any(
            '/chassis/brakes/' in t or '/perception/lidar/' in t
            for t in topic_paths
        )

        self.assertTrue(
            has_local,
            f'Missing local data topics: {topic_paths}',
        )
        self.assertTrue(
            has_peer,
            f'Missing peer data topics via fan-out: {topic_paths}',
        )

    # ------------------------------------------------------------------
    # Operations fan-out
    # ------------------------------------------------------------------

    def test_operations_include_peer_operations(self):
        """Function operations on primary includes peer services.

        Primary hosts calibration service (/powertrain/engine/calibrate).
        Peer hosts lidar calibrate service (/perception/lidar/calibrate).
        Fan-out merges peer operations into the response.
        """
        data = _poll_until(
            f'{PRIMARY_URL}{FUNC_ENDPOINT}/operations',
            lambda d: d if d.get('items') else None,
            timeout=30.0,
        )
        self.assertIsNotNone(
            data, 'No operations found for vehicle_health function'
        )
        items = data['items']

        svc_paths = set()
        for item in items:
            ext = item.get('x-medkit', {})
            svc = ext.get('ros2', {}).get('service', '')
            if svc:
                svc_paths.add(svc)

        has_local = any('/powertrain/engine/' in s for s in svc_paths)
        has_peer = any('/perception/lidar/' in s for s in svc_paths)

        self.assertTrue(
            has_local,
            f'Missing local operation: {svc_paths}',
        )
        self.assertTrue(
            has_peer,
            f'Missing peer operation via fan-out: {svc_paths}',
        )

    # ------------------------------------------------------------------
    # Faults fan-out
    # ------------------------------------------------------------------

    def test_faults_include_peer_faults(self):
        """Function faults on primary includes peer lidar faults.

        The peer lidar_sensor runs with faulty params (min_range >= max_range),
        triggering LIDAR_RANGE_INVALID. The primary has no local faults.
        Fan-out merges peer faults into the response.
        """
        data = _poll_until(
            f'{PRIMARY_URL}{FUNC_ENDPOINT}/faults',
            lambda d: d if d.get('items') else None,
            timeout=45.0,
        )
        self.assertIsNotNone(
            data, 'No faults found via fan-out for vehicle_health'
        )
        items = data['items']

        fault_codes = {
            f.get('fault_code', f.get('code', '')) for f in items
        }
        self.assertTrue(
            len(fault_codes) > 0,
            f'Expected peer faults, got: {items}',
        )

    # ------------------------------------------------------------------
    # Configurations fan-out
    # ------------------------------------------------------------------

    def test_configs_include_peer_params(self):
        """Function configs on primary includes peer app parameters.

        Primary hosts temp_sensor (publish_rate, min_temp, etc.).
        Peer hosts lidar_sensor (min_range, max_range, scan_frequency, etc.).
        Fan-out merges peer configuration items into the response.
        """
        data = _poll_until(
            f'{PRIMARY_URL}{FUNC_ENDPOINT}/configurations',
            lambda d: d if d.get('items') else None,
            timeout=30.0,
        )
        self.assertIsNotNone(
            data, 'No configurations found for vehicle_health'
        )
        items = data['items']

        param_ids = {item.get('id', '') for item in items}

        has_local = any('temp_sensor' in pid for pid in param_ids)
        has_peer = any(
            'lidar_sensor' in pid or 'pressure_sensor' in pid
            or 'actuator' in pid
            for pid in param_ids
        )

        self.assertTrue(
            has_local,
            f'Missing local config params: {param_ids}',
        )
        self.assertTrue(
            has_peer,
            f'Missing peer config params via fan-out: {param_ids}',
        )

    # ------------------------------------------------------------------
    # Peer isolation (sanity check)
    # ------------------------------------------------------------------

    def test_peer_logs_are_local_only(self):
        """Secondary function logs contain only local entries."""
        data = _get_json(f'{PEER_URL}{FUNC_ENDPOINT}/logs')
        self.assertIsNotNone(data)
        for entry in data.get('items', []):
            node = entry.get('context', {}).get('node', '')
            self.assertNotIn(
                'powertrain.engine', node,
                f'Secondary should not have primary logs: {node}',
            )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
