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

"""Integration tests for cross-ECU log aggregation via peer fan-out.

Launches TWO manifest_only gateways in separate DDS domains with a shared
cross-ECU function 'vehicle_health':

- Primary (port 0, domain 0): aggregation enabled, manages powertrain nodes.
  Manifest defines vehicle_health function hosted by temp_sensor, rpm_sensor.
- Secondary (port 1, domain 1): standalone, manages chassis nodes.
  Manifest defines the same vehicle_health function hosted by pressure_sensor,
  actuator.

Tests verify that querying function logs on the primary returns logs from
BOTH gateways via AggregationManager::fan_out_get, including severity
filter forwarding and x-medkit aggregation metadata.
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

PRIMARY_NODES = ['temp_sensor', 'rpm_sensor']
PEER_NODES = ['pressure_sensor', 'actuator']

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
functions:
  - id: vehicle_health
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - temp_sensor
      - rpm_sensor
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
functions:
  - id: vehicle_health
    name: "Vehicle Health Monitoring"
    category: monitoring
    hosted_by:
      - pressure_sensor
      - actuator
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
            'discovery.mode': 'manifest_only',
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
            'discovery.mode': 'manifest_only',
            'discovery.manifest_path': peer_manifest_path,
            'discovery.manifest_strict_validation': False,
        }],
        additional_env=peer_domain_env,
    )

    primary_demo_nodes = create_demo_nodes(PRIMARY_NODES, lidar_faulty=False)
    peer_demo_nodes = create_demo_nodes(
        PEER_NODES, lidar_faulty=False, extra_env=peer_domain_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=primary_demo_nodes + peer_demo_nodes,
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


class TestLogPeerAggregation(unittest.TestCase):
    """Verify cross-ECU function log aggregation via peer fan-out."""

    @classmethod
    def setUpClass(cls):
        """Wait for both gateways, entity discovery, and log accumulation."""
        cls._wait_for_health(PRIMARY_URL, 'Primary')
        cls._wait_for_health(PEER_URL, 'Secondary')

        cls._wait_for_apps(
            PEER_URL, {'pressure_sensor', 'actuator'}, 'Secondary',
        )
        cls._wait_for_apps(
            PRIMARY_URL,
            {'temp_sensor', 'rpm_sensor', 'pressure_sensor', 'actuator'},
            'Primary (merged)',
        )

        cls._wait_for_logs(
            PEER_URL, '/apps/pressure_sensor/logs',
            'Secondary pressure_sensor',
        )
        cls._wait_for_logs(
            PRIMARY_URL, '/apps/temp_sensor/logs',
            'Primary temp_sensor',
        )

    @classmethod
    def _wait_for_health(cls, base_url, label):
        """Poll /health until a gateway responds with 200."""
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{base_url}/health', timeout=2)
                if response.status_code == 200:
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
        """Poll /apps until all required app IDs are present."""
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{base_url}/apps', timeout=5)
                if response.status_code == 200:
                    items = response.json().get('items', [])
                    found_ids = {a.get('id', '') for a in items}
                    missing = required_apps - found_ids
                    if not missing:
                        print(
                            f'{label}: all apps discovered '
                            f'({len(found_ids)} total)'
                        )
                        return
                    print(
                        f'  {label}: waiting for {missing} '
                        f'(have {found_ids})'
                    )
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(
            f'{label}: apps not discovered after {DISCOVERY_TIMEOUT}s. '
            f'Missing: {required_apps}'
        )

    @classmethod
    def _wait_for_logs(cls, base_url, endpoint, label, timeout=30.0):
        """Poll a logs endpoint until it returns at least one entry."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                response = requests.get(f'{base_url}{endpoint}', timeout=5)
                if response.status_code == 200:
                    items = response.json().get('items', [])
                    if items:
                        print(f'{label}: {len(items)} log entries available')
                        return
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(f'{label}: no log entries after {timeout}s')

    # ------------------------------------------------------------------
    # Cross-ECU function log aggregation
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_061
    def test_function_logs_include_local_entries(self):
        """Primary function logs contain entries from local powertrain apps."""
        response = requests.get(
            f'{PRIMARY_URL}/functions/vehicle_health/logs', timeout=10
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        items = data.get('items', [])

        local_nodes = {
            entry['context']['node']
            for entry in items
            if 'context' in entry and 'node' in entry.get('context', {})
        }

        has_primary_logs = any(
            'powertrain.engine' in node for node in local_nodes
        )
        self.assertTrue(
            has_primary_logs,
            f'Expected logs from powertrain.engine nodes, '
            f'got nodes: {local_nodes}'
        )

    # @verifies REQ_INTEROP_061
    def test_function_logs_include_peer_entries_via_fanout(self):
        """Primary function logs include peer entries via fan-out.

        The vehicle_health function on primary hosts temp_sensor and
        rpm_sensor. The same function on the secondary hosts
        pressure_sensor and actuator. Fan-out sends the log request to
        the secondary, which returns its local logs. The primary merges
        both into the response.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/vehicle_health/logs', timeout=10
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        items = data.get('items', [])

        all_nodes = {
            entry['context']['node']
            for entry in items
            if 'context' in entry and 'node' in entry.get('context', {})
        }

        has_peer_logs = any(
            'chassis.brakes' in node for node in all_nodes
        )
        self.assertTrue(
            has_peer_logs,
            f'Expected logs from chassis.brakes peer nodes via fan-out, '
            f'but only found: {all_nodes}'
        )

    def test_function_logs_have_aggregation_metadata(self):
        """Function logs include x-medkit aggregation extension."""
        response = requests.get(
            f'{PRIMARY_URL}/functions/vehicle_health/logs', timeout=10
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()

        self.assertIn('x-medkit', data)
        ext = data['x-medkit']
        self.assertEqual(ext.get('aggregation_level'), 'function')
        self.assertTrue(ext.get('aggregated'))

    # @verifies REQ_INTEROP_061
    def test_function_logs_severity_filter_forwarded_to_peer(self):
        """Severity filter is forwarded to peer during fan-out.

        Querying with ?severity=fatal should return only fatal entries
        from both gateways. Demo nodes rarely produce fatal logs, so the
        result should be empty or contain only fatal-severity entries.
        """
        response = requests.get(
            f'{PRIMARY_URL}/functions/vehicle_health/logs?severity=fatal',
            timeout=10,
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        items = data.get('items', [])

        for entry in items:
            self.assertIn(
                entry['severity'], {'fatal'},
                f'Entry has unexpected severity: {entry["severity"]}'
            )

    def test_peer_function_logs_are_local_only(self):
        """Secondary function logs contain only local entries.

        The secondary gateway has no aggregation, so its vehicle_health
        logs should only contain chassis.brakes entries, not any
        powertrain nodes from the primary.
        """
        response = requests.get(
            f'{PEER_URL}/functions/vehicle_health/logs', timeout=10
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        items = data.get('items', [])

        for entry in items:
            if 'context' in entry and 'node' in entry.get('context', {}):
                node = entry['context']['node']
                self.assertNotIn(
                    'powertrain.engine', node,
                    f'Secondary should not have primary logs, found: {node}'
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
