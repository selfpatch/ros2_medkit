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

"""Integration tests for peer aggregation with two gateway instances.

Launches TWO gateway instances on different ports AND in separate DDS domains:
- Primary gateway (port offset 0, domain offset 0): aggregation enabled,
  static peer pointing to the secondary gateway. Manages powertrain demo
  nodes (temp_sensor, rpm_sensor).
- Peer gateway (port offset 1, domain offset 1): standard gateway, no
  aggregation. Manages chassis demo nodes (pressure_sensor, actuator).

DDS domain isolation ensures the primary cannot discover peer nodes via DDS
graph introspection - it can only learn about them through HTTP aggregation.
This validates that the aggregation layer is doing the actual work.

Tests verify:
- DDS isolation: peer gateway does NOT see primary's nodes
- Merged entity list: primary /apps includes apps from both gateways
- Request forwarding: GET /apps/{remote_app}/data is forwarded to peer
- Health shows peers: GET /health includes peer status
"""

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


# Port assignments: primary at base port, peer at base + 1
PRIMARY_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
PRIMARY_URL = f'http://localhost:{PRIMARY_PORT}{API_BASE_PATH}'
PEER_URL = f'http://localhost:{PEER_PORT}{API_BASE_PATH}'

# DDS domain isolation: each gateway and its demo nodes run in a separate DDS
# domain so that the peer gateway cannot discover the primary's nodes via DDS.
# The primary can only learn about peer entities through aggregation (HTTP).
PRIMARY_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

# Demo nodes split between gateways:
# Primary manages powertrain nodes
PRIMARY_NODES = ['temp_sensor', 'rpm_sensor']
# Peer manages chassis nodes
PEER_NODES = ['pressure_sensor', 'actuator']


def generate_test_description():
    peer_domain_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}

    # Primary gateway: aggregation enabled with static peer, in PRIMARY_DOMAIN_ID
    primary_gateway = create_gateway_node(
        port=PRIMARY_PORT,
        extra_params={
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://localhost:{PEER_PORT}'],
            'aggregation.peer_names': ['peer_gateway'],
        },
    )

    # Peer gateway: standard configuration, no aggregation, in PEER_DOMAIN_ID
    peer_gateway = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='peer_gateway_node',
        output='screen',
        parameters=[{
            'refresh_interval_ms': 1000,
            'server.port': PEER_PORT,
        }],
        additional_env=peer_domain_env,
    )

    # Demo nodes: each set runs in its gateway's DDS domain.
    # Primary nodes inherit ROS_DOMAIN_ID from SetEnvironmentVariable below.
    # Peer nodes get an explicit override via extra_env.
    primary_demo_nodes = create_demo_nodes(PRIMARY_NODES, lidar_faulty=False)
    peer_demo_nodes = create_demo_nodes(
        PEER_NODES, lidar_faulty=False, extra_env=peer_domain_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=primary_demo_nodes + peer_demo_nodes,
    )

    # SetEnvironmentVariable sets the default ROS_DOMAIN_ID for the launch
    # context (primary domain). The peer nodes override it via additional_env.
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


class TestPeerAggregation(unittest.TestCase):
    """Verify peer aggregation between two gateway instances."""

    @classmethod
    def setUpClass(cls):
        """Wait for both gateways to be healthy and discover their nodes."""
        cls._wait_for_health(PRIMARY_URL, 'Primary')
        cls._wait_for_health(PEER_URL, 'Peer')

        # Wait for peer gateway to discover its nodes
        cls._wait_for_apps(PEER_URL, {'pressure_sensor', 'actuator'}, 'Peer')

        # Wait for primary gateway to discover its own nodes AND merge peer's
        # This requires multiple refresh cycles for aggregation to kick in
        cls._wait_for_apps(
            PRIMARY_URL,
            {'temp_sensor', 'rpm_sensor', 'pressure_sensor', 'actuator'},
            'Primary (merged)',
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
                            f'{label}: all apps discovered ({len(found_ids)} total)'
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

    # ------------------------------------------------------------------
    # Merged entity list
    # ------------------------------------------------------------------

    def test_primary_apps_include_peer_apps(self):
        """Primary gateway's /apps includes apps from both gateways.

        The primary discovers temp_sensor and rpm_sensor locally, and
        merges pressure_sensor and actuator from the peer gateway.
        """
        response = requests.get(f'{PRIMARY_URL}/apps', timeout=10)
        self.assertEqual(response.status_code, 200)
        items = response.json().get('items', [])
        app_ids = {a['id'] for a in items}

        # Local apps
        self.assertIn('temp_sensor', app_ids)
        self.assertIn('rpm_sensor', app_ids)
        # Peer apps (merged via aggregation)
        self.assertIn('pressure_sensor', app_ids)
        self.assertIn('actuator', app_ids)

    def test_peer_no_aggregation_no_peers(self):
        """Peer gateway does not aggregate - it has no peer status.

        The peer gateway runs in its own DDS domain and has aggregation
        disabled, so it should NOT have 'peers' in its health response
        or see the primary's nodes.
        """
        response = requests.get(f'{PEER_URL}/health', timeout=10)
        self.assertEqual(response.status_code, 200)
        data = response.json()

        self.assertNotIn(
            'peers', data,
            'Peer gateway should not report peers (aggregation disabled)'
        )

        # Peer's own apps should still be discovered
        apps_response = requests.get(f'{PEER_URL}/apps', timeout=10)
        self.assertEqual(apps_response.status_code, 200)
        items = apps_response.json().get('items', [])
        app_ids = {a['id'] for a in items}
        self.assertIn('pressure_sensor', app_ids)
        self.assertIn('actuator', app_ids)

    def test_peer_does_not_see_primary_nodes_via_dds(self):
        """Peer gateway must NOT see primary's nodes (DDS domain isolation).

        Because the two gateways run in separate DDS domains, the peer
        gateway should only see its own chassis nodes (pressure_sensor,
        actuator) and NOT the primary's powertrain nodes (temp_sensor,
        rpm_sensor). This proves the DDS isolation is working.
        """
        response = requests.get(f'{PEER_URL}/apps', timeout=10)
        self.assertEqual(response.status_code, 200)
        items = response.json().get('items', [])
        app_ids = {a['id'] for a in items}

        # Peer must NOT see primary's nodes
        self.assertNotIn(
            'temp_sensor', app_ids,
            'Peer gateway should not discover primary nodes via DDS'
        )
        self.assertNotIn(
            'rpm_sensor', app_ids,
            'Peer gateway should not discover primary nodes via DDS'
        )

    def test_primary_functions_include_peer_functions(self):
        """Primary gateway merges Functions from both gateways.

        Primary has powertrain (from /powertrain namespace), peer has
        chassis (from /chassis namespace). Primary should see both.
        """
        response = requests.get(f'{PRIMARY_URL}/functions', timeout=10)
        self.assertEqual(response.status_code, 200)
        items = response.json().get('items', [])
        func_ids = {f['id'] for f in items}

        self.assertIn('powertrain', func_ids)
        self.assertIn('chassis', func_ids)

    # ------------------------------------------------------------------
    # Request forwarding
    # ------------------------------------------------------------------

    def test_forward_data_request_to_peer(self):
        """GET /apps/{remote_app}/data on primary is forwarded to peer.

        pressure_sensor lives on the peer gateway. When the primary receives
        a data request for it, it should forward the request to the peer
        and return the peer's response.
        """
        response = requests.get(
            f'{PRIMARY_URL}/apps/pressure_sensor/data', timeout=10
        )
        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 for forwarded data request, got '
            f'{response.status_code}: {response.text}'
        )
        data = response.json()
        self.assertIn('items', data)

    def test_forward_app_detail_to_peer(self):
        """GET /apps/{remote_app} on primary returns peer app detail.

        The primary should transparently proxy the request and return
        the app detail from the peer gateway.
        """
        response = requests.get(
            f'{PRIMARY_URL}/apps/actuator', timeout=10
        )
        self.assertEqual(
            response.status_code, 200,
            f'Expected 200 for forwarded app detail, got '
            f'{response.status_code}: {response.text}'
        )
        data = response.json()
        self.assertEqual(data['id'], 'actuator')

    # ------------------------------------------------------------------
    # Health shows peers
    # ------------------------------------------------------------------

    def test_health_includes_peer_status(self):
        """GET /health on primary includes peer status information.

        When aggregation is enabled, the health endpoint should include
        a 'peers' array showing connected peer gateways and their status.
        """
        response = requests.get(f'{PRIMARY_URL}/health', timeout=10)
        self.assertEqual(response.status_code, 200)
        data = response.json()

        self.assertIn(
            'peers', data,
            'Health response should include peers when aggregation is enabled'
        )
        peers = data['peers']
        self.assertIsInstance(peers, list)
        self.assertGreaterEqual(len(peers), 1)

        # Find our peer gateway
        peer_entry = None
        for peer in peers:
            if peer.get('name') == 'peer_gateway':
                peer_entry = peer
                break
        self.assertIsNotNone(
            peer_entry,
            f'Expected peer_gateway in peers list: {peers}'
        )
        assert peer_entry is not None  # type narrowing for Pyright
        self.assertEqual(peer_entry['status'], 'online')
        self.assertIn(str(PEER_PORT), peer_entry['url'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
