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

"""A fan-out names every peer it should have asked and did not.

The aggregator has two peers: ``live_gateway``, a real gateway, and
``ghost_gateway``, a port nothing listens on. The health check marks the ghost
offline, and the fan-out does not ask a peer marked offline.

R1  The global fault list is ``partial`` and names the ghost in
    ``failed_peers``, reason ``unreachable``.
R2  A global fault list whose visited path is full asks no peer. It names the
    live gateway ``too-deep`` and the ghost ``unreachable``.
R3  An entity route asks only the peers that contribute the entity. The ghost
    contributes nothing to the merged Function, so its data list names nobody.
R4  Once the live gateway is killed and marked offline, the Function's data
    list is ``partial`` and names it, reason ``unreachable``.
"""

import os
import signal
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_testing.actions
import requests
from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_INTERVAL,
    DISCOVERY_TIMEOUT,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_fault_manager_node,
    create_gateway_node,
)

AGG_PORT = get_test_port(0)
LIVE_PORT = get_test_port(1)
# Nothing listens here: the ghost peer.
GHOST_PORT = get_test_port(2)

AGG_URL = f'http://localhost:{AGG_PORT}{API_BASE_PATH}'
LIVE_URL = f'http://localhost:{LIVE_PORT}{API_BASE_PATH}'

LIVE_DOMAIN_ID = get_test_domain_id(1)

LIVE = 'live_gateway'
GHOST = 'ghost_gateway'
MERGED_FUNCTION = 'engine_health'
LIVE_APP = 'temp_sensor'
LIVE_TOPIC = '/powertrain/engine/temperature'
# A visited path of this many ids is full: the gateway asks no peer.
FULL_VISITED_PATH = ','.join(f'gateway-{index}' for index in range(16))

_KILLED_PIDS = set()

AGG_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Aggregator"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
functions:
  - id: {MERGED_FUNCTION}
    name: "Engine Health"
"""

LIVE_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Live ECU"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
components:
  - id: engine-ecu
    name: "Engine ECU"
apps:
  - id: {LIVE_APP}
    name: "Engine Temperature Sensor"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: /powertrain/engine
functions:
  - id: {MERGED_FUNCTION}
    name: "Engine Health"
    hosted_by:
      - {LIVE_APP}
"""


def _write_manifest(content, prefix):
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix=prefix)
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    return path


def generate_test_description():
    live_env = {'ROS_DOMAIN_ID': str(LIVE_DOMAIN_ID)}

    aggregator = create_gateway_node(
        name='aggregator_gateway_node',
        port=AGG_PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': _write_manifest(AGG_MANIFEST, 'test_fan_out_agg_'),
            'discovery.manifest_strict_validation': False,
            # Each refresh re-checks peer health; a short one notices the kill soon.
            'refresh_interval_ms': 500,
            'aggregation.enabled': True,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.peer_urls': [f'http://localhost:{LIVE_PORT}',
                                      f'http://localhost:{GHOST_PORT}'],
            'aggregation.peer_names': [LIVE, GHOST],
        },
    )
    live_gateway = create_gateway_node(
        name='live_gateway_node',
        port=LIVE_PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': _write_manifest(LIVE_MANIFEST, 'test_fan_out_live_'),
            'discovery.manifest_strict_validation': False,
        },
        extra_env=live_env,
    )

    return (
        LaunchDescription([
            aggregator,
            create_fault_manager_node(rosbag_enabled=False),
            live_gateway,
            create_fault_manager_node(rosbag_enabled=False, extra_env=live_env),
            *create_demo_nodes([LIVE_APP], extra_env=live_env),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': aggregator, 'live_gateway': live_gateway},
    )


def _peer_status(base_url):
    body = requests.get(f'{base_url}/health', timeout=10).json()
    peers = body.get('x-medkit', {}).get('peers') or body.get('peers') or []
    return {peer.get('name'): peer.get('status') for peer in peers}


class TestFanOutUnhealthyPeer(unittest.TestCase):
    """Runs in name order: test_4 kills the live gateway."""

    def _wait_for_peers(self, expected):
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        seen = None
        while time.monotonic() < deadline:
            try:
                seen = _peer_status(AGG_URL)
                if all(seen.get(name) == status for name, status in expected.items()):
                    return
            except requests.RequestException as exc:
                seen = str(exc)
            time.sleep(DISCOVERY_INTERVAL)
        self.fail(f'the aggregator never reported peers {expected}; last saw {seen}')

    def _function_data(self):
        response = requests.get(f'{AGG_URL}/functions/{MERGED_FUNCTION}/data', timeout=30)
        self.assertEqual(response.status_code, 200, response.text[:400])
        return response.json()

    def _assert_names(self, x_medkit, peer):
        self.assertIs(x_medkit.get('partial'), True, x_medkit)
        self.assertEqual(x_medkit.get('failed_peers'), [peer], x_medkit)
        self.assertEqual(x_medkit.get('peer_failures'),
                         [{'peer': peer, 'reason': 'unreachable'}], x_medkit)

    def test_1_the_global_fault_list_names_a_peer_it_did_not_ask(self):
        """R1."""
        self._wait_for_peers({LIVE: 'online', GHOST: 'offline'})
        response = requests.get(f'{AGG_URL}/faults', timeout=30)
        self.assertEqual(response.status_code, 200, response.text[:400])
        self._assert_names(response.json().get('x-medkit', {}), GHOST)

    def test_2_a_full_visited_path_names_every_peer(self):
        """R2."""
        self._wait_for_peers({LIVE: 'online', GHOST: 'offline'})
        response = requests.get(f'{AGG_URL}/faults', timeout=30,
                                headers={'X-Medkit-Fan-Out-Visited': FULL_VISITED_PATH})
        self.assertEqual(response.status_code, 200, response.text[:400])
        x_medkit = response.json().get('x-medkit', {})
        self.assertIs(x_medkit.get('partial'), True, x_medkit)
        self.assertEqual(x_medkit.get('failed_peers'), [LIVE, GHOST], x_medkit)
        self.assertEqual(x_medkit.get('peer_failures'),
                         [{'peer': LIVE, 'reason': 'too-deep'},
                          {'peer': GHOST, 'reason': 'unreachable'}], x_medkit)

    def test_3_an_entity_route_names_no_peer_it_had_no_reason_to_ask(self):
        """R3: the ghost contributes nothing to the Function."""
        self._wait_for_peers({LIVE: 'online', GHOST: 'offline'})
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        body = {}
        while time.monotonic() < deadline:
            body = self._function_data()
            if any(item.get('id') == LIVE_TOPIC for item in body.get('items', [])):
                break
            time.sleep(DISCOVERY_INTERVAL)
        self.assertTrue(any(item.get('id') == LIVE_TOPIC for item in body.get('items', [])),
                        f'the live gateway never answered for {MERGED_FUNCTION}: {body}')
        x_medkit = body.get('x-medkit', {})
        for key in ('partial', 'failed_peers', 'peer_failures'):
            self.assertNotIn(key, x_medkit, x_medkit)

    def test_4_an_entity_route_names_its_peer_once_it_is_marked_offline(self, live_gateway):
        """R4."""
        self._wait_for_peers({LIVE: 'online'})
        pid = live_gateway.process_details['pid']
        _KILLED_PIDS.add(pid)
        os.kill(pid, signal.SIGKILL)
        self._wait_for_peers({LIVE: 'offline'})
        self._assert_names(self._function_data().get('x-medkit', {}), LIVE)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            allowed = set(ALLOWED_EXIT_CODES)
            if info.pid in _KILLED_PIDS:
                allowed.add(-9)
            self.assertIn(info.returncode, allowed,
                          f'{info.process_name} exited with code {info.returncode}')
