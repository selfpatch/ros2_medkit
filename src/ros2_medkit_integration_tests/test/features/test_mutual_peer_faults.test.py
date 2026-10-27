#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""GET /faults across a loop of peers and down a chain.

Gateway A lists B as a peer and B lists A, so the two aggregate each other. B
also lists C, a leaf: A -> B -> C is the chain topology from
docs/config/aggregation.rst. Each gateway runs its own faulty lidar and fault
manager on its own domain, so each holds one LIDAR_RANGE_INVALID.

GET /faults on A must list all three, each once:

- before the visited path, A asked B, B asked A again, and the request bounced
  until the peer timeouts;
- answering a peer's request from the local fault manager alone stops the
  bounce but drops C, because B never asks it;
- a gateway that answered a request coming back round the loop with its own
  faults would list them twice;
- a gateway that sent the request back round the loop at all would hold a
  worker on each side, so enough concurrent requests end at the peer timeout.
"""

from concurrent.futures import ThreadPoolExecutor
import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_INTERVAL,
    DISCOVERY_TIMEOUT,
    FAULT_TIMEOUT,
    GATEWAY_STARTUP_INTERVAL,
    GATEWAY_STARTUP_TIMEOUT,
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_fault_manager_node,
    create_gateway_node,
)

A_PORT = get_test_port(0)
B_PORT = get_test_port(1)
C_PORT = get_test_port(2)
A_URL = f'http://localhost:{A_PORT}{API_BASE_PATH}'
B_URL = f'http://localhost:{B_PORT}{API_BASE_PATH}'
C_URL = f'http://localhost:{C_PORT}{API_BASE_PATH}'
A_DOMAIN_ID = get_test_domain_id(0)
B_DOMAIN_ID = get_test_domain_id(1)
C_DOMAIN_ID = get_test_domain_id(2)

PEER_TIMEOUT_MS = int(2000 * get_time_scale())
FAULT_CODE = 'LIDAR_RANGE_INVALID'
NO_FAN_OUT = {'X-Medkit-No-Fan-Out': '1'}
# Pinned to the gateway's default worker count, and as many concurrent
# requests as that: enough to tie up every worker A has.
HTTP_WORKERS = 6
CONCURRENT_REQUESTS = HTTP_WORKERS


def _aggregation_params(peers):
    return {
        'server.http_thread_pool_size': HTTP_WORKERS,
        'aggregation.enabled': True,
        'aggregation.timeout_ms': PEER_TIMEOUT_MS,
        'aggregation.announce': False,
        'aggregation.discover': False,
        'aggregation.peer_urls': [f'http://localhost:{port}' for port, _ in peers],
        'aggregation.peer_names': [name for _, name in peers],
    }


def generate_test_description():
    b_env = {'ROS_DOMAIN_ID': str(B_DOMAIN_ID)}
    c_env = {'ROS_DOMAIN_ID': str(C_DOMAIN_ID)}

    gateway_a = create_gateway_node(
        port=A_PORT,
        extra_params=_aggregation_params([(B_PORT, 'gateway_b')]),
    )
    gateway_b = create_gateway_node(
        name='gateway_b_node',
        port=B_PORT,
        extra_params=_aggregation_params([(A_PORT, 'gateway_a'), (C_PORT, 'gateway_c')]),
        extra_env=b_env,
    )
    gateway_c = create_gateway_node(
        name='gateway_c_node',
        port=C_PORT,
        extra_env=c_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=(
            create_demo_nodes(['lidar_sensor'], lidar_faulty=True)
            + create_demo_nodes(['lidar_sensor'], lidar_faulty=True, extra_env=b_env)
            + create_demo_nodes(['lidar_sensor'], lidar_faulty=True, extra_env=c_env)
            + [
                create_fault_manager_node(rosbag_enabled=False),
                create_fault_manager_node(rosbag_enabled=False, extra_env=b_env),
                create_fault_manager_node(rosbag_enabled=False, extra_env=c_env),
            ]
        ),
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable('ROS_DOMAIN_ID', str(A_DOMAIN_ID)),
            gateway_a,
            gateway_b,
            gateway_c,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_a, 'gateway_b': gateway_b, 'gateway_c': gateway_c},
    )


def _first_seen(response):
    return [f.get('first_occurred') for f in response.json().get('items', [])
            if f.get('fault_code') == FAULT_CODE]


class TestMutualPeerFaults(GatewayTestCase):

    BASE_URL = A_URL
    REQUIRED_APPS = {'lidar_sensor'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        for url in (B_URL, C_URL):
            cls._wait_for_health(url)
        cls._wait_for_peers_online(A_URL, {'gateway_b'})
        cls._wait_for_peers_online(B_URL, {'gateway_a', 'gateway_c'})
        # Every gateway reports the same fault code, so the faults are told
        # apart by when each fault manager first saw its own.
        cls.first_seen = {
            url: cls._wait_for_local_fault(url) for url in (A_URL, B_URL, C_URL)
        }
        assert len(set(cls.first_seen.values())) == 3, cls.first_seen

    @staticmethod
    def _wait_for_health(url):
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline:
            try:
                if requests.get(f'{url}/health', timeout=2).status_code == 200:
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(GATEWAY_STARTUP_INTERVAL)
        raise AssertionError(f'{url} not healthy after {GATEWAY_STARTUP_TIMEOUT}s')

    @staticmethod
    def _wait_for_peers_online(url, names):
        # A skips a peer it holds as unhealthy without marking the answer
        # partial, so a test that runs before its health check lands would
        # fail on correct code.
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        last = None
        while time.monotonic() < deadline:
            try:
                r = requests.get(f'{url}/health', timeout=5)
                if r.status_code == 200:
                    last = r.json().get('peers', [])
                    online = {p.get('name') for p in last if p.get('status') == 'online'}
                    if names <= online:
                        return
            except requests.exceptions.RequestException as exc:
                last = str(exc)
            time.sleep(DISCOVERY_INTERVAL)
        raise AssertionError(f'{url} never saw {names} online; last saw {last}')

    @staticmethod
    def _wait_for_local_fault(url):
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            try:
                r = requests.get(f'{url}/faults', headers=NO_FAN_OUT, timeout=5)
                if r.status_code == 200:
                    seen = _first_seen(r)
                    if seen:
                        return seen[0]
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(f'{FAULT_CODE} never reached the fault manager behind {url}')

    def _assert_complete(self, response, gateways):
        self.assertEqual(response.status_code, 200, response.text)
        body = response.json()
        x_medkit = body.get('x-medkit', {})
        self.assertNotIn('partial', x_medkit, x_medkit)
        self.assertNotIn('failed_peers', x_medkit, x_medkit)
        self.assertEqual(x_medkit.get('count'), len(body.get('items', [])), x_medkit)
        self.assertEqual(
            sorted(_first_seen(response)),
            sorted(self.first_seen[url] for url in gateways),
            f'expected the faults of {gateways}, one each')

    def test_no_fan_out_request_answers_local_faults_only(self):
        """X-Medkit-No-Fan-Out alone is answered from the local fault manager."""
        r = requests.get(f'{A_URL}/faults', headers=NO_FAN_OUT, timeout=30)
        self._assert_complete(r, [A_URL])

    def test_head_of_chain_lists_every_gateway_once(self):
        """A lists A, B and C, without bouncing between A and B."""
        start = time.monotonic()
        r = requests.get(f'{A_URL}/faults', timeout=30)
        elapsed = time.monotonic() - start
        self._assert_complete(r, [A_URL, B_URL, C_URL])
        self.assertLess(elapsed, PEER_TIMEOUT_MS / 1000.0)

    def test_middle_of_chain_lists_every_gateway_once(self):
        """B asks A and C, and A does not ask B back."""
        r = requests.get(f'{B_URL}/faults', timeout=30)
        self._assert_complete(r, [A_URL, B_URL, C_URL])

    def test_status_filter_reaches_every_peer(self):
        """The query string goes to the peers, so they filter too."""
        r = requests.get(f'{A_URL}/faults', params={'status': 'cleared'}, timeout=30)
        self._assert_complete(r, [])
        r = requests.get(f'{A_URL}/faults', params={'status': 'all'}, timeout=30)
        self._assert_complete(r, [A_URL, B_URL, C_URL])

    def test_concurrent_requests_do_not_exhaust_workers(self):
        """As many concurrent requests as A has workers all answer complete.

        A request sent back to A would wait for a free worker on A while every
        worker waits on B, so each would end at the peer timeout.
        """
        with ThreadPoolExecutor(max_workers=CONCURRENT_REQUESTS) as pool:
            responses = list(pool.map(
                lambda _: requests.get(f'{A_URL}/faults', timeout=30),
                range(CONCURRENT_REQUESTS)))
        for r in responses:
            self._assert_complete(r, [A_URL, B_URL, C_URL])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited with code {info.returncode}')
