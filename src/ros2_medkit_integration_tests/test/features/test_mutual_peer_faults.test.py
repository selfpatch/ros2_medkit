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

"""Two gateways that aggregate each other must not bounce GET /faults.

Gateway A lists B as a peer and B lists A. B runs the faulty lidar, so its
fault manager holds LIDAR_RANGE_INVALID. GET /faults on A fans out to B with
X-Medkit-No-Fan-Out. B must answer with its own faults only. Before the fix B
fanned out again, back to A, and the request bounced between the two until
the peer timeouts.
"""

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
    GATEWAY_STARTUP_INTERVAL,
    GATEWAY_STARTUP_TIMEOUT,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_fault_manager_node,
    create_gateway_node,
)

A_PORT = get_test_port(0)
B_PORT = get_test_port(1)
A_URL = f'http://localhost:{A_PORT}{API_BASE_PATH}'
B_URL = f'http://localhost:{B_PORT}{API_BASE_PATH}'
A_DOMAIN_ID = get_test_domain_id(0)
B_DOMAIN_ID = get_test_domain_id(1)

PEER_TIMEOUT_MS = 2000
FAULT_CODE = 'LIDAR_RANGE_INVALID'
NO_FAN_OUT = {'X-Medkit-No-Fan-Out': '1'}


def _aggregation_params(peer_port, peer_name):
    return {
        'aggregation.enabled': True,
        'aggregation.timeout_ms': PEER_TIMEOUT_MS,
        'aggregation.announce': False,
        'aggregation.discover': False,
        'aggregation.peer_urls': [f'http://localhost:{peer_port}'],
        'aggregation.peer_names': [peer_name],
    }


def generate_test_description():
    b_env = {'ROS_DOMAIN_ID': str(B_DOMAIN_ID)}

    gateway_a = create_gateway_node(
        port=A_PORT,
        extra_params=_aggregation_params(B_PORT, 'gateway_b'),
    )
    gateway_b = create_gateway_node(
        name='gateway_b_node',
        port=B_PORT,
        extra_params=_aggregation_params(A_PORT, 'gateway_a'),
        extra_env=b_env,
    )

    delayed = TimerAction(
        period=2.0,
        actions=(
            create_demo_nodes(['lidar_sensor'], lidar_faulty=True, extra_env=b_env)
            + [
                create_fault_manager_node(rosbag_enabled=False),
                create_fault_manager_node(rosbag_enabled=False, extra_env=b_env),
            ]
        ),
    )

    return (
        LaunchDescription([
            SetEnvironmentVariable('ROS_DOMAIN_ID', str(A_DOMAIN_ID)),
            gateway_a,
            gateway_b,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_a, 'gateway_b': gateway_b},
    )


def _codes(response):
    return [f.get('fault_code', f.get('code', '')) for f in response.json().get('items', [])]


class TestMutualPeerFaults(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        for url in (A_URL, B_URL):
            deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
            while time.monotonic() < deadline:
                try:
                    if requests.get(f'{url}/health', timeout=2).status_code == 200:
                        break
                except requests.exceptions.RequestException:
                    pass
                time.sleep(GATEWAY_STARTUP_INTERVAL)
            else:
                raise AssertionError(f'{url} not healthy after {GATEWAY_STARTUP_TIMEOUT}s')

        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            try:
                r = requests.get(f'{B_URL}/faults', headers=NO_FAN_OUT, timeout=5)
                if r.status_code == 200 and FAULT_CODE in _codes(r):
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)
        raise AssertionError(f'{FAULT_CODE} never reached gateway B')

    def test_no_fan_out_request_answers_local_faults_only(self):
        """A request from a peer is answered from the local fault manager."""
        r = requests.get(f'{A_URL}/faults', headers=NO_FAN_OUT, timeout=10)
        self.assertEqual(r.status_code, 200)
        self.assertNotIn(FAULT_CODE, _codes(r))

    def test_mutual_peers_answer_without_bouncing(self):
        """GET /faults on A returns B's fault once, well inside one peer timeout."""
        start = time.monotonic()
        r = requests.get(f'{A_URL}/faults', timeout=30)
        elapsed = time.monotonic() - start
        self.assertEqual(r.status_code, 200)
        self.assertEqual(_codes(r).count(FAULT_CODE), 1, _codes(r))
        x_medkit = r.json().get('x-medkit', {})
        self.assertFalse(x_medkit.get('partial', False), x_medkit)
        self.assertLess(elapsed, PEER_TIMEOUT_MS / 1000.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited with code {info.returncode}')
