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

"""A running node keeps its operations while the graph also lists a leftover of its name.

The DDS RMWs answer a by-node query from the first graph entry of the name, in participant
GID order. ``ghost_node_injector --ghost`` adds an entry with no enclave and no endpoints whose
GID sorts before every real participant's: what a dead instance leaves behind when the node is
restarted. ``calibration`` serves a service outside its own namespace and ``long_calibration``
an action, so both ways a node's operations are found are covered.
"""

import os
import time
import unittest

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import rclpy
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DEFAULT_BASE_URL,
    DEFAULT_DOMAIN_ID,
    get_time_scale,
)
from ros2_medkit_test_utils.graph_fixtures import (
    GhostInjection,
    observed_enclaves,
    split_fqn,
    wait_observed,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
    get_coverage_env,
)

TIME_SCALE = get_time_scale()

REFRESH_INTERVAL_MS = 500
REFRESH_DEBOUNCE_MS = 500
# Six refreshes. An observation window, so TIME_SCALE does not apply.
OBSERVE_SEC = 6 * REFRESH_INTERVAL_MS / 1000.0
APPEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE
INJECTION_TIMEOUT_SEC = 45.0 * TIME_SCALE

# App id -> FQN of the demo nodes the ghosts shadow.
SHADOWED = {
    'calibration': '/powertrain/engine/calibration',
    'long_calibration': '/powertrain/engine/long_calibration',
}
# Injected after the leftovers. Once GET /apps lists it, the gateway's graph holds them too.
MARKER = '/shadow_marker_ns/shadow_marker'


def generate_test_description():
    gateway_node = create_gateway_node(
        extra_params={
            'refresh_interval_ms': REFRESH_INTERVAL_MS,
            'discovery.refresh_debounce_ms': REFRESH_DEBOUNCE_MS,
        },
    )
    return (
        LaunchDescription([
            gateway_node,
            *create_demo_nodes(['calibration', 'long_calibration']),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


def _fixture_env():
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
    env.update(get_coverage_env())
    return env


def _app_listed(app_id):
    try:
        response = requests.get(f'{DEFAULT_BASE_URL}/apps', timeout=10)
    except requests.exceptions.RequestException:
        return False
    if response.status_code != 200:
        return False
    return any(item.get('id') == app_id for item in response.json().get('items', []))


def _operation_ids(app_id):
    """Operation ids of an App, or None when GET fails."""
    try:
        response = requests.get(f'{DEFAULT_BASE_URL}/apps/{app_id}/operations', timeout=10)
    except requests.exceptions.RequestException:
        return None
    if response.status_code != 200:
        return None
    return sorted(item.get('id') for item in response.json().get('items', []))


class TestGraphShadowedNodeServices(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.observer = rclpy.create_node('_graph_shadow_observer')

    @classmethod
    def tearDownClass(cls):
        cls.observer.destroy_node()
        rclpy.shutdown()

    def test_operations_survive_a_leftover_of_the_same_name(self):
        before = {}
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while time.monotonic() < deadline and len(before) < len(SHADOWED):
            for app_id in SHADOWED:
                ids = _operation_ids(app_id)
                if ids:
                    before[app_id] = ids
            time.sleep(0.2)
        self.assertEqual(
            sorted(before), sorted(SHADOWED),
            f'these Apps never listed operations before the injection: {before}')
        self.assertIn('calibrate', before['calibration'])
        self.assertIn('long_calibration', before['long_calibration'])

        injection = GhostInjection(ghosts=[*SHADOWED.values(), MARKER], env=_fixture_env())
        try:
            self.assertIsNotNone(
                injection.wait_status(INJECTION_TIMEOUT_SEC),
                f'ghost_node_injector never published:\n{injection.output()}')
            for fqn in SHADOWED.values():
                self.assertTrue(
                    wait_observed(self.observer, fqn,
                                  lambda found: sorted(found) == ['', '/'],
                                  INJECTION_TIMEOUT_SEC),
                    f'the test process graph lists {fqn} with '
                    f'{observed_enclaves(self.observer, fqn)}. The test needs one running entry '
                    'and one leftover.')
                name, namespace = split_fqn(fqn)
                self.assertEqual(
                    self.observer.get_service_names_and_types_by_node(name, namespace), [],
                    f'the by-node query for {fqn} still answers for the running node, so the '
                    'leftover does not shadow it and this test proves nothing')

            # The gateway's discovery can trail this process's by seconds.
            marker_id = split_fqn(MARKER)[0]
            deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
            while time.monotonic() < deadline and not _app_listed(marker_id):
                time.sleep(0.2)
            self.assertTrue(
                _app_listed(marker_id),
                f'GET /apps never listed {marker_id}, so the gateway graph may not hold the '
                'leftovers')

            deadline = time.monotonic() + OBSERVE_SEC
            while time.monotonic() < deadline:
                self.assertEqual(
                    {app_id: _operation_ids(app_id) for app_id in before}, before,
                    'operations changed once a leftover of the node name was listed')
                time.sleep(REFRESH_INTERVAL_MS / 1000.0)
        finally:
            injection.stop()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
