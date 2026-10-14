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

"""Discovery remembers a bounded number of departed nodes, and running nodes cost nothing.

One participant announces more nodes than discovery remembers. Checks how many leftovers stay
hidden, that a node departing with no entry left is forgotten first past the capacity, and
that new nodes are still listed. Own gateway: a graph this size slows later reads.
"""

import os
import subprocess
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
    fixture_path,
    LeftoverNode,
    observed_enclaves,
    split_fqn,
    stop_process,
    wait_observed,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node, get_coverage_env

TIME_SCALE = get_time_scale()

REFRESH_INTERVAL_MS = 500
REFRESH_DEBOUNCE_MS = 500
# How long a claim is watched: three refreshes. An observation window, so not scaled.
OBSERVE_SEC = 3 * REFRESH_INTERVAL_MS / 1000.0
INJECTION_TIMEOUT_SEC = 45.0 * TIME_SCALE
APPEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE
PROCESS_EXIT_TIMEOUT_SEC = 30.0 * TIME_SCALE
# Listing bound of test_graph_leftover_nodes.test.py doubled: each remembered leftover costs
# endpoint queries on every read.
LISTED_AFTER_OBSERVED_SEC = 2 * ((REFRESH_DEBOUNCE_MS + 100) / 1000.0 + 4.0) * TIME_SCALE
LEFTOVER_DELAY_SEC = 1.0
# A --delay longer than any run of this file: the late sample goes out on `publish`.
PUBLISH_ON_COMMAND_DELAY_SEC = 3600.0
# GraphNodeListReader::kDefaultCapacity.
CAPACITY = 1024
# Announced nodes: with the announcing node, more than discovery remembers.
ANNOUNCED = CAPACITY + 76

CAP_FQN = '/leftover_cap_ns/leftover_cap'
DEPART_FQN = '/leftover_depart_ns/leftover_depart'
RECENT_FQN = '/leftover_recent_ns/leftover_recent'
LATE_FQN = '/leftover_cap_live/late'


def generate_test_description():
    gateway_node = create_gateway_node(
        extra_params={
            'refresh_interval_ms': REFRESH_INTERVAL_MS,
            'discovery.refresh_debounce_ms': REFRESH_DEBOUNCE_MS,
        },
    )
    return (
        LaunchDescription([gateway_node, launch_testing.actions.ReadyToTest()]),
        {'gateway_node': gateway_node},
    )


def _fixture_env():
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
    env.update(get_coverage_env())
    return env


class TestGraphLeftoverNodesScale(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while True:
            try:
                if requests.get(f'{DEFAULT_BASE_URL}/health', timeout=2).status_code == 200:
                    break
            except requests.exceptions.RequestException:
                pass
            if time.monotonic() > deadline:
                raise AssertionError('the gateway never answered GET /health')
            time.sleep(0.2)
        rclpy.init()
        cls.observer = rclpy.create_node('_graph_leftover_scale_observer')

    @classmethod
    def tearDownClass(cls):
        cls.observer.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        self._stop = []

    def tearDown(self):
        for stop in reversed(self._stop):
            stop()

    @staticmethod
    def _app_ids():
        response = requests.get(f'{DEFAULT_BASE_URL}/apps', timeout=30)
        response.raise_for_status()
        return {item.get('id') for item in response.json().get('items', [])}

    def _wait_ids(self, predicate, timeout, interval=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                ids = self._app_ids()
                if predicate(ids):
                    return ids
            except requests.exceptions.RequestException:
                pass
            time.sleep(interval)
        return None

    def _observe(self, check, interval=0.5):
        deadline = time.monotonic() + OBSERVE_SEC
        while time.monotonic() < deadline:
            check(self._app_ids())
            time.sleep(interval)

    def _leftover(self, fqn, delay_sec, announce=0):
        leftover = LeftoverNode(fqn, delay_sec, announce=announce, env=_fixture_env())
        self._stop.append(lambda: leftover.stop(PROCESS_EXIT_TIMEOUT_SEC))
        self.assertTrue(leftover.wait_ready(INJECTION_TIMEOUT_SEC), leftover.output())
        self.assertIsNotNone(
            self._wait_ids(lambda ids: split_fqn(fqn)[0] in ids, APPEAR_TIMEOUT_SEC),
            f'{fqn} runs but never appeared in GET /apps')
        return leftover

    def _wait_left_over(self, fqn, timeout):
        self.assertTrue(
            wait_observed(self.observer, fqn, lambda enclaves: enclaves == [''], timeout),
            f'the test process graph lists {fqn} with {observed_enclaves(self.observer, fqn)} '
            'rather than as a leftover')

    def _warned(self, proc_output, gateway_node, fqns):
        text = ''.join(
            output.text.decode(errors='replace') for output in proc_output[gateway_node])
        return sum(1 for fqn in fqns if f"Node '{fqn}' is not exposed" in text)

    def _wait_warned(self, proc_output, gateway_node, fqns, expected):
        deadline = time.monotonic() + LISTED_AFTER_OBSERVED_SEC
        while (self._warned(proc_output, gateway_node, fqns) < expected
               and time.monotonic() < deadline):
            time.sleep(0.2)
        return self._warned(proc_output, gateway_node, fqns)

    def test_more_nodes_than_discovery_remembers(self, proc_output, gateway_node):
        cap = self._leftover(CAP_FQN, PUBLISH_ON_COMMAND_DELAY_SEC, announce=ANNOUNCED)
        self.assertTrue(cap.announce_nodes(INJECTION_TIMEOUT_SEC), cap.output())
        cap_fqns = [CAP_FQN] + cap.announced_fqns()
        cap_ids = {split_fqn(fqn)[0] for fqn in cap_fqns}
        self.assertIsNotNone(
            self._wait_ids(lambda ids: cap_ids <= ids, 2 * APPEAR_TIMEOUT_SEC),
            f'the gateway never listed all {len(cap_ids)} announced nodes running')

        # Running nodes do not count against the capacity: this node's leftover is hidden.
        depart_id = split_fqn(DEPART_FQN)[0]
        depart = self._leftover(DEPART_FQN, LEFTOVER_DELAY_SEC)
        depart.leave()
        self._wait_left_over(DEPART_FQN, LEFTOVER_DELAY_SEC + INJECTION_TIMEOUT_SEC)
        self.assertEqual(
            self._wait_warned(proc_output, gateway_node, [DEPART_FQN], 1), 1,
            f'the gateway never warned about the leftover of {DEPART_FQN} next to '
            f'{len(cap_ids)} running nodes, so it forgot the node')
        self.assertIsNotNone(
            self._wait_ids(lambda ids: depart_id not in ids and cap_ids <= ids,
                           LISTED_AFTER_OBSERVED_SEC),
            f'the leftover of {DEPART_FQN} is listed next to {len(cap_ids)} running nodes')

        # Every announced node departs with no entry of it left, then leaves a leftover.
        cap.leave()
        self.assertIsNotNone(
            self._wait_ids(lambda ids: not (ids & cap_ids),
                           LISTED_AFTER_OBSERVED_SEC + PROCESS_EXIT_TIMEOUT_SEC),
            'the announced nodes left, but GET /apps kept listing some of them')
        cap.publish()
        namespace = split_fqn(CAP_FQN)[1]

        def left_over(entries):
            enclaves = [enclave for _, node_namespace, enclave in entries
                        if node_namespace == namespace]
            return len(enclaves) == len(cap_fqns) and set(enclaves) == {''}

        deadline = time.monotonic() + INJECTION_TIMEOUT_SEC
        while (not left_over(self.observer.get_node_names_and_namespaces_with_enclaves())
               and time.monotonic() < deadline):
            time.sleep(0.2)
        self.assertTrue(
            left_over(self.observer.get_node_names_and_namespaces_with_enclaves()),
            f'the test process graph does not list all {len(cap_fqns)} nodes as leftovers')

        # As many leftovers as the capacity allows stay hidden; the rest are listed again.
        listed_count = len(cap_fqns) - (CAPACITY - 1)
        listed = self._wait_ids(
            lambda ids: len(ids & cap_ids) == listed_count, LISTED_AFTER_OBSERVED_SEC)
        self.assertIsNotNone(
            listed,
            f'of {len(cap_ids)} leftovers GET /apps lists {len(self._app_ids() & cap_ids)}; with '
            f'a capacity of {CAPACITY} and one more departed node remembered it must list '
            f'{listed_count}')

        def cap_check(ids):
            self.assertEqual(len(ids & cap_ids), listed_count)
            self.assertNotIn(depart_id, ids, f'the leftover of {DEPART_FQN} is listed')

        self._observe(cap_check)
        self.assertEqual(
            self._wait_warned(proc_output, gateway_node, cap_fqns, CAPACITY - 1), CAPACITY - 1)

        # Past the capacity a node with no entry left is forgotten first: its late sample is
        # listed, and no older leftover takes its place.
        recent_id = split_fqn(RECENT_FQN)[0]
        recent = self._leftover(RECENT_FQN, PUBLISH_ON_COMMAND_DELAY_SEC)
        recent.leave()
        self.assertIsNotNone(
            self._wait_ids(lambda ids: recent_id not in ids,
                           LISTED_AFTER_OBSERVED_SEC + PROCESS_EXIT_TIMEOUT_SEC),
            f'{RECENT_FQN} left, but GET /apps kept listing it')
        recent.publish()
        self._wait_left_over(RECENT_FQN, INJECTION_TIMEOUT_SEC)
        self.assertIsNotNone(
            self._wait_ids(lambda ids: recent_id in ids, LISTED_AFTER_OBSERVED_SEC),
            f'the leftover of {RECENT_FQN} is hidden: past the capacity discovery forgot an '
            'older, listed leftover rather than the name no entry listed')

        def recent_check(ids):
            self.assertIn(recent_id, ids, f'the leftover of {RECENT_FQN} dropped out of GET /apps')
            self.assertEqual(
                len(ids & cap_ids), listed_count,
                'an older leftover was listed in place of the name no entry listed')
            self.assertNotIn(depart_id, ids, f'the leftover of {DEPART_FQN} is listed')

        self._observe(recent_check)
        self.assertEqual(self._warned(proc_output, gateway_node, [RECENT_FQN]), 0)

        name, namespace = split_fqn(LATE_FQN)
        late = subprocess.Popen(
            [fixture_path('demo_rpm_sensor'), '--ros-args', '-r', f'__ns:={namespace}',
             '-r', f'__node:={name}'],
            env=_fixture_env(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self._stop.append(lambda: stop_process(late, PROCESS_EXIT_TIMEOUT_SEC))
        self.assertTrue(
            wait_observed(self.observer, LATE_FQN, lambda enclaves: '/' in enclaves,
                          APPEAR_TIMEOUT_SEC),
            f'the test process graph never listed {LATE_FQN}')
        observed = time.monotonic()
        self.assertIsNotNone(
            self._wait_ids(lambda ids: name in ids, APPEAR_TIMEOUT_SEC, interval=0.1),
            f'{LATE_FQN} never appeared in GET /apps next to {len(cap_ids)} leftovers')
        latency = time.monotonic() - observed
        self.assertLessEqual(
            latency, LISTED_AFTER_OBSERVED_SEC,
            f'{LATE_FQN} took {latency:.2f} s to reach GET /apps after the graph listed it, '
            f'next to {len(cap_ids)} leftovers')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
