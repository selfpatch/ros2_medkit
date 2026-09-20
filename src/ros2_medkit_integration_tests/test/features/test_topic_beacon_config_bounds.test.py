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

"""topic_beacon bounds for beacon_ttl_sec, beacon_expiry_sec, max_messages_per_second, max_hints.

One gateway loads the plugin once per sweep point, each instance on its own topic. Across
the instances every key takes every point: NaN, -inf, below the minimum, the minimum, the
maximum, just above the maximum, +inf and 3e9. More instances take one max_hints point each:
0, -5, 1, 2147483647, 2147483648, 2^32 + 1, 1e12 and NaN. The test publishes beacons for
entities no discovery knows, so the gateway logs each hint an instance stored. The instance
loaded first answers the beacon endpoint; it has NaN TTL and expiry and takes a beacon for
the test's own node.
"""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
import requests
from ros2_medkit_msgs.msg import MedkitDiscoveryHint

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, DEFAULT_BASE_URL, get_time_scale
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

TIME_SCALE = get_time_scale()
APPEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE
# Longer than a 1-per-second bucket takes to refill.
QUIET_SEC = 2.0
BURST = 10

MAX_SECONDS = '2147483647'
KEYS = {
    # key: (minimum, below it, maximum)
    'beacon_ttl_sec': ('0.1', '0.05', MAX_SECONDS),
    'beacon_expiry_sec': ('1', '0.5', MAX_SECONDS),
    'max_messages_per_second': ('1', '0.5', '10000'),
}
POINTS = ('nan', 'neg_inf', 'below', 'min', 'max', 'above', 'inf', 'huge')
LOW_POINTS = POINTS[:4]


def _point(name, key):
    """(YAML spelling, how the plugin prints it, what it clamps to or None) of a sweep point."""
    minimum, below, maximum = KEYS[key]
    above = str(int(maximum) + 1)
    return {
        'nan': ('.nan', 'nan', minimum),
        'neg_inf': ('-.inf', '-inf', minimum),
        'below': (below, below, minimum),
        'min': (minimum + '.0' if '.' not in minimum else minimum, minimum, None),
        'max': (maximum + '.0', maximum, None),
        'above': (above + '.0', above, maximum),
        'inf': ('.inf', 'inf', maximum),
        'huge': ('3000000000.0', '3000000000', maximum),
    }[name]


# Instance k takes point k for the rate, and shifted points for TTL and expiry.
INSTANCES = {
    f'tb_{k}': {
        'max_messages_per_second': POINTS[k],
        'beacon_ttl_sec': POINTS[(k + 3) % len(POINTS)],
        'beacon_expiry_sec': POINTS[(k + 6) % len(POINTS)],
    }
    for k in range(len(POINTS))
}


# max_hints points: instance -> (YAML spelling, warning or None, hints kept of HINTS_PUBLISHED)
HINT_INSTANCES = {
    'tb_hints_zero': ('0', 'max_hints clamped from 0 to 1', 1),
    'tb_hints_negative': ('-5', 'max_hints clamped from -5 to 1', 1),
    'tb_hints_min': ('1', None, 1),
    'tb_hints_max': ('2147483647', None, 2),
    'tb_hints_above': ('2147483648', 'max_hints clamped from 2147483648 to 2147483647', 2),
    'tb_hints_wrapping': ('4294967297', 'max_hints clamped from 4294967297 to 2147483647', 2),
    'tb_hints_double': ('1.0e12', 'max_hints 1e+12 is not an integer, using 10000', 2),
    'tb_hints_nan': ('.nan', 'max_hints nan is not an integer, using 10000', 2),
}
HINTS_PUBLISHED = 2
CAPACITY_WARNING = 'BeaconHintStore capacity reached (max_hints=1)'

# Loaded first, so its route answers the beacon endpoint. NaN TTL and expiry become 0.1 s and 1 s.
LIFETIME = 'tb_lifetime'
LIFETIME_BOUND_SEC = 3.0 * TIME_SCALE


def _topic(instance):
    return f'/{instance}/discovery'


def _parameter_file():
    plugin_path = os.path.join(
        get_package_prefix('ros2_medkit_topic_beacon'), 'lib', 'ros2_medkit_topic_beacon',
        'libtopic_beacon_plugin.so')
    settings = {LIFETIME: {'beacon_ttl_sec': '.nan', 'beacon_expiry_sec': '.nan'}}
    for instance, points in INSTANCES.items():
        settings[instance] = {key: _point(point, key)[0] for key, point in points.items()}
    for instance, (value, _, _) in HINT_INSTANCES.items():
        settings[instance] = {'max_hints': value}
    lines = [
        'ros2_medkit_gateway:',
        '  ros__parameters:',
        '    plugins: [' + ', '.join(settings) + ']',
    ]
    for instance, values in settings.items():
        lines.append(f'    plugins.{instance}.path: "{plugin_path}"')
        lines.append(f'    plugins.{instance}.topic: "{_topic(instance)}"')
        for key, value in values.items():
            lines.append(f'    plugins.{instance}.{key}: {value}')
    handle = tempfile.NamedTemporaryFile(
        'w', prefix='topic_beacon_bounds_', suffix='.yaml', delete=False)
    with handle:
        handle.write('\n'.join(lines) + '\n')
    return handle.name


PARAMETER_FILE = _parameter_file()


def generate_test_description():
    # Frequent refreshes, so every stored hint is logged before a 1 s expiry removes it.
    gateway_node = create_gateway_node(
        extra_params={'refresh_interval_ms': 200}, parameter_files=[PARAMETER_FILE])
    return (
        LaunchDescription([gateway_node, launch_testing.actions.ReadyToTest()]),
        {'gateway_node': gateway_node},
    )


class TestTopicBeaconBounds(unittest.TestCase):

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
        cls.node = rclpy.create_node('topic_beacon_bounds')
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        cls.publishers = {
            instance: cls.node.create_publisher(MedkitDiscoveryHint, _topic(instance), qos)
            for instance in [LIFETIME, *INSTANCES, *HINT_INSTANCES]}
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while (any(p.get_subscription_count() == 0 for p in cls.publishers.values())
               and time.monotonic() < deadline):
            time.sleep(0.05)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        os.remove(PARAMETER_FILE)

    @staticmethod
    def _output(proc_output, gateway_node):
        return ''.join(item.text.decode(errors='replace') for item in proc_output[gateway_node])

    @staticmethod
    def _stored(output, entity_id):
        return f"Beacon entity '{entity_id}' not in known entities" in output

    def _publish(self, instance, entity_id):
        self.publishers[instance].publish(MedkitDiscoveryHint(entity_id=entity_id))

    def test_01_clamps_are_logged(self, proc_output, gateway_node):
        output = self._output(proc_output, gateway_node)
        for instance, points in INSTANCES.items():
            for key, point in points.items():
                _, printed, clamped = _point(point, key)
                if clamped is None:
                    self.assertNotIn(
                        f'{key} clamped from {printed} ', output,
                        f'{instance}: {key} {printed} is in range and must not be clamped')
                else:
                    line = f'{key} clamped from {printed} to {clamped}'
                    self.assertIn(line, output, f'{instance} never logged "{line}"')

    def test_02_beacons_are_stored_and_rate_limited(self, proc_output, gateway_node):
        self.assertTrue(all(p.get_subscription_count() > 0 for p in self.publishers.values()),
                        'an instance never subscribed to its topic')
        pending = set(INSTANCES)
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while pending and time.monotonic() < deadline:
            for instance in pending:
                self._publish(instance, f'{instance}_first')
            time.sleep(0.2)
            output = self._output(proc_output, gateway_node)
            pending = {i for i in pending if not self._stored(output, f'{i}_first')}
        self.assertFalse(pending, f'instances that stored no beacon: {sorted(pending)}')

        time.sleep(QUIET_SEC)
        for instance in INSTANCES:
            for i in range(BURST):
                self._publish(instance, f'{instance}_burst_{i}')

        def stored_from_burst(output):
            return {instance: sum(self._stored(output, f'{instance}_burst_{i}')
                                  for i in range(BURST))
                    for instance in INSTANCES}

        def burst_seen():
            stored = stored_from_burst(self._output(proc_output, gateway_node))
            return all(count == BURST if points['max_messages_per_second'] not in LOW_POINTS
                       else count >= 1 for count, points in
                       ((stored[i], INSTANCES[i]) for i in INSTANCES))

        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while not burst_seen() and time.monotonic() < deadline:
            time.sleep(0.1)
        # Late hints of a rate-limited instance would show within a refresh.
        time.sleep(1.0 * TIME_SCALE)
        stored = stored_from_burst(self._output(proc_output, gateway_node))
        for instance, points in INSTANCES.items():
            rate_point = points['max_messages_per_second']
            if rate_point in LOW_POINTS:
                self.assertTrue(
                    1 <= stored[instance] <= 2,
                    f'{instance} (rate {rate_point}, 1 per second) stored {stored[instance]} of '
                    f'a burst of {BURST}')
            else:
                self.assertEqual(
                    stored[instance], BURST,
                    f'{instance} (rate {rate_point}, 10000 per second) dropped part of a burst')
        self.assertEqual(requests.get(f'{DEFAULT_BASE_URL}/health', timeout=5).status_code, 200)

    def test_03_max_hints_is_read_as_int64(self, proc_output, gateway_node):
        output = self._output(proc_output, gateway_node)
        for instance, (value, warning, _) in HINT_INSTANCES.items():
            if warning is None:
                self.assertNotIn(f'max_hints clamped from {value} to', output,
                                 f'{instance}: max_hints {value} is in range')
            else:
                self.assertIn(warning, output, f'{instance} never logged "{warning}"')

        low = [i for i, (_, _, keeps) in HINT_INSTANCES.items() if keeps == 1]

        def kept(output):
            return {instance: sum(self._stored(output, f'{instance}_{i}')
                                  for i in range(HINTS_PUBLISHED))
                    for instance in HINT_INSTANCES}

        def settled(output):
            counts = kept(output)
            return (all(counts[i] >= keeps for i, (_, _, keeps) in HINT_INSTANCES.items())
                    and output.count(CAPACITY_WARNING) >= len(low))

        # Each low instance keeps one hint and warns once when it refuses the other.
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while not settled(output) and time.monotonic() < deadline:
            for instance in HINT_INSTANCES:
                for i in range(HINTS_PUBLISHED):
                    self._publish(instance, f'{instance}_{i}')
            time.sleep(0.2)
            output = self._output(proc_output, gateway_node)
        counts = kept(output)
        for instance, (value, _, keeps) in HINT_INSTANCES.items():
            self.assertEqual(counts[instance], keeps,
                             f'{instance} (max_hints {value}) kept {counts[instance]} of '
                             f'{HINTS_PUBLISHED} hints')
        self.assertEqual(output.count(CAPACITY_WARNING), len(low),
                         f'only the instances at max_hints 1 ({low}) may reach capacity')

    def test_04_nan_ttl_and_expiry_become_their_minimums(self):
        app_url = f'{DEFAULT_BASE_URL}/apps/{self.node.get_name()}'
        beacon_url = f'{app_url}/x-medkit-topic-beacon'
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while requests.get(app_url, timeout=5).status_code != 200:
            self.assertLess(time.monotonic(), deadline, 'the gateway never listed the test node')
            time.sleep(0.2)

        def state():
            response = requests.get(beacon_url, timeout=5)
            if response.status_code == 200:
                return response.json()['status']
            self.assertEqual(response.status_code, 404, response.text)
            self.assertIn('x-medkit-beacon-not-found', response.text)
            return 'removed'

        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        while True:
            last_publish = time.monotonic()
            self._publish(LIFETIME, self.node.get_name())
            time.sleep(0.05)
            if state() != 'removed':
                break
            self.assertLess(time.monotonic(), deadline, 'the lifetime instance served no beacon')

        # (state, seconds after the last publish) at each change.
        seen = []
        deadline = time.monotonic() + 2 * LIFETIME_BOUND_SEC
        while not (seen and seen[-1][0] == 'removed') and time.monotonic() < deadline:
            current = state()
            if not seen or seen[-1][0] != current:
                seen.append((current, time.monotonic() - last_publish))
            time.sleep(0.02)
        states = [name for name, _ in seen]
        self.assertIn('stale', states, f'the beacon never went stale: {seen}')
        self.assertEqual(states[-1], 'removed', f'the beacon was never removed: {seen}')
        stale_at = dict(seen)['stale']
        removed_at = seen[-1][1]
        self.assertGreaterEqual(stale_at, 0.1, f'stale before a 0.1 s TTL: {seen}')
        self.assertGreaterEqual(removed_at, 1.0, f'removed before a 1 s expiry: {seen}')
        self.assertLess(removed_at, LIFETIME_BOUND_SEC, f'kept past a 1 s expiry: {seen}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
