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

"""parameter_beacon duration and max_hints bounds, and polling of nodes that answer badly or never.

One gateway loads the plugin once per instance, each with its own prefix. The sweep sets
durations to NaN, -inf, below the minimum, the minimum, the maximum, just above it, +inf and
3e9; the steady instance has a working configuration. More instances take one max_hints point
each: 0, -5, 1, 2147483647, 2147483648, 2^32 + 1, 1e12 and NaN. The instance loaded first
answers the beacon endpoint and has NaN TTL and expiry. Polled nodes: answering, unserved
(never answers), get_silent (answers list only), typed_unset (rclpy, a typed parameter without
a value, so its get answers carry no values), two hint sources that give each max_hints
instance a hint of its own, and lifetime_beacon, which gives the first instance a hint for
itself until the test stops it.
"""

import collections
import os
import tempfile
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, ListParameters
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, DEFAULT_BASE_URL, get_time_scale
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

TIME_SCALE = get_time_scale()
APPEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE
# Give-up bound for anything the instances do; every wait ends as soon as its condition holds.
POLL_TIMEOUT_SEC = 30.0 * TIME_SCALE
# A thread busier than this over the CPU window is spinning.
SPIN_CPU_FRACTION = 0.5
CPU_WINDOW_SEC = 3.0

MAX_SECONDS = '2147483647'
DURATIONS = ('poll_interval_sec', 'poll_budget_sec', 'param_timeout_sec', 'beacon_ttl_sec',
             'beacon_expiry_sec')
MINIMUM = dict.fromkeys(DURATIONS, '0.1')
MINIMUM['beacon_expiry_sec'] = '1'
BELOW = dict.fromkeys(DURATIONS, '0.05')
BELOW['beacon_expiry_sec'] = '0.5'


def _point(name, key):
    """(YAML spelling, how the plugin prints it, what it clamps to or None) of a sweep point."""
    return {
        'nan': ('.nan', 'nan', MINIMUM[key]),
        'neg_inf': ('-.inf', '-inf', MINIMUM[key]),
        'below': (BELOW[key], BELOW[key], MINIMUM[key]),
        'min': (MINIMUM[key] if key != 'beacon_expiry_sec' else '1.0', MINIMUM[key], None),
        'max': (MAX_SECONDS + '.0', MAX_SECONDS, None),
        'above': ('2147483648.0', '2147483648', MAX_SECONDS),
        'inf': ('.inf', 'inf', MAX_SECONDS),
        'huge': ('3000000000.0', '3000000000', MAX_SECONDS),
    }[name]


LOW_POINTS = ('nan', 'neg_inf', 'below', 'min')
HIGH_POINTS = ('max', 'above', 'inf', 'huge')
# Every duration at a low point: the instance keeps cycling with 0.1 s timeouts.
CYCLING = {f'low_{p}': {key: p for key in DURATIONS} for p in LOW_POINTS}
# Timeout, budget, TTL and expiry at a high point, the interval at its minimum: the instance
# reaches a node that never answers and waits there.
WAITING = {f'high_{p}': dict({key: p for key in DURATIONS[1:]}, poll_interval_sec='min')
           for p in HIGH_POINTS}
# The interval at a high point: the instance runs one cycle.
ONE_CYCLE = {f'interval_{p}': {'poll_interval_sec': p} for p in HIGH_POINTS}
SWEEP = dict(**CYCLING, **WAITING, **ONE_CYCLE)
STEADY = 'steady'
STEADY_CONFIG = {'poll_interval_sec': '0.1', 'poll_budget_sec': '30.0', 'param_timeout_sec': '0.3'}
# Bound for the steady counts to settle: a list answer of get_silent is counted before its get.
SETTLE_TIMEOUT_SEC = 1.0 * TIME_SCALE

# max_hints points: instance -> (YAML spelling, warning or None, hints kept of the two sources)
HINT_INSTANCES = {
    'hints_zero': ('0', 'max_hints clamped from 0 to 1', 1),
    'hints_negative': ('-5', 'max_hints clamped from -5 to 1', 1),
    'hints_min': ('1', None, 1),
    'hints_max': ('2147483647', None, 2),
    'hints_above': ('2147483648', 'max_hints clamped from 2147483648 to 2147483647', 2),
    'hints_wrapping': ('4294967297', 'max_hints clamped from 4294967297 to 2147483647', 2),
    'hints_double': ('1.0e12', 'max_hints 1e+12 is not an integer, using 10000', 2),
    'hints_nan': ('.nan', 'max_hints nan is not an integer, using 10000', 2),
}
HINT_CONFIG = {'poll_interval_sec': '1.0', 'param_timeout_sec': '0.1'}
HINT_SOURCES = ('a', 'b')
CAPACITY_WARNING = 'BeaconHintStore capacity reached (max_hints=1)'

# Loaded first, so its route answers the beacon endpoint. TTL becomes 3 * 0.1 s, expiry 1 s.
LIFETIME = 'lifetime'
LIFETIME_CONFIG = {'poll_interval_sec': '0.1', 'param_timeout_sec': '0.3',
                   'beacon_ttl_sec': '.nan', 'beacon_expiry_sec': '.nan'}
LIFETIME_TTL_SEC = 0.3
LIFETIME_EXPIRY_SEC = 1.0
# How late the test may see the beacon go stale or be removed.
LIFETIME_LATENESS_SEC = 0.3 * TIME_SCALE


def _prefix(instance):
    return f'oor_{instance}'


def _parameter_file():
    plugin_path = os.path.join(
        get_package_prefix('ros2_medkit_param_beacon'), 'lib', 'ros2_medkit_param_beacon',
        'libparam_beacon_plugin.so')
    settings = {LIFETIME: LIFETIME_CONFIG}
    for instance, points in SWEEP.items():
        settings[instance] = {key: _point(point, key)[0] for key, point in points.items()}
    settings[STEADY] = STEADY_CONFIG
    for instance, (value, _, _) in HINT_INSTANCES.items():
        settings[instance] = dict(HINT_CONFIG, max_hints=value)
    lines = [
        'ros2_medkit_gateway:',
        '  ros__parameters:',
        '    plugins: [' + ', '.join(settings) + ']',
    ]
    for instance, values in settings.items():
        lines.append(f'    plugins.{instance}.path: "{plugin_path}"')
        lines.append(f'    plugins.{instance}.parameter_prefix: "{_prefix(instance)}"')
        for key, value in values.items():
            lines.append(f'    plugins.{instance}.{key}: {value}')
    handle = tempfile.NamedTemporaryFile(
        'w', prefix='param_beacon_bounds_', suffix='.yaml', delete=False)
    with handle:
        handle.write('\n'.join(lines) + '\n')
    return handle.name


PARAMETER_FILE = _parameter_file()


def generate_test_description():
    gateway_node = create_gateway_node(parameter_files=[PARAMETER_FILE])
    return (
        LaunchDescription([gateway_node, launch_testing.actions.ReadyToTest()]),
        {'gateway_node': gateway_node},
    )


def _prefix_of(name):
    return name.split('.', 1)[0]


class _Counter:
    """Thread-safe request counts per (node, service, prefix)."""

    def __init__(self):
        self._counts = collections.Counter()
        self._lock = threading.Lock()

    def add(self, node, service, prefixes):
        with self._lock:
            for prefix in prefixes:
                self._counts[(node, service, prefix)] += 1

    def get(self, node, service, instance):
        with self._lock:
            return self._counts[(node, service, _prefix(instance))]

    def snapshot(self):
        with self._lock:
            return collections.Counter(self._counts)


class _UnansweredServices:
    """Takes requests from services no executor serves, counts them and never answers."""

    def __init__(self, counter):
        self._counter = counter
        self._services = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def add(self, node_label, service):
        self._services.append((node_label, service))

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=10)

    def _run(self):
        while not self._stop.is_set():
            for node_label, service in self._services:
                while True:
                    with service.handle:
                        request, _header = service.handle.service_take_request(
                            service.srv_type.Request)
                    if request is None:
                        break
                    if service.srv_type is ListParameters:
                        self._counter.add(node_label, 'list', request.prefixes)
                    else:
                        self._counter.add(node_label, 'get',
                                          {_prefix_of(name) for name in request.names})
            self._stop.wait(0.02)


def _thread_cpu_ticks(pid):
    ticks = {}
    for tid in os.listdir(f'/proc/{pid}/task'):
        try:
            with open(f'/proc/{pid}/task/{tid}/stat') as stat:
                fields = stat.read().rsplit(')', 1)[1].split()
        except OSError:
            continue
        ticks[tid] = int(fields[11]) + int(fields[12])
    return ticks


class TestParamBeaconBounds(unittest.TestCase):

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
        cls.counter = _Counter()
        cls.nodes = []

        # The max_hints instances get their hints from the hint sources only.
        hint_prefixes = {_prefix(instance) for instance in HINT_INSTANCES}
        answering = cls._node('answering_beacon')
        answering.create_service(
            ListParameters, '~/list_parameters', cls._answer_list('answering', hint_prefixes))
        answering.create_service(GetParameters, '~/get_parameters', cls._answer_get)

        get_silent = cls._node('get_silent')
        get_silent.create_service(
            ListParameters, '~/list_parameters', cls._answer_list('get_silent'))
        # Its get service lives on a hidden node no executor spins.
        get_backend = cls._node('_get_silent_backend')
        get_silent_get = get_backend.create_service(
            GetParameters, '/get_silent/get_parameters', cls._answer_get)

        cls.unserved = cls._node('unserved_parameters')
        unserved_list = cls.unserved.create_service(
            ListParameters, '~/list_parameters', cls._answer_list('unserved'))
        unserved_get = cls.unserved.create_service(
            GetParameters, '~/get_parameters', cls._answer_get)

        typed_unset = rclpy.create_node('typed_unset')
        cls.nodes.append(typed_unset)
        typed_unset.declare_parameter(f'{_prefix(STEADY)}.entity_id', 'typed_unset')
        typed_unset.declare_parameter(f'{_prefix(STEADY)}.display_name', 'Typed Unset')
        typed_unset.declare_parameter(f'{_prefix(STEADY)}.stable_id', Parameter.Type.STRING)
        get_service = next(
            service for service in typed_unset.services
            if service.srv_name.endswith('/get_parameters'))
        rclpy_get = get_service.callback

        def counting_get(request, response):
            cls.counter.add('typed_unset', 'get', {_prefix_of(name) for name in request.names})
            return rclpy_get(request, response)

        get_service.callback = counting_get

        sources = []
        for label in HINT_SOURCES:
            source = cls._node(f'hint_source_{label}')
            source.create_service(ListParameters, '~/list_parameters',
                                  cls._answer_list_for(hint_prefixes))
            source.create_service(GetParameters, '~/get_parameters', cls._answer_hint(label))
            sources.append(source)

        cls.lifetime_serving = threading.Event()
        cls.lifetime_serving.set()
        cls.lifetime_last_get = None
        lifetime = cls._node('lifetime_beacon')
        lifetime.create_service(ListParameters, '~/list_parameters', cls._answer_lifetime_list)
        lifetime.create_service(GetParameters, '~/get_parameters', cls._answer_lifetime_get)

        cls.unanswered = _UnansweredServices(cls.counter)
        cls.unanswered.add('unserved', unserved_list)
        cls.unanswered.add('unserved', unserved_get)
        cls.unanswered.add('get_silent', get_silent_get)
        cls.unanswered.start()

        cls.executor = SingleThreadedExecutor()
        for node in (answering, get_silent, typed_unset, *sources, lifetime):
            cls.executor.add_node(node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.unanswered.stop()
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=10)
        for node in cls.nodes:
            node.destroy_node()
        rclpy.shutdown()
        os.remove(PARAMETER_FILE)

    @classmethod
    def _node(cls, name):
        node = rclpy.create_node(name, start_parameter_services=False)
        cls.nodes.append(node)
        return node

    @classmethod
    def _answer_list(cls, label, skipped=frozenset()):
        def answer(request, response):
            cls.counter.add(label, 'list', request.prefixes)
            response.result.names = [
                f'{prefix}.entity_id' for prefix in request.prefixes if prefix not in skipped]
            return response
        return answer

    @staticmethod
    def _answer_get(request, response):
        response.values = [
            ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='answering_beacon')
            for _ in request.names]
        return response

    @staticmethod
    def _answer_list_for(prefixes):
        def answer(request, response):
            response.result.names = [
                f'{prefix}.entity_id' for prefix in request.prefixes if prefix in prefixes]
            return response
        return answer

    @staticmethod
    def _answer_hint(label):
        """Entity id '<prefix>_<label>', so each instance stores a hint no discovery knows."""
        def answer(request, response):
            response.values = [
                ParameterValue(type=ParameterType.PARAMETER_STRING,
                               string_value=f'{_prefix_of(name)}_{label}')
                for name in request.names]
            return response
        return answer

    @classmethod
    def _answer_lifetime_list(cls, request, response):
        if cls.lifetime_serving.is_set():
            response.result.names = [
                f'{prefix}.entity_id' for prefix in request.prefixes
                if prefix == _prefix(LIFETIME)]
        return response

    @classmethod
    def _answer_lifetime_get(cls, request, response):
        response.values = [
            ParameterValue(type=ParameterType.PARAMETER_STRING, string_value='lifetime_beacon')
            for _ in request.names]
        cls.lifetime_last_get = time.monotonic()
        return response

    def _count(self, node, service, instance):
        return self.counter.get(node, service, instance)

    def _unanswered(self, instance):
        """Count the requests of an instance that no node answered."""
        return (self._count('unserved', 'list', instance)
                + self._count('unserved', 'get', instance)
                + self._count('get_silent', 'get', instance))

    def _cycles(self, instance):
        return self._count('answering', 'list', instance)

    def _steady_counts(self):
        counts = self.counter.snapshot()
        prefix = _prefix(STEADY)
        return {
            'cycles': counts[('answering', 'list', prefix)],
            'typed_unset_gets': counts[('typed_unset', 'get', prefix)],
            'get_silent_lists': counts[('get_silent', 'list', prefix)],
            'get_silent_gets': counts[('get_silent', 'get', prefix)],
            'unserved_lists': counts[('unserved', 'list', prefix)],
        }

    def _settled_steady_counts(self):
        """Steady counts once every list answer of get_silent has its get counted."""
        counts = self._steady_counts()
        deadline = time.monotonic() + SETTLE_TIMEOUT_SEC
        while (counts['get_silent_lists'] != counts['get_silent_gets']
               and time.monotonic() < deadline):
            time.sleep(0.005)
            counts = self._steady_counts()
        self.assertEqual(
            counts['get_silent_lists'], counts['get_silent_gets'],
            f'get_silent answered {counts["get_silent_lists"]} list requests of the steady '
            f'instance and got {counts["get_silent_gets"]} get requests: every list answer must '
            'lead to a get')
        return counts

    def _unserved_writers(self):
        infos = self.unserved.get_publishers_info_by_topic(
            'rq/unserved_parameters/list_parametersRequest', no_mangle=True)
        return {bytes(info.endpoint_gid) for info in infos
                if info.node_name == '_param_beacon_node'}

    def _wait(self, predicate, timeout, on_poll=None):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            if on_poll:
                on_poll()
            time.sleep(0.05)
        return predicate()

    def test_01_clamps_are_logged(self, proc_output, gateway_node):
        output = ''.join(
            item.text.decode(errors='replace') for item in proc_output[gateway_node])
        for instance, points in SWEEP.items():
            for key, point in points.items():
                _, printed, clamped = _point(point, key)
                if clamped is None:
                    self.assertNotIn(
                        f'{key} clamped from {printed} ', output,
                        f'{instance}: {key} {printed} is in range and must not be clamped')
                else:
                    line = f'{key} clamped from {printed} to {clamped}'
                    self.assertIn(line, output, f'{instance} never logged "{line}"')

    def test_02_instances_poll_as_their_durations_say(self, gateway_node):
        self.assertTrue(
            self._wait(lambda: all(self._unanswered(i) >= 1 for i in [*CYCLING, *WAITING]),
                       POLL_TIMEOUT_SEC),
            'instances that never sent a request to a node that never answers: '
            f'{[i for i in [*CYCLING, *WAITING] if self._unanswered(i) == 0]}; the graph '
            f'lists {sorted(self.unserved.get_node_names_and_namespaces())} on domain '
            f'{os.environ.get("ROS_DOMAIN_ID")}')
        self.assertTrue(
            self._wait(lambda: self._steady_counts()['unserved_lists'] >= 1
                       and self._steady_counts()['get_silent_gets'] >= 1, POLL_TIMEOUT_SEC),
            f'the steady instance never polled both silent nodes: {self._steady_counts()}')

        start = self._settled_steady_counts()
        cycling_start = {i: self._cycles(i) for i in CYCLING}
        waiting_start = {i: (self._cycles(i), self._unanswered(i)) for i in WAITING}
        writers_seen = self._unserved_writers()
        pid = gateway_node.process_details['pid']
        cpu_start = _thread_cpu_ticks(pid)
        window_start = time.monotonic()

        def sample_writers():
            writers_seen.update(self._unserved_writers())

        def cycling_progress():
            return {i: self._cycles(i) - cycling_start[i] for i in CYCLING}

        def window_done():
            now = self._steady_counts()
            return (time.monotonic() - window_start >= CPU_WINDOW_SEC
                    and now['cycles'] - start['cycles'] >= 12
                    and now['unserved_lists'] - start['unserved_lists'] >= 1
                    and all(self._cycles(i) - cycling_start[i] >= 2 for i in CYCLING))

        self.assertTrue(self._wait(window_done, POLL_TIMEOUT_SEC, sample_writers),
                        f'the steady instance went from {start} to {self._steady_counts()}; '
                        'cycling instances polled the answering node '
                        f'{cycling_progress()} more times')
        # A writer that disappears or appears late shows only after discovery settles.
        self._wait(lambda: False, 1.0 * TIME_SCALE, sample_writers)
        cpu_end = _thread_cpu_ticks(pid)
        window = time.monotonic() - window_start
        end = self._settled_steady_counts()

        busiest = max((cpu_end[tid] - cpu_start.get(tid, 0)) for tid in cpu_end)
        fraction = busiest / os.sysconf('SC_CLK_TCK') / window
        self.assertLess(
            fraction, SPIN_CPU_FRACTION,
            f'a gateway thread used {fraction:.0%} of a core over {window:.1f} s')

        for instance in WAITING:
            cycles, unanswered = waiting_start[instance]
            self.assertEqual(
                (self._cycles(instance), self._unanswered(instance)), (cycles, 1),
                f'{instance} must wait for its first request that gets no answer and send '
                'nothing more')
        for instance in ONE_CYCLE:
            self.assertLessEqual(self._cycles(instance), 1, f'{instance} ran more than one cycle')
            self.assertLessEqual(self._unanswered(instance), 1)

        cycles = end['cycles'] - start['cycles']
        typed_unset_gets = end['typed_unset_gets'] - start['typed_unset_gets']
        self.assertGreaterEqual(
            typed_unset_gets, cycles - 1,
            f'the node whose get answer has no values was asked on {typed_unset_gets} of '
            f'{cycles} cycles: it was backed off')
        get_silent_lists = end['get_silent_lists'] - start['get_silent_lists']
        self.assertEqual(end['get_silent_gets'] - start['get_silent_gets'], get_silent_lists,
                         'every list answer of get_silent must lead to a get that times out')
        self.assertLessEqual(
            2 * get_silent_lists, cycles + 2,
            f'the node whose get requests time out was asked on {get_silent_lists} of {cycles} '
            'cycles: it was not backed off')

        final = self._unserved_writers()
        self.assertEqual(
            writers_seen, final,
            'a parameter client for the unserved node was replaced while the instances kept '
            f'polling it: {len(writers_seen)} request writers seen, {len(final)} at the end')
        self.assertTrue(final, 'no instance holds a client for the unserved node')
        self.assertEqual(requests.get(f'{DEFAULT_BASE_URL}/health', timeout=5).status_code, 200)

    def test_03_max_hints_is_read_as_int64(self, proc_output, gateway_node):
        def output():
            return ''.join(
                item.text.decode(errors='replace') for item in proc_output[gateway_node])

        logged = output()
        for instance, (value, warning, _) in HINT_INSTANCES.items():
            if warning is None:
                self.assertNotIn(f'max_hints clamped from {value} to', logged,
                                 f'{instance}: max_hints {value} is in range')
            else:
                self.assertIn(warning, logged, f'{instance} never logged "{warning}"')

        low = [i for i, (_, _, keeps) in HINT_INSTANCES.items() if keeps == 1]

        def kept(logged):
            return {instance: sum(f"Beacon entity '{_prefix(instance)}_{label}' not in known "
                                  'entities' in logged for label in HINT_SOURCES)
                    for instance in HINT_INSTANCES}

        # Each low instance keeps one hint and warns once when it refuses the other.
        def settled():
            logged = output()
            counts = kept(logged)
            return (all(counts[i] >= keeps for i, (_, _, keeps) in HINT_INSTANCES.items())
                    and logged.count(CAPACITY_WARNING) >= len(low))

        self._wait(settled, POLL_TIMEOUT_SEC)
        logged = output()
        counts = kept(logged)
        for instance, (value, _, keeps) in HINT_INSTANCES.items():
            self.assertEqual(counts[instance], keeps,
                             f'{instance} (max_hints {value}) kept {counts[instance]} of '
                             f'{len(HINT_SOURCES)} hints')
        self.assertEqual(logged.count(CAPACITY_WARNING), len(low),
                         f'only the instances at max_hints 1 ({low}) may reach capacity')

    def test_04_nan_ttl_and_expiry_become_their_minimums(self):
        app_url = f'{DEFAULT_BASE_URL}/apps/lifetime_beacon'
        beacon_url = f'{app_url}/x-medkit-param-beacon'

        def state():
            response = requests.get(beacon_url, timeout=5)
            if response.status_code == 200:
                return response.json()['status']
            self.assertEqual(response.status_code, 404, response.text)
            self.assertIn('x-medkit-beacon-not-found', response.text)
            return 'removed'

        self.assertTrue(
            self._wait(lambda: requests.get(app_url, timeout=5).status_code == 200
                       and state() != 'removed', POLL_TIMEOUT_SEC),
            'the lifetime instance never served a beacon for lifetime_beacon')
        # Stop refreshing while the beacon is active, so the test sees it go stale.
        deadline = time.monotonic() + POLL_TIMEOUT_SEC
        while state() != 'active':
            self.assertLess(time.monotonic(), deadline, 'the beacon was never active')
            time.sleep(0.02)
        self.lifetime_serving.clear()

        # (state, seconds after the last get answer) at each change. The plugin takes that
        # answer later, so the hint is never older than this.
        seen = []
        deadline = time.monotonic() + LIFETIME_EXPIRY_SEC + LIFETIME_LATENESS_SEC
        while not (seen and seen[-1][0] == 'removed') and time.monotonic() < deadline:
            current = state()
            if not seen or seen[-1][0] != current:
                seen.append((current, time.monotonic() - self.lifetime_last_get))
            time.sleep(0.02)
        states = [name for name, _ in seen]
        self.assertIn('stale', states, f'the beacon never went stale: {seen}')
        self.assertEqual(states[-1], 'removed', f'the beacon was never removed: {seen}')
        stale_at = [age for name, age in seen if name == 'stale'][-1]
        self.assertGreaterEqual(stale_at, LIFETIME_TTL_SEC,
                                f'stale before the {LIFETIME_TTL_SEC} s TTL: {seen}')
        self.assertLess(stale_at, LIFETIME_TTL_SEC + LIFETIME_LATENESS_SEC,
                        f'not stale within {LIFETIME_LATENESS_SEC:.1f} s of the TTL: {seen}')
        removed_at = seen[-1][1]
        self.assertGreaterEqual(removed_at, LIFETIME_EXPIRY_SEC,
                                f'removed before the {LIFETIME_EXPIRY_SEC} s expiry: {seen}')
        self.assertLess(removed_at, LIFETIME_EXPIRY_SEC + LIFETIME_LATENESS_SEC,
                        f'not removed within {LIFETIME_LATENESS_SEC:.1f} s of the expiry: {seen}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
