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

"""A trigger on /{entity}/logs fires for the entries GET /{entity}/logs lists.

Checked with the gateway's own log buffer (no LogProvider plugin) and far
below the change-queue and pending-event bounds.

Four gateways share one ROS graph:

- runtime: runtime_only. Two ``temp_sensor`` nodes collide, ``rpm_sensor`` does
  not, ``engine_temp`` is a root-namespace node.
- hybrid, manifest: one manifest, app ids differ from node names. The app id
  ``engine_temp`` equals the root node's name. Areas with a subarea, two
  components, a function hosted by an app and a component.
- capped: runtime_only with ``logs.buffer_size`` 3, so the distinct-logger cap
  (buffer size x 10) drops entries from new logger names.

The test publishes /rosout entries under the logger names nodes use (dot form),
the slash form, child loggers and a root-namespace name. Each stream is checked
three ways: the entries its trigger delivered, the entries GET lists, and the
owners written in the tables below. A sentinel entry published after every
probe proves each stream live, so an absent event is a real absence.
"""

import json
import os
import re
import tempfile
import threading
import time
import unittest
import uuid

from launch import LaunchDescription
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rcl_interfaces.msg import Log
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.coverage import get_coverage_env
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

RUNTIME_PORT = get_test_port(0)
HYBRID_PORT = get_test_port(1)
MANIFEST_PORT = get_test_port(2)
CAPPED_PORT = get_test_port(3)


def _base_url(port):
    return f'http://localhost:{port}{API_BASE_PATH}'


RUNTIME_URL = _base_url(RUNTIME_PORT)
HYBRID_URL = _base_url(HYBRID_PORT)
MANIFEST_URL = _base_url(MANIFEST_PORT)
CAPPED_URL = _base_url(CAPPED_PORT)

TIME_SCALE = get_time_scale()
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE
DISCOVERY_TIMEOUT_SEC = 60.0 * TIME_SCALE
DELIVERY_TIMEOUT_SEC = 20.0 * TIME_SCALE
POLL_INTERVAL_SEC = 0.3

# Every marker this run writes starts with RUN, so entries of other runs and
# of the demo nodes never match.
RUN = uuid.uuid4().hex[:10]
MARKER = re.compile(rf'{RUN}:([A-Za-z0-9_.-]+)')

INFO = Log.INFO
WARN = Log.WARN

PROBE_NODE = 'log_trigger_probe'

# (executable, node name, namespace)
GRAPH_NODES = [
    ('demo_engine_temp_sensor', 'temp_sensor', '/powertrain/engine'),
    ('demo_engine_temp_sensor', 'temp_sensor', '/backup'),
    ('demo_rpm_sensor', 'rpm_sensor', '/powertrain/engine'),
    ('demo_rpm_sensor', 'engine_temp', '/'),
    # Logs only at startup, so its buffer on the capped gateway holds what the
    # test writes.
    ('demo_calibration_service', 'cap_probe', '/capped'),
]

MANIFEST_YAML = """\
manifest_version: "1.0"
metadata:
  name: "log trigger scope"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: powertrain
    name: "Powertrain"
    namespace: /powertrain
    subareas:
      - id: engine
        name: "Engine"
        namespace: /powertrain/engine
  - id: backup-area
    name: "Backup"
    namespace: /backup
components:
  - id: engine-ecu
    name: "Engine ECU"
    area: engine
    namespace: /powertrain
  - id: backup-ecu
    name: "Backup ECU"
    area: backup-area
apps:
  - id: engine_temp
    name: "Engine temperature"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: /powertrain/engine
  - id: engine-rpm
    name: "Engine RPM"
    is_located_on: engine-ecu
    ros_binding:
      node_name: rpm_sensor
      namespace: /powertrain/engine
  - id: backup-temp
    name: "Backup temperature"
    is_located_on: backup-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: /backup
functions:
  - id: engine-watch
    name: "Engine watch"
    hosted_by:
      - engine-rpm
      - backup-ecu
"""

# (case id, logger name, level)
PROBES = [
    ('tw1-dot', 'powertrain.engine.temp_sensor', INFO),
    ('tw1-slash', 'powertrain/engine/temp_sensor', INFO),
    ('tw1-child-dot', 'powertrain.engine.temp_sensor.worker', INFO),
    # Child logger whose last segment is the runtime app id of another node.
    ('tw1-child-slash', 'powertrain/engine/temp_sensor/rpm_sensor', INFO),
    ('tw2-warn', 'backup.temp_sensor', WARN),
    ('tw2-info', 'backup.temp_sensor', INFO),
    ('rpm-dot', 'powertrain.engine.rpm_sensor', INFO),
    ('rpm-slash', 'powertrain/engine/rpm_sensor', INFO),
    # Root-namespace node. Namesake of the manifest app id engine_temp.
    ('root', 'engine_temp', INFO),
    ('ns-token', 'powertrain', INFO),
    # Under the engine-ecu fqn, not under the /powertrain/engine namespace.
    ('ecu-child', 'powertrain.engine-ecu.fw', INFO),
    ('stranger', 'unrelated/node', INFO),
]
PROBE_IDS = {case for case, _, _ in PROBES}

# Published after every probe. Each has the logger name and level of the
# probe it maps to, so it has the same owners.
SENTINELS = {
    's-tw1': 'tw1-dot',
    's-tw2': 'tw2-warn',
    's-rpm': 'rpm-dot',
    's-root': 'root',
}
_PROBE_BY_ID = {case: (name, level) for case, name, level in PROBES}

HOST = '{host}'

# Owners per the log routes' rule: exact sources (slash or dot form) for every
# entity type, plus the namespace prefix for components and areas.
RUNTIME_EXPECTED = {
    'apps/powertrain_engine_temp_sensor': {'tw1-dot', 'tw1-slash'},
    # severity_filter is set to warning on this app.
    'apps/backup_temp_sensor': {'tw2-warn'},
    'apps/rpm_sensor': {'rpm-dot', 'rpm-slash'},
    'apps/engine_temp': {'root'},
    f'components/{HOST}': {
        'tw1-dot', 'tw1-slash', 'tw2-warn', 'tw2-info', 'rpm-dot', 'rpm-slash', 'root',
    },
    'functions/powertrain': {'tw1-dot', 'tw1-slash', 'rpm-dot', 'rpm-slash'},
    'functions/backup': {'tw2-warn', 'tw2-info'},
}
MANIFEST_EXPECTED = {
    'apps/engine_temp': {'tw1-dot', 'tw1-slash'},
    'apps/engine-rpm': {'rpm-dot', 'rpm-slash'},
    'apps/backup-temp': {'tw2-warn', 'tw2-info'},
    'components/engine-ecu': {'tw1-dot', 'tw1-slash', 'rpm-dot', 'rpm-slash', 'ecu-child'},
    'components/backup-ecu': {'tw2-warn', 'tw2-info'},
    'areas/powertrain': {
        'tw1-dot', 'tw1-slash', 'tw1-child-dot', 'tw1-child-slash', 'rpm-dot', 'rpm-slash',
        'ns-token', 'ecu-child',
    },
    'areas/engine': {
        'tw1-dot', 'tw1-slash', 'tw1-child-dot', 'tw1-child-slash', 'rpm-dot', 'rpm-slash',
    },
    'areas/backup-area': {'tw2-warn', 'tw2-info'},
    'functions/engine-watch': {'rpm-dot', 'rpm-slash', 'tw2-warn', 'tw2-info'},
}
GATEWAY_EXPECTED = {
    RUNTIME_URL: RUNTIME_EXPECTED,
    HYBRID_URL: MANIFEST_EXPECTED,
    MANIFEST_URL: MANIFEST_EXPECTED,
}


def _expected_sentinels(owned):
    return {s for s, twin in SENTINELS.items() if twin in owned}


_MANIFEST_PATHS = []


def _write_manifest():
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_triggers_logs_scope_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(MANIFEST_YAML)
    _MANIFEST_PATHS.append(path)
    return path


def generate_test_description():
    manifest_path = _write_manifest()
    # Up to nine streams per gateway at once; one stream pins one HTTP worker.
    streams = {'sse.max_clients': 12, 'server.http_thread_pool_size': 20}

    def gateway(name, port, params):
        merged = dict(streams)
        merged.update(params)
        return create_gateway_node(port=port, name=name, extra_params=merged)

    manifest_params = {
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
    }
    gateways = [
        gateway('gw_runtime', RUNTIME_PORT, {'discovery.mode': 'runtime_only'}),
        gateway('gw_hybrid', HYBRID_PORT, dict(
            manifest_params, **{
                'discovery.mode': 'hybrid',
                'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': False,
            })),
        gateway('gw_manifest', MANIFEST_PORT, dict(
            manifest_params, **{'discovery.mode': 'manifest_only'})),
        gateway('gw_capped', CAPPED_PORT, {
            'discovery.mode': 'runtime_only',
            'logs.buffer_size': 3,
        }),
    ]

    env = get_coverage_env('ros2_medkit_integration_tests')
    nodes = [
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable=executable,
            name=name,
            namespace=namespace,
            output='screen',
            additional_env=env,
            sigterm_timeout='30',
            sigkill_timeout='15',
        )
        for executable, name, namespace in GRAPH_NODES
    ]

    return (
        LaunchDescription(gateways + nodes + [launch_testing.actions.ReadyToTest()]),
        {'gateway_node': gateways[0]},
    )


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------

def _poll(predicate, *, what, timeout=DISCOVERY_TIMEOUT_SEC):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
            if last:
                return last
        except requests.exceptions.RequestException as exc:
            last = exc
        time.sleep(POLL_INTERVAL_SEC)
    raise AssertionError(f'timed out after {timeout}s waiting for {what}; last={last!r}')


def _status(url):
    return requests.get(url, timeout=HTTP_TIMEOUT_SEC).status_code


def _wait_for_entities(base_url, paths):
    for path in paths:
        _poll(lambda p=path: _status(f'{base_url}/{p}') == 200,
              what=f'{base_url}/{path}')


def _host_component(base_url):
    def one_component():
        items = requests.get(f'{base_url}/components', timeout=HTTP_TIMEOUT_SEC).json()
        items = items.get('items', [])
        return items[0]['id'] if len(items) == 1 else None
    return _poll(one_component, what=f'{base_url} to serve its host component')


def _listed(base_url, path):
    """Return the RUN markers GET /{path}/logs lists, with their entry ids."""
    r = requests.get(f'{base_url}/{path}/logs', timeout=HTTP_TIMEOUT_SEC)
    r.raise_for_status()
    found = {}
    for item in r.json().get('items', []):
        match = MARKER.search(item.get('message', ''))
        if match:
            found.setdefault(match.group(1), []).append(item.get('id'))
    return found


def _configure_logs(base_url, path, **fields):
    body = {'max_entries': 10000}
    body.update(fields)
    r = requests.put(f'{base_url}/{path}/logs/configuration', json=body,
                     timeout=HTTP_TIMEOUT_SEC)
    assert r.status_code == 204, f'PUT {base_url}/{path}/logs/configuration: {r.text}'


class EventStream:
    """One trigger plus a thread reading its SSE event stream."""

    def __init__(self, base_url, path, body=None):
        self.base_url = base_url
        self.path = path
        self.body = body or {
            'resource': f'{API_BASE_PATH}/{path}/logs',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 600,
        }
        self.trigger_id = None
        self._events_url = None
        self.payloads = []
        self.error = None
        self._cond = threading.Condition()
        self._resp = None
        self._closed = False
        self._thread = None

    def start(self):
        return self.create().connect()

    def create(self):
        """POST the trigger; its events queue until connect() reads them."""
        r = requests.post(f'{self.base_url}/{self.path}/triggers', json=self.body,
                          timeout=HTTP_TIMEOUT_SEC)
        assert r.status_code == 201, f'POST {self.base_url}/{self.path}/triggers: {r.text}'
        trigger = r.json()
        self.trigger_id = trigger['id']
        self._events_url = self.base_url + trigger['event_source'].removeprefix(API_BASE_PATH)
        return self

    def connect(self):
        events_url = self._events_url
        self._resp = requests.get(events_url, stream=True,
                                  timeout=(HTTP_TIMEOUT_SEC, 120.0 * TIME_SCALE))
        assert self._resp.status_code == 200, f'GET {events_url}: {self._resp.status_code}'
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()
        return self

    def _read(self):
        try:
            for line in self._resp.iter_lines(chunk_size=None, decode_unicode=True):
                if not line or not line.startswith('data: '):
                    continue
                envelope = json.loads(line[len('data: '):])
                with self._cond:
                    self.payloads.append(envelope.get('payload'))
                    self._cond.notify_all()
        except Exception as exc:
            if not self._closed:
                self.error = exc

    def markers(self):
        """Return {marker: [entry id, ...]} over the log entries delivered so far."""
        with self._cond:
            payloads = list(self.payloads)
        found = {}
        for payload in payloads:
            if not isinstance(payload, dict):
                continue
            match = MARKER.search(payload.get('message', ''))
            if match:
                found.setdefault(match.group(1), []).append(payload.get('id'))
        return found

    def ordered_markers(self):
        """Return the markers of the delivered entries in delivery order."""
        with self._cond:
            payloads = list(self.payloads)
        return [m.group(1) for m in (MARKER.search(p.get('message', ''))
                                     for p in payloads if isinstance(p, dict)) if m]

    def event_count(self):
        with self._cond:
            return len(self.payloads)

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        with self._cond:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(timeout=min(remaining, 0.5))
        return True

    def close(self):
        self._closed = True
        if self.trigger_id:
            try:
                requests.delete(f'{self.base_url}/{self.path}/triggers/{self.trigger_id}',
                                timeout=HTTP_TIMEOUT_SEC)
            except requests.exceptions.RequestException:
                pass
        if self._resp is not None:
            self._resp.close()


class RosoutProbe:
    """Publish rcl_interfaces/Log entries on /rosout from one publisher."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node(PROBE_NODE)
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.node.create_publisher(Log, '/rosout', qos)
        # Warm-up entries of an earlier publisher prove nothing about this one.
        self.token = uuid.uuid4().hex[:6]

    def publish(self, case_id, logger_name, level=INFO):
        msg = Log()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.level = level
        msg.name = logger_name
        msg.msg = f'{RUN}:{case_id}'
        msg.file = 'test_triggers_logs_scope.test.py'
        msg.function = 'publish'
        msg.line = 1
        self.pub.publish(msg)

    def wait_for_gateways(self, count):
        _poll(lambda: self.pub.get_subscription_count() >= count,
              what=f'{count} /rosout subscribers')

    def warm_up(self, checks, logger_name):
        """Publish on logger_name until every (base_url, path) GET lists it."""
        attempt = 0
        prefix = f'warm-{self.token}-'

        def all_listed():
            nonlocal attempt
            attempt += 1
            self.publish(f'{prefix}{attempt}', logger_name)
            time.sleep(0.2)
            return all(
                any(m.startswith(prefix) for m in _listed(url, path)) for url, path in checks
            )
        _poll(all_listed, what=f'{checks} to list a warm-up entry')

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# Distinct-logger cap: a dropped entry is never listed, so no trigger fires
# --------------------------------------------------------------------------

class TestCappedRingBuffer(unittest.TestCase):
    """An entry the ring buffer drops fires no trigger (capped gateway)."""

    @classmethod
    def setUpClass(cls):
        _wait_for_entities(CAPPED_URL, ['apps/cap_probe'])
        cls.probe = RosoutProbe()
        # Class cleanups also run when setUpClass fails part-way.
        cls.addClassCleanup(cls.probe.close)
        cls.probe.wait_for_gateways(4)
        cls.probe.warm_up([(CAPPED_URL, 'apps/cap_probe')], 'capped.cap_probe')
        cls.stream = EventStream(CAPPED_URL, 'apps/cap_probe').start()
        cls.addClassCleanup(cls.stream.close)

    # @verifies REQ_INTEROP_097
    def test_dropped_entry_fires_no_trigger(self):
        """Fill the distinct-logger cap, then log under a new name the app owns."""
        # Cap is 3 x 10 = 30 distinct logger names; 40 new ones reach it.
        for i in range(40):
            self.probe.publish(f'junk-{i}', f'junk/n{i}')
            if i % 10 == 9:
                time.sleep(0.2)
        time.sleep(0.5)
        self.probe.publish('cap-drop', 'capped/cap_probe')
        self.probe.publish('cap-sentinel', 'capped.cap_probe')

        arrived = self.stream.wait_until(
            lambda: 'cap-sentinel' in self.stream.markers(), DELIVERY_TIMEOUT_SEC)
        listed = _listed(CAPPED_URL, 'apps/cap_probe')
        received = self.stream.markers()

        self.assertIn('cap-sentinel', listed, 'the sentinel went to an existing buffer')
        self.assertNotIn('cap-drop', listed,
                         'the cap did not engage, so this test measures nothing')
        self.assertNotIn('cap-drop', received,
                         'a trigger fired for an entry GET never lists')
        self.assertTrue(arrived, f'sentinel not delivered; received={sorted(received)}')


# --------------------------------------------------------------------------
# Owner rule across discovery modes, entity types and logger-name forms
# --------------------------------------------------------------------------

class TestLogTriggerScope(unittest.TestCase):
    """Each trigger delivers exactly the probes its entity's GET lists."""

    @classmethod
    def setUpClass(cls):
        _wait_for_entities(RUNTIME_URL, [
            p for p in RUNTIME_EXPECTED if HOST not in p])
        cls.host = _host_component(RUNTIME_URL)
        for url in (HYBRID_URL, MANIFEST_URL):
            _wait_for_entities(url, list(MANIFEST_EXPECTED))

        cls.probe = RosoutProbe()
        # Class cleanups also run when setUpClass fails part-way.
        cls.addClassCleanup(cls.probe.close)
        cls.probe.wait_for_gateways(4)
        cls.probe.warm_up([
            (RUNTIME_URL, 'apps/rpm_sensor'),
            (HYBRID_URL, 'apps/engine-rpm'),
            (MANIFEST_URL, 'apps/engine-rpm'),
        ], 'powertrain.engine.rpm_sensor')

        cls.expected = {}
        for url, table in GATEWAY_EXPECTED.items():
            for path, owned in table.items():
                cls.expected[(url, path.replace(HOST, cls.host))] = owned

        for url, path in cls.expected:
            _configure_logs(url, path)
        _configure_logs(RUNTIME_URL, 'apps/backup_temp_sensor', severity_filter='warning')

        cls.streams = {}
        for key in cls.expected:
            cls.streams[key] = EventStream(*key).start()
            cls.addClassCleanup(cls.streams[key].close)

        for case_id, name, level in PROBES:
            cls.probe.publish(case_id, name, level)
        for sentinel, twin in SENTINELS.items():
            name, level = _PROBE_BY_ID[twin]
            cls.probe.publish(sentinel, name, level)

        deadline = time.monotonic() + DELIVERY_TIMEOUT_SEC
        for key, stream in cls.streams.items():
            want = _expected_sentinels(cls.expected[key])
            stream.wait_until(lambda s=stream, w=want: w <= set(s.markers()),
                              max(0.0, deadline - time.monotonic()))

        cls.received = {key: stream.markers() for key, stream in cls.streams.items()}
        cls.listed = {key: _listed(*key) for key in cls.expected}

    def _problems(self, key):
        owned = self.expected[key]
        want = owned | _expected_sentinels(owned)
        tracked = PROBE_IDS | set(SENTINELS)
        received = {m: ids for m, ids in self.received[key].items() if m in tracked}
        listed = {m for m in self.listed[key] if m in tracked}

        problems = []
        if self.streams[key].error is not None:
            problems.append(f'stream failed: {self.streams[key].error!r}')
        if listed != want:
            problems.append(f'GET lists {sorted(listed)}, expected {sorted(want)}')
        extra = set(received) - want
        if extra:
            problems.append(f'delivered, not owned: {sorted(extra)}')
        missing = want - set(received)
        if missing:
            problems.append(f'owned, not delivered: {sorted(missing)}')
        repeated = {m: len(ids) for m, ids in received.items() if len(ids) != 1}
        if repeated:
            problems.append(f'delivered more than once: {repeated}')
        return problems

    # @verifies REQ_INTEROP_097
    def test_events_match_get_listing(self):
        """Every stream delivers exactly the probes its GET lists, once each."""
        for key in sorted(self.expected):
            with self.subTest(gateway=key[0], entity=key[1]):
                problems = self._problems(key)
                self.assertFalse(problems, f'{key[1]}: ' + '; '.join(problems))

    # @verifies REQ_INTEROP_097
    def test_namesake_app_gets_no_event(self):
        """An app whose id is the last segment of another logger gets no event."""
        cases = [
            ((RUNTIME_URL, 'apps/rpm_sensor'), 'tw1-child-slash', 's-rpm'),
            ((HYBRID_URL, 'apps/engine_temp'), 'root', 's-tw1'),
            ((MANIFEST_URL, 'apps/engine_temp'), 'root', 's-tw1'),
        ]
        for key, namesake, sentinel in cases:
            with self.subTest(gateway=key[0], entity=key[1], probe=namesake):
                received = self.received[key]
                self.assertNotIn(namesake, self.listed[key])
                self.assertNotIn(namesake, received,
                                 f'{key[1]} got an event for the logger of another node')
                self.assertIn(sentinel, received, 'stream not proven live')


# --------------------------------------------------------------------------
# Entries a trigger's log_settings writes are listed and fire nothing
# --------------------------------------------------------------------------

class TestLogSettingsEntries(unittest.TestCase):
    """log_settings entries are listed where GET covers them and fire no logs trigger."""

    RPM_TOPIC = '/powertrain/engine/rpm'
    ROOT_TOPIC = '/rpm'

    @classmethod
    def _data_ready(cls, app, topic):
        def ready():
            r = requests.get(f'{RUNTIME_URL}/apps/{app}/data', timeout=HTTP_TIMEOUT_SEC)
            return r.status_code == 200 and any(
                item.get('id') == topic for item in r.json().get('items', []))
        _poll(ready, what=f'{app} to expose {topic}')

    @classmethod
    def setUpClass(cls):
        _wait_for_entities(RUNTIME_URL, ['apps/rpm_sensor', 'apps/engine_temp'])
        cls.host = _host_component(RUNTIME_URL)
        cls._data_ready('rpm_sensor', cls.RPM_TOPIC)
        cls._data_ready('engine_temp', cls.ROOT_TOPIC)
        cls.probe = RosoutProbe()
        # Class cleanups also run when setUpClass fails part-way.
        cls.addClassCleanup(cls.probe.close)
        cls.probe.wait_for_gateways(4)
        cls.probe.warm_up([(RUNTIME_URL, 'apps/rpm_sensor')], 'powertrain.engine.rpm_sensor')

        cls.log_paths = ['apps/rpm_sensor', 'apps/engine_temp', f'components/{cls.host}']
        for path in cls.log_paths:
            _configure_logs(RUNTIME_URL, path)
        cls.logs = {p: EventStream(RUNTIME_URL, p).start() for p in cls.log_paths}
        for stream in cls.logs.values():
            cls.addClassCleanup(stream.close)

        def data_trigger(app, topic, marker):
            return EventStream(RUNTIME_URL, f'apps/{app}', body={
                'resource': f'{API_BASE_PATH}/apps/{app}/data{topic}',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 600,
                'log_settings': {'severity': 'warning', 'marker': f'{RUN}:{marker}'},
            }).start()

        cls.data = [
            data_trigger('rpm_sensor', cls.RPM_TOPIC, 'ls-rpm'),
            data_trigger('engine_temp', cls.ROOT_TOPIC, 'ls-root'),
        ]
        for stream in cls.data:
            cls.addClassCleanup(stream.close)

    # @verifies REQ_INTEROP_097
    def test_log_settings_entries_are_listed_and_fire_nothing(self):
        """Entry named engine_temp: listed by the root app and the host. rpm_sensor: by none."""
        for stream in self.data:
            self.assertTrue(stream.wait_until(lambda s=stream: s.event_count() >= 3,
                                              DELIVERY_TIMEOUT_SEC),
                            f'data trigger on {stream.path} did not fire')
        self.probe.publish('ls-s-rpm', 'powertrain.engine.rpm_sensor')
        self.probe.publish('ls-s-root', 'engine_temp')

        sentinels = {
            'apps/rpm_sensor': 'ls-s-rpm',
            'apps/engine_temp': 'ls-s-root',
            f'components/{self.host}': 'ls-s-root',
        }
        for path, sentinel in sentinels.items():
            stream = self.logs[path]
            arrived = stream.wait_until(
                lambda s=stream, m=sentinel: m in s.markers(), DELIVERY_TIMEOUT_SEC)
            received = stream.markers()
            listed = _listed(RUNTIME_URL, path)
            with self.subTest(entity=path):
                self.assertTrue(arrived, f'stream not proven live: {sorted(received)}')
                self.assertEqual(received.get('ls-root', []) + received.get('ls-rpm', []), [],
                                 f'{path}: a log_settings entry fired a logs trigger')
                if path != 'apps/rpm_sensor':
                    self.assertTrue(listed.get('ls-root'),
                                    f'{path}: GET does not list the ls-root entries')

        self.assertNotIn('ls-rpm', _listed(RUNTIME_URL, 'apps/rpm_sensor'))
        self.assertIn('ls-root', _listed(RUNTIME_URL, 'apps/engine_temp'),
                      'the root app lists its own log_settings entries')


# --------------------------------------------------------------------------
# severity_filter changes under a live trigger
# --------------------------------------------------------------------------

class TestSeverityFilterChange(unittest.TestCase):
    """The trigger follows the entity's severity_filter as it is changed."""

    PATH = 'apps/rpm_sensor'
    LOGGER = 'powertrain.engine.rpm_sensor'

    @classmethod
    def setUpClass(cls):
        _wait_for_entities(RUNTIME_URL, [cls.PATH])
        cls.probe = RosoutProbe()
        # Class cleanups also run when setUpClass fails part-way.
        cls.addClassCleanup(cls.probe.close)
        cls.probe.wait_for_gateways(4)
        cls.probe.warm_up([(RUNTIME_URL, cls.PATH)], cls.LOGGER)
        cls.stream = EventStream(RUNTIME_URL, cls.PATH).start()
        cls.addClassCleanup(cls.stream.close)

    @classmethod
    def tearDownClass(cls):
        _configure_logs(RUNTIME_URL, cls.PATH, severity_filter='debug')

    def _round(self, severity_filter, probes, sentinel):
        _configure_logs(RUNTIME_URL, self.PATH, severity_filter=severity_filter)
        for case_id, level in probes:
            self.probe.publish(case_id, self.LOGGER, level)
        self.probe.publish(sentinel, self.LOGGER, Log.FATAL)
        arrived = self.stream.wait_until(
            lambda: sentinel in self.stream.markers(), DELIVERY_TIMEOUT_SEC)
        self.assertTrue(arrived, f'{sentinel} not delivered')

    # @verifies REQ_INTEROP_097
    def test_filter_endpoints_and_change(self):
        """Fatal admits only fatal; back to debug admits debug again."""
        self._round('fatal', [('sf-error', Log.ERROR), ('sf-fatal', Log.FATAL)], 'sf-s1')
        self._round('debug', [('sf-debug', Log.DEBUG)], 'sf-s2')

        received = set(self.stream.markers())
        self.assertEqual(received & {'sf-error', 'sf-fatal', 'sf-debug', 'sf-s1', 'sf-s2'},
                         {'sf-fatal', 'sf-s1', 'sf-debug', 'sf-s2'})
        # GET applies the filter at read time, so it is checked under each one.
        _configure_logs(RUNTIME_URL, self.PATH, severity_filter='fatal')
        self.assertNotIn('sf-error', _listed(RUNTIME_URL, self.PATH))
        self.assertIn('sf-fatal', _listed(RUNTIME_URL, self.PATH))
        _configure_logs(RUNTIME_URL, self.PATH, severity_filter='debug')
        self.assertIn('sf-debug', _listed(RUNTIME_URL, self.PATH))

    # @verifies REQ_INTEROP_097
    def test_levels_without_a_severity_name(self):
        """A level between the named ones fires exactly when GET lists it."""
        rounds = [
            ('info', [('lv-info-25', 25), ('lv-info-19', 19)], 'lv-s1'),
            ('debug', [('lv-debug-0', 0), ('lv-debug-9', 9)], 'lv-s2'),
            ('fatal', [('lv-fatal-99', 99), ('lv-fatal-49', 49)], 'lv-s3'),
        ]
        expected_listed = {'lv-info-25', 'lv-fatal-99'}
        listed = set()
        for severity_filter, probes, sentinel in rounds:
            self._round(severity_filter, probes, sentinel)
            round_ids = {case_id for case_id, _ in probes}
            listed |= set(_listed(RUNTIME_URL, self.PATH)) & round_ids

        probe_ids = {case_id for _, probes, _ in rounds for case_id, _ in probes}
        received = set(self.stream.markers()) & probe_ids
        self.assertEqual(listed, expected_listed, 'GET under each filter')
        self.assertEqual(received, listed, 'delivered != listed')


# --------------------------------------------------------------------------
# Events no client has read
# --------------------------------------------------------------------------

class TestUnreadEventCap(unittest.TestCase):
    """A trigger keeps the newest 100 events no client has read (capped gateway)."""

    PATH = 'apps/cap_probe'
    # Its node logs only at startup, so every queued event is one of ours.
    LOGGER = 'capped.cap_probe'
    COUNT = 150
    CAP = 100

    @classmethod
    def setUpClass(cls):
        _wait_for_entities(CAPPED_URL, [cls.PATH])
        cls.probe = RosoutProbe()
        # Class cleanups also run when setUpClass fails part-way.
        cls.addClassCleanup(cls.probe.close)
        cls.probe.wait_for_gateways(4)
        cls.probe.warm_up([(CAPPED_URL, cls.PATH)], cls.LOGGER)

    # @verifies REQ_INTEROP_097
    def test_newest_events_survive_past_the_cap(self):
        """150 unread events leave the newest 100, oldest dropped, in order."""
        unread = EventStream(CAPPED_URL, self.PATH).create()
        # Every change reaches both triggers in the same dispatch, so once this
        # stream has the last entry the unread trigger has judged all of them.
        barrier = EventStream(CAPPED_URL, self.PATH).start()
        try:
            names = [f'uc-{i:03d}' for i in range(self.COUNT)]
            for i, name in enumerate(names):
                self.probe.publish(name, self.LOGGER)
                if i % 50 == 49:
                    time.sleep(0.2)
            self.assertTrue(barrier.wait_until(lambda: names[-1] in barrier.markers(),
                                               DELIVERY_TIMEOUT_SEC),
                            f'barrier stream got {len(barrier.markers())} markers')
            self.assertEqual([m for m in barrier.ordered_markers() if m.startswith('uc-')], names,
                             'every entry reached the gateway once, in order')

            unread.connect()
            self.probe.publish('uc-sentinel', self.LOGGER)
            self.assertTrue(unread.wait_until(lambda: 'uc-sentinel' in unread.markers(),
                                              DELIVERY_TIMEOUT_SEC))
            got = unread.ordered_markers()
            self.assertEqual(len(got), self.CAP + 1, f'{len(got)} events')
            self.assertEqual(got, names[-self.CAP:] + ['uc-sentinel'])
        finally:
            barrier.close()
            unread.close()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for path in _MANIFEST_PATHS:
            try:
                os.unlink(path)
            except OSError:
                pass
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
