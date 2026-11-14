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

"""Entries that a trigger's log_settings writes never fire a logs trigger.

A trigger with log_settings writes a marker entry each time it fires. The
marker is logged under the trigger's entity id, so GET /{entity}/logs lists it
where that logger name is in the entity's log. It fires no logs trigger: not
its own, not another one on the same entity, not one above or below it.

Each case creates its triggers, publishes five real /rosout entries under a
logger its entities list, each listed before the next is logged, then one
closing entry. A trigger delivers in log order, so once the closing entry
arrives no earlier marker can still fire it. The case then compares per
trigger the events it delivered with what its entity's GET lists: one event per
real entry, none for a marker, and one marker per event wherever the entity
lists the trigger's markers.

Two gateways share one ROS graph: runtime (runtime_only) and manifest
(manifest_only), with a fault manager for the faults trigger case. The graph
nodes log only at startup.
"""

import json
import os
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
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.coverage import get_coverage_env
from ros2_medkit_test_utils.launch_helpers import create_fault_manager_node, create_gateway_node

RUNTIME_PORT = get_test_port(0)
MANIFEST_PORT = get_test_port(1)
RUNTIME_URL = f'http://localhost:{RUNTIME_PORT}{API_BASE_PATH}'
MANIFEST_URL = f'http://localhost:{MANIFEST_PORT}{API_BASE_PATH}'

TIME_SCALE = get_time_scale()
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE
DISCOVERY_TIMEOUT_SEC = 60.0 * TIME_SCALE
SETTLE_TIMEOUT_SEC = 15.0 * TIME_SCALE
DELIVERY_TIMEOUT_SEC = 20.0 * TIME_SCALE
POLL_INTERVAL_SEC = 0.5

REAL_ENTRIES = 5

RUN = uuid.uuid4().hex[:10]
HOST = '{host}'

# The look-alike's stamp tells it apart from markers in the test only.
LOOKALIKE_STAMP_SEC = 1000000000
LOOKALIKE_TIMESTAMP = '2001-09-09T01:46:40.'

# (executable, node name, namespace)
GRAPH_NODES = [
    ('demo_calibration_service', 'loop_root', '/'),
    ('demo_calibration_service', 'loop_ns', '/loopns'),
    ('demo_calibration_service', 'zapp', '/zone'),
    ('demo_calibration_service', 'papp', '/plant'),
]

MANIFEST_YAML = """\
manifest_version: "1.0"
metadata:
  name: "log trigger markers"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: zone
    name: "Zone"
  - id: plant-area
    name: "Plant area"
    namespace: /plant
components:
  - id: ecu
    name: "ECU"
    area: zone
  - id: plant-ecu
    name: "Plant ECU"
    area: plant-area
    namespace: /plant
apps:
  - id: zapp
    name: "Zone app"
    is_located_on: ecu
    ros_binding:
      node_name: zapp
      namespace: /zone
  - id: papp
    name: "Plant app"
    is_located_on: plant-ecu
    ros_binding:
      node_name: papp
      namespace: /plant
"""

RUNTIME_PATHS = ['apps/loop_root', 'apps/loop_ns', 'functions/loopns']
MANIFEST_PATHS = [
    'areas/zone', 'areas/plant-area', 'components/ecu', 'components/plant-ecu', 'apps/zapp',
]

_MANIFEST_FILES = []


def _write_manifest():
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_triggers_logs_markers_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(MANIFEST_YAML)
    _MANIFEST_FILES.append(path)
    return path


def generate_test_description():
    streams = {'sse.max_clients': 8, 'server.http_thread_pool_size': 16}
    gateways = [
        create_gateway_node(port=RUNTIME_PORT, name='gw_markers_runtime',
                            extra_params=dict(streams, **{'discovery.mode': 'runtime_only'})),
        create_gateway_node(port=MANIFEST_PORT, name='gw_markers_manifest',
                            extra_params=dict(streams, **{
                                'discovery.mode': 'manifest_only',
                                'discovery.manifest_path': _write_manifest(),
                                'discovery.manifest_strict_validation': False,
                            })),
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
    fault_manager = create_fault_manager_node(rosbag_enabled=False)
    return (
        LaunchDescription(
            gateways + nodes + [fault_manager, launch_testing.actions.ReadyToTest()]),
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
        time.sleep(0.3)
    raise AssertionError(f'timed out after {timeout}s waiting for {what}; last={last!r}')


def _status(url):
    return requests.get(url, timeout=HTTP_TIMEOUT_SEC).status_code


def _listing(base_url, path):
    r = requests.get(f'{base_url}/{path}/logs', timeout=HTTP_TIMEOUT_SEC)
    r.raise_for_status()
    return r.json().get('items', [])


def _count(base_url, path, message):
    """Return how many entries GET /{path}/logs lists with exactly this message."""
    return sum(1 for i in _listing(base_url, path) if i.get('message') == message)


def _num(entry_id):
    return int(str(entry_id).removeprefix('log_'))


def _configure_logs(base_url, path):
    r = requests.put(f'{base_url}/{path}/logs/configuration',
                     json={'max_entries': 10000}, timeout=HTTP_TIMEOUT_SEC)
    assert r.status_code == 204, f'PUT {base_url}/{path}/logs/configuration: {r.text}'


def _marker_text(label):
    return f'{RUN}:mark-{label}'


def _is_marker_of(message, label):
    return str(message).startswith(_marker_text(label) + ' ')


def _describe(entry):
    return (entry.get('id'), entry.get('severity'), entry.get('context', {}).get('node'),
            str(entry.get('message', ''))[:80])


def _is_lookalike(entry, lookalike):
    return (entry.get('message') == lookalike[0]
            and str(entry.get('timestamp', '')).startswith(LOOKALIKE_TIMESTAMP))


class TriggerStream:
    """A trigger, by default a logs trigger with log_settings, and its event reader."""

    def __init__(self, base_url, path, label, *, collection='logs', marker=True):
        self.base_url = base_url
        self.path = path
        self.label = label
        self.collection = collection
        self.marker = marker
        self.trigger_id = None
        self.payloads = []
        self.error = None
        self._lock = threading.Lock()
        self._resp = None
        self._closed = False

    def start(self):
        body = {
            'resource': f'{API_BASE_PATH}/{self.path}/{self.collection}',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 600,
        }
        if self.marker:
            body['log_settings'] = {'severity': 'info', 'marker': _marker_text(self.label)}
        r = requests.post(f'{self.base_url}/{self.path}/triggers', json=body,
                          timeout=HTTP_TIMEOUT_SEC)
        assert r.status_code == 201, f'POST {self.base_url}/{self.path}/triggers: {r.text}'
        trigger = r.json()
        self.trigger_id = trigger['id']
        events_url = self.base_url + trigger['event_source'].removeprefix(API_BASE_PATH)
        self._resp = requests.get(events_url, stream=True,
                                  timeout=(HTTP_TIMEOUT_SEC, 120.0 * TIME_SCALE))
        assert self._resp.status_code == 200, f'GET {events_url}: {self._resp.status_code}'
        threading.Thread(target=self._read, daemon=True).start()
        return self

    def _read(self):
        try:
            for line in self._resp.iter_lines(chunk_size=None, decode_unicode=True):
                if line and line.startswith('data: '):
                    payload = json.loads(line[len('data: '):]).get('payload')
                    with self._lock:
                        self.payloads.append(payload if isinstance(payload, dict) else {})
        except Exception as exc:
            if not self._closed:
                self.error = exc

    def snapshot(self):
        with self._lock:
            return list(self.payloads)

    def marker_message(self):
        """Return the message of this trigger's marker entries."""
        metadata = {
            'condition_type': 'OnChange',
            'resource': f'{API_BASE_PATH}/{self.path}/{self.collection}',
            'trigger_id': self.trigger_id,
        }
        return _marker_text(self.label) + ' ' + json.dumps(
            metadata, sort_keys=True, separators=(',', ':'))

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
        self.node = rclpy.create_node('marker_probe')
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.node.create_publisher(Log, '/rosout', qos)

    def publish(self, logger_name, message, *, source=True, stamp_sec=None):
        msg = Log()
        msg.stamp = self.node.get_clock().now().to_msg()
        if stamp_sec is not None:
            msg.stamp.sec = stamp_sec
            msg.stamp.nanosec = 0
        msg.level = Log.INFO
        msg.name = logger_name
        msg.msg = message
        if source:
            msg.file = 'test_triggers_logs_markers.test.py'
            msg.function = 'publish'
            msg.line = 1
        self.pub.publish(msg)

    def report_fault(self, code, source_id):
        """Report a CRITICAL fault, which the fault manager confirms at once."""
        client = self.node.create_client(ReportFault, '/fault_manager/report_fault')
        try:
            assert client.wait_for_service(timeout_sec=DISCOVERY_TIMEOUT_SEC), \
                'report_fault service not available'
            req = ReportFault.Request()
            req.fault_code = code
            req.event_type = ReportFault.Request.EVENT_FAILED
            req.severity = Fault.SEVERITY_CRITICAL
            req.description = 'marker test fault'
            req.source_id = source_id
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=HTTP_TIMEOUT_SEC)
            return future.result() is not None and future.result().accepted
        finally:
            self.node.destroy_client(client)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------------------------
# Cases
# --------------------------------------------------------------------------

class TestLogSettingsMarkers(unittest.TestCase):
    """A marker is listed where its logger name is in scope and fires nothing."""

    @classmethod
    def setUpClass(cls):
        for path in RUNTIME_PATHS:
            _poll(lambda p=path: _status(f'{RUNTIME_URL}/{p}') == 200, what=f'runtime {path}')
        for path in MANIFEST_PATHS:
            _poll(lambda p=path: _status(f'{MANIFEST_URL}/{p}') == 200, what=f'manifest {path}')

        def one_component():
            items = requests.get(f'{RUNTIME_URL}/components',
                                 timeout=HTTP_TIMEOUT_SEC).json().get('items', [])
            return items[0]['id'] if len(items) == 1 else None
        cls.host = _poll(one_component, what='the runtime host component')

        cls.probe = RosoutProbe()
        cls.addClassCleanup(cls.probe.close)
        _poll(lambda: cls.probe.pub.get_subscription_count() >= 2, what='2 /rosout subscribers')
        for url, path, logger in ((RUNTIME_URL, 'apps/loop_root', 'loop_root'),
                                  (MANIFEST_URL, 'areas/zone', 'zone/zapp')):
            token = f'{RUN}:warm-{uuid.uuid4().hex[:6]}'
            count = [0]

            def listed(u=url, p=path, lg=logger, t=token):
                count[0] += 1
                cls.probe.publish(lg, f'{t}-{count[0]}')
                time.sleep(0.2)
                return any(str(i.get('message', '')).startswith(t) for i in _listing(u, p))
            _poll(listed, what=f'{url}/{path} to list a warm-up entry')

    def _path(self, path):
        return path.replace(HOST, self.host)

    def _measure(self, url, streams, lists, watermarks, case, kicks, lookalike):
        """Return the problems one snapshot of the listings and the events shows."""
        labels = [s.label for s in streams]
        events = {s.label: s.snapshot() for s in streams}
        listings = {p: _listing(url, p) for p in watermarks}

        def marker_like(entry):
            return any(_is_marker_of(entry.get('message', ''), lb) for lb in labels)

        problems = []
        for path, items in listings.items():
            mark = watermarks[path]
            fresh = [i for i in items if _num(i['id']) > mark]
            mimic = {i['id'] for i in fresh if lookalike and _is_lookalike(i, lookalike)}
            real = {i['id'] for i in fresh if not marker_like(i)}
            fresh_markers = len(fresh) - len(real) - len(mimic)
            if lookalike and lookalike[1] == path and len(mimic) != 1:
                problems.append(f'{path}: GET lists the look-alike {len(mimic)} times')
            # One marker per firing of each trigger whose markers this entity
            # lists; a firing before the watermark left its marker below it.
            want_markers = 0
            for label in lists.get(path, set()):
                older = sum(1 for i in items if _num(i['id']) <= mark
                            and _is_marker_of(i.get('message', ''), label))
                want_markers += len(events[label]) - older
            published = sum(1 for i in items
                            if str(i.get('message', '')).startswith(f'{RUN}:real-{case}-'))
            if published != REAL_ENTRIES * len(kicks):
                problems.append(f'{path}: GET lists {published} of the '
                                f'{REAL_ENTRIES * len(kicks)} real entries published')
            if fresh_markers != want_markers:
                problems.append(f'{path}: {len(fresh)} new entries listed = {len(real)} real + '
                                f'{len(mimic)} look-alike + {fresh_markers} markers; expected '
                                f'{want_markers} markers, one per firing')

        for stream in streams:
            path = stream.path
            payloads = events[stream.label]
            tag = f'{stream.label} on {path}'
            items = listings[path]
            fresh = [i for i in items if _num(i['id']) > watermarks[path]]
            real = {i['id'] for i in fresh if not marker_like(i)}
            is_mimic = bool(lookalike) and lookalike[2] == stream.label
            mimic = {i['id'] for i in fresh if is_mimic and _is_lookalike(i, lookalike)}
            delivered = [p.get('id') for p in payloads
                         if p.get('id') and _num(p['id']) > watermarks[path]]
            extra = set(delivered) - real
            missing = real - set(delivered)
            if len(delivered) != len(set(delivered)):
                problems.append(f'{tag}: {len(delivered) - len(set(delivered))} '
                                f'entries delivered twice')
            if missing:
                shown = [_describe(i) for i in fresh if i['id'] in missing][:3]
                problems.append(f'{tag}: {len(missing)} real entries its GET lists '
                                f'were not delivered, e.g. {shown}')
            want_extra = 1 if is_mimic else 0
            if len(extra) != want_extra or not extra <= mimic:
                problems.append(f'{tag}: {len(extra)} deliveries of entries that are not '
                                f'real entries its GET lists (expected {want_extra}: the '
                                f'look-alike)')
            marker_events = sum(
                1 for p in payloads
                if marker_like(p) and not (lookalike and _is_lookalike(p, lookalike)))
            if marker_events:
                problems.append(f'{tag}: {marker_events} events for markers')
            if stream.label not in lists.get(path, set()):
                own = sum(1 for i in items if _is_marker_of(i.get('message', ''), stream.label))
                if own:
                    problems.append(f'{tag}: GET lists {own} of its markers, expected none')
            if stream.error is not None:
                problems.append(f'{tag}: stream error {stream.error!r}')
        return problems

    def _run(self, url, triggers, kicks, lists, case, lookalike_of=None):
        """Create the triggers, log for at least 5 s, then compare events and GET."""
        triggers = [(label, self._path(path)) for label, path in triggers]
        lists = {self._path(p): set(v) for p, v in lists.items()}
        paths = sorted({p for _, p in triggers})
        for path in paths:
            _configure_logs(url, path)
        streams = []
        try:
            for label, path in triggers:
                streams.append(TriggerStream(url, path, label).start())
            watermarks = {p: max((_num(i['id']) for i in _listing(url, p)), default=0)
                          for p in paths}
            lookalike = None
            if lookalike_of:
                stream = next(s for s in streams if s.label == lookalike_of[0])
                lookalike = (stream.marker_message(), stream.path, stream.label, lookalike_of[1])

            for n in range(REAL_ENTRIES):
                message = f'{RUN}:real-{case}-{n}'
                for kick in kicks:
                    self.probe.publish(kick, message)
                if lookalike and n == REAL_ENTRIES // 2:
                    # Same logger, level, message and empty source as a marker.
                    self.probe.publish(lookalike[3], lookalike[0], source=False,
                                       stamp_sec=LOOKALIKE_STAMP_SEC)
                _poll(lambda m=message: all(_count(url, p, m) == len(kicks) for p in paths),
                      what=f'every listing to hold {message}')

            closing = f'{RUN}:done-{case}'
            for kick in kicks:
                self.probe.publish(kick, closing)
            deadline = time.monotonic() + SETTLE_TIMEOUT_SEC
            clean_polls = 0
            problems = []
            while time.monotonic() < deadline and clean_polls < 2:
                time.sleep(POLL_INTERVAL_SEC)
                if all(_count(url, p, closing) == len(kicks) for p in paths):
                    problems = self._measure(url, streams, lists, watermarks, case, kicks,
                                             lookalike)
                else:
                    problems = [f'{closing} is not listed everywhere yet']
                clean_polls = clean_polls + 1 if not problems else 0
            same = []
            if lookalike:
                same = [i for i in _listing(url, lookalike[1]) if i.get('message') == lookalike[0]]
        finally:
            for stream in streams:
                stream.close()
        self.assertFalse(problems, '; '.join(problems))
        if lookalike:
            shapes = {json.dumps([i.get('severity'), i.get('context')], sort_keys=True)
                      for i in same}
            self.assertTrue(len(same) >= 2 and len(shapes) == 1,
                            f'the look-alike must share message, severity and context with a '
                            f'listed marker, or it proves nothing: {same[:3]}')

    # L1: one trigger per entity kind.

    # @verifies REQ_INTEROP_097
    def test_l1_root_namespace_app(self):
        self._run(RUNTIME_URL, [('root-app', 'apps/loop_root')], ['loop_root'],
                  {'apps/loop_root': {'root-app'}}, 'root-app')

    # @verifies REQ_INTEROP_097
    def test_l1_namespaced_app(self):
        self._run(RUNTIME_URL, [('ns-app', 'apps/loop_ns')], ['loopns.loop_ns'],
                  {}, 'ns-app')

    # @verifies REQ_INTEROP_097
    def test_l1_host_component(self):
        self._run(RUNTIME_URL, [('host', f'components/{HOST}')], ['loopns/loop_ns'],
                  {}, 'host')

    # @verifies REQ_INTEROP_097
    def test_l1_runtime_function(self):
        self._run(RUNTIME_URL, [('function', 'functions/loopns')], ['loopns.loop_ns'],
                  {}, 'function')

    # @verifies REQ_INTEROP_097
    def test_l1_area_with_default_namespace(self):
        self._run(MANIFEST_URL, [('area', 'areas/zone')], ['zone/zapp'],
                  {'areas/zone': {'area'}}, 'area')

    # @verifies REQ_INTEROP_097
    def test_l1_area_with_namespace(self):
        self._run(MANIFEST_URL, [('ns-area', 'areas/plant-area')], ['plant.papp'],
                  {}, 'ns-area')

    # @verifies REQ_INTEROP_097
    def test_l1_component_with_default_fqn(self):
        self._run(MANIFEST_URL, [('component', 'components/ecu')], ['zone.zapp'],
                  {'components/ecu': {'component'}}, 'component')

    # @verifies REQ_INTEROP_097
    def test_l1_component_with_namespace(self):
        self._run(MANIFEST_URL, [('ns-component', 'components/plant-ecu')], ['plant/papp'],
                  {}, 'ns-component')

    # L2: triggers that list each other's markers.

    # @verifies REQ_INTEROP_097
    def test_l2_two_triggers_on_one_app(self):
        self._run(RUNTIME_URL, [('twin-a', 'apps/loop_root'), ('twin-b', 'apps/loop_root')],
                  ['loop_root'], {'apps/loop_root': {'twin-a', 'twin-b'}}, 'twins')

    # @verifies REQ_INTEROP_097
    def test_l2_app_and_the_host_component_above_it(self):
        self._run(RUNTIME_URL,
                  [('pair-app', 'apps/loop_root'), ('pair-host', f'components/{HOST}')],
                  ['loop_root'],
                  {'apps/loop_root': {'pair-app'}, f'components/{HOST}': {'pair-app'}},
                  'app-host')

    # @verifies REQ_INTEROP_097
    def test_l2_component_and_the_app_on_it(self):
        self._run(MANIFEST_URL,
                  [('pair-ecu', 'components/ecu'), ('pair-zapp', 'apps/zapp')],
                  ['zone/zapp'], {'components/ecu': {'pair-ecu'}}, 'ecu-app')

    # A marker written by a faults trigger.

    # @verifies REQ_INTEROP_097
    def test_marker_of_a_faults_trigger_fires_no_logs_trigger(self):
        """The root app lists the faults trigger's marker; its logs trigger skips it."""
        path = 'apps/loop_root'
        _configure_logs(RUNTIME_URL, path)
        logs = TriggerStream(RUNTIME_URL, path, 'fault-logs', marker=False)
        faults = TriggerStream(RUNTIME_URL, path, 'fault', collection='faults')
        try:
            logs.start()
            faults.start()
            self.assertTrue(self.probe.report_fault(f'MARKER_FAULT_{RUN.upper()}', '/loop_root'),
                            'the fault manager refused the report')

            def markers():
                return [i for i in _listing(RUNTIME_URL, path)
                        if _is_marker_of(i.get('message', ''), 'fault')]
            _poll(lambda: markers() and faults.snapshot(),
                  what='the faults trigger to fire and GET to list its marker',
                  timeout=DELIVERY_TIMEOUT_SEC)
            # Queued after the markers, so delivered after the logs trigger judged them.
            sentinel = f'{RUN}:fault-sentinel'
            self.probe.publish('loop_root', sentinel)
            _poll(lambda: any(p.get('message') == sentinel for p in logs.snapshot()),
                  what='the logs trigger to deliver the sentinel', timeout=DELIVERY_TIMEOUT_SEC)

            def settled():
                first = (len(markers()), len(faults.snapshot()))
                time.sleep(POLL_INTERVAL_SEC)
                return first == (len(markers()), len(faults.snapshot())) and first
            listed, firings = _poll(settled, what='the fault events to settle',
                                    timeout=SETTLE_TIMEOUT_SEC)
            fired = [_describe(p) for p in logs.snapshot()
                     if _is_marker_of(p.get('message', ''), 'fault')]
        finally:
            logs.close()
            faults.close()
        self.assertEqual(fired, [], 'a faults trigger marker fired the logs trigger')
        self.assertEqual(listed, firings, 'GET lists one marker per firing of the faults trigger')

    # L4: a real entry with a marker's content still fires.

    # @verifies REQ_INTEROP_097
    def test_l4_real_entry_that_looks_like_a_marker_fires(self):
        self._run(RUNTIME_URL, [('mimic', 'apps/loop_root')], ['loop_root'],
                  {'apps/loop_root': {'mimic'}}, 'mimic', lookalike_of=('mimic', 'loop_root'))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
        for path in _MANIFEST_FILES:
            if os.path.exists(path):
                os.unlink(path)
