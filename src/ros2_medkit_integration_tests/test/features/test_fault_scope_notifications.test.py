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

"""Fault notifications reach exactly the entities whose source scope holds the record.

A fault record belongs to an entity when one of its reporting sources equals
one of the entity's owned sources or is a path child of one.
``GET /{entity}/faults`` uses that rule. This file checks that the two
notification paths use it too, on the record as the event carries it and
whatever its status (what ``GET /{entity}/faults?status=all`` lists after the
event): a trigger on ``/{entity}/faults`` gets the event iff the entity's scope
holds the record, and ``x-medkit`` in ``GET /faults/stream`` names the app whose
scope holds it, never a namesake.

One fault manager feeds three gateways, one per discovery mode, so every report
is judged in hybrid, runtime_only and manifest_only at once. The ROS nodes are
created by this process:

- ``/scope/x/detector`` and ``/scope/y/detector`` share a name, so runtime
  discovery calls them ``scope_x_detector`` and ``scope_y_detector``.
- ``/scope/z/goal_status`` is an app named ``goal_status``: the last segment of
  every sub-path source below. A notification keyed on that segment lands on it.
- ``/scope/w/solo`` has a unique name, so its runtime id is ``solo``.
- ``/scope/n/arm`` and ``/scope/n/arm/wrist`` nest, so a source below the
  wrist node is owned by both apps. The inner one has the larger id in every
  mode, so "most specific owner" and "smallest id" name different apps.

The manifest (hybrid and manifest_only) gives the detector an id that is not
its node name, puts it in a subarea, hosts it in a Function through its
Component, declares an external app that reports under its bare id, and binds
two apps to one node path (spelled two ways) that has no node.

A negative check is only as good as the stream it reads, so every trigger
stream ends with a probe fault that must reach it. Events on one gateway are
delivered in order, so a probe that arrived means every earlier event has
been judged.
"""

import json
import os
import tempfile
import threading
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, ReportFault

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    FAULT_TIMEOUT,
    get_test_port,
    ROSBAG_TIMEOUT,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_fault_manager_node,
    create_gateway_node,
)
from ros2_medkit_test_utils.service_round_trip import answer_once_each

from std_msgs.msg import Float32


HYBRID = 'hybrid'
RUNTIME = 'runtime_only'
MANIFEST = 'manifest_only'
PORTS = {HYBRID: get_test_port(0), RUNTIME: get_test_port(1), MANIFEST: get_test_port(2)}

DETECTOR_X = '/scope/x/detector'
DETECTOR_Y = '/scope/y/detector'
GOAL_STATUS = '/scope/z/goal_status'
SOLO = '/scope/w/solo'
EXTERNAL_APP = 'plc-app'
SIGNAL_TOPIC = '/scope/signal'

MANIFEST_YAML = """\
manifest_version: "1.0"
metadata:
  name: fault-scope-notifications
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: plant
    name: Plant
    namespace: /scope
    subareas:
      - id: cell
        name: Cell
        namespace: /scope/x
  - id: aux
    name: Aux
    namespace: /aux
  - id: yard
    name: Yard
    namespace: /yard
  - id: nest
    name: Nest
    namespace: /nest
components:
  - id: detector-unit
    name: Detector unit
    area: cell
  - id: aux-unit
    name: Aux unit
    area: aux
  - id: plc-unit
    name: PLC unit
    area: yard
  - id: nest-unit
    name: Nest unit
    area: nest
apps:
  - id: detector-app
    name: Detector
    is_located_on: detector-unit
    ros_binding:
      node_name: detector
      namespace: /scope/x
  - id: goal_status
    name: Goal status
    is_located_on: aux-unit
    ros_binding:
      node_name: goal_status
      namespace: /scope/z
  - id: plc-app
    name: PLC app
    external: true
    is_located_on: plc-unit
  - id: arm-app
    name: Arm
    is_located_on: nest-unit
    ros_binding:
      node_name: arm
      namespace: /scope/n
  - id: wrist-app
    name: Wrist
    is_located_on: nest-unit
    ros_binding:
      node_name: wrist
      namespace: /scope/n/arm
  - id: twin-a
    name: Twin A
    is_located_on: nest-unit
    ros_binding:
      node_name: twin
      namespace: /scope/t
  # The same node path spelled with a trailing slash: an identical binding
  # would be rejected as a duplicate (R010).
  - id: twin-b
    name: Twin B
    is_located_on: nest-unit
    ros_binding:
      node_name: twin
      namespace: /scope/t/
functions:
  - id: detection
    name: Detection
    hosted_by:
      - detector-unit
  - id: site-watch
    name: Site watch
    hosted_by:
      - detector-app
      - goal_status
"""


def _write_manifest():
    fd, path = tempfile.mkstemp(prefix='medkit_fault_scope_', suffix='.yaml')
    os.write(fd, MANIFEST_YAML.encode('utf-8'))
    os.close(fd)
    return path


MANIFEST_PATH = _write_manifest()

# Every stream a case opens stays open until its trigger is deleted; the
# default two SSE clients and six HTTP workers would turn later streams
# into 503s, which read as "no event".
STREAM_CAPACITY = {'sse.max_clients': 40, 'server.http_thread_pool_size': 48}


def generate_test_description():
    manifest_params = {
        'discovery.manifest_path': MANIFEST_PATH,
        'discovery.manifest_strict_validation': False,
    }
    gateways = [
        create_gateway_node(
            port=PORTS[HYBRID], name='gateway_hybrid',
            extra_params={**STREAM_CAPACITY, **manifest_params, 'discovery.mode': 'hybrid'}),
        create_gateway_node(
            port=PORTS[RUNTIME], name='gateway_runtime',
            extra_params={**STREAM_CAPACITY, 'discovery.mode': 'runtime_only'}),
        create_gateway_node(
            port=PORTS[MANIFEST], name='gateway_manifest',
            extra_params={**STREAM_CAPACITY, **manifest_params,
                          'discovery.mode': 'manifest_only'}),
    ]
    fault_manager = create_fault_manager_node(
        rosbag_topics=[SIGNAL_TOPIC],
        snapshot_topics=[SIGNAL_TOPIC],
        # One FAILED report confirms; a later report from another source is
        # a fault_updated event on the confirmed fault. Two PASSED reports heal.
        extra_params={'confirmation_threshold': -1, 'healing_enabled': True,
                      'healing_threshold': 1},
    )
    return (
        LaunchDescription([
            *gateways,
            TimerAction(period=2.0, actions=[fault_manager]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateways[0]},
    )


def _url(mode, path):
    return f'http://localhost:{PORTS[mode]}{API_BASE_PATH}{path}'


class SseReader:
    """Collects the JSON data frames of one SSE stream on a background thread.

    The thread owns the connection. A trigger stream sends its headers with
    the first event or keepalive, up to 15 s later, and the trigger buffers
    events until then, so the constructor does not wait for it. Closing a
    streamed response from another thread blocks until the next frame
    arrives, so close() only asks the thread to stop at its next frame; a
    deleted trigger ends its stream at once.
    """

    def __init__(self, url, label):
        self.label = label
        self.payloads = []
        self.error = None
        self._url = url
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._thread = threading.Thread(target=self._pump, daemon=True)
        self._thread.start()

    def _pump(self):
        try:
            response = requests.get(self._url, stream=True, timeout=(5, 300))
        except requests.exceptions.RequestException as e:
            self.error = f'{self._url}: {e}'
            return
        try:
            if response.status_code != 200:
                self.error = f'{self._url} answered {response.status_code}: {response.text}'
                return
            data = None
            for line in response.iter_lines(decode_unicode=True):
                if self._closed.is_set():
                    return
                if line is None:
                    continue
                if line == '':
                    if data is not None:
                        with self._lock:
                            self.payloads.append(json.loads(data))
                    data = None
                    continue
                if line.startswith('data:'):
                    data = line[len('data:'):].strip()
        except Exception:  # noqa: BLE001 - the stream ends when its trigger goes
            pass
        finally:
            response.close()

    def snapshot(self):
        with self._lock:
            return list(self.payloads)

    def close(self):
        self._closed.set()


def _trigger_events(reader):
    """(fault_code, event_type) of every fault payload a trigger stream delivered."""
    events = []
    for envelope in reader.snapshot():
        fault = envelope.get('payload', {})
        events.append((fault.get('fault_code'), fault.get('event_type')))
    return events


def _trigger_details(reader):
    """(fault_code, event_type, status, source count) of every delivered fault payload."""
    return [(f.get('fault_code'), f.get('event_type'), f.get('status'),
             len(f.get('reporting_sources', [])))
            for f in (envelope.get('payload', {}) for envelope in reader.snapshot())]


def _stream_events(reader):
    """(fault_code, event_type, source count) -> x-medkit of every /faults/stream event."""
    return {(p['fault']['fault_code'], p['event_type'], len(p['fault']['reporting_sources'])):
            p.get('x-medkit') for p in reader.snapshot() if 'fault' in p}


class TestFaultScopeNotifications(GatewayTestCase):
    """Triggers and the fault stream follow the fault-listing scope rule."""

    BASE_URL = f'http://localhost:{PORTS[HYBRID]}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 3
    REQUIRED_APPS = {'detector-app', 'goal_status', EXTERNAL_APP}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._nodes = [
            Node('detector', namespace='/scope/x'),
            Node('detector', namespace='/scope/y'),
            Node('goal_status', namespace='/scope/z'),
            Node('solo', namespace='/scope/w'),
            Node('arm', namespace='/scope/n'),
            Node('wrist', namespace='/scope/n/arm'),
        ]
        cls._reporter = Node('fault_scope_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault')
        cls._clear_client = cls._reporter.create_client(
            ClearFault, '/fault_manager/clear_fault')
        # The rosbag ring buffer needs data on the captured topic before a
        # fault confirms, and the fault manager resolves an explicit topic's
        # type only in its first seconds, so the publisher starts first.
        signal = cls._reporter.create_publisher(Float32, SIGNAL_TOPIC, 10)
        cls._reporter.create_timer(0.05, lambda: signal.publish(Float32(data=1.0)))
        cls._executor = SingleThreadedExecutor()
        for node in [*cls._nodes, cls._reporter]:
            cls._executor.add_node(node)
        cls._spin_thread = threading.Thread(target=cls._executor.spin, daemon=True)
        cls._spin_thread.start()

        super().setUpClass()
        assert cls._report_client.wait_for_service(timeout_sec=30.0), \
            'report_fault service not available'
        answer_once_each(cls._reporter, (cls._report_client, cls._clear_client), 30.0, spin=False)
        cls._wait_for_entities()
        cls._host_component = cls._runtime_host_component()
        cls._report_counter = 0
        cls._prime_fault_streams()

    @classmethod
    def tearDownClass(cls):
        cls._executor.shutdown()
        cls._spin_thread.join(timeout=5)
        for node in [*cls._nodes, cls._reporter]:
            node.destroy_node()
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Fixture helpers
    # ------------------------------------------------------------------

    @classmethod
    def _wait_until(cls, what, condition, timeout=DISCOVERY_TIMEOUT):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if condition():
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(f'timed out after {timeout}s waiting for {what}')

    @classmethod
    def _app_online(cls, mode, app_id):
        response = requests.get(_url(mode, f'/apps/{app_id}'), timeout=5)
        if response.status_code != 200:
            return False
        return response.json().get('x-medkit', {}).get('is_online') is True

    @classmethod
    def _entity_exists(cls, mode, path):
        return requests.get(_url(mode, path), timeout=5).status_code == 200

    @classmethod
    def _wait_for_entities(cls):
        # Hybrid links a manifest app to its node on a refresh; until then the
        # node -> app map is empty and the test would measure the unlinked case.
        for app_id in ('detector-app', 'goal_status', 'arm-app', 'wrist-app'):
            cls._wait_until(f'hybrid app {app_id} linked',
                            lambda a=app_id: cls._app_online(HYBRID, a))
        for app_id in ('scope_x_detector', 'scope_y_detector', 'goal_status', 'solo',
                       'arm', 'wrist'):
            cls._wait_until(f'runtime app {app_id}',
                            lambda a=app_id: cls._entity_exists(RUNTIME, f'/apps/{a}'))
        for path in ('/apps/detector-app', '/apps/goal_status', f'/apps/{EXTERNAL_APP}',
                     '/apps/arm-app', '/apps/wrist-app', '/apps/twin-a', '/apps/twin-b',
                     '/areas/plant', '/functions/detection', '/functions/site-watch'):
            cls._wait_until(f'manifest_only {path}',
                            lambda p=path: cls._entity_exists(MANIFEST, p))
        for path in ('/apps/twin-a', '/apps/twin-b'):
            cls._wait_until(f'hybrid {path}', lambda p=path: cls._entity_exists(HYBRID, p))

    @classmethod
    def _runtime_host_component(cls):
        items = requests.get(_url(RUNTIME, '/components'), timeout=5).json().get('items', [])
        assert items, 'runtime_only gateway has no host component'
        return items[0]['id']

    @classmethod
    def _next_code(cls, stem):
        cls._report_counter += 1
        return f'{stem}_{cls._report_counter}'

    @classmethod
    def _send_report(cls, code, source, answer_within, passed=False):
        """Send one FAILED (or PASSED) report; True once the fault manager accepted it in time."""
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = (ReportFault.Request.EVENT_PASSED if passed
                              else ReportFault.Request.EVENT_FAILED)
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'fault scope notification test'
        request.source_id = source
        future = cls._report_client.call_async(request)
        deadline = time.monotonic() + answer_within
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done() and future.result().accepted

    @classmethod
    def _report(cls, code, source):
        """Report one FAILED event and wait for the fault manager to accept it.

        Waiting keeps the reports ordered: the fault manager publishes the
        event before it answers, so the next report's event comes after it.
        """
        assert cls._send_report(code, source, answer_within=10.0), \
            f'report_fault {code} from {source} got no accepted answer'

    @classmethod
    def _report_passed(cls, code, source):
        assert cls._send_report(code, source, answer_within=10.0, passed=True), \
            f'PASSED report {code} from {source} got no accepted answer'

    @classmethod
    def _clear(cls, code, source):
        """Clear the record of ``code`` owned by ``source`` and wait for the answer."""
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        future = cls._clear_client.call_async(request)
        deadline = time.monotonic() + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        assert future.done() and future.result().success, f'clear_fault {code} failed'

    @classmethod
    def _prime_fault_streams(cls):
        """Prove each gateway receives fault events before any case measures them.

        A service reply can be lost while the fault manager is still matching
        this client, even though the request arrived, so priming does not wait
        for answers. It ends with a round trip that must be answered.
        """
        readers = {mode: SseReader(_url(mode, '/faults/stream'), f'{mode} prime')
                   for mode in PORTS}
        try:
            deadline = time.monotonic() + FAULT_TIMEOUT
            pending = set(readers)
            while pending and time.monotonic() < deadline:
                code = cls._next_code('SCOPE_PRIME')
                cls._send_report(code, '/scope/prime', answer_within=0.0)
                settle = time.monotonic() + 1.0
                while pending and time.monotonic() < settle:
                    for mode in list(pending):
                        codes = {p.get('fault', {}).get('fault_code')
                                 for p in readers[mode].snapshot()}
                        if code in codes:
                            pending.discard(mode)
                    time.sleep(0.1)
            assert not pending, \
                f'no fault event reached {[(m, readers[m].error) for m in sorted(pending)]}'
            while not cls._send_report(cls._next_code('SCOPE_PRIME'), '/scope/prime',
                                       answer_within=2.0):
                assert time.monotonic() < deadline, 'report_fault never answered'
        finally:
            for reader in readers.values():
                reader.close()

    def _watch(self, mode, entity_path):
        """Create a multishot OnChange trigger on ``{entity_path}/faults`` and stream it."""
        body = {
            'resource': f'{API_BASE_PATH}{entity_path}/faults',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 600,
        }
        response = requests.post(_url(mode, f'{entity_path}/triggers'), json=body, timeout=10)
        self.assertEqual(response.status_code, 201,
                         f'{mode} trigger on {entity_path}: {response.text}')
        trigger = response.json()
        delete_url = _url(mode, f'{entity_path}/triggers/{trigger["id"]}')
        self.addCleanup(lambda: requests.delete(delete_url, timeout=10))
        event_source = trigger['event_source'].removeprefix(API_BASE_PATH)
        reader = SseReader(_url(mode, event_source), f'{mode} {entity_path}')
        self.addCleanup(reader.close)
        return reader

    def _probe(self, targets):
        """Report one probe per source and wait until it reaches every listed reader.

        ``targets`` maps a reporting source to the readers whose entity owns it.
        Returns the failures, one line per reader the probe did not reach.
        """
        probes = []
        for source, readers in targets.items():
            code = self._next_code('SCOPE_PROBE')
            self._report(code, source)
            probes.extend((reader, code) for reader in readers)
        deadline = time.monotonic() + FAULT_TIMEOUT
        missing = list(probes)
        while missing and time.monotonic() < deadline:
            missing = [(reader, code) for reader, code in missing
                       if code not in {c for c, _ in _trigger_events(reader)}]
            time.sleep(0.2)
        return [f'{reader.label}: probe {code} never arrived; stream is not live '
                f'(error: {reader.error}, events: {_trigger_events(reader)})'
                for reader, code in missing]

    def _listed_codes(self, mode, entity_path):
        response = requests.get(_url(mode, f'{entity_path}/faults'), timeout=10)
        self.assertEqual(response.status_code, 200, f'{mode} {entity_path}: {response.text}')
        return {item['fault_code'] for item in response.json().get('items', [])}

    def _check(self, failures, reader, expected, unexpected):
        """Record missing expected (code, event_type) pairs and forbidden ones that arrived."""
        events = _trigger_events(reader)
        for pair in expected:
            if pair not in events:
                failures.append(f'{reader.label}: missing {pair}; got {events}')
        for pair in unexpected:
            if pair in events:
                failures.append(f'{reader.label}: must not receive {pair}; got {events}')

    def _check_listing(self, failures, mode, entity_path, listed, not_listed):
        codes = self._listed_codes(mode, entity_path)
        for code in listed:
            if code not in codes:
                failures.append(f'{mode} GET {entity_path}/faults does not list {code}: {codes}')
        for code in not_listed:
            if code in codes:
                failures.append(f'{mode} GET {entity_path}/faults lists {code}: {codes}')

    # ------------------------------------------------------------------
    # Triggers
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_097
    def test_01_subpath_source_fires_on_every_owner(self):
        """A sub-path source fires the triggers of its owning app and every ancestor.

        Depth 1 and depth 2 below the node FQN, in all three discovery modes;
        on a Component, on an Area that holds the app only through a subarea,
        and on a Function that hosts the app through its Component.
        """
        watched = {}
        for mode in (HYBRID, MANIFEST):
            for path in ('/apps/detector-app', '/components/detector-unit',
                         '/areas/plant', '/functions/detection'):
                watched[(mode, path)] = self._watch(mode, path)
        host = f'/components/{self._host_component}'
        for path in ('/apps/scope_x_detector', '/apps/solo', host):
            watched[(RUNTIME, path)] = self._watch(RUNTIME, path)

        d1 = self._next_code('SUB_D1')
        d2 = self._next_code('SUB_D2')
        s1 = self._next_code('SUB_S1')
        s2 = self._next_code('SUB_S2')
        self._report(d1, f'{DETECTOR_X}/goal_status')
        self._report(d2, f'{DETECTOR_X}/diag/goal_status')
        self._report(s1, f'{SOLO}/goal_status')
        self._report(s2, f'{SOLO}/diag/goal_status')

        detector_readers = [r for (mode, path), r in watched.items()
                            if path not in ('/apps/solo',)]
        failures = self._probe({
            DETECTOR_X: detector_readers,
            SOLO: [watched[(RUNTIME, '/apps/solo')], watched[(RUNTIME, host)]],
        })

        confirmed = 'fault_confirmed'
        for (mode, path), reader in watched.items():
            if path == '/apps/solo':
                codes = (s1, s2)
            elif path == host:
                codes = (d1, d2, s1, s2)
            else:
                codes = (d1, d2)
            self._check(failures, reader, [(c, confirmed) for c in codes], [])
            self._check_listing(failures, mode, path, codes, [])
        if failures:
            self.fail('\n'.join(failures))

    # @verifies REQ_INTEROP_097
    def test_02_namesake_of_last_segment_is_not_notified(self):
        """The app named like a sub-path's last segment neither lists nor hears the fault."""
        namesakes = {mode: self._watch(mode, '/apps/goal_status') for mode in PORTS}

        d1 = self._next_code('NAME_D1')
        d2 = self._next_code('NAME_D2')
        s1 = self._next_code('NAME_S1')
        self._report(d1, f'{DETECTOR_X}/goal_status')
        self._report(d2, f'{DETECTOR_X}/diag/goal_status')
        self._report(s1, f'{SOLO}/goal_status')

        failures = self._probe({GOAL_STATUS: list(namesakes.values())})
        for mode, reader in namesakes.items():
            forbidden = [(code, 'fault_confirmed') for code in (d1, d2, s1)]
            self._check(failures, reader, [], forbidden)
            self._check_listing(failures, mode, '/apps/goal_status', [], [d1, d2, s1])
        if failures:
            self.fail('\n'.join(failures))

    # @verifies REQ_INTEROP_097
    def test_03_exact_fqn_fires_in_manifest_only_and_on_a_name_collision(self):
        """An exact node FQN fires its app's trigger whatever the app id looks like."""
        manifest_detector = self._watch(MANIFEST, '/apps/detector-app')
        hybrid_detector = self._watch(HYBRID, '/apps/detector-app')
        runtime_x = self._watch(RUNTIME, '/apps/scope_x_detector')
        runtime_y = self._watch(RUNTIME, '/apps/scope_y_detector')

        exact_x = self._next_code('EXACT_X')
        exact_y = self._next_code('EXACT_Y')
        self._report(exact_x, DETECTOR_X)
        self._report(exact_y, DETECTOR_Y)

        failures = self._probe({
            DETECTOR_X: [manifest_detector, hybrid_detector, runtime_x],
            DETECTOR_Y: [runtime_y],
        })
        confirmed = 'fault_confirmed'
        self._check(failures, manifest_detector, [(exact_x, confirmed)], [])
        self._check(failures, hybrid_detector, [(exact_x, confirmed)], [])
        self._check(failures, runtime_x, [(exact_x, confirmed)], [(exact_y, confirmed)])
        self._check(failures, runtime_y, [(exact_y, confirmed)], [(exact_x, confirmed)])
        if failures:
            self.fail('\n'.join(failures))

    # @verifies REQ_INTEROP_097
    def test_04_record_outside_the_scope_fires_no_component_area_or_function(self):
        """A Component, Area or Function trigger stays silent for a record it does not own.

        The detector's record is owned by detector-unit, the plant area and the
        detection function; goal_status's record by aux-unit and the aux area.
        Each entity is checked against the other's record.
        """
        watched = {}
        for mode in (HYBRID, MANIFEST):
            for path in ('/components/detector-unit', '/areas/plant', '/areas/aux',
                         '/functions/detection'):
                watched[(mode, path)] = self._watch(mode, path)

        detector_code = self._next_code('OUT_DETECTOR')
        goal_code = self._next_code('OUT_GOAL')
        self._report(detector_code, f'{DETECTOR_X}/diag')
        self._report(goal_code, f'{GOAL_STATUS}/diag')

        aux = [r for (_, path), r in watched.items() if path == '/areas/aux']
        failures = self._probe({
            DETECTOR_X: [r for (_, path), r in watched.items() if path != '/areas/aux'],
            GOAL_STATUS: aux,
        })
        confirmed = 'fault_confirmed'
        for (mode, path), reader in watched.items():
            owned, foreign = ((goal_code, detector_code) if path == '/areas/aux'
                              else (detector_code, goal_code))
            self._check(failures, reader, [(owned, confirmed)], [(foreign, confirmed)])
            self._check_listing(failures, mode, path, [owned], [foreign])
        if failures:
            self.fail('\n'.join(failures))

    # @verifies REQ_INTEROP_097
    def test_05_external_app_bare_id_fires_its_app_and_component(self):
        """A source equal to an external app's id fires that app and its Component."""
        watched = [self._watch(mode, path)
                   for mode in (HYBRID, MANIFEST)
                   for path in (f'/apps/{EXTERNAL_APP}', '/components/plc-unit')]
        code = self._next_code('EXTERNAL')
        self._report(code, EXTERNAL_APP)
        failures = self._probe({EXTERNAL_APP: watched})
        for reader in watched:
            self._check(failures, reader, [(code, 'fault_confirmed')], [])
        if failures:
            self.fail('\n'.join(failures))

    # ------------------------------------------------------------------
    # Global fault stream
    # ------------------------------------------------------------------

    def test_06_fault_stream_names_the_owner_and_its_bag_downloads(self):
        """``x-medkit`` names the app that lists the record, or is absent.

        The documented use of ``x-medkit`` is the rosbag address
        ``/{entity_type}/{entity_id}/bulk-data/rosbags/{fault_code}``, so a named
        entity must answer 200 there. A record no app owns carries no
        ``x-medkit``, and the namesake of a sub-path's last segment is never
        named. The owner's rosbag listing includes the sub-path bag and leaves
        out the bag of a record it does not list.
        """
        streams = {mode: SseReader(_url(mode, '/faults/stream'), f'{mode} /faults/stream')
                   for mode in PORTS}
        for reader in streams.values():
            self.addCleanup(reader.close)

        detector_sub = self._next_code('STREAM_DETECTOR_SUB')
        solo_sub = self._next_code('STREAM_SOLO_SUB')
        unowned = self._next_code('STREAM_UNOWNED')
        self._report(detector_sub, f'{DETECTOR_X}/goal_status')
        self._report(solo_sub, f'{SOLO}/goal_status')
        self._report(unowned, '/scope/a/unowned')

        def events(reader):
            return {(p['fault']['fault_code'], p['event_type']): p.get('x-medkit')
                    for p in reader.snapshot() if 'fault' in p}

        last = (unowned, 'fault_confirmed')
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            if all(last in events(r) for r in streams.values()):
                break
            time.sleep(0.2)

        owner = {
            HYBRID: {detector_sub: 'detector-app', solo_sub: None},
            MANIFEST: {detector_sub: 'detector-app', solo_sub: None},
            RUNTIME: {detector_sub: 'scope_x_detector', solo_sub: 'solo'},
        }
        failures = []
        named = []
        for mode, reader in streams.items():
            seen = events(reader)
            if last not in seen:
                failures.append(f'{mode}: {last} never arrived on /faults/stream '
                                f'(error: {reader.error}): {seen}')
                continue
            if seen[last] is not None:
                failures.append(f'{mode}: {unowned} is owned by no app, '
                                f'x-medkit must be absent: {seen[last]}')
            for code, app_id in owner[mode].items():
                got = seen.get((code, 'fault_confirmed'), 'missing')
                want = None if app_id is None else {'entity_type': 'apps', 'entity_id': app_id}
                if got != want:
                    failures.append(f'{mode}: x-medkit for {code} is {got}, want {want}')
                elif app_id is not None:
                    named.append((mode, app_id, code))

        for mode, app_id, code in named:
            status = self._wait_for_status(
                _url(mode, f'/apps/{app_id}/bulk-data/rosbags/{code}'), 200)
            if status != 200:
                failures.append(f'{mode}: rosbag of {code} at the x-medkit address of '
                                f'{app_id} answered {status}')

        listed = self._listed_bag_codes(HYBRID, '/apps/detector-app', detector_sub)
        if detector_sub not in listed:
            failures.append(f'hybrid /apps/detector-app/bulk-data/rosbags does not list the '
                            f'bag of {detector_sub}: {sorted(listed)}')
        if unowned in listed:
            failures.append(f'hybrid /apps/detector-app/bulk-data/rosbags lists the bag of '
                            f'{unowned}, which the app does not list')
        if failures:
            self.fail('\n'.join(failures))

    # ------------------------------------------------------------------
    # Record end and nested owners
    # ------------------------------------------------------------------

    def _open_fault_streams(self):
        streams = {mode: SseReader(_url(mode, '/faults/stream'), f'{mode} /faults/stream')
                   for mode in PORTS}
        for reader in streams.values():
            self.addCleanup(reader.close)
        return streams

    def _await_stream_event(self, streams, key):
        """Wait until every fault stream carries ``key``; returns the failures."""
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            if all(key in _stream_events(r) for r in streams.values()):
                return []
            time.sleep(0.2)
        return [f'{mode}: {key} never arrived on /faults/stream (error: {r.error}): '
                f'{sorted(_stream_events(r))}' for mode, r in streams.items()
                if key not in _stream_events(r)]

    @staticmethod
    def _check_x_medkit(failures, streams, key, wanted):
        """``wanted`` maps each mode to the app id x-medkit must name, or None for absent."""
        for mode, reader in streams.items():
            got = _stream_events(reader).get(key, 'missing')
            want = (None if wanted[mode] is None
                    else {'entity_type': 'apps', 'entity_id': wanted[mode]})
            if got != want:
                failures.append(f'{mode}: x-medkit for {key} is {got}, want {want}')

    def _listed_all(self, mode, entity_path):
        """Fault code -> status of every fault ``GET {entity_path}/faults?status=all`` lists."""
        response = requests.get(_url(mode, f'{entity_path}/faults'),
                                params={'status': 'all'}, timeout=10)
        self.assertEqual(response.status_code, 200, f'{mode} {entity_path}: {response.text}')
        return {item['fault_code']: item.get('status')
                for item in response.json().get('items', [])}

    # @verifies REQ_INTEROP_097
    def test_07_clear_and_heal_reach_exactly_the_entities_that_own_the_fault(self):
        """A record's end reaches the entities whose scope holds it, and no other app.

        The default listing drops a cleared or healed record, so the
        fault_cleared event is how a subscriber learns the record ended. Its
        payload status tells a clear (CLEARED) from a heal (HEALED).
        """
        owner = {HYBRID: '/apps/detector-app', MANIFEST: '/apps/detector-app',
                 RUNTIME: '/apps/scope_x_detector'}
        host = f'/components/{self._host_component}'
        group_path = {HYBRID: '/functions/site-watch', MANIFEST: '/functions/site-watch',
                      RUNTIME: host}
        detectors = {mode: self._watch(mode, owner[mode]) for mode in PORTS}
        goals = {mode: self._watch(mode, '/apps/goal_status') for mode in PORTS}
        groups = {mode: self._watch(mode, group_path[mode]) for mode in PORTS}
        streams = self._open_fault_streams()

        cleared = self._next_code('END_CLEARED_SUB')
        healed = self._next_code('END_HEALED_SUB')
        self._report(cleared, f'{DETECTOR_X}/goal_status')
        self._report(healed, f'{DETECTOR_X}/diag')
        self._clear(cleared, f'{DETECTOR_X}/goal_status')
        self._report_passed(healed, f'{DETECTOR_X}/diag')
        self._report_passed(healed, f'{DETECTOR_X}/diag')

        failures = self._probe({
            DETECTOR_X: [*detectors.values(), *groups.values()],
            GOAL_STATUS: list(goals.values()),
        })
        end = 'fault_cleared'
        for mode in PORTS:
            ends = {(c, st) for c, e, st, _ in _trigger_details(detectors[mode]) if e == end}
            for pair in ((cleared, 'CLEARED'), (healed, 'HEALED')):
                if pair not in ends:
                    failures.append(f'{mode} {owner[mode]}: missing end {pair}; got {ends}')
            goal_ends = {c for c, e, _, _ in _trigger_details(goals[mode]) if e == end}
            if goal_ends & {cleared, healed}:
                failures.append(f'{mode} /apps/goal_status: must not receive ends of '
                                f'{sorted(goal_ends & {cleared, healed})}')
            group_ends = {c for c, e, _, _ in _trigger_details(groups[mode]) if e == end}
            if not {cleared, healed} <= group_ends:
                failures.append(f'{mode} {group_path[mode]}: missing ends of '
                                f'{sorted({cleared, healed} - group_ends)}')

            listed = self._listed_all(mode, owner[mode])
            if listed.get(cleared) != 'CLEARED' or listed.get(healed) != 'HEALED':
                failures.append(f'{mode} GET {owner[mode]}/faults?status=all: {listed}')

        failures += self._await_stream_event(streams, (healed, end, 1))
        owner_app = {HYBRID: 'detector-app', MANIFEST: 'detector-app',
                     RUNTIME: 'scope_x_detector'}
        self._check_x_medkit(failures, streams, (cleared, end, 1), owner_app)
        self._check_x_medkit(failures, streams, (healed, end, 1), owner_app)
        if failures:
            self.fail('\n'.join(failures))

    def test_09_fault_stream_names_the_nearest_owner_and_breaks_ties_by_id(self):
        """Nested node paths: x-medkit names the app owning the longest source.

        Both the arm and the wrist app list a fault from below the wrist node,
        and the wrist app, the more specific owner, has the larger id. Two
        manifest apps bound to one node path list the same faults; the
        smaller id is named.
        """
        streams = self._open_fault_streams()
        inner = self._next_code('NEST_INNER')
        outer = self._next_code('NEST_OUTER')
        tie = self._next_code('NEST_TIE')
        self._report(inner, '/scope/n/arm/wrist/diag')
        self._report(outer, '/scope/n/arm/diag')
        self._report(tie, '/scope/t/twin')

        c = 'fault_confirmed'
        failures = self._await_stream_event(streams, (tie, c, 1))
        self._check_x_medkit(failures, streams, (inner, c, 1),
                             {HYBRID: 'wrist-app', MANIFEST: 'wrist-app', RUNTIME: 'wrist'})
        self._check_x_medkit(failures, streams, (outer, c, 1),
                             {HYBRID: 'arm-app', MANIFEST: 'arm-app', RUNTIME: 'arm'})
        self._check_x_medkit(failures, streams, (tie, c, 1),
                             {HYBRID: 'twin-a', MANIFEST: 'twin-a', RUNTIME: None})
        # Preconditions: every app the rules choose between does list the fault.
        both = {HYBRID: ('/apps/arm-app', '/apps/wrist-app'),
                MANIFEST: ('/apps/arm-app', '/apps/wrist-app'),
                RUNTIME: ('/apps/arm', '/apps/wrist')}
        for mode, paths in both.items():
            for path in paths:
                self._check_listing(failures, mode, path, [inner], [])
        for mode in (HYBRID, MANIFEST):
            for path in ('/apps/twin-a', '/apps/twin-b'):
                self._check_listing(failures, mode, path, [tie], [])
        if failures:
            self.fail('\n'.join(failures))

    def _wait_for_status(self, url, wanted):
        """Poll a download URL until it answers ``wanted``; a bag finalizes after the fault."""
        deadline = time.monotonic() + ROSBAG_TIMEOUT
        status = None
        while time.monotonic() < deadline:
            status = requests.get(url, timeout=10).status_code
            if status == wanted:
                return status
            time.sleep(0.5)
        return status

    def _listed_bag_codes(self, mode, entity_path, awaited):
        """Fault codes on the entity's rosbag descriptors, once ``awaited`` shows or time is up."""
        deadline = time.monotonic() + ROSBAG_TIMEOUT
        codes = set()
        while time.monotonic() < deadline:
            response = requests.get(_url(mode, f'{entity_path}/bulk-data/rosbags'), timeout=10)
            self.assertEqual(response.status_code, 200, response.text)
            codes = {code for item in response.json().get('items', [])
                     for code in item.get('x-medkit', {}).get('fault_codes', [])}
            if awaited in codes:
                return codes
            time.sleep(0.5)
        return codes


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
