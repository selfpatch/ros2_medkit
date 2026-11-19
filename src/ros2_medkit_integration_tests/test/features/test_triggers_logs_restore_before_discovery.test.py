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

"""Feature test: a restored logs trigger fires for entries logged before discovery.

A persistent logs trigger is restored while the gateway is constructed, and its
entity may reach the discovery cache much later. An entry its entity's
GET /logs lists once the entity is discovered must reach the trigger, even when
it was logged before that, and must reach no other entity's trigger. Both
triggers have log_settings and write one marker per firing, the first one
during the replay. A marker fires no trigger. It is logged under the trigger's
entity id, so the root app lists its own markers and neither app lists the
namespaced app's.

Two nodes named late_logger make the owner rule visible. Node
/late/late_logger gets the app id late_late_logger, because the root node
/late_logger holds the bare name, so the last segment of its logger is the id
of the other app. A rule that guesses the owner from that segment sends
the namespaced node's entries to late_logger and none to late_late_logger.

The restarted gateway runs on a DDS domain of its own. Its nodes are started by
the test, root node first so the ids never change, after the test has published
an entry under each node's logger name, so each entry provably arrives before
its entity exists.
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
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rcl_interfaces.msg import Log
import rclpy
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.coverage import get_coverage_env
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

PORT_PRIMARY = get_test_port(0)
PORT_RESTARTED = get_test_port(1)
BASE_URL_PRIMARY = f'http://localhost:{PORT_PRIMARY}{API_BASE_PATH}'
BASE_URL_RESTARTED = f'http://localhost:{PORT_RESTARTED}{API_BASE_PATH}'

PRIMARY_DOMAIN = get_test_domain_id(0)
RESTARTED_DOMAIN = get_test_domain_id(1)

TIME_SCALE = get_time_scale()
HTTP_TIMEOUT_SEC = 5.0 * TIME_SCALE
DISCOVERY_TIMEOUT_SEC = 60.0 * TIME_SCALE
DELIVERY_TIMEOUT_SEC = 20.0 * TIME_SCALE


def _level(constant):
    """Return a Log severity constant as an int. Humble declares the constants as byte."""
    return constant if isinstance(constant, int) else ord(constant)


# Logs one INFO line at startup and nothing after.
NODE_EXECUTABLE = 'demo_calibration_service'
# The startup line reaches the gateway only when /rosout matched first, and it fires the
# trigger like any entry. At warn the node publishes nothing, so the test's entries are
# the only entries of each app.
NODE_LOG_LEVEL = 'warn'
# Root node: its logger name is its app id.
NAMESAKE_APP = 'late_logger'
# /late/late_logger: collides with the root node, so its id carries the namespace.
OWNER_APP = 'late_late_logger'
OWNER_NAMESPACE = '/late'
# Slash form: the last segment is NAMESAKE_APP.
OWNER_LOGGER = 'late/late_logger'

RUN = uuid.uuid4().hex[:10]
MARKER = re.compile(rf'{RUN}:([A-Za-z0-9_.-]+)')

DB_PATH = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_logs_restore_before_discovery_{os.getpid()}.db',
)
GATE_RESTART = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_logs_restore_before_discovery_restart_{os.getpid()}',
)
GATE_NAMESAKE = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_logs_restore_before_discovery_namesake_{os.getpid()}',
)
GATE_OWNER = os.path.join(
    tempfile.gettempdir(),
    f'test_triggers_logs_restore_before_discovery_owner_{os.getpid()}',
)


def _gate_process(name, path):
    return ExecuteProcess(
        cmd=['sh', '-c', f'while [ ! -e "{path}" ]; do sleep 0.2; done'],
        name=name,
        output='screen',
    )


def _app_node(domain, namespace):
    env = get_coverage_env('ros2_medkit_integration_tests')
    env['ROS_DOMAIN_ID'] = str(domain)
    return launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable=NODE_EXECUTABLE,
        name='late_logger',
        namespace=namespace,
        arguments=['--ros-args', '--log-level', NODE_LOG_LEVEL],
        output='screen',
        additional_env=env,
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


def generate_test_description():
    primary = create_gateway_node(
        name=f'ros2_medkit_gateway_{PORT_PRIMARY}',
        port=PORT_PRIMARY,
        extra_params={
            'triggers.enabled': True,
            'triggers.storage.path': DB_PATH,
            'triggers.on_restart_behavior': 'reset',
        },
        extra_env={'ROS_DOMAIN_ID': str(PRIMARY_DOMAIN)},
    )
    restarted = create_gateway_node(
        name=f'ros2_medkit_gateway_{PORT_RESTARTED}',
        port=PORT_RESTARTED,
        extra_params={
            'triggers.enabled': True,
            'triggers.storage.path': DB_PATH,
            'triggers.on_restart_behavior': 'restore',
            'refresh_interval_ms': 1000,
        },
        extra_env={'ROS_DOMAIN_ID': str(RESTARTED_DOMAIN)},
    )

    restart_gate = _gate_process('restart_gate', GATE_RESTART)
    namesake_gate = _gate_process('namesake_gate', GATE_NAMESAKE)
    owner_gate = _gate_process('owner_gate', GATE_OWNER)

    return (
        LaunchDescription([
            primary,
            restart_gate,
            RegisterEventHandler(
                OnProcessExit(target_action=restart_gate, on_exit=[restarted]),
            ),
            namesake_gate,
            RegisterEventHandler(
                OnProcessExit(target_action=namesake_gate,
                              on_exit=[_app_node(RESTARTED_DOMAIN, '/')]),
            ),
            owner_gate,
            RegisterEventHandler(
                OnProcessExit(target_action=owner_gate,
                              on_exit=[_app_node(RESTARTED_DOMAIN, OWNER_NAMESPACE)]),
            ),
            TimerAction(
                period=2.0,
                actions=[
                    _app_node(PRIMARY_DOMAIN, '/'),
                    _app_node(PRIMARY_DOMAIN, OWNER_NAMESPACE),
                    launch_testing.actions.ReadyToTest(),
                ],
            ),
        ]),
        {'primary': primary, 'restarted': restarted},
    )


def _remove_gates():
    for path in (GATE_RESTART, GATE_NAMESAKE, GATE_OWNER):
        if os.path.exists(path):
            try:
                os.unlink(path)
            except OSError:
                pass


def _open_gate(path):
    with open(path, 'w', encoding='utf-8') as gate:
        gate.write('open')


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


def _at_least(items, count):
    """Return `items` when it holds `count` or more entries, else None."""
    return items if len(items) >= count else None


def _id_number(entry_id):
    """Return n of a log entry id "log_<n>"; n grows with every entry the gateway logs."""
    return int(entry_id.removeprefix('log_'))


def _status(url):
    return requests.get(url, timeout=HTTP_TIMEOUT_SEC).status_code


def _listed(base_url, app_id):
    r = requests.get(f'{base_url}/apps/{app_id}/logs', timeout=HTTP_TIMEOUT_SEC)
    r.raise_for_status()
    found = {}
    for item in r.json().get('items', []):
        match = MARKER.search(item.get('message', ''))
        if match:
            found.setdefault(match.group(1), []).append(item.get('id'))
    return found


def _listed_past(base_url, app_id, marker, last_id):
    """Return the listing of `app_id` once it holds `marker` with an id past `last_id`."""
    found = _listed(base_url, app_id)
    return found if any(_id_number(i) > last_id for i in found.get(marker, [])) else None


def _markers(payloads):
    """Return [(marker, entry id), ...] in delivery order."""
    out = []
    for payload in payloads:
        if isinstance(payload, dict):
            match = MARKER.search(payload.get('message', ''))
            if match:
                out.append((match.group(1), payload.get('id')))
    return out


class EventReader:
    """Read a trigger's SSE stream on a daemon thread."""

    def __init__(self, events_url):
        self.payloads = []
        self.error = None
        self._closed = False
        self._cond = threading.Condition()
        self._resp = requests.get(events_url, stream=True,
                                  timeout=(HTTP_TIMEOUT_SEC, 120.0 * TIME_SCALE))
        assert self._resp.status_code == 200, f'GET {events_url}: {self._resp.status_code}'
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        try:
            for line in self._resp.iter_lines(chunk_size=None, decode_unicode=True):
                if line and line.startswith('data: '):
                    envelope = json.loads(line[len('data: '):])
                    with self._cond:
                        self.payloads.append(envelope.get('payload'))
                        self._cond.notify_all()
        except Exception as exc:
            if not self._closed:
                self.error = exc

    def markers(self):
        with self._cond:
            return _markers(self.payloads)

    def count(self):
        """Return the number of events delivered so far, one per firing."""
        with self._cond:
            return len(self.payloads)

    def others(self):
        """Return the delivered payloads that carry no marker of this run."""
        with self._cond:
            return [p for p in self.payloads
                    if not (isinstance(p, dict) and MARKER.search(p.get('message', '')))]

    def wait_for(self, marker, timeout):
        deadline = time.monotonic() + timeout
        with self._cond:
            while not any(m == marker for m, _ in _markers(self.payloads)):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._cond.wait(timeout=min(remaining, 0.5))
        return True

    def ended(self, timeout):
        """Return True once the gateway has ended the stream."""
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def close(self):
        self._closed = True
        self._resp.close()


class RosoutProbe:
    """Publish /rosout entries on the restarted gateway's domain."""

    def __init__(self, domain):
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = rclpy.create_node('log_restore_probe', context=self.context)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.pub = self.node.create_publisher(Log, '/rosout', qos)

    def publish(self, case_id, logger_name):
        msg = Log()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.level = _level(Log.INFO)
        msg.name = logger_name
        msg.msg = f'{RUN}:{case_id}'
        msg.file = 'test_triggers_logs_restore_before_discovery.test.py'
        msg.function = 'publish'
        msg.line = 1
        self.pub.publish(msg)

    def wait_delivered(self):
        """Return True once every subscriber has acknowledged every published entry."""
        return self.pub.wait_for_all_acked(Duration(seconds=DELIVERY_TIMEOUT_SEC))

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


class TestLogsTriggerRestoreBeforeDiscovery(unittest.TestCase):
    """An entry logged before discovery reaches its owner's restored trigger only."""

    trigger_ids = {}

    @classmethod
    def setUpClass(cls):
        cls.addClassCleanup(_remove_gates)
        for app_id in (NAMESAKE_APP, OWNER_APP):
            _poll(lambda a=app_id: _status(f'{BASE_URL_PRIMARY}/apps/{a}') == 200,
                  what=f'{app_id} on the primary gateway')

    # @verifies REQ_INTEROP_097
    def test_01_create_persistent_logs_triggers(self):
        """POST a persistent logs trigger on each app, then restart."""
        for app_id in (NAMESAKE_APP, OWNER_APP):
            r = requests.post(
                f'{BASE_URL_PRIMARY}/apps/{app_id}/triggers',
                json={
                    'resource': f'{API_BASE_PATH}/apps/{app_id}/logs',
                    'trigger_condition': {'condition_type': 'OnChange'},
                    'multishot': True,
                    'persistent': True,
                    'lifetime': 3600,
                    'log_settings': {'severity': 'info', 'marker': f'{RUN}:ls-{app_id}'},
                },
                timeout=HTTP_TIMEOUT_SEC,
            )
            self.assertEqual(r.status_code, 201, r.text)
            TestLogsTriggerRestoreBeforeDiscovery.trigger_ids[app_id] = r.json()['id']
        _open_gate(GATE_RESTART)

    def _open_stream(self, app_id):
        r = requests.get(
            f'{BASE_URL_RESTARTED}/apps/{app_id}/triggers/{self.trigger_ids[app_id]}',
            timeout=HTTP_TIMEOUT_SEC)
        self.assertEqual(r.status_code, 200, r.text)
        return EventReader(BASE_URL_RESTARTED
                           + r.json()['event_source'].removeprefix(API_BASE_PATH))

    def _discover(self, gate, app_id, marker):
        _open_gate(gate)
        _poll(lambda: _status(f'{BASE_URL_RESTARTED}/apps/{app_id}') == 200,
              what=f'{app_id} on the restarted gateway')
        return _poll(lambda: _listed(BASE_URL_RESTARTED, app_id).get(marker),
                     what=f'GET /apps/{app_id}/logs to list {marker}')

    # @verifies REQ_INTEROP_097
    def test_02_entries_before_discovery_reach_their_owner_only(self):
        """Each early entry is delivered once, first, to the app that lists it."""
        self.assertEqual(len(self.trigger_ids), 2, 'test_01 must create both triggers')
        _poll(lambda: _status(f'{BASE_URL_RESTARTED}/health') == 200,
              what='the restarted gateway')

        probe = RosoutProbe(RESTARTED_DOMAIN)
        readers = {}
        problems_at_close = []
        try:
            _poll(lambda: probe.pub.get_subscription_count() >= 1,
                  what='the restarted gateway to subscribe /rosout')
            for app_id in (NAMESAKE_APP, OWNER_APP):
                self.assertEqual(_status(f'{BASE_URL_RESTARTED}/apps/{app_id}'), 404,
                                 f'{app_id} must not exist before its node starts')
            probe.publish('owner-early', OWNER_LOGGER)
            probe.publish('namesake-early', NAMESAKE_APP)
            self.assertTrue(probe.wait_delivered(),
                            'the gateway did not receive the early entries')
            for app_id in (NAMESAKE_APP, OWNER_APP):
                self.assertEqual(_status(f'{BASE_URL_RESTARTED}/apps/{app_id}'), 404,
                                 f'{app_id} appeared before the early entries were logged')

            listed = {
                NAMESAKE_APP: self._discover(GATE_NAMESAKE, NAMESAKE_APP, 'namesake-early'),
                OWNER_APP: self._discover(GATE_OWNER, OWNER_APP, 'owner-early'),
            }
            self.assertEqual(_status(f'{BASE_URL_RESTARTED}/apps/{NAMESAKE_APP}'), 200)
            self.assertNotIn('owner-early', _listed(BASE_URL_RESTARTED, NAMESAKE_APP))

            for app_id in (NAMESAKE_APP, OWNER_APP):
                readers[app_id] = self._open_stream(app_id)
            probe.publish('owner-late', OWNER_LOGGER)
            probe.publish('namesake-late', NAMESAKE_APP)
            for app_id, marker in ((OWNER_APP, 'owner-late'), (NAMESAKE_APP, 'namesake-late')):
                reader = readers[app_id]
                self.assertTrue(reader.wait_for(marker, DELIVERY_TIMEOUT_SEC),
                                f'{app_id}: {marker} not delivered: {reader.markers()}, '
                                f'stream error {reader.error!r}')

            # A firing writes its marker after it delivers the event. Once each firing
            # so far has its marker listed, a marker with an id past the next entry's
            # belongs to that entry or a later one.
            fired = readers[NAMESAKE_APP].count()
            _poll(lambda: _at_least(
                _listed(BASE_URL_RESTARTED, NAMESAKE_APP).get(f'ls-{NAMESAKE_APP}', []), fired),
                what=f'GET /apps/{NAMESAKE_APP}/logs to list a marker per delivered event')
            # Each trigger delivers in log order, so once an entry logged after the
            # others arrives, no earlier entry can still reach it.
            probe.publish('owner-done', OWNER_LOGGER)
            probe.publish('namesake-done', NAMESAKE_APP)
            for app_id, marker in ((OWNER_APP, 'owner-done'), (NAMESAKE_APP, 'namesake-done')):
                self.assertTrue(readers[app_id].wait_for(marker, DELIVERY_TIMEOUT_SEC),
                                f'{app_id}: {marker} not delivered: {readers[app_id].markers()}')
            # One notifier worker dispatches the entries in log order and writes the
            # markers of an entry before it takes the next. namesake-done is the last
            # entry, so once its marker is listed no trigger writes another.
            done_id = _id_number(dict(readers[NAMESAKE_APP].markers())['namesake-done'])
            listings = {
                NAMESAKE_APP: _poll(
                    lambda: _listed_past(BASE_URL_RESTARTED, NAMESAKE_APP,
                                         f'ls-{NAMESAKE_APP}', done_id),
                    what=f'GET /apps/{NAMESAKE_APP}/logs to list the marker of namesake-done'),
                OWNER_APP: _listed(BASE_URL_RESTARTED, OWNER_APP),
            }
            owner = readers[OWNER_APP].markers()
            namesake = readers[NAMESAKE_APP].markers()
            problems = []
            # The root app lists one marker per own entry. Neither app lists the namespaced
            # app's markers: they are logged under its id, which neither app's log holds.
            expected_markers = {NAMESAKE_APP: {f'ls-{NAMESAKE_APP}': 3}, OWNER_APP: {}}
            for app_id, want in expected_markers.items():
                got = {name: len(ids) for name, ids in listings[app_id].items()
                       if name.startswith('ls-')}
                if got != want:
                    problems.append(f'{app_id}: GET lists markers {got}, expected {want}: '
                                    f'{listings[app_id]}')
            for app_id, reader in readers.items():
                if reader.others():
                    problems.append(f'{app_id}: delivered entries the test never logged: '
                                    f'{reader.others()}')
            if [i for m, i in owner if m == 'owner-early'] != listed[OWNER_APP]:
                problems.append(f'{OWNER_APP}: early entry listed as {listed[OWNER_APP]}, '
                                f'delivered {owner}')
            if [m for m, _ in owner] != ['owner-early', 'owner-late', 'owner-done']:
                problems.append(f'{OWNER_APP}: expected each own entry once, in log order, '
                                f'got {owner}')
            if [m for m, _ in namesake] != ['namesake-early', 'namesake-late', 'namesake-done']:
                problems.append(f'{NAMESAKE_APP}: expected only its own entries, once, in log '
                                f'order, got {namesake}')
            self.assertFalse(problems, '; '.join(problems))
        finally:
            # A trigger's stream wakes only for an event, the trigger's removal or its
            # 15 s keepalive. Removing the triggers ends both streams now, so neither the
            # close below nor the gateway's shutdown waits for a keepalive.
            for app_id, reader in readers.items():
                requests.delete(
                    f'{BASE_URL_RESTARTED}/apps/{app_id}/triggers/{self.trigger_ids[app_id]}',
                    timeout=HTTP_TIMEOUT_SEC)
                if not reader.ended(DELIVERY_TIMEOUT_SEC):
                    problems_at_close.append(f'{app_id}: the stream outlived its trigger')
            for reader in readers.values():
                reader.close()
            probe.close()
        self.assertFalse(problems_at_close, '; '.join(problems_at_close))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
        for path in (DB_PATH, f'{DB_PATH}-wal', f'{DB_PATH}-shm'):
            if os.path.exists(path):
                try:
                    os.unlink(path)
                except OSError:
                    pass
        _remove_gates()
