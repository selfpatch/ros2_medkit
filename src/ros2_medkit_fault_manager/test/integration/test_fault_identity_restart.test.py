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

"""
Record identity across restarts on one SQLite file.

- Changing a code between shared and per_source keeps the stored records as they
  were written and sends new reports to the configured identity. The fault
  manager says so once per code at startup.
- A database written when every code was one shared record opens as shared
  records: each row keeps its reporting sources and every child row (near
  misses, snapshots, freeze frame, recording link) attaches to it.

The test starts the fault manager itself, so a restart is a process exit and a
new process, not a respawn.
"""

import os
import queue
import signal
import sqlite3
import subprocess
import tempfile
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
from launch import LaunchDescription
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetFault, ListFaults, ReportFault

TIMEOUT_SEC = 30.0
ALL_STATUSES = [
    Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED, Fault.STATUS_CONFIRMED,
    Fault.STATUS_HEALED, Fault.STATUS_CLEARED,
]
SERVICE = '/fault_manager/report_fault'

# The fault manager's tables as the release before record identity created them: one row per
# fault code, no owner column anywhere.
MAIN_SCHEMA = """
CREATE TABLE faults (
  fault_code TEXT PRIMARY KEY, severity INTEGER NOT NULL, description TEXT NOT NULL,
  first_occurred_ns INTEGER NOT NULL, last_occurred_ns INTEGER NOT NULL,
  occurrence_count INTEGER NOT NULL, status TEXT NOT NULL, reporting_sources TEXT NOT NULL,
  debounce_counter INTEGER NOT NULL DEFAULT 0, last_failed_ns INTEGER NOT NULL DEFAULT 0,
  last_passed_ns INTEGER NOT NULL DEFAULT 0, confirmed_at_ns INTEGER NOT NULL DEFAULT 0);
CREATE TABLE snapshots (
  id INTEGER PRIMARY KEY AUTOINCREMENT, fault_code TEXT NOT NULL, topic TEXT NOT NULL,
  message_type TEXT NOT NULL, data TEXT NOT NULL, captured_at_ns INTEGER NOT NULL,
  capture_id INTEGER NOT NULL DEFAULT 0);
CREATE INDEX idx_snapshots_fault_code ON snapshots(fault_code);
CREATE INDEX idx_snapshots_fault_topic ON snapshots(fault_code, topic);
CREATE TABLE freeze_frames (
  fault_code TEXT PRIMARY KEY, data TEXT NOT NULL, captured_at_ns INTEGER NOT NULL);
CREATE TABLE near_misses (
  id INTEGER PRIMARY KEY AUTOINCREMENT, fault_code TEXT NOT NULL, occurred_at_ns INTEGER NOT NULL,
  debounce_counter INTEGER NOT NULL, confirmation_threshold INTEGER NOT NULL,
  severity INTEGER NOT NULL, source_id TEXT NOT NULL, resulting_status TEXT NOT NULL DEFAULT '');
CREATE INDEX idx_near_misses_fault_code ON near_misses(fault_code, id);
CREATE TABLE rosbag_files (
  id INTEGER PRIMARY KEY AUTOINCREMENT, fault_code TEXT NOT NULL,
  recording_id TEXT NOT NULL DEFAULT '', file_path TEXT NOT NULL, format TEXT NOT NULL,
  duration_sec REAL NOT NULL, size_bytes INTEGER NOT NULL, created_at_ns INTEGER NOT NULL);
CREATE INDEX idx_rosbag_files_fault_code ON rosbag_files(fault_code);
CREATE INDEX idx_rosbag_files_created_at ON rosbag_files(created_at_ns);
CREATE INDEX idx_rosbag_files_fault_created ON rosbag_files(fault_code, created_at_ns, id);
CREATE INDEX idx_rosbag_files_recording ON rosbag_files(recording_id);
CREATE INDEX idx_rosbag_files_path ON rosbag_files(file_path);
CREATE UNIQUE INDEX idx_rosbag_files_fault_path ON rosbag_files(fault_code, file_path);
"""


def generate_test_description():
    """Start only a placeholder process, so launch keeps the test running."""
    keep_alive = launch.actions.ExecuteProcess(cmd=['sleep', '600'])
    return LaunchDescription([keep_alive, launch_testing.actions.ReadyToTest()])


class FaultManagerProcess:
    """One fault_manager_node process with its output readable line by line."""

    def __init__(self, exe, params_file):
        self.proc = subprocess.Popen(
            [exe, '--ros-args', '-r', '__node:=fault_manager', '--params-file', params_file],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, errors='replace')
        self.lines = queue.Queue()
        self.seen = []
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self):
        for line in self.proc.stdout:
            self.lines.put(line)
        self.lines.put(None)

    def wait_for_line(self, text):
        """Return the first output line containing text, or None at the end or the timeout."""
        for line in self.seen:
            if text in line:
                return line
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            try:
                line = self.lines.get(timeout=1.0)
            except queue.Empty:
                continue
            if line is None:
                return None
            self.seen.append(line)
            if text in line:
                return line
        return None

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        return self.proc.wait(timeout=TIMEOUT_SEC)

    def kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()


class TestFaultIdentityRestart(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.exe = os.path.join(
            get_package_prefix('ros2_medkit_fault_manager'),
            'lib', 'ros2_medkit_fault_manager', 'fault_manager_node')
        rclpy.init()
        cls.node = Node('test_fault_identity_restart_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.tmp.cleanup()

    def _path(self, name):
        return os.path.join(self.tmp.name, name)

    def _write(self, name, text):
        path = self._path(name)
        with open(path, 'w') as f:
            f.write(text)
        return path

    def _wait_until_service_is_gone(self):
        """Wait until the graph no longer lists the previous process's services."""
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            names = [name for name, _ in self.node.get_service_names_and_types()]
            if SERVICE not in names:
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail('the stopped fault manager is still listed in the graph')

    def _start(self, db_path, thresholds):
        """Start the fault manager on db_path with a thresholds file, and wait until it answers."""
        self._wait_until_service_is_gone()
        thresholds_file = self._write('thresholds.yaml', thresholds)
        params = self._write('params.yaml', (
            'fault_manager:\n'
            '  ros__parameters:\n'
            '    storage_type: sqlite\n'
            f'    database_path: {db_path}\n'
            '    confirmation_threshold: -1\n'
            '    snapshots.enabled: false\n'
            '    snapshots.rosbag.enabled: false\n'
            f'    fault_thresholds.config_file: {thresholds_file}\n'
        ))
        process = FaultManagerProcess(self.exe, params)
        self.addCleanup(process.kill)
        self.report = self.node.create_client(ReportFault, SERVICE)
        self.get = self.node.create_client(GetFault, '/fault_manager/get_fault')
        self.list = self.node.create_client(ListFaults, '/fault_manager/list_faults')
        self.clear = self.node.create_client(ClearFault, '/fault_manager/clear_fault')
        for client in (self.report, self.get, self.list, self.clear):
            self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT_SEC))
        self.assertIsNotNone(
            process.wait_for_line('FaultManager node started'), 'the node never finished starting')
        return process

    def _stop(self, process):
        for client in (self.report, self.get, self.list, self.clear):
            self.node.destroy_client(client)
        self.assertEqual(process.stop(), 0)

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), 'service call timed out')
        return future.result()

    def _report(self, code, source):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = f'{code} from {source}'
        request.source_id = source
        self.assertTrue(self._call(self.report, request).accepted)

    def _get(self, code, source=''):
        request = GetFault.Request()
        request.fault_code = code
        request.source_id = source
        return self._call(self.get, request)

    def _records(self, code):
        request = ListFaults.Request()
        request.statuses = ALL_STATUSES
        return [f for f in self._call(self.list, request).faults if f.fault_code == code]

    def _sources_of_records(self, code):
        return sorted(list(f.reporting_sources) for f in self._records(code))

    def test_changing_the_mode_keeps_stored_records_and_routes_new_reports(self):
        db_path = self._path('mode_change.db')
        shared_then_per_source = 'M.SHARED:\n  confirmation_threshold: -1\n' \
            'N.PER:\n  per_source: true\n  confirmation_threshold: -1\n'
        per_source_then_shared = 'M.SHARED:\n  per_source: true\n  confirmation_threshold: -1\n' \
            'N.PER:\n  confirmation_threshold: -1\n'

        process = self._start(db_path, shared_then_per_source)
        try:
            for code in ('M.SHARED', 'N.PER'):
                for source in ('/a', '/b'):
                    self._report(code, source)
            self.assertEqual(self._sources_of_records('M.SHARED'), [['/a', '/b']])
            self.assertEqual(self._sources_of_records('N.PER'), [['/a'], ['/b']])
            # The startup check runs before the node says it started, and _start waited for that.
            self.assertFalse(
                [line for line in process.seen if 'is configured' in line],
                'a store that matches its configuration must not warn')
        finally:
            self._stop(process)

        process = self._start(db_path, per_source_then_shared)
        try:
            self.assertIsNotNone(
                process.wait_for_line("Fault code 'M.SHARED' is configured per_source"),
                'no startup warning for the shared record of a per_source code')
            self.assertIsNotNone(
                process.wait_for_line("Fault code 'N.PER' is configured shared"),
                'no startup warning for the per-source records of a shared code')

            # The stored records are as they were written, and addressable by their owner.
            self.assertEqual(self._sources_of_records('M.SHARED'), [['/a', '/b']])
            self.assertEqual(self._sources_of_records('N.PER'), [['/a'], ['/b']])
            shared = self._get('M.SHARED', '*')
            self.assertTrue(shared.success, shared.error_message)
            self.assertEqual(list(shared.fault.reporting_sources), ['/a', '/b'])
            for source in ('/a', '/b'):
                owned = self._get('N.PER', source)
                self.assertTrue(owned.success, owned.error_message)
                self.assertEqual(list(owned.fault.reporting_sources), [source])

            # New reports go to the configured identity.
            self._report('M.SHARED', '/c')
            self._report('N.PER', '/c')
            self.assertEqual(self._sources_of_records('M.SHARED'), [['/a', '/b'], ['/c']])
            self.assertEqual(self._sources_of_records('N.PER'), [['/a'], ['/b'], ['/c']])
            self.assertEqual(
                list(self._get('M.SHARED', '*').fault.reporting_sources), ['/a', '/b'])
            self.assertEqual(list(self._get('N.PER', '*').fault.reporting_sources), ['/c'])
        finally:
            self._stop(process)

        with sqlite3.connect(db_path) as db:
            rows = db.execute(
                'SELECT fault_code, owner FROM faults ORDER BY fault_code, owner').fetchall()
        self.assertEqual(rows, [
            ('M.SHARED', '*'), ('M.SHARED', '/c'),
            ('N.PER', '*'), ('N.PER', '/a'), ('N.PER', '/b'),
        ])

    def test_a_malformed_per_source_value_rejects_the_whole_file(self):
        thresholds = 'A:\n  per_source: true\nB:\n  per_source: "yes"\n'
        process = self._start(self._path('malformed.db'), thresholds)
        try:
            self.assertIsNotNone(
                process.wait_for_line('Failed to parse fault thresholds config'),
                'the malformed per_source value was not reported')
            # The file is dropped as a whole, so A is not per_source either: one shared record.
            self._report('A', '/a')
            self._report('A', '/b')
            records = self._records('A')
            self.assertEqual(len(records), 1)
            self.assertEqual(list(records[0].reporting_sources), ['/a', '/b'])
            self.assertTrue(self._get('A', '*').success)
        finally:
            self._stop(process)

    def test_a_store_written_with_one_record_per_code_opens_as_shared_records(self):
        db_path = self._path('legacy.db')
        bag_path = self._path('fault_MIG.CODE_1')
        os.makedirs(bag_path)
        with sqlite3.connect(db_path) as db:
            db.executescript(MAIN_SCHEMA)
            db.execute(
                "INSERT INTO faults VALUES ('MIG.CODE', 2, 'from the old release', 1, 1, 1, "
                "'CONFIRMED', '[\"/s1\",\"/s2\"]', -1, 1, 0, 0)")
            db.execute(
                "INSERT INTO faults VALUES ('MIG.OLD', 1, 'cleared long ago', 1, 1, 1, "
                "'CLEARED', '[\"/s3\"]', 0, 1, 0, 0)")
            db.execute(
                "INSERT INTO faults VALUES ('MIG.TWO', 2, 'second code', 1, 1, 1, "
                "'CONFIRMED', '[\"/s5\"]', -1, 1, 0, 0)")
            for second, source in ((5, '/s1'), (6, '/s1'), (7, '/s2')):
                db.execute(
                    'INSERT INTO near_misses (fault_code, occurred_at_ns, debounce_counter, '
                    'confirmation_threshold, severity, source_id, resulting_status) '
                    "VALUES ('MIG.CODE', ?, -1, -3, 2, ?, 'PREFAILED')", (second, source))
            db.execute(
                'INSERT INTO snapshots (fault_code, topic, message_type, data, captured_at_ns) '
                "VALUES ('MIG.CODE', '/t', 'std_msgs/msg/Float64', '{\"data\": 1.5}', 9)")
            db.execute("INSERT INTO freeze_frames VALUES ('MIG.CODE', '{\"/t\": 1.5}', 9)")
            db.execute(
                'INSERT INTO rosbag_files (fault_code, recording_id, file_path, format, '
                'duration_sec, size_bytes, created_at_ns) '
                "VALUES ('MIG.CODE', 'fault_MIG.CODE_1', ?, 'mcap', 5.0, 100, 11)", (bag_path,))

        process = self._start(db_path, 'UNRELATED.CODE:\n  confirmation_threshold: -1\n')
        try:
            # Each code is one record, with every source it recorded.
            self.assertEqual(self._sources_of_records('MIG.CODE'), [['/s1', '/s2']])
            self.assertEqual(self._sources_of_records('MIG.OLD'), [['/s3']])
            for source in ('', '*', '/s1', '/s2'):
                record = self._get('MIG.CODE', source)
                self.assertTrue(record.success, f'source {source!r}: {record.error_message}')
                self.assertEqual(record.fault.status, Fault.STATUS_CONFIRMED)
                self.assertEqual(list(record.fault.reporting_sources), ['/s1', '/s2'])
            detail = self._get('MIG.CODE').environment_data.snapshots
            self.assertEqual(sorted(s.type for s in detail), ['freeze_frame', 'rosbag'])

            # It then behaves as any shared record: a new source joins it, a clear ends it, a later
            # FAILED starts a new occurrence.
            self._report('MIG.CODE', '/s4')
            self.assertEqual(self._sources_of_records('MIG.CODE'), [['/s1', '/s2', '/s4']])
            self.assertEqual(self._get('MIG.CODE').fault.occurrence_count, 1)
            request = ClearFault.Request()
            request.fault_code = 'MIG.TWO'
            self.assertTrue(self._call(self.clear, request).success)
            self.assertEqual(self._get('MIG.TWO').fault.status, Fault.STATUS_CLEARED)
            self._report('MIG.TWO', '/s1')
            self.assertEqual(self._get('MIG.TWO').fault.occurrence_count, 2)
        finally:
            self._stop(process)

        with sqlite3.connect(db_path) as db:
            self.assertEqual(
                db.execute('SELECT fault_code, owner FROM faults ORDER BY fault_code').fetchall(),
                [('MIG.CODE', '*'), ('MIG.OLD', '*'), ('MIG.TWO', '*')])
            for table in ('snapshots', 'freeze_frames', 'rosbag_files'):
                self.assertEqual(
                    db.execute(f'SELECT owner FROM {table}').fetchall(), [('*',)], table)
            # Every near miss of both sources belongs to the one shared record.
            self.assertEqual(
                db.execute(
                    "SELECT source_id FROM near_misses WHERE fault_code = 'MIG.CODE' "
                    "AND owner = '*' ORDER BY id").fetchall(),
                [('/s1',), ('/s1',), ('/s2',)])
            self.assertEqual(
                db.execute(
                    'SELECT COUNT(*) FROM near_misses n WHERE NOT EXISTS '
                    '(SELECT 1 FROM faults f WHERE f.fault_code = n.fault_code '
                    'AND f.owner = n.owner)').fetchone(),
                (0,))
