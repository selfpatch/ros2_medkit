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
A PostgreSQL database written by the release before record identity, migrated through the node.

The tables are created with that release's layout and rows, the fault manager is started on them
with storage_type: postgres, and the migrated records are read over its services: every code is one
shared record (owner '*') with its reporting sources, and its snapshot, freeze frame and recording
link attach to it. A report from a new source joins the record, and a restart changes nothing.
Needs a server: ROS2_MEDKIT_TEST_PG_CONN, as for the storage tests.
"""

import os
import queue
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import uuid

from ament_index_python.packages import get_package_prefix
import launch
from launch import LaunchDescription
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetFault, GetRosbag, GetSnapshots, ListFaults, ReportFault

TIMEOUT_SEC = 30.0
ALL_STATUSES = [
    Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED, Fault.STATUS_CONFIRMED,
    Fault.STATUS_HEALED, Fault.STATUS_CLEARED,
]
SERVICE = '/fault_manager/report_fault'

# The tables as the release before record identity created them: one row per fault code.
MAIN_SCHEMA = """
CREATE TABLE faults (
  fault_code TEXT PRIMARY KEY, severity INTEGER NOT NULL, description TEXT NOT NULL,
  first_occurred_ns BIGINT NOT NULL, last_occurred_ns BIGINT NOT NULL,
  occurrence_count BIGINT NOT NULL, status TEXT NOT NULL, reporting_sources TEXT NOT NULL,
  debounce_counter INTEGER NOT NULL DEFAULT 0, last_failed_ns BIGINT NOT NULL DEFAULT 0,
  last_passed_ns BIGINT NOT NULL DEFAULT 0, confirmed_at_ns BIGINT NOT NULL DEFAULT 0);
CREATE TABLE snapshots (
  id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY, fault_code TEXT NOT NULL,
  topic TEXT NOT NULL, message_type TEXT NOT NULL, data TEXT NOT NULL,
  captured_at_ns BIGINT NOT NULL, capture_id BIGINT NOT NULL DEFAULT 0);
CREATE INDEX idx_snapshots_fault_code ON snapshots(fault_code);
CREATE INDEX idx_snapshots_fault_topic ON snapshots(fault_code, topic);
CREATE TABLE freeze_frames (
  fault_code TEXT PRIMARY KEY, data TEXT NOT NULL, captured_at_ns BIGINT NOT NULL);
CREATE TABLE near_misses (
  id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY, fault_code TEXT NOT NULL,
  occurred_at_ns BIGINT NOT NULL, debounce_counter INTEGER NOT NULL,
  confirmation_threshold INTEGER NOT NULL, severity INTEGER NOT NULL,
  source_id TEXT NOT NULL, resulting_status TEXT NOT NULL DEFAULT '');
CREATE INDEX idx_near_misses_fault_code ON near_misses(fault_code, id);
CREATE TABLE rosbag_files (
  id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY, fault_code TEXT NOT NULL,
  recording_id TEXT NOT NULL DEFAULT '', file_path TEXT NOT NULL, format TEXT NOT NULL,
  duration_sec DOUBLE PRECISION NOT NULL, size_bytes BIGINT NOT NULL,
  created_at_ns BIGINT NOT NULL);
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


class TestPostgresMigration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        prefix = get_package_prefix('ros2_medkit_fault_manager')
        cls.exe = os.path.join(prefix, 'lib', 'ros2_medkit_fault_manager', 'fault_manager_node')
        cls.pg_exec = os.path.join(prefix, 'lib', 'ros2_medkit_fault_manager', 'pg_exec')
        cls.base = os.environ.get('ROS2_MEDKIT_TEST_PG_CONN')
        assert cls.base, 'set ROS2_MEDKIT_TEST_PG_CONN to a PostgreSQL server'
        cls.schema = 'medkit_migrate_' + uuid.uuid4().hex[:12]
        cls.conn = cls.base + '?options=-csearch_path%3D' + cls.schema
        cls._sql(cls.base, f'CREATE SCHEMA {cls.schema}')
        rclpy.init()
        cls.node = Node('test_postgres_migration_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls._sql(cls.base, f'DROP SCHEMA IF EXISTS {cls.schema} CASCADE')
        cls.tmp.cleanup()

    @classmethod
    def _sql(cls, conn, statements):
        result = subprocess.run(
            [cls.pg_exec, conn], input=statements, text=True,
            capture_output=True)
        assert result.returncode == 0, result.stderr

    def _wait_until_service_is_gone(self):
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            names = [name for name, _ in self.node.get_service_names_and_types()]
            if SERVICE not in names:
                return
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail('the stopped fault manager is still listed in the graph')

    def _start(self):
        self._wait_until_service_is_gone()
        params = os.path.join(self.tmp.name, 'params.yaml')
        with open(params, 'w') as f:
            f.write('fault_manager:\n  ros__parameters:\n'
                    '    storage_type: postgres\n'
                    f'    database_url: "{self.conn}"\n'
                    '    confirmation_threshold: -1\n'
                    '    snapshots.enabled: false\n'
                    '    snapshots.rosbag.enabled: false\n')
        process = FaultManagerProcess(self.exe, params)
        self.addCleanup(process.kill)
        self.clients = {
            'report': self.node.create_client(ReportFault, SERVICE),
            'get': self.node.create_client(GetFault, '/fault_manager/get_fault'),
            'list': self.node.create_client(ListFaults, '/fault_manager/list_faults'),
            'snapshots': self.node.create_client(GetSnapshots, '/fault_manager/get_snapshots'),
            'rosbag': self.node.create_client(GetRosbag, '/fault_manager/get_rosbag'),
        }
        for client in self.clients.values():
            self.assertTrue(client.wait_for_service(timeout_sec=TIMEOUT_SEC))
        self.assertIsNotNone(
            process.wait_for_line('FaultManager node started'), 'the node never finished starting')
        return process

    def _stop(self, process):
        for client in self.clients.values():
            self.node.destroy_client(client)
        self.assertEqual(process.stop(), 0)

    def _call(self, name, request):
        future = self.clients[name].call_async(request)
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
        self.assertTrue(self._call('report', request).accepted)

    def _records(self, code):
        request = ListFaults.Request()
        request.statuses = ALL_STATUSES
        return [f for f in self._call('list', request).faults if f.fault_code == code]

    def _check_migrated(self, sources):
        records = self._records('MIG.CODE')
        self.assertEqual([list(r.reporting_sources) for r in records], [sources])
        for source in ('', '*', '/s1'):
            request = GetFault.Request()
            request.fault_code = 'MIG.CODE'
            request.source_id = source
            record = self._call('get', request)
            self.assertTrue(record.success, f'source {source!r}: {record.error_message}')
            self.assertEqual(record.fault.status, Fault.STATUS_CONFIRMED)
            self.assertEqual(list(record.fault.reporting_sources), sources)
        self.assertEqual([list(r.reporting_sources) for r in self._records('MIG.OLD')], [['/s3']])

        snapshots = GetSnapshots.Request()
        snapshots.fault_code = 'MIG.CODE'
        snapshots.source_id = '*'
        response = self._call('snapshots', snapshots)
        self.assertTrue(response.success, response.error_message)
        self.assertIn('/t', response.data)

        rosbag = GetRosbag.Request()
        rosbag.fault_code = 'MIG.CODE'
        rosbag.source_id = '*'
        bag = self._call('rosbag', rosbag)
        self.assertTrue(bag.success, bag.error_message)
        self.assertEqual(list(bag.fault_codes), ['MIG.CODE'])
        self.assertEqual(list(bag.owners), ['*'])

    def test_a_database_written_with_one_record_per_code_is_migrated_by_the_node(self):
        bag_path = os.path.join(self.tmp.name, 'fault_MIG.CODE_1')
        os.makedirs(bag_path)
        self._sql(self.conn, MAIN_SCHEMA + f"""
INSERT INTO faults VALUES ('MIG.CODE', 2, 'from the old release', 1, 1, 1, 'CONFIRMED',
  '["/s1","/s2"]', -1, 1, 0, 0);
INSERT INTO faults VALUES ('MIG.OLD', 1, 'cleared long ago', 1, 1, 1, 'CLEARED',
  '["/s3"]', 0, 1, 0, 0);
INSERT INTO snapshots (fault_code, topic, message_type, data, captured_at_ns)
  VALUES ('MIG.CODE', '/t', 'std_msgs/msg/Float64', '{{"data": 1.5}}', 9);
INSERT INTO freeze_frames VALUES ('MIG.CODE', '{{"/t": 1.5}}', 9);
INSERT INTO rosbag_files (fault_code, recording_id, file_path, format, duration_sec,
  size_bytes, created_at_ns)
  VALUES ('MIG.CODE', 'fault_MIG.CODE_1', '{bag_path}', 'mcap', 5.0, 100, 11);
""")

        process = self._start()
        try:
            self._check_migrated(['/s1', '/s2'])
            # A new source joins the shared record.
            self._report('MIG.CODE', '/s4')
            self._check_migrated(['/s1', '/s2', '/s4'])
        finally:
            self._stop(process)

        process = self._start()
        try:
            self._check_migrated(['/s1', '/s2', '/s4'])
        finally:
            self._stop(process)
