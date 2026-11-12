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
What the audit rows of a real fault manager say, read from the audit store it wrote.

A row's source is the report's source for a row a report drove, and the mechanism for an
automatic one (auto_confirm_timer, auto_heal, clear_service, correlation_auto_clear,
startup_reclassify). A row's owner is the owner of a per-source record and empty for a shared
record. The hash chain is recomputed here with the serialization the format had before the
owner existed, so every row of a shared record must still match it byte for byte.
"""

import hashlib
import json
import os
import signal
import sqlite3
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
from launch import LaunchDescription
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetFault, ReportFault

GENESIS = '0' * 64
HEAL = ('  confirmation_threshold: -1\n'
        '  healing_enabled: true\n'
        '  healing_threshold: 1\n')
TIMER = '  confirmation_threshold: -5\n'
PER_SOURCE = '  per_source: true\n'

RUN1_CODES = (
    'AU.SH.TIMER:\n' + TIMER +
    'AU.PS.TIMER:\n' + PER_SOURCE + TIMER +
    'AU.SH.HEAL:\n' + HEAL +
    'AU.PS.HEAL:\n' + PER_SOURCE + HEAL +
    'AU.PS.OP:\n' + PER_SOURCE +
    'AU.PS.ROOT:\n' + PER_SOURCE +
    'AU.PS.SYM:\n' + PER_SOURCE
)
# Healing is off for the HEAL codes, so a restart turns their HEALED records into CLEARED.
RUN2_CODES = 'AU.PS.HEAL:\n' + PER_SOURCE

CORRELATION = """\
correlation:
  enabled: true
  patterns:
    sh_sym:
      codes: ["AU.SH.SYM"]
    ps_sym:
      codes: ["AU.PS.SYM"]
  rules:
    - id: sh_cascade
      name: "Shared cascade"
      mode: hierarchical
      root_cause:
        codes: ["AU.SH.ROOT"]
      symptoms:
        - pattern: sh_sym
      window_ms: 5000
      mute_symptoms: true
      auto_clear_with_root: true
    - id: ps_cascade
      name: "Per-source cascade"
      mode: hierarchical
      root_cause:
        codes: ["AU.PS.ROOT"]
      symptoms:
        - pattern: ps_sym
      window_ms: 5000
      mute_symptoms: true
      auto_clear_with_root: true
"""


def generate_test_description():
    """Start only a placeholder process, the test starts the fault manager itself."""
    keep_alive = launch.actions.ExecuteProcess(cmd=['sleep', '600'])
    return LaunchDescription([keep_alive, launch_testing.actions.ReadyToTest()])


def canonical_before_owner(seq, row):
    """Serialize a row exactly as the audit format did before the owner column existed."""
    def enc(value):
        return json.dumps(value, ensure_ascii=False)
    return ('{"seq":%d,"ts":%d,"code":%s,"transition":%s,"severity":%d,"status":%s,'
            '"source":%s,"description":%s}' % (
                seq, row['occurred_at_ns'], enc(row['fault_code']), enc(row['transition']),
                row['severity'], enc(row['status']), enc(row['source_id']),
                enc(row['description'])))


def canonical_with_owner(seq, row):
    canonical = canonical_before_owner(seq, row)
    if row['owner']:
        canonical = canonical[:-1] + ',"owner":%s}' % json.dumps(row['owner'], ensure_ascii=False)
    return canonical


class TestFaultAuditRows(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.db_path = os.path.join(cls.tmp.name, 'faults.db')
        cls.audit_path = os.path.join(cls.tmp.name, 'audit.db')
        cls.run1_codes = cls._write('run1_codes.yaml', RUN1_CODES)
        cls.run2_codes = cls._write('run2_codes.yaml', RUN2_CODES)
        cls.correlation = cls._write('correlation.yaml', CORRELATION)
        cls.exe = os.path.join(
            get_package_prefix('ros2_medkit_fault_manager'),
            'lib', 'ros2_medkit_fault_manager', 'fault_manager_node')
        rclpy.init()
        cls.node = Node('test_fault_audit_rows_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.tmp.cleanup()

    @classmethod
    def _write(cls, name, text):
        path = os.path.join(cls.tmp.name, name)
        with open(path, 'w') as f:
            f.write(text)
        return path

    def _start(self, codes_file):
        params = self._write('params.yaml', (
            'fault_manager:\n'
            '  ros__parameters:\n'
            '    storage_type: sqlite\n'
            f'    database_path: {self.db_path}\n'
            '    confirmation_threshold: -1\n'
            '    healing_enabled: false\n'
            '    auto_confirm_after_sec: 1.0\n'
            '    snapshots.enabled: false\n'
            '    snapshots.rosbag.enabled: false\n'
            '    audit_log.enabled: true\n'
            f'    audit_log.database_path: {self.audit_path}\n'
            f'    fault_thresholds.config_file: {codes_file}\n'
            f'    correlation.config_file: {self.correlation}\n'
        ))
        proc = subprocess.Popen(
            [self.exe, '--ros-args', '-r', '__node:=fault_manager', '--params-file', params])
        self.addCleanup(self._kill, proc)
        clients = {
            'report': self.node.create_client(ReportFault, '/fault_manager/report_fault'),
            'get': self.node.create_client(GetFault, '/fault_manager/get_fault'),
            'clear': self.node.create_client(ClearFault, '/fault_manager/clear_fault'),
        }
        for client in clients.values():
            self.assertTrue(client.wait_for_service(timeout_sec=15.0))
        # The graph can still list the previous node's services, so wait for an answer.
        probe = GetFault.Request()
        probe.fault_code = 'NOT.A.FAULT'
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            future = clients['get'].call_async(probe)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            if future.result() is not None:
                break
        else:
            self.fail('fault_manager did not answer after start')
        return proc, clients

    @staticmethod
    def _kill(proc):
        if proc.poll() is None:
            proc.kill()
            proc.wait()

    def _stop(self, proc, clients):
        for client in clients.values():
            self.node.destroy_client(client)
        proc.send_signal(signal.SIGINT)
        self.assertEqual(proc.wait(timeout=30), 0)

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'service call timed out')
        return future.result()

    def _report(self, clients, code, source, event_type=ReportFault.Request.EVENT_FAILED):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = event_type
        request.severity = Fault.SEVERITY_ERROR
        request.description = f'{code} from {source}'
        request.source_id = source
        self.assertTrue(self._call(clients['report'], request).accepted)

    def _clear(self, clients, code, source=''):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        response = self._call(clients['clear'], request)
        self.assertTrue(response.success, response.message)

    def _status(self, clients, code, source=''):
        request = GetFault.Request()
        request.fault_code = code
        request.source_id = source
        response = self._call(clients['get'], request)
        self.assertTrue(response.success, response.error_message)
        return response.fault.status

    def _wait_status(self, clients, code, expected):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            if self._status(clients, code) == expected:
                return
        self.fail(f'{code} never reached {expected}')

    def _heal(self, clients, code, source):
        self._report(clients, code, source)
        for _ in range(4):
            if self._status(clients, code) == Fault.STATUS_HEALED:
                return
            self._report(clients, code, source, ReportFault.Request.EVENT_PASSED)
        self.assertEqual(self._status(clients, code), Fault.STATUS_HEALED)

    def _rows(self):
        connection = sqlite3.connect(f'file:{self.audit_path}?mode=ro', uri=True)
        connection.row_factory = sqlite3.Row
        try:
            return [dict(r) for r in connection.execute('SELECT * FROM audit_log ORDER BY seq')]
        finally:
            connection.close()

    def _of(self, rows, code):
        return [(r['transition'], r['source_id'], r['owner']) for r in rows
                if r['fault_code'] == code]

    def test_rows_say_what_drove_them_and_chain_matches_the_old_format(self):
        proc, clients = self._start(self.run1_codes)
        try:
            # Timer confirmation: one FAILED leaves the record PREFAILED, the timer confirms it.
            self._report(clients, 'AU.SH.TIMER', '/a')
            self._report(clients, 'AU.PS.TIMER', '/a')
            self._wait_status(clients, 'AU.SH.TIMER', Fault.STATUS_CONFIRMED)
            self._wait_status(clients, 'AU.PS.TIMER', Fault.STATUS_CONFIRMED)

            # Healing.
            self._heal(clients, 'AU.SH.HEAL', '/a')
            self._heal(clients, 'AU.PS.HEAL', '/a')

            # Operator clear.
            self._report(clients, 'AU.SH.OP', '/a')
            self._report(clients, 'AU.PS.OP', '/a')
            self._clear(clients, 'AU.SH.OP')
            self._clear(clients, 'AU.PS.OP')

            # Correlation cascade: the symptom is cleared with its root, from another node.
            self._report(clients, 'AU.SH.ROOT', '/root_node')
            self._report(clients, 'AU.SH.SYM', '/sym_node')
            self._clear(clients, 'AU.SH.ROOT')
            self._report(clients, 'AU.PS.ROOT', '/root_node')
            self._report(clients, 'AU.PS.SYM', '/sym_node')
            self._clear(clients, 'AU.PS.ROOT')
        finally:
            self._stop(proc, clients)

        # Restart with healing off for both HEAL codes: their HEALED records become CLEARED.
        proc, clients = self._start(self.run2_codes)
        self._stop(proc, clients)

        rows = self._rows()

        # Shared records: no owner on any row.
        self.assertEqual(self._of(rows, 'AU.SH.TIMER'), [
            ('occurred', '/a', ''), ('confirmed', 'auto_confirm_timer', '')])
        self.assertEqual(self._of(rows, 'AU.SH.HEAL'), [
            ('occurred', '/a', ''), ('confirmed', '/a', ''), ('healed', 'auto_heal', ''),
            ('cleared', 'startup_reclassify', '')])
        self.assertEqual(self._of(rows, 'AU.SH.OP'), [
            ('occurred', '/a', ''), ('confirmed', '/a', ''), ('cleared', 'clear_service', '')])
        self.assertEqual(self._of(rows, 'AU.SH.ROOT'), [
            ('occurred', '/root_node', ''), ('confirmed', '/root_node', ''),
            ('cleared', 'clear_service', '')])
        self.assertEqual(self._of(rows, 'AU.SH.SYM'), [
            ('occurred', '/sym_node', ''), ('confirmed', '/sym_node', ''),
            ('cleared', 'correlation_auto_clear', '')])

        # Per-source records: the same mechanisms, and the owner of the record.
        self.assertEqual(self._of(rows, 'AU.PS.TIMER'), [
            ('occurred', '/a', '/a'), ('confirmed', 'auto_confirm_timer', '/a')])
        self.assertEqual(self._of(rows, 'AU.PS.HEAL'), [
            ('occurred', '/a', '/a'), ('confirmed', '/a', '/a'), ('healed', 'auto_heal', '/a'),
            ('cleared', 'startup_reclassify', '/a')])
        self.assertEqual(self._of(rows, 'AU.PS.OP'), [
            ('occurred', '/a', '/a'), ('confirmed', '/a', '/a'),
            ('cleared', 'clear_service', '/a')])
        self.assertEqual(self._of(rows, 'AU.PS.ROOT'), [
            ('occurred', '/root_node', '/root_node'), ('confirmed', '/root_node', '/root_node'),
            ('cleared', 'clear_service', '/root_node')])
        self.assertEqual(self._of(rows, 'AU.PS.SYM'), [
            ('occurred', '/sym_node', '/sym_node'), ('confirmed', '/sym_node', '/sym_node'),
            ('cleared', 'correlation_auto_clear', '/sym_node')])

        # The chain verifies, and every row of a shared record hashes as it did before the
        # owner existed.
        previous = GENESIS
        shared_rows = 0
        for row in rows:
            self.assertEqual(row['prev_hash'], previous)
            expected = hashlib.sha256(
                (previous + canonical_with_owner(row['seq'], row)).encode()).hexdigest()
            self.assertEqual(row['record_hash'], expected, row)
            if not row['owner']:
                shared_rows += 1
                old = hashlib.sha256(
                    (previous + canonical_before_owner(row['seq'], row)).encode()).hexdigest()
                self.assertEqual(row['record_hash'], old, row)
            previous = row['record_hash']
        self.assertGreater(shared_rows, 10)
