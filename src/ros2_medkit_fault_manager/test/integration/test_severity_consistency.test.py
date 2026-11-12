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
Severity contract of a fault record through a real fault manager on SQLite.

The database starts as one written before records had an owner, so the first
start migrates it. The fault manager is then restarted twice on it. Every
record here is the shared record of its code. A record keeps the highest
severity ever reported for it. A report on a CRITICAL record confirms it. A reactivated occurrence,
migrated or not, has no confirmation time until it confirms. A report whose
severity differs from the stored one is logged once per process.
"""

import os
import shutil
import signal
import sqlite3
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import ClearFault, ListFaults, ReportFault

DB_DIR = tempfile.mkdtemp(prefix='medkit_severity_')
DB_PATH = os.path.join(DB_DIR, 'faults.db')
EVENT_TOPIC = '/fault_manager/events'
SOURCE = '/robot/reporter'
SHARED_OWNER = '*'
ALL_STATUSES = [
    Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED, Fault.STATUS_CONFIRMED,
    Fault.STATUS_HEALED, Fault.STATUS_CLEARED,
]
MISMATCH_LINE = "Fault 'SEV_CRIT' reported with severity WARN"
TIMEOUT_SEC = 30.0
LEGACY_CONFIRMED_AT_NS = 1_000_000_000

# The faults table as releases before record identity created it: one row per code.
LEGACY_FAULTS_DDL = (
    'CREATE TABLE faults (fault_code TEXT PRIMARY KEY, severity INTEGER NOT NULL, '
    'description TEXT NOT NULL, first_occurred_ns INTEGER NOT NULL, '
    'last_occurred_ns INTEGER NOT NULL, occurrence_count INTEGER NOT NULL, '
    'status TEXT NOT NULL, reporting_sources TEXT NOT NULL, '
    'debounce_counter INTEGER NOT NULL DEFAULT 0, '
    'last_failed_ns INTEGER NOT NULL DEFAULT 0, last_passed_ns INTEGER NOT NULL DEFAULT 0, '
    'confirmed_at_ns INTEGER NOT NULL DEFAULT 0)')


# Every legacy row carries LEGACY_CONFIRMED_AT_NS. The pending ones were left so by a reactivation.
LEGACY_ROWS = (
    ('LEGACY_CONF_AT', Fault.SEVERITY_ERROR, Fault.STATUS_CLEARED),
    ('LEGACY_CRIT', Fault.SEVERITY_CRITICAL, Fault.STATUS_CLEARED),
    ('LEGACY_PREFAILED', Fault.SEVERITY_ERROR, Fault.STATUS_PREFAILED),
    ('LEGACY_PREPASSED', Fault.SEVERITY_WARN, Fault.STATUS_PREPASSED),
    ('LEGACY_CONFIRMED', Fault.SEVERITY_ERROR, Fault.STATUS_CONFIRMED),
    ('LEGACY_HEALED', Fault.SEVERITY_ERROR, Fault.STATUS_HEALED),
)


def seed_legacy_database():
    """Write the LEGACY_ROWS into a faults table that has no owner column."""
    with sqlite3.connect(DB_PATH) as db:
        db.execute(LEGACY_FAULTS_DDL)
        for code, severity, status in LEGACY_ROWS:
            db.execute(
                'INSERT INTO faults VALUES (?, ?, ?, ?, ?, 1, ?, ?, -3, ?, 0, ?)',
                (code, severity, 'legacy', 500_000_000, 500_000_000, status,
                 f'["{SOURCE}"]', 500_000_000, LEGACY_CONFIRMED_AT_NS))


# Codes whose records are per source, so the stored severity decides immediate confirmation. The
# migrated rows and every other code are shared records, where the report's severity decides.
PER_SOURCE_CODES = ('SEV_CRIT', 'HEAL_CRIT') + tuple(f'SEV_MARK_{i}' for i in range(1, 9))
THRESHOLDS_PATH = os.path.join(DB_DIR, 'fault_thresholds.yaml')


def generate_test_description():
    seed_legacy_database()
    with open(THRESHOLDS_PATH, 'w') as f:
        for code in PER_SOURCE_CODES:
            f.write(f'{code}:\n  per_source: true\n')
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'sqlite',
            'database_path': DB_PATH,
            'confirmation_threshold': -3,
            'healing_enabled': True,
            'healing_threshold': 1,
            'fault_thresholds.config_file': THRESHOLDS_PATH,
            'snapshots.enabled': False,
            'snapshots.rosbag.enabled': False,
        }],
        # The test restarts the node by PID; launch brings it back on the same database.
        respawn=True,
        respawn_delay=1.0,
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([fault_manager_node, launch_testing.actions.ReadyToTest()]),
        {'fault_manager_node': fault_manager_node},
    )


class TestSeverityConsistency(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_severity_consistency_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        cls.events = []
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        cls.event_sub = cls.node.create_subscription(
            FaultEvent, EVENT_TOPIC, cls.events.append, qos)
        assert cls.report_client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        assert cls.list_client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        assert cls.clear_client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        cls.primes = 0
        cls.markers = 0

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request, timeout=10.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        if not future.done():
            client.remove_pending_request(future)
            return None
        return future.result()

    def _report(self, code, severity, event_type=ReportFault.Request.EVENT_FAILED):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = event_type
        request.severity = severity
        request.description = f'{code} report'
        request.source_id = SOURCE
        response = self._call(self.report_client, request)
        self.assertIsNotNone(response, 'report_fault timed out')
        self.assertTrue(response.accepted)

    def _passed(self, code):
        self._report(code, Fault.SEVERITY_INFO, ReportFault.Request.EVENT_PASSED)

    def _clear(self, code):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = SOURCE
        response = self._call(self.clear_client, request)
        self.assertIsNotNone(response, 'clear_fault timed out')
        self.assertTrue(response.success)

    def _list(self, statuses=None, severity=None):
        request = ListFaults.Request()
        request.statuses = statuses if statuses is not None else ALL_STATUSES
        if severity is not None:
            request.filter_by_severity = True
            request.severity = severity
        response = self._call(self.list_client, request)
        self.assertIsNotNone(response, 'list_faults timed out')
        return {f.fault_code: f for f in response.faults}

    def _fault(self, code):
        fault = self._list().get(code)
        self.assertIsNotNone(fault, f'{code} is not stored')
        return fault

    def _confirmed_at(self, code):
        with sqlite3.connect(DB_PATH, timeout=10.0) as db:
            row = db.execute(
                'SELECT confirmed_at_ns FROM faults WHERE fault_code = ? AND owner = ?',
                (code, SHARED_OWNER)).fetchone()
        self.assertIsNotNone(row, f'{code} has no row in the database')
        return row[0]

    def _wait_for_event(self, code, event_type, occurrence):
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            for event in list(self.events):
                if (event.fault.fault_code, event.event_type, event.fault.occurrence_count) == \
                        (code, event_type, occurrence):
                    return event
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return None

    def _prime_events(self):
        """Report fresh CRITICAL codes until one event arrives, so the event topic is matched."""
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            type(self).primes += 1
            code = f'PRIME_{self.primes}'
            self._report(code, Fault.SEVERITY_CRITICAL)
            settle = time.monotonic() + 1.0
            while time.monotonic() < settle:
                if any(e.fault.fault_code == code for e in self.events):
                    return
                rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail('no fault event arrived from the fault manager')

    def _restart(self, fault_manager_node):
        old_pid = fault_manager_node.process_details['pid']
        os.kill(old_pid, signal.SIGTERM)
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline and fault_manager_node.process_details['pid'] == old_pid:
            time.sleep(0.2)
        self.assertNotEqual(
            fault_manager_node.process_details['pid'], old_pid,
            'the fault manager was not restarted')

        probe = ListFaults.Request()
        probe.statuses = ALL_STATUSES
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            if self.list_client.wait_for_service(timeout_sec=1.0):
                if self._call(self.list_client, probe, timeout=2.0) is not None:
                    self._prime_events()
                    return
        self.fail('the restarted fault manager never answered list_faults')

    def _mismatch_lines(self, proc_output):
        """
        Count the SEV_CRIT mismatch warnings logged so far.

        A marker code mismatches once, after everything reported before it. The output is one
        ordered stream, so once the marker's line has arrived every earlier line has too.
        """
        type(self).markers += 1
        marker = f'SEV_MARK_{self.markers}'
        self._report(marker, Fault.SEVERITY_CRITICAL)
        self._report(marker, Fault.SEVERITY_WARN)
        marker_line = f"Fault '{marker}' reported with severity WARN"

        def text():
            return ''.join(io.text.decode(errors='replace') for io in proc_output)
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline and marker_line not in text():
            time.sleep(0.1)
        self.assertIn(marker_line, text(), 'the marker mismatch was never logged')
        return text().count(MISMATCH_LINE)

    def test_severity_contract_across_restarts(self, fault_manager_node, proc_output):
        self._prime_events()

        # Process 1 migrated the legacy rows. Only a row that confirmed keeps its time.
        for code, _, status in LEGACY_ROWS:
            pending = status in (Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED)
            with self.subTest(case='migrated row', code=code):
                self.assertEqual(self._fault(code).status, status)
                self.assertEqual(self._confirmed_at(code),
                                 0 if pending else LEGACY_CONFIRMED_AT_NS)

        # A cleared row's next FAILED starts a new occurrence.
        self._report('LEGACY_CONF_AT', Fault.SEVERITY_ERROR)
        self._report('LEGACY_CRIT', Fault.SEVERITY_WARN)
        with self.subTest(case='migrated, reactivated, not yet confirmed'):
            fault = self._fault('LEGACY_CONF_AT')
            self.assertEqual(fault.status, Fault.STATUS_PREFAILED)
            self.assertEqual(fault.occurrence_count, 2)
            self.assertEqual(self._confirmed_at('LEGACY_CONF_AT'), 0)
        with self.subTest(case='migrated shared CRITICAL, WARN: the report decides'):
            fault = self._fault('LEGACY_CRIT')
            self.assertEqual(fault.severity, Fault.SEVERITY_CRITICAL)
            self.assertEqual(fault.status, Fault.STATUS_PREFAILED)

        # Process 1: CRITICAL, cleared; CRITICAL, healed; ERROR confirmed, cleared.
        self._report('SEV_CRIT', Fault.SEVERITY_CRITICAL)
        self.assertEqual(self._fault('SEV_CRIT').status, Fault.STATUS_CONFIRMED)
        self._clear('SEV_CRIT')

        self._report('HEAL_CRIT', Fault.SEVERITY_CRITICAL)
        self._passed('HEAL_CRIT')
        self._passed('HEAL_CRIT')
        self.assertEqual(self._fault('HEAL_CRIT').status, Fault.STATUS_HEALED)

        for _ in range(3):
            self._report('CONF_AT', Fault.SEVERITY_ERROR)
        self.assertEqual(self._fault('CONF_AT').status, Fault.STATUS_CONFIRMED)
        first_confirmed_at = self._confirmed_at('CONF_AT')
        self.assertGreater(first_confirmed_at, 0)
        self._clear('CONF_AT')

        self._restart(fault_manager_node)

        # Process 2: one WARN report on a CRITICAL code confirms it, after a clear or a heal.
        self._report('SEV_CRIT', Fault.SEVERITY_WARN)
        self._report('HEAL_CRIT', Fault.SEVERITY_WARN)
        with self.subTest(case='CRITICAL, clear, restart, WARN'):
            fault = self._fault('SEV_CRIT')
            self.assertEqual(fault.severity, Fault.SEVERITY_CRITICAL)
            self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
            self.assertEqual(fault.occurrence_count, 2)
            event = self._wait_for_event('SEV_CRIT', FaultEvent.EVENT_CONFIRMED, 2)
            self.assertIsNotNone(event, 'no fault_confirmed event for occurrence 2')
            self.assertEqual(event.fault.severity, Fault.SEVERITY_CRITICAL)

        with self.subTest(case='CRITICAL, heal, restart, WARN'):
            fault = self._fault('HEAL_CRIT')
            self.assertEqual(fault.severity, Fault.SEVERITY_CRITICAL)
            self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
            self.assertEqual(fault.occurrence_count, 1)

        with self.subTest(case='no pending CRITICAL'):
            pending = self._list(statuses=[Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED],
                                 severity=Fault.SEVERITY_CRITICAL)
            # LEGACY_CRIT is a shared record, which a WARN report leaves pending.
            self.assertEqual(sorted(pending), ['LEGACY_CRIT'])

        for _ in range(9):
            self._report('SEV_CRIT', Fault.SEVERITY_WARN)

        self._report('CONF_AT', Fault.SEVERITY_ERROR)
        with self.subTest(case='reactivated, not yet confirmed'):
            self.assertEqual(self._fault('CONF_AT').status, Fault.STATUS_PREFAILED)
            self.assertEqual(self._confirmed_at('CONF_AT'), 0)

        with self.subTest(case='mismatch logged once in this process'):
            self.assertEqual(self._mismatch_lines(proc_output), 1)

        self._restart(fault_manager_node)

        # Process 3: the reactivated occurrences still have no confirmation time, then confirm.
        with self.subTest(case='migrated rows after two restarts'):
            self.assertEqual(self._confirmed_at('LEGACY_PREPASSED'), 0)
            self.assertEqual(self._confirmed_at('LEGACY_CONFIRMED'), LEGACY_CONFIRMED_AT_NS)
            self.assertEqual(self._confirmed_at('LEGACY_HEALED'), LEGACY_CONFIRMED_AT_NS)
        with self.subTest(case='reactivated, restarted, not yet confirmed'):
            self.assertEqual(self._confirmed_at('CONF_AT'), 0)
            self.assertEqual(self._confirmed_at('LEGACY_CONF_AT'), 0)
        self._report('LEGACY_CONF_AT', Fault.SEVERITY_ERROR)
        self._report('LEGACY_CONF_AT', Fault.SEVERITY_ERROR)
        self.assertEqual(self._fault('LEGACY_CONF_AT').status, Fault.STATUS_CONFIRMED)
        with self.subTest(case='migrated record confirms its new occurrence'):
            self.assertGreater(self._confirmed_at('LEGACY_CONF_AT'), LEGACY_CONFIRMED_AT_NS)
        self._report('CONF_AT', Fault.SEVERITY_ERROR)
        self._report('CONF_AT', Fault.SEVERITY_ERROR)
        fault = self._fault('CONF_AT')
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        started = fault.first_occurred
        occurrence_start_ns = started.sec * 1_000_000_000 + started.nanosec
        with self.subTest(case='confirmation time belongs to this occurrence'):
            self.assertGreater(self._confirmed_at('CONF_AT'), occurrence_start_ns)
            self.assertGreater(occurrence_start_ns, first_confirmed_at)

        for _ in range(10):
            self._report('SEV_CRIT', Fault.SEVERITY_WARN)
        with self.subTest(case='the bound restarts with the process'):
            self.assertEqual(self._mismatch_lines(proc_output), 2)


@launch_testing.post_shutdown_test()
class TestSeverityConsistencyShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        shutil.rmtree(DB_DIR, ignore_errors=True)
        launch_testing.asserts.assertExitCodes(proc_info)
