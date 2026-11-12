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
Record identity per fault code, through a real fault manager.

A code listed with per_source: true in fault_thresholds.yaml gets one record per
reporting source. Every other code, listed or not, has one record shared by all
its sources, with one debounce counter. The shared record is owned by the
reserved id '*', which no reporter may use.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import ClearFault, GetFault, ListFaults, ReportFault

TIMEOUT_SEC = 30.0
ALL_STATUSES = [
    Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED, Fault.STATUS_CONFIRMED,
    Fault.STATUS_HEALED, Fault.STATUS_CLEARED,
]

# Codes with an entry: per_source absent, false and true. A code with no entry is shared too.
THRESHOLDS = """\
ID.ABSENT:
  confirmation_threshold: -3
ID.FALSE:
  per_source: false
  confirmation_threshold: -3
ID.PER:
  per_source: true
  confirmation_threshold: -1
  healing_enabled: true
  healing_threshold: 1
ID.PER_LIST:
  per_source: true
  confirmation_threshold: -1
ID.CRIT_SHARED:
  confirmation_threshold: -3
ID.CRIT_PER:
  per_source: true
  confirmation_threshold: -3
"""

THRESHOLDS_FILE = tempfile.NamedTemporaryFile(
    'w', suffix='.yaml', prefix='fault_identity_', delete=False)
THRESHOLDS_FILE.write(THRESHOLDS)
THRESHOLDS_FILE.close()


def generate_test_description():
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'healing_enabled': False,
            'fault_thresholds.config_file': THRESHOLDS_FILE.name,
            'snapshots.enabled': False,
            'snapshots.rosbag.enabled': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([fault_manager_node, launch_testing.actions.ReadyToTest()]),
        {'fault_manager_node': fault_manager_node},
    )


class TestFaultIdentity(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_fault_identity_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.get_client = cls.node.create_client(GetFault, '/fault_manager/get_fault')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        cls.events = []
        cls.event_sub = cls.node.create_subscription(
            FaultEvent, '/fault_manager/events', cls.events.append,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        for client in (cls.report_client, cls.list_client, cls.get_client, cls.clear_client):
            assert client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        deadline = time.monotonic() + TIMEOUT_SEC
        while cls.event_sub.get_publisher_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        assert cls.event_sub.get_publisher_count() > 0

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), 'service call timed out')
        return future.result()

    def _report(self, code, source, event_type=ReportFault.Request.EVENT_FAILED,
                severity=Fault.SEVERITY_ERROR):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = event_type
        request.severity = severity
        request.description = f'{code} from {source}'
        request.source_id = source
        return self._call(self.report_client, request).accepted

    def _get(self, code, source=''):
        request = GetFault.Request()
        request.fault_code = code
        request.source_id = source
        return self._call(self.get_client, request)

    def _clear(self, code, source=''):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        return self._call(self.clear_client, request)

    def _records(self, code):
        request = ListFaults.Request()
        request.statuses = ALL_STATUSES
        return [f for f in self._call(self.list_client, request).faults if f.fault_code == code]

    def test_default_code_has_one_counter_for_all_sources(self):
        """Three sources, one FAILED each: one record that confirms on the third."""
        sources = ['/debounce/a', '/debounce/b', '/debounce/c']
        expected = [Fault.STATUS_PREFAILED, Fault.STATUS_PREFAILED, Fault.STATUS_CONFIRMED]
        for code in ('ID.ABSENT', 'ID.FALSE'):
            with self.subTest(code=code):
                for source, status in zip(sources, expected):
                    self.assertTrue(self._report(code, source))
                    records = self._records(code)
                    self.assertEqual(len(records), 1)
                    self.assertEqual(records[0].status, status)
                self.assertEqual(list(records[0].reporting_sources), sources)

    def test_default_code_is_one_record_that_any_source_reaches(self):
        """Any reporting source and the unscoped call reach the one record; a clear ends it."""
        code = 'ID.PLAIN'
        sources = ['/plain/a', '/plain/b', '/plain/c']
        for source in sources:
            self.assertTrue(self._report(code, source))

        for source in ['', '*'] + sources:
            with self.subTest(get_source=source):
                response = self._get(code, source)
                self.assertTrue(response.success, response.error_message)
                self.assertEqual(response.fault.status, Fault.STATUS_CONFIRMED)
                self.assertEqual(list(response.fault.reporting_sources), sources)
                self.assertEqual(response.fault.occurrence_count, 1)

        cleared = self._clear(code, '/plain/b')
        self.assertTrue(cleared.success, cleared.message)
        self.assertEqual(self._get(code).fault.status, Fault.STATUS_CLEARED)
        self.assertEqual(len(self._records(code)), 1)

        # A later FAILED reactivates the same record as a new occurrence.
        self.assertTrue(self._report(code, '/plain/a'))
        response = self._get(code)
        self.assertEqual(response.fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(response.fault.occurrence_count, 2)

        unscoped = self._clear(code)
        self.assertTrue(unscoped.success, unscoped.message)
        self.assertEqual(self._get(code).fault.status, Fault.STATUS_CLEARED)

    def test_per_source_code_has_a_record_per_source(self):
        """Three records; an unscoped call is refused; clear and heal touch one record."""
        code = 'ID.PER'
        sources = ['/per/a', '/per/b', '/per/c']
        for source in sources:
            self.assertTrue(self._report(code, source))

        records = self._records(code)
        owners = sorted(list(r.reporting_sources) for r in records)
        self.assertEqual(owners, [[s] for s in sources])
        self.assertTrue(all(r.status == Fault.STATUS_CONFIRMED for r in records))

        ambiguous_get = self._get(code)
        self.assertFalse(ambiguous_get.success)
        self.assertTrue(ambiguous_get.error_message.startswith('ambiguous:'),
                        ambiguous_get.error_message)
        ambiguous_clear = self._clear(code)
        self.assertFalse(ambiguous_clear.success)
        self.assertTrue(ambiguous_clear.message.startswith('ambiguous:'), ambiguous_clear.message)

        # A clear of /per/a leaves /per/b and /per/c CONFIRMED.
        self.assertTrue(self._clear(code, '/per/a').success)
        self.assertEqual(self._get(code, '/per/a').fault.status, Fault.STATUS_CLEARED)
        self.assertEqual(self._get(code, '/per/b').fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(self._get(code, '/per/c').fault.status, Fault.STATUS_CONFIRMED)

        # Two PASSED from /per/b heal /per/b only.
        for _ in range(2):
            self.assertTrue(self._report(code, '/per/b', ReportFault.Request.EVENT_PASSED))
        self.assertEqual(self._get(code, '/per/b').fault.status, Fault.STATUS_HEALED)
        self.assertEqual(self._get(code, '/per/c').fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(self._get(code, '/per/a').fault.status, Fault.STATUS_CLEARED)

    def test_default_and_per_source_codes_side_by_side(self):
        """The same three sources: one record for a default code, three for a per_source one."""
        sources = ['/side/a', '/side/b', '/side/c']
        for source in sources:
            self.assertTrue(self._report('ID.PLAIN_LIST', source))
            self.assertTrue(self._report('ID.PER_LIST', source))
        self.assertEqual(len(self._records('ID.PLAIN_LIST')), 1)
        self.assertEqual(len(self._records('ID.PER_LIST')), 3)

    def _confirmed_events(self, code):
        return [e for e in self.events
                if e.fault.fault_code == code and e.event_type == FaultEvent.EVENT_CONFIRMED]

    def _wait_for_confirmed_event(self, code):
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline and not self._confirmed_events(code):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertTrue(self._confirmed_events(code), f'no fault_confirmed event for {code}')

    def _critical_clear_warn(self, code, first_source, second_source):
        """CRITICAL, clear the record, then a WARN: the sequence that tells the two rules apart."""
        self.assertTrue(self._report(code, first_source, severity=Fault.SEVERITY_CRITICAL))
        self._wait_for_confirmed_event(code)
        cleared = self._clear(code)
        self.assertTrue(cleared.success, cleared.message)
        self.assertTrue(self._report(code, second_source, severity=Fault.SEVERITY_WARN))
        # The events arrive in order, so once a marker's confirmation is here, any event the WARN
        # report produced is too.
        marker = f'{code}.MARKER'
        self.assertTrue(self._report(marker, '/crit/marker', severity=Fault.SEVERITY_CRITICAL))
        self._wait_for_confirmed_event(marker)

    def test_a_shared_record_confirms_on_the_severity_of_the_report(self):
        """CRITICAL from /a, clear, WARN from /b on a default code stays pending."""
        code = 'ID.CRIT_SHARED'
        self._critical_clear_warn(code, '/crit/a', '/crit/b')
        record = self._get(code).fault
        self.assertEqual(record.status, Fault.STATUS_PREFAILED)
        self.assertEqual(record.occurrence_count, 2)
        request = ListFaults.Request()
        listed = self._call(self.list_client, request).faults
        self.assertEqual([f for f in listed if f.fault_code == code], [])
        self.assertEqual(len(self._confirmed_events(code)), 1)

    def test_a_per_source_record_confirms_on_its_stored_severity(self):
        """CRITICAL from /a, clear, WARN from /a on a per_source code confirms at once."""
        code = 'ID.CRIT_PER'
        self._critical_clear_warn(code, '/crit/a', '/crit/a')
        record = self._get(code).fault
        self.assertEqual(record.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(record.severity, Fault.SEVERITY_CRITICAL)
        self.assertEqual(record.occurrence_count, 2)
        self.assertEqual(len(self._confirmed_events(code)), 2)

    def test_the_shared_owner_is_not_a_source_id(self):
        """A report claiming the reserved owner is refused and creates nothing."""
        for code in ('ID.RESERVED_SHARED', 'ID.PER'):
            with self.subTest(code=code):
                before = len(self._records(code))
                self.assertFalse(self._report(code, '*'))
                self.assertEqual(len(self._records(code)), before)


@launch_testing.post_shutdown_test()
class TestFaultIdentityShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE.name)
