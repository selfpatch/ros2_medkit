#!/usr/bin/env python3
# Copyright 2026 selfpatch
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
The numbers behind a fault reach the fault record and stay there.

A reporter that already computed why it is unhappy - outlier counts, gate
statistics, a covariance that went negative - can attach them to the report. The
fault manager keeps them in their own table, per occurrence, and ``GetFault``
serves them in the freeze frame under ``"x-medkit": {"reported": ...}``, which
is what the gateway's ``GET /apps/{app}/faults/{code}`` serializes.

Driven through the ReportFault service rather than through the bridge on
purpose: this pins the storage and serving contract every reporter depends on,
not one caller's use of it.

CAPTURED_CODE has a capture config and a publisher on its topic, so the
confirmation capture runs next to the evidence.
"""

import json
import os
import tempfile
import threading
import time
import unittest

from diagnostic_msgs.msg import KeyValue
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault, Snapshot
from ros2_medkit_msgs.srv import ClearFault, GetFault, ReportFault
from sensor_msgs.msg import Temperature

FAULT_CODE = 'FUSION_DIVERGED'
SOURCE_ID = '/sensor_fusion'
OTHER_SOURCE_ID = '/gnss_monitor'

CAPTURED_CODE = 'CAPTURED_EVIDENCE'
CAPTURED_TOPIC = '/evidence_test/temperature'

_CAPTURE_CONFIG = tempfile.NamedTemporaryFile(
    'w', suffix='.yaml', prefix='evidence_snapshots_', delete=False)
_CAPTURE_CONFIG.write(f'fault_specific:\n  {CAPTURED_CODE}:\n    - {CAPTURED_TOPIC}\n')
_CAPTURE_CONFIG.close()


def generate_test_description():
    """Launch a fault_manager with in-memory storage and one capture config."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            # Deeper than one event, so the fault stays PREFAILED while the
            # first reports land: evidence must be kept before anything
            # confirms, not only at confirmation time.
            'confirmation_threshold': -3,
            'healing_enabled': False,
            'snapshots.enabled': True,
            'snapshots.config_file': _CAPTURE_CONFIG.name,
            'snapshots.timeout_sec': 3.0,
            'snapshots.background_capture': False,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            fault_manager_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
        },
    )


class TestReportedEvidence(unittest.TestCase):
    """Evidence attached to a report survives into the served fault."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context, service clients and the capture publisher."""
        rclpy.init()
        cls.node = Node('test_reported_evidence_client')

        cls.report_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.get_client = cls.node.create_client(
            GetFault, '/fault_manager/get_fault'
        )
        cls.clear_client = cls.node.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )
        cls.temp_publisher = cls.node.create_publisher(Temperature, CAPTURED_TOPIC, 10)

        assert cls.report_client.wait_for_service(timeout_sec=10.0), \
            'report_fault service not available'
        assert cls.get_client.wait_for_service(timeout_sec=10.0), \
            'get_fault service not available'
        assert cls.clear_client.wait_for_service(timeout_sec=10.0), \
            'clear_fault service not available'

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        cls.node.destroy_node()
        rclpy.shutdown()
        os.unlink(_CAPTURE_CONFIG.name)

    def _call(self, client, request):
        """Call service synchronously."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report(self, fault_code, evidence=None, *, source_id=SOURCE_ID,
                severity=Fault.SEVERITY_ERROR):
        """Report a FAILED event, optionally carrying evidence."""
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = severity
        req.description = 'fusion diverged'
        req.source_id = source_id
        for key, value in (evidence or {}).items():
            req.evidence.append(KeyValue(key=key, value=value))
        resp = self._call(self.report_client, req)
        self.assertTrue(resp.accepted)

    def _get(self, fault_code):
        req = GetFault.Request()
        req.fault_code = fault_code
        resp = self._call(self.get_client, req)
        self.assertTrue(resp.success, resp.error_message)
        return resp

    def _freeze_frame(self, fault_code):
        """Return the parsed freeze-frame document served for a fault, or None."""
        frames = [
            s for s in self._get(fault_code).environment_data.snapshots
            if s.type == Snapshot.TYPE_FREEZE_FRAME and s.name == 'freeze_frame'
        ]
        self.assertLessEqual(len(frames), 1, 'more than one freeze frame served')
        return json.loads(frames[0].data) if frames else None

    def _reported(self, fault_code, source_id=SOURCE_ID):
        """Return {key: value} of the evidence served for one source."""
        frame = self._freeze_frame(fault_code)
        self.assertIsNotNone(frame, 'no freeze frame served')
        entries = frame['x-medkit']['reported'][source_id]
        for entry in entries.values():
            self.assertIn('reported_at', entry)
        return {key: entry['value'] for key, entry in entries.items()}

    def test_01_evidence_reaches_the_served_fault(self):
        """An ERROR report carrying rejected_fixes=37 and nis=0.03 serves both back."""
        self._report(FAULT_CODE, {'rejected_fixes': '37', 'nis': '0.03'})

        self.assertEqual(self._reported(FAULT_CODE), {'rejected_fixes': '37', 'nis': '0.03'})

    def test_02_evidence_is_kept_before_the_fault_confirms(self):
        """The fault is still PREFAILED and the numbers are already stored."""
        self.assertEqual(self._get(FAULT_CODE).fault.status, Fault.STATUS_PREFAILED)

    def test_03_a_later_report_updates_keys_and_keeps_the_rest(self):
        """A second report refreshes what it names and leaves the rest."""
        self._report(FAULT_CODE, {'nis': '0.31'})

        self.assertEqual(self._reported(FAULT_CODE), {'rejected_fixes': '37', 'nis': '0.31'})

    def test_04_a_report_without_evidence_does_not_erase_it(self):
        """A plain report leaves the stored numbers alone."""
        self._report(FAULT_CODE)

        self.assertEqual(self._reported(FAULT_CODE), {'rejected_fixes': '37', 'nis': '0.31'})

    def test_05_another_source_keeps_its_own_values(self):
        """Two sources of one fault do not overwrite each other's keys."""
        self._report(FAULT_CODE, {'nis': '9.9'}, source_id=OTHER_SOURCE_ID)

        self.assertEqual(self._reported(FAULT_CODE), {'rejected_fixes': '37', 'nis': '0.31'})
        self.assertEqual(self._reported(FAULT_CODE, OTHER_SOURCE_ID), {'nis': '9.9'})

    def test_06_a_code_never_carrying_evidence_gets_no_frame(self):
        """No capture config and no evidence: no freeze frame at all."""
        self._report('PLAIN_FAULT')

        self.assertIsNone(self._freeze_frame('PLAIN_FAULT'))

    def test_07_evidence_survives_the_confirmation_capture(self):
        """
        A confirmation capture runs next to the evidence and does not erase it.

        With per-topic snapshots present, the frame carries only the evidence:
        the topic values are already in the snapshots.
        """
        stop = threading.Event()

        def keep_publishing():
            msg = Temperature(temperature=85.5, variance=0.1)
            while not stop.is_set():
                self.temp_publisher.publish(msg)
                time.sleep(0.05)

        publisher = threading.Thread(target=keep_publishing)
        publisher.start()
        try:
            # CRITICAL confirms on the first report, which starts the capture.
            self._report(CAPTURED_CODE, {'nis': '0.03'}, severity=Fault.SEVERITY_CRITICAL)
            deadline = time.monotonic() + 15.0
            topic_snapshots = []
            while not topic_snapshots and time.monotonic() < deadline:
                topic_snapshots = [
                    s for s in self._get(CAPTURED_CODE).environment_data.snapshots
                    if s.topic == CAPTURED_TOPIC
                ]
                if not topic_snapshots:
                    time.sleep(0.5)
        finally:
            stop.set()
            publisher.join()
        self.assertTrue(topic_snapshots, 'the confirmation capture took nothing')

        frame = self._freeze_frame(CAPTURED_CODE)
        self.assertIsNotNone(frame)
        self.assertEqual(set(frame), {'x-medkit'}, 'topic values served twice')
        self.assertEqual(self._reported(CAPTURED_CODE), {'nis': '0.03'})

    def test_08_a_new_occurrence_starts_from_nothing(self):
        """After clear_fault, the next FAILED report starts a new set of evidence."""
        clear = ClearFault.Request()
        clear.fault_code = CAPTURED_CODE
        self.assertTrue(self._call(self.clear_client, clear).success)
        # The cleared occurrence still shows its evidence.
        self.assertEqual(self._reported(CAPTURED_CODE), {'nis': '0.03'})

        self._report(CAPTURED_CODE, {'gate': '9.2'})

        self.assertEqual(self._get(CAPTURED_CODE).fault.occurrence_count, 2)
        self.assertEqual(self._reported(CAPTURED_CODE), {'gate': '9.2'})


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify fault_manager exits cleanly."""

    def test_exit_code(self, proc_info):
        """Check process exit code."""
        launch_testing.asserts.assertExitCodes(proc_info)
