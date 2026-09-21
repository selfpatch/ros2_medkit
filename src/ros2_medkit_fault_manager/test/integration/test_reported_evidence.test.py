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
fault manager keeps them in the fault's freeze frame, and ``GetFault`` serves
them back, which is what the gateway's ``GET /apps/{app}/faults/{code}``
serializes. Without this the fault record says a node complained and not what it
saw.

Driven through the ReportFault service rather than through the bridge on
purpose: this pins the storage and serving contract every reporter depends on,
not one caller's use of it.
"""

import json
import unittest

from diagnostic_msgs.msg import KeyValue
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault, Snapshot
from ros2_medkit_msgs.srv import GetFault, ReportFault

# The reserved key inside the freeze-frame document. Topic keys are fully
# qualified ROS names and start with '/', so this cannot collide with one.
REPORTED_KEY = 'x-reported'

FAULT_CODE = 'FUSION_DIVERGED'
SOURCE_ID = '/sensor_fusion'


def generate_test_description():
    """Launch a fault_manager with in-memory storage and no capture config."""
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
        """Initialize ROS 2 context and service clients."""
        rclpy.init()
        cls.node = Node('test_reported_evidence_client')

        cls.report_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.get_client = cls.node.create_client(
            GetFault, '/fault_manager/get_fault'
        )

        assert cls.report_client.wait_for_service(timeout_sec=10.0), \
            'report_fault service not available'
        assert cls.get_client.wait_for_service(timeout_sec=10.0), \
            'get_fault service not available'

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        """Call service synchronously."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report(self, fault_code, evidence=None, event_type=None):
        """Report an event, optionally carrying evidence."""
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = (
            ReportFault.Request.EVENT_FAILED if event_type is None
            else event_type
        )
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'fusion diverged'
        req.source_id = SOURCE_ID
        for key, value in (evidence or {}).items():
            kv = KeyValue()
            kv.key = key
            kv.value = value
            req.evidence.append(kv)
        resp = self._call(self.report_client, req)
        self.assertTrue(resp.accepted)

    def _freeze_frame(self, fault_code):
        """Return the parsed freeze-frame document served for a fault."""
        req = GetFault.Request()
        req.fault_code = fault_code
        resp = self._call(self.get_client, req)
        self.assertTrue(resp.success, resp.error_message)

        frames = [
            s for s in resp.environment_data.snapshots
            if s.type == Snapshot.TYPE_FREEZE_FRAME and s.name == 'freeze_frame'
        ]
        self.assertEqual(
            len(frames), 1,
            f'expected exactly one freeze frame, got {len(frames)}',
        )
        return json.loads(frames[0].data)

    # @verifies REQ_INTEROP_110
    def test_01_evidence_reaches_the_served_fault(self):
        """
        Report the issue's own example and read both numbers back.

        This is the acceptance criterion from #668: an ERROR status carrying
        rejected_fixes=37 and nis=0.03 gives a fault whose snapshot lists both.
        """
        self._report(FAULT_CODE, {'rejected_fixes': '37', 'nis': '0.03'})

        reported = self._freeze_frame(FAULT_CODE)[REPORTED_KEY]
        self.assertEqual(reported['rejected_fixes'], '37')
        self.assertEqual(reported['nis'], '0.03')

    # @verifies REQ_INTEROP_110
    def test_02_evidence_is_kept_before_the_fault_confirms(self):
        """
        The fault is still PREFAILED and the numbers are already stored.

        Evidence written only at confirmation time would lose exactly the
        reports that explain why a fault nearly happened.
        """
        req = GetFault.Request()
        req.fault_code = FAULT_CODE
        resp = self._call(self.get_client, req)
        self.assertEqual(resp.fault.status, Fault.STATUS_PREFAILED)

    # @verifies REQ_INTEROP_110
    def test_03_a_later_report_updates_keys_and_keeps_the_rest(self):
        """A second report refreshes what it names and leaves the rest."""
        self._report(FAULT_CODE, {'nis': '0.31'})

        reported = self._freeze_frame(FAULT_CODE)[REPORTED_KEY]
        self.assertEqual(reported['nis'], '0.31')
        self.assertEqual(reported['rejected_fixes'], '37')

    # @verifies REQ_INTEROP_110
    def test_04_a_report_without_evidence_does_not_erase_it(self):
        """
        A plain report leaves the stored numbers alone.

        Reporters mix the two calls - the same code can be raised by a node
        that has measurements and one that does not - and the second must not
        blank the record the first wrote.
        """
        self._report(FAULT_CODE)

        reported = self._freeze_frame(FAULT_CODE)[REPORTED_KEY]
        self.assertEqual(reported['nis'], '0.31')
        self.assertEqual(reported['rejected_fixes'], '37')

    # @verifies REQ_INTEROP_110
    def test_05_a_code_never_carrying_evidence_gets_no_frame(self):
        """
        Absence still means absence.

        A fault code with no capture configured and no evidence reported must
        not gain a freeze frame - an empty one would claim a capture ran.
        """
        self._report('PLAIN_FAULT')

        req = GetFault.Request()
        req.fault_code = 'PLAIN_FAULT'
        resp = self._call(self.get_client, req)
        self.assertTrue(resp.success, resp.error_message)
        frames = [
            s for s in resp.environment_data.snapshots
            if s.name == 'freeze_frame'
        ]
        self.assertEqual(frames, [])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify fault_manager exits cleanly."""

    def test_exit_code(self, proc_info):
        """Check process exit code."""
        launch_testing.asserts.assertExitCodes(proc_info)
