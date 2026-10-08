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
Bridge and fault manager together, with confirmation_threshold: -3.

test_integration uses -1, where every report confirms at once, so it cannot
show that a STALE override is debounced. This file checks:

- STALE with an override is debounced; STALE without one confirms CRITICAL.
- The values of an ERROR status come back from GetFault.
- A value that is not valid UTF-8 does not crash the fault manager.
"""

import json
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from ros2_medkit_msgs.msg import Fault, Snapshot
from ros2_medkit_msgs.srv import GetFault, ListFaults

REPORTED_KEY = 'x-medkit-reported'

# Replaced in the serialized message. Same length as the new bytes, so the
# string length in the message stays correct.
UTF8_PLACEHOLDER = b'80#C'
LATIN1_DEGREES = b'80\xb0C'


def generate_test_description():
    """Launch a debouncing fault_manager and a bridge with one STALE override."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -3,
            'healing_enabled': False,
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    diagnostic_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_diagnostic_bridge',
        executable='diagnostic_bridge_node',
        name='diagnostic_bridge',
        output='screen',
        parameters=[{
            'diagnostics_topic': '/diagnostics',
            'auto_generate_codes': True,
            'stale_severity_overrides.planned_gps': 'WARN',
        }],
        # Give the node room to flush coverage data at shutdown before SIGKILL.
        sigterm_timeout='30',
        sigkill_timeout='15',
    )

    return (
        LaunchDescription([
            fault_manager_node,
            diagnostic_bridge_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'diagnostic_bridge_node': diagnostic_bridge_node,
        },
    )


class TestStaleAndEvidence(unittest.TestCase):
    """STALE overrides debounce, and status values survive into the fault."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context, publisher and service clients."""
        rclpy.init()
        cls.node = Node('test_stale_and_evidence_client')
        cls.diag_pub = cls.node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults')
        cls.get_fault_client = cls.node.create_client(
            GetFault, '/fault_manager/get_fault')

        assert cls.list_faults_client.wait_for_service(timeout_sec=10.0), \
            'ListFaults service not available'
        assert cls.get_fault_client.wait_for_service(timeout_sec=10.0), \
            'GetFault service not available'

        # Messages sent before the bridge subscribes are lost, so wait for it.
        deadline = time.monotonic() + 15.0
        while cls.diag_pub.get_subscription_count() < 1:
            assert time.monotonic() < deadline, (
                'diagnostic_bridge did not subscribe to /diagnostics')
            rclpy.spin_once(cls.node, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result()

    def _find_fault(self, code):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.statuses = [Fault.STATUS_PREFAILED, Fault.STATUS_CONFIRMED]
        result = self._call(self.list_faults_client, request)
        faults = result.faults if result is not None else []
        return next((f for f in faults if f.fault_code == code), None)

    def _reported_evidence(self, code):
        """Return the evidence object served by GetFault, or None."""
        request = GetFault.Request()
        request.fault_code = code
        result = self._call(self.get_fault_client, request)
        self.assertIsNotNone(result, 'GetFault timed out: is the fault manager still up?')
        if not result.success:
            return None
        frames = [
            s for s in result.environment_data.snapshots
            if s.type == Snapshot.TYPE_FREEZE_FRAME and s.name == 'freeze_frame'
        ]
        if not frames:
            return None
        return json.loads(frames[0].data).get(REPORTED_KEY)

    def _status(self, name, level, values=()):
        return DiagnosticStatus(
            level=level, name=name, message='test', hardware_id='',
            values=[KeyValue(key=k, value=v) for k, v in values],
        )

    def _publish(self, status):
        msg = DiagnosticArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)
        time.sleep(0.3)

    def _publish_until(self, publish, check, *, timeout=25.0):
        """
        Publish until *check* returns something truthy, and return it.

        The bridge drops reports until its client finds the fault manager, so
        one publish is not enough.
        """
        deadline = time.monotonic() + timeout
        last = None
        while time.monotonic() < deadline:
            publish()
            last = check()
            if last:
                return last
        self.fail(f'Condition not met within {timeout}s (last: {last})')

    def test_01_stale_with_override_is_debounced(self):
        """STALE overridden to WARN is debounced instead of confirming at once."""
        status = self._status('planned_gps', DiagnosticStatus.STALE)

        first = self._publish_until(
            lambda: self._publish(status), lambda: self._find_fault('PLANNED_GPS'))
        self.assertEqual(first.severity, Fault.SEVERITY_WARN)
        self.assertEqual(
            first.status, Fault.STATUS_PREFAILED,
            'a STALE status overridden to WARN confirmed on its first report')

        # If it keeps coming, it still confirms.
        def confirmed_fault():
            fault = self._find_fault('PLANNED_GPS')
            return fault if fault and fault.status == Fault.STATUS_CONFIRMED else None

        confirmed = self._publish_until(lambda: self._publish(status), confirmed_fault)
        self.assertEqual(confirmed.severity, Fault.SEVERITY_WARN)

    def test_02_stale_without_override_still_confirms_critical(self):
        """STALE without an override is still CRITICAL and confirms at once."""
        status = self._status('unplanned_lidar', DiagnosticStatus.STALE)

        first = self._publish_until(
            lambda: self._publish(status), lambda: self._find_fault('UNPLANNED_LIDAR'))
        self.assertEqual(first.severity, Fault.SEVERITY_CRITICAL)
        self.assertEqual(first.status, Fault.STATUS_CONFIRMED)

    def test_03_error_values_are_served_by_get_fault(self):
        """The values of an ERROR status come back from GetFault."""
        status = self._status(
            'fusion_monitor', DiagnosticStatus.ERROR,
            values=[('rejected_fixes', '37'), ('nis', '0.03')])

        reported = self._publish_until(
            lambda: self._publish(status), lambda: self._reported_evidence('FUSION_MONITOR'))
        self.assertEqual(reported, {'rejected_fixes': '37', 'nis': '0.03'})

    def test_04_invalid_utf8_value_does_not_crash_the_fault_manager(self):
        """A Latin-1 byte in a value is stored as U+FFFD and nothing crashes."""
        status = self._status(
            'thermal_probe', DiagnosticStatus.ERROR,
            values=[('temp', UTF8_PLACEHOLDER.decode())])
        msg = DiagnosticArray()
        msg.status = [status]
        raw = serialize_message(msg)
        self.assertEqual(raw.count(UTF8_PLACEHOLDER), 1)
        # rclpy cannot put invalid UTF-8 in a str field, so patch the bytes.
        raw = raw.replace(UTF8_PLACEHOLDER, LATIN1_DEGREES)

        def publish_raw():
            self.diag_pub.publish(raw)
            time.sleep(0.3)

        reported = self._publish_until(
            publish_raw, lambda: self._reported_evidence('THERMAL_PROBE'))
        self.assertEqual(reported, {'temp': '80�C'})

        # Send it a few more times, then check the fault manager still answers.
        for _ in range(5):
            publish_raw()
        self.assertEqual(self._reported_evidence('THERMAL_PROBE'), {'temp': '80�C'})


@launch_testing.post_shutdown_test()
class TestStaleAndEvidenceShutdown(unittest.TestCase):
    """Check that no node crashed."""

    def test_exit_code(self, proc_info):
        """Verify nodes exit cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
