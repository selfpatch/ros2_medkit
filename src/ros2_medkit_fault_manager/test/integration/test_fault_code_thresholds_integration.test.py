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
Integration tests for per-fault_code debounce thresholds.

The node is launched with both layers configured, because the point of the
fault-code layer is what it does to the entity layer underneath it: the debounce
counter belongs to the fault code while an entity override is chosen by the
reporting source, so without this layer two entities reporting one code debounce
it two ways (issue #275). The last case here covers the warning that says so when
no fault-code override settles it (issue #276).
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetFault, ReportFault

# The node's own wording, so a reworded warning fails here rather than going quiet.
CONFLICT_WARNING = 'is debounced two ways'

# Entities whose debounce policies differ, from test_entity_thresholds.yaml.
LIDAR = '/sensors/lidar/front'
MOTOR = '/powertrain/motor/left'


def _output_text(proc_output, process):
    """
    Return everything the process has written so far, as one string.

    Concatenated with NO separator: proc_output yields raw stream chunks, not
    lines, so joining on a newline would splice one into the middle of a log
    line and break a substring match on a message that is plainly there.
    """
    return ''.join(
        output.text.decode(errors='replace') for output in proc_output[process]
    )


def generate_test_description():
    """Launch fault_manager with both the entity and fault-code layers."""
    pkg_share = get_package_share_directory('ros2_medkit_fault_manager')
    entity_config = os.path.join(
        pkg_share, 'test', 'test_entity_thresholds.yaml'
    )
    fault_config = os.path.join(pkg_share, 'test', 'test_fault_thresholds.yaml')

    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            # Global: 5 events. Neither entity nor fault code inherits it below,
            # so a case that passes by falling through to the global would fail.
            'confirmation_threshold': -5,
            'healing_enabled': False,
            'healing_threshold': 10,
            'entity_thresholds.config_file': entity_config,
            'fault_thresholds.config_file': fault_config,
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


class TestPerFaultCodeThresholds(unittest.TestCase):
    """Integration tests for per-fault_code debounce thresholds."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context and service clients."""
        rclpy.init()
        cls.node = Node('test_fault_code_thresholds_client')

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

    def _report(self, fault_code, source_id):
        """Report a FAILED event."""
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = f'Test fault from {source_id}'
        req.source_id = source_id
        resp = self._call(self.report_client, req)
        self.assertTrue(resp.accepted)

    def _status(self, fault_code):
        """Get a fault's current status."""
        req = GetFault.Request()
        req.fault_code = fault_code
        resp = self._call(self.get_client, req)
        self.assertTrue(resp.success, resp.error_message)
        return resp.fault.status

    # @verifies REQ_INTEROP_107
    def test_01_fault_code_beats_the_entity_that_reports_it(self):
        """
        Pin SHARED.OVERHEAT to -3, over the lidar entity's -1.

        Lidar confirms on the first event for any other code, so a single
        PREFAILED here is only possible if the fault-code layer was applied on
        top of the entity one.
        """
        self._report('SHARED.OVERHEAT', LIDAR)
        self.assertEqual(self._status('SHARED.OVERHEAT'), Fault.STATUS_PREFAILED)

        self._report('SHARED.OVERHEAT', LIDAR)
        self.assertEqual(self._status('SHARED.OVERHEAT'), Fault.STATUS_PREFAILED)

        self._report('SHARED.OVERHEAT', LIDAR)
        self.assertEqual(self._status('SHARED.OVERHEAT'), Fault.STATUS_CONFIRMED)

    # @verifies REQ_INTEROP_107
    def test_02_one_code_debounces_alike_from_two_entities(self):
        """
        Confirm SHARED.JAM on its own fourth event, whoever reported it.

        Alternating the two sources is the case issue #275 is about: under the
        entity layer alone lidar's -1 would confirm this on the second event,
        bypassing the motor's policy.
        """
        self._report('SHARED.JAM', MOTOR)
        self.assertEqual(self._status('SHARED.JAM'), Fault.STATUS_PREFAILED)

        self._report('SHARED.JAM', LIDAR)
        self.assertEqual(self._status('SHARED.JAM'), Fault.STATUS_PREFAILED)

        self._report('SHARED.JAM', MOTOR)
        self.assertEqual(self._status('SHARED.JAM'), Fault.STATUS_PREFAILED)

        self._report('SHARED.JAM', LIDAR)
        self.assertEqual(self._status('SHARED.JAM'), Fault.STATUS_CONFIRMED)

    # @verifies REQ_INTEROP_107
    def test_03_a_code_with_no_override_keeps_the_entity_policy(self):
        """The new layer is opt-in: an unlisted code still follows its entity."""
        self._report('MOTOR.ONLY', MOTOR)
        self.assertEqual(self._status('MOTOR.ONLY'), Fault.STATUS_PREFAILED)

        self._report('LIDAR.ONLY', LIDAR)
        self.assertEqual(self._status('LIDAR.ONLY'), Fault.STATUS_CONFIRMED)

    # @verifies REQ_INTEROP_107, REQ_INTEROP_108
    def test_04_a_half_pinned_code_is_still_reported(self, proc_output,
                                                     fault_manager_node):
        """
        Report SHARED.PARTIAL, which pins confirmation but not healing.

        The override settles the direction it names and no more: the two sources
        heal this code under thresholds 1 and 5. Warning on the whole policy
        rather than on the presence of an override is what keeps that visible.
        """
        self._report('SHARED.PARTIAL', LIDAR)
        self.assertEqual(self._status('SHARED.PARTIAL'), Fault.STATUS_PREFAILED)

        self._report('SHARED.PARTIAL', MOTOR)
        self.assertEqual(self._status('SHARED.PARTIAL'), Fault.STATUS_CONFIRMED)

        proc_output.assertWaitFor(
            f"Fault code 'SHARED.PARTIAL' {CONFLICT_WARNING}",
            process=fault_manager_node,
            timeout=10.0,
        )

    # @verifies REQ_INTEROP_108
    def test_05_conflicting_policies_are_reported_once(self, proc_output,
                                                       fault_manager_node):
        """
        Warn on two entities reporting one unlisted code, and warn once.

        Without the warning the bypass is invisible - the fault confirms under
        whichever policy happened to report, and nothing in the log distinguishes
        that from the policy the operator configured.
        """
        self._report('CONFLICTING.CODE', LIDAR)
        self._report('CONFLICTING.CODE', MOTOR)

        proc_output.assertWaitFor(
            f"Fault code 'CONFLICTING.CODE' {CONFLICT_WARNING}",
            process=fault_manager_node,
            timeout=10.0,
        )

        # A warning per report would be a warning per fault event on a busy
        # robot, which is why the node keeps a witness and warns once per code.
        self._report('CONFLICTING.CODE', LIDAR)
        self._report('CONFLICTING.CODE', MOTOR)
        text = _output_text(proc_output, fault_manager_node)
        self.assertEqual(text.count("Fault code 'CONFLICTING.CODE'"), 1)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify fault_manager exits cleanly and stayed quiet where it should."""

    # @verifies REQ_INTEROP_108
    def test_an_overridden_code_never_warns(self, proc_output,
                                            fault_manager_node):
        """
        Stay quiet about a code whose override settles it for both sources.

        SHARED.JAM had two sources and no conflict: a fault-code override
        resolves the same for both, so there is nothing to warn about. Checked
        after shutdown, when the whole output is in hand - asserting an absence
        against a stream still being written proves nothing.
        """
        text = _output_text(proc_output, fault_manager_node)
        self.assertNotIn("Fault code 'SHARED.JAM'", text)

        # And a code only one source ever reports has nothing to conflict with.
        # A witness compared against the global config instead of against the
        # first report would warn here, on every ordinary single-reporter robot.
        self.assertNotIn("Fault code 'SHARED.OVERHEAT'", text)
        self.assertNotIn("Fault code 'MOTOR.ONLY'", text)
        self.assertNotIn("Fault code 'LIDAR.ONLY'", text)

    def test_exit_code(self, proc_info):
        """Check process exit code."""
        launch_testing.asserts.assertExitCodes(proc_info)
