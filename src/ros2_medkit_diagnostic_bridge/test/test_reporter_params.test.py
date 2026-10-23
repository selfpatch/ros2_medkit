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
Out-of-range fault_reporter parameters on a real bridge and fault manager.

The bridge owns a FaultReporter. It is started with a default_threshold above
INT_MAX and a bypass_severity above 255. Both fall back to their defaults with
a warning. A narrowed threshold (2^32+1 -> 1) or bypass_severity (256 -> 0) would
let a WARN fault through at once; the defaults filter it until the third report.
"""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.srv import ListFaults


def generate_test_description():
    """Launch a fault_manager and a bridge with out-of-range reporter parameters."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
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
            'fault_reporter.local_filtering.default_threshold': 4294967297,
            'fault_reporter.local_filtering.bypass_severity': 256,
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


class TestReporterParams(unittest.TestCase):
    """Out-of-range reporter parameters fall back to defaults."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context, publisher and service client."""
        rclpy.init()
        cls.node = Node('test_reporter_params_client')
        cls.diag_pub = cls.node.create_publisher(DiagnosticArray, '/diagnostics', 10)
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults')
        assert cls.list_faults_client.wait_for_service(timeout_sec=10.0), \
            'ListFaults service not available'

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

    def _publish_warn(self, name):
        msg = DiagnosticArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(
            level=DiagnosticStatus.WARN, name=name, message='warn', hardware_id='param_hw')]
        self.diag_pub.publish(msg)
        # Let the bridge and the fault manager process it.
        end = time.monotonic() + 0.5
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _has_fault(self, code):
        request = ListFaults.Request()
        request.filter_by_severity = False
        future = self.list_faults_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        result = future.result()
        self.assertIsNotNone(result, 'ListFaults timed out: is the fault manager still up?')
        return any(f.fault_code == code for f in result.faults)

    def test_out_of_range_parameters_warn_and_keep_defaults(
            self, proc_output, diagnostic_bridge_node):
        """Warn for both values, and keep threshold 3 for a WARN fault."""
        # The reporter is built on the first diagnostic.
        self._publish_warn('param_probe')
        self._publish_warn('param_probe')

        proc_output.assertWaitFor(
            'default_threshold 4294967297 is above 2147483647, using 3',
            process=diagnostic_bridge_node, timeout=15)
        proc_output.assertWaitFor(
            'bypass_severity 256 is outside [0, 255], using 2',
            process=diagnostic_bridge_node, timeout=15)

        self.assertFalse(
            self._has_fault('PARAM_PROBE'),
            'two WARN reports reached the fault manager: threshold or bypass wrapped')

        # Exactly one more report: the third one reaches the threshold of 3.
        self._publish_warn('param_probe')
        deadline = time.monotonic() + 10.0
        while not self._has_fault('PARAM_PROBE'):
            self.assertLess(time.monotonic(), deadline,
                            'the third WARN report never reached the fault manager')
            time.sleep(0.2)


@launch_testing.post_shutdown_test()
class TestReporterParamsShutdown(unittest.TestCase):
    """Exit codes after shutdown."""

    def test_exit_code(self, proc_info):
        """All processes exit cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
