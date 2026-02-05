#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""Integration tests for ros2_medkit_diagnostic_bridge."""

import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults


def generate_test_description():
    """Generate launch description with fault_manager and diagnostic_bridge nodes."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,  # Immediate confirmation
            'healing_enabled': True,
            'healing_threshold': 1,
        }],
    )

    diagnostic_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_diagnostic_bridge',
        executable='diagnostic_bridge_node',
        name='diagnostic_bridge',
        output='screen',
        parameters=[{
            'diagnostics_topic': '/diagnostics',
            'auto_generate_codes': True,
        }],
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


class TestDiagnosticBridgeIntegration(unittest.TestCase):
    """Integration tests for DiagnosticBridge."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context."""
        rclpy.init()
        cls.node = Node('test_diagnostic_bridge_client')

        # Create publisher for diagnostics
        cls.diag_pub = cls.node.create_publisher(DiagnosticArray, '/diagnostics', 10)

        # Create client for ListFaults service
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults'
        )

        # Wait for services
        assert cls.list_faults_client.wait_for_service(timeout_sec=10.0), \
            'ListFaults service not available'

        # Wait for bridge to connect
        time.sleep(1.0)

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def list_faults(self, statuses=None):
        """Get faults from FaultManager."""
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.statuses = statuses or []

        future = self.list_faults_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result().faults

    def publish_diagnostic(self, name, level, message='Test message'):
        """Publish a single diagnostic message."""
        msg = DiagnosticArray()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(
            level=level,
            name=name,
            message=message,
            hardware_id='test_hw',
        )]
        self.diag_pub.publish(msg)

        # Give time for message to be processed
        time.sleep(0.5)

    def test_01_error_diagnostic_creates_fault(self):
        """Test that ERROR diagnostic creates a fault in FaultManager."""
        self.publish_diagnostic('test_sensor', DiagnosticStatus.ERROR, 'Sensor failure')

        faults = self.list_faults()
        fault_codes = [f.fault_code for f in faults]

        self.assertIn('TEST_SENSOR', fault_codes)
        fault = next(f for f in faults if f.fault_code == 'TEST_SENSOR')
        self.assertEqual(fault.severity, Fault.SEVERITY_ERROR)

    def test_02_warn_diagnostic_creates_fault(self):
        """Test that WARN diagnostic creates a fault."""
        # Send multiple times to bypass FaultReporter's local filtering (default threshold=3)
        for _ in range(3):
            self.publish_diagnostic('warning_component', DiagnosticStatus.WARN, 'Low battery')

        faults = self.list_faults()
        fault_codes = [f.fault_code for f in faults]

        self.assertIn('WARNING_COMPONENT', fault_codes)
        fault = next(f for f in faults if f.fault_code == 'WARNING_COMPONENT')
        self.assertEqual(fault.severity, Fault.SEVERITY_WARN)

    def test_03_stale_diagnostic_creates_critical_fault(self):
        """Test that STALE diagnostic creates a CRITICAL fault."""
        self.publish_diagnostic('stale_sensor', DiagnosticStatus.STALE, 'No data')

        faults = self.list_faults()
        fault_codes = [f.fault_code for f in faults]

        self.assertIn('STALE_SENSOR', fault_codes)
        fault = next(f for f in faults if f.fault_code == 'STALE_SENSOR')
        self.assertEqual(fault.severity, Fault.SEVERITY_CRITICAL)

    def test_04_ok_diagnostic_heals_fault(self):
        """Test that OK diagnostic sends PASSED and heals fault."""
        # First create a fault
        self.publish_diagnostic('healing_test', DiagnosticStatus.ERROR, 'Error')

        faults = self.list_faults()
        self.assertIn('HEALING_TEST', [f.fault_code for f in faults])

        # Send OK multiple times to ensure PASSED event reaches FaultManager
        for _ in range(2):
            self.publish_diagnostic('healing_test', DiagnosticStatus.OK)

        # Check fault is healed (query all statuses to find it)
        faults = self.list_faults(statuses=[Fault.STATUS_CONFIRMED, Fault.STATUS_HEALED])
        fault = next((f for f in faults if f.fault_code == 'HEALING_TEST'), None)
        self.assertIsNotNone(fault, 'HEALING_TEST fault not found')
        # After PASSED events with healing_threshold=1, fault should be HEALED
        self.assertEqual(fault.status, Fault.STATUS_HEALED)

    def test_05_fault_code_generation(self):
        """Test auto-generation of fault codes from diagnostic names."""
        test_cases = [
            ('motor temp', 'MOTOR_TEMP'),
            ('sensor: status', 'SENSOR_STATUS'),
            ('/robot/arm', 'ROBOT_ARM'),
        ]

        for diag_name, expected_code in test_cases:
            self.publish_diagnostic(diag_name, DiagnosticStatus.ERROR)

            faults = self.list_faults()
            fault_codes = [f.fault_code for f in faults]

            self.assertIn(expected_code, fault_codes,
                          f'Expected {expected_code} from diagnostic "{diag_name}"')


@launch_testing.post_shutdown_test()
class TestDiagnosticBridgeShutdown(unittest.TestCase):
    """Test clean shutdown."""

    def test_exit_code(self, proc_info):
        """Verify nodes exit cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
