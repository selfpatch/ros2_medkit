#!/usr/bin/env python3
# Copyright 2025 mfaferek93
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

"""Integration tests for ros2_medkit_fault_reporter library."""

import os
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetFaults, ReportFault


def get_coverage_env():
    """
    Get environment variables for gcov coverage data collection.

    Returns
    -------
    dict
        Environment variables dict with GCOV_PREFIX and GCOV_PREFIX_STRIP,
        or empty dict if coverage path cannot be determined.

    """
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_fault_reporter')
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_fault_reporter')

        if os.path.exists(build_dir):
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        # Coverage setup is best-effort; ignore failures and run tests without gcov.
        pass
    return {}


def generate_test_description():
    """Generate launch description with fault_manager node."""
    # We need fault_manager running for FaultReporter to connect to
    # Use in-memory storage to avoid filesystem permission issues in CI
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=get_coverage_env(),
        parameters=[{'storage_type': 'memory'}],
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


class TestFaultReporterIntegration(unittest.TestCase):
    """
    Integration tests for FaultReporter library.

    Since FaultReporter is a C++ library, these tests verify the behavior by
    calling the FaultManager services directly to simulate what FaultReporter
    does, and checking the results.
    """

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context."""
        rclpy.init()
        cls.node = Node('test_fault_reporter_client')

        # Create service clients
        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.get_faults_client = cls.node.create_client(
            GetFaults, '/fault_manager/get_faults'
        )

        # Wait for services to be available
        assert cls.report_fault_client.wait_for_service(timeout_sec=10.0), \
            'report_fault service not available'
        assert cls.get_faults_client.wait_for_service(timeout_sec=10.0), \
            'get_faults service not available'

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 context."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_service(self, client, request):
        """Call a service and wait for response."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report_fault(self, fault_code, severity, description, source_id,
                      event_type=ReportFault.Request.EVENT_FAILED):
        """Report a fault via service call."""
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = event_type
        request.severity = severity
        request.description = description
        request.source_id = source_id
        return self._call_service(self.report_fault_client, request)

    def _get_faults(self, statuses=None):
        """Get faults with given statuses via service call."""
        request = GetFaults.Request()
        request.filter_by_severity = False
        request.severity = 0
        request.statuses = statuses or ['PREFAILED', 'CONFIRMED']
        return self._call_service(self.get_faults_client, request)

    def test_01_service_connectivity(self):
        """Test that FaultManager services are available."""
        self.assertTrue(self.report_fault_client.service_is_ready())
        self.assertTrue(self.get_faults_client.service_is_ready())

    def test_02_report_fault_is_stored(self):
        """Test that reported faults are stored in FaultManager."""
        # Use CRITICAL severity to bypass debounce and get immediate CONFIRMED status
        response = self._report_fault(
            fault_code='REPORTER_TEST_001',
            severity=Fault.SEVERITY_CRITICAL,
            description='Test from FaultReporter integration test',
            source_id='/test_reporter_node'
        )

        self.assertTrue(response.accepted)

        # Verify fault is in storage with CONFIRMED status
        faults_response = self._get_faults(statuses=['CONFIRMED'])
        fault_codes = [f.fault_code for f in faults_response.faults]
        self.assertIn('REPORTER_TEST_001', fault_codes)

    def test_03_multiple_reports_aggregate(self):
        """Test that multiple reports of same fault aggregate."""
        # First report - use CRITICAL to bypass debounce
        self._report_fault(
            fault_code='REPORTER_TEST_002',
            severity=Fault.SEVERITY_CRITICAL,
            description='First occurrence',
            source_id='/node_a'
        )

        # Second report from different source
        self._report_fault(
            fault_code='REPORTER_TEST_002',
            severity=Fault.SEVERITY_CRITICAL,
            description='Second occurrence',
            source_id='/node_b'
        )

        # Third report
        self._report_fault(
            fault_code='REPORTER_TEST_002',
            severity=Fault.SEVERITY_CRITICAL,
            description='Third occurrence',
            source_id='/node_a'
        )

        # Verify aggregation
        faults_response = self._get_faults(statuses=['CONFIRMED'])
        fault = next(
            (f for f in faults_response.faults if f.fault_code == 'REPORTER_TEST_002'),
            None
        )

        self.assertIsNotNone(fault)
        self.assertEqual(fault.occurrence_count, 3)
        self.assertEqual(fault.severity, Fault.SEVERITY_CRITICAL)
        self.assertIn('/node_a', fault.reporting_sources)
        self.assertIn('/node_b', fault.reporting_sources)

    def test_04_source_id_tracked(self):
        """Test that source_id is correctly tracked."""
        # Use CRITICAL severity to bypass debounce
        self._report_fault(
            fault_code='REPORTER_TEST_003',
            severity=Fault.SEVERITY_CRITICAL,
            description='Source tracking test',
            source_id='/perception/lidar/scanner_node'
        )

        faults_response = self._get_faults(statuses=['CONFIRMED'])
        fault = next(
            (f for f in faults_response.faults if f.fault_code == 'REPORTER_TEST_003'),
            None
        )

        self.assertIsNotNone(fault)
        self.assertIn('/perception/lidar/scanner_node', fault.reporting_sources)
