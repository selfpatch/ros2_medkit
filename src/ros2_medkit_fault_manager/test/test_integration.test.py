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

"""Integration tests for ros2_medkit_fault_manager services."""

import os
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetFaults, ReportFault


def get_coverage_env():
    """
    Get environment variables for gcov coverage data collection.

    When running with coverage enabled (ENABLE_COVERAGE=ON), subprocess nodes
    need GCOV_PREFIX set to write coverage data to the correct build directory.
    This allows integration test coverage to be captured alongside unit tests.

    Returns
    -------
    dict
        Environment variables dict with GCOV_PREFIX and GCOV_PREFIX_STRIP,
        or empty dict if coverage path cannot be determined.

    """
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_fault_manager')
        # pkg_prefix is like /path/to/workspace/install/ros2_medkit_fault_manager
        # workspace is 2 levels up from install/package_name
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_fault_manager')

        if os.path.exists(build_dir):
            # GCOV_PREFIX_STRIP removes leading path components from compiled-in paths
            # GCOV_PREFIX prepends the new path for .gcda file output
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        # Ignore: if coverage environment cannot be determined,
        # return empty dict so tests proceed without coverage data.
        pass
    return {}


def generate_test_description():
    """Generate launch description with fault_manager node."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        additional_env=get_coverage_env(),
        parameters=[{
            'storage_type': 'memory',  # Use in-memory storage for integration tests
            'confirmation_threshold': -3,  # Use debounce for testing lifecycle
        }],
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


class TestFaultManagerIntegration(unittest.TestCase):
    """Integration tests for FaultManager services."""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 context."""
        rclpy.init()
        cls.node = Node('test_fault_manager_client')

        # Create service clients
        cls.report_fault_client = cls.node.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        cls.get_faults_client = cls.node.create_client(
            GetFaults, '/fault_manager/get_faults'
        )
        cls.clear_fault_client = cls.node.create_client(
            ClearFault, '/fault_manager/clear_fault'
        )

        # Wait for services to be available
        assert cls.report_fault_client.wait_for_service(timeout_sec=10.0), \
            'report_fault service not available'
        assert cls.get_faults_client.wait_for_service(timeout_sec=10.0), \
            'get_faults service not available'
        assert cls.clear_fault_client.wait_for_service(timeout_sec=10.0), \
            'clear_fault service not available'

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

    def test_01_report_fault_success(self):
        """Test reporting a new fault."""
        request = ReportFault.Request()
        request.fault_code = 'TEST_FAULT_001'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'Test fault from integration test'
        request.source_id = '/test_node'

        response = self._call_service(self.report_fault_client, request)

        self.assertTrue(response.accepted)
        print('Report fault accepted')

    def test_02_report_fault_empty_code_fails(self):
        """Test that empty fault_code is rejected."""
        request = ReportFault.Request()
        request.fault_code = ''
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'Test fault'
        request.source_id = '/test_node'

        response = self._call_service(self.report_fault_client, request)

        self.assertFalse(response.accepted)
        print('Empty code rejected as expected')

    def test_03_report_fault_empty_source_fails(self):
        """Test that empty source_id is rejected."""
        request = ReportFault.Request()
        request.fault_code = 'TEST_FAULT_002'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'Test fault'
        request.source_id = ''

        response = self._call_service(self.report_fault_client, request)

        self.assertFalse(response.accepted)
        print('Empty source rejected as expected')

    def test_04_report_fault_invalid_severity_fails(self):
        """Test that invalid severity is rejected."""
        request = ReportFault.Request()
        request.fault_code = 'TEST_FAULT_003'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = 99  # Invalid severity
        request.description = 'Test fault'
        request.source_id = '/test_node'

        response = self._call_service(self.report_fault_client, request)

        self.assertFalse(response.accepted)
        print('Invalid severity rejected as expected')

    def test_05_get_faults_prefailed(self):
        """Test getting faults with PREFAILED status."""
        # First report a fault
        report_request = ReportFault.Request()
        report_request.fault_code = 'TEST_FAULT_GET_001'
        report_request.event_type = ReportFault.Request.EVENT_FAILED
        report_request.severity = Fault.SEVERITY_WARN
        report_request.description = 'Fault for get test'
        report_request.source_id = '/test_node'
        self._call_service(self.report_fault_client, report_request)

        # Query PREFAILED faults
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREFAILED]

        response = self._call_service(self.get_faults_client, get_request)

        self.assertGreater(len(response.faults), 0)
        fault_codes = [f.fault_code for f in response.faults]
        self.assertIn('TEST_FAULT_GET_001', fault_codes)
        print(f'Get faults returned {len(response.faults)} faults')

    def test_06_get_faults_filter_by_severity(self):
        """Test filtering faults by severity."""
        # Report faults with different severities
        for i, severity in enumerate([Fault.SEVERITY_INFO, Fault.SEVERITY_ERROR]):
            request = ReportFault.Request()
            request.fault_code = f'TEST_FAULT_SEV_{i}'
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = severity
            request.description = f'Fault with severity {severity}'
            request.source_id = '/test_node'
            self._call_service(self.report_fault_client, request)

        # Query ERROR severity only
        get_request = GetFaults.Request()
        get_request.filter_by_severity = True
        get_request.severity = Fault.SEVERITY_ERROR
        get_request.statuses = [Fault.STATUS_PREFAILED]

        response = self._call_service(self.get_faults_client, get_request)

        for fault in response.faults:
            self.assertEqual(fault.severity, Fault.SEVERITY_ERROR)
        print(f'Severity filter returned {len(response.faults)} ERROR faults')

    def test_07_clear_fault_success(self):
        """Test clearing a fault."""
        # First report a fault
        report_request = ReportFault.Request()
        report_request.fault_code = 'TEST_FAULT_CLEAR'
        report_request.event_type = ReportFault.Request.EVENT_FAILED
        report_request.severity = Fault.SEVERITY_ERROR
        report_request.description = 'Fault to be cleared'
        report_request.source_id = '/test_node'
        self._call_service(self.report_fault_client, report_request)

        # Clear the fault
        clear_request = ClearFault.Request()
        clear_request.fault_code = 'TEST_FAULT_CLEAR'

        response = self._call_service(self.clear_fault_client, clear_request)

        self.assertTrue(response.success)
        self.assertIn('cleared', response.message.lower())
        print(f'Clear fault response: {response.message}')

        # Verify fault is now CLEARED
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CLEARED]

        get_response = self._call_service(self.get_faults_client, get_request)

        cleared_codes = [f.fault_code for f in get_response.faults]
        self.assertIn('TEST_FAULT_CLEAR', cleared_codes)

    def test_08_clear_nonexistent_fault_fails(self):
        """Test clearing a non-existent fault returns false."""
        request = ClearFault.Request()
        request.fault_code = 'NONEXISTENT_FAULT_XYZ'

        response = self._call_service(self.clear_fault_client, request)

        self.assertFalse(response.success)
        self.assertIn('not found', response.message.lower())
        print(f'Clear nonexistent response: {response.message}')

    def test_09_clear_fault_empty_code_fails(self):
        """Test that clearing with empty fault_code fails."""
        request = ClearFault.Request()
        request.fault_code = ''

        response = self._call_service(self.clear_fault_client, request)

        self.assertFalse(response.success)
        self.assertIn('empty', response.message.lower())
        print(f'Clear empty code response: {response.message}')

    def test_10_report_updates_existing_fault(self):
        """Test that reporting same fault_code updates existing fault."""
        fault_code = 'TEST_FAULT_UPDATE'

        # Report first time
        request1 = ReportFault.Request()
        request1.fault_code = fault_code
        request1.event_type = ReportFault.Request.EVENT_FAILED
        request1.severity = Fault.SEVERITY_WARN
        request1.description = 'First report'
        request1.source_id = '/node1'

        response1 = self._call_service(self.report_fault_client, request1)
        self.assertTrue(response1.accepted)

        # Report second time from different source
        request2 = ReportFault.Request()
        request2.fault_code = fault_code
        request2.event_type = ReportFault.Request.EVENT_FAILED
        request2.severity = Fault.SEVERITY_ERROR  # Higher severity
        request2.description = 'Second report'
        request2.source_id = '/node2'

        response2 = self._call_service(self.report_fault_client, request2)
        self.assertTrue(response2.accepted)

        # Verify fault was updated
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREFAILED]

        get_response = self._call_service(self.get_faults_client, get_request)

        updated_fault = None
        for f in get_response.faults:
            if f.fault_code == fault_code:
                updated_fault = f
                break

        self.assertIsNotNone(updated_fault)
        self.assertEqual(updated_fault.severity, Fault.SEVERITY_ERROR)
        self.assertEqual(updated_fault.occurrence_count, 2)
        self.assertEqual(len(updated_fault.reporting_sources), 2)
        print(f'Updated fault: occurrence_count={updated_fault.occurrence_count}, '
              f'sources={updated_fault.reporting_sources}')

    def test_11_confirmation_workflow(self):
        """Test fault auto-confirms after reaching threshold (default=-3)."""
        fault_code = 'TEST_FAULT_CONFIRM'

        # Report fault 3 times (default threshold)
        for i in range(3):
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_ERROR
            request.description = f'Report {i + 1}'
            request.source_id = f'/node{i + 1}'
            self._call_service(self.report_fault_client, request)

        # Query CONFIRMED faults (should include our fault now)
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CONFIRMED]

        response = self._call_service(self.get_faults_client, get_request)

        confirmed_codes = [f.fault_code for f in response.faults]
        self.assertIn(fault_code, confirmed_codes)

        # Find the fault and verify its state
        confirmed_fault = next(f for f in response.faults if f.fault_code == fault_code)
        self.assertEqual(confirmed_fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(confirmed_fault.occurrence_count, 3)
        print(f'Fault confirmed: status={confirmed_fault.status}, '
              f'occurrence_count={confirmed_fault.occurrence_count}')

    def test_12_prefailed_excluded_from_default_query(self):
        """Test that PREFAILED faults are not returned in default query."""
        fault_code = 'TEST_FAULT_PREFAILED_ONLY'

        # Report fault once (stays PREFAILED since threshold=-3)
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_WARN
        request.description = 'Single report - should stay prefailed'
        request.source_id = '/test_node'
        self._call_service(self.report_fault_client, request)

        # Query with empty statuses (default = CONFIRMED only)
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = []  # Empty = CONFIRMED only

        response = self._call_service(self.get_faults_client, get_request)

        # Fault should NOT be in results (it's PREFAILED)
        fault_codes = [f.fault_code for f in response.faults]
        self.assertNotIn(fault_code, fault_codes)
        print('Default query excluded PREFAILED fault as expected')

        # But it should be visible when querying PREFAILED explicitly
        get_prefailed = GetFaults.Request()
        get_prefailed.filter_by_severity = False
        get_prefailed.severity = 0
        get_prefailed.statuses = [Fault.STATUS_PREFAILED]

        prefailed_response = self._call_service(self.get_faults_client, get_prefailed)
        prefailed_codes = [f.fault_code for f in prefailed_response.faults]
        self.assertIn(fault_code, prefailed_codes)
        print('PREFAILED query found the fault as expected')

    def test_13_multi_source_confirmation(self):
        """Test fault confirms when multiple sources report same fault."""
        fault_code = 'TEST_FAULT_MULTI_SRC'

        # Report from 3 different sources
        sources = ['/sensor1', '/sensor2', '/sensor3']
        for source in sources:
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_ERROR
            request.description = 'Multi-source fault'
            request.source_id = source
            self._call_service(self.report_fault_client, request)

        # Query and verify
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CONFIRMED]

        response = self._call_service(self.get_faults_client, get_request)

        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(len(fault.reporting_sources), 3)
        self.assertEqual(fault.occurrence_count, 3)
        print(f'Multi-source fault confirmed: sources={fault.reporting_sources}')

    def test_14_critical_severity_immediate_confirmation(self):
        """Test CRITICAL severity bypasses debounce and confirms immediately."""
        fault_code = 'TEST_FAULT_CRITICAL'

        # Report single CRITICAL fault
        request = ReportFault.Request()
        request.fault_code = fault_code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'Critical fault - should confirm immediately'
        request.source_id = '/test_node'
        self._call_service(self.report_fault_client, request)

        # Query CONFIRMED faults
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_CONFIRMED]

        response = self._call_service(self.get_faults_client, get_request)

        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_CONFIRMED)
        self.assertEqual(fault.occurrence_count, 1)  # Only one report needed
        print('CRITICAL fault confirmed immediately as expected')

    def test_15_passed_event_increments_counter(self):
        """Test PASSED events increment debounce counter towards healing."""
        fault_code = 'TEST_FAULT_PASSED'

        # Report 2 FAILED events (counter = -2)
        for i in range(2):
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_FAILED
            request.severity = Fault.SEVERITY_ERROR
            request.description = f'Failed report {i + 1}'
            request.source_id = f'/node{i + 1}'
            self._call_service(self.report_fault_client, request)

        # Verify fault is PREFAILED
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREFAILED]

        response = self._call_service(self.get_faults_client, get_request)
        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_PREFAILED)

        # Report 3 PASSED events (counter = -2 + 3 = +1)
        for i in range(3):
            request = ReportFault.Request()
            request.fault_code = fault_code
            request.event_type = ReportFault.Request.EVENT_PASSED
            request.severity = 0  # Ignored for PASSED
            request.description = ''  # Ignored for PASSED
            request.source_id = '/test_node'
            self._call_service(self.report_fault_client, request)

        # Verify fault is PREPASSED (counter > 0)
        get_request = GetFaults.Request()
        get_request.filter_by_severity = False
        get_request.severity = 0
        get_request.statuses = [Fault.STATUS_PREPASSED]

        response = self._call_service(self.get_faults_client, get_request)
        fault = next((f for f in response.faults if f.fault_code == fault_code), None)
        self.assertIsNotNone(fault)
        self.assertEqual(fault.status, Fault.STATUS_PREPASSED)
        print('PASSED events moved fault to PREPASSED as expected')


@launch_testing.post_shutdown_test()
class TestFaultManagerShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_code(self, proc_info):
        """Verify fault_manager exits cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
