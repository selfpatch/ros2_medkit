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
fault-code layer is what it does to the entity layer underneath it: an entity
override is chosen by the reporting source, so without this layer two entities
reporting one code debounce it two ways. Each owner's record keeps its own
counter, and the fault-code override sets the band it is judged against.
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

# Entities whose debounce policies differ, from test_entity_thresholds.yaml.
LIDAR = '/sensors/lidar/front'
MOTOR = '/powertrain/motor/left'


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

    def _status(self, fault_code, source_id):
        """Get the current status of the record one source owns."""
        req = GetFault.Request()
        req.fault_code = fault_code
        req.source_id = source_id
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
        self.assertEqual(self._status('SHARED.OVERHEAT', LIDAR), Fault.STATUS_PREFAILED)

        self._report('SHARED.OVERHEAT', LIDAR)
        self.assertEqual(self._status('SHARED.OVERHEAT', LIDAR), Fault.STATUS_PREFAILED)

        self._report('SHARED.OVERHEAT', LIDAR)
        self.assertEqual(self._status('SHARED.OVERHEAT', LIDAR), Fault.STATUS_CONFIRMED)

    # @verifies REQ_INTEROP_107
    def test_02_one_code_debounces_alike_for_each_entitys_record(self):
        """
        Confirm each owner's SHARED.JAM record on its own fourth event.

        The two sources own two records on their own counters. Under the entity
        layer alone lidar's -1 would confirm its record on the first event and
        the motor's -5 would take five.
        """
        for source in (MOTOR, LIDAR):
            for _ in range(3):
                self._report('SHARED.JAM', source)
                self.assertEqual(
                    self._status('SHARED.JAM', source), Fault.STATUS_PREFAILED)

            self._report('SHARED.JAM', source)
            self.assertEqual(
                self._status('SHARED.JAM', source), Fault.STATUS_CONFIRMED)

    # @verifies REQ_INTEROP_107
    def test_03_a_code_with_no_override_keeps_the_entity_policy(self):
        """The new layer is opt-in: an unlisted code still follows its entity."""
        self._report('MOTOR.ONLY', MOTOR)
        self.assertEqual(self._status('MOTOR.ONLY', MOTOR), Fault.STATUS_PREFAILED)

        self._report('LIDAR.ONLY', LIDAR)
        self.assertEqual(self._status('LIDAR.ONLY', LIDAR), Fault.STATUS_CONFIRMED)

    # @verifies REQ_INTEROP_107
    def test_04_a_half_pinned_code_confirms_on_the_pinned_threshold(self):
        """
        Report SHARED.PARTIAL, which pins confirmation but not healing.

        Both owners' records confirm on their own second event, whatever
        confirmation threshold their entity carries.
        """
        for source in (LIDAR, MOTOR):
            self._report('SHARED.PARTIAL', source)
            self.assertEqual(
                self._status('SHARED.PARTIAL', source), Fault.STATUS_PREFAILED)

            self._report('SHARED.PARTIAL', source)
            self.assertEqual(
                self._status('SHARED.PARTIAL', source), Fault.STATUS_CONFIRMED)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify fault_manager exits cleanly."""

    def test_exit_code(self, proc_info):
        """Check process exit code."""
        launch_testing.asserts.assertExitCodes(proc_info)
