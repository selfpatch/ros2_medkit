#!/usr/bin/env python3
# Copyright 2026 mfaferek93, bburda
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

"""End-to-end integration tests for ros2_medkit_action_status_bridge."""

import os
import time
import unittest

from action_msgs.msg import GoalStatus, GoalStatusArray
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults
from unique_identifier_msgs.msg import UUID

STATUS_TOPIC = '/test_action/_action/status'
ABORTED_CODE = 'ACTION_TEST_ACTION_ABORTED'


def generate_test_description():
    """Launch description with fault_manager and action_status_bridge nodes."""
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,  # immediate confirmation
            'healing_enabled': True,
            # The bridge emits one PASSED per recovery (state machine), so the
            # fault heals on that single event only with healing_threshold 0.
            'healing_threshold': 0,
        }],
    )

    # Load the packaged config first so assertExitCodes guards against the
    # shipped action_status_bridge.yaml regressing the empty-untyped-list
    # startup crash; the inline override only speeds up discovery for the test.
    config_file = os.path.join(
        get_package_share_directory('ros2_medkit_action_status_bridge'),
        'config', 'action_status_bridge.yaml')
    action_status_bridge_node = launch_ros.actions.Node(
        package='ros2_medkit_action_status_bridge',
        executable='action_status_bridge_node',
        name='action_status_bridge',
        output='screen',
        parameters=[config_file, {
            'rescan_period_sec': 1.0,  # fast discovery for the test
        }],
    )

    return (
        LaunchDescription([
            fault_manager_node,
            action_status_bridge_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'fault_manager_node': fault_manager_node,
            'action_status_bridge_node': action_status_bridge_node,
        },
    )


def _status_array(status, goal_byte):
    gsa = GoalStatusArray()
    gs = GoalStatus()
    gs.goal_info.goal_id = UUID(uuid=[goal_byte] * 16)
    gs.status = int(status)
    gsa.status_list = [gs]
    return gsa


class TestActionStatusBridgeIntegration(unittest.TestCase):
    """End-to-end action-status -> fault discovery and raise/heal."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_action_status_client')
        # Match the bridge's subscription QoS so the latched status is delivered.
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        cls.status_pub = cls.node.create_publisher(GoalStatusArray, STATUS_TOPIC, qos)
        cls.list_faults_client = cls.node.create_client(
            ListFaults, '/fault_manager/list_faults'
        )
        assert cls.list_faults_client.wait_for_service(timeout_sec=10.0), \
            'ListFaults service not available'

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def list_faults(self, statuses=None):
        request = ListFaults.Request()
        request.filter_by_severity = False
        request.statuses = statuses or []
        future = self.list_faults_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        return future.result().faults

    def find(self, faults, code):
        return next((f for f in faults if f.fault_code == code), None)

    def wait_for_fault(self, code, statuses=None, want_status=None, timeout=15.0):
        """Poll list_faults until the fault exists and optionally reaches a status."""
        deadline = time.time() + timeout
        fault = None
        while time.time() < deadline:
            fault = self.find(self.list_faults(statuses), code)
            if fault is not None and (want_status is None or fault.status == want_status):
                return fault
            time.sleep(0.5)
        return fault

    def test_01_aborted_goal_raises_fault(self):
        """An ABORTED goal on a discovered status topic raises a fault."""
        self.status_pub.publish(_status_array(GoalStatus.STATUS_ABORTED, 1))
        # Poll through runtime discovery (rescan) + subscription + processing.
        fault = self.wait_for_fault(ABORTED_CODE)
        self.assertIsNotNone(fault, f'expected a {ABORTED_CODE} fault')
        self.assertEqual(fault.severity, Fault.SEVERITY_ERROR)
        # Attributed to the action server's node FQN (this test node publishes it).
        self.assertIn('/test_action_status_client', list(fault.reporting_sources))

    def test_02_succeeded_goal_heals_fault(self):
        """A later SUCCEEDED goal (no failing goal in the array) heals it."""
        self.status_pub.publish(_status_array(GoalStatus.STATUS_SUCCEEDED, 2))
        fault = self.wait_for_fault(
            ABORTED_CODE,
            statuses=[Fault.STATUS_CONFIRMED, Fault.STATUS_HEALED],
            want_status=Fault.STATUS_HEALED,
        )
        self.assertIsNotNone(fault, f'{ABORTED_CODE} fault not HEALED in time')
        self.assertEqual(fault.status, Fault.STATUS_HEALED)


@launch_testing.post_shutdown_test()
class TestActionStatusBridgeShutdown(unittest.TestCase):
    """Test clean shutdown."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
