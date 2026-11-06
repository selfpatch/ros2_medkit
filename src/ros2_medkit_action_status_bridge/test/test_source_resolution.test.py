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

"""A fault's reporting source is the action server node, not the action name."""

import os
import signal
import time
import unittest

from action_msgs.msg import GoalStatus, GoalStatusArray
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from ros2_medkit_msgs.srv import ListFaults
from unique_identifier_msgs.msg import UUID

SERVERS = 6
ACTIONS_PER_SERVER = 4
QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def generate_test_description():
    fault_manager = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'healing_enabled': True,
            'healing_threshold': 0,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    # A long rescan period: the bridge subscribes to a status topic on its first
    # tick after the topic appears, and nothing prunes it before the test is done.
    bridge = launch_ros.actions.Node(
        package='ros2_medkit_action_status_bridge',
        executable='action_status_bridge_node',
        name='action_status_bridge',
        output='screen',
        parameters=[{'rescan_period_sec': 3.0}],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return LaunchDescription([fault_manager, bridge, launch_testing.actions.ReadyToTest()])


def _aborted(goal_byte):
    gsa = GoalStatusArray()
    gs = GoalStatus()
    gs.goal_info.goal_id = UUID(uuid=[goal_byte] * 16)
    gs.status = int(GoalStatus.STATUS_ABORTED)
    gsa.status_list = [gs]
    return gsa


def _empty():
    return GoalStatusArray()


class TestSourceResolution(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client_node = Node('source_resolution_client')
        cls.list_faults = cls.client_node.create_client(
            ListFaults, '/fault_manager/list_faults')
        assert cls.list_faults.wait_for_service(timeout_sec=20.0)

    @classmethod
    def tearDownClass(cls):
        cls.client_node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _bridge_pid():
        """PID of the bridge started for this test (same ROS_DOMAIN_ID)."""
        domain = os.environ['ROS_DOMAIN_ID'].encode()
        for entry in os.listdir('/proc'):
            if not entry.isdigit():
                continue
            try:
                with open(f'/proc/{entry}/cmdline', 'rb') as f:
                    cmd = f.read()
                with open(f'/proc/{entry}/environ', 'rb') as f:
                    env = f.read().split(b'\0')
            except OSError:
                continue
            if b'action_status_bridge_node' in cmd and b'ROS_DOMAIN_ID=' + domain in env:
                return int(entry)
        raise AssertionError('bridge process not found')

    def _faults(self):
        req = ListFaults.Request()
        req.filter_by_severity = False
        req.statuses = []
        fut = self.list_faults.call_async(req)
        rclpy.spin_until_future_complete(self.client_node, fut, timeout_sec=5.0)
        return fut.result().faults if fut.result() is not None else []

    def test_source_is_the_server_node_while_the_bridge_is_stalled(self):
        """
        The status arrives before the bridge has the server's node name.

        The bridge is stopped while the servers appear and publish, so on resume
        the status data and the node names reach it together and in no fixed order.
        """
        pid = self._bridge_pid()
        # Each action first has a publisher with no verdict, so the bridge
        # subscribes to its status topic; that publisher's node then goes away.
        probe = Node('source_resolution_probe')
        actions = {}
        pubs = []
        for s in range(SERVERS):
            for a in range(ACTIONS_PER_SERVER):
                actions[f'/sr_{s}_{a}'] = f'/server_{s}'
        for name in actions:
            pubs.append(probe.create_publisher(
                GoalStatusArray, name + '/_action/status', QOS))
        deadline = time.time() + 30.0
        while time.time() < deadline:
            watching = sum(
                1 for name in actions
                if any(i.node_name == 'action_status_bridge' for i in
                       probe.get_subscriptions_info_by_topic(name + '/_action/status')))
            if watching == len(actions):
                break
            rclpy.spin_once(probe, timeout_sec=0.2)
        self.assertEqual(watching, len(actions), 'bridge did not subscribe to every action')

        os.kill(pid, signal.SIGSTOP)
        try:
            for p in pubs:
                probe.destroy_publisher(p)
            probe.destroy_node()
            servers = []
            for s in range(SERVERS):
                node = Node(f'server_{s}')
                held = []
                for name, server in actions.items():
                    if server == f'/server_{s}':
                        pub = node.create_publisher(
                            GoalStatusArray, name + '/_action/status', QOS)
                        pub.publish(_aborted(s + 1))
                        held.append(pub)
                servers.append((node, held))
        finally:
            os.kill(pid, signal.SIGCONT)

        want = len(actions)
        deadline = time.time() + 30.0
        faults = []
        while time.time() < deadline:
            faults = [f for f in self._faults() if f.fault_code.startswith('ACTION_SR_')]
            if len(faults) == want:
                break
            time.sleep(0.5)
        self.assertEqual(len(faults), want)
        wrong = []
        for f in faults:
            s = f.fault_code.split('_')[2]
            expected = f'/server_{s}'
            if expected not in list(f.reporting_sources):
                wrong.append((f.fault_code, list(f.reporting_sources)))
        for node, _ in servers:
            node.destroy_node()
        self.assertEqual(
            wrong, [], f'{len(wrong)} of {want} faults lack the server node as source')
