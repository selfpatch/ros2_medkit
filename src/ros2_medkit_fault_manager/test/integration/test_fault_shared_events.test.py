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
The shared record of a default code, through a real fault manager on both stores.

Two sources reporting one default code feed one record: the first report confirms it, the
second updates it, and the update names both sources with the occurrence count still 1.
"""

import os
import shutil
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.msg import Fault, FaultEvent
from ros2_medkit_msgs.srv import ReportFault

TIMEOUT_SEC = 30.0
_temp_dirs = []


@launch_testing.parametrize('storage_type', ['memory', 'sqlite'])
@launch_testing.markers.keep_alive
def generate_test_description(storage_type):
    temp_dir = tempfile.mkdtemp(prefix='shared_events_')
    _temp_dirs.append(temp_dir)
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': storage_type,
            'database_path': os.path.join(temp_dir, 'faults.db'),
            'confirmation_threshold': -1,
            'snapshots.enabled': False,
            'snapshots.rosbag.enabled': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([fault_manager_node, launch_testing.actions.ReadyToTest()]),
        {'fault_manager_node': fault_manager_node, 'storage_type': storage_type},
    )


class TestSharedRecordEvents(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_shared_events_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        assert cls.report_client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        cls.events = []
        cls.subscription = cls.node.create_subscription(
            FaultEvent, '/fault_manager/events', cls.events.append,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        deadline = time.monotonic() + TIMEOUT_SEC
        while cls.subscription.get_publisher_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(cls.node, timeout_sec=0.1)
        assert cls.subscription.get_publisher_count() > 0

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _report(self, code, source):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = f'{code} from {source}'
        request.source_id = source
        future = self.report_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().accepted)

    def _wait_events(self, count):
        deadline = time.monotonic() + TIMEOUT_SEC
        while len(self.events) < count and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertGreaterEqual(len(self.events), count)

    def test_a_second_source_updates_the_one_shared_record(self, storage_type):
        self._report('SE.DEFAULT', '/a')
        self._report('SE.DEFAULT', '/b')
        self._wait_events(2)

        events = [(e.event_type, e.fault) for e in self.events
                  if e.fault.fault_code == 'SE.DEFAULT']
        self.assertEqual([t for t, _ in events],
                         [FaultEvent.EVENT_CONFIRMED, FaultEvent.EVENT_UPDATED], storage_type)
        updated = events[1][1]
        self.assertEqual(list(updated.reporting_sources), ['/a', '/b'])
        self.assertEqual(updated.occurrence_count, 1)


@launch_testing.post_shutdown_test()
class TestSharedRecordEventsShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, fault_manager_node):
        launch_testing.asserts.assertExitCodes(proc_info, process=fault_manager_node)

    def test_temp_dirs_removed(self):
        for path in _temp_dirs:
            shutil.rmtree(path, ignore_errors=True)
            self.assertFalse(os.path.exists(path))
