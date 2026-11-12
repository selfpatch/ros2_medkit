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
Snapshot capture of per-source records, through a real fault manager.

Per-source records of one code that confirm inside the recapture cooldown share ONE capture of
the code's topics, and every record lists those snapshots under its own owner. The rows of one
capture all carry the same captured_at, so two captures of one code cannot be mistaken for one.
A record that confirms again after it was acknowledged gets a capture of its own.
"""

import json
import os
import tempfile
import threading
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetSnapshots, ReportFault
from std_msgs.msg import Int32

TIMEOUT_SEC = 30.0
SOURCES = ['/grp/a', '/grp/b', '/grp/c']


def _temp_file(prefix, content):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(content)
    handle.close()
    return handle.name


THRESHOLDS_FILE = _temp_file(
    'snapshot_grouping_codes_',
    'XG.GROUP:\n  per_source: true\nXG.AGAIN:\n  per_source: true\n')
SNAPSHOTS_FILE = _temp_file(
    'snapshot_grouping_topics_',
    'fault_specific:\n'
    '  XG.GROUP:\n    - /grouping/counter\n'
    '  XG.AGAIN:\n    - /grouping/counter\n'
    '  XG.SHARED:\n    - /grouping/counter\n')


def generate_test_description():
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'fault_thresholds.config_file': THRESHOLDS_FILE,
            'snapshots.enabled': True,
            'snapshots.config_file': SNAPSHOTS_FILE,
            'snapshots.timeout_sec': 5.0,
            'snapshots.capture_pool_size': 3,
            'snapshots.rosbag.enabled': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([fault_manager_node, launch_testing.actions.ReadyToTest()]),
        {'fault_manager_node': fault_manager_node},
    )


class TestSnapshotGrouping(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_snapshot_grouping_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        cls.snapshots_client = cls.node.create_client(
            GetSnapshots, '/fault_manager/get_snapshots')
        for client in (cls.report_client, cls.clear_client, cls.snapshots_client):
            assert client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        cls.publisher = cls.node.create_publisher(Int32, '/grouping/counter', 10)
        cls.stop = threading.Event()
        cls.publisher_thread = threading.Thread(target=cls._publish)
        cls.publisher_thread.start()

    @classmethod
    def _publish(cls):
        value = 0
        while not cls.stop.wait(0.2):
            value += 1
            cls.publisher.publish(Int32(data=value))

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()
        cls.publisher_thread.join()
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), 'service call timed out')
        return future.result()

    def _report(self, code, source):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = f'{code} from {source}'
        request.source_id = source
        self.assertTrue(self._call(self.report_client, request).accepted)

    def _snapshot(self, code, source):
        request = GetSnapshots.Request()
        request.fault_code = code
        request.source_id = source
        return self._call(self.snapshots_client, request)

    def _wait_snapshot(self, code, source, newer_than=0.0):
        """Poll GetSnapshots until the record lists a capture newer than @p newer_than."""
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            response = self._snapshot(code, source)
            if response.success and response.data:
                data = json.loads(response.data)
                if data.get('topics') and data['captured_at'] > newer_than:
                    return data
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail(f'{code}/{source} never listed a snapshot')

    def test_per_source_records_share_one_capture(self):
        for source in SOURCES:
            self._report('XG.GROUP', source)

        captures = [self._wait_snapshot('XG.GROUP', source) for source in SOURCES]
        for data in captures:
            self.assertIn('/grouping/counter', data['topics'])
        # One capture: every owner lists the very same sampled values and the same moment.
        self.assertEqual({d['captured_at'] for d in captures}, {captures[0]['captured_at']})
        self.assertEqual(
            {json.dumps(d['topics'], sort_keys=True) for d in captures},
            {json.dumps(captures[0]['topics'], sort_keys=True)})

    def test_a_record_that_confirms_again_gets_a_capture_of_its_own(self):
        code = 'XG.AGAIN'
        source = '/grp/again'
        self._report(code, source)
        first = self._wait_snapshot(code, source)

        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        self.assertTrue(self._call(self.clear_client, request).success)

        self._report(code, source)
        second = self._wait_snapshot(code, source, newer_than=first['captured_at'])
        self.assertGreater(second['captured_at'], first['captured_at'])

    def test_a_shared_record_is_captured_once(self):
        for source in SOURCES:
            self._report('XG.SHARED', source)
        data = self._wait_snapshot('XG.SHARED', '')
        self.assertIn('/grouping/counter', data['topics'])


@launch_testing.post_shutdown_test()
class TestSnapshotGroupingShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE)
        os.unlink(SNAPSHOTS_FILE)
