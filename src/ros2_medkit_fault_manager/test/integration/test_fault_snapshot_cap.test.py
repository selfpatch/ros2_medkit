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
Snapshots of per-source records with the recapture cooldown off, through a real fault manager.

  * a cooldown of 0 turns grouping off: two owners confirmed back to back are two captures;
  * the per-record cap keeps the newest capture of each record, both topics of it, and one
    owner's captures never trim another owner's.

The cap is read from the store the fault manager writes, because GetSnapshots only ever
returns the newest capture of a record.
"""

import json
import os
import shutil
import sqlite3
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

TIMEOUT_SEC = 40.0
FIRST = '/cap/first'
SECOND = '/cap/second'
TEMP_DIR = tempfile.mkdtemp(prefix='snapshot_cap_')
DB_PATH = os.path.join(TEMP_DIR, 'faults.db')


def _temp_file(prefix, content):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(content)
    handle.close()
    return handle.name


THRESHOLDS_FILE = _temp_file(
    'snapshot_cap_codes_', ''.join(f'{c}:\n  per_source: true\n' for c in ('XC.TWO', 'XC.CAP')))
SNAPSHOTS_FILE = _temp_file(
    'snapshot_cap_topics_',
    'fault_specific:\n'
    f'  XC.TWO:\n    - {FIRST}\n'
    f'  XC.CAP:\n    - {FIRST}\n    - {SECOND}\n')


def generate_test_description():
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'sqlite',
            'database_path': DB_PATH,
            'confirmation_threshold': -1,
            'fault_thresholds.config_file': THRESHOLDS_FILE,
            'snapshots.enabled': True,
            'snapshots.config_file': SNAPSHOTS_FILE,
            'snapshots.timeout_sec': 3.0,
            'snapshots.capture_pool_size': 3,
            'snapshots.recapture_cooldown_sec': 0.0,
            'snapshots.max_per_fault': 1,
            'snapshots.retain_on_clear': True,
            'snapshots.rosbag.enabled': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([fault_manager_node, launch_testing.actions.ReadyToTest()]),
        {'fault_manager_node': fault_manager_node},
    )


class TestSnapshotCap(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_snapshot_cap_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        cls.snapshots_client = cls.node.create_client(
            GetSnapshots, '/fault_manager/get_snapshots')
        for client in (cls.report_client, cls.clear_client, cls.snapshots_client):
            assert client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        cls.publishers = [cls.node.create_publisher(Int32, topic, 10) for topic in (FIRST, SECOND)]
        cls.stop = threading.Event()
        cls.publisher_thread = threading.Thread(target=cls._publish)
        cls.publisher_thread.start()

    @classmethod
    def _publish(cls):
        value = 0
        while not cls.stop.wait(0.1):
            value += 1
            for publisher in cls.publishers:
                publisher.publish(Int32(data=value))

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

    def _clear(self, code, source):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        self.assertTrue(self._call(self.clear_client, request).success)

    def _wait_snapshot(self, code, source, newer_than=0.0):
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            request = GetSnapshots.Request()
            request.fault_code = code
            request.source_id = source
            response = self._call(self.snapshots_client, request)
            if response.success and response.data:
                data = json.loads(response.data)
                if data.get('topics') and data['captured_at'] > newer_than:
                    return data
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail(f'{code}/{source} never listed a snapshot newer than {newer_than}')

    def _stored(self, code):
        """Return {owner: {capture_id: [topics]}} as the store holds it."""
        with sqlite3.connect(f'file:{DB_PATH}?mode=ro', uri=True) as db:
            rows = db.execute(
                'SELECT owner, capture_id, topic FROM snapshots WHERE fault_code = ?',
                (code,)).fetchall()
        stored = {}
        for owner, capture_id, topic in rows:
            stored.setdefault(owner, {}).setdefault(capture_id, []).append(topic)
        return stored

    def test_a_cooldown_of_zero_turns_grouping_off(self):
        self._report('XC.TWO', '/cap/a')
        self._report('XC.TWO', '/cap/b')
        first = self._wait_snapshot('XC.TWO', '/cap/a')
        second = self._wait_snapshot('XC.TWO', '/cap/b')
        self.assertNotEqual(first['captured_at'], second['captured_at'])
        stored = self._stored('XC.TWO')
        self.assertEqual(sorted(stored), ['/cap/a', '/cap/b'])
        capture_ids = {cid for captures in stored.values() for cid in captures}
        self.assertEqual(len(capture_ids), 2, 'two owners must be two captures')

    def test_the_cap_keeps_the_newest_capture_of_each_record(self):
        self._report('XC.CAP', '/cap/a')
        first = self._wait_snapshot('XC.CAP', '/cap/a')
        self._clear('XC.CAP', '/cap/a')
        self._report('XC.CAP', '/cap/a')
        second = self._wait_snapshot('XC.CAP', '/cap/a', newer_than=first['captured_at'])
        self._report('XC.CAP', '/cap/b')
        own = self._wait_snapshot('XC.CAP', '/cap/b')

        self.assertEqual(sorted(second['topics']), [FIRST, SECOND])
        self.assertEqual(sorted(own['topics']), [FIRST, SECOND])
        stored = self._stored('XC.CAP')
        self.assertEqual(len(stored['/cap/a']), 1, 'the cap keeps one capture per record')
        self.assertEqual(sorted(next(iter(stored['/cap/a'].values()))), [FIRST, SECOND])
        self.assertEqual(len(stored['/cap/b']), 1)
        self.assertEqual(sorted(next(iter(stored['/cap/b'].values()))), [FIRST, SECOND])
        self.assertNotEqual(stored['/cap/a'].keys(), stored['/cap/b'].keys())


@launch_testing.post_shutdown_test()
class TestSnapshotCapShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE)
        os.unlink(SNAPSHOTS_FILE)
        shutil.rmtree(TEMP_DIR, ignore_errors=True)
