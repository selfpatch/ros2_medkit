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
Shared snapshot capture of per-source records under queue pressure, through a real fault manager.

One worker, a queue of two and drop_oldest, so every extra queued job evicts another fault's
capture. A topic nobody publishes on keeps a capture running for its whole timeout.

  * a record joining the queued capture of its code queues no job of its own;
  * a capture that outlives the recapture cooldown keeps every record that joined it;
  * a record that confirms again while its old capture runs gets a sample taken afterwards.
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

TIMEOUT_SEC = 40.0
FAST = '/queue/fast'
SLOW = '/queue/slow'


def _temp_file(prefix, content):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(content)
    handle.close()
    return handle.name


THRESHOLDS_FILE = _temp_file(
    'snapshot_queue_codes_',
    ''.join(f'{code}:\n  per_source: true\n'
            for code in ('XQ.QUEUE', 'XQ.PRUNE', 'XQ.PRUNE.TRIGGER', 'XQ.AGAIN')))
SNAPSHOTS_FILE = _temp_file(
    'snapshot_queue_topics_',
    'fault_specific:\n'
    f'  XQ.BUSY:\n    - {SLOW}\n'
    f'  XQ.OTHER:\n    - {FAST}\n'
    f'  XQ.QUEUE:\n    - {FAST}\n'
    f'  XQ.PRUNE:\n    - {FAST}\n    - {SLOW}\n'
    f'  XQ.PRUNE.TRIGGER:\n    - {FAST}\n'
    f'  XQ.AGAIN:\n    - {FAST}\n    - {SLOW}\n'
    'patterns:\n'
    f'  "XQ_PROBE_.*":\n    - {FAST}\n')


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
            'snapshots.timeout_sec': 2.0,
            'snapshots.capture_pool_size': 1,
            'snapshots.capture_queue_depth': 2,
            'snapshots.capture_queue_full_policy': 'drop_oldest',
            'snapshots.recapture_cooldown_sec': 0.3,
            'snapshots.rosbag.enabled': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([fault_manager_node, launch_testing.actions.ReadyToTest()]),
        {'fault_manager_node': fault_manager_node},
    )


class TestSnapshotQueue(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_snapshot_queue_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        cls.snapshots_client = cls.node.create_client(
            GetSnapshots, '/fault_manager/get_snapshots')
        for client in (cls.report_client, cls.clear_client, cls.snapshots_client):
            assert client.wait_for_service(timeout_sec=TIMEOUT_SEC)
        cls.fast = cls.node.create_publisher(Int32, FAST, 10)
        # Nothing is ever published here, so a capture of this topic runs for its whole timeout.
        cls.slow = cls.node.create_publisher(Int32, SLOW, 10)
        cls.stop = threading.Event()
        cls.publisher_thread = threading.Thread(target=cls._publish)
        cls.publisher_thread.start()
        cls._wait_for_discovery('XQ_PROBE_')

    @classmethod
    def _wait_for_discovery(cls, topic_probe_prefix):
        """Report fresh probe codes until the fault manager has discovered the probed topic."""
        deadline = time.monotonic() + TIMEOUT_SEC
        index = 0
        while time.monotonic() < deadline:
            index += 1
            code = f'{topic_probe_prefix}{index}'
            report = ReportFault.Request()
            report.fault_code = code
            report.event_type = ReportFault.Request.EVENT_FAILED
            report.severity = Fault.SEVERITY_CRITICAL
            report.description = 'discovery probe'
            report.source_id = '/probe'
            future = cls.report_client.call_async(report)
            rclpy.spin_until_future_complete(cls.node, future, timeout_sec=10.0)
            request = GetSnapshots.Request()
            request.fault_code = code
            attempt_end = time.monotonic() + 1.5
            while time.monotonic() < attempt_end:
                future = cls.snapshots_client.call_async(request)
                rclpy.spin_until_future_complete(cls.node, future, timeout_sec=10.0)
                response = future.result()
                if response is not None and response.success and response.data:
                    if json.loads(response.data).get('topics'):
                        return
                rclpy.spin_once(cls.node, timeout_sec=0.1)
        raise AssertionError('the fault manager never discovered the published topic')

    @classmethod
    def _publish(cls):
        value = 0
        while not cls.stop.wait(0.1):
            value += 1
            cls.fast.publish(Int32(data=value))

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

    def _snapshot(self, code, source):
        """Return the record's snapshot data, or None while it lists none."""
        request = GetSnapshots.Request()
        request.fault_code = code
        request.source_id = source
        response = self._call(self.snapshots_client, request)
        if response.success and response.data:
            data = json.loads(response.data)
            if data.get('topics'):
                return data
        return None

    def _wait_snapshot(self, code, source, between=None):
        """Poll until the record lists a snapshot; run @p between after every miss."""
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            data = self._snapshot(code, source)
            if data:
                return data
            if between:
                between()
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.fail(f'{code}/{source} never listed a snapshot')

    def _wait_capture_started(self, proc_output, fault_manager_node, code):
        proc_output.assertWaitFor(
            f"Capturing snapshots for fault '{code}'",
            process=fault_manager_node, timeout=TIMEOUT_SEC)

    def test_a_joining_record_does_not_evict_another_faults_capture(
            self, proc_output, fault_manager_node):
        # The worker is busy with a capture that waits on a silent topic.
        self._report('XQ.BUSY', '/q/busy')
        self._wait_capture_started(proc_output, fault_manager_node, 'XQ.BUSY')
        # The oldest queued job belongs to another fault.
        self._report('XQ.OTHER', '/q/other')
        # Two owners of one code: one queued job for both. A second job would fill the queue and
        # evict the oldest one, which is the other fault's.
        self._report('XQ.QUEUE', '/q/a')
        self._report('XQ.QUEUE', '/q/b')

        self._wait_snapshot('XQ.OTHER', '/q/other')
        first = self._wait_snapshot('XQ.QUEUE', '/q/a')
        second = self._wait_snapshot('XQ.QUEUE', '/q/b')
        self.assertEqual(first['captured_at'], second['captured_at'])

    def test_a_capture_that_outlives_the_cooldown_keeps_its_records(
            self, proc_output, fault_manager_node):
        self._report('XQ.PRUNE', '/q/a')
        self._wait_capture_started(proc_output, fault_manager_node, 'XQ.PRUNE')
        self._report('XQ.PRUNE', '/q/b')

        # Every admission of another grouped code is a chance to drop an aged-out group. The
        # capture above runs for the whole topic timeout, much longer than the cooldown.
        triggers = iter(range(10000))
        first = self._wait_snapshot(
            'XQ.PRUNE', '/q/a',
            between=lambda: self._report('XQ.PRUNE.TRIGGER', f'/q/trigger{next(triggers)}'))
        second = self._wait_snapshot('XQ.PRUNE', '/q/b')
        self.assertEqual(first['captured_at'], second['captured_at'])

    def test_a_record_confirming_again_is_not_given_the_old_sample(
            self, proc_output, fault_manager_node):
        self._report('XQ.AGAIN', '/q/a')
        self._wait_capture_started(proc_output, fault_manager_node, 'XQ.AGAIN')
        self._report('XQ.AGAIN', '/q/b')

        # The record is acknowledged and confirms again while the first capture still waits on
        # the silent topic. That capture sampled the previous occurrence.
        self._clear('XQ.AGAIN', '/q/a')
        confirmed_again = time.time()
        self._report('XQ.AGAIN', '/q/a')

        old = self._wait_snapshot('XQ.AGAIN', '/q/b')
        again = self._wait_snapshot('XQ.AGAIN', '/q/a')
        self.assertNotEqual(again['captured_at'], old['captured_at'],
                            'the record was given the sample of its previous occurrence')
        self.assertGreater(again['captured_at'], confirmed_again)


@launch_testing.post_shutdown_test()
class TestSnapshotQueueShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE)
        os.unlink(SNAPSHOTS_FILE)
