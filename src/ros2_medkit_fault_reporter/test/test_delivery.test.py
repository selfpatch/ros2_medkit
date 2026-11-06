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

"""FaultReporter delivery against a real fault_manager_node."""

import os
import queue
import signal
import subprocess
import sys
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ListFaults

ALL_STATUSES = [Fault.STATUS_CONFIRMED, Fault.STATUS_HEALED]


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
    )
    return LaunchDescription([fault_manager, launch_testing.actions.ReadyToTest()])


def probe_path():
    return os.path.join(get_package_prefix('ros2_medkit_fault_reporter'),
                        'lib', 'ros2_medkit_fault_reporter', 'delivery_probe_node')


def find_pid(name):
    """PID of the process of this test's ROS domain whose command line holds name."""
    domain = b'ROS_DOMAIN_ID=' + os.environ['ROS_DOMAIN_ID'].encode()
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
        if name.encode() in cmd and domain in env:
            return int(entry)
    raise AssertionError(f'{name} not found')


class Probe:
    """A delivery_probe_node process with its stdout read on a thread."""

    def __init__(self, **params):
        args = [probe_path(), '--ros-args']
        for key, value in params.items():
            args += ['-p', f'{key.replace("__", ".")}:={value}']
        self.proc = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        self.lines = queue.Queue()
        self.seen = []
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for line in self.proc.stdout:
            self.lines.put(line.rstrip())

    def wait_for(self, predicate, timeout=60.0):
        """Return the first output line (new or already seen) for which predicate is true."""
        deadline = time.time() + timeout
        index = 0
        while time.time() < deadline:
            while index < len(self.seen):
                if predicate(self.seen[index]):
                    return self.seen[index]
                index += 1
            try:
                self.seen.append(self.lines.get(timeout=0.2))
            except queue.Empty:
                pass
        raise AssertionError(f'no matching output; saw {self.seen[-10:]}')

    def count(self, text):
        while True:
            try:
                self.seen.append(self.lines.get_nowait())
            except queue.Empty:
                break
        return sum(1 for line in self.seen if text in line)

    def stop(self, exits_alone=False):
        if exits_alone:
            try:
                self.proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                pass
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()
        return self.proc.returncode


def stalled_probe(manager_pid, go_file, **params):
    """Start a probe, stop the manager once the reporters are ready, then let the probe report."""
    if os.path.exists(go_file):
        os.remove(go_file)
    probe = Probe(mode='stalled', go_file=go_file, **params)
    probe.wait_for(lambda line: line == 'READY')
    os.kill(manager_pid, signal.SIGSTOP)
    open(go_file, 'w').close()
    return probe


class TestDelivery(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('delivery_test_client')
        cls.lister = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        assert cls.lister.wait_for_service(timeout_sec=20.0)
        cls.manager_pid = find_pid('fault_manager_node')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def faults(self, prefix):
        req = ListFaults.Request()
        req.filter_by_severity = False
        req.statuses = ALL_STATUSES
        fut = self.lister.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        result = fut.result()
        found = result.faults if result is not None else []
        return {f.fault_code: f for f in found if f.fault_code.startswith(prefix)}

    def wait_faults(self, prefix, predicate, timeout=30.0):
        deadline = time.time() + timeout
        found = {}
        while time.time() < deadline:
            found = self.faults(prefix)
            if predicate(found):
                break
            time.sleep(0.2)
        return found

    def test_burst_of_fresh_reporters_loses_nothing(self):
        """Fresh reporters that report the moment their client is ready lose no report."""
        rounds, count = 12, 24
        # Busy processes crowd the host so a request can overtake the server's match.
        load = [subprocess.Popen([sys.executable, '-c', 'while True: pass'])
                for _ in range((os.cpu_count() or 4) * 2)]
        try:
            probe = Probe(mode='burst', count=count, rounds=rounds, prefix='BURST')
            probe.wait_for(lambda line: line == 'DONE', timeout=200)
            self.assertEqual(probe.stop(exits_alone=True), 0)
        finally:
            for p in load:
                p.kill()
                p.wait()
        found = self.faults('BURST_')
        self.assertEqual(
            len(found), rounds * count,
            f'{rounds * count - len(found)} of {rounds * count} reports never reached the manager')

    def test_events_of_one_code_arrive_in_order(self):
        """FAILED then PASSED reported at once on a fresh reporter ends healed for every code."""
        count = 48
        probe = Probe(mode='order', count=count, prefix='ORDER',
                      fault_reporter__local_filtering__enabled='false')
        probe.wait_for(lambda line: line.startswith('DONE'))
        probe.stop(exits_alone=True)
        found = self.wait_faults(
            'ORDER_',
            lambda f: len(f) == count and all(
                x.status == Fault.STATUS_HEALED for x in f.values()))
        self.assertEqual(len(found), count)
        wrong = sorted(c for c, f in found.items() if f.status != Fault.STATUS_HEALED)
        self.assertEqual(wrong, [], f'{len(wrong)} codes not healed: {wrong[:5]}')

    def test_reports_wait_for_an_unresponsive_manager(self):
        """Reports made while the manager does not answer arrive once it does."""
        count = 6
        probe = None
        try:
            probe = stalled_probe(
                self.manager_pid, '/tmp/delivery_go_wait', count=count,
                fault_reporter__delivery__ack_timeout_sec=0.2,
                fault_reporter__delivery__max_attempts=100, prefix='WAIT')
            probe.wait_for(lambda line: line.startswith('REPORTED'))
        finally:
            os.kill(self.manager_pid, signal.SIGCONT)
        found = self.wait_faults('WAIT_', lambda f: len(f) == count)
        probe.wait_for(lambda line: line == 'PENDING 0')
        probe.stop()
        self.assertEqual(len(found), count)

    def test_report_is_dropped_after_the_attempt_budget(self):
        """With no response, a report is sent max_attempts times, then dropped with a warning."""
        count = 4
        probe = None
        try:
            probe = stalled_probe(
                self.manager_pid, '/tmp/delivery_go_budget', count=count,
                fault_reporter__delivery__ack_timeout_sec=0.1,
                fault_reporter__delivery__max_attempts=3, prefix='BUDGET')
            probe.wait_for(lambda line: line.startswith('REPORTED'))
            probe.wait_for(lambda line: line == 'PENDING 0')
            for i in range(count):
                probe.wait_for(lambda line, i=i: f"dropping the report for 'BUDGET_{i}'" in line
                               and 'after 3 attempts' in line)
        finally:
            os.kill(self.manager_pid, signal.SIGCONT)
            if probe is not None:
                probe.stop()

    def test_pending_reports_are_capped(self):
        """Past max_pending a new report is dropped with a warning; memory stays bounded."""
        cap, count = 8, 20
        probe = None
        try:
            probe = stalled_probe(
                self.manager_pid, '/tmp/delivery_go_cap', count=count, reporters=1,
                fault_reporter__delivery__max_pending=cap,
                fault_reporter__delivery__ack_timeout_sec=60.0, prefix='CAP')
            probe.wait_for(lambda line: line == f'REPORTED pending={cap}')
        finally:
            os.kill(self.manager_pid, signal.SIGCONT)
        found = self.wait_faults('CAP_', lambda f: len(f) >= cap)
        probe.wait_for(lambda line: line == 'PENDING 0')
        self.assertEqual(probe.count('dropping the report for'), count - cap)
        # The reports kept are the first `cap`; none after them gets in.
        self.assertEqual(sorted(found), sorted(f'CAP_{i}' for i in range(cap)))
        probe.stop()

    def test_delivery_parameters(self):
        """Each parameter is accepted at both ends of its range; a bad value gets the default."""
        default = 'CONFIG ack=0.5 attempts=10 pending=1024'
        cases = [
            ({'ack_timeout_sec': 0.05, 'max_attempts': 1, 'max_pending': 1},
             'CONFIG ack=0.05 attempts=1 pending=1'),
            ({'ack_timeout_sec': 60.0, 'max_attempts': 100, 'max_pending': 100000},
             'CONFIG ack=60 attempts=100 pending=100000'),
            ({'ack_timeout_sec': '.nan'}, default),
            ({'ack_timeout_sec': -1.0}, default),
            ({'ack_timeout_sec': 0.04}, default),
            ({'ack_timeout_sec': 60.5}, default),
            ({'max_attempts': 0}, default),
            ({'max_attempts': 101}, default),
            ({'max_attempts': 4294967306}, default),
            ({'max_pending': -5}, default),
            ({'max_pending': 100001}, default),
            ({'max_pending': 4294967297}, default),
        ]
        for params, expected in cases:
            with self.subTest(params=params):
                probe = Probe(mode='config', **{
                    f'fault_reporter__delivery__{k}': v for k, v in params.items()})
                line = probe.wait_for(lambda line: line.startswith('CONFIG'), timeout=30)
                self.assertEqual(probe.stop(exits_alone=True), 0)
                self.assertEqual(line, expected)
