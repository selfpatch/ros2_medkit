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
MANAGER_PARAMS = {
    'storage_type': 'memory',
    'confirmation_threshold': -1,
    'healing_enabled': True,
    'healing_threshold': 0,
}


def generate_test_description():
    fault_manager = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[MANAGER_PARAMS],
    )
    return LaunchDescription([fault_manager, launch_testing.actions.ReadyToTest()])


def probe_path():
    return os.path.join(get_package_prefix('ros2_medkit_fault_reporter'),
                        'lib', 'ros2_medkit_fault_reporter', 'delivery_probe_node')


class Manager:
    """A fault_manager_node process named name, so its services are /<name>/..."""

    def __init__(self, name):
        args = [os.path.join(get_package_prefix('ros2_medkit_fault_manager'),
                             'lib', 'ros2_medkit_fault_manager', 'fault_manager_node'),
                '--ros-args', '-r', f'__node:={name}']
        for key, value in MANAGER_PARAMS.items():
            args += ['-p', f'{key}:={value}']
        self.proc = subprocess.Popen(args)

    def kill(self):
        """SIGKILL, which also ends a stopped process: requests it has not read are lost."""
        if self.proc.poll() is None:
            self.proc.kill()
        self.proc.wait()

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=30)
        except subprocess.TimeoutExpired:
            self.kill()


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

    def faults(self, prefix, lister=None):
        lister = lister or self.lister
        req = ListFaults.Request()
        req.filter_by_severity = False
        req.statuses = ALL_STATUSES
        fut = lister.call_async(req)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        result = fut.result()
        found = result.faults if result is not None else []
        return {f.fault_code: f for f in found if f.fault_code.startswith(prefix)}

    def wait_faults(self, prefix, predicate, timeout=30.0, lister=None):
        deadline = time.time() + timeout
        found = {}
        while time.time() < deadline:
            found = self.faults(prefix, lister)
            if predicate(found):
                break
            time.sleep(0.2)
        return found

    def wait_clients_gone(self, service, timeout=30.0):
        """Poll the graph until the probe node has no client of service; return what is left."""
        deadline = time.time() + timeout
        left = []
        while time.time() < deadline:
            clients = self.node.get_client_names_and_types_by_node('delivery_probe', '/')
            left = [n for n, _ in clients if n == service]
            if not left:
                break
            time.sleep(0.2)
        return left

    def restart_case(self, prefix, name, count, **params):
        """
        Report to a stopped manager, kill it, start a new one: every report must reach the new one.

        The killed manager never read the requests, so each one arrives only if it is sent again.
        """
        lister = self.node.create_client(ListFaults, f'/{name}/list_faults')
        first = Manager(name)
        second = None
        probe = None
        try:
            self.assertTrue(lister.wait_for_service(timeout_sec=20.0))
            # A budget of 30 s, well past the start of a new manager.
            probe = stalled_probe(
                first.proc.pid, f'/tmp/delivery_go_{name}', count=count, prefix=prefix,
                service=f'/{name}/report_fault',
                fault_reporter__delivery__ack_timeout_sec=0.3,
                fault_reporter__delivery__max_attempts=100, **params)
            probe.wait_for(lambda line: line == f'REPORTED pending={count}')
            if params.get('destroy') == 'true':
                probe.wait_for(lambda line: line == 'DESTROYED')
            first.kill()
            second = Manager(name)
            found = self.wait_faults(prefix + '_', lambda f: len(f) == count,
                                     timeout=60.0, lister=lister)
            self.assertEqual(sorted(found), sorted(f'{prefix}_{i}' for i in range(count)))
            if params.get('destroy') == 'true':
                # Once confirmed, the reports of destroyed reporters free their clients.
                self.assertEqual(self.wait_clients_gone(f'/{name}/report_fault'), [])
            else:
                probe.wait_for(lambda line: line == 'PENDING 0')
            self.assertEqual(probe.count('dropping the report'), 0,
                             [line for line in probe.seen if 'dropping' in line])
        finally:
            if probe is not None:
                probe.stop()
            first.kill()
            if second is not None:
                second.stop()
            self.node.destroy_client(lister)

    def test_reports_reach_a_restarted_manager(self):
        """Reports pending when the manager dies reach the manager started after it."""
        self.restart_case('RESTART', 'fm_restart', count=6)

    def test_raw_interfaces_reporter_sends_a_lone_report_again(self):
        """A reporter built from node interfaces without timers sends its one report again."""
        self.restart_case('RAW', 'fm_raw', count=1, reporters=1, ctor='raw')

    def test_destroyed_reporters_deliver_their_pending_reports(self):
        """The pending reports of destroyed reporters are sent until the manager confirms them."""
        self.restart_case('GONE', 'fm_gone', count=6, destroy='true')

    def test_reports_pending_at_shutdown_are_dropped_with_a_warning(self):
        """At shutdown each report still waiting for the manager is dropped with a warning."""
        count = 3
        for destroy in ('false', 'true'):
            prefix = 'DOWN_GONE' if destroy == 'true' else 'DOWN_LIVE'
            with self.subTest(destroy=destroy):
                probe = None
                try:
                    probe = stalled_probe(
                        self.manager_pid, f'/tmp/delivery_go_down_{destroy}', count=count,
                        prefix=prefix, destroy=destroy,
                        fault_reporter__delivery__ack_timeout_sec=60.0)
                    probe.wait_for(lambda line: line == f'REPORTED pending={count}')
                    if destroy == 'true':
                        probe.wait_for(lambda line: line == 'DESTROYED')
                    probe.proc.send_signal(signal.SIGINT)
                    for i in range(count):
                        probe.wait_for(
                            lambda line, i=i: f"dropping the report for '{prefix}_{i}'" in line)
                    self.assertEqual(probe.stop(exits_alone=True), 0)
                finally:
                    os.kill(self.manager_pid, signal.SIGCONT)
                    if probe is not None:
                        probe.stop()

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
