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
A per-code healing_enabled: true survives a restart.

Global healing is off. At startup the fault manager turns stale HEALED faults
into CLEARED, and it used to do that for every code, so a restart undid a
per-code healing_enabled: true. The node is started twice here on the same
SQLite file:

- run 1: X.HEAL and X.GONE both have healing on and reach HEALED.
- run 2: only X.HEAL still has healing on. X.HEAL must stay HEALED, and X.GONE
  must become CLEARED, which shows the startup pass still runs.
"""

import os
import signal
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
from launch import LaunchDescription
import launch_testing.actions
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import GetFault, ReportFault

HEAL_ENTRY = (
    '  confirmation_threshold: -1\n'
    '  healing_enabled: true\n'
    '  healing_threshold: 1\n'
)


def generate_test_description():
    """
    Start only a placeholder process.

    The test starts the fault manager itself, twice. Launch stops the test when
    it has no process left, so keep one running.
    """
    keep_alive = launch.actions.ExecuteProcess(cmd=['sleep', '600'])
    return LaunchDescription([keep_alive, launch_testing.actions.ReadyToTest()])


class TestFaultCodeHealingRestart(unittest.TestCase):
    """Per-code healing is still honoured after a restart."""

    @classmethod
    def setUpClass(cls):
        """Create the temp dir, config files and ROS context."""
        cls.tmp = tempfile.TemporaryDirectory()
        cls.db_path = os.path.join(cls.tmp.name, 'faults.db')
        cls.run1_codes = cls._write(
            'run1_codes.yaml', 'X.HEAL:\n' + HEAL_ENTRY + 'X.GONE:\n' + HEAL_ENTRY)
        cls.run2_codes = cls._write('run2_codes.yaml', 'X.HEAL:\n' + HEAL_ENTRY)
        cls.exe = os.path.join(
            get_package_prefix('ros2_medkit_fault_manager'),
            'lib', 'ros2_medkit_fault_manager', 'fault_manager_node')
        rclpy.init()
        cls.node = Node('test_fault_code_healing_restart_client')

    @classmethod
    def tearDownClass(cls):
        """Shut down ROS and remove the temp dir."""
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.tmp.cleanup()

    @classmethod
    def _write(cls, name, text):
        path = os.path.join(cls.tmp.name, name)
        with open(path, 'w') as f:
            f.write(text)
        return path

    def _start(self, codes_file):
        """Start the fault manager and wait for its services."""
        params = self._write('params.yaml', (
            'fault_manager:\n'
            '  ros__parameters:\n'
            '    storage_type: sqlite\n'
            f'    database_path: {self.db_path}\n'
            '    confirmation_threshold: -1\n'
            '    healing_enabled: false\n'
            f'    fault_thresholds.config_file: {codes_file}\n'
        ))
        proc = subprocess.Popen(
            [self.exe, '--ros-args', '-r', '__node:=fault_manager', '--params-file', params])
        report = self.node.create_client(ReportFault, '/fault_manager/report_fault')
        get = self.node.create_client(GetFault, '/fault_manager/get_fault')
        self.assertTrue(report.wait_for_service(timeout_sec=15.0), 'report_fault not available')
        self.assertTrue(get.wait_for_service(timeout_sec=15.0), 'get_fault not available')
        # Right after a restart the graph can still list the old node's service,
        # so wait until the new node actually answers.
        req = GetFault.Request()
        req.fault_code = 'NOT.A.FAULT'
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            future = get.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            if future.result() is not None:
                break
        else:
            self.fail('fault_manager did not answer after start')
        return proc, report, get

    def _stop(self, proc, clients):
        """Stop the fault manager cleanly and check it exited with 0."""
        for client in clients:
            self.node.destroy_client(client)
        proc.send_signal(signal.SIGINT)
        self.assertEqual(proc.wait(timeout=30), 0)
        # Let discovery forget the old services before the next run.
        time.sleep(1.0)

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        self.assertIsNotNone(future.result(), 'Service call timed out')
        return future.result()

    def _report(self, client, code, event_type):
        req = ReportFault.Request()
        req.fault_code = code
        req.event_type = event_type
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'test'
        req.source_id = '/test_source'
        self.assertTrue(self._call(client, req).accepted)

    def _status(self, client, code):
        req = GetFault.Request()
        req.fault_code = code
        resp = self._call(client, req)
        self.assertTrue(resp.success, resp.error_message)
        return resp.fault.status

    def test_per_code_healing_survives_restart(self):
        """X.HEAL stays HEALED after a restart; X.GONE is cleared."""
        proc, report, get = self._start(self.run1_codes)
        try:
            for code in ('X.HEAL', 'X.GONE'):
                self._report(report, code, ReportFault.Request.EVENT_FAILED)
                self.assertEqual(self._status(get, code), Fault.STATUS_CONFIRMED)
                for _ in range(3):
                    if self._status(get, code) == Fault.STATUS_HEALED:
                        break
                    self._report(report, code, ReportFault.Request.EVENT_PASSED)
                self.assertEqual(self._status(get, code), Fault.STATUS_HEALED)
        finally:
            self._stop(proc, (report, get))

        proc, report, get = self._start(self.run2_codes)
        try:
            self.assertEqual(self._status(get, 'X.HEAL'), Fault.STATUS_HEALED)
            self.assertEqual(self._status(get, 'X.GONE'), Fault.STATUS_CLEARED)
        finally:
            self._stop(proc, (report, get))
