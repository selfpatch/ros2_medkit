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

"""SIGINT and SIGTERM reach rclcpp's handler in the gateway within a bound after they are sent.

Each run starts a real ``gateway_node`` with a large main executor, signals it once the executor
answers a parameter request and measures until rclcpp logs ``signal_handler``. Under TSan a signal
on a thread that waits for a mutex runs only once that thread gets the mutex, so the signal must
not land on an executor thread.
"""

import ctypes
import os
import re
import signal
import subprocess
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import launch_testing.util
from rcl_interfaces.srv import ListParameters
import rclpy

from ros2_medkit_test_utils.constants import get_test_port, get_time_scale, sanitizers_enabled
from ros2_medkit_test_utils.coverage import get_coverage_env

TIME_SCALE = get_time_scale()
# test_thread_pool_starvation's ceiling: under TSan 256 threads outlast launch's grace period.
EXECUTOR_THREADS = 16 if sanitizers_enabled() else 256
STARTUP_TIMEOUT_SEC = 60.0 * TIME_SCALE
HANDLER_BOUND_SEC = 0.5 * TIME_SCALE
EXIT_TIMEOUT_SEC = 30.0 * TIME_SCALE
SIGINT_STOPS = 20
# get_test_port offsets this test may use.
PORT_OFFSETS = 10
# prctl option: the kernel sends the child this signal when the test process dies.
PR_SET_PDEATHSIG = 1
READY_LINE = re.compile(re.escape(f'Main executor bounded to {EXECUTOR_THREADS} threads'))
# Answered on the gateway's main executor, so a reply means it is spinning.
PARAMETER_SERVICE = '/ros2_medkit_gateway/list_parameters'
PARAMETER_CALL_TIMEOUT_SEC = 1.0 * TIME_SCALE
# Humble and Jazzy log signal_handler(SIGINT/SIGTERM), Lyrical logs signal_handler(signum=N).
HANDLER_LINE = re.compile(
    r'\[(\d+\.\d+)\] \[rclcpp\]: signal_handler\((?:SIGINT/SIGTERM|signum=\d+)\)')


def generate_test_description():
    # The test starts its own gateways; this keeps the launch alive while it runs.
    return LaunchDescription([launch_testing.util.KeepAliveProc(),
                              launch_testing.actions.ReadyToTest()]), {}


def _die_with_parent():
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGKILL)


def _gateway_binary():
    return os.path.join(get_package_prefix('ros2_medkit_gateway'), 'lib', 'ros2_medkit_gateway',
                        'gateway_node')


class _Gateway:
    """A gateway_node process whose output lines are read on a thread, each with its read time."""

    def __init__(self, port):
        env = os.environ.copy()
        env.update(get_coverage_env())
        env['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'
        self.proc = subprocess.Popen(
            [_gateway_binary(), '--ros-args', '-p', f'server.port:={port}',
             '-p', 'refresh_interval_ms:=1000',
             '-p', f'server.executor_threads:={EXECUTOR_THREADS}'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
            preexec_fn=_die_with_parent)
        self.lines = []
        self._eof = False
        self._seen = threading.Condition()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stdout:
            read_at = time.monotonic()
            with self._seen:
                self.lines.append((read_at, line))
                self._seen.notify_all()
        with self._seen:
            self._eof = True
            self._seen.notify_all()

    def wait_line(self, regex, timeout):
        """Return (monotonic read time, match) of the first matching line, or None."""
        deadline = time.monotonic() + timeout
        with self._seen:
            while True:
                for read_at, line in self.lines:
                    match = regex.search(line)
                    if match:
                        return read_at, match
                # EOF comes after the last line, so an exited process's output is all searched.
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._eof:
                    return None
                self._seen.wait(remaining)

    def output(self):
        with self._seen:
            return ''.join(line for _, line in self.lines)

    def close(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()
        self._reader.join(timeout=5)


class TestGatewaySignalLatency(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.probe = rclpy.create_node('_signal_latency_probe')

    @classmethod
    def tearDownClass(cls):
        cls.probe.destroy_node()
        rclpy.shutdown()

    def _executor_answers(self, gateway, timeout):
        """Return True once the gateway's main executor answers a parameter request."""
        client = self.probe.create_client(ListParameters, PARAMETER_SERVICE)
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline and gateway.proc.poll() is None:
                if not client.wait_for_service(timeout_sec=PARAMETER_CALL_TIMEOUT_SEC):
                    continue
                # A request sent before the new server is matched is lost, so it is sent again.
                future = client.call_async(ListParameters.Request())
                rclpy.spin_until_future_complete(
                    self.probe, future, timeout_sec=PARAMETER_CALL_TIMEOUT_SEC)
                if future.done():
                    return True
                client.remove_pending_request(future)
            return False
        finally:
            self.probe.destroy_client(client)

    def _stop(self, run, signum):
        """Start a gateway, signal it, return the delay to the handler line and by its stamp."""
        gateway = _Gateway(get_test_port(run % PORT_OFFSETS))
        try:
            self.assertIsNotNone(
                gateway.wait_line(READY_LINE, STARTUP_TIMEOUT_SEC),
                f'run {run}: the gateway never logged {READY_LINE.pattern!r}:\n{gateway.output()}')
            self.assertTrue(
                self._executor_answers(gateway, STARTUP_TIMEOUT_SEC),
                f'run {run}: the gateway never answered {PARAMETER_SERVICE}:\n{gateway.output()}')
            sent_wall = time.time()
            sent = time.monotonic()
            gateway.proc.send_signal(signum)
            handled = gateway.wait_line(HANDLER_LINE, EXIT_TIMEOUT_SEC)
            returncode = gateway.proc.wait(timeout=EXIT_TIMEOUT_SEC)
        finally:
            gateway.close()
        name = signal.Signals(signum).name
        self.assertIsNotNone(
            handled, f'run {run}: {name} never reached the handler:\n{gateway.output()[-3000:]}')
        self.assertEqual(
            returncode, 0,
            f'run {run}: {name} ended the gateway with {returncode}:\n{gateway.output()[-3000:]}')
        # The read time is monotonic. The log stamp is wall clock, which can step: report only.
        return handled[0] - sent, float(handled[1].group(1)) - sent_wall

    def test_a_signal_reaches_the_handler_within_the_bound(self):
        stops = [(run, signal.SIGINT) for run in range(SIGINT_STOPS)]
        stops.append((SIGINT_STOPS, signal.SIGTERM))
        delays = []
        for run, signum in stops:
            delays.append((run, signal.Signals(signum).name) + self._stop(run, signum))
        report = '\n'.join(
            f'run {run} {name}: {read * 1000:.1f} ms (log stamp {stamp * 1000:.1f} ms)'
            for run, name, read, stamp in delays)
        print(f'executor_threads={EXECUTOR_THREADS} bound={HANDLER_BOUND_SEC:.1f} s\n{report}')
        slow = [d for d in delays if d[2] > HANDLER_BOUND_SEC]
        if slow:
            self.fail(
                f'{len(slow)} of {len(delays)} signals reached the handler later than '
                f'{HANDLER_BOUND_SEC:.1f} s at executor_threads={EXECUTOR_THREADS}:\n{report}')
