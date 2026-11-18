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

"""A SIGINT or SIGTERM during the gateway's startup ends it like one after startup: exit 0.

Each run starts the real ``gateway_node`` and signals it a set delay after a startup log line.
The lines span the node's construction and the steps between the executor and spin. The last
runs signal a gateway that is already spinning.
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

from ros2_medkit_test_utils.constants import get_test_port, get_time_scale
from ros2_medkit_test_utils.coverage import get_coverage_env

TIME_SCALE = get_time_scale()
LINE_TIMEOUT_SEC = 30.0 * TIME_SCALE
EXIT_TIMEOUT_SEC = 30.0 * TIME_SCALE

# Startup log lines in the order main() logs them.
TRIGGERS = [
    r'Initializing ROS 2 Medkit Gateway',
    r'Loaded \d+ plugin\(s\)',
    r'ROS 2 Medkit Gateway ready on',
    r'Main executor bounded to',
    r'TopicDataProvider attached',
]
DELAYS_MS = [0, 1, 3, 10]
# After the last startup line: the gateway is spinning.
SPINNING_DELAY_MS = 1000
# get_test_port offsets this test may use.
PORT_OFFSETS = 10
# prctl option: the kernel sends the child this signal when the test process dies.
PR_SET_PDEATHSIG = 1


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
    """A gateway_node process whose merged output is read on a thread."""

    def __init__(self, port):
        env = os.environ.copy()
        env.update(get_coverage_env())
        env['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'
        self.proc = subprocess.Popen(
            [_gateway_binary(), '--ros-args', '-p', f'server.port:={port}'],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
            preexec_fn=_die_with_parent)
        self.lines = []
        self._seen = threading.Condition()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stdout:
            with self._seen:
                self.lines.append(line)
                self._seen.notify_all()

    def wait_line(self, pattern, timeout):
        regex = re.compile(pattern)
        deadline = time.monotonic() + timeout
        with self._seen:
            while not any(regex.search(line) for line in self.lines):
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self.proc.poll() is not None:
                    return False
                self._seen.wait(min(remaining, 0.05))
        return True

    def output(self):
        with self._seen:
            return ''.join(self.lines)

    def close(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()
        self._reader.join(timeout=5)


class TestGatewayStartupSignal(unittest.TestCase):

    def _run(self, run, trigger, delay_ms, signum):
        gateway = _Gateway(get_test_port(run % PORT_OFFSETS))
        try:
            self.assertTrue(
                gateway.wait_line(trigger, LINE_TIMEOUT_SEC),
                f'the gateway never logged {trigger!r}:\n{gateway.output()}')
            time.sleep(delay_ms / 1000.0)
            gateway.proc.send_signal(signum)
            returncode = gateway.proc.wait(timeout=EXIT_TIMEOUT_SEC)
        finally:
            gateway.close()
        self.assertEqual(
            returncode, 0,
            f'{signal.Signals(signum).name} {delay_ms} ms after {trigger!r} ended the gateway '
            f'with {returncode}:\n{gateway.output()[-3000:]}')

    def test_a_signal_during_startup_exits_cleanly(self):
        run = 0
        for trigger in TRIGGERS:
            for delay_ms in DELAYS_MS:
                with self.subTest(trigger=trigger, delay_ms=delay_ms, signal='SIGINT'):
                    self._run(run, trigger, delay_ms, signal.SIGINT)
                run += 1
        for delay_ms in DELAYS_MS:
            with self.subTest(trigger=TRIGGERS[3], delay_ms=delay_ms, signal='SIGTERM'):
                self._run(run, TRIGGERS[3], delay_ms, signal.SIGTERM)
            run += 1
        for signum in (signal.SIGINT, signal.SIGTERM):
            with self.subTest(trigger=TRIGGERS[-1], delay_ms=SPINNING_DELAY_MS,
                              signal=signal.Signals(signum).name):
                self._run(run, TRIGGERS[-1], SPINNING_DELAY_MS, signum)
            run += 1
