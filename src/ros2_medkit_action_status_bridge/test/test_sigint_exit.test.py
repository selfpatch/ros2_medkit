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

"""The bridge exits with code 0 on SIGINT, wherever the signal lands."""

import os
import random
import signal
import subprocess
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_testing.actions

RUNS = 40


def generate_test_description():
    # The bridge runs are spawned by the test body; this idle process keeps the
    # launch alive while they do.
    idle = launch.actions.ExecuteProcess(cmd=['sleep', '600'])
    return launch.LaunchDescription([idle, launch_testing.actions.ReadyToTest()])


class TestSigintExit(unittest.TestCase):

    def test_sigint_during_rescan_exits_cleanly(self):
        """SIGINT lands in a rescan on every run; each run must exit 0."""
        exe = os.path.join(
            get_package_prefix('ros2_medkit_action_status_bridge'),
            'lib', 'ros2_medkit_action_status_bridge', 'action_status_bridge_node')
        rng = random.Random(7)
        failures = []
        for run in range(RUNS):
            # A 1 ms rescan period keeps the node inside rescan_actions() or
            # reconcile_pending() almost continuously.
            proc = subprocess.Popen(
                [exe, '--ros-args', '-p', 'rescan_period_sec:=0.001',
                 '-p', 'retry_period_sec:=0.001'],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
            try:
                proc.wait(timeout=0.6 + rng.random() * 0.4)
            except subprocess.TimeoutExpired:
                pass
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
            try:
                out, _ = proc.communicate(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
                out, _ = proc.communicate()
            if proc.returncode != 0:
                failures.append((run, proc.returncode, out[-300:]))
        self.assertEqual(
            failures, [],
            f'{len(failures)} of {RUNS} runs did not exit 0: {failures[:3]}')
