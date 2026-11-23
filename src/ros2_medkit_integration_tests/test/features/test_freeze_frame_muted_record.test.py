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

"""The startup catch-up skips a muted record, not every record of its code.

A per_source code is confirmed by App B, then a root cause opens a correlation
window, and App A confirms the same code inside it: A's record is muted, B's is
not. The gateway starts after that. Its catch-up frames B's record and leaves
A's record without a frame, as a live confirm would.
"""

import os
import tempfile
import unittest

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_test_port
from ros2_medkit_test_utils.freeze_frames import (
    capture_origin,
    detail_frames,
    FaultManagerCalls,
    gate_process,
    open_gate,
    route_plugin_gateway,
    TIMEOUT_SEC,
    wait_frames,
)
from ros2_medkit_test_utils.launch_helpers import create_fault_manager_node

APP_A = 'test_route_plc_app'
APP_B = 'test_route_plc_down_app'
CODE = 'FF_MUTED_SYMPTOM'
ROOT = 'FF_MUTED_ROOT'
PORT = get_test_port(0)
TAG = f'{os.getpid()}'
STORE_PATH = os.path.join(tempfile.gettempdir(), f'test_freeze_frame_muted_record_{TAG}.db')
GATE = os.path.join(tempfile.gettempdir(), f'test_freeze_frame_muted_record_gate_{TAG}')


def _temp_file(prefix, text):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(text)
    handle.close()
    return handle.name


PER_SOURCE_FILE = _temp_file('freeze_frame_muted_', f'{CODE}:\n  per_source: true\n')
CORRELATION_FILE = _temp_file('freeze_frame_muted_rules_', f"""\
correlation:
  enabled: true
  default_window_ms: 60000
  patterns:
    symptom:
      codes: ["{CODE}"]
  rules:
    - id: freeze_frame_mute
      name: "Freeze frame mute"
      mode: hierarchical
      root_cause:
        codes: ["{ROOT}"]
      symptoms:
        - pattern: symptom
      window_ms: 60000
      mute_symptoms: true
""")


def generate_test_description():
    for path in (STORE_PATH, GATE):
        if os.path.exists(path):
            os.unlink(path)
    gateway = route_plugin_gateway('freeze_frame_muted_record_gateway', PORT, STORE_PATH)
    gate = gate_process('gate_gw', GATE)
    return LaunchDescription([
        create_fault_manager_node(
            rosbag_enabled=False, snapshot_topics=[''],
            extra_params={
                'confirmation_threshold': -1,
                'fault_thresholds.config_file': PER_SOURCE_FILE,
                'correlation.config_file': CORRELATION_FILE,
            }),
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[gateway])),
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway': gateway}


class TestFreezeFrameMutedRecord(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('freeze_frame_muted_record_client')
        cls.fm = FaultManagerCalls(cls.node)
        cls.fm.ready()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        for path in (STORE_PATH, GATE, PER_SOURCE_FILE, CORRELATION_FILE):
            if os.path.exists(path):
                os.unlink(path)

    def test_the_catch_up_frames_the_unmuted_record_only(self, proc_output, gateway):
        self.fm.report(CODE, APP_B)
        self.fm.report(ROOT, 'freeze_frame_muted_root_source')
        self.fm.report(CODE, APP_A)
        listing = self.fm.listing()
        muted = {(m.fault_code, m.source_id) for m in listing.muted_faults}
        self.assertIn((CODE, APP_A), muted)
        self.assertNotIn((CODE, APP_B), muted)
        records = {f.owner: f.status for f in listing.faults if f.fault_code == CODE}
        self.assertEqual(records, {APP_A: 'CONFIRMED', APP_B: 'CONFIRMED'})
        open_gate(GATE)

        wait_frames(PORT, f'apps/{APP_B}', CODE, [APP_B],
                    ready=lambda frame: capture_origin(frame) == 'startup')
        proc_output.assertWaitFor('that were already confirmed at startup', process=gateway,
                                  timeout=TIMEOUT_SEC)
        found = detail_frames(PORT, f'apps/{APP_A}', CODE)
        self.assertIsNotNone(found, "A's muted record has no fault detail")
        self.assertEqual(found['frames'], {}, "A's muted record was framed")


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            if info.process_name.startswith('gate_'):
                continue
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited {info.returncode}')
