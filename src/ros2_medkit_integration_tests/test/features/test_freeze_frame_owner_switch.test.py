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

"""A shared record and a per-source record of one code each keep their own frame.

The fault manager first runs with the code shared, so App A's report opens the
shared record, owner "*". It restarts on the same storage with the code
per_source, and A's next report opens the record owned by A next to it. Both
records hold A as a reporting source. After a gateway restart, the App's fault
detail serves the frame of whichever record it presents: the per-source one
while both are confirmed, the shared one once the per-source record is cleared.
"""

import os
import signal
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
    captured_at,
    FaultManagerCalls,
    gate_process,
    open_gate,
    route_plugin_gateway,
    TIMEOUT_SEC,
    wait_frames,
)
from ros2_medkit_test_utils.launch_helpers import create_fault_manager_node

APP_A = 'test_route_plc_app'
CODE = 'FF_OWNER_SWITCH'
SHARED = '*'

PORTS = [get_test_port(0), get_test_port(1)]
TAG = f'{os.getpid()}'
TMP = tempfile.gettempdir()
STORE_PATH = os.path.join(TMP, f'test_freeze_frame_owner_switch_{TAG}.db')
FAULTS_PATH = os.path.join(TMP, f'test_freeze_frame_owner_switch_faults_{TAG}.db')
FM_GATE = os.path.join(TMP, f'test_freeze_frame_owner_switch_fm_gate_{TAG}')
GW_GATE = os.path.join(TMP, f'test_freeze_frame_owner_switch_gw_gate_{TAG}')

PER_SOURCE_FILE = tempfile.NamedTemporaryFile(
    'w', suffix='.yaml', prefix='freeze_frame_owner_switch_', delete=False)
PER_SOURCE_FILE.write(f'{CODE}:\n  per_source: true\n')
PER_SOURCE_FILE.close()


def _fault_manager(per_source):
    params = {'confirmation_threshold': -1, 'database_path': FAULTS_PATH}
    if per_source:
        params['fault_thresholds.config_file'] = PER_SOURCE_FILE.name
    return create_fault_manager_node(
        storage_type='sqlite', rosbag_enabled=False, snapshot_topics=[''], extra_params=params)


def generate_test_description():
    for path in (STORE_PATH, FAULTS_PATH, FM_GATE, GW_GATE):
        if os.path.exists(path):
            os.unlink(path)
    shared_fm = _fault_manager(per_source=False)
    per_source_fm = _fault_manager(per_source=True)
    gateways = [route_plugin_gateway(f'freeze_frame_owner_switch_gateway_{life}', PORTS[life],
                                     STORE_PATH)
                for life in range(2)]
    fm_gate = gate_process('gate_fm', FM_GATE)
    gw_gate = gate_process('gate_gw', GW_GATE)
    return LaunchDescription([
        shared_fm,
        gateways[0],
        fm_gate,
        RegisterEventHandler(OnProcessExit(target_action=fm_gate, on_exit=[per_source_fm])),
        gw_gate,
        RegisterEventHandler(OnProcessExit(target_action=gw_gate, on_exit=[gateways[1]])),
        launch_testing.actions.ReadyToTest(),
    ]), {'shared_fm': shared_fm, 'gateway_0': gateways[0]}


class TestFreezeFrameOwnerSwitch(unittest.TestCase):
    """Two records of one code that share their reporting source."""

    frames = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('freeze_frame_owner_switch_client')
        cls.fm = FaultManagerCalls(cls.node)
        cls.fm.ready()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        for path in (STORE_PATH, FAULTS_PATH, FM_GATE, GW_GATE, PER_SOURCE_FILE.name):
            if os.path.exists(path):
                os.unlink(path)

    def _stop(self, proc_info, process):
        os.kill(process.process_details['pid'], signal.SIGINT)
        proc_info.assertWaitForShutdown(process=process, timeout=TIMEOUT_SEC)

    def test_1_the_shared_record_is_framed(self):
        self.fm.wait_event_route(PORTS[0], APP_A, 'FF_OWNER_SWITCH_SENTINEL_0')
        self.fm.report(CODE, APP_A)
        found = wait_frames(PORTS[0], f'apps/{APP_A}', CODE, [APP_A])
        self.assertEqual(set(self.fm.records(CODE)), {SHARED})
        type(self).frames = {'shared': captured_at(found['frames'][APP_A])}

    def test_2_the_per_source_record_is_framed_next_to_it(self, proc_info, shared_fm):
        self.assertTrue(self.frames, 'the shared record was not framed')
        self._stop(proc_info, shared_fm)
        open_gate(FM_GATE)
        self.fm.ready()
        self.fm.wait_event_route(PORTS[0], APP_A, 'FF_OWNER_SWITCH_SENTINEL_1')
        self.fm.report(CODE, APP_A)
        records = self.fm.records(CODE)
        self.assertEqual(set(records), {SHARED, APP_A})
        self.assertEqual({r.status for r in records.values()}, {'CONFIRMED'})
        found = wait_frames(
            PORTS[0], f'apps/{APP_A}', CODE, [APP_A],
            ready=lambda frame: captured_at(frame) != self.frames['shared'])
        type(self).frames['per_source'] = captured_at(found['frames'][APP_A])

    def test_3_each_record_serves_its_own_frame_after_a_restart(self, proc_info, gateway_0):
        self.assertEqual(set(self.frames), {'shared', 'per_source'}, 'a record was not framed')
        self._stop(proc_info, gateway_0)
        open_gate(GW_GATE)
        self.fm.wait_event_route(PORTS[1], APP_A, 'FF_OWNER_SWITCH_SENTINEL_2')

        # Both confirmed: the detail presents the per-source record, reported last.
        found = wait_frames(PORTS[1], f'apps/{APP_A}', CODE, [APP_A])
        self.assertEqual(captured_at(found['frames'][APP_A]), self.frames['per_source'],
                         'the per-source record does not serve its own frame')
        self.assertIsNone(capture_origin(found['frames'][APP_A]))

        # The per-source record cleared: the detail presents the shared record.
        self.fm.clear(CODE, APP_A)
        records = self.fm.records(CODE)
        self.assertEqual(records[APP_A].status, 'CLEARED')
        self.assertEqual(records[SHARED].status, 'CONFIRMED')
        found = wait_frames(PORTS[1], f'apps/{APP_A}', CODE, [APP_A])
        self.assertEqual(captured_at(found['frames'][APP_A]), self.frames['shared'],
                         'the shared record does not serve its own frame')
        self.assertIsNone(capture_origin(found['frames'][APP_A]))


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            if info.process_name.startswith('gate_'):
                continue
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited {info.returncode}')
