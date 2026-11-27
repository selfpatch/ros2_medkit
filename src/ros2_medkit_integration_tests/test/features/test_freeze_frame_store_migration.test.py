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

"""A freeze-frame store written before rows named their record is served after an upgrade.

Before the store keyed its rows by record, a row was keyed by fault code and entity, and
every record was shared. The test writes such a file with that schema and one row of the
shared record App A reported, then starts the gateway on it. The gateway serves that
frame, as stored, for the shared record.
"""

import os
import sqlite3
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
CODE = 'FF_STORE_MIGRATION'
PORT = get_test_port(0)
TAG = f'{os.getpid()}'
STORE_PATH = os.path.join(tempfile.gettempdir(), f'test_freeze_frame_store_migration_{TAG}.db')
GATE = os.path.join(tempfile.gettempdir(), f'test_freeze_frame_store_migration_gate_{TAG}')

# The table as the store created it before rows named their record.
EARLIER_SCHEMA = """
    CREATE TABLE IF NOT EXISTS entity_freeze_frames (
      fault_code     TEXT NOT NULL,
      entity_id      TEXT NOT NULL,
      frame          TEXT NOT NULL,
      captured_at_ns INTEGER NOT NULL,
      source         TEXT NOT NULL DEFAULT '',
      capture_origin TEXT NOT NULL DEFAULT '',
      occurrence_count  INTEGER NOT NULL DEFAULT 0,
      first_occurred_ns INTEGER NOT NULL DEFAULT 0,
      PRIMARY KEY (fault_code, entity_id)
    );
"""
STORED_AT_NS = 1_700_000_000_123_000_000
STORED_AT = '2023-11-14T22:13:20.123Z'
STORED_LEVEL = 12.5


def generate_test_description():
    for path in (STORE_PATH, GATE):
        if os.path.exists(path):
            os.unlink(path)
    gateway = route_plugin_gateway('freeze_frame_store_migration_gateway', PORT, STORE_PATH)
    gate = gate_process('gate_gw', GATE)
    return LaunchDescription([
        create_fault_manager_node(rosbag_enabled=False, snapshot_topics=[''],
                                  extra_params={'confirmation_threshold': -1}),
        gate,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[gateway])),
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway': gateway}


def _occurrence_ns(stamp):
    """Nanoseconds of a Fault time, rounded as the gateway rounds first_occurred."""
    return int((stamp.sec + stamp.nanosec / 1e9) * 1e9)


class TestFreezeFrameStoreMigration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('freeze_frame_store_migration_client')
        cls.fm = FaultManagerCalls(cls.node)
        cls.fm.ready()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        for path in (STORE_PATH, GATE):
            if os.path.exists(path):
                os.unlink(path)

    def test_a_row_of_the_earlier_schema_is_served_for_the_shared_record(self, proc_output,
                                                                         gateway):
        self.fm.report(CODE, APP_A)
        shared = self.fm.records(CODE)['*']
        db = sqlite3.connect(STORE_PATH)
        db.executescript(EARLIER_SCHEMA)
        db.execute(
            'INSERT INTO entity_freeze_frames (fault_code, entity_id, frame, captured_at_ns, '
            'source, capture_origin, occurrence_count, first_occurred_ns) '
            'VALUES (?, ?, ?, ?, ?, ?, ?, ?)',
            (CODE, APP_A, '{"values": {"level": %s, "alarm": false}, "connected": true}'
             % STORED_LEVEL, STORED_AT_NS, 'plugin_x_plc_data_route', '',
             shared.occurrence_count, _occurrence_ns(shared.first_occurred)))
        db.commit()
        db.close()
        open_gate(GATE)

        self.fm.wait_event_route(PORT, APP_A, 'FF_STORE_MIGRATION_SENTINEL')
        found = wait_frames(PORT, f'apps/{APP_A}', CODE, [APP_A])
        frame = found['frames'][APP_A]
        self.assertEqual(captured_at(frame), STORED_AT, f'the stored row was not served: {frame}')
        self.assertEqual(frame['data'].get('level'), STORED_LEVEL)
        self.assertIsNone(capture_origin(frame))
        proc_output.assertWaitFor(
            'Entity freeze-frame store: moved 1 row(s) of the earlier schema to the shared '
            "record ('*') of their code",
            process=gateway, timeout=TIMEOUT_SEC)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            if info.process_name.startswith('gate_'):
                continue
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited {info.returncode}')
