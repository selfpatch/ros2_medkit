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

"""Entity freeze frames are kept per fault record across gateway restarts.

One fault manager runs for the whole test. Three gateway lives share one
freeze-frame store file; each life starts when the test opens its gate, after
the previous life has exited.

- Life 1 captures: a per_source code from two Apps, a shared code reported by
  the Component and framed from the Apps it hosts, and a per_source code of
  one App.
- Life 2 reloads them: every record serves its own frame from life 1.
- Between life 2 and life 3 one record of the per_source code is cleared and
  raised again. Life 3 drops only that record's stored frame.

A frame served from the store keeps the captured_at of its capture and has no
capture_origin; a frame read again at start has capture_origin "startup".
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
APP_B = 'test_route_plc_down_app'
COMPONENT = 'test_route_plc'

PER_SOURCE_CODE = 'FF_REC_PER_SOURCE'
COMPONENT_CODE = 'FF_REC_COMPONENT'
APP_OWN_CODE = 'FF_REC_APP_OWN'

LIVES = 3
PORTS = [get_test_port(i) for i in range(LIVES)]
TAG = f'{os.getpid()}'
STORE_PATH = os.path.join(tempfile.gettempdir(), f'test_freeze_frame_records_restart_{TAG}.db')
GATES = [os.path.join(tempfile.gettempdir(), f'test_freeze_frame_records_gate_{TAG}_{i}')
         for i in range(LIVES)]

PER_SOURCE_FILE = tempfile.NamedTemporaryFile(
    'w', suffix='.yaml', prefix='freeze_frame_records_', delete=False)
PER_SOURCE_FILE.write(
    f'{PER_SOURCE_CODE}:\n  per_source: true\n{APP_OWN_CODE}:\n  per_source: true\n')
PER_SOURCE_FILE.close()


def generate_test_description():
    for path in GATES + [STORE_PATH]:
        if os.path.exists(path):
            os.unlink(path)
    fault_manager = create_fault_manager_node(
        rosbag_enabled=False,
        snapshot_topics=[''],
        extra_params={
            'confirmation_threshold': -1,
            'fault_thresholds.config_file': PER_SOURCE_FILE.name,
        },
    )
    gateways = [route_plugin_gateway(f'freeze_frame_records_gateway_{life}', PORTS[life],
                                     STORE_PATH)
                for life in range(LIVES)]
    actions = [fault_manager, gateways[0]]
    for life in range(1, LIVES):
        gate = gate_process(f'gate_{life}', GATES[life])
        start = OnProcessExit(target_action=gate, on_exit=[gateways[life]])
        actions += [gate, RegisterEventHandler(start)]
    actions.append(launch_testing.actions.ReadyToTest())
    return LaunchDescription(actions), {f'gateway_{life}': gateways[life] for life in range(LIVES)}


class TestFreezeFrameRecordsRestart(unittest.TestCase):
    """Per-record freeze frames across gateway restarts."""

    life1 = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('freeze_frame_records_client')
        cls.fm = FaultManagerCalls(cls.node)
        cls.fm.ready()

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        for path in GATES + [STORE_PATH, PER_SOURCE_FILE.name]:
            if os.path.exists(path):
                os.unlink(path)

    def _stop(self, proc_info, gateway):
        os.kill(gateway.process_details['pid'], signal.SIGINT)
        proc_info.assertWaitForShutdown(process=gateway, timeout=TIMEOUT_SEC)

    def test_1_life_one_frames_every_record(self):
        """Each record gets its own frames in life 1."""
        self.fm.wait_event_route(PORTS[0], APP_A, 'FF_REC_SENTINEL_0')
        self.fm.report(PER_SOURCE_CODE, APP_A)
        self.fm.report(PER_SOURCE_CODE, APP_B)
        self.fm.report(COMPONENT_CODE, COMPONENT)
        self.fm.report(APP_OWN_CODE, APP_A)

        a = wait_frames(PORTS[0], f'apps/{APP_A}', PER_SOURCE_CODE, [APP_A])
        b = wait_frames(PORTS[0], f'apps/{APP_B}', PER_SOURCE_CODE, [APP_B])
        component = wait_frames(PORTS[0], f'components/{COMPONENT}', COMPONENT_CODE,
                                [APP_A, APP_B])
        own = wait_frames(PORTS[0], f'apps/{APP_A}', APP_OWN_CODE, [APP_A])
        self.assertEqual(set(a['frames']), {APP_A})
        self.assertEqual(set(b['frames']), {APP_B})
        self.assertEqual(set(own['frames']), {APP_A})
        type(self).life1 = {
            'a': captured_at(a['frames'][APP_A]),
            'b': captured_at(b['frames'][APP_B]),
            'component': {e: captured_at(component['frames'][e]) for e in (APP_A, APP_B)},
            'own': captured_at(own['frames'][APP_A]),
            'occurrence_a': a['occurrence'],
            'occurrence_b': b['occurrence'],
        }

    def test_2_every_record_comes_back_after_a_restart(self, proc_info, gateway_0):
        """P1 and P2: after a restart each record serves its own stored frame."""
        self.assertTrue(self.life1, 'life 1 did not run')
        self._stop(proc_info, gateway_0)
        open_gate(GATES[1])
        self.fm.wait_event_route(PORTS[1], APP_A, 'FF_REC_SENTINEL_1')

        # P1: the two records of one per_source code keep their own frame and occurrence.
        a = wait_frames(PORTS[1], f'apps/{APP_A}', PER_SOURCE_CODE, [APP_A])
        b = wait_frames(PORTS[1], f'apps/{APP_B}', PER_SOURCE_CODE, [APP_B])
        self.assertEqual(set(a['frames']), {APP_A})
        self.assertEqual(set(b['frames']), {APP_B})
        self.assertEqual(captured_at(a['frames'][APP_A]), self.life1['a'],
                         'A was not served from the store')
        self.assertEqual(captured_at(b['frames'][APP_B]), self.life1['b'],
                         'B was not served from the store')
        self.assertIsNone(capture_origin(a['frames'][APP_A]))
        self.assertIsNone(capture_origin(b['frames'][APP_B]))
        self.assertEqual(a['occurrence'], self.life1['occurrence_a'])
        self.assertEqual(b['occurrence'], self.life1['occurrence_b'])

        # P2: the Component's record and the App's own record of another code both come back.
        component = wait_frames(PORTS[1], f'components/{COMPONENT}', COMPONENT_CODE,
                                [APP_A, APP_B])
        own = wait_frames(PORTS[1], f'apps/{APP_A}', APP_OWN_CODE, [APP_A])
        for entity in (APP_A, APP_B):
            self.assertEqual(captured_at(component['frames'][entity]),
                             self.life1['component'][entity],
                             f'the Component record lost its stored frame of {entity}')
            self.assertIsNone(capture_origin(component['frames'][entity]))
        self.assertEqual(captured_at(own['frames'][APP_A]), self.life1['own'],
                         'the App record lost its stored frame')
        self.assertIsNone(capture_origin(own['frames'][APP_A]))

    def test_3_a_record_raised_again_while_down_drops_only_its_frame(
            self, proc_info, proc_output, gateway_1, gateway_2):
        """P3: per-record judging drops only the frame of the record that occurred again."""
        self.assertTrue(self.life1, 'life 1 did not run')
        self._stop(proc_info, gateway_1)
        self.fm.clear(PER_SOURCE_CODE, APP_A)
        self.fm.report(PER_SOURCE_CODE, APP_A)
        open_gate(GATES[2])

        proc_output.assertWaitFor(
            f"Entity freeze-frame for fault '{PER_SOURCE_CODE}': the stored frame of entity "
            f"'{APP_A}' is from an earlier occurrence and was discarded (stored occurrence 1, "
            'the fault manager lists occurrence 2, CONFIRMED)',
            process=gateway_2, timeout=TIMEOUT_SEC)
        self.fm.wait_event_route(PORTS[2], APP_A, 'FF_REC_SENTINEL_2')

        a = wait_frames(PORTS[2], f'apps/{APP_A}', PER_SOURCE_CODE, [APP_A],
                        ready=lambda frame: capture_origin(frame) == 'startup')
        self.assertGreater(captured_at(a['frames'][APP_A]), self.life1['a'])
        self.assertEqual(a['occurrence'], 2)
        b = wait_frames(PORTS[2], f'apps/{APP_B}', PER_SOURCE_CODE, [APP_B])
        self.assertEqual(captured_at(b['frames'][APP_B]), self.life1['b'],
                         "B's frame was dropped with A's")
        self.assertIsNone(capture_origin(b['frames'][APP_B]))
        self.assertEqual(b['occurrence'], 1)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            if info.process_name.startswith('gate_'):
                continue
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited {info.returncode}')
