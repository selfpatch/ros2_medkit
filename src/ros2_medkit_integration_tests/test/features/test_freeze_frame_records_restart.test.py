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
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH, get_test_port
from ros2_medkit_test_utils.launch_helpers import create_fault_manager_node, create_gateway_node
from ros2_medkit_test_utils.service_round_trip import answer_once_each, call_until_answered

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

TIMEOUT_SEC = 60.0


def _plugin_path(so_name):
    prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(prefix, 'lib', 'ros2_medkit_gateway', so_name)


def _gate(name, path):
    """Return a process that exits once ``path`` exists."""
    return ExecuteProcess(
        cmd=['sh', '-c', f'while [ ! -e "{path}" ]; do sleep 0.2; done'],
        name=name, output='screen')


def _gateway(life):
    return create_gateway_node(
        name=f'freeze_frame_records_gateway_{life}',
        port=PORTS[life],
        extra_params={
            'plugins': ['test_route_data'],
            'plugins.test_route_data.path': _plugin_path('libtest_route_data_plugin.so'),
            'entity_freeze_frame.storage.path': STORE_PATH,
        },
    )


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
    gateways = [_gateway(life) for life in range(LIVES)]
    actions = [fault_manager, gateways[0]]
    for life in range(1, LIVES):
        gate = _gate(f'gate_{life}', GATES[life])
        start = OnProcessExit(target_action=gate, on_exit=[gateways[life]])
        actions += [gate, RegisterEventHandler(start)]
    actions.append(launch_testing.actions.ReadyToTest())
    return LaunchDescription(actions), {f'gateway_{life}': gateways[life] for life in range(LIVES)}


def _base(life):
    return f'http://localhost:{PORTS[life]}{API_BASE_PATH}'


def _frames(life, entity_path, code):
    """Return the freeze frames of a fault detail by entity id, or None when it is not served."""
    try:
        response = requests.get(f'{_base(life)}/{entity_path}/faults/{code}', timeout=5)
    except requests.exceptions.RequestException:
        return None
    if response.status_code != 200:
        return None
    body = response.json()
    snapshots = body.get('environment_data', {}).get('snapshots', [])
    return {
        'frames': {s.get('name'): s for s in snapshots if s.get('type') == 'freeze_frame'},
        'occurrence': body.get('x-medkit', {}).get('occurrence_count'),
    }


def _wait_frames(life, entity_path, code, entities, *, ready=lambda frame: True):
    """Poll until the detail serves a frame of each of ``entities`` that is ``ready``."""
    deadline = time.monotonic() + TIMEOUT_SEC
    last = None
    while time.monotonic() < deadline:
        last = _frames(life, entity_path, code)
        if last and all(e in last['frames'] and ready(last['frames'][e]) for e in entities):
            return last
        time.sleep(0.2)
    raise AssertionError(
        f'{entity_path}/faults/{code} on life {life}: no frames of {entities}; last {last}')


def _captured_at(frame):
    return frame.get('x-medkit', {}).get('captured_at')


def _origin(frame):
    return frame.get('x-medkit', {}).get('capture_origin')


class TestFreezeFrameRecordsRestart(unittest.TestCase):
    """Per-record freeze frames across gateway restarts."""

    life1 = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('freeze_frame_records_client')
        cls.report = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.clear = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        assert cls.report.wait_for_service(timeout_sec=TIMEOUT_SEC), 'report_fault not available'
        assert cls.clear.wait_for_service(timeout_sec=TIMEOUT_SEC), 'clear_fault not available'
        answer_once_each(cls.node, (cls.report, cls.clear), TIMEOUT_SEC)
        cls.sentinels = 0

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()
        for path in GATES + [STORE_PATH, PER_SOURCE_FILE.name]:
            if os.path.exists(path):
                os.unlink(path)

    def _report(self, code, source):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'freeze frame records restart test'
        request.source_id = source
        self.assertTrue(call_until_answered(self.node, self.report, request, TIMEOUT_SEC).accepted)

    def _clear(self, code, source):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        response = call_until_answered(self.node, self.clear, request, TIMEOUT_SEC)
        self.assertTrue(response.success, response.message)

    def _sentinel_framed(self, life):
        """
        Report fresh sentinel codes until one is framed on ``life``.

        A frame of a code confirmed after the start shows that the gateway receives
        confirm events and that its startup catch-up has returned.
        """
        deadline = time.monotonic() + TIMEOUT_SEC
        while time.monotonic() < deadline:
            type(self).sentinels += 1
            code = f'FF_REC_SENTINEL_{life}_{self.sentinels}'
            self._report(code, APP_A)
            wait_until = time.monotonic() + 5.0
            while time.monotonic() < wait_until:
                found = _frames(life, f'apps/{APP_A}', code)
                if found and APP_A in found['frames']:
                    return
                time.sleep(0.2)
        self.fail(f'no sentinel was framed on gateway life {life}')

    def _restart(self, life, proc_info, gateway):
        """Stop gateway life ``life - 1`` and start life ``life``."""
        os.kill(gateway.process_details['pid'], signal.SIGINT)
        proc_info.assertWaitForShutdown(process=gateway, timeout=TIMEOUT_SEC)
        with open(GATES[life], 'w', encoding='utf-8') as gate:
            gate.write('open')

    def test_1_life_one_frames_every_record(self):
        """Each record gets its own frames in life 1."""
        self._sentinel_framed(0)
        self._report(PER_SOURCE_CODE, APP_A)
        self._report(PER_SOURCE_CODE, APP_B)
        self._report(COMPONENT_CODE, COMPONENT)
        self._report(APP_OWN_CODE, APP_A)

        a = _wait_frames(0, f'apps/{APP_A}', PER_SOURCE_CODE, [APP_A])
        b = _wait_frames(0, f'apps/{APP_B}', PER_SOURCE_CODE, [APP_B])
        component = _wait_frames(0, f'components/{COMPONENT}', COMPONENT_CODE, [APP_A, APP_B])
        own = _wait_frames(0, f'apps/{APP_A}', APP_OWN_CODE, [APP_A])
        self.assertEqual(set(a['frames']), {APP_A})
        self.assertEqual(set(b['frames']), {APP_B})
        self.assertEqual(set(own['frames']), {APP_A})
        type(self).life1 = {
            'a': _captured_at(a['frames'][APP_A]),
            'b': _captured_at(b['frames'][APP_B]),
            'component': {e: _captured_at(component['frames'][e]) for e in (APP_A, APP_B)},
            'own': _captured_at(own['frames'][APP_A]),
            'occurrence_a': a['occurrence'],
            'occurrence_b': b['occurrence'],
        }

    def test_2_every_record_comes_back_after_a_restart(self, proc_info, gateway_0):
        """P1 and P2: after a restart each record serves its own stored frame."""
        self.assertTrue(self.life1, 'life 1 did not run')
        self._restart(1, proc_info, gateway_0)
        self._sentinel_framed(1)

        # P1: the two records of one per_source code keep their own frame and occurrence.
        a = _wait_frames(1, f'apps/{APP_A}', PER_SOURCE_CODE, [APP_A])
        b = _wait_frames(1, f'apps/{APP_B}', PER_SOURCE_CODE, [APP_B])
        self.assertEqual(set(a['frames']), {APP_A})
        self.assertEqual(set(b['frames']), {APP_B})
        self.assertEqual(_captured_at(a['frames'][APP_A]), self.life1['a'],
                         'A was not served from the store')
        self.assertEqual(_captured_at(b['frames'][APP_B]), self.life1['b'],
                         'B was not served from the store')
        self.assertIsNone(_origin(a['frames'][APP_A]))
        self.assertIsNone(_origin(b['frames'][APP_B]))
        self.assertEqual(a['occurrence'], self.life1['occurrence_a'])
        self.assertEqual(b['occurrence'], self.life1['occurrence_b'])

        # P2: the Component's record and the App's own record of another code both come back.
        component = _wait_frames(1, f'components/{COMPONENT}', COMPONENT_CODE, [APP_A, APP_B])
        own = _wait_frames(1, f'apps/{APP_A}', APP_OWN_CODE, [APP_A])
        for entity in (APP_A, APP_B):
            self.assertEqual(_captured_at(component['frames'][entity]),
                             self.life1['component'][entity],
                             f'the Component record lost its stored frame of {entity}')
            self.assertIsNone(_origin(component['frames'][entity]))
        self.assertEqual(_captured_at(own['frames'][APP_A]), self.life1['own'],
                         'the App record lost its stored frame')
        self.assertIsNone(_origin(own['frames'][APP_A]))

    def test_3_a_record_raised_again_while_down_drops_only_its_frame(
            self, proc_info, proc_output, gateway_1, gateway_2):
        """P3: per-record judging drops only the frame of the record that occurred again."""
        self.assertTrue(self.life1, 'life 1 did not run')
        os.kill(gateway_1.process_details['pid'], signal.SIGINT)
        proc_info.assertWaitForShutdown(process=gateway_1, timeout=TIMEOUT_SEC)
        self._clear(PER_SOURCE_CODE, APP_A)
        self._report(PER_SOURCE_CODE, APP_A)
        with open(GATES[2], 'w', encoding='utf-8') as gate:
            gate.write('open')

        proc_output.assertWaitFor(
            f"Entity freeze-frame for fault '{PER_SOURCE_CODE}': the stored frame of entity "
            f"'{APP_A}' is from an earlier occurrence and was discarded (stored occurrence 1, "
            'the fault manager lists occurrence 2, CONFIRMED)',
            process=gateway_2, timeout=TIMEOUT_SEC)
        self._sentinel_framed(2)

        a = _wait_frames(2, f'apps/{APP_A}', PER_SOURCE_CODE, [APP_A],
                         ready=lambda frame: _origin(frame) == 'startup')
        self.assertGreater(_captured_at(a['frames'][APP_A]), self.life1['a'])
        self.assertEqual(a['occurrence'], 2)
        b = _wait_frames(2, f'apps/{APP_B}', PER_SOURCE_CODE, [APP_B])
        self.assertEqual(_captured_at(b['frames'][APP_B]), self.life1['b'],
                         "B's frame was dropped with A's")
        self.assertIsNone(_origin(b['frames'][APP_B]))
        self.assertEqual(b['occurrence'], 1)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            if info.process_name.startswith('gate_'):
                continue
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited {info.returncode}')
