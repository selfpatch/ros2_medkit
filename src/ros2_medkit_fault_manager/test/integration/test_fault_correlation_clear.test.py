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

"""
The relations of a root cause across its re-reports and its clear, through a real fault manager.

  * a root cleared with skip_correlation_auto_clear leaves its symptoms standing and unmuted,
    and stops muting later reports;
  * a root reported again while active keeps the symptoms it already muted, so its clear
    cascades only to the ones it still mutes.

Each scenario runs on a default (shared) code and on a per_source code whose root and symptom
come from different nodes.
"""

import os
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetFault, ListFaults, ReportFault

TIMEOUT_SEC = 30.0
ALL_STATUSES = [
    Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED, Fault.STATUS_CONFIRMED,
    Fault.STATUS_HEALED, Fault.STATUS_CLEARED,
]
KINDS = ('P7', 'P8', 'P11')
MODES = ('SH', 'PS')
# A short window lets a test move past it by reporting until the symptom is no longer muted.
WINDOW_MS = {'P11': 200}

THRESHOLDS = ''.join(
    f'CL_{kind}_PS_{role}:\n  per_source: true\n' for kind in KINDS for role in ('ROOT', 'SYM'))
CORRELATION = 'correlation:\n  enabled: true\n  rules:\n' + ''.join(
    f"""    - id: {kind.lower()}_{mode.lower()}
      name: "{kind} {mode}"
      mode: hierarchical
      root_cause:
        codes: ["CL_{kind}_{mode}_ROOT"]
      symptoms:
        - codes: ["CL_{kind}_{mode}_SYM"]
      window_ms: {WINDOW_MS.get(kind, 30000)}
      mute_symptoms: true
      auto_clear_with_root: true
""" for kind in KINDS for mode in MODES)


def _temp_file(prefix, content):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(content)
    handle.close()
    return handle.name


THRESHOLDS_FILE = _temp_file('correlation_clear_thresholds_', THRESHOLDS)
CORRELATION_FILE = _temp_file('correlation_clear_rules_', CORRELATION)


def generate_test_description():
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'healing_enabled': False,
            'fault_thresholds.config_file': THRESHOLDS_FILE,
            'correlation.config_file': CORRELATION_FILE,
            'snapshots.enabled': False,
            'snapshots.rosbag.enabled': False,
        }],
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    return (
        LaunchDescription([fault_manager_node, launch_testing.actions.ReadyToTest()]),
        {'fault_manager_node': fault_manager_node},
    )


class TestCorrelationClear(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_correlation_clear_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.get_client = cls.node.create_client(GetFault, '/fault_manager/get_fault')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        for client in (cls.report_client, cls.list_client, cls.get_client, cls.clear_client):
            assert client.wait_for_service(timeout_sec=TIMEOUT_SEC)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.done(), 'service call timed out')
        return future.result()

    def _report(self, code, source, event=ReportFault.Request.EVENT_FAILED):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = event
        request.severity = Fault.SEVERITY_ERROR
        request.description = f'{code} from {source}'
        request.source_id = source
        self.assertTrue(self._call(self.report_client, request).accepted)

    def _clear(self, code, source, skip):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        request.skip_correlation_auto_clear = skip
        response = self._call(self.clear_client, request)
        self.assertTrue(response.success, response.message)
        return response

    def _status(self, code, source):
        request = GetFault.Request()
        request.fault_code = code
        request.source_id = source
        response = self._call(self.get_client, request)
        self.assertTrue(response.success, response.error_message)
        return response.fault.status

    def _visible(self):
        """Return the codes the default listing shows."""
        return {f.fault_code for f in self._call(self.list_client, ListFaults.Request()).faults}

    def _muted(self):
        request = ListFaults.Request()
        request.include_muted = True
        return {m.fault_code for m in self._call(self.list_client, request).muted_faults}

    def _scenario_sources(self, mode):
        """Return (root owner, symptom owner, root source, symptom source)."""
        # A per-source record is addressed by its owner, a shared one by '*'.
        if mode == 'PS':
            return '/root_node', '/sym_node', '/root_node', '/sym_node'
        return '*', '*', '/root_node', '/sym_node'

    def _skip_clear_ends_the_relations(self, mode):
        root, symptom = f'CL_P7_{mode}_ROOT', f'CL_P7_{mode}_SYM'
        root_owner, sym_owner, root_source, sym_source = self._scenario_sources(mode)
        self._report(root, root_source)
        self._report(symptom, sym_source)
        self.assertIn(symptom, self._muted())
        self.assertNotIn(symptom, self._visible())

        response = self._clear(root, root_owner, skip=True)
        self.assertEqual(list(response.auto_cleared_codes), [], 'skip must not cascade')
        self.assertEqual(self._status(root, root_owner), Fault.STATUS_CLEARED)
        self.assertEqual(self._status(symptom, sym_owner), Fault.STATUS_CONFIRMED)
        self.assertNotIn(symptom, self._muted())
        self.assertIn(symptom, self._visible(), 'the symptom stayed muted after its root cleared')

        # The cleared root no longer mutes what is reported after it.
        self._report(symptom, sym_source)
        self.assertNotIn(symptom, self._muted())
        self.assertIn(symptom, self._visible())

    def _re_reported_root_keeps_its_symptoms(self, mode):
        root, symptom = f'CL_P8_{mode}_ROOT', f'CL_P8_{mode}_SYM'
        root_owner, sym_owner, root_source, sym_source = self._scenario_sources(mode)
        self._report(root, root_source)
        self._report(symptom, sym_source)
        self.assertIn(symptom, self._muted())
        self._report(root, root_source)

        response = self._clear(root, root_owner, skip=False)
        self.assertIn(symptom, list(response.auto_cleared_codes))
        self.assertEqual(self._status(symptom, sym_owner), Fault.STATUS_CLEARED)

    def _list_exact_symptoms(self, mode):
        root, symptom = f'CL_P11_{mode}_ROOT', f'CL_P11_{mode}_SYM'
        root_owner, sym_owner, root_source, sym_source = self._scenario_sources(mode)
        self._report(root, root_source)
        self._report(symptom, sym_source)
        self.assertIn(symptom, self._muted())
        self._clear(symptom, sym_owner, skip=False)
        self._report(root, root_source)
        # Report the symptom until it falls outside the root's window and is no longer muted.
        deadline = time.monotonic() + TIMEOUT_SEC
        while True:
            self._report(symptom, sym_source)
            if symptom not in self._muted():
                break
            self.assertLess(time.monotonic(), deadline, 'the symptom stayed muted past its window')
            self._clear(symptom, sym_owner, skip=False)

        response = self._clear(root, root_owner, skip=False)
        self.assertEqual(list(response.auto_cleared_codes), [],
                         'the root cleared a symptom it did not mute')
        self.assertEqual(self._status(symptom, sym_owner), Fault.STATUS_CONFIRMED)

    def test_p11_default_code_cascade_clears_only_the_muted_symptoms(self):
        self._list_exact_symptoms('SH')

    def test_p11_per_source_code_cascade_clears_only_the_muted_symptoms(self):
        self._list_exact_symptoms('PS')

    def test_p7_default_code_skip_clear_ends_the_relations(self):
        self._skip_clear_ends_the_relations('SH')

    def test_p7_per_source_code_skip_clear_ends_the_relations(self):
        self._skip_clear_ends_the_relations('PS')

    def test_p8_default_code_re_reported_root_keeps_its_symptoms(self):
        self._re_reported_root_keeps_its_symptoms('SH')

    def test_p8_per_source_code_re_reported_root_keeps_its_symptoms(self):
        self._re_reported_root_keeps_its_symptoms('PS')


@launch_testing.post_shutdown_test()
class TestCorrelationClearShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE)
        os.unlink(CORRELATION_FILE)
