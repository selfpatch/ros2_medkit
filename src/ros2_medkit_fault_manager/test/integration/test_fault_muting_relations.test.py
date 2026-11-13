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
A record stays muted while any live relation still mutes it.

Two relations can mute one record: a hierarchical root cause that lists it, and an active
show_as_single cluster. Ending one relation, by a clear with or without the skip flag or by a
heal, must not unmute a record the other relation still mutes. The same holds for two root
causes that list one symptom. Each scenario runs on shared codes (one code per record) and on
per_source codes (one record per reporting node).
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
from ros2_medkit_msgs.srv import ClearFault, ListFaults, ReportFault

TIMEOUT_SEC = 30.0
MODES = ('SH', 'PS')
# variant -> mute_symptoms of the hierarchical rule
VARIANTS = {'CLR': 'true', 'HEAL': 'true', 'NOMUTE': 'false'}


def _code(variant, mode, role):
    return f'CL_MX_{variant}_{mode}_{role}'


THRESHOLDS = ''.join(
    f'{_code(v, "PS", role)}:\n  per_source: true\n'
    for v in VARIANTS for role in ('ROOT', 'T_X')) + ''.join(
    f'CL_TR_PS_{role}:\n  per_source: true\n' for role in ('R1', 'R2', 'S')) + ''.join(
    f'CL_F5_PS_{role}:\n  per_source: true\n' for role in ('X', 'Y', 'S_1'))
CORRELATION = 'correlation:\n  enabled: true\n  patterns:\n' + ''.join(
    f'    mx_{v.lower()}_{m.lower()}:\n      codes: ["{_code(v, m, "T")}_*"]\n'
    for v in VARIANTS for m in MODES) + '  rules:\n' + ''.join(
    f"""    - id: mx_h_{v.lower()}_{m.lower()}
      name: "MX hierarchical {v} {m}"
      mode: hierarchical
      root_cause:
        codes: ["{_code(v, m, 'ROOT')}"]
      symptoms:
        - codes: ["{_code(v, m, 'T')}_*"]
      window_ms: 30000
      mute_symptoms: {mute}
      auto_clear_with_root: true
    - id: mx_c_{v.lower()}_{m.lower()}
      name: "MX cluster {v} {m}"
      mode: auto_cluster
      match:
        - pattern: mx_{v.lower()}_{m.lower()}
      min_count: 3
      window_ms: 30000
      show_as_single: true
      representative: first
""" for v, mute in VARIANTS.items() for m in MODES) + ''.join(
    f"""    - id: tr_{m.lower()}
      name: "TR {m}"
      mode: hierarchical
      root_cause:
        codes: ["CL_TR_{m}_R1", "CL_TR_{m}_R2"]
      symptoms:
        - codes: ["CL_TR_{m}_S", "CL_TR_{m}_SENT_*"]
      window_ms: 300
      mute_symptoms: true
      auto_clear_with_root: true
""" for m in MODES) + ''.join(
    f"""    - id: f5_a_{m.lower()}
      name: "F5 A {m}"
      mode: hierarchical
      root_cause:
        codes: ["CL_F5_{m}_X"]
      symptoms:
        - codes: ["CL_F5_{m}_S_*"]
      window_ms: 30000
      mute_symptoms: false
      auto_clear_with_root: true
    - id: f5_b_{m.lower()}
      name: "F5 B {m}"
      mode: hierarchical
      root_cause:
        codes: ["CL_F5_{m}_X"]
      symptoms:
        - codes: ["CL_F5_{m}_S_*"]
      window_ms: 30000
      mute_symptoms: true
      auto_clear_with_root: true
    - id: f5_c_{m.lower()}
      name: "F5 C {m}"
      mode: hierarchical
      root_cause:
        codes: ["CL_F5_{m}_Y"]
      symptoms:
        - codes: ["CL_F5_{m}_S_*", "CL_F5_{m}_Z_*"]
      window_ms: 300
      mute_symptoms: true
      auto_clear_with_root: true
""" for m in MODES)


def _temp_file(prefix, content):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(content)
    handle.close()
    return handle.name


THRESHOLDS_FILE = _temp_file('muting_relations_thresholds_', THRESHOLDS)
CORRELATION_FILE = _temp_file('muting_relations_rules_', CORRELATION)


def generate_test_description():
    fault_manager_node = launch_ros.actions.Node(
        package='ros2_medkit_fault_manager',
        executable='fault_manager_node',
        name='fault_manager',
        output='screen',
        parameters=[{
            'storage_type': 'memory',
            'confirmation_threshold': -1,
            'healing_enabled': True,
            'healing_threshold': 0,
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


class TestMutingRelations(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_muting_relations_client')
        cls.report_client = cls.node.create_client(ReportFault, '/fault_manager/report_fault')
        cls.list_client = cls.node.create_client(ListFaults, '/fault_manager/list_faults')
        cls.clear_client = cls.node.create_client(ClearFault, '/fault_manager/clear_fault')
        for client in (cls.report_client, cls.list_client, cls.clear_client):
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

    def _clear(self, code, owner, skip):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = owner
        request.skip_correlation_auto_clear = skip
        response = self._call(self.clear_client, request)
        self.assertTrue(response.success, response.message)

    def _shown(self, codes):
        """Return the (code, first source) pairs of the default listing among codes."""
        faults = self._call(self.list_client, ListFaults.Request()).faults
        return {(f.fault_code, f.reporting_sources[0]) for f in faults if f.fault_code in codes}

    def _muted(self, codes):
        """Return {(code, owner): root_cause_code} of include_muted among codes."""
        request = ListFaults.Request()
        request.include_muted = True
        muted = self._call(self.list_client, request).muted_faults
        return {(m.fault_code, m.source_id): m.root_cause_code for m in muted
                if m.fault_code in codes}

    @staticmethod
    def _records(variant, mode):
        """Return (root, root source, root owner, [(code, source, owner)] of three records)."""
        root = _code(variant, mode, 'ROOT')
        if mode == 'PS':
            return root, '/r', '/r', [(_code(variant, mode, 'T_X'), f'/n{i}', f'/n{i}')
                                      for i in (1, 2, 3)]
        return root, '/r', '*', [(f'{_code(variant, mode, "T")}_{i}', '/n', '*')
                                 for i in (1, 2, 3)]

    def _cluster_and_root(self, variant, mode, end_root):
        root, root_source, root_owner, records = self._records(variant, mode)
        codes = {code for code, _, _ in records}
        for code, source, _ in records:
            self._report(code, source)
        first, second, third = records
        self.assertEqual(self._shown(codes), {(first[0], first[1])})
        self._report(root, root_source)
        self._report(second[0], second[1])
        end_root(root, root_source, root_owner)
        self.assertEqual(
            self._shown(codes), {(first[0], first[1])},
            'a record its active cluster mutes was shown when the root cause ended')
        self.assertEqual(set(self._muted(codes)), {(second[0], second[2]), (third[0], third[2])})

    def _each_mode(self, check):
        for mode in MODES:
            with self.subTest(mode=mode):
                check(mode)

    def test_a_cleared_root_leaves_the_cluster_mute(self):
        def end_root(root, source, owner):
            self._clear(root, owner, skip=True)
        self._each_mode(lambda mode: self._cluster_and_root('CLR', mode, end_root))

    def test_a_healed_root_leaves_the_cluster_mute(self):
        def end_root(root, source, owner):
            self._report(root, source, event=ReportFault.Request.EVENT_PASSED)
        self._each_mode(lambda mode: self._cluster_and_root('HEAL', mode, end_root))

    def test_a_root_that_does_not_mute_leaves_the_cluster_mute(self):
        def end_root(root, source, owner):
            self._clear(root, owner, skip=True)
        self._each_mode(lambda mode: self._cluster_and_root('NOMUTE', mode, end_root))

    def _two_roots(self, mode):
        r1, r2 = f'CL_TR_{mode}_R1', f'CL_TR_{mode}_R2'
        symptom = f'CL_TR_{mode}_S'
        owner = lambda source: source if mode == 'PS' else '*'  # noqa: E731
        self._report(r1, '/r1')
        self._report(symptom, '/s')
        self.assertIn((symptom, owner('/s')), self._muted({symptom}))
        # Report fresh symptoms until one comes back unmuted: R1's window has passed.
        deadline = time.monotonic() + TIMEOUT_SEC
        index = 0
        while True:
            index += 1
            sentinel = f'CL_TR_{mode}_SENT_{index}'
            self._report(sentinel, '/s')
            if not self._muted({sentinel}):
                break
            self.assertLess(time.monotonic(), deadline, 'the window of the first root never ended')
        self._report(r2, '/r2')
        self._report(symptom, '/s')
        self.assertEqual(self._muted({symptom}), {(symptom, owner('/s')): r2})

        self._clear(r1, owner('/r1'), skip=True)
        self.assertEqual(self._shown({symptom}), set(), 'the second root still mutes the symptom')
        self.assertEqual(self._muted({symptom}), {(symptom, owner('/s')): r2})

    def test_a_cleared_root_leaves_the_mute_of_another_root(self):
        self._each_mode(self._two_roots)

    def _muted_info(self, codes):
        """Return {(code, owner): (root_cause_code, delay_ms)} of include_muted among codes."""
        request = ListFaults.Request()
        request.include_muted = True
        muted = self._call(self.list_client, request).muted_faults
        return {(m.fault_code, m.source_id): (m.root_cause_code, m.delay_ms) for m in muted
                if m.fault_code in codes}

    def _cross_window(self, prefix, source):
        """Report fresh symptoms until one is no longer muted: the window of the root passed."""
        deadline = time.monotonic() + TIMEOUT_SEC
        index = 0
        while True:
            index += 1
            sentinel = f'{prefix}_{index}'
            self._report(sentinel, source)
            if not self._muted({sentinel}):
                return
            self.assertLess(time.monotonic(), deadline, 'the window of the root never ended')

    def _root_bound_to_its_own_rule(self, mode):
        x, y = f'CL_F5_{mode}_X', f'CL_F5_{mode}_Y'
        symptom = f'CL_F5_{mode}_S_1'
        owner = '/s' if mode == 'PS' else '*'
        self._report(y, '/y')
        self._report(symptom, '/s')
        self.assertEqual(self._muted({symptom}), {(symptom, owner): y})
        self._cross_window(f'CL_F5_{mode}_Z', '/s')
        # X is bound to its first rule, which does not mute. The later rule for X never applies.
        self._report(x, '/x')
        self._report(symptom, '/s')

        self._clear(y, '/y' if mode == 'PS' else '*', skip=True)
        self.assertEqual(self._muted({symptom}), {}, 'a rule the root is not bound to muted it')
        self.assertEqual(self._shown({symptom}), {(symptom, '/s')})

    def test_a_root_mutes_only_through_its_own_rule(self):
        self._each_mode(self._root_bound_to_its_own_rule)

    def _delay_follows_the_relation(self, mode):
        r1, r2 = f'CL_TR_{mode}_R1', f'CL_TR_{mode}_R2'
        symptom = f'CL_TR_{mode}_S'
        owner = '/s' if mode == 'PS' else '*'
        self._report(r1, '/r1')
        # Report again until the delay behind the first root is more than a millisecond.
        deadline = time.monotonic() + TIMEOUT_SEC
        while True:
            self._report(symptom, '/s')
            root, delay = self._muted_info({symptom})[(symptom, owner)]
            if delay > 0:
                break
            self.assertLess(time.monotonic(), deadline, 'the delay behind the first root stayed 0')
        self.assertEqual(root, r1)
        self._cross_window(f'CL_TR_{mode}_SENT', '/s')
        self._report(r2, '/r2')
        self._report(symptom, '/s')
        self.assertEqual(self._muted({symptom}), {(symptom, owner): r2})

        self._clear(r2, '/r2' if mode == 'PS' else '*', skip=True)
        self.assertEqual(self._muted_info({symptom}), {(symptom, owner): (r1, delay)})

    def test_the_delay_moves_with_the_mute(self):
        self._each_mode(self._delay_follows_the_relation)


@launch_testing.post_shutdown_test()
class TestMutingRelationsShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE)
        os.unlink(CORRELATION_FILE)
