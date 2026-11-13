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
A show_as_single cluster shows its current representative and mutes the other members.

Muting is display only: the default listing hides a muted member, include_muted lists it, and
the record GetFault returns is the one an unmuted member has. The representative the policy
selects is the one shown. When a member leaves, the representative that follows is shown.

Each scenario runs on shared codes (one code per member) and on a per_source code (one record
per reporting node).
"""

import os
import tempfile
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
MODES = ('SH', 'PS')
# scenario -> representative policy
SCENARIOS = {
    'ONLY': 'first',
    'NEXT': 'first',
    'ESC': 'highest_severity',
    'REC': 'most_recent',
    'GET': 'first',
}

THRESHOLDS = ''.join(
    f'CM_PS_{scenario}_X:\n  per_source: true\n' for scenario in SCENARIOS)
CORRELATION = 'correlation:\n  enabled: true\n  patterns:\n' + ''.join(
    f'    cm_{mode.lower()}_{scenario.lower()}:\n      codes: ["CM_{mode}_{scenario}_*"]\n'
    for mode in MODES for scenario in SCENARIOS) + '  rules:\n' + ''.join(
    f"""    - id: cm_{mode.lower()}_{scenario.lower()}
      name: "CM {mode} {scenario}"
      mode: auto_cluster
      match:
        - pattern: cm_{mode.lower()}_{scenario.lower()}
      min_count: 3
      window_ms: 30000
      show_as_single: true
      representative: {policy}
""" for mode in MODES for scenario, policy in SCENARIOS.items())


def _temp_file(prefix, content):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(content)
    handle.close()
    return handle.name


THRESHOLDS_FILE = _temp_file('cluster_muting_thresholds_', THRESHOLDS)
CORRELATION_FILE = _temp_file('cluster_muting_rules_', CORRELATION)


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


class TestClusterMuting(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_cluster_muting_client')
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

    @staticmethod
    def _members(mode, scenario):
        """Return [(code, source, owner)] of the three members, in report order."""
        if mode == 'PS':
            return [(f'CM_PS_{scenario}_X', f'/n{i}', f'/n{i}') for i in (1, 2, 3)]
        return [(f'CM_SH_{scenario}_{i}', '/n', '*') for i in (1, 2, 3)]

    def _report(self, member, severity=Fault.SEVERITY_WARN):
        code, source, _ = member
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = severity
        request.description = f'{code} from {source}'
        request.source_id = source
        self.assertTrue(self._call(self.report_client, request).accepted)

    def _clear(self, member):
        code, _, owner = member
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = owner
        response = self._call(self.clear_client, request)
        self.assertTrue(response.success, response.message)

    def _get(self, member):
        code, _, owner = member
        request = GetFault.Request()
        request.fault_code = code
        request.source_id = owner
        response = self._call(self.get_client, request)
        self.assertTrue(response.success, response.error_message)
        return response.fault

    def _shown(self, members):
        """Return the (code, owner) pairs of the default listing that belong to members."""
        owner_of = {(code, source): owner for code, source, owner in members}
        faults = self._call(self.list_client, ListFaults.Request()).faults
        return {(f.fault_code, owner_of[(f.fault_code, f.reporting_sources[0])])
                for f in faults if (f.fault_code, f.reporting_sources[0]) in owner_of}

    def _muted(self, members):
        """Return the (code, owner) pairs of include_muted that belong to members."""
        request = ListFaults.Request()
        request.include_muted = True
        response = self._call(self.list_client, request)
        ours = {self._key(member) for member in members}
        return {(m.fault_code, m.source_id) for m in response.muted_faults} & ours

    def _key(self, member):
        return (member[0], member[2])

    def _each_mode(self, scenario, check):
        for mode in MODES:
            with self.subTest(mode=mode):
                check(self._members(mode, scenario))

    def test_the_representative_is_shown_and_the_others_are_muted(self):
        def check(members):
            for member in members:
                self._report(member)
            first, second, third = members
            self.assertEqual(self._shown(members), {self._key(first)})
            self.assertEqual(self._muted(members), {self._key(second), self._key(third)})
        self._each_mode('ONLY', check)

    def test_the_next_representative_is_shown_when_the_shown_one_is_cleared(self):
        def check(members):
            first, second, third = members
            for member in members:
                self._report(member)
            self._clear(first)
            self.assertEqual(self._shown(members), {self._key(second)})
            self.assertEqual(self._muted(members), {self._key(third)})
            self._clear(second)
            self.assertEqual(self._shown(members), {self._key(third)})
            self.assertEqual(self._muted(members), set())
            self._clear(third)
            self.assertEqual(self._shown(members), set())
            self.assertEqual(self._muted(members), set())
        self._each_mode('NEXT', check)

    def test_a_member_that_escalates_becomes_the_shown_one(self):
        def check(members):
            first, second, third = members
            for member in members:
                self._report(member)
            self.assertEqual(self._shown(members), {self._key(first)})
            self._report(third, Fault.SEVERITY_CRITICAL)
            self.assertEqual(self._shown(members), {self._key(third)})
            self.assertEqual(self._muted(members), {self._key(first), self._key(second)})
        self._each_mode('ESC', check)

    def test_the_most_recent_member_is_shown(self):
        def check(members):
            first, second, third = members
            for member in members:
                self._report(member)
            self.assertEqual(self._shown(members), {self._key(third)})
            self.assertEqual(self._muted(members), {self._key(first), self._key(second)})
        self._each_mode('REC', check)

    def test_muting_does_not_change_the_record(self):
        def check(members):
            for member in members:
                self._report(member, Fault.SEVERITY_ERROR)
            shown, muted = self._get(members[0]), self._get(members[1])
            self.assertIn(self._key(members[1]), self._muted(members))
            self.assertEqual(muted.status, shown.status)
            self.assertEqual(muted.status, Fault.STATUS_CONFIRMED)
            self.assertEqual(muted.severity, Fault.SEVERITY_ERROR)
            self.assertEqual(muted.occurrence_count, shown.occurrence_count)
        self._each_mode('GET', check)


@launch_testing.post_shutdown_test()
class TestClusterMutingShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE)
        os.unlink(CORRELATION_FILE)
