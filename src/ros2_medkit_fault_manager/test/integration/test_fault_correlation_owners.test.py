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
Correlation across owners, through a real fault manager.

Rules match by fault code, whoever reports it. A root cause of one node mutes and
auto-clears the matching symptom records of every node; a cluster holds the matching
records of every node. The relations hold records, so they work the same for per_source
codes (one record per node) and for shared codes (one record, owner '*').
"""

import os
import tempfile
import unittest

from launch import LaunchDescription
import launch_ros.actions
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import rclpy
from rclpy.node import Node
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, GetFault, ListFaults, ReportFault

TIMEOUT_SEC = 30.0
ALL_STATUSES = [
    Fault.STATUS_PREFAILED, Fault.STATUS_PREPASSED, Fault.STATUS_CONFIRMED,
    Fault.STATUS_HEALED, Fault.STATUS_CLEARED,
]

THRESHOLDS = """\
XO_PS_ESTOP:
  per_source: true
XO_PS_MOTOR_A:
  per_source: true
XO_PS_MOTOR_B:
  per_source: true
XO_PS_STORM_1:
  per_source: true
XO_PS_STORM_2:
  per_source: true
XO_PS_STORM_3:
  per_source: true
"""

CORRELATION = """\
correlation:
  enabled: true
  default_window_ms: 5000
  patterns:
    ps_motor:
      codes: ["XO_PS_MOTOR_*"]
    sh_motor:
      codes: ["XO_SH_MOTOR_*"]
    ps_storm:
      codes: ["XO_PS_STORM_*"]
    sh_storm:
      codes: ["XO_SH_STORM_*"]
  rules:
    - id: ps_estop
      name: "Per-source e-stop"
      mode: hierarchical
      root_cause:
        codes: ["XO_PS_ESTOP"]
      symptoms:
        - pattern: ps_motor
      window_ms: 5000
      mute_symptoms: true
      auto_clear_with_root: true
    - id: sh_estop
      name: "Shared e-stop"
      mode: hierarchical
      root_cause:
        codes: ["XO_SH_ESTOP"]
      symptoms:
        - pattern: sh_motor
      window_ms: 5000
      mute_symptoms: true
      auto_clear_with_root: true
    - id: ps_storm
      name: "Per-source storm"
      mode: auto_cluster
      match:
        - pattern: ps_storm
      min_count: 3
      window_ms: 5000
      show_as_single: true
      representative: first
    - id: sh_storm
      name: "Shared storm"
      mode: auto_cluster
      match:
        - pattern: sh_storm
      min_count: 3
      window_ms: 5000
      show_as_single: true
      representative: first
"""


def _temp_file(prefix, content):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', prefix=prefix, delete=False)
    handle.write(content)
    handle.close()
    return handle.name


THRESHOLDS_FILE = _temp_file('correlation_owners_thresholds_', THRESHOLDS)
CORRELATION_FILE = _temp_file('correlation_owners_rules_', CORRELATION)


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


class TestCorrelationAcrossOwners(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_correlation_owners_client')
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

    def _report(self, code, source, severity=Fault.SEVERITY_ERROR):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = severity
        request.description = f'{code} from {source}'
        request.source_id = source
        self.assertTrue(self._call(self.report_client, request).accepted)

    def _list(self, statuses=(), include_muted=False, include_clusters=False):
        request = ListFaults.Request()
        request.statuses = list(statuses)
        request.include_muted = include_muted
        request.include_clusters = include_clusters
        return self._call(self.list_client, request)

    def _clear(self, code, source=''):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        return self._call(self.clear_client, request)

    def _get(self, code, source=''):
        request = GetFault.Request()
        request.fault_code = code
        request.source_id = source
        return self._call(self.get_client, request)

    def _status_of(self, code, owner):
        records = [f for f in self._list(ALL_STATUSES, include_muted=True).faults
                   if f.fault_code == code]
        response = self._get(code, owner)
        self.assertTrue(response.success, f'{code}/{owner}: {response.error_message} {records}')
        return response.fault.status

    def _cascade(self, root, root_node, symptoms, symptom_owner_of_record):
        """Report a root and symptoms from other nodes, expect them muted, then cleared with it."""
        self._report(root, root_node)
        for code, node in symptoms:
            self._report(code, node)

        muted = self._list(include_muted=True)
        muted_records = sorted((m.fault_code, m.source_id) for m in muted.muted_faults)
        expected = sorted((code, symptom_owner_of_record(code, node)) for code, node in symptoms)
        self.assertEqual(muted_records, expected)
        for m in muted.muted_faults:
            self.assertEqual(m.root_cause_code, root)

        visible = {f.fault_code for f in self._list().faults}
        self.assertIn(root, visible)
        for code, _ in symptoms:
            self.assertNotIn(code, visible, 'a muted symptom of another node stays hidden')

        cleared = self._clear(root, root_node)
        self.assertTrue(cleared.success, cleared.message)
        self.assertEqual(sorted(cleared.auto_cleared_codes), sorted(code for code, _ in symptoms))
        for code, node in symptoms:
            self.assertEqual(self._status_of(code, symptom_owner_of_record(code, node)),
                             Fault.STATUS_CLEARED)
        remaining = [m.fault_code for m in self._list(include_muted=True).muted_faults]
        for code, _ in symptoms:
            self.assertNotIn(code, remaining)

    def _storm(self, rule_id, reports, owner_of_record):
        """Report records of different nodes inside one window and expect one cluster."""
        for code, node in reports:
            self._report(code, node)

        clusters = [c for c in self._list(include_clusters=True).clusters if c.rule_id == rule_id]
        self.assertEqual(len(clusters), 1)
        cluster = clusters[0]
        members = sorted(zip(cluster.fault_codes, cluster.fault_owners))
        expected = sorted((code, owner_of_record(node)) for code, node in reports)
        self.assertEqual(members, expected)
        self.assertEqual(len(cluster.fault_owners), len(cluster.fault_codes))
        self.assertEqual(cluster.count, len(reports))

        # The representative resolves to exactly one record through the single-record services.
        response = self._get(cluster.representative_code, cluster.representative_owner)
        self.assertTrue(response.success, response.error_message)
        self.assertEqual(response.fault.fault_code, cluster.representative_code)
        self.assertIn((cluster.representative_code, cluster.representative_owner), members)
        return cluster

    def test_per_source_root_mutes_and_clears_symptoms_of_other_nodes(self):
        self._cascade(
            'XO_PS_ESTOP', '/estop_node',
            [('XO_PS_MOTOR_A', '/motor_node'), ('XO_PS_MOTOR_B', '/drive_node')],
            lambda code, node: node)

    def test_shared_root_mutes_and_clears_symptoms_of_other_nodes(self):
        self._cascade(
            'XO_SH_ESTOP', '/estop_node',
            [('XO_SH_MOTOR_A', '/motor_node'), ('XO_SH_MOTOR_B', '/drive_node')],
            lambda code, node: '*')

    def test_per_source_storm_across_nodes_forms_one_cluster(self):
        # One code from two nodes is two member records of the cluster.
        cluster = self._storm(
            'ps_storm',
            [('XO_PS_STORM_1', '/lidar'), ('XO_PS_STORM_1', '/camera'),
             ('XO_PS_STORM_2', '/motor_node')],
            lambda node: node)
        self.assertEqual(cluster.representative_code, 'XO_PS_STORM_1')
        self.assertEqual(cluster.representative_owner, '/lidar')

    def test_shared_storm_across_nodes_forms_one_cluster(self):
        cluster = self._storm(
            'sh_storm',
            [('XO_SH_STORM_1', '/lidar'), ('XO_SH_STORM_2', '/camera'),
             ('XO_SH_STORM_3', '/motor_node')],
            lambda node: '*')
        self.assertEqual(cluster.representative_owner, '*')


@launch_testing.post_shutdown_test()
class TestCorrelationOwnersShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
        os.unlink(THRESHOLDS_FILE)
        os.unlink(CORRELATION_FILE)
