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

"""The graph readers outside discovery apply the same leftover rule.

Covers the startup summary's peer count (discovery's reader) and parameter_beacon (a reader
of its own). Before the gateway starts, the graph lists a ``--ghost`` node, a ``--backed``
node, a running node, and a running twin whose name is also listed without an enclave. The
``--backed`` node in GET /apps shows the gateway got the injector's transient-local history.
"""

import os
import re
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DEFAULT_BASE_URL,
    DEFAULT_DOMAIN_ID,
    get_time_scale,
)
from ros2_medkit_test_utils.graph_fixtures import (
    fixture_path,
    LeftoverNode,
    observed_enclaves,
    split_fqn,
    wait_observed,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node, get_coverage_env

TIME_SCALE = get_time_scale()
PACKAGE = 'ros2_medkit_integration_tests'

GHOST = '/ghost_boot_ns/ghost_boot'
BACKED_CONTROL = '/ghost_boot_control/ghost_boot_control'
LIVE = '/ghost_boot_live/ghost_boot_live'
TWIN = '/ghost_boot_twin/ghost_boot_twin'
LEFTOVER = '/leftover_beacon_ns/leftover_beacon'
# Ghost, backed, live and twin (once): the injector's node is hidden, the gateway's are not peers.
EXPECTED_PEERS = 4

# The gateway starts after the injected entries are in every graph.
GATEWAY_START_DELAY_SEC = 3.0
STARTUP_TIMEOUT_SEC = 60.0 * TIME_SCALE
APPEAR_TIMEOUT_SEC = 30.0 * TIME_SCALE
INJECTION_TIMEOUT_SEC = 45.0 * TIME_SCALE
PROCESS_EXIT_TIMEOUT_SEC = 30.0 * TIME_SCALE
LEFTOVER_DELAY_SEC = 1.0
POLL_INTERVAL_SEC = 1.0
# Three 1 s poll cycles of watching parameter_beacon's clients. A window, so not scaled.
POLL_HOLD_SEC = 3.0


def _node_action(fqn):
    namespace, _, name = fqn.rpartition('/')
    return launch_ros.actions.Node(
        package=PACKAGE,
        executable='demo_rpm_sensor',
        name=name,
        namespace=namespace,
        output='screen',
        additional_env=get_coverage_env(PACKAGE),
        sigterm_timeout='30',
        sigkill_timeout='15',
    )


def generate_test_description():
    injector = ExecuteProcess(
        cmd=[fixture_path('ghost_node_injector'), '--ghost', GHOST, '--backed', BACKED_CONTROL,
             '--ghost', TWIN],
        output='screen',
        sigterm_timeout='30',
        sigkill_timeout='15',
    )
    plugin_path = os.path.join(
        get_package_prefix('ros2_medkit_param_beacon'), 'lib', 'ros2_medkit_param_beacon',
        'libparam_beacon_plugin.so')
    gateway_node = create_gateway_node(
        extra_params={
            'plugins': ['parameter_beacon'],
            'plugins.parameter_beacon.path': plugin_path,
            'plugins.parameter_beacon.poll_interval_sec': POLL_INTERVAL_SEC,
            'plugins.parameter_beacon.param_timeout_sec': 1.0,
        },
    )
    return (
        LaunchDescription([
            injector,
            _node_action(LIVE),
            _node_action(TWIN),
            TimerAction(period=GATEWAY_START_DELAY_SEC, actions=[gateway_node]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node, 'injector': injector},
    )


def _output(proc_output, process):
    return ''.join(output.text.decode(errors='replace') for output in proc_output[process])


def _fixture_env():
    env = os.environ.copy()
    env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
    env.update(get_coverage_env())
    return env


class TestGraphLeftoverNodesAtStartup(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
        while True:
            try:
                if requests.get(f'{DEFAULT_BASE_URL}/health', timeout=2).status_code == 200:
                    break
            except requests.exceptions.RequestException:
                pass
            if time.monotonic() > deadline:
                raise AssertionError('the gateway never answered GET /health')
            time.sleep(0.2)
        rclpy.init()
        cls.observer = rclpy.create_node('_graph_leftover_startup_observer')

    @classmethod
    def tearDownClass(cls):
        cls.observer.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _app_ids():
        return [item.get('id') for item in
                requests.get(f'{DEFAULT_BASE_URL}/apps', timeout=10).json().get('items', [])]

    def _assert_injected(self, proc_output, injector):
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        match = None
        while match is None and time.monotonic() < deadline:
            match = re.search(r'ghost_node_injector: matched_subscriptions=(\d+)',
                              _output(proc_output, injector))
            time.sleep(0.2)
        self.assertIsNotNone(match, 'ghost_node_injector printed no status line')
        self.assertGreater(
            int(match.group(1)), 0,
            'ghost_node_injector timed out waiting for a matched subscription on '
            'ros_discovery_info and never published')
        for fqn, enclaves in ((GHOST, ['']), (TWIN, ['', '/'])):
            self.assertTrue(
                wait_observed(self.observer, fqn,
                              lambda found, want=enclaves: sorted(found) == want,
                              APPEAR_TIMEOUT_SEC),
                f'the test process graph lists {fqn} with '
                f'{observed_enclaves(self.observer, fqn)}, not {enclaves}')
        control_id = split_fqn(BACKED_CONTROL)[0]
        while control_id not in self._app_ids() and time.monotonic() < deadline:
            time.sleep(0.2)
        self.assertIn(control_id, self._app_ids(),
                      f'{BACKED_CONTROL} never appeared in GET /apps, so the gateway never '
                      'received the injection')

    def _beacon_clients(self):
        try:
            return [name for name, _ in self.observer.get_client_names_and_types_by_node(
                '_param_beacon_node', '/')]
        except Exception:
            # rclpy raises for a node this graph does not list yet.
            return []

    def _wait_polled(self, fqn, timeout, polled=True):
        prefix = f'{fqn}/'
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if any(name.startswith(prefix) for name in self._beacon_clients()) == polled:
                return True
            time.sleep(0.2)
        return False

    def test_01_startup_peer_count_counts_what_discovery_exposes(
            self, proc_output, gateway_node, injector):
        self._assert_injected(proc_output, injector)
        deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
        summary = None
        while summary is None and time.monotonic() < deadline:
            summary = re.search(r'Discovery summary: (\d+) peer node\(s\)',
                                _output(proc_output, gateway_node))
            time.sleep(0.2)
        self.assertIsNotNone(summary, 'the gateway never logged its startup discovery summary')
        self.assertEqual(
            int(summary.group(1)), EXPECTED_PEERS,
            f'the startup summary must count {LIVE}, {BACKED_CONTROL}, {GHOST} and {TWIN} once')

    def test_02_parameter_beacon_polls_a_node_it_never_saw_running(self, proc_output, injector):
        self._assert_injected(proc_output, injector)
        self.assertTrue(
            self._wait_polled(GHOST, APPEAR_TIMEOUT_SEC),
            f'parameter_beacon never created a parameter client for {GHOST}: it has no enclave '
            'and no endpoints, but the plugin never saw it running, so it is not a leftover')

    def _participant_prefix(self, node_name):
        """GUID prefix of the participant `node_name` publishes /rosout from, or None."""
        for info in self.observer.get_publishers_info_by_topic('/rosout'):
            if info.node_name == node_name and info.node_namespace == '/':
                return bytes(info.endpoint_gid[:12])
        return None

    def test_03_parameter_beacon_does_not_poll_the_leftover_of_a_node_it_polled(
            self, proc_output, gateway_node):
        leftover = LeftoverNode(LEFTOVER, LEFTOVER_DELAY_SEC, env=_fixture_env())
        self.addCleanup(leftover.stop, PROCESS_EXIT_TIMEOUT_SEC)
        self.assertTrue(leftover.wait_ready(INJECTION_TIMEOUT_SEC), leftover.output())
        self.assertTrue(
            self._wait_polled(LEFTOVER, APPEAR_TIMEOUT_SEC),
            f'parameter_beacon never polled {LEFTOVER} while it ran, so it never saw it running')

        leftover.leave()
        self.assertTrue(
            wait_observed(self.observer, LEFTOVER, lambda enclaves: enclaves == [''],
                          LEFTOVER_DELAY_SEC + INJECTION_TIMEOUT_SEC),
            f'the test process graph lists {LEFTOVER} with '
            f'{observed_enclaves(self.observer, LEFTOVER)} rather than as a leftover')
        # The plugin's node shares the gateway's participant, so discovery's warning shows the
        # plugin's graph lists the leftover too.
        gateway_prefix = self._participant_prefix('ros2_medkit_gateway')
        self.assertIsNotNone(gateway_prefix, 'the gateway node publishes no /rosout')
        self.assertEqual(
            self._participant_prefix('_param_beacon_node'), gateway_prefix,
            "parameter_beacon's node does not share the gateway node's participant, so the "
            'discovery warning would say nothing about the graph the plugin reads')
        deadline = time.monotonic() + APPEAR_TIMEOUT_SEC
        warning = f"Node '{LEFTOVER}' is not exposed"
        while warning not in _output(proc_output, gateway_node) and time.monotonic() < deadline:
            time.sleep(0.2)
        self.assertTrue(
            warning in _output(proc_output, gateway_node),
            f'discovery never warned about the leftover of {LEFTOVER}, so the graph the plugin '
            'reads may never have listed it')

        self.assertTrue(
            self._wait_polled(LEFTOVER, APPEAR_TIMEOUT_SEC, polled=False),
            f'parameter_beacon still has a parameter client for the leftover of {LEFTOVER}')
        deadline = time.monotonic() + POLL_HOLD_SEC
        while time.monotonic() < deadline:
            clients = [name for name in self._beacon_clients()
                       if name.startswith(f'{LEFTOVER}/')]
            self.assertFalse(
                clients,
                f'parameter_beacon polls the leftover of {LEFTOVER}, a node it polled while it '
                f'ran: {clients}')
            time.sleep(0.2)
        self.assertEqual(observed_enclaves(self.observer, LEFTOVER), [''])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
