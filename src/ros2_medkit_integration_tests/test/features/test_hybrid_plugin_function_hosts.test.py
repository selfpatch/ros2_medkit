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

"""A plugin Function that runtime discovery also derives stays without hosts.

One gateway in hybrid mode with ``allow_heuristic_functions`` true, so runtime
discovery derives a Function per namespace. Namespace ``plugin_ns`` holds one
hidden node only. The in-tree test plugin declares a Function ``plugin_ns``
that hosts that node's App. The runtime layer contributes the same id first, so
the merged Function reads source ``runtime``, and the plugin's hosts win the
hierarchy merge. Its only host is hidden. A plugin contributes it, so it stays,
with no hosts.

Namespace ``hidden_only_ns`` holds one hidden node and nothing else declares
it, so it has no Function. Namespace ``visible_ns`` is the positive control.
"""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

PORT = get_test_port(0)
BASE_URL = f'http://127.0.0.1:{PORT}{API_BASE_PATH}'

PLUGIN_FUNCTION_ID = 'plugin_ns'
PLUGIN_FUNCTION_HOST = '_plugin_ns_internal'
# (namespace, node name). A name that starts with '_' is hidden.
NODES = (
    ('/plugin_ns', PLUGIN_FUNCTION_HOST),
    ('/hidden_only_ns', '_hidden_only_internal'),
    ('/visible_ns', 'visible_ns_node'),
)
# Created last, so a snapshot that lists it post-dates the nodes above.
WITNESS = 'hybrid_function_hosts_witness'

HEALTH_BUDGET = 30.0
SNAPSHOT_BUDGET = 30.0

_MANIFEST_PATHS = []


def _plugin_path():
    return os.path.join(
        get_package_prefix('ros2_medkit_integration_tests'),
        'lib', 'ros2_medkit_integration_tests', 'libtopics_test_plugin.so',
    )


def _manifest_yaml():
    """Declare nothing a namespace Function could merge with."""
    return """\
manifest_version: "1.0"
metadata:
  name: "Hybrid plugin Function hosts"
  version: "1.0.0"
config:
  unmanifested_nodes: "warn"
areas:
  - id: hybrid-fn-area
    name: "Hybrid Function Area"
"""


@pytest.mark.launch_test
def generate_test_description():
    """Launch one hybrid gateway with the test plugin declaring a Function."""
    fd, manifest_path = tempfile.mkstemp(suffix='.yaml', prefix='test_hybrid_plugin_function_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(_manifest_yaml())
    _MANIFEST_PATHS.append(manifest_path)

    gateway = create_gateway_node(
        port=PORT,
        extra_params={
            'server.host': '127.0.0.1',
            'refresh_interval_ms': 1000,
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'discovery.merge_pipeline.gap_fill.allow_heuristic_functions': True,
            'plugins': ['topics_test'],
            'plugins.topics_test.path': _plugin_path(),
            'plugins.topics_test.function_id': PLUGIN_FUNCTION_ID,
            'plugins.topics_test.function_host': PLUGIN_FUNCTION_HOST,
        },
    )
    return launch.LaunchDescription([
        gateway,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway}


def _get(path):
    return requests.get(f'{BASE_URL}{path}', timeout=10)


class TestHybridPluginFunctionHosts(unittest.TestCase):
    """Only a Function runtime discovery alone created is removed."""

    @classmethod
    def setUpClass(cls):
        """Start the nodes, then wait for a snapshot that lists the witness."""
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.nodes = []
        cls._wait_for_health()
        for namespace, name in NODES:
            cls.nodes.append(Node(name, namespace=namespace, context=cls.context))
        cls.nodes.append(Node(WITNESS, context=cls.context))
        cls.apps = cls._apps_until(WITNESS)
        cls.responses = {
            path: _get(path)
            for function_id in (PLUGIN_FUNCTION_ID, 'hidden_only_ns', 'visible_ns')
            for path in (f'/functions/{function_id}', f'/functions/{function_id}/hosts')
        }

    @classmethod
    def tearDownClass(cls):
        for node in cls.nodes:
            node.destroy_node()
        rclpy.shutdown(context=cls.context)

    @classmethod
    def _wait_for_health(cls):
        deadline = time.monotonic() + HEALTH_BUDGET * get_time_scale()
        last = None
        while time.monotonic() < deadline:
            try:
                response = _get('/health')
                if response.status_code == 200:
                    return
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(f'gateway not ready within {HEALTH_BUDGET}s (last: {last})')

    @classmethod
    def _apps_until(cls, app_id):
        deadline = time.monotonic() + SNAPSHOT_BUDGET * get_time_scale()
        apps = set()
        while time.monotonic() < deadline:
            try:
                apps = {item['id'] for item in _get('/apps').json().get('items', [])}
                if app_id in apps:
                    return apps
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(f'gateway never listed {app_id}. Listed: {sorted(apps)}')

    def _assert_hosts(self, function_id, expected):
        detail = self.responses[f'/functions/{function_id}']
        hosts = self.responses[f'/functions/{function_id}/hosts']
        self.assertEqual(detail.status_code, 200, detail.text)
        x_medkit = detail.json().get('x-medkit', {})
        self.assertEqual(set(x_medkit.get('hosts', [])), expected, x_medkit)
        self.assertEqual(hosts.status_code, 200, hosts.text)
        self.assertEqual({item['id'] for item in hosts.json().get('items', [])}, expected,
                         hosts.text)
        return x_medkit

    def test_a_plugin_function_that_loses_every_host_stays(self):
        """The runtime layer contributed first, the plugin keeps it served."""
        x_medkit = self._assert_hosts(PLUGIN_FUNCTION_ID, set())
        # Precondition: the runtime layer derived the same id, first.
        self.assertEqual(x_medkit.get('source'), 'runtime', x_medkit)

    def test_a_runtime_function_with_no_served_host_is_not_served(self):
        """Nothing but runtime discovery derived hidden_only_ns."""
        for path in ('/functions/hidden_only_ns', '/functions/hidden_only_ns/hosts'):
            with self.subTest(path=path):
                response = self.responses[path]
                self.assertEqual(response.status_code, 404, response.text)
                body = response.json()
                self.assertEqual(body.get('error_code'), 'entity-not-found', body)
                self.assertIn('hidden_only_ns', body.get('parameters', {}).values(), body)

    def test_a_runtime_function_keeps_its_served_hosts(self):
        """Positive control: runtime discovery derives Functions here."""
        self._assert_hosts('visible_ns', {'visible_ns_node'})
        self.assertNotIn(PLUGIN_FUNCTION_HOST, self.apps)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')

    def test_manifests_are_cleaned_up(self):
        """The temp manifest this module wrote does not outlive the run."""
        while _MANIFEST_PATHS:
            try:
                os.unlink(_MANIFEST_PATHS.pop())
            except OSError:
                pass
