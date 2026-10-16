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

"""In hybrid mode a Function hosts only Apps and Components the gateway serves.

Four gateways in one DDS domain. Each runs in a namespace of its own, so the
root namespace holds only a hidden node. The three hybrid gateways run with
``allow_heuristic_functions`` true, so runtime discovery derives a Function per
namespace from every node it sees.

Gateway A, hybrid, ``unmanifested_nodes: warn`` with ``/blocked_ns``
blacklisted, and the in-tree test plugin:

- ``plugin_ns``: one hidden node. The plugin declares a Function with that id
  hosting the node's App. The runtime layer contributes the id first, so the
  merged Function reads source ``runtime``. A plugin contributes it, so it
  stays, with no hosts.
- ``copied_ns``: one hidden node. The plugin returns an unchanged copy of the
  runtime layer's Function. That copy is a plugin contribution, so it stays.
- ``shared_hidden_ns``: one hidden node. The manifest declares a Function with
  the id runtime discovery also derives, hosted by that node. The manifest
  contributes it, so it stays, with no hosts.
- ``hidden_only_ns``, ``linked_ns`` (its node is bound to a manifest App, served
  under the manifest id), ``blocked_ns`` (blacklisted) and ``root``: no host is
  served and only runtime discovery derived the Function, so none is served.
- ``declared_fn``: a manifest Function whose only host names no entity. It
  stays, with no hosts.
- ``visible_ns`` and ``ignored_ns``: positive controls.

Gateway B, hybrid, ``unmanifested_nodes: ignore``: ``ignored_ns`` holds one
undeclared node, which B does not serve, so B serves no Function
``ignored_ns``.

Gateway C runs ``runtime_only`` with the same test plugin, which the gateway
injects next to runtime discovery's Functions instead of merging them. The
plugin's ``plugin_ns`` keeps that Function served there too. C serves
``blocked_node`` and ``linked_node`` under their runtime ids, so ``blocked_ns``
and ``linked_ns`` list them: the positive controls for A.

Gateway D, hybrid, ``allow_heuristic_apps`` false: it serves no undeclared
node, so ``visible_ns`` loses its only host and is not served.
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

GATEWAY_A_URL = f'http://127.0.0.1:{get_test_port(0)}{API_BASE_PATH}'
GATEWAY_B_URL = f'http://127.0.0.1:{get_test_port(1)}{API_BASE_PATH}'
GATEWAY_C_URL = f'http://127.0.0.1:{get_test_port(2)}{API_BASE_PATH}'
GATEWAY_D_URL = f'http://127.0.0.1:{get_test_port(3)}{API_BASE_PATH}'

PLUGIN_FUNCTION_ID = 'plugin_ns'
PLUGIN_FUNCTION_HOST = '_plugin_ns_internal'
COPIED_FUNCTION_ID = 'copied_ns'
SHARED_FUNCTION_ID = 'shared_hidden_ns'
SHARED_FUNCTION_HOST = '_shared_hidden_internal'
LINKED_APP_ID = 'linked_app'
WITNESS = 'hybrid_function_hosts_witness'
WITNESS_APP_ID = 'witness_app'
# (namespace, node name). A name that starts with '_' is hidden.
NODES = (
    ('/', '_root_internal'),
    ('/plugin_ns', PLUGIN_FUNCTION_HOST),
    ('/copied_ns', '_copied_internal'),
    ('/shared_hidden_ns', SHARED_FUNCTION_HOST),
    ('/hidden_only_ns', '_hidden_only_internal'),
    ('/visible_ns', 'visible_ns_node'),
    ('/linked_ns', 'linked_node'),
    ('/blocked_ns', 'blocked_node'),
    ('/ignored_ns', 'ignored_node'),
)

# Function id -> its hosts on gateway A, written out. None: not served.
EXPECTED_A = {
    PLUGIN_FUNCTION_ID: set(),
    COPIED_FUNCTION_ID: set(),
    SHARED_FUNCTION_ID: set(),
    'declared_fn': set(),
    'visible_ns': {'visible_ns_node'},
    'ignored_ns': {'ignored_node'},
    'hidden_only_ns': None,
    'linked_ns': None,
    'blocked_ns': None,
    'root': None,
}
EXPECTED_B = {
    'ignored_ns': None,
}
EXPECTED_C = {
    PLUGIN_FUNCTION_ID: set(),
    'blocked_ns': {'blocked_node'},
    'linked_ns': {'linked_node'},
}
EXPECTED_D = {
    'visible_ns': None,
}
# Gateway URL -> (App whose online state dates the snapshot, expected Functions)
GATEWAYS = (
    (GATEWAY_A_URL, WITNESS_APP_ID, EXPECTED_A),
    (GATEWAY_B_URL, WITNESS_APP_ID, EXPECTED_B),
    (GATEWAY_C_URL, WITNESS, EXPECTED_C),
    (GATEWAY_D_URL, WITNESS_APP_ID, EXPECTED_D),
)

HEALTH_BUDGET = 30.0
SNAPSHOT_BUDGET = 30.0

_MANIFEST_PATHS = []


def _plugin_path():
    return os.path.join(
        get_package_prefix('ros2_medkit_integration_tests'),
        'lib', 'ros2_medkit_integration_tests', 'libtopics_test_plugin.so',
    )


def _manifest_yaml(policy):
    """Declare the witness, one linked App and two Functions with no served host."""
    return f"""\
manifest_version: "1.0"
metadata:
  name: "Hybrid Function hosts"
  version: "1.0.0"
config:
  unmanifested_nodes: "{policy}"
areas:
  - id: hybrid-fn-area
    name: "Hybrid Function Area"
apps:
  - id: {WITNESS_APP_ID}
    name: "Witness"
    ros_binding:
      node_name: {WITNESS}
      namespace: /witness_ns
  - id: {LINKED_APP_ID}
    name: "Linked"
    ros_binding:
      node_name: linked_node
      namespace: /linked_ns
functions:
  - id: declared_fn
    name: "Declared Function"
    hosted_by:
      - no_such_app
  - id: {SHARED_FUNCTION_ID}
    name: "Shared Hidden Function"
    hosted_by:
      - {SHARED_FUNCTION_HOST}
"""


def _write_manifest(policy):
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix=f'test_hybrid_function_hosts_{policy}_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(_manifest_yaml(policy))
    _MANIFEST_PATHS.append(path)
    return path


@pytest.mark.launch_test
def generate_test_description():
    """Launch hybrid gateways A, B and D and the runtime_only gateway C."""
    common = {
        'server.host': '127.0.0.1',
        'refresh_interval_ms': 1000,
        'discovery.mode': 'hybrid',
        'discovery.manifest_strict_validation': False,
        'discovery.merge_pipeline.gap_fill.allow_heuristic_functions': True,
    }
    gateway_a = create_gateway_node(
        port=get_test_port(0), name='hybrid_gateway_a', namespace='/hybrid_a',
        extra_params={
            **common,
            'discovery.manifest_path': _write_manifest('warn'),
            'discovery.merge_pipeline.gap_fill.namespace_blacklist': ['/blocked_ns'],
            'plugins': ['topics_test'],
            'plugins.topics_test.path': _plugin_path(),
            'plugins.topics_test.function_id': PLUGIN_FUNCTION_ID,
            'plugins.topics_test.function_host': PLUGIN_FUNCTION_HOST,
            'plugins.topics_test.copy_function_id': COPIED_FUNCTION_ID,
        },
    )
    gateway_b = create_gateway_node(
        port=get_test_port(1), name='hybrid_gateway_b', namespace='/hybrid_b',
        extra_params={**common, 'discovery.manifest_path': _write_manifest('ignore')},
    )
    gateway_c = create_gateway_node(
        port=get_test_port(2), name='runtime_gateway_c', namespace='/runtime_c',
        extra_params={
            'server.host': '127.0.0.1',
            'refresh_interval_ms': 1000,
            'plugins': ['topics_test'],
            'plugins.topics_test.path': _plugin_path(),
            'plugins.topics_test.function_id': PLUGIN_FUNCTION_ID,
            'plugins.topics_test.function_host': PLUGIN_FUNCTION_HOST,
        },
    )
    gateway_d = create_gateway_node(
        port=get_test_port(3), name='hybrid_gateway_d', namespace='/hybrid_d',
        extra_params={
            **common,
            'discovery.manifest_path': _write_manifest('warn'),
            'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': False,
        },
    )
    return launch.LaunchDescription([
        gateway_a,
        gateway_b,
        gateway_c,
        gateway_d,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_a}


def _get(url):
    return requests.get(url, timeout=10)


class TestHybridFunctionHosts(unittest.TestCase):
    """Only a Function runtime discovery alone created is removed."""

    @classmethod
    def setUpClass(cls):
        """Start the nodes, then wait until each gateway linked the witness."""
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.nodes = []
        for url, _, _ in GATEWAYS:
            cls._wait_for_health(url)
        for namespace, name in NODES:
            cls.nodes.append(Node(name, namespace=namespace, context=cls.context))
        # Created last on the same context, so a snapshot in which the witness
        # App is online post-dates the nodes above.
        cls.nodes.append(Node(WITNESS, namespace='/witness_ns', context=cls.context))
        cls.apps = {url: cls._apps_once_online(url, witness) for url, witness, _ in GATEWAYS}
        cls.responses = {}
        for url, _, expected in GATEWAYS:
            for function_id in expected:
                for path in (f'/functions/{function_id}', f'/functions/{function_id}/hosts'):
                    cls.responses[(url, path)] = _get(f'{url}{path}')

    @classmethod
    def tearDownClass(cls):
        for node in cls.nodes:
            node.destroy_node()
        rclpy.shutdown(context=cls.context)

    @classmethod
    def _wait_for_health(cls, url):
        deadline = time.monotonic() + HEALTH_BUDGET * get_time_scale()
        last = None
        while time.monotonic() < deadline:
            try:
                response = _get(f'{url}/health')
                if response.status_code == 200:
                    return
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(f'{url} not ready within {HEALTH_BUDGET}s (last: {last})')

    @classmethod
    def _apps_once_online(cls, url, app_id):
        """Poll /apps until *app_id* is listed and bound to its node."""
        deadline = time.monotonic() + SNAPSHOT_BUDGET * get_time_scale()
        items = {}
        while time.monotonic() < deadline:
            try:
                items = {item['id']: item for item in _get(f'{url}/apps').json().get('items', [])}
                if items.get(app_id, {}).get('x-medkit', {}).get('is_online'):
                    return items
            except requests.RequestException:
                pass
            time.sleep(0.5)
        raise AssertionError(f'{url} never listed {app_id} online. Listed: {sorted(items)}')

    def _assert_function(self, url, function_id, expected):
        detail = self.responses[(url, f'/functions/{function_id}')]
        hosts = self.responses[(url, f'/functions/{function_id}/hosts')]
        if expected is None:
            for response in (detail, hosts):
                self.assertEqual(response.status_code, 404, response.text)
                body = response.json()
                self.assertEqual(body.get('error_code'), 'entity-not-found', body)
                self.assertIn(function_id, body.get('parameters', {}).values(), body)
            return None
        self.assertEqual(detail.status_code, 200, detail.text)
        x_medkit = detail.json().get('x-medkit', {})
        self.assertEqual(set(x_medkit.get('hosts', [])), expected, x_medkit)
        self.assertEqual(hosts.status_code, 200, hosts.text)
        self.assertEqual({item['id'] for item in hosts.json().get('items', [])}, expected,
                         hosts.text)
        return x_medkit

    def test_every_function_hosts_only_served_entities(self):
        """Each Function has exactly the expected hosts, or is not served."""
        for url, _, expected in GATEWAYS:
            for function_id, hosts in expected.items():
                with self.subTest(gateway=url, function=function_id):
                    self._assert_function(url, function_id, hosts)

    def test_the_runtime_layer_contributed_the_plugin_function_first(self):
        """Precondition: plugin_ns reads source runtime, so source cannot decide."""
        detail = self.responses[(GATEWAY_A_URL, f'/functions/{PLUGIN_FUNCTION_ID}')]
        self.assertEqual(detail.status_code, 200, detail.text)
        self.assertEqual(detail.json().get('x-medkit', {}).get('source'), 'runtime', detail.text)

    def test_no_removed_host_is_a_served_app(self):
        """Precondition: each removed host names no App the gateway serves.

        The positive controls on A and C show that each node is live.
        """
        apps_a = self.apps[GATEWAY_A_URL]
        self.assertIn(LINKED_APP_ID, apps_a)
        self.assertTrue(apps_a[LINKED_APP_ID].get('x-medkit', {}).get('is_online'), apps_a)
        for app_id in ('linked_node', 'blocked_node', PLUGIN_FUNCTION_HOST,
                       SHARED_FUNCTION_HOST, '_root_internal'):
            self.assertNotIn(app_id, apps_a)
        self.assertIn('visible_ns_node', apps_a)
        self.assertNotIn('ignored_node', self.apps[GATEWAY_B_URL])
        self.assertNotIn('visible_ns_node', self.apps[GATEWAY_D_URL])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')

    def test_manifests_are_cleaned_up(self):
        """The temp manifests this module wrote do not outlive the run."""
        while _MANIFEST_PATHS:
            try:
                os.unlink(_MANIFEST_PATHS.pop())
            except OSError:
                pass
