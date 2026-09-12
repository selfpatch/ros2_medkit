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

"""The gateway does not ask the operator to declare its own plumbing.

In hybrid mode with ``unmanifested_nodes: error``, every running node that no
manifest app binds is reported on ``GET /health`` as an ``unmanifested_nodes``
warning, and the message is an instruction: "Declare them in the manifest".

The gateway's three in-process helper nodes must not be in that list. They can
never become apps - the app filter removes them - so declaring them would
silence the warning and produce manifest entities the gateway then deletes. The
two halves have to agree: what is never an App is never something to declare.

Launched without a ``__node`` remap for the same reason as
``test_own_node_apps.test.py``: ``launch_ros``' ``name=`` renames every node in
the process, so under the suite's usual launch the helper names do not exist
and nothing here could be observed.
"""

import os
import tempfile
import time
import unittest

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

GATEWAY_PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{GATEWAY_PORT}{API_BASE_PATH}'

GATEWAY_NODE = 'ros2_medkit_gateway'
HELPER_FQNS = (
    f'/{GATEWAY_NODE}_sub',
    f'/{GATEWAY_NODE}_fault_clients',
    f'/{GATEWAY_NODE}_lifecycle_state_reader',
)
# An undeclared node of the test's own, so the warning below is known to be
# listing things rather than empty for an unrelated reason.
WITNESS_NODE = 'own_node_undeclared_witness'

WARN_UNMANIFESTED_NODES = 'unmanifested_nodes'

HEALTH_BUDGET = 30.0
HELPERS_ON_GRAPH_BUDGET = 20.0
WARNING_BUDGET = 25.0

_MANIFEST_DIR = tempfile.mkdtemp(prefix='medkit-own-node-undeclared-')
_MANIFEST_PATH = os.path.join(_MANIFEST_DIR, 'manifest.yaml')

# One app bound to a node this launch never starts: the manifest is valid and
# loaded, nothing links, so every running node is undeclared and the policy has
# something to report.
with open(_MANIFEST_PATH, 'w') as _manifest:
    _manifest.write("""\
manifest_version: "1.0"
metadata:
  name: "Own node undeclared test vehicle"
  version: "1.0.0"
config:
  unmanifested_nodes: "error"
areas:
  - id: test_area
    name: "Test Area"
components:
  - id: test_ecu
    name: "Test ECU"
    area: test_area
apps:
  - id: absent_app
    name: "An app whose node is not running"
    is_located_on: test_ecu
    ros_binding:
      node_name: absent_node
      namespace: /nowhere
""")


@pytest.mark.launch_test
def generate_test_description():
    """Launch one hybrid gateway keeping its process's real node names."""
    gateway_node = create_gateway_node(
        port=GATEWAY_PORT,
        name=None,
        extra_params={
            'server.host': '127.0.0.1',
            'refresh_interval_ms': 1000,
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': _MANIFEST_PATH,
        },
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


class TestOwnNodeUndeclared(unittest.TestCase):
    """The undeclared-node warning and the app filter agree on the helpers."""

    @classmethod
    def setUpClass(cls):
        """Wait for the gateway, then settle the warning against the helpers."""
        cls.session = requests.Session()
        cls.context = Context()
        rclpy.init(context=cls.context)
        cls.probe = Node('own_node_undeclared_probe', context=cls.context)
        cls.witness = None
        cls.graph_fqns = set()
        cls.warning = None

        cls._wait_for_health()
        cls.graph_fqns = cls._graph_fqns_until(set(HELPER_FQNS), HELPERS_ON_GRAPH_BUDGET)
        if not set(HELPER_FQNS) <= cls.graph_fqns:
            return

        # Created after the helpers, and undeclared like them. Waiting for the
        # warning to name it dates the report: a report that has seen the
        # witness has seen the helpers, which appeared earlier.
        cls.witness = Node(WITNESS_NODE, context=cls.context)
        cls.warning = cls._warning_until_lists(f'/{WITNESS_NODE}', WARNING_BUDGET)

    @classmethod
    def tearDownClass(cls):
        if cls.witness is not None:
            cls.witness.destroy_node()
        cls.probe.destroy_node()
        rclpy.shutdown(context=cls.context)
        cls.session.close()

    @classmethod
    def _wait_for_health(cls):
        deadline = time.monotonic() + HEALTH_BUDGET * get_time_scale()
        last = None
        while time.monotonic() < deadline:
            try:
                response = cls.session.get(f'{BASE_URL}/health', timeout=5)
                if response.status_code == 200:
                    return
                last = response.status_code
            except requests.RequestException as exc:
                last = str(exc)
            time.sleep(0.5)
        raise AssertionError(
            f'gateway not ready within {HEALTH_BUDGET}s (last: {last})')

    @classmethod
    def _graph_fqns_until(cls, awaited, budget):
        """Node FQNs on the graph, polled until *awaited* is a subset."""
        deadline = time.monotonic() + budget * get_time_scale()
        while True:
            fqns = {
                (namespace.rstrip('/') + '/' + name)
                for name, namespace in cls.probe.get_node_names_and_namespaces()
            }
            if awaited <= fqns or time.monotonic() >= deadline:
                return fqns
            time.sleep(0.2)

    @classmethod
    def _warning_until_lists(cls, node_fqn, budget):
        """Poll /health until the unmanifested_nodes warning names *node_fqn*."""
        deadline = time.monotonic() + budget * get_time_scale()
        warning = None
        while True:
            body = cls.session.get(f'{BASE_URL}/health', timeout=10).json()
            for candidate in body.get('warnings', []):
                if candidate.get('code') == WARN_UNMANIFESTED_NODES:
                    warning = candidate
                    break
            if warning is not None and node_fqn in warning.get('ros_node_fqns', []):
                return warning
            if time.monotonic() >= deadline:
                return warning
            time.sleep(0.5)

    def test_the_undeclared_warning_is_reporting(self):
        """The policy is active and the warning lists the nodes it found.

        Without this the absence check below would pass on a gateway that
        reported nothing at all.
        """
        self.assertTrue(
            set(HELPER_FQNS) <= self.graph_fqns,
            f'helper nodes missing from the graph: '
            f'{sorted(set(HELPER_FQNS) - self.graph_fqns)}. '
            f'Nodes seen: {sorted(self.graph_fqns)}')

        self.assertIsNotNone(
            self.warning,
            f'no "{WARN_UNMANIFESTED_NODES}" warning on /health, so this file '
            f'cannot tell a filtered helper from an unreported one')
        self.assertIn(
            f'/{WITNESS_NODE}', self.warning.get('ros_node_fqns', []),
            f'the warning never named "{WITNESS_NODE}", so no report is known '
            f'to post-date the helper nodes. Listed: '
            f'{sorted(self.warning.get("ros_node_fqns", []))}')

    def test_the_helper_nodes_are_not_reported_as_undeclared(self):
        """The gateway's own helper nodes are not something to declare."""
        self.assertIsNotNone(self.warning, 'no unmanifested_nodes warning to read')
        reported = set(self.warning.get('ros_node_fqns', []))
        self.assertFalse(
            reported & set(HELPER_FQNS),
            f'the gateway asked the operator to declare its own in-process '
            f'helper nodes: {sorted(reported & set(HELPER_FQNS))}. Declaring '
            f'them produces entities the app filter then removes. Reported: '
            f'{sorted(reported)}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
