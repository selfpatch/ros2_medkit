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

"""Integration test: an unreachable scoped node degrades pipeline_status.

A manifest defines a Function hosting two apps (a sensor and a monitor).
Both nodes run, so the Function's x-medkit-graph reads ``"healthy"``. The
monitor node is then killed. The manifest keeps it scoped, the runtime
linker marks it offline, and its ``node_status`` becomes ``"unreachable"``.
Because a dead node carries no topics it contributes no edge, so edge health
alone would still read ``"healthy"`` - this drives the whole stack end to end
to prove the ``pipeline_status`` rollup folds node reachability in: the graph
must read ``"degraded"``.

The monitor is launched as a subprocess (not through the launch harness) so
the test can kill it mid-suite; the sensor stays up throughout as the
surviving reachable node.
"""

import os
import signal
import subprocess
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix
import launch_testing

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DEFAULT_DOMAIN_ID,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_test_launch,
    DEMO_NODE_REGISTRY,
    get_coverage_env,
)


FUNCTION_ID = 'engine-monitoring'
NAMESPACE = '/powertrain/engine'
SENSOR_ID = 'engine-temp-sensor'
MONITOR_ID = 'engine-temp-monitor'
MONITOR_NODE_KEY = 'temp_monitor'


def _graph_plugin_path():
    prefix = get_package_prefix('ros2_medkit_graph_provider')
    return os.path.join(
        prefix, 'lib', 'ros2_medkit_graph_provider',
        'libros2_medkit_graph_provider_plugin.so',
    )


def _write_manifest(content):
    fd, path = tempfile.mkstemp(
        suffix='.yaml', prefix='test_graph_provider_unreachable_manifest_',
    )
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


MANIFEST_YAML = f"""\
manifest_version: "1.0"
metadata:
  name: "Unreachable Node Test Vehicle"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: powertrain
    name: "Powertrain"
components:
  - id: engine-ecu
    name: "Engine ECU"
    area: powertrain
apps:
  - id: {SENSOR_ID}
    name: "Engine Temperature Sensor"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: {NAMESPACE}
  - id: {MONITOR_ID}
    name: "Engine Temperature Monitor"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_monitor
      namespace: {NAMESPACE}
functions:
  - id: {FUNCTION_ID}
    name: "Engine Monitoring"
    category: monitoring
    hosted_by:
      - {SENSOR_ID}
      - {MONITOR_ID}
"""


def generate_test_description():
    manifest_path = _write_manifest(MANIFEST_YAML)
    # Only the sensor is launched through the harness; the monitor is spawned
    # as a killable subprocess in setUpClass.
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'plugins': ['graph_provider'],
            'plugins.graph_provider.path': _graph_plugin_path(),
        },
    )


def _resolve_demo_executable(name):
    pkg = 'ros2_medkit_integration_tests'
    prefix = get_package_prefix(pkg)
    candidate = os.path.join(prefix, 'lib', pkg, name)
    if not os.path.isfile(candidate):
        raise FileNotFoundError(f'demo executable not found: {candidate}')
    return candidate


def _terminate(proc):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


class TestGraphProviderUnreachable(GatewayTestCase):
    """A killed, manifest-scoped node degrades the Function's pipeline_status."""

    REQUIRED_FUNCTIONS = {FUNCTION_ID}
    REQUIRED_APPS = {SENSOR_ID}
    MIN_EXPECTED_APPS = 1

    GRAPH_ENDPOINT = f'/functions/{FUNCTION_ID}/x-medkit-graph'

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[MONITOR_NODE_KEY]
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
        env.update(get_coverage_env())
        binary = _resolve_demo_executable(executable)
        cls._monitor_proc = subprocess.Popen(
            [
                binary,
                '--ros-args',
                '-r', f'__ns:={namespace}',
                '-r', f'__node:={ros_name}',
            ],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    @classmethod
    def tearDownClass(cls):
        _terminate(cls._monitor_proc)
        cls._monitor_proc = None
        super().tearDownClass()

    @staticmethod
    def _find_node(graph, entity_id):
        for node in graph.get('nodes', []):
            if node.get('entity_id') == entity_id:
                return node
        return None

    def test_killed_scoped_node_degrades_pipeline(self):
        # Both nodes up: the monitor is reachable and the graph reads healthy.
        def _monitor_reachable_and_healthy(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            monitor = self._find_node(graph, MONITOR_ID)
            if (monitor and monitor.get('node_status') == 'reachable'
                    and graph.get('pipeline_status') == 'healthy'):
                return graph
            return None

        self.poll_endpoint_until(
            self.GRAPH_ENDPOINT, _monitor_reachable_and_healthy,
            timeout=40.0, interval=1.0,
        )

        # Kill the monitor. The manifest keeps it scoped, the runtime linker
        # marks it offline, so it must read unreachable and pull the pipeline
        # down to degraded even though its (now absent) edge no longer errors.
        _terminate(type(self)._monitor_proc)
        type(self)._monitor_proc = None

        def _monitor_unreachable_and_degraded(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            monitor = self._find_node(graph, MONITOR_ID)
            if (monitor and monitor.get('node_status') == 'unreachable'
                    and graph.get('pipeline_status') == 'degraded'):
                return graph
            return None

        graph = self.poll_endpoint_until(
            self.GRAPH_ENDPOINT, _monitor_unreachable_and_degraded,
            timeout=60.0, interval=2.0,
        )

        # The surviving sensor is still reachable; only the killed monitor is
        # unreachable, and that alone makes the pipeline degraded.
        sensor = self._find_node(graph, SENSOR_ID)
        monitor = self._find_node(graph, MONITOR_ID)
        self.assertIsNotNone(sensor, 'sensor node missing from graph')
        self.assertIsNotNone(monitor, 'monitor node missing from graph')
        self.assertEqual(sensor['node_status'], 'reachable')
        self.assertEqual(monitor['node_status'], 'unreachable')
        self.assertEqual(graph['pipeline_status'], 'degraded')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All launch-managed processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
