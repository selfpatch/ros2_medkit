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

"""Integration tests for the graph provider's freshness/staleness model.

Drives the SAME real-``greenwave_monitor`` harness as
``test_graph_provider_greenwave.test.py`` (Task 8's acceptance-gate suite),
then goes one step further: it makes the primary topic's real publisher go
silent and proves the edge transitions to ``metrics_status: "error"`` with
``error_reason: "metrics_stale"`` - the freshness-window failure signal that
replaced the old, buggy global ``no_data_source`` flag - and that
``pipeline_status`` becomes ``"broken"``. It then restores the publisher and
proves recovery back to ``"active"`` / ``"healthy"``.

Silencing mechanism (see the module docstring investigation notes below for
why this is the reliable lever, not a guess):

``engine_temp_sensor``'s ``publish_rate`` parameter is live-tunable (the same
direct ``rcl_interfaces/srv/SetParameters`` service-client mechanism the
acceptance-gate suite uses for its degradation case) but is clamped to
>= 0.1 Hz - it can never be driven to a literal 0. Dropping it to 0.1 Hz
(one message every 10s) still reliably triggers ``metrics_stale``:
empirically probing the real
``greenwave_monitor`` binary (ros-jazzy-greenwave-monitor, vendored, no
local source) shows it declares its OWN "DIAGNOSTICS STALE" state roughly
1-1.5s after a monitored topic's last message, and when in that state its
published ``/diagnostics`` status for the topic carries NO recognized
key/value pairs at all (not even ``frame_rate_msg: 0`` - the key is simply
absent). ``parse_topic_metrics`` requires at least one recognized key to
produce a ``TopicMetrics`` update, so a status with none is skipped
entirely and the plugin's cached ``last_update_ns`` for that topic stops
advancing well before the next real message arrives 10s later. That gives a
clean, predictable 50/50 duty cycle once the rate is dropped: ~5s "active"
(the plugin's own default ``freshness_floor_sec``), then ~5s
"error"/"metrics_stale", repeating every 10s - so a poll budgeting well
past one full cycle reliably observes the stale state.

This mechanism was chosen over killing the sensor's process or killing
greenwave_monitor's process: killing the publisher's node would make its
App go offline, which empties its topic list and removes the EDGE from the
graph entirely (a different code path - "topic missing" - not the freshness
regression this suite targets). Killing greenwave_monitor works too, but
would need out-of-band subprocess control instead of reusing the launch
harness's ``create_greenwave_node`` factory outright, and would make
"resume" require restarting a process rather than a live parameter set.
"""

import os
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
    create_greenwave_node,
)


FUNCTION_ID = 'engine-monitoring'

PRIMARY_NAMESPACE = '/powertrain/engine'
PRIMARY_TOPIC = f'{PRIMARY_NAMESPACE}/temperature'

PRIMARY_SOURCE_ID = 'engine-temp-sensor'
PRIMARY_TARGET_ID = 'engine-temp-monitor'

GREENWAVE_NODE_NAME = 'greenwave_monitor'

# temp_sensor's own default publish_rate. Aligning the graph provider's
# expected frequency to this value keeps case 1 (baseline) and case 3
# (recovery) reading "active"/"healthy" instead of "degraded" - see
# test_graph_provider_greenwave.test.py's module docstring for the full
# 2/30 Hz misalignment rationale this mirrors.
EXPECTED_FREQUENCY_HZ = 2.0
HEALTHY_PUBLISH_RATE_HZ = 2.0

# temp_sensor clamps publish_rate to >= 0.1 Hz (see engine_temp_sensor.cpp's
# on_parameter_change) - it can never be driven to a literal 0. A 10s period
# is comfortably past the plugin's default 5s freshness floor, so every
# 10s cycle spends ~5s "active" (the last real message survives the floor)
# and ~5s "error"/"metrics_stale" (empirically confirmed against the real
# greenwave_monitor binary - see the module docstring).
STALE_PUBLISH_RATE_HZ = 0.1

# Demo nodes start on this timer (matches create_test_launch's default).
DEMO_DELAY_SEC = 2.0
# greenwave_monitor resolves its monitored topics ONCE at startup - see
# create_greenwave_node's docstring - so it must start strictly after the
# demo pair is already publishing.
GREENWAVE_DELAY_SEC = 10.0


def _get_graph_plugin_path():
    """Get path to the graph provider plugin .so."""
    pkg_prefix = get_package_prefix('ros2_medkit_graph_provider')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_graph_provider',
        'libros2_medkit_graph_provider_plugin.so',
    )


def _write_manifest(content):
    """Write manifest YAML to a temporary file, return its path."""
    fd, path = tempfile.mkstemp(
        suffix='.yaml', prefix='test_graph_provider_stale_manifest_',
    )
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


# Private manifest (not the shared demo_nodes_manifest.yaml): a single
# temp_sensor -> temp_monitor pair, the minimum needed to exercise one edge's
# freshness lifecycle.
MANIFEST_YAML = f"""\
manifest_version: "1.0"
metadata:
  name: "Graph Provider Staleness Test Vehicle"
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
  - id: {PRIMARY_SOURCE_ID}
    name: "Engine Temperature Sensor"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: {PRIMARY_NAMESPACE}
  - id: {PRIMARY_TARGET_ID}
    name: "Engine Temperature Monitor"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_monitor
      namespace: {PRIMARY_NAMESPACE}
functions:
  - id: {FUNCTION_ID}
    name: "Engine Monitoring"
    category: monitoring
    hosted_by:
      - {PRIMARY_SOURCE_ID}
      - {PRIMARY_TARGET_ID}
"""


def generate_test_description():
    """Launch gateway + graph provider + demo pair + real greenwave_monitor."""
    manifest_path = _write_manifest(MANIFEST_YAML)
    plugin_path = _get_graph_plugin_path()

    demo_pair = create_demo_nodes(['temp_sensor', 'temp_monitor'])
    demo_timer = TimerAction(period=DEMO_DELAY_SEC, actions=demo_pair)

    # See create_greenwave_node's docstring and GREENWAVE_DELAY_SEC above:
    # this MUST fire strictly after demo_timer's nodes are already
    # publishing, so it gets its own, later TimerAction.
    greenwave_timer = TimerAction(
        period=GREENWAVE_DELAY_SEC,
        actions=[create_greenwave_node(
            monitored_topics=[PRIMARY_TOPIC],
            name=GREENWAVE_NODE_NAME,
        )],
    )

    override_key = (
        f'plugins.graph_provider.function_overrides.{FUNCTION_ID}'
        '.expected_frequency_hz'
    )
    gateway_node = create_gateway_node(
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'plugins': ['graph_provider'],
            'plugins.graph_provider.path': plugin_path,
            override_key: EXPECTED_FREQUENCY_HZ,
        },
    )

    launch_description = LaunchDescription([
        gateway_node,
        demo_timer,
        greenwave_timer,
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {'gateway_node': gateway_node},
    )


class TestGraphProviderStale(GatewayTestCase):
    """Freshness/staleness lifecycle of the x-medkit-graph metrics model.

    @verifies REQ_INTEROP_003
    """

    REQUIRED_FUNCTIONS = {FUNCTION_ID}
    REQUIRED_APPS = {PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID}
    MIN_EXPECTED_APPS = 2

    GRAPH_ENDPOINT = f'/functions/{FUNCTION_ID}/x-medkit-graph'

    @classmethod
    def setUpClass(cls):
        """Wait for gateway/discovery, then set up a live parameter client.

        Targets the real temp_sensor node's ``set_parameters`` service so
        tests can drop and restore its publish_rate live, mid-suite (see the
        module docstring for why this is the reliable lever for both the
        stale and recovery cases). A raw ``rcl_interfaces/srv/
        SetParameters`` client is used directly (rather than
        ``rclpy.parameter_client.AsyncParameterClient``, which is Jazzy+
        only) so this suite also loads and runs on Humble.
        """
        super().setUpClass()
        rclpy.init()
        cls._param_node = Node('graph_provider_stale_param_client')
        cls._param_client = cls._param_node.create_client(
            SetParameters, f'{PRIMARY_NAMESPACE}/temp_sensor/set_parameters',
        )

    @classmethod
    def tearDownClass(cls):
        cls._param_node.destroy_node()
        rclpy.shutdown()

    @staticmethod
    def _find_edge(graph, source_id, target_id):
        """Find an edge by (source entity id, target entity id)."""
        for edge in graph.get('edges', []):
            if edge.get('source') == source_id and edge.get('target') == target_id:
                return edge
        return None

    def _set_publish_rate(self, rate_hz):
        """Live-set temp_sensor's publish_rate parameter, asserting success."""
        self.assertTrue(
            self._param_client.wait_for_service(timeout_sec=15.0),
            'temp_sensor parameter services not available',
        )
        request = SetParameters.Request()
        request.parameters = [
            Parameter('publish_rate', Parameter.Type.DOUBLE, rate_hz).to_parameter_msg(),
        ]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self._param_node, future, timeout_sec=10.0)
        result = future.result()
        self.assertIsNotNone(result, 'set_parameters call to temp_sensor timed out')
        self.assertTrue(
            result.results[0].successful,
            f'temp_sensor rejected publish_rate={rate_hz}: {result.results[0].reason}',
        )

    def _poll_until_primary_active(self, *, timeout=40.0):
        """Poll the function graph until the primary edge reports active.

        Mirrors test_graph_provider_greenwave.test.py's helper of the same
        name: requires a populated ``frequency_hz``, not just
        ``metrics_status == 'active'``, to skip past greenwave's own
        transient first sample(s).
        """

        def _condition(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            edge = self._find_edge(graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID)
            if (edge and edge['metrics'].get('metrics_status') == 'active'
                    and edge['metrics'].get('frequency_hz') is not None):
                return graph
            return None

        return self.poll_endpoint_until(
            self.GRAPH_ENDPOINT, _condition, timeout=timeout, interval=1.0,
        )

    def test_01_baseline_edge_becomes_active(self):
        """Baseline: real /diagnostics from greenwave_monitor drive the edge active.

        Establishes the known-good starting point before the staleness case
        silences the publisher.

        @verifies REQ_INTEROP_003
        """
        graph = self._poll_until_primary_active()
        edge = self._find_edge(graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID)
        self.assertIsNotNone(edge, 'temp_sensor -> temp_monitor edge not in graph')
        self.assertEqual(edge['metrics']['metrics_status'], 'active')
        self.assertEqual(graph['pipeline_status'], 'healthy')

    def test_02_publisher_goes_silent_edge_becomes_stale(self):
        """Silencing the real publisher trips metrics_stale and breaks the pipeline.

        This is the regression guard for the freshness model (Task 2): a
        plugin that never re-checks freshness (e.g. one that latches
        "active" forever once any message has ever been merged) would never
        observe 'error'/'metrics_stale' here and this poll would time out -
        it does NOT pass by construction.

        @verifies REQ_INTEROP_003
        """
        # Make sure we start from a confirmed-active baseline (test_01
        # already established this, but re-confirm in case test ordering
        # or reruns land here first).
        self._poll_until_primary_active()

        self._set_publish_rate(STALE_PUBLISH_RATE_HZ)

        def _condition(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            edge = self._find_edge(graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID)
            if (edge and edge['metrics'].get('metrics_status') == 'error'
                    and graph.get('pipeline_status') == 'broken'):
                return (graph, edge)
            return None

        # Generous budget: the stale window only occupies ~5s of every 10s
        # duty cycle once publish_rate drops (see module docstring), so
        # comfortably cover several cycles rather than betting on the first.
        graph, edge = self.poll_endpoint_until(
            self.GRAPH_ENDPOINT, _condition, timeout=60.0, interval=1.0,
        )
        self.assertEqual(edge['metrics']['metrics_status'], 'error')
        self.assertEqual(edge['metrics']['error_reason'], 'metrics_stale')
        self.assertEqual(graph['pipeline_status'], 'broken')

    def test_03_publisher_resumes_edge_recovers_to_active(self):
        """Restoring the publisher's rate recovers the edge to active/healthy.

        Waits for ``pipeline_status == 'healthy'``, not just
        ``metrics_status == 'active'``: greenwave's own frequency estimate
        is a rolling measurement and reads low for the first sample or two
        right after the rate change (its window still contains the old
        stale-period timing gaps), which briefly computes a "degraded"
        ratio even though the edge is already freshness-wise "active"
        again. Waiting for the pipeline to read healthy skips past that
        transient the same way ``_poll_until_primary_active`` skips past
        greenwave's own zero-rate warmup sample.

        @verifies REQ_INTEROP_003
        """
        self._set_publish_rate(HEALTHY_PUBLISH_RATE_HZ)

        def _condition(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            edge = self._find_edge(graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID)
            if (edge and edge['metrics'].get('metrics_status') == 'active'
                    and graph.get('pipeline_status') == 'healthy'):
                return (graph, edge)
            return None

        # Generous budget: greenwave needs several new, real 2 Hz samples
        # after the parameter change to flush the stale-period timing gaps
        # out of its own rolling rate estimate.
        graph, edge = self.poll_endpoint_until(
            self.GRAPH_ENDPOINT, _condition, timeout=60.0, interval=2.0,
        )
        self.assertEqual(edge['metrics']['metrics_status'], 'active')
        self.assertNotIn('error_reason', edge['metrics'])
        self.assertEqual(graph['pipeline_status'], 'healthy')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify gateway and demo processes exit cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
