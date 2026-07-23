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

"""Acceptance-gate integration tests for the graph provider metrics path.

Every earlier graph provider task (status model, config, filter, versioning,
provenance) was proven at the unit level only. This suite is the first to
drive REAL ``/diagnostics`` metrics from a real ``greenwave_monitor`` node
through the whole stack (gateway + hybrid manifest + demo nodes + greenwave)
and assert on the operator-visible ``x-medkit-graph`` document - the
end-to-end proof that the feature actually works.

Topology (a private manifest, NOT the shared demo_nodes_manifest.yaml,
so this test cannot affect any other integration test):

- A "primary" temp_sensor -> temp_monitor pair under ``/powertrain/engine``
  (reusing the existing ``demo_engine_temp_sensor`` /
  ``demo_engine_temp_monitor`` executables). greenwave_monitor watches this
  pair's topic, so its edge in the ``engine-monitoring`` function graph goes
  active.
- An "uncovered" second instance of the same pair under
  ``/powertrain/engine_uncovered``, hosted by the SAME function.
  greenwave_monitor never monitors this topic, so its edge must stay
  ``pending`` forever - the end-to-end form of the false-"broken"
  regression (unit-covered already; this confirms the whole stack agrees).

Degradation (case 3) is triggered by a LIVE parameter change to the primary
temp_sensor's ``publish_rate`` partway through the suite, via a direct
``rcl_interfaces/srv/SetParameters`` service call against the node's own
``<node>/set_parameters`` service (``rclpy.parameter_client`` is Jazzy+ only
and would break this suite's launch under Humble CI), not by a node that is
slow from launch. A permanently-slow third pair would race case 2's
"pipeline stays healthy" assertion, since nothing paces when greenwave first
observes enough low-rate samples to call it "active" - the live change makes
the transition deterministic and keeps case 2 and case 3 from interfering.

Expected-frequency alignment: temp_sensor publishes at 2 Hz, but the graph
provider's global ``expected_frequency_hz_default`` is 30 Hz, so an unaligned
2/30 ratio would read "degraded" even when perfectly healthy. This suite
aligns via the PER-FUNCTION override
(``plugins.graph_provider.function_overrides.<function_id>.expected_frequency_hz``,
fixed by an earlier task in this series) rather than greenwave's own
``gw_frequency_monitored_topics`` per-topic ``expected_frequency`` field,
because the per-message field always wins over any graph-provider-side
config and would make the override inert. Using the override here instead
gives this acceptance test live coverage of that code path.
"""

import os
import tempfile
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.coverage import get_coverage_env
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
    create_greenwave_node,
)


FUNCTION_ID = 'engine-monitoring'

PRIMARY_NAMESPACE = '/powertrain/engine'
UNCOVERED_NAMESPACE = '/powertrain/engine_uncovered'
PRIMARY_TOPIC = f'{PRIMARY_NAMESPACE}/temperature'

PRIMARY_SOURCE_ID = 'engine-temp-sensor'
PRIMARY_TARGET_ID = 'engine-temp-monitor'
UNCOVERED_SOURCE_ID = 'engine-temp-sensor-uncovered'
UNCOVERED_TARGET_ID = 'engine-temp-monitor-uncovered'

GREENWAVE_NODE_NAME = 'greenwave_monitor'
EXPECTED_GREENWAVE_SOURCE = f'/{GREENWAVE_NODE_NAME}'

# temp_sensor's own default publish_rate (engine_temp_sensor.cpp). Aligning
# the graph provider's expected frequency to this value means a healthy
# temp_sensor measures at ~1.0x ratio instead of 2/30 ~= 0.067 (degraded).
EXPECTED_FREQUENCY_HZ = 2.0

# Well under EXPECTED_FREQUENCY_HZ * degraded_frequency_ratio (0.5), so the
# degraded ratio (~0.15) clears the threshold with a wide safety margin
# against greenwave's own measurement noise.
DEGRADED_PUBLISH_RATE_HZ = 0.3

# Demo nodes start on this timer (matches create_test_launch's default).
DEMO_DELAY_SEC = 2.0
# greenwave_monitor resolves its monitored topics ONCE at startup and never
# re-resolves them, so it MUST start strictly after the demo nodes above are
# already publishing (their publishers must already be advertised in the ROS
# graph). This gives an 8s buffer past DEMO_DELAY_SEC - generous relative to
# demo node process/DDS-discovery startup, which is sub-second in practice.
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
        suffix='.yaml', prefix='test_graph_provider_greenwave_manifest_',
    )
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


# Private manifest (not the shared demo_nodes_manifest.yaml): only the two
# pairs this suite needs, both hosted by the same function so both edges
# show up in one GET /functions/engine-monitoring/x-medkit-graph query.
MANIFEST_YAML = f"""\
manifest_version: "1.0"
metadata:
  name: "Greenwave Acceptance Test Vehicle"
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
  - id: {UNCOVERED_SOURCE_ID}
    name: "Engine Temperature Sensor (uncovered by greenwave)"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_sensor_uncovered
      namespace: {UNCOVERED_NAMESPACE}
  - id: {UNCOVERED_TARGET_ID}
    name: "Engine Temperature Monitor (uncovered by greenwave)"
    is_located_on: engine-ecu
    ros_binding:
      node_name: temp_monitor_uncovered
      namespace: {UNCOVERED_NAMESPACE}
functions:
  - id: {FUNCTION_ID}
    name: "Engine Monitoring"
    category: monitoring
    hosted_by:
      - {PRIMARY_SOURCE_ID}
      - {PRIMARY_TARGET_ID}
      - {UNCOVERED_SOURCE_ID}
      - {UNCOVERED_TARGET_ID}
"""


def generate_test_description():
    """Launch gateway + graph provider + demo pairs + real greenwave_monitor."""
    manifest_path = _write_manifest(MANIFEST_YAML)
    plugin_path = _get_graph_plugin_path()

    primary_pair = create_demo_nodes(['temp_sensor', 'temp_monitor'])
    uncovered_pair = [
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_engine_temp_sensor',
            name='temp_sensor_uncovered',
            namespace=UNCOVERED_NAMESPACE,
            output='screen',
            additional_env=get_coverage_env(),
            # Give the node room to flush coverage data at shutdown before SIGKILL.
            sigterm_timeout='30',
            sigkill_timeout='15',
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_engine_temp_monitor',
            name='temp_monitor_uncovered',
            namespace=UNCOVERED_NAMESPACE,
            output='screen',
            additional_env=get_coverage_env(),
            # Give the node room to flush coverage data at shutdown before SIGKILL.
            sigterm_timeout='30',
            sigkill_timeout='15',
        ),
    ]
    demo_timer = TimerAction(
        period=DEMO_DELAY_SEC,
        actions=primary_pair + uncovered_pair,
    )

    # See create_greenwave_node's docstring and GREENWAVE_DELAY_SEC above:
    # this MUST fire strictly after demo_timer's nodes are already
    # publishing, so it gets its own, later TimerAction rather than being
    # bundled into demo_timer.
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


class TestGraphProviderGreenwave(GatewayTestCase):
    """Graph provider metrics path, driven by a real greenwave_monitor.

    @verifies REQ_INTEROP_003
    """

    REQUIRED_FUNCTIONS = {FUNCTION_ID}
    REQUIRED_APPS = {
        PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID,
        UNCOVERED_SOURCE_ID, UNCOVERED_TARGET_ID,
    }
    MIN_EXPECTED_APPS = 4

    GRAPH_ENDPOINT = f'/functions/{FUNCTION_ID}/x-medkit-graph'

    @classmethod
    def setUpClass(cls):
        """Wait for gateway/discovery, then set up a live parameter client.

        The client (used only by test_03) targets the real temp_sensor
        node's ``set_parameters`` service to drop its publish_rate live,
        mid-suite - see the module docstring for why this beats a
        permanently-slow third node. A raw ``rcl_interfaces/srv/
        SetParameters`` client is used directly (rather than
        ``rclpy.parameter_client.AsyncParameterClient``, which is Jazzy+
        only) so this suite also loads and runs on Humble.
        """
        super().setUpClass()
        rclpy.init()
        cls._param_node = Node('graph_provider_greenwave_param_client')
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

    def _poll_until_primary_active(self, *, timeout=40.0):
        """Poll the function graph until the primary edge reports active.

        greenwave_monitor publishes ``/diagnostics`` at only ~1 Hz with the
        first useful reading a few seconds in, so this needs a much longer
        budget than the suite's usual 15s polls.

        Requires ``frequency_hz`` to already be non-null, not just
        ``metrics_status == 'active'``: the plugin marks a topic "active"
        as soon as ANY fresh /diagnostics sample was merged in, even one
        whose ``frame_rate_msg`` was exactly 0 (greenwave's own first one
        or two samples, before it has observed enough messages to compute
        a rate) - "active" tracks freshness, not field completeness, by
        design. Waiting for a populated frequency_hz skips past that
        transient and gives every caller (metrics-arrival, pending/healthy,
        and provenance assertions alike) a real measured rate to check.
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

    def test_01_metrics_arrive_edge_becomes_active(self):
        """Real /diagnostics from greenwave_monitor drive the edge active.

        @verifies REQ_INTEROP_003
        """
        graph = self._poll_until_primary_active()
        edge = self._find_edge(graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID)
        self.assertIsNotNone(edge, 'temp_sensor -> temp_monitor edge not in graph')

        metrics = edge['metrics']
        self.assertEqual(metrics['metrics_status'], 'active')
        self.assertIsNotNone(
            metrics['frequency_hz'], 'frequency_hz should not be null when active',
        )
        # temp_sensor publishes at 2 Hz; greenwave's own test suite allows up
        # to 60% relative tolerance on measured rate at low frequencies, so
        # use a generous band rather than an exact match.
        self.assertGreater(metrics['frequency_hz'], 0.5)
        self.assertLess(metrics['frequency_hz'], 5.0)
        self.assertEqual(graph['pipeline_status'], 'healthy')

    def test_02_uncovered_topic_stays_pending_graph_stays_healthy(self):
        """A topic greenwave never monitors stays pending; pipeline stays healthy.

        End-to-end form of the false-"broken" regression: an edge with no
        /diagnostics data must never demote the whole function's
        pipeline_status, even while a sibling edge in the same function is
        actively reporting metrics.

        @verifies REQ_INTEROP_003
        """
        graph = self._poll_until_primary_active()

        uncovered_edge = self._find_edge(graph, UNCOVERED_SOURCE_ID, UNCOVERED_TARGET_ID)
        self.assertIsNotNone(
            uncovered_edge, 'uncovered temp_sensor -> temp_monitor edge not in graph',
        )
        self.assertEqual(uncovered_edge['metrics']['metrics_status'], 'pending')
        self.assertIsNone(uncovered_edge['metrics']['frequency_hz'])
        self.assertEqual(graph['pipeline_status'], 'healthy')

    def test_03_degradation_is_real(self):
        """Dropping temp_sensor's rate well below expected degrades the pipeline.

        @verifies REQ_INTEROP_003
        """
        self.assertTrue(
            self._param_client.wait_for_service(timeout_sec=15.0),
            'temp_sensor parameter services not available',
        )
        rate_param = Parameter(
            'publish_rate', Parameter.Type.DOUBLE, DEGRADED_PUBLISH_RATE_HZ,
        )
        request = SetParameters.Request()
        request.parameters = [rate_param.to_parameter_msg()]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self._param_node, future, timeout_sec=10.0)
        result = future.result()
        self.assertIsNotNone(result, 'set_parameters call to temp_sensor timed out')
        self.assertTrue(
            result.results[0].successful,
            f'temp_sensor rejected publish_rate change: {result.results[0].reason}',
        )

        def _condition(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            edge = self._find_edge(graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID)
            if (edge and edge['metrics'].get('metrics_status') == 'active'
                    and graph.get('pipeline_status') == 'degraded'):
                return (graph, edge)
            return None

        # Generous budget: greenwave needs several new, real low-rate
        # samples (period ~3.3s at DEGRADED_PUBLISH_RATE_HZ) after the
        # parameter change above before its own rate estimate settles.
        graph, edge = self.poll_endpoint_until(
            self.GRAPH_ENDPOINT, _condition, timeout=90.0, interval=2.0,
        )
        self.assertLess(edge['metrics']['frequency_hz'], 1.0)
        self.assertIsNotNone(graph['bottleneck_edge'])
        bottleneck = next(
            e for e in graph['edges'] if e['edge_id'] == graph['bottleneck_edge']
        )
        self.assertEqual(bottleneck['source'], PRIMARY_SOURCE_ID)
        self.assertEqual(bottleneck['target'], PRIMARY_TARGET_ID)

    def test_04_provenance_is_greenwave_node(self):
        """metrics.source on the active edge is greenwave's actual FQN.

        Never a bare literal ("greenwave_monitor") and never a vendor
        literal ("nvidia") - only the resolved, fully-qualified node name.

        metrics.source resolves from a fresh
        ``get_publishers_info_by_topic("/diagnostics")`` query against the
        gateway's own graph cache (see resolve_publisher_source in
        graph_provider_plugin.cpp), which can lag the edge's first "active"
        sample by a discovery cycle - the edge going active only requires a
        message to have arrived, not that DDS discovery has yet propagated
        greenwave's publisher endpoint. So this polls past the first active
        sample specifically for ``source`` to appear, rather than asserting
        on whatever the first active poll happens to return.

        @verifies REQ_INTEROP_003
        """

        def _condition(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            edge = self._find_edge(graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID)
            if (edge and edge['metrics'].get('metrics_status') == 'active'
                    and edge['metrics'].get('frequency_hz') is not None
                    and edge['metrics'].get('source') is not None):
                return edge
            return None

        # Same generous budget as _poll_until_primary_active: source needs
        # everything that helper waits for, plus one more discovery cycle.
        edge = self.poll_endpoint_until(
            self.GRAPH_ENDPOINT, _condition, timeout=40.0, interval=1.0,
        )
        source = edge['metrics'].get('source')
        self.assertEqual(source, EXPECTED_GREENWAVE_SOURCE)
        self.assertNotEqual(source, GREENWAVE_NODE_NAME)
        self.assertNotEqual(source, 'nvidia')


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
