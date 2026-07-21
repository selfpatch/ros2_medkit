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

"""Integration tests for Function scoping in the graph provider plugin.

``resolve_scoped_apps`` (which Function apps are considered) and the
pub/sub cross product built on top of it (``build_graph_document_for_apps``
in ``graph_provider_plugin.cpp``) have no prior integration coverage: every
earlier suite exercises a single Function. This suite drives a manifest with
THREE functions over a disjoint set of demo pub/sub pairs and asserts:

1. No cross-Function contamination - a Function's document contains only
   its own apps/edges, never another Function's.
2. A Function with no hosted apps returns a valid 200 empty document
   (nodes/edges/topics all empty), never a 500.
3. System topics (``/parameter_events``, ``/rosout``, ``/diagnostics``)
   never appear as edges, even when a real publisher/subscriber pair shares
   one of those exact topic names inside a Function.
4. A topic where "nitros" is a substring of a longer path segment (e.g.
   ``/nitros_bridge/data``) SURVIVES the filter and produces an edge, while
   a topic where "nitros" is itself a whole path segment (e.g. a trailing
   ``/nitros``) is still correctly filtered. This is the integration-level
   guard for Task 5's segment-anchored filter fix
   (``has_path_segment``/``has_trailing_path_segment`` in
   ``graph_provider_plugin.cpp``), which Task 5 already covers at unit
   level; this suite proves the whole stack (real ROS graph topic names,
   real discovery) agrees.

The system-topic and nitros probe pairs are the SAME ``demo_engine_temp_sensor``
/ ``demo_engine_temp_monitor`` executables used everywhere else in this
package, just launched with a ``remappings=`` override on their relative
``temperature`` topic - no new demo-node fixture code, only launch
declarations, to keep this minimal per the task brief.
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

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.coverage import get_coverage_env
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


ENGINE_FUNCTION_ID = 'engine-monitoring'
FILTER_GUARD_FUNCTION_ID = 'filter-guard'
EMPTY_FUNCTION_ID = 'empty-function'

ENGINE_NAMESPACE = '/powertrain/engine'
ENGINE_SOURCE_ID = 'engine-temp-sensor'
ENGINE_TARGET_ID = 'engine-temp-monitor'

# System-topic filter-guard pair: shares the EXACT filtered literal
# "/diagnostics" as its topic. If the filter were ever removed or broken,
# this would show up as a real edge.
DIAG_NAMESPACE = '/filterguard/diag'
DIAG_SOURCE_ID = 'diag-sensor'
DIAG_TARGET_ID = 'diag-monitor'
DIAG_TOPIC = '/diagnostics'

# Negative nitros control: "nitros" as a whole trailing path segment - a
# real REP-2009 NITROS negotiation-shaped topic. Must stay filtered.
NITROS_NEG_NAMESPACE = '/filterguard/nitros_neg'
NITROS_NEG_SOURCE_ID = 'nitros-neg-sensor'
NITROS_NEG_TARGET_ID = 'nitros-neg-monitor'
NITROS_NEG_TOPIC = '/filterguard/probe/nitros'

# Positive nitros control: "nitros" is only a substring of the longer
# segment "nitros_bridge" - must survive filtering (Task 5's fix).
NITROS_POS_NAMESPACE = '/filterguard/nitros_pos'
NITROS_POS_SOURCE_ID = 'nitros-pos-sensor'
NITROS_POS_TARGET_ID = 'nitros-pos-monitor'
NITROS_POS_TOPIC = '/filterguard/probe/nitros_bridge/data'

SYSTEM_TOPIC_NAMES = {'/parameter_events', '/rosout', '/diagnostics'}

DEMO_DELAY_SEC = 2.0


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
        suffix='.yaml', prefix='test_graph_provider_scoping_manifest_',
    )
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


def _probe_pair(namespace, source_id, target_id, remap_topic):
    """Build a sensor/monitor pub-sub pair remapped onto one absolute topic.

    Both nodes use the SAME relative topic name ("temperature"); remapping
    it to the same absolute ``remap_topic`` on both ends makes them share
    that topic regardless of namespace, giving a real
    publisher/subscriber pair to test the topic filter against.
    """
    env = get_coverage_env()
    return [
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_engine_temp_sensor',
            name='sensor',
            namespace=namespace,
            output='screen',
            additional_env=env,
            remappings=[('temperature', remap_topic)],
        ),
        launch_ros.actions.Node(
            package='ros2_medkit_integration_tests',
            executable='demo_engine_temp_monitor',
            name='monitor',
            namespace=namespace,
            output='screen',
            additional_env=env,
            remappings=[('temperature', remap_topic)],
        ),
    ]


# Private manifest (not the shared demo_nodes_manifest.yaml): three
# Functions over disjoint app sets, purpose-built for scoping and
# topic-filter regression coverage.
MANIFEST_YAML = f"""\
manifest_version: "1.0"
metadata:
  name: "Graph Provider Scoping Test Vehicle"
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: test-area
    name: "Test Area"
components:
  - id: test-ecu
    name: "Test ECU"
    area: test-area
apps:
  - id: {ENGINE_SOURCE_ID}
    name: "Engine Temperature Sensor"
    is_located_on: test-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: {ENGINE_NAMESPACE}
  - id: {ENGINE_TARGET_ID}
    name: "Engine Temperature Monitor"
    is_located_on: test-ecu
    ros_binding:
      node_name: temp_monitor
      namespace: {ENGINE_NAMESPACE}
  - id: {DIAG_SOURCE_ID}
    name: "Diagnostics-topic Filter-guard Sensor"
    is_located_on: test-ecu
    ros_binding:
      node_name: sensor
      namespace: {DIAG_NAMESPACE}
  - id: {DIAG_TARGET_ID}
    name: "Diagnostics-topic Filter-guard Monitor"
    is_located_on: test-ecu
    ros_binding:
      node_name: monitor
      namespace: {DIAG_NAMESPACE}
  - id: {NITROS_NEG_SOURCE_ID}
    name: "Nitros-segment Negative-control Sensor"
    is_located_on: test-ecu
    ros_binding:
      node_name: sensor
      namespace: {NITROS_NEG_NAMESPACE}
  - id: {NITROS_NEG_TARGET_ID}
    name: "Nitros-segment Negative-control Monitor"
    is_located_on: test-ecu
    ros_binding:
      node_name: monitor
      namespace: {NITROS_NEG_NAMESPACE}
  - id: {NITROS_POS_SOURCE_ID}
    name: "Nitros-substring Positive-control Sensor"
    is_located_on: test-ecu
    ros_binding:
      node_name: sensor
      namespace: {NITROS_POS_NAMESPACE}
  - id: {NITROS_POS_TARGET_ID}
    name: "Nitros-substring Positive-control Monitor"
    is_located_on: test-ecu
    ros_binding:
      node_name: monitor
      namespace: {NITROS_POS_NAMESPACE}
functions:
  - id: {ENGINE_FUNCTION_ID}
    name: "Engine Monitoring"
    category: monitoring
    hosted_by:
      - {ENGINE_SOURCE_ID}
      - {ENGINE_TARGET_ID}
  - id: {FILTER_GUARD_FUNCTION_ID}
    name: "Filter Guard"
    category: monitoring
    hosted_by:
      - {DIAG_SOURCE_ID}
      - {DIAG_TARGET_ID}
      - {NITROS_NEG_SOURCE_ID}
      - {NITROS_NEG_TARGET_ID}
      - {NITROS_POS_SOURCE_ID}
      - {NITROS_POS_TARGET_ID}
  - id: {EMPTY_FUNCTION_ID}
    name: "Empty Function"
    category: monitoring
    hosted_by: []
"""


def generate_test_description():
    """Launch gateway + graph provider + three disjoint demo pub/sub pairs."""
    manifest_path = _write_manifest(MANIFEST_YAML)
    plugin_path = _get_graph_plugin_path()

    demo_actions = create_demo_nodes(['temp_sensor', 'temp_monitor'])
    demo_actions += _probe_pair(
        DIAG_NAMESPACE, DIAG_SOURCE_ID, DIAG_TARGET_ID, DIAG_TOPIC,
    )
    demo_actions += _probe_pair(
        NITROS_NEG_NAMESPACE, NITROS_NEG_SOURCE_ID, NITROS_NEG_TARGET_ID,
        NITROS_NEG_TOPIC,
    )
    demo_actions += _probe_pair(
        NITROS_POS_NAMESPACE, NITROS_POS_SOURCE_ID, NITROS_POS_TARGET_ID,
        NITROS_POS_TOPIC,
    )
    demo_timer = TimerAction(period=DEMO_DELAY_SEC, actions=demo_actions)

    gateway_node = create_gateway_node(
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'plugins': ['graph_provider'],
            'plugins.graph_provider.path': plugin_path,
        },
    )

    launch_description = LaunchDescription([
        gateway_node,
        demo_timer,
        launch_testing.actions.ReadyToTest(),
    ])

    return (
        launch_description,
        {'gateway_node': gateway_node},
    )


class TestGraphProviderScoping(GatewayTestCase):
    """Function scoping and topic-filter regression coverage.

    @verifies REQ_INTEROP_003
    """

    REQUIRED_FUNCTIONS = {
        ENGINE_FUNCTION_ID, FILTER_GUARD_FUNCTION_ID, EMPTY_FUNCTION_ID,
    }
    REQUIRED_APPS = {
        ENGINE_SOURCE_ID, ENGINE_TARGET_ID,
        DIAG_SOURCE_ID, DIAG_TARGET_ID,
        NITROS_NEG_SOURCE_ID, NITROS_NEG_TARGET_ID,
        NITROS_POS_SOURCE_ID, NITROS_POS_TARGET_ID,
    }
    MIN_EXPECTED_APPS = 8

    @staticmethod
    def _find_edge(graph, source_id, target_id):
        for edge in graph.get('edges', []):
            if edge.get('source') == source_id and edge.get('target') == target_id:
                return edge
        return None

    def _get_graph(self, function_id, *, timeout=30.0):
        data = self.poll_endpoint_until(
            f'/functions/{function_id}/x-medkit-graph',
            lambda d: d if 'x-medkit-graph' in d else None,
            timeout=timeout,
        )
        return data['x-medkit-graph']

    def _get_graph_with_edge(self, function_id, source_id, target_id, *, timeout=30.0):
        """Poll a Function's graph until a SPECIFIC edge has formed.

        Function/App presence is manifest-driven and appears immediately in
        hybrid mode, but an edge additionally needs runtime topic-linking
        (both ends' publisher/subscriber topics resolved from the live ROS
        graph), which lags behind. A bare "'x-medkit-graph' in d" wait
        succeeds on the very first response - before topics have linked -
        so callers that need a specific edge to exist (or need discovery to
        have settled before trusting an edge's ABSENCE) must poll on the
        edge itself, not just the wrapper key.
        """

        def _condition(data):
            graph = data.get('x-medkit-graph')
            if not graph:
                return None
            if self._find_edge(graph, source_id, target_id):
                return graph
            return None

        return self.poll_endpoint_until(
            f'/functions/{function_id}/x-medkit-graph', _condition,
            timeout=timeout, interval=0.5,
        )

    def test_01_function_scoping_no_cross_contamination(self):
        """Each Function's document contains only its own apps and edges.

        @verifies REQ_INTEROP_003
        """
        engine_graph = self._get_graph_with_edge(
            ENGINE_FUNCTION_ID, ENGINE_SOURCE_ID, ENGINE_TARGET_ID,
        )
        engine_node_ids = {n['entity_id'] for n in engine_graph['nodes']}
        self.assertEqual(
            engine_node_ids, {ENGINE_SOURCE_ID, ENGINE_TARGET_ID},
            f'engine-monitoring leaked foreign apps: {engine_node_ids}',
        )
        foreign_ids = (
            DIAG_SOURCE_ID, DIAG_TARGET_ID,
            NITROS_NEG_SOURCE_ID, NITROS_NEG_TARGET_ID,
            NITROS_POS_SOURCE_ID, NITROS_POS_TARGET_ID,
        )
        for foreign_id in foreign_ids:
            self.assertNotIn(
                foreign_id, engine_node_ids,
                f'{foreign_id} leaked into engine-monitoring nodes',
            )

        # Gate on the nitros-pos edge (a real, unfiltered edge) so discovery
        # has settled for filter-guard's app set before trusting the
        # engine-pair edge's ABSENCE below as a true negative rather than
        # "just not discovered yet".
        guard_graph = self._get_graph_with_edge(
            FILTER_GUARD_FUNCTION_ID, NITROS_POS_SOURCE_ID, NITROS_POS_TARGET_ID,
        )
        guard_node_ids = {n['entity_id'] for n in guard_graph['nodes']}
        self.assertNotIn(
            ENGINE_SOURCE_ID, guard_node_ids,
            'engine-temp-sensor leaked into filter-guard nodes',
        )
        self.assertNotIn(
            ENGINE_TARGET_ID, guard_node_ids,
            'engine-temp-monitor leaked into filter-guard nodes',
        )
        self.assertIsNone(
            self._find_edge(guard_graph, ENGINE_SOURCE_ID, ENGINE_TARGET_ID),
            'engine pair edge leaked into filter-guard document',
        )

    def test_02_empty_function_returns_valid_empty_document(self):
        """A Function with no hosted apps returns 200 with empty arrays, not 500.

        @verifies REQ_INTEROP_003
        """
        graph = self._get_graph(EMPTY_FUNCTION_ID)
        self.assertEqual(graph['nodes'], [])
        self.assertEqual(graph['edges'], [])
        self.assertEqual(graph['topics'], [])
        self.assertEqual(graph['pipeline_status'], 'healthy')
        self.assertIsNone(graph['bottleneck_edge'])

    def test_03_system_topics_never_appear_as_edges(self):
        """A real pub/sub pair sharing '/diagnostics' never produces an edge.

        @verifies REQ_INTEROP_003
        """
        # Gate on the nitros-pos edge so discovery has settled before
        # trusting the diag-pair edge's absence as a true negative.
        guard_graph = self._get_graph_with_edge(
            FILTER_GUARD_FUNCTION_ID, NITROS_POS_SOURCE_ID, NITROS_POS_TARGET_ID,
        )
        self.assertIsNone(
            self._find_edge(guard_graph, DIAG_SOURCE_ID, DIAG_TARGET_ID),
            'a /diagnostics pub/sub pair produced an edge - system-topic '
            'filter regression',
        )

        engine_graph = self._get_graph_with_edge(
            ENGINE_FUNCTION_ID, ENGINE_SOURCE_ID, ENGINE_TARGET_ID,
        )
        for graph in (guard_graph, engine_graph):
            topic_names = {t['name'] for t in graph['topics']}
            leaked = topic_names & SYSTEM_TOPIC_NAMES
            self.assertFalse(
                leaked,
                f'system topic(s) leaked into the topics array: {leaked}',
            )

    def test_04_nitros_segment_topic_survives_filtering(self):
        """A "nitros" substring survives; a whole "nitros" path segment is filtered.

        Task 5's segment-anchored fix (``has_path_segment`` /
        ``has_trailing_path_segment``) at the integration level: a plain
        substring search would have wrongly filtered
        '/filterguard/probe/nitros_bridge/data' too.

        @verifies REQ_INTEROP_003
        """
        guard_graph = self._get_graph_with_edge(
            FILTER_GUARD_FUNCTION_ID, NITROS_POS_SOURCE_ID, NITROS_POS_TARGET_ID,
        )

        pos_edge = self._find_edge(guard_graph, NITROS_POS_SOURCE_ID, NITROS_POS_TARGET_ID)
        self.assertIsNotNone(
            pos_edge,
            f'{NITROS_POS_TOPIC} was wrongly filtered - "nitros" is only a '
            'substring of "nitros_bridge", not a whole path segment',
        )

        neg_edge = self._find_edge(guard_graph, NITROS_NEG_SOURCE_ID, NITROS_NEG_TARGET_ID)
        self.assertIsNone(
            neg_edge,
            f'{NITROS_NEG_TOPIC} survived filtering - "nitros" is a whole '
            'trailing path segment there and must be filtered',
        )

        topic_names = {t['name'] for t in guard_graph['topics']}
        self.assertIn(NITROS_POS_TOPIC, topic_names)
        self.assertNotIn(NITROS_NEG_TOPIC, topic_names)


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
