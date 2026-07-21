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

"""Integration tests for the graph provider's cyclic-subscription (SSE) sampler.

The graph provider plugin registers a resource sampler for collection
``x-medkit-graph`` (``register_sampler("x-medkit-graph", ...)`` in
``graph_provider_plugin.cpp``), which lets a client open a cyclic
subscription on ``GET /functions/{id}/x-medkit-graph`` and receive periodic
Server-Sent-Events frames instead of polling. This is a shipped feature with
no prior integration coverage.

Reuses the same real-``greenwave_monitor`` harness as
``test_graph_provider_greenwave.test.py`` (Task 8) and
``test_graph_provider_stale.test.py``, and the SSE stream-consumption idiom
from ``test_sse.test.py`` / ``test_multi_collection_subscriptions.test.py``
(POST to create, GET the ``event_source`` with ``stream=True``, parse
``data: `` lines as JSON).

Per the module docstring's flakiness warning: this suite does NOT assert
that a float metric value changed between two SSE frames (greenwave's ~1 Hz
cadence and the document's per-request ``timestamp`` field would make that
assertion pass or fail for the wrong reason). Instead it asserts the
``metrics_status`` field's ``pending`` -> ``active`` transition, which is a
one-way, stable liveness proof: the edge starts ``pending`` (greenwave has
not started yet - see ``GREENWAVE_DELAY_SEC``) and is only ever able to
become ``active`` once real ``/diagnostics`` data has arrived.
"""

import json
import os
import tempfile
import threading
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
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

# Aligns the graph provider's expected frequency with temp_sensor's own 2 Hz
# default - see test_graph_provider_greenwave.test.py's module docstring for
# the full rationale. Not load-bearing for this suite's assertions (which
# only look at metrics_status, not pipeline_status), kept for consistency
# with the shared harness and to avoid a spurious "degraded" read.
FUNCTION_ID_OVERRIDE_KEY = (
    f'plugins.graph_provider.function_overrides.{FUNCTION_ID}.expected_frequency_hz'
)
EXPECTED_FREQUENCY_HZ = 2.0

DEMO_DELAY_SEC = 2.0
# greenwave_monitor resolves its monitored topics ONCE at startup - see
# create_greenwave_node's docstring - so it must start strictly after the
# demo pair is already publishing. Because this suite's whole point is to
# observe the pending -> active transition, it deliberately opens the SSE
# subscription BEFORE this timer fires.
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
        suffix='.yaml', prefix='test_graph_provider_sse_manifest_',
    )
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    return path


# Private manifest (not the shared demo_nodes_manifest.yaml): a single
# temp_sensor -> temp_monitor pair, the minimum needed for one edge.
MANIFEST_YAML = f"""\
manifest_version: "1.0"
metadata:
  name: "Graph Provider SSE Test Vehicle"
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

    greenwave_timer = TimerAction(
        period=GREENWAVE_DELAY_SEC,
        actions=[create_greenwave_node(
            monitored_topics=[PRIMARY_TOPIC],
            name=GREENWAVE_NODE_NAME,
        )],
    )

    gateway_node = create_gateway_node(
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'plugins': ['graph_provider'],
            'plugins.graph_provider.path': plugin_path,
            FUNCTION_ID_OVERRIDE_KEY: EXPECTED_FREQUENCY_HZ,
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


class TestGraphProviderSse(GatewayTestCase):
    """SSE cyclic-subscription coverage for the x-medkit-graph resource.

    @verifies REQ_INTEROP_003
    """

    REQUIRED_FUNCTIONS = {FUNCTION_ID}
    REQUIRED_APPS = {PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID}
    MIN_EXPECTED_APPS = 2

    GRAPH_RESOURCE = f'/api/v1/functions/{FUNCTION_ID}/x-medkit-graph'

    @staticmethod
    def _find_edge(graph, source_id, target_id):
        for edge in graph.get('edges', []):
            if edge.get('source') == source_id and edge.get('target') == target_id:
                return edge
        return None

    def _create_subscription(self, *, interval='normal', duration=90):
        r = requests.post(
            f'{self.BASE_URL}/functions/{FUNCTION_ID}/cyclic-subscriptions',
            json={
                'resource': self.GRAPH_RESOURCE,
                'interval': interval,
                'duration': duration,
            },
            timeout=5,
        )
        return r

    def _delete_subscription(self, sub_id):
        try:
            requests.delete(
                f'{self.BASE_URL}/functions/{FUNCTION_ID}'
                f'/cyclic-subscriptions/{sub_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    def test_01_sse_stream_carries_graph_document_and_pending_becomes_active(self):
        """Create a cyclic subscription and observe the edge go pending -> active.

        The subscription is opened immediately (well before
        ``GREENWAVE_DELAY_SEC`` elapses), so the very first frames carrying
        this edge must show it ``pending``; only once greenwave_monitor has
        started and warmed up should later frames show it ``active``. Both
        observations coming from the SAME SSE stream is the point of this
        test - it proves the sampler serves live, not frozen, data.

        @verifies REQ_INTEROP_003
        """
        r = self._create_subscription(interval='normal', duration=90)
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        sub_id = data['id']
        self.addCleanup(self._delete_subscription, sub_id)

        self.assertTrue(sub_id.startswith('sub_'))
        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'normal')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)
        self.assertTrue(data['event_source'].endswith('/events'))

        events_url = (
            f'{self.BASE_URL}{data["event_source"].removeprefix(API_BASE_PATH)}'
        )

        seen_pending = threading.Event()
        seen_active = threading.Event()
        stop_event = threading.Event()
        errors = []

        def collect_events():
            try:
                with requests.get(events_url, stream=True, timeout=60) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop_event.is_set():
                            break
                        if not line or not line.startswith('data: '):
                            continue
                        try:
                            envelope = json.loads(line[len('data: '):])
                        except json.JSONDecodeError:
                            continue
                        payload = envelope.get('payload')
                        if not payload or 'x-medkit-graph' not in payload:
                            continue
                        graph = payload['x-medkit-graph']
                        edge = self._find_edge(
                            graph, PRIMARY_SOURCE_ID, PRIMARY_TARGET_ID,
                        )
                        if not edge:
                            continue
                        status = edge['metrics'].get('metrics_status')
                        if status == 'pending':
                            seen_pending.set()
                        elif status == 'active':
                            seen_active.set()
                        if seen_pending.is_set() and seen_active.is_set():
                            stop_event.set()
                            break
            except requests.exceptions.RequestException as exc:
                errors.append(str(exc))
            finally:
                stop_event.set()

        thread = threading.Thread(target=collect_events, daemon=True)
        thread.start()

        # Must clear GREENWAVE_DELAY_SEC (10s) plus warmup for the first
        # useful greenwave sample, so budget generously past that.
        transitioned = stop_event.wait(timeout=45.0)
        thread.join(timeout=5)

        self.assertEqual(errors, [], f'SSE stream error(s): {errors}')
        self.assertTrue(
            seen_pending.is_set(),
            'Never observed the primary edge as pending - expected before '
            'greenwave_monitor starts',
        )
        self.assertTrue(
            seen_active.is_set(),
            'Never observed the primary edge become active - greenwave_monitor '
            'data never arrived via the SSE stream within budget',
        )
        self.assertTrue(transitioned, 'Timed out waiting for pending -> active')

    def test_02_delete_subscription_stops_stream_and_removes_it(self):
        """Deleting a subscription mid-stream closes the SSE connection.

        Uses a fresh, short-lived subscription (no need to wait for the
        greenwave warmup here) - proves the CRUD lifecycle of the
        x-medkit-graph cyclic subscription independent of case 1's data
        assertions.

        @verifies REQ_INTEROP_003
        """
        r = self._create_subscription(interval='fast', duration=30)
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        data = r.json()
        sub_id = data['id']
        events_url = (
            f'{self.BASE_URL}{data["event_source"].removeprefix(API_BASE_PATH)}'
        )

        connected = threading.Event()
        stream_ended = threading.Event()
        errors = []

        def collect_events():
            try:
                with requests.get(events_url, stream=True, timeout=20) as resp:
                    connected.set()
                    for _line in resp.iter_lines(decode_unicode=True):
                        pass  # Drain until the server closes the connection.
            except requests.exceptions.RequestException as exc:
                errors.append(str(exc))
            finally:
                connected.set()
                stream_ended.set()

        thread = threading.Thread(target=collect_events, daemon=True)
        thread.start()
        self.assertTrue(connected.wait(timeout=10), 'SSE stream never connected')

        # Give the stream a brief moment to have emitted at least one frame
        # before we delete out from under it.
        stream_ended.wait(timeout=1.0)

        del_response = requests.delete(
            f'{self.BASE_URL}/functions/{FUNCTION_ID}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(del_response.status_code, 204)

        # The stream must stop: the server-side next_event loop checks
        # sub_mgr_.is_active() and returns false once deleted, closing the
        # connection from its end.
        thread.join(timeout=10)
        self.assertTrue(
            stream_ended.is_set(),
            'SSE stream did not stop after its subscription was deleted',
        )

        # The subscription itself is gone.
        get_response = requests.get(
            f'{self.BASE_URL}/functions/{FUNCTION_ID}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(get_response.status_code, 404)


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
