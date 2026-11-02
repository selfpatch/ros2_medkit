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
Integration tests for hybrid mode gap-fill configuration.

Tests that gap-fill controls restrict which heuristic entities the
runtime layer can create when a manifest is present.

Note: Areas and Components are never created by runtime discovery.
Gap-fill only controls heuristic Apps and Functions.

A second gateway on the same graph runs an inline manifest built to make the
merge-by-id visible: two of its apps are named after running nodes but bind to
a namespace where nothing runs. Runtime discovery names each node's app after
the node, so the merge joins those runtime apps into the manifest apps by id.
Only ``ros_binding`` may tie a manifest app to a node, so those apps must come
out unbound and without the nodes' topics or services.
"""

import os
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions

import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    DISCOVERY_TIMEOUT,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node

# --------------------------------------------------------------------------
# Second gateway: manifest apps named after running nodes they do not bind to
# --------------------------------------------------------------------------

BINDING_PORT = get_test_port(1)
BINDING_URL = f'http://localhost:{BINDING_PORT}{API_BASE_PATH}'

# Control: bound to a node that runs where the binding says, so its link is
# the proof that this gateway has run discovery against the live graph.
LINKED_APP_ID = 'gf-rpm'
LINKED_FQN = '/powertrain/engine/rpm_sensor'

# Named after a running node, bound to a namespace where nothing runs. The
# node /powertrain/engine/temp_sensor publishes the temperature topic and
# /powertrain/engine/calibration serves the calibrate service.
DECLARED_NAMESPACE = '/declared/elsewhere'
TOPIC_APP_ID = 'temp_sensor'
TOPIC_NODE_FQN = '/powertrain/engine/temp_sensor'
FOREIGN_TOPIC = '/powertrain/engine/temperature'
SERVICE_APP_ID = 'calibration'
SERVICE_NODE_FQN = '/powertrain/engine/calibration'
FOREIGN_SERVICE = '/powertrain/engine/calibrate'

WARN_UNMANIFESTED_NODES = 'unmanifested_nodes'

# Wall-clock budgets scale with MEDKIT_TEST_TIME_SCALE; sampling intervals do
# not. DISCOVERY_TIMEOUT already carries the scale.
HTTP_TIMEOUT_SEC = 5.0 * get_time_scale()
POLL_INTERVAL_SEC = 0.5
# Spaced wider than the 1000 ms refresh so every sample is a different pass.
SAMPLE_INTERVAL_SEC = 1.25
SAMPLE_COUNT = 3

BINDING_MANIFEST = f"""\
manifest_version: "1.0"
metadata:
  name: "Gap-fill binding test vehicle"
  version: "1.0.0"
config:
  unmanifested_nodes: "error"
areas:
  - id: gf-area
    name: "Gap-fill Binding Area"
components:
  - id: gf-ecu
    name: "Gap-fill Binding ECU"
    area: gf-area
apps:
  - id: {LINKED_APP_ID}
    name: "Engine RPM Sensor"
    is_located_on: gf-ecu
    ros_binding:
      node_name: rpm_sensor
      namespace: /powertrain/engine
  - id: {TOPIC_APP_ID}
    name: "Temperature Sensor (declared elsewhere)"
    is_located_on: gf-ecu
    ros_binding:
      node_name: temp_sensor
      namespace: {DECLARED_NAMESPACE}
  - id: {SERVICE_APP_ID}
    name: "Calibration Service (declared elsewhere)"
    is_located_on: gf-ecu
    ros_binding:
      node_name: calibration
      namespace: {DECLARED_NAMESPACE}
"""

_MANIFEST_PATHS = []


def _write_manifest(content):
    """Write manifest YAML to a temporary file, return its path."""
    fd, path = tempfile.mkstemp(suffix='.yaml', prefix='test_discovery_gap_fill_binding_')
    with os.fdopen(fd, 'w') as handle:
        handle.write(content)
    _MANIFEST_PATHS.append(path)
    return path


def _remove_manifests():
    """Delete every manifest this module wrote."""
    while _MANIFEST_PATHS:
        try:
            os.unlink(_MANIFEST_PATHS.pop())
        except OSError:
            pass


def _get_json(path):
    response = requests.get(f'{BINDING_URL}{path}', timeout=HTTP_TIMEOUT_SEC)
    response.raise_for_status()
    return response.json()


def generate_test_description():
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    # Hybrid mode with gap-fill:
    # - Apps allowed (for linking)
    # - Areas and Components come from manifest only (runtime never creates them)
    gateway_node = create_gateway_node(extra_params={
        'discovery.mode': 'hybrid',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
        'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': True,
    })

    binding_gateway = create_gateway_node(
        port=BINDING_PORT,
        name='gw_gap_fill_binding',
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': _write_manifest(BINDING_MANIFEST),
            'discovery.merge_pipeline.gap_fill.allow_heuristic_apps': True,
        },
    )

    # Launch a subset of demo nodes (some in manifest, some not)
    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'calibration'],
    )
    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway_node,
            binding_gateway,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


# @verifies REQ_INTEROP_003
class TestGapFillConfig(GatewayTestCase):
    """Test gap-fill restrictions in hybrid mode."""

    def test_only_manifest_areas_present(self):
        """Areas come from manifest only - no heuristic areas from runtime.

        GET /areas only returns top-level areas; subareas are filtered
        and accessible via GET /areas/{id}/subareas.
        """
        data = self.poll_endpoint_until(
            '/areas',
            lambda d: d if len(d.get('items', [])) >= 1 else None,
            timeout=30.0,
        )
        area_ids = [a['id'] for a in data['items']]

        # Only top-level manifest areas should appear in GET /areas
        # Subareas (engine, brakes, lidar, door, lights, front-left-door)
        # are filtered from the top-level listing
        for area_id in area_ids:
            self.assertIn(area_id, [
                'powertrain', 'chassis', 'body', 'perception',
                # HATEOAS edge-case fixture, manifest-only.
                'hateoas-edge-area',
            ], f'Unexpected area found in top-level listing: {area_id}')

    def test_only_manifest_components_present(self):
        """Components come from manifest only - runtime never creates components."""
        data = self.poll_endpoint_until(
            '/components',
            lambda d: d if len(d.get('items', [])) >= 1 else None,
            timeout=30.0,
        )
        component_ids = [c['id'] for c in data['items']]

        # Only manifest-defined components should be present
        manifest_components = [
            'engine-ecu', 'temp-sensor-hw', 'rpm-sensor-hw',
            'brake-ecu', 'brake-pressure-sensor-hw', 'brake-actuator-hw',
            'door-sensor-hw', 'light-module',
            'lidar-unit',
            # HATEOAS edge-case fixture (component without area assignment).
            'hateoas-component-no-area',
        ]
        for comp_id in component_ids:
            self.assertIn(
                comp_id, manifest_components,
                f'Unexpected heuristic component found: {comp_id}',
            )

    def test_manifest_apps_present_and_linked(self):
        """With allow_heuristic_apps=true, manifest apps matching launched nodes are present."""
        # 4 demo nodes launched: temp_sensor, rpm_sensor, pressure_sensor, calibration
        # These match manifest apps: engine-temp-sensor, engine-rpm-sensor,
        # brake-pressure-sensor, engine-calibration-service
        expected_linked = {
            'engine-temp-sensor', 'engine-rpm-sensor',
            'brake-pressure-sensor', 'engine-calibration-service',
        }
        data = self.poll_endpoint_until(
            '/apps',
            lambda d: d if len(d.get('items', [])) >= len(expected_linked) else None,
            timeout=30.0,
        )
        app_ids = {a['id'] for a in data['items']}
        for app_id in expected_linked:
            self.assertIn(
                app_id, app_ids,
                f'Expected manifest app {app_id} not found in apps list',
            )

    def test_health_shows_discovery_info(self):
        """Health endpoint should show discovery information."""
        health = self.poll_endpoint_until(
            '/health',
            lambda data: data if 'discovery' in data else None,
            timeout=30.0,
        )
        discovery = health.get('discovery', {})
        pipeline = discovery.get('pipeline', {})

        # Pipeline info should be present in hybrid mode
        self.assertIn('total_entities', pipeline)


# @verifies REQ_INTEROP_003
class TestManifestAppBindsByRosBindingOnly(GatewayTestCase):
    """A manifest app is tied to a node by its ros_binding, never by its id."""

    BASE_URL = BINDING_URL
    REQUIRED_APPS = {LINKED_APP_ID}

    @classmethod
    def setUpClass(cls):
        """Wait until this gateway has seen the nodes the absorbed apps are named after."""
        super().setUpClass()
        deadline = time.monotonic() + DISCOVERY_TIMEOUT
        last = None
        while time.monotonic() < deadline:
            try:
                last = cls._graph_seen()
                if last is True:
                    break
            except requests.exceptions.RequestException as exc:
                last = exc
            time.sleep(POLL_INTERVAL_SEC)
        else:
            raise AssertionError(
                f'{BINDING_URL} did not see the live graph within '
                f'{DISCOVERY_TIMEOUT}s; last={last!r}')
        # Several samples, each on a later discovery pass than the barrier, so a
        # single pass that predates the nodes cannot pass for a fixed gateway.
        cls.samples = []
        for _ in range(SAMPLE_COUNT):
            time.sleep(SAMPLE_INTERVAL_SEC)
            cls.samples.append({
                'topic_app': _get_json(f'/apps/{TOPIC_APP_ID}'),
                'topic_app_data': _get_json(f'/apps/{TOPIC_APP_ID}/data'),
                'service_app': _get_json(f'/apps/{SERVICE_APP_ID}'),
                'service_app_operations': _get_json(f'/apps/{SERVICE_APP_ID}/operations'),
            })

    @classmethod
    def _graph_seen(cls):
        """Return True once the control app is linked and both foreign nodes are reported.

        ``unmanifested_nodes: error`` lists every running node no manifest app
        binds to in ``/health``. Both nodes the absorbed apps are named after
        must be listed there: the gateway has seen them, and has not bound
        them to any app.
        """
        x_medkit = _get_json(f'/apps/{LINKED_APP_ID}').get('x-medkit', {})
        linked = x_medkit.get('ros2', {}).get('node')
        if linked != LINKED_FQN:
            return f'{LINKED_APP_ID} bound to {linked!r}'
        warnings = _get_json('/health').get('warnings', [])
        orphans = set()
        for warning in warnings:
            if warning.get('code') == WARN_UNMANIFESTED_NODES:
                orphans.update(warning.get('ros_node_fqns', []))
        missing = {TOPIC_NODE_FQN, SERVICE_NODE_FQN} - orphans
        return True if not missing else f'not reported as unmanifested yet: {sorted(missing)}'

    def test_app_named_after_a_node_elsewhere_is_not_bound_to_it(self):
        """Neither app names the node its id matches, and neither is online."""
        for i, sample in enumerate(self.samples):
            for key, app_id in (('topic_app', TOPIC_APP_ID), ('service_app', SERVICE_APP_ID)):
                x_medkit = sample[key].get('x-medkit', {})
                self.assertEqual(x_medkit.get('source'), 'manifest')
                self.assertNotIn(
                    'node', x_medkit.get('ros2', {}),
                    f'sample {i}: {app_id} names a node its ros_binding does not match: '
                    f'{x_medkit.get("ros2")}')
                self.assertFalse(
                    x_medkit.get('is_online'),
                    f'sample {i}: {app_id} reports online, '
                    f'but no node runs in {DECLARED_NAMESPACE}')

    def test_app_named_after_a_node_elsewhere_serves_none_of_its_data(self):
        """The temperature topic belongs to /powertrain/engine/temp_sensor, not to the app."""
        # @verifies REQ_INTEROP_018
        for i, sample in enumerate(self.samples):
            items = sample['topic_app_data'].get('items', [])
            ids = {item.get('id') for item in items}
            self.assertNotIn(
                FOREIGN_TOPIC, ids, f'sample {i}: foreign topic served by {TOPIC_APP_ID}')
            self.assertEqual(items, [], f'sample {i}: {TOPIC_APP_ID} declares no data')

    def test_app_named_after_a_node_elsewhere_serves_none_of_its_operations(self):
        """The calibrate service belongs to /powertrain/engine/calibration, not to the app."""
        # @verifies REQ_INTEROP_033
        for i, sample in enumerate(self.samples):
            items = sample['service_app_operations'].get('items', [])
            services = {item.get('x-medkit', {}).get('ros2', {}).get('service') for item in items}
            self.assertNotIn(
                FOREIGN_SERVICE, services,
                f'sample {i}: foreign service offered by {SERVICE_APP_ID}')
            self.assertEqual(items, [], f'sample {i}: {SERVICE_APP_ID} declares no operations')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )

    def test_remove_manifests(self):
        """Remove the inline manifest the second gateway read."""
        _remove_manifests()
