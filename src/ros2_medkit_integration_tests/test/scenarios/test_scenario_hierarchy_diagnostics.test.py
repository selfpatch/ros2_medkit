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

"""Scenario: Fleet diagnostics with hierarchy-scoped triggers.

A fleet monitoring dashboard sets up triggers at different levels of the
entity hierarchy to provide layered diagnostic visibility:

1. Area-level fault trigger (OnChange on area's faults) - catches any
   fault from any component/app within the area
2. App-level data trigger (LeaveRange on specific app's data) - monitors
   a specific sensor value for out-of-range conditions

Uses manifest_only discovery mode with demo_nodes_manifest.yaml to get
proper area -> component -> app hierarchy.

This scenario validates that triggers can be created on different entity
types/IDs and that they are truly independent (different entity scopes,
different collections, different conditions). SSE connectivity is
verified for both.

"""

import os
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
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
)


def generate_test_description():
    """Launch gateway in manifest mode + demo nodes for hierarchy testing."""
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    gateway = create_gateway_node(extra_params={
        'discovery.mode': 'manifest_only',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
    })

    # Launch temp_sensor and rpm_sensor for data topics
    demo_nodes = create_demo_nodes(
        ['temp_sensor', 'rpm_sensor'],
        lidar_faulty=False,
    )

    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


class TestScenarioHierarchyDiagnostics(GatewayTestCase):
    """Scenario: Fleet diagnostics with hierarchy-scoped triggers.

    A fleet monitoring dashboard sets up triggers at different levels
    of the entity hierarchy for layered diagnostic visibility.

    Steps:
    1. Create area-level fault trigger (OnChange on area's faults)
    2. Create app-level data trigger (LeaveRange on app's data)
    3. Both triggers created successfully
    4. Triggers are on different entity types/IDs
    5. SSE connectivity verified for both
    6. Clean up
    """

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'engine-temp-sensor', 'engine-rpm-sensor'}
    REQUIRED_AREAS = {'powertrain', 'engine'}

    app_topic_id = None
    app_resource_uri = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway, discover entities, find data topic."""
        super().setUpClass()

        # Wait for data availability on engine-temp-sensor
        system_topics = {'/parameter_events', '/rosout'}
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                r = requests.get(
                    f'{cls.BASE_URL}/apps/engine-temp-sensor/data',
                    timeout=5,
                )
                if r.status_code == 200:
                    items = r.json().get('items', [])
                    for item in items:
                        if item['id'] not in system_topics:
                            cls.app_topic_id = item['id']
                            cls.app_resource_uri = (
                                f'/api/v1/apps/engine-temp-sensor/data'
                                f'{cls.app_topic_id}'
                            )
                            break
                    if cls.app_topic_id:
                        break
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)

        if not cls.app_topic_id:
            raise AssertionError(
                'No data items available for app engine-temp-sensor'
            )

    def _delete_trigger(self, entity_type, entity_id, trigger_id):
        """Delete a trigger, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/{entity_type}/{entity_id}/triggers/{trigger_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    # ===================================================================
    # Scenario: Hierarchy-scoped diagnostics
    # ===================================================================

    def test_01_create_area_level_fault_trigger(self):
        """Create trigger on area's faults collection.

        An area-level fault trigger catches faults from all components
        and apps within the area. The 'engine' area contains
        temp-sensor-hw and rpm-sensor-hw components.
        """
        resource = '/api/v1/areas/engine/faults'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 120,
        }
        r = requests.post(
            f'{self.BASE_URL}/areas/engine/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'areas', 'engine', trig['id']
        )

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], resource)
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'OnChange',
        )
        self.assertIn('event_source', trig)
        self.assertTrue(trig['event_source'].endswith('/events'))

    def test_02_create_app_level_data_trigger(self):
        """Create LeaveRange trigger on app's specific data topic.

        Monitors the engine-temp-sensor's temperature data for values
        leaving the normal range [15.0, 90.0].
        """
        body = {
            'resource': self.app_resource_uri,
            'trigger_condition': {
                'condition_type': 'LeaveRange',
                'lower_bound': 15.0,
                'upper_bound': 90.0,
            },
            'multishot': True,
            'lifetime': 120,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'apps', 'engine-temp-sensor', trig['id']
        )

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], self.app_resource_uri)
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'LeaveRange',
        )
        self.assertEqual(
            trig['trigger_condition']['lower_bound'], 15.0,
        )
        self.assertEqual(
            trig['trigger_condition']['upper_bound'], 90.0,
        )

    def test_03_triggers_on_different_entity_types(self):
        """Both triggers are on different entity types and IDs."""
        # Create area trigger
        area_body = {
            'resource': '/api/v1/areas/engine/faults',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/areas/engine/triggers',
            json=area_body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        area_trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'areas', 'engine', area_trig['id']
        )

        # Create app trigger
        app_body = {
            'resource': self.app_resource_uri,
            'trigger_condition': {
                'condition_type': 'LeaveRange',
                'lower_bound': 15.0,
                'upper_bound': 90.0,
            },
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers',
            json=app_body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        app_trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'apps', 'engine-temp-sensor', app_trig['id']
        )

        # Different IDs
        self.assertNotEqual(area_trig['id'], app_trig['id'])

        # Different observed_resources
        self.assertNotEqual(
            area_trig['observed_resource'],
            app_trig['observed_resource'],
        )

        # Area trigger is on faults, app trigger is on data
        self.assertIn('faults', area_trig['observed_resource'])
        self.assertIn('data', app_trig['observed_resource'])

        # They are listed under their respective entities, not cross-listed
        r = requests.get(
            f'{self.BASE_URL}/areas/engine/triggers', timeout=5
        )
        self.assertEqual(r.status_code, 200)
        area_ids = {item['id'] for item in r.json().get('items', [])}
        self.assertIn(area_trig['id'], area_ids)
        self.assertNotIn(app_trig['id'], area_ids)

        r = requests.get(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers', timeout=5
        )
        self.assertEqual(r.status_code, 200)
        app_ids = {item['id'] for item in r.json().get('items', [])}
        self.assertIn(app_trig['id'], app_ids)
        self.assertNotIn(area_trig['id'], app_ids)

    def test_04_sse_connectivity_area_trigger(self):
        """SSE connection works on the area-level fault trigger."""
        body = {
            'resource': '/api/v1/areas/engine/faults',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/areas/engine/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'areas', 'engine', trig['id']
        )

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        stream_connected = threading.Event()

        def read_stream():
            try:
                with requests.get(events_url, stream=True, timeout=5) as resp:
                    stream_connected.set()
                    for _line in resp.iter_lines(decode_unicode=True):
                        break
            except requests.exceptions.RequestException:
                pass
            finally:
                stream_connected.set()

        thread = threading.Thread(target=read_stream, daemon=True)
        thread.start()

        self.assertTrue(
            stream_connected.wait(timeout=5),
            'SSE stream for area trigger failed to connect',
        )
        thread.join(timeout=5)

    def test_05_sse_connectivity_app_trigger(self):
        """SSE connection works on the app-level data trigger."""
        body = {
            'resource': self.app_resource_uri,
            'trigger_condition': {
                'condition_type': 'LeaveRange',
                'lower_bound': 15.0,
                'upper_bound': 90.0,
            },
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'apps', 'engine-temp-sensor', trig['id']
        )

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        stream_connected = threading.Event()

        def read_stream():
            try:
                with requests.get(events_url, stream=True, timeout=5) as resp:
                    stream_connected.set()
                    for _line in resp.iter_lines(decode_unicode=True):
                        break
            except requests.exceptions.RequestException:
                pass
            finally:
                stream_connected.set()

        thread = threading.Thread(target=read_stream, daemon=True)
        thread.start()

        self.assertTrue(
            stream_connected.wait(timeout=5),
            'SSE stream for app data trigger failed to connect',
        )
        thread.join(timeout=5)

    def test_06_cleanup_both_triggers(self):
        """Delete both triggers and verify they are gone."""
        # Create area trigger
        r = requests.post(
            f'{self.BASE_URL}/areas/engine/triggers',
            json={
                'resource': '/api/v1/areas/engine/faults',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 60,
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        area_trig_id = r.json()['id']

        # Create app trigger
        r = requests.post(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers',
            json={
                'resource': self.app_resource_uri,
                'trigger_condition': {
                    'condition_type': 'LeaveRange',
                    'lower_bound': 15.0,
                    'upper_bound': 90.0,
                },
                'multishot': True,
                'lifetime': 60,
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        app_trig_id = r.json()['id']

        # Delete both
        r = requests.delete(
            f'{self.BASE_URL}/areas/engine/triggers/{area_trig_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        r = requests.delete(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers/{app_trig_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify both are gone
        r = requests.get(
            f'{self.BASE_URL}/areas/engine/triggers/{area_trig_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

        r = requests.get(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers/{app_trig_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
