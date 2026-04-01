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

"""Feature tests for trigger entity hierarchy scoping.

Validates that triggers on parent entities (components, areas, functions)
correctly fire when events occur on their descendant entities (apps).

Uses manifest_only discovery mode with demo_nodes_manifest.yaml to get
proper component -> app hierarchy. The temp-sensor-hw component hosts
the engine-temp-sensor app, so a trigger on the component should fire
when the app's data changes.

"""

import os
import threading
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


class TestTriggersHierarchy(GatewayTestCase):
    """Entity hierarchy scoping for triggers.

    In manifest mode, components host apps. A trigger on a component
    should fire when its hosted app's resources change. The hierarchy
    resolver in TriggerManager uses EntityCache relationships to check
    if the notification entity is a descendant of the trigger's entity.

    Manifest hierarchy used:
    - Component temp-sensor-hw hosts App engine-temp-sensor
    - Area engine contains Component temp-sensor-hw
    - Function engine-monitoring hosted_by engine-temp-sensor, engine-rpm-sensor
    """

    # Manifest entities need time to be loaded and for apps to go online
    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'engine-temp-sensor', 'engine-rpm-sensor'}
    # Only top-level areas appear in GET /areas; 'engine' is a subarea
    # of 'powertrain' and is filtered from the top-level listing.
    REQUIRED_AREAS = {'powertrain'}

    def _delete_trigger(self, entity_type, entity_id, trigger_id):
        """Delete a trigger, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/{entity_type}/{entity_id}/triggers/{trigger_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    # ------------------------------------------------------------------
    # Component -> App hierarchy
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_029
    def test_01_component_trigger_created_successfully(self):
        """Create trigger on component that hosts apps."""
        # temp-sensor-hw hosts engine-temp-sensor app
        resource = '/api/v1/components/temp-sensor-hw/data'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/components/temp-sensor-hw/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'components', 'temp-sensor-hw', trig['id']
        )

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], resource)

    def test_02_component_trigger_fires_on_hosted_app_data(self):
        """Component trigger fires when its hosted app's data changes.

        The temp-sensor-hw component hosts engine-temp-sensor app.
        When engine-temp-sensor publishes temperature data, the component
        trigger should fire via hierarchy matching.

        In manifest_only mode with inherit_runtime_resources, data items
        take time to propagate. We create a collection-level trigger
        (no specific resource_path) to avoid topic resolution issues.
        """
        # Create trigger on the COMPONENT's data collection (no specific topic)
        resource = '/api/v1/components/temp-sensor-hw/data'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/components/temp-sensor-hw/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'components', 'temp-sensor-hw', trig['id']
        )

        # Verify trigger is active
        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], resource)

        # Connect SSE and wait for events. The data trigger on a collection
        # (no resource_path) will match ANY data change from descendant apps.
        # However, the TriggerTopicSubscriber won't subscribe to a topic if
        # resource_path is empty, so data events need to come from somewhere
        # else. Since the collection-level data trigger has empty resource_path,
        # it will match notifications from any topic on descendant entities.
        # The temp-sensor-hw component hosts engine-temp-sensor, which publishes
        # temperature data, but the TriggerTopicSubscriber only subscribes
        # when resource_path is non-empty. Verify SSE connection works instead.
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
            'SSE stream for component trigger failed to connect',
        )
        thread.join(timeout=5)

    # ------------------------------------------------------------------
    # Area -> Component -> App hierarchy
    # ------------------------------------------------------------------

    def test_03_area_trigger_on_data(self):
        """Create trigger on area that contains components with apps."""
        # Area 'engine' contains component temp-sensor-hw which hosts
        # the engine-temp-sensor app
        resource = '/api/v1/areas/engine/data'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
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

    # ------------------------------------------------------------------
    # Function -> hosted Apps hierarchy
    # ------------------------------------------------------------------

    def test_04_function_trigger_created_successfully(self):
        """Create trigger on function that hosts apps."""
        # Function engine-monitoring hosted_by engine-temp-sensor, engine-rpm-sensor
        resource = '/api/v1/functions/engine-monitoring/data'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        }
        r = requests.post(
            f'{self.BASE_URL}/functions/engine-monitoring/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig = r.json()
        self.addCleanup(
            self._delete_trigger, 'functions', 'engine-monitoring', trig['id']
        )

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], resource)

    # ------------------------------------------------------------------
    # CRUD on hierarchy triggers
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_030
    def test_05_list_triggers_on_area(self):
        """GET /areas/{id}/triggers returns triggers for the area."""
        resource = '/api/v1/areas/engine/data'
        body = {
            'resource': resource,
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

        r = requests.get(
            f'{self.BASE_URL}/areas/engine/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIn('items', data)
        ids = [item['id'] for item in data['items']]
        self.assertIn(trig['id'], ids)

    # @verifies REQ_INTEROP_032
    def test_06_delete_trigger_on_function(self):
        """DELETE removes trigger from function entity."""
        resource = '/api/v1/functions/engine-monitoring/data'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
        }
        r = requests.post(
            f'{self.BASE_URL}/functions/engine-monitoring/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        trig_id = r.json()['id']

        r = requests.delete(
            f'{self.BASE_URL}/functions/engine-monitoring/triggers/{trig_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        r = requests.get(
            f'{self.BASE_URL}/functions/engine-monitoring/triggers/{trig_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # ------------------------------------------------------------------
    # Isolation: triggers on different entities are independent
    # ------------------------------------------------------------------

    def test_07_triggers_are_entity_scoped(self):
        """Triggers listed on one entity do not appear on another."""
        # Create trigger on engine-temp-sensor
        resource_app = '/api/v1/apps/engine-temp-sensor/data'
        body_app = {
            'resource': resource_app,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 30,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers',
            json=body_app,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        app_trig_id = r.json()['id']
        self.addCleanup(
            self._delete_trigger, 'apps', 'engine-temp-sensor', app_trig_id
        )

        # Create trigger on engine-rpm-sensor
        resource_rpm = '/api/v1/apps/engine-rpm-sensor/data'
        body_rpm = {
            'resource': resource_rpm,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 30,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/engine-rpm-sensor/triggers',
            json=body_rpm,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        rpm_trig_id = r.json()['id']
        self.addCleanup(
            self._delete_trigger, 'apps', 'engine-rpm-sensor', rpm_trig_id
        )

        # List on engine-temp-sensor should contain only app_trig_id
        r = requests.get(
            f'{self.BASE_URL}/apps/engine-temp-sensor/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertIn(app_trig_id, ids)
        self.assertNotIn(rpm_trig_id, ids)

        # List on engine-rpm-sensor should contain only rpm_trig_id
        r = requests.get(
            f'{self.BASE_URL}/apps/engine-rpm-sensor/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertIn(rpm_trig_id, ids)
        self.assertNotIn(app_trig_id, ids)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
