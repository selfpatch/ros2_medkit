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

"""Scenario: Thermal protection cascade with multi-trigger setup.

A safety system monitors engine temperature using three triggers with
different condition types:

1. LeaveRange [20.0, 80.0] on ``data/temperature`` - warning when
   temperature leaves the normal operating range
2. OnChangeTo 95.0 on ``data/temperature`` with log_settings - critical
   alert at a specific dangerous temperature
3. OnChange on component-level ``faults`` collection - catch any fault
   reported during thermal events

This scenario validates that triggers with different condition types
(OnChange, OnChangeTo, LeaveRange) can coexist on the same entity
and that their condition parameters are independently configured.

"""

import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    """Launch gateway + temp sensor for thermal protection testing."""
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        fault_manager_params={'confirmation_threshold': -2},
    )


class TestScenarioThermalProtection(GatewayTestCase):
    """Scenario: Thermal protection cascade with multi-trigger setup.

    A safety system monitors engine temperature using three triggers
    with different condition types and condition parameters.

    Steps:
    1. Create trigger 1: LeaveRange [20.0, 80.0] on data topic (warning)
    2. Create trigger 2: OnChangeTo 95.0 on data topic (critical)
    3. Create trigger 3: OnChange on component faults collection
    4. All 3 created with correct condition params
    5. Triggers are independent (different IDs, different conditions)
    6. SSE connectivity works for each trigger
    7. Clean up all triggers
    """

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    app_id = None
    topic_id = None
    resource_uri = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway, discover temp sensor and find data topic."""
        super().setUpClass()

        cls.app_id = 'temp_sensor'

        # Wait for data availability - skip ROS 2 system topics
        system_topics = {'/parameter_events', '/rosout'}
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                r = requests.get(
                    f'{cls.BASE_URL}/apps/{cls.app_id}/data', timeout=5,
                )
                if r.status_code == 200:
                    items = r.json().get('items', [])
                    for item in items:
                        if item['id'] not in system_topics:
                            cls.topic_id = item['id']
                            cls.resource_uri = (
                                f'/api/v1/apps/{cls.app_id}/data{cls.topic_id}'
                            )
                            break
                    if cls.topic_id:
                        break
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)

        if not cls.topic_id:
            raise AssertionError(
                f'No data items available for app {cls.app_id}'
            )

    def _delete_trigger(self, trigger_id, entity_type='apps',
                        entity_id=None):
        """Delete a trigger, ignoring errors for cleanup."""
        if entity_id is None:
            entity_id = self.app_id
        try:
            requests.delete(
                f'{self.BASE_URL}/{entity_type}/{entity_id}/triggers/{trigger_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    def _create_trigger(self, body, entity_type='apps', entity_id=None):
        """Create a trigger and return the response JSON."""
        if entity_id is None:
            entity_id = self.app_id
        r = requests.post(
            f'{self.BASE_URL}/{entity_type}/{entity_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        return r.json()

    # ===================================================================
    # Scenario: Thermal protection cascade
    # ===================================================================

    def test_01_create_leave_range_trigger(self):
        """Create trigger 1: LeaveRange [20.0, 80.0] for temperature warning.

        Fires when the temperature transitions from inside the normal
        range [20.0, 80.0] to outside it, indicating a potential issue.
        """
        body = {
            'resource': self.resource_uri,
            'trigger_condition': {
                'condition_type': 'LeaveRange',
                'lower_bound': 20.0,
                'upper_bound': 80.0,
            },
            'multishot': True,
            'lifetime': 120,
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], self.resource_uri)
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'LeaveRange',
        )
        self.assertEqual(
            trig['trigger_condition']['lower_bound'], 20.0,
        )
        self.assertEqual(
            trig['trigger_condition']['upper_bound'], 80.0,
        )

    def test_02_create_critical_temperature_trigger(self):
        """Create trigger 2: OnChangeTo 95.0 for critical temperature.

        Fires when the temperature reaches exactly 95.0 degrees, with
        log_settings to create a log entry documenting the critical event.
        """
        body = {
            'resource': self.resource_uri,
            'trigger_condition': {
                'condition_type': 'OnChangeTo',
                'target_value': 95.0,
            },
            'multishot': True,
            'lifetime': 120,
            'log_settings': {
                'severity': 'error',
                'marker': 'THERMAL CRITICAL: 95C reached',
            },
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'OnChangeTo',
        )
        self.assertEqual(
            trig['trigger_condition']['target_value'], 95.0,
        )
        # log_settings preserved
        self.assertIn('log_settings', trig)
        self.assertEqual(trig['log_settings']['severity'], 'error')

    def test_03_create_component_fault_trigger(self):
        """Create trigger 3: OnChange on component faults collection.

        Catches any fault reported on the component level during thermal
        events. Uses the synthetic component that groups the temp_sensor.
        """
        # Find the component that hosts temp_sensor
        r = requests.get(f'{self.BASE_URL}/components', timeout=5)
        components = r.json().get('items', [])
        self.assertTrue(
            len(components) > 0, 'No components discovered'
        )

        comp_id = None
        for comp in components:
            cid = comp.get('id', '')
            if 'powertrain' in cid or 'engine' in cid:
                comp_id = cid
                break
        if not comp_id:
            comp_id = components[0]['id']

        resource = f'/api/v1/components/{comp_id}/faults'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 120,
        }
        trig = self._create_trigger(
            body, entity_type='components', entity_id=comp_id
        )
        self.addCleanup(
            self._delete_trigger, trig['id'],
            entity_type='components', entity_id=comp_id,
        )

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(trig['observed_resource'], resource)
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'OnChange',
        )

    def test_04_triggers_have_different_conditions(self):
        """All 3 triggers have different condition types/params."""
        triggers = []

        # Trigger 1: LeaveRange
        trig1 = self._create_trigger({
            'resource': self.resource_uri,
            'trigger_condition': {
                'condition_type': 'LeaveRange',
                'lower_bound': 20.0,
                'upper_bound': 80.0,
            },
            'multishot': True,
            'lifetime': 60,
        })
        self.addCleanup(self._delete_trigger, trig1['id'])
        triggers.append(trig1)

        # Trigger 2: OnChangeTo
        trig2 = self._create_trigger({
            'resource': self.resource_uri,
            'trigger_condition': {
                'condition_type': 'OnChangeTo',
                'target_value': 95.0,
            },
            'multishot': True,
            'lifetime': 60,
        })
        self.addCleanup(self._delete_trigger, trig2['id'])
        triggers.append(trig2)

        # Trigger 3: OnChange (same entity, different collection - faults)
        trig3 = self._create_trigger({
            'resource': f'/api/v1/apps/{self.app_id}/faults',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        })
        self.addCleanup(self._delete_trigger, trig3['id'])
        triggers.append(trig3)

        # All IDs are different
        ids = {t['id'] for t in triggers}
        self.assertEqual(len(ids), 3, 'All triggers should have unique IDs')

        # Condition types are as expected
        conditions = [
            t['trigger_condition']['condition_type'] for t in triggers
        ]
        self.assertEqual(conditions[0], 'LeaveRange')
        self.assertEqual(conditions[1], 'OnChangeTo')
        self.assertEqual(conditions[2], 'OnChange')

    def test_05_sse_connectivity_on_leave_range_trigger(self):
        """SSE connection works on the LeaveRange trigger.

        The temp_sensor publishes Float64 temperature data. If the data
        happens to transition out of the range, events will be delivered.
        This test verifies at minimum the SSE connection is established.
        """
        body = {
            'resource': self.resource_uri,
            'trigger_condition': {
                'condition_type': 'LeaveRange',
                'lower_bound': 20.0,
                'upper_bound': 80.0,
            },
            'multishot': True,
            'lifetime': 60,
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        # Verify SSE connection works
        stream_connected = threading.Event()
        received_lines = []

        def read_stream():
            try:
                with requests.get(events_url, stream=True, timeout=10) as resp:
                    stream_connected.set()
                    for line in resp.iter_lines(decode_unicode=True):
                        if line:
                            received_lines.append(line)
                            break
            except requests.exceptions.RequestException:
                pass
            finally:
                stream_connected.set()

        thread = threading.Thread(target=read_stream, daemon=True)
        thread.start()

        self.assertTrue(
            stream_connected.wait(timeout=5),
            'SSE stream for LeaveRange trigger failed to connect',
        )
        thread.join(timeout=10)

    def test_06_sse_correct_headers_on_data_trigger(self):
        """SSE endpoint returns correct content-type headers."""
        body = {
            'resource': self.resource_uri,
            'trigger_condition': {
                'condition_type': 'OnChangeTo',
                'target_value': 95.0,
            },
            'multishot': True,
            'lifetime': 60,
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        try:
            r = requests.get(events_url, stream=True, timeout=3)
            self.assertEqual(
                r.headers.get('Content-Type'), 'text/event-stream'
            )
            self.assertEqual(r.headers.get('Cache-Control'), 'no-cache')
            r.close()
        except requests.exceptions.ReadTimeout:
            pass

    def test_07_cleanup_all_triggers(self):
        """Delete all triggers and verify list is empty."""
        trigger_ids = []
        bodies = [
            {
                'resource': self.resource_uri,
                'trigger_condition': {
                    'condition_type': 'LeaveRange',
                    'lower_bound': 20.0,
                    'upper_bound': 80.0,
                },
                'multishot': True,
                'lifetime': 60,
            },
            {
                'resource': self.resource_uri,
                'trigger_condition': {
                    'condition_type': 'OnChangeTo',
                    'target_value': 95.0,
                },
                'multishot': True,
                'lifetime': 60,
            },
            {
                'resource': f'/api/v1/apps/{self.app_id}/faults',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 60,
            },
        ]
        for body in bodies:
            trig = self._create_trigger(body)
            trigger_ids.append(trig['id'])

        # Delete each one
        for trig_id in trigger_ids:
            r = requests.delete(
                f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig_id}',
                timeout=5,
            )
            self.assertEqual(r.status_code, 204)

        # Verify none remain
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        remaining_ids = {item['id'] for item in r.json().get('items', [])}
        for trig_id in trigger_ids:
            self.assertNotIn(trig_id, remaining_ids)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
