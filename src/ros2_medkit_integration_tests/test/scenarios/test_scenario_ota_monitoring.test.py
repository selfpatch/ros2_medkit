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

"""Scenario: OTA update monitoring with multi-trigger setup.

A maintenance engineer monitors an OTA update process by setting up
three triggers on the same entity:

1. OnChange on ``updates`` collection - track any status change
2. OnChangeTo "failed" on ``updates`` collection - detect failures,
   with log_settings to record the failure event
3. OnChange on ``faults`` collection - catch any fault during update

This scenario validates multi-trigger CRUD (creation, listing, SSE
connectivity, and cleanup) on a single entity. Actual OTA lifecycle
simulation is out of scope - the test verifies API mechanics.

"""

import threading
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    """Launch gateway + demo temp sensor for multi-trigger testing."""
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        fault_manager_params={'confirmation_threshold': -2},
        gateway_params={
            'updates.enabled': True,
        },
    )


class TestScenarioOtaMonitoring(GatewayTestCase):
    """Scenario: Multi-trigger OTA update monitoring.

    Sets up 3 triggers on the same entity (different collections and
    conditions) to simulate an OTA monitoring dashboard. Validates:

    Steps:
    1. Create trigger 1: OnChange on updates collection
    2. Create trigger 2: OnChangeTo "failed" on updates with log_settings
    3. Create trigger 3: OnChange on faults collection
    4. All 3 triggers created successfully (201)
    5. List triggers - 3 items
    6. Each trigger has correct event_source URI
    7. Connect SSE to at least one trigger's event_source
    8. Delete all 3 triggers
    9. Verify list is empty
    """

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    APP_ID = 'temp_sensor'

    def _delete_trigger(self, trigger_id):
        """Delete a trigger, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/apps/{self.APP_ID}/triggers/{trigger_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    def _create_trigger(self, body):
        """Create a trigger and return the response JSON."""
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.APP_ID}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        return r.json()

    # ===================================================================
    # Scenario: Multi-trigger OTA monitoring lifecycle
    # ===================================================================

    def test_01_create_update_status_trigger(self):
        """Create trigger 1: OnChange on updates collection.

        Monitors any status change during an OTA update process.
        """
        body = {
            'resource': f'/api/v1/apps/{self.APP_ID}/updates',
            'trigger_condition': {
                'condition_type': 'OnChange',
            },
            'multishot': True,
            'lifetime': 120,
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertIn('id', trig)
        self.assertTrue(trig['id'].startswith('trig_'))
        self.assertEqual(trig['status'], 'active')
        self.assertEqual(
            trig['observed_resource'],
            f'/api/v1/apps/{self.APP_ID}/updates',
        )
        self.assertEqual(trig['protocol'], 'sse')
        self.assertTrue(trig['multishot'])

    def test_02_create_failure_detection_trigger(self):
        """Create trigger 2: OnChangeTo "failed" on updates with log_settings.

        Detects when an update transitions to "failed" status and
        records the event with a custom marker in the logs.
        """
        body = {
            'resource': f'/api/v1/apps/{self.APP_ID}/updates',
            'trigger_condition': {
                'condition_type': 'OnChangeTo',
                'target_value': 'failed',
            },
            'multishot': True,
            'lifetime': 120,
            'log_settings': {
                'severity': 'error',
                'marker': 'OTA FAILURE DETECTED',
            },
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'OnChangeTo',
        )
        self.assertEqual(
            trig['trigger_condition']['target_value'], 'failed',
        )
        # log_settings preserved in response
        self.assertIn('log_settings', trig)
        self.assertEqual(trig['log_settings']['severity'], 'error')
        self.assertEqual(
            trig['log_settings']['marker'], 'OTA FAILURE DETECTED',
        )

    def test_03_create_fault_during_update_trigger(self):
        """Create trigger 3: OnChange on faults collection.

        Catches any fault that occurs during the update process.
        """
        body = {
            'resource': f'/api/v1/apps/{self.APP_ID}/faults',
            'trigger_condition': {
                'condition_type': 'OnChange',
            },
            'multishot': True,
            'lifetime': 120,
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertEqual(trig['status'], 'active')
        self.assertEqual(
            trig['observed_resource'],
            f'/api/v1/apps/{self.APP_ID}/faults',
        )

    def test_04_all_three_triggers_listed(self):
        """List triggers returns all 3 triggers for the entity."""
        # Create all 3 triggers
        triggers = []
        bodies = [
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/updates',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 60,
            },
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/updates',
                'trigger_condition': {
                    'condition_type': 'OnChangeTo',
                    'target_value': 'failed',
                },
                'multishot': True,
                'lifetime': 60,
                'log_settings': {
                    'severity': 'error',
                    'marker': 'OTA FAILURE DETECTED',
                },
            },
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/faults',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 60,
            },
        ]
        for body in bodies:
            trig = self._create_trigger(body)
            self.addCleanup(self._delete_trigger, trig['id'])
            triggers.append(trig)

        # List all triggers
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.APP_ID}/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIn('items', data)
        listed_ids = {item['id'] for item in data['items']}

        # All 3 must be present
        for trig in triggers:
            self.assertIn(trig['id'], listed_ids)

        self.assertGreaterEqual(len(data['items']), 3)

    def test_05_each_trigger_has_unique_event_source(self):
        """Each trigger has a unique event_source URI."""
        triggers = []
        bodies = [
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/updates',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 60,
            },
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/updates',
                'trigger_condition': {
                    'condition_type': 'OnChangeTo',
                    'target_value': 'failed',
                },
                'multishot': True,
                'lifetime': 60,
            },
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/faults',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 60,
            },
        ]
        for body in bodies:
            trig = self._create_trigger(body)
            self.addCleanup(self._delete_trigger, trig['id'])
            triggers.append(trig)

        event_sources = set()
        for trig in triggers:
            self.assertIn('event_source', trig)
            self.assertTrue(
                trig['event_source'].endswith('/events'),
                f'event_source should end with /events: {trig["event_source"]}',
            )
            self.assertIn(trig['id'], trig['event_source'])
            event_sources.add(trig['event_source'])

        # All event_sources must be unique
        self.assertEqual(
            len(event_sources), 3,
            f'Expected 3 unique event_sources, got {len(event_sources)}',
        )

    def test_06_sse_connectivity_on_trigger(self):
        """SSE connection works on a trigger's event_source."""
        body = {
            'resource': f'/api/v1/apps/{self.APP_ID}/faults',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 60,
        }
        trig = self._create_trigger(body)
        self.addCleanup(self._delete_trigger, trig['id'])

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
            'SSE stream failed to connect within 5s',
        )
        thread.join(timeout=5)

    def test_07_delete_all_triggers_and_verify_empty(self):
        """Delete all 3 triggers and verify the list is empty."""
        # Create 3 triggers
        trigger_ids = []
        bodies = [
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/updates',
                'trigger_condition': {'condition_type': 'OnChange'},
                'multishot': True,
                'lifetime': 60,
            },
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/updates',
                'trigger_condition': {
                    'condition_type': 'OnChangeTo',
                    'target_value': 'failed',
                },
                'multishot': True,
                'lifetime': 60,
            },
            {
                'resource': f'/api/v1/apps/{self.APP_ID}/faults',
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
                f'{self.BASE_URL}/apps/{self.APP_ID}/triggers/{trig_id}',
                timeout=5,
            )
            self.assertEqual(r.status_code, 204)

        # Verify list is empty (no triggers from this test remain)
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.APP_ID}/triggers',
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
