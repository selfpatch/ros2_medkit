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

"""Feature tests for triggers on update status changes.

Validates trigger CRUD operations on the updates collection. Since the
UpdateManager requires a loaded backend plugin to actually produce status
changes, this test focuses on CRUD correctness (create, list, get,
update lifetime, delete) which works independently of whether a backend
is loaded.

"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        gateway_params={
            'updates.enabled': True,
        },
    )


class TestTriggersUpdates(GatewayTestCase):
    """Trigger CRUD tests on updates collection."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def _create_updates_trigger(self, entity_id='temp_sensor', *,
                                multishot=True, lifetime=None):
        """Create a trigger on the updates collection."""
        resource = f'/api/v1/apps/{entity_id}/updates'
        body = {
            'resource': resource,
            'trigger_condition': {
                'condition_type': 'OnChange',
            },
            'multishot': multishot,
        }
        if lifetime is not None:
            body['lifetime'] = lifetime
        r = requests.post(
            f'{self.BASE_URL}/apps/{entity_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        return r.json()

    def _delete_trigger(self, trigger_id, entity_id='temp_sensor'):
        """Delete a trigger, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/apps/{entity_id}/triggers/{trigger_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    # ------------------------------------------------------------------
    # CRUD operations
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_029
    def test_01_create_updates_trigger(self):
        """POST creates a trigger on updates collection."""
        trig = self._create_updates_trigger(lifetime=120)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertIn('id', trig)
        self.assertTrue(trig['id'].startswith('trig_'))
        self.assertEqual(trig['status'], 'active')
        self.assertEqual(
            trig['observed_resource'],
            '/api/v1/apps/temp_sensor/updates'
        )
        self.assertEqual(trig['protocol'], 'sse')
        self.assertTrue(trig['multishot'])
        self.assertIn('event_source', trig)
        self.assertTrue(trig['event_source'].endswith('/events'))

    # @verifies REQ_INTEROP_030
    def test_02_list_updates_triggers(self):
        """GET /triggers lists updates triggers for the entity."""
        trig1 = self._create_updates_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig1['id'])
        trig2 = self._create_updates_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig2['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIn('items', data)
        ids = [item['id'] for item in data['items']]
        self.assertIn(trig1['id'], ids)
        self.assertIn(trig2['id'], ids)

    # @verifies REQ_INTEROP_096
    def test_03_get_updates_trigger(self):
        """GET /triggers/{id} returns updates trigger details."""
        trig = self._create_updates_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data['id'], trig['id'])
        self.assertEqual(data['status'], 'active')
        self.assertEqual(
            data['observed_resource'],
            '/api/v1/apps/temp_sensor/updates'
        )
        self.assertEqual(
            data['trigger_condition']['condition_type'], 'OnChange'
        )

    # @verifies REQ_INTEROP_031
    def test_04_update_updates_trigger_lifetime(self):
        """PUT /triggers/{id} updates the trigger lifetime."""
        trig = self._create_updates_trigger(lifetime=30)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trig["id"]}',
            json={'lifetime': 600},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data['lifetime'], 600)
        self.assertEqual(data['id'], trig['id'])
        self.assertEqual(data['status'], 'active')

    # @verifies REQ_INTEROP_032
    def test_05_delete_updates_trigger(self):
        """DELETE /triggers/{id} removes the updates trigger."""
        trig = self._create_updates_trigger()

        r = requests.delete(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_06_empty_list_after_delete(self):
        """After deleting, trigger no longer appears in list."""
        trig = self._create_updates_trigger()
        self._delete_trigger(trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertNotIn(trig['id'], ids)

    # ------------------------------------------------------------------
    # SSE endpoint validation
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_097
    def test_10_events_endpoint_returns_sse_headers(self):
        """SSE events endpoint returns correct content-type headers."""
        trig = self._create_updates_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        events_path = trig['event_source'].removeprefix('/api/v1')
        events_url = f'{self.BASE_URL}{events_path}'

        try:
            r = requests.get(events_url, stream=True, timeout=3)
            self.assertEqual(
                r.headers.get('Content-Type'), 'text/event-stream'
            )
            self.assertEqual(r.headers.get('Cache-Control'), 'no-cache')
            r.close()
        except requests.exceptions.ReadTimeout:
            pass  # Expected - SSE keeps connection open

    # @verifies REQ_INTEROP_097
    def test_11_events_for_nonexistent_trigger_returns_404(self):
        """GET /events for a nonexistent trigger returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/trig_nonexistent/events',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # ------------------------------------------------------------------
    # Error handling
    # ------------------------------------------------------------------

    def test_20_update_with_invalid_lifetime_returns_400(self):
        """PUT with zero lifetime returns 400."""
        trig = self._create_updates_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trig["id"]}',
            json={'lifetime': 0},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_21_update_with_negative_lifetime_returns_400(self):
        """PUT with negative lifetime returns 400."""
        trig = self._create_updates_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trig["id"]}',
            json={'lifetime': -10},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_22_update_nonexistent_trigger_returns_404(self):
        """PUT for a nonexistent trigger returns 404."""
        r = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/trig_nonexistent',
            json={'lifetime': 60},
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_23_create_with_negative_lifetime_returns_400(self):
        """POST with negative lifetime returns 400."""
        body = {
            'resource': '/api/v1/apps/temp_sensor/updates',
            'trigger_condition': {'condition_type': 'OnChange'},
            'lifetime': -1,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/temp_sensor/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
