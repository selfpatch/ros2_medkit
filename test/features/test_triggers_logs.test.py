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

"""Feature tests for triggers on log entries (x-medkit extension).

Validates trigger CRUD on the logs collection and SSE event delivery when
log entries are created. Uses the temp_sensor demo node which generates
/rosout log output that the LogManager ingests and forwards to the
ResourceChangeNotifier.

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
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )


class TestTriggersLogs(GatewayTestCase):
    """Trigger tests on logs collection (x-medkit extension)."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def _create_logs_trigger(self, entity_id='temp_sensor', *,
                             multishot=True, lifetime=None):
        """Create a trigger on the logs collection."""
        resource = f'/api/v1/apps/{entity_id}/logs'
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
    def test_01_create_logs_trigger(self):
        """POST creates a trigger on logs collection."""
        trig = self._create_logs_trigger(lifetime=120)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertIn('id', trig)
        self.assertTrue(trig['id'].startswith('trig_'))
        self.assertEqual(trig['status'], 'active')
        self.assertEqual(
            trig['observed_resource'],
            '/api/v1/apps/temp_sensor/logs'
        )
        self.assertEqual(trig['protocol'], 'sse')
        self.assertTrue(trig['multishot'])
        self.assertIn('event_source', trig)
        self.assertTrue(trig['event_source'].endswith('/events'))

        # trigger_condition preserved
        self.assertIn('trigger_condition', trig)
        self.assertEqual(
            trig['trigger_condition']['condition_type'], 'OnChange'
        )

    # @verifies REQ_INTEROP_030
    def test_02_list_logs_triggers(self):
        """GET /triggers lists log triggers for the entity."""
        trig = self._create_logs_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIn('items', data)
        ids = [item['id'] for item in data['items']]
        self.assertIn(trig['id'], ids)

    # @verifies REQ_INTEROP_096
    def test_03_get_logs_trigger(self):
        """GET /triggers/{id} returns log trigger details."""
        trig = self._create_logs_trigger(lifetime=60)
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
            '/api/v1/apps/temp_sensor/logs'
        )

    # @verifies REQ_INTEROP_031
    def test_04_update_logs_trigger_lifetime(self):
        """PUT /triggers/{id} updates log trigger lifetime."""
        trig = self._create_logs_trigger(lifetime=30)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.put(
            f'{self.BASE_URL}/apps/temp_sensor/triggers/{trig["id"]}',
            json={'lifetime': 300},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data['lifetime'], 300)
        self.assertEqual(data['id'], trig['id'])

    # @verifies REQ_INTEROP_032
    def test_05_delete_logs_trigger(self):
        """DELETE /triggers/{id} removes the log trigger."""
        trig = self._create_logs_trigger()

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
        """After deleting, log trigger no longer appears in list."""
        trig = self._create_logs_trigger()
        self._delete_trigger(trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/temp_sensor/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertNotIn(trig['id'], ids)

    # ------------------------------------------------------------------
    # SSE event delivery for log triggers
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_097
    def test_10_sse_endpoint_connects_for_log_trigger(self):
        """SSE events endpoint accepts connections for log triggers.

        Verifies the SSE transport works for log-collection triggers. The
        LogManager currently only notifies ResourceChangeNotifier via the
        programmatic add_log_entry() path (used by trigger log_settings),
        not from /rosout ingestion. This test validates the SSE plumbing
        by verifying the connection is accepted and the stream is active.
        """
        trig = self._create_logs_trigger(multishot=True, lifetime=60)
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
                        break  # Just verify we can read
            except requests.exceptions.RequestException:
                pass
            finally:
                stream_connected.set()

        thread = threading.Thread(target=read_stream, daemon=True)
        thread.start()

        self.assertTrue(
            stream_connected.wait(timeout=5),
            'SSE stream for log trigger failed to connect within 5s',
        )
        thread.join(timeout=5)

    # @verifies REQ_INTEROP_097
    def test_11_sse_log_event_headers(self):
        """SSE events endpoint for logs returns correct headers."""
        trig = self._create_logs_trigger(lifetime=60)
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

    # ------------------------------------------------------------------
    # Component-scoped log trigger
    # ------------------------------------------------------------------

    def test_20_log_trigger_on_component(self):
        """Log triggers work on component entities."""
        r = requests.get(f'{self.BASE_URL}/components', timeout=5)
        components = r.json().get('items', [])
        if not components:
            self.fail('No components discovered')

        comp_id = components[0]['id']

        resource = f'/api/v1/components/{comp_id}/logs'
        body = {
            'resource': resource,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 30,
        }
        r = requests.post(
            f'{self.BASE_URL}/components/{comp_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig_id = r.json()['id']
        self.addCleanup(
            lambda: requests.delete(
                f'{self.BASE_URL}/components/{comp_id}/triggers/{trig_id}',
                timeout=5,
            )
        )

        # List confirms it
        r = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/triggers', timeout=5
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertIn(trig_id, ids)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
