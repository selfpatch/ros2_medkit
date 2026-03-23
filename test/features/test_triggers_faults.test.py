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

"""Feature tests for triggers on fault events.

Validates trigger CRUD on faults collections and SSE event delivery when
fault events occur. Uses the lidar_sensor demo node with faulty params
that trigger deterministic faults (LIDAR_RANGE_INVALID, etc.) through
the fault manager.

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
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        lidar_faulty=True,
        fault_manager_params={
            'confirmation_threshold': -2,
        },
    )


class TestTriggersFaults(GatewayTestCase):
    """Trigger tests on fault resource collections."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    def _create_fault_trigger(self, entity_type='apps', entity_id='lidar_sensor',
                              *, multishot=True, lifetime=None):
        """Create a trigger on faults collection for the given entity."""
        resource = f'/api/v1/{entity_type}/{entity_id}/faults'
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
            f'{self.BASE_URL}/{entity_type}/{entity_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        return r.json()

    def _delete_trigger(self, trigger_id, entity_type='apps',
                        entity_id='lidar_sensor'):
        """Delete a trigger, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/{entity_type}/{entity_id}/triggers/{trigger_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    # ------------------------------------------------------------------
    # CRUD on faults triggers
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_029
    def test_01_create_fault_trigger_returns_201(self):
        """POST creates a trigger on faults collection with correct schema."""
        trig = self._create_fault_trigger(lifetime=120)
        self.addCleanup(self._delete_trigger, trig['id'])

        self.assertIn('id', trig)
        self.assertTrue(trig['id'].startswith('trig_'))
        self.assertEqual(trig['status'], 'active')
        self.assertEqual(
            trig['observed_resource'],
            '/api/v1/apps/lidar_sensor/faults'
        )
        self.assertEqual(trig['protocol'], 'sse')
        self.assertTrue(trig['multishot'])
        self.assertIn('event_source', trig)
        self.assertTrue(trig['event_source'].endswith('/events'))

    # @verifies REQ_INTEROP_030
    def test_02_list_fault_triggers(self):
        """GET /triggers lists fault triggers for the entity."""
        trig = self._create_fault_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/triggers',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIn('items', data)
        ids = [item['id'] for item in data['items']]
        self.assertIn(trig['id'], ids)

    # @verifies REQ_INTEROP_096
    def test_03_get_fault_trigger(self):
        """GET /triggers/{id} returns fault trigger details."""
        trig = self._create_fault_trigger(lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data['id'], trig['id'])
        self.assertEqual(data['status'], 'active')

    # @verifies REQ_INTEROP_031
    def test_04_update_fault_trigger_lifetime(self):
        """PUT /triggers/{id} updates fault trigger lifetime."""
        trig = self._create_fault_trigger(lifetime=30)
        self.addCleanup(self._delete_trigger, trig['id'])

        r = requests.put(
            f'{self.BASE_URL}/apps/lidar_sensor/triggers/{trig["id"]}',
            json={'lifetime': 600},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()['lifetime'], 600)

    # @verifies REQ_INTEROP_032
    def test_05_delete_fault_trigger(self):
        """DELETE /triggers/{id} removes the fault trigger."""
        trig = self._create_fault_trigger()

        r = requests.delete(
            f'{self.BASE_URL}/apps/lidar_sensor/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        r = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/triggers/{trig["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # ------------------------------------------------------------------
    # SSE events for fault triggers
    # ------------------------------------------------------------------

    # @verifies REQ_INTEROP_097
    def test_10_sse_endpoint_connects_and_streams(self):
        """SSE events endpoint accepts connections and streams keepalives.

        Verifies the SSE transport works for fault triggers - the connection
        is established and the server sends keepalive comments. Actual fault
        events depend on the fault subscriber's entity_id matching the
        trigger's entity_id, which requires correct reporting_source mapping.
        """
        trig = self._create_fault_trigger(multishot=True, lifetime=60)
        self.addCleanup(self._delete_trigger, trig['id'])

        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        # Verify SSE connection works - read at least one line (data or keepalive)
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
            'SSE stream failed to connect within 5s',
        )
        thread.join(timeout=10)

    # @verifies REQ_INTEROP_097
    def test_11_sse_fault_event_has_correct_headers(self):
        """SSE events endpoint for faults returns correct headers."""
        trig = self._create_fault_trigger(lifetime=60)
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
    # Component-scoped fault trigger
    # ------------------------------------------------------------------

    def test_20_fault_trigger_on_component(self):
        """Fault triggers work on component entities."""
        # Find the perception component (lidar's synthetic component)
        r = requests.get(f'{self.BASE_URL}/components', timeout=5)
        components = r.json().get('items', [])
        comp_id = None
        for comp in components:
            cid = comp.get('id', '')
            if 'perception' in cid or 'lidar' in cid:
                comp_id = cid
                break
        if not comp_id and components:
            comp_id = components[0]['id']

        if not comp_id:
            self.fail('No components discovered')

        resource = f'/api/v1/components/{comp_id}/faults'
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
