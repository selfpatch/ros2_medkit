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

import json
import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH, FAULT_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

LIDAR_FQN = '/perception/lidar/lidar_sensor'


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

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('triggers_faults_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault')
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report(self, fault_code):
        """Report a FAILED event from the lidar node; the reply is not needed."""
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'trigger stream probe'
        req.source_id = LIDAR_FQN
        self._report_client.call_async(req)
        rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def _open_stream(self, events_url):
        """Read an SSE stream on a thread; returns (state, stop, thread).

        ``state`` gets the response headers under 'headers', every data
        payload under 'events', and any request error under 'error'. A
        trigger stream sends its headers with the first frame, which is why
        the read timeout exceeds the 15 s keepalive.
        """
        state = {'headers': None, 'events': [], 'error': None}
        stop = threading.Event()

        def read_stream():
            try:
                with requests.get(events_url, stream=True, timeout=(5, 30)) as resp:
                    state['headers'] = {
                        'status': resp.status_code,
                        'content_type': resp.headers.get('Content-Type'),
                        'cache_control': resp.headers.get('Cache-Control'),
                    }
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop.is_set():
                            return
                        if line and line.startswith('data:'):
                            state['events'].append(json.loads(line[len('data:'):]))
            except requests.exceptions.RequestException as e:
                state['error'] = repr(e)

        thread = threading.Thread(target=read_stream, daemon=True)
        thread.start()
        return state, stop, thread

    def _events_url(self, trig):
        return f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'

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
    def test_10_sse_endpoint_delivers_fault_events(self):
        """The trigger's event stream answers and delivers a fault event of its app.

        The lidar node's own FQN is the reporting source, so the lidar app's
        trigger must deliver the event. The stream is read on a thread and a
        fresh fault is reported until one arrives: the trigger buffers events,
        so reporting before the stream connects loses nothing.
        """
        trig = self._create_fault_trigger(multishot=True, lifetime=120)
        self.addCleanup(self._delete_trigger, trig['id'])

        state, stop, _ = self._open_stream(self._events_url(trig))
        self.addCleanup(stop.set)
        code = f'TRIGGER_STREAM_PROBE_{trig["id"]}'
        deadline = time.monotonic() + FAULT_TIMEOUT
        delivered = False
        while time.monotonic() < deadline and not delivered:
            self._report(code)
            delivered = any(e.get('payload', {}).get('fault_code') == code
                            for e in list(state['events']))
            time.sleep(0.5)

        self.assertIsNone(state['error'], f'event stream failed: {state["error"]}')
        self.assertIsNotNone(state['headers'], 'event stream never answered')
        self.assertEqual(state['headers']['status'], 200, state['headers'])
        received = [e.get('payload', {}).get('fault_code') for e in list(state['events'])]
        self.assertTrue(delivered, f'no event for {code}; received {received}')

    # @verifies REQ_INTEROP_097
    def test_11_sse_fault_event_has_correct_headers(self):
        """SSE events endpoint for faults returns correct headers."""
        trig = self._create_fault_trigger(lifetime=120)
        self.addCleanup(self._delete_trigger, trig['id'])

        state, stop, _ = self._open_stream(self._events_url(trig))
        self.addCleanup(stop.set)
        self._report(f'TRIGGER_HEADERS_PROBE_{trig["id"]}')
        deadline = time.monotonic() + FAULT_TIMEOUT
        while state['headers'] is None and state['error'] is None and time.monotonic() < deadline:
            time.sleep(0.2)

        self.assertIsNone(state['error'], f'event stream failed: {state["error"]}')
        self.assertIsNotNone(state['headers'], 'event stream never answered')
        self.assertEqual(state['headers']['status'], 200, state['headers'])
        self.assertEqual(state['headers']['content_type'], 'text/event-stream')
        self.assertEqual(state['headers']['cache_control'], 'no-cache')

    # ------------------------------------------------------------------
    # Component-scoped fault trigger
    # ------------------------------------------------------------------

    def test_20_fault_trigger_on_component(self):
        """Fault triggers work on component entities."""
        # Find the host-derived default component (all apps belong to it)
        r = requests.get(f'{self.BASE_URL}/components', timeout=5)
        components = r.json().get('items', [])
        comp_id = components[0]['id'] if components else None

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
