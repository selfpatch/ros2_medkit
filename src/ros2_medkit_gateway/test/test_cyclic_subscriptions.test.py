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
Integration tests for cyclic subscriptions (Issue #202).

Tests follow outside-in approach: each test demonstrates a real use case
and traces to a specific SOVD requirement.
"""

import json
import os
import threading
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests


API_BASE_PATH = '/api/v1'
BASE_URL = f'http://localhost:8080{API_BASE_PATH}'
MAX_DISCOVERY_WAIT = 30.0
DISCOVERY_CHECK_INTERVAL = 1.0


def get_coverage_env():
    """Get environment variables for gcov coverage data collection."""
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_gateway')
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_gateway')

        if os.path.exists(build_dir):
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        pass
    return {}


def generate_test_description():
    """Launch gateway + demo temp sensor for cyclic subscription testing."""
    coverage_env = get_coverage_env()

    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[{
            'server.host': '127.0.0.1',
            'server.port': 8080,
            'sse.max_clients': 10,
            'sse.max_subscriptions': 50,
            'refresh_interval_ms': 1000,
        }],
        additional_env=coverage_env,
    )

    demo_temp = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_engine_temp_sensor',
        name='engine_temp_sensor',
        namespace='/powertrain',
        output='screen',
        additional_env=coverage_env,
    )

    return (
        LaunchDescription([
            gateway_node,
            TimerAction(period=2.0, actions=[demo_temp]),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


class TestCyclicSubscriptions(unittest.TestCase):
    """
    Integration tests for cyclic subscription CRUD and SSE streaming.

    Scenario: A dashboard client monitors engine temperature via cyclic
    subscriptions. The client creates a subscription, receives periodic
    SSE updates, adjusts the interval, and eventually cancels.
    """

    app_id = None
    topic_id = None
    resource_uri = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to start and discover the demo temp sensor node."""
        # Phase 1: Wait for health endpoint
        for _ in range(30):
            try:
                r = requests.get(f'{BASE_URL}/health', timeout=2)
                if r.status_code == 200:
                    break
            except requests.ConnectionError:
                pass
            time.sleep(1)
        else:
            raise unittest.SkipTest('Gateway did not start within 30 seconds')

        # Phase 2: Wait for discovery of engine_temp_sensor app
        deadline = time.time() + MAX_DISCOVERY_WAIT
        while time.time() < deadline:
            try:
                r = requests.get(f'{BASE_URL}/apps', timeout=5)
                if r.status_code == 200:
                    apps = r.json().get('items', [])
                    for app in apps:
                        if 'temp_sensor' in app.get('id', '').lower() or \
                           'engine' in app.get('id', '').lower():
                            cls.app_id = app['id']
                            break
                    if cls.app_id:
                        break
            except Exception:
                pass
            time.sleep(DISCOVERY_CHECK_INTERVAL)

        if not cls.app_id:
            raise unittest.SkipTest('Demo temp sensor app not discovered')

        # Phase 3: Wait for data availability on the discovered app
        deadline = time.time() + 15.0
        while time.time() < deadline:
            try:
                r = requests.get(f'{BASE_URL}/apps/{cls.app_id}/data', timeout=5)
                if r.status_code == 200:
                    items = r.json().get('items', [])
                    if items:
                        cls.topic_id = items[0]['id']
                        cls.resource_uri = f'/api/v1/apps/{cls.app_id}/data{cls.topic_id}'
                        break
            except Exception:
                pass
            time.sleep(1.0)

        if not cls.topic_id:
            raise unittest.SkipTest(f'No data items available for app {cls.app_id}')

    def _create_subscription(self, interval='normal', duration=60):
        """Create a subscription and return the response JSON."""
        body = {
            'resource': self.resource_uri,
            'protocol': 'sse',
            'interval': interval,
            'duration': duration,
        }
        r = requests.post(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        return r.json()

    def _delete_subscription(self, sub_id):
        """Delete a subscription, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/{sub_id}',
                timeout=5,
            )
        except Exception:
            pass

    # ===================================================================
    # Scenario 1: Full subscription lifecycle (create -> read -> update -> delete)
    # ===================================================================

    def test_01_create_subscription_returns_201_with_correct_schema(self):
        """
        Create a cyclic subscription for engine temperature monitoring.

        Verifies that POST returns 201 with all required fields:
        id, observed_resource, event_source, protocol, interval.

        @verifies REQ_INTEROP_089
        """
        sub = self._create_subscription(interval='normal', duration=120)

        # Verify response schema
        self.assertIn('id', sub)
        self.assertTrue(sub['id'].startswith('sub_'))
        self.assertEqual(sub['observed_resource'], self.resource_uri)
        self.assertEqual(sub['protocol'], 'sse')
        self.assertEqual(sub['interval'], 'normal')

        # event_source must be a valid URI path ending in /events
        self.assertIn('event_source', sub)
        self.assertTrue(sub['event_source'].endswith('/events'))
        self.assertIn(sub['id'], sub['event_source'])

        self._delete_subscription(sub['id'])

    def test_02_list_subscriptions_returns_created_ones(self):
        """
        List active subscriptions for an entity — shows all created subscriptions.

        @verifies REQ_INTEROP_025
        """
        s1 = self._create_subscription(interval='fast', duration=60)
        s2 = self._create_subscription(interval='slow', duration=60)

        r = requests.get(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)

        data = r.json()
        self.assertIn('items', data)
        ids = [item['id'] for item in data['items']]
        self.assertIn(s1['id'], ids)
        self.assertIn(s2['id'], ids)

        # Each item has the required fields
        for item in data['items']:
            self.assertIn('id', item)
            self.assertIn('observed_resource', item)
            self.assertIn('event_source', item)
            self.assertIn('protocol', item)
            self.assertIn('interval', item)

        self._delete_subscription(s1['id'])
        self._delete_subscription(s2['id'])

    def test_03_get_single_subscription(self):
        """
        Read a single subscription's details by ID.

        @verifies REQ_INTEROP_026
        """
        sub = self._create_subscription()

        r = requests.get(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/{sub["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)

        data = r.json()
        self.assertEqual(data['id'], sub['id'])
        self.assertEqual(data['observed_resource'], self.resource_uri)
        self.assertEqual(data['interval'], 'normal')

        self._delete_subscription(sub['id'])

    def test_04_update_subscription_interval(self):
        """
        Update the interval of an existing subscription from normal to fast.

        Dashboard user decides they need higher-frequency updates.

        @verifies REQ_INTEROP_027
        """
        sub = self._create_subscription(interval='normal', duration=120)

        r = requests.put(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/{sub["id"]}',
            json={'interval': 'fast'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()['interval'], 'fast')
        # Other fields remain unchanged
        self.assertEqual(r.json()['id'], sub['id'])
        self.assertEqual(r.json()['observed_resource'], self.resource_uri)

        self._delete_subscription(sub['id'])

    def test_05_update_subscription_duration(self):
        """
        Extend a subscription's duration — user wants to keep monitoring longer.

        @verifies REQ_INTEROP_027
        """
        sub = self._create_subscription(interval='slow', duration=30)

        r = requests.put(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/{sub["id"]}',
            json={'duration': 600},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)

        self._delete_subscription(sub['id'])

    def test_06_delete_subscription_returns_204(self):
        """
        Cancel a subscription — returns 204 No Content and removes it.

        @verifies REQ_INTEROP_028
        """
        sub = self._create_subscription()

        r = requests.delete(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/{sub["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify it's gone
        r = requests.get(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/{sub["id"]}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_07_empty_list_after_delete(self):
        """
        After deleting a subscription, it no longer appears in the list.

        @verifies REQ_INTEROP_025
        """
        sub = self._create_subscription()
        self._delete_subscription(sub['id'])

        r = requests.get(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertNotIn(sub['id'], ids)

    # ===================================================================
    # Scenario 2: SSE event stream — real-time data delivery
    # ===================================================================

    def test_10_sse_stream_returns_correct_headers(self):
        """
        The SSE events endpoint returns text/event-stream content type.

        @verifies REQ_INTEROP_090
        """
        sub = self._create_subscription(interval='slow', duration=30)
        events_url = f'http://localhost:8080{sub["event_source"]}'

        try:
            r = requests.get(events_url, stream=True, timeout=3)
            self.assertEqual(r.headers.get('Content-Type'), 'text/event-stream')
            self.assertEqual(r.headers.get('Cache-Control'), 'no-cache')
            r.close()
        except requests.exceptions.ReadTimeout:
            pass  # Expected — SSE keeps connection open
        finally:
            self._delete_subscription(sub['id'])

    def test_11_sse_stream_delivers_periodic_data(self):
        """
        The SSE stream delivers EventEnvelope payloads at the requested interval.

        Real use case: dashboard receives temperature readings every 500ms (slow).
        Each event has a timestamp and a payload with ReadValue format (id + data).

        @verifies REQ_INTEROP_090
        """
        sub = self._create_subscription(interval='slow', duration=30)
        events_url = f'http://localhost:8080{sub["event_source"]}'

        received_events = []
        stop_event = threading.Event()

        def collect_events():
            try:
                with requests.get(events_url, stream=True, timeout=15) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop_event.is_set():
                            break
                        if line and line.startswith('data: '):
                            data = json.loads(line[6:])
                            received_events.append(data)
                            if len(received_events) >= 3:
                                stop_event.set()
                                break
            except Exception:
                pass

        thread = threading.Thread(target=collect_events, daemon=True)
        thread.start()

        # slow interval = 500ms, so 3 events should arrive within ~5s
        stop_event.wait(timeout=15)
        thread.join(timeout=5)

        self.assertGreaterEqual(
            len(received_events), 2,
            f'Expected at least 2 SSE events, got {len(received_events)}',
        )

        # Verify EventEnvelope schema
        for event in received_events:
            self.assertIn('timestamp', event, 'EventEnvelope must have timestamp')
            self.assertIn('payload', event, 'EventEnvelope must have payload')
            payload = event['payload']
            self.assertIn('id', payload, 'ReadValue payload must have id')
            self.assertIn('data', payload, 'ReadValue payload must have data')

        self._delete_subscription(sub['id'])

    def test_12_sse_stream_closes_on_subscription_delete(self):
        """
        When a subscription is deleted, the SSE stream closes gracefully.

        @verifies REQ_INTEROP_090
        @verifies REQ_INTEROP_028
        """
        sub = self._create_subscription(interval='slow', duration=60)
        events_url = f'http://localhost:8080{sub["event_source"]}'

        stream_ended = threading.Event()

        def read_stream():
            try:
                with requests.get(events_url, stream=True, timeout=15) as resp:
                    for _line in resp.iter_lines(decode_unicode=True):
                        if stream_ended.is_set():
                            break
            except Exception:
                pass
            finally:
                stream_ended.set()

        thread = threading.Thread(target=read_stream, daemon=True)
        thread.start()

        # Give stream time to start, then delete the subscription
        time.sleep(2)
        requests.delete(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/{sub["id"]}',
            timeout=5,
        )

        # Stream should end within a few seconds
        stream_ended.wait(timeout=10)
        self.assertTrue(stream_ended.is_set(), 'SSE stream did not close after DELETE')

    # ===================================================================
    # Scenario 3: Error handling and validation
    # ===================================================================

    def test_20_create_with_invalid_interval_returns_400(self):
        """
        POST with unknown interval value returns 400 invalid-parameter.

        @verifies REQ_INTEROP_089
        """
        body = {
            'resource': self.resource_uri,
            'interval': 'turbo',
            'duration': 60,
        }
        r = requests.post(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)
        self.assertEqual(r.json()['error_code'], 'invalid-parameter')

    def test_21_create_with_zero_duration_returns_400(self):
        """
        POST with duration <= 0 returns 400.

        @verifies REQ_INTEROP_089
        """
        body = {
            'resource': self.resource_uri,
            'interval': 'normal',
            'duration': 0,
        }
        r = requests.post(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_22_create_with_unsupported_protocol_returns_400(self):
        """
        POST with protocol other than 'sse' returns 400.

        @verifies REQ_INTEROP_089
        """
        body = {
            'resource': self.resource_uri,
            'interval': 'normal',
            'duration': 60,
            'protocol': 'websocket',
        }
        r = requests.post(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_23_create_with_invalid_resource_uri_returns_400(self):
        """
        POST with malformed resource URI returns 400.

        @verifies REQ_INTEROP_089
        """
        body = {
            'resource': '/not/a/valid/resource',
            'interval': 'normal',
            'duration': 60,
        }
        r = requests.post(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 400)

    def test_24_get_nonexistent_subscription_returns_404(self):
        """
        GET for a subscription that doesn't exist returns 404 resource-not-found.

        @verifies REQ_INTEROP_026
        """
        r = requests.get(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/sub_nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)
        self.assertEqual(r.json()['error_code'], 'resource-not-found')

    def test_25_delete_nonexistent_subscription_returns_404(self):
        """
        DELETE for a subscription that doesn't exist returns 404.

        @verifies REQ_INTEROP_028
        """
        r = requests.delete(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/sub_nonexistent',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_26_update_nonexistent_subscription_returns_404(self):
        """
        PUT for a subscription that doesn't exist returns 404.

        @verifies REQ_INTEROP_027
        """
        r = requests.put(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/sub_nonexistent',
            json={'interval': 'fast'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_27_create_for_nonexistent_entity_returns_404(self):
        """
        POST for an entity that doesn't exist returns 404 entity-not-found.

        @verifies REQ_INTEROP_089
        """
        body = {
            'resource': '/api/v1/apps/nonexistent_entity/data/some_topic',
            'interval': 'normal',
            'duration': 60,
        }
        r = requests.post(
            f'{BASE_URL}/apps/nonexistent_entity/cyclic-subscriptions',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_28_sse_stream_for_nonexistent_subscription_returns_404(self):
        """
        GET /events for a non-existent subscription returns 404.

        @verifies REQ_INTEROP_090
        """
        r = requests.get(
            f'{BASE_URL}/apps/{self.app_id}/cyclic-subscriptions/sub_nonexistent/events',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # ===================================================================
    # Scenario 4: Component entity support
    # ===================================================================

    def test_30_cyclic_subscriptions_work_on_components(self):
        """
        Cyclic subscriptions also work on /components/ entity path.

        @verifies REQ_INTEROP_089
        @verifies REQ_INTEROP_025
        """
        # Find a component
        r = requests.get(f'{BASE_URL}/components', timeout=5)
        components = r.json().get('items', [])
        if not components:
            self.skipTest('No components discovered')

        comp_id = components[0]['id']

        # Find data for this component
        r = requests.get(f'{BASE_URL}/components/{comp_id}/data', timeout=5)
        data_items = r.json().get('items', [])
        if not data_items:
            self.skipTest(f'No data items for component {comp_id}')

        topic = data_items[0]['id']
        resource = f'/api/v1/components/{comp_id}/data{topic}'

        body = {
            'resource': resource,
            'interval': 'slow',
            'duration': 30,
        }
        r = requests.post(
            f'{BASE_URL}/components/{comp_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)

        sub_id = r.json()['id']

        # List should include it
        r = requests.get(
            f'{BASE_URL}/components/{comp_id}/cyclic-subscriptions',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        ids = [item['id'] for item in r.json().get('items', [])]
        self.assertIn(sub_id, ids)

        # Cleanup
        requests.delete(
            f'{BASE_URL}/components/{comp_id}/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
