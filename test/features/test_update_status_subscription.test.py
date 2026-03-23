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

"""Integration test: cyclic subscription on /updates/{id}/status.

Validates that a client can create a cyclic subscription to monitor
update progress via SSE, receiving periodic status snapshots as the
update transitions through pending -> inProgress -> completed.
"""

import json
import os
import threading
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import get_coverage_env


PORT = get_test_port(0)


def _get_test_plugin_path():
    """Get path to test_update_backend.so demo plugin."""
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_gateway', 'libtest_update_backend.so'
    )


def generate_test_description():
    """Launch gateway with update plugin and a demo node."""
    coverage_env = get_coverage_env()
    plugin_path = _get_test_plugin_path()

    gateway = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='gateway_update_sub',
        output='screen',
        parameters=[{
            'server.host': '127.0.0.1',
            'server.port': PORT,
            'refresh_interval_ms': 1000,
            'updates.enabled': True,
            'plugins': ['test_update_backend'],
            'plugins.test_update_backend.path': plugin_path,
            'sse.max_clients': 10,
            'sse.max_subscriptions': 50,
        }],
        additional_env=coverage_env,
    )

    # Need at least one demo node so we have an entity for subscriptions
    demo_node = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_engine_temp_sensor',
        name='temp_sensor',
        namespace='/powertrain/engine',
        output='screen',
        additional_env=coverage_env,
    )

    return LaunchDescription([
        gateway,
        TimerAction(
            period=2.0,
            actions=[demo_node],
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestUpdateStatusSubscription(GatewayTestCase):
    """Test cyclic subscription on /updates/{id}/status."""

    BASE_URL = f'http://127.0.0.1:{PORT}{API_BASE_PATH}'
    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    app_id = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway and discover an app for subscriptions."""
        super().setUpClass()

        # Find the temp_sensor app
        r = requests.get(f'{cls.BASE_URL}/apps', timeout=10)
        apps = r.json().get('items', [])
        for app in apps:
            if 'temp_sensor' in app.get('id', '').lower():
                cls.app_id = app['id']
                break

        if not cls.app_id:
            raise AssertionError('No app discovered for subscription host')

    def _register_package(self, pkg_id, automated=False):
        """Register update package and schedule cleanup."""
        pkg = {
            'id': pkg_id,
            'update_name': f'Test {pkg_id}',
            'automated': automated,
            'origins': ['proximity'],
        }
        r = requests.post(f'{self.BASE_URL}/updates', json=pkg, timeout=5)
        self.assertEqual(r.status_code, 201)
        self.addCleanup(self._cleanup_package, pkg_id)

    def _cleanup_package(self, pkg_id):
        """Best-effort cleanup."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline:
            r = requests.get(
                f'{self.BASE_URL}/updates/{pkg_id}/status', timeout=5
            )
            if r.status_code == 404:
                break
            if r.status_code == 200 and r.json().get('status') in (
                'completed', 'failed',
            ):
                break
            time.sleep(0.2)
        requests.delete(f'{self.BASE_URL}/updates/{pkg_id}', timeout=5)

    def _cleanup_subscription(self, sub_id):
        """Best-effort subscription cleanup."""
        requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )

    def test_01_create_subscription_on_update_status(self):
        """POST cyclic-subscriptions with /updates/{id}/status resource."""
        self._register_package('sub-test-pkg')

        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json={
                'resource': '/api/v1/updates/sub-test-pkg/status',
                'interval': 'fast',
                'duration': 60,
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        data = r.json()
        self.assertIn('id', data)
        self.assertEqual(
            data['observed_resource'],
            '/api/v1/updates/sub-test-pkg/status',
        )
        self.assertIn('event_source', data)
        self.assertEqual(data['protocol'], 'sse')
        self.addCleanup(self._cleanup_subscription, data['id'])

    def test_02_subscription_listed_for_entity(self):
        """Subscription on update status appears in entity's subscription list."""
        self._register_package('sub-list-pkg')

        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json={
                'resource': '/api/v1/updates/sub-list-pkg/status',
                'interval': 'normal',
                'duration': 60,
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']
        self.addCleanup(self._cleanup_subscription, sub_id)

        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        items = r.json().get('items', [])
        sub_ids = [s['id'] for s in items]
        self.assertIn(sub_id, sub_ids)

    def test_03_sse_stream_delivers_update_progress(self):
        """SSE stream delivers status snapshots during update prepare."""
        self._register_package('sub-sse-pkg')

        # Create subscription
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json={
                'resource': '/api/v1/updates/sub-sse-pkg/status',
                'interval': 'fast',
                'duration': 120,
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        sub_data = r.json()
        sub_id = sub_data['id']
        event_source = sub_data['event_source']
        self.addCleanup(self._cleanup_subscription, sub_id)

        # Construct full URL for SSE events endpoint
        # event_source is like /api/v1/apps/{id}/cyclic-subscriptions/{sub}/events
        events_url = f'http://127.0.0.1:{PORT}{event_source}'

        # Collect SSE events in background
        events = []
        stop_event = threading.Event()

        def collect_sse():
            try:
                with requests.get(
                    events_url,
                    stream=True,
                    timeout=30,
                    headers={'Accept': 'text/event-stream'},
                ) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop_event.is_set():
                            break
                        if line and line.startswith('data: '):
                            try:
                                payload = json.loads(line[6:])
                                events.append(payload)
                            except json.JSONDecodeError:
                                pass
            except Exception:
                pass

        sse_thread = threading.Thread(target=collect_sse, daemon=True)
        sse_thread.start()

        # Give SSE connection time to establish
        time.sleep(1.0)

        # Trigger update prepare to generate status changes
        requests.put(
            f'{self.BASE_URL}/updates/sub-sse-pkg/prepare', timeout=5
        )

        # Wait for prepare to complete and SSE events to arrive
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            r = requests.get(
                f'{self.BASE_URL}/updates/sub-sse-pkg/status', timeout=5
            )
            if r.status_code == 200 and r.json().get('status') == 'completed':
                break
            time.sleep(0.5)

        # Give a bit more time for final SSE events
        time.sleep(2.0)
        stop_event.set()
        sse_thread.join(timeout=5)

        # Validate we received SSE events with EventEnvelope format
        self.assertGreater(len(events), 0, 'No SSE events received')

        # Each event should be an EventEnvelope with timestamp + payload
        statuses_seen = set()
        for event in events:
            self.assertIn('timestamp', event, 'EventEnvelope must have timestamp')
            if 'payload' in event:
                payload = event['payload']
                if 'status' in payload:
                    statuses_seen.add(payload['status'])

        # We should see at least one update status
        self.assertTrue(
            statuses_seen & {'pending', 'inProgress', 'completed'},
            f'Expected update status in events, saw: {statuses_seen}',
        )

    def test_04_subscription_with_nonexistent_package(self):
        """Subscription on nonexistent update package - accepts 201 or 503."""
        # Package not registered - sampler will return error
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json={
                'resource': '/api/v1/updates/nonexistent-pkg/status',
                'interval': 'normal',
                'duration': 60,
            },
            timeout=5,
        )
        # Subscription creation should succeed (sampler check passes - updates
        # sampler is registered). But the SSE stream will deliver error frames.
        # OR: if the sampler does an initial check, it returns 503.
        # Accept either 201 (lazy) or 503 (eager validation).
        self.assertIn(r.status_code, [201, 503])
        if r.status_code == 201:
            self.addCleanup(self._cleanup_subscription, r.json()['id'])

    def test_05_delete_subscription(self):
        """DELETE removes the update status subscription."""
        self._register_package('sub-del-pkg')

        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json={
                'resource': '/api/v1/updates/sub-del-pkg/status',
                'interval': 'normal',
                'duration': 60,
            },
            timeout=5,
        )
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']

        r = requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify gone
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
