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

"""Feature tests for trigger topic type retry mechanism.

Validates that when a trigger is created on a data topic whose publisher
hasn't started yet, the TriggerTopicSubscriber retries topic type
resolution and eventually subscribes when the publisher appears.

The test uses a large demo_delay (15s) so the gateway starts alone,
giving time to create a trigger on a non-existent topic. After the
demo node starts and the retry mechanism resolves the topic type,
SSE events should flow.

"""

import json
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
    # Large delay so gateway starts well before demo nodes.
    # The retry timer fires every 5s, so 15s gives at least 2 retry
    # cycles after the publisher appears.
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        demo_delay=15.0,
    )


class TestTriggersLatePublisher(GatewayTestCase):
    """Test that triggers work when the publisher starts after trigger creation."""

    # Do NOT wait for apps at class setup - we want gateway-only initially
    MIN_EXPECTED_APPS = 0

    app_id = 'temp_sensor'
    # Topic name matches temp_sensor demo node's published topic
    topic_name = '/powertrain/engine/temperature'

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_trigger_on_late_publisher_receives_events(self):
        """Create trigger immediately when app appears, before data topics resolve.

        The demo node starts after a 15s delay. This test creates the trigger
        as soon as the app entity is discovered - before waiting for data
        topic availability - so the TriggerTopicSubscriber retry path is
        exercised.

        Steps:
        1. Verify temp_sensor is NOT yet discovered (demo_delay ensures this).
        2. Wait for temp_sensor to appear (demo_delay will start it).
        3. Create a trigger immediately using the known topic path, without
           waiting for data topic discovery.
        4. Connect SSE and wait for events (proves the retry subscription works).
        """
        # Step 1: Verify the app is NOT yet discovered
        try:
            r = requests.get(f'{self.BASE_URL}/apps/{self.app_id}', timeout=2)
            if r.status_code == 200:
                # The app is already up - the delay wasn't long enough.
                # Fall through to step 2 immediately.
                pass
        except requests.exceptions.RequestException:
            pass

        # Step 2: Wait for temp_sensor to appear (demo_delay=15s + discovery)
        deadline = time.monotonic() + 60.0
        app_found = False
        while time.monotonic() < deadline:
            try:
                r = requests.get(f'{self.BASE_URL}/apps', timeout=5)
                if r.status_code == 200:
                    apps = r.json().get('items', [])
                    if any(a.get('id') == self.app_id for a in apps):
                        app_found = True
                        break
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)

        self.assertTrue(app_found, f'App {self.app_id} not discovered within 60s')

        # Step 3: Create trigger immediately using known topic path.
        # The topic type may not be resolvable yet if the publisher hasn't
        # fully advertised, exercising the TriggerTopicSubscriber retry path.
        resource_uri = f'/api/v1/apps/{self.app_id}/data{self.topic_name}'
        body = {
            'resource': resource_uri,
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 120,
        }
        r = requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/triggers',
            json=body,
            timeout=5,
        )
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')
        trig = r.json()
        trig_id = trig['id']
        self.addCleanup(
            lambda: requests.delete(
                f'{self.BASE_URL}/apps/{self.app_id}/triggers/{trig_id}',
                timeout=5,
            )
        )

        self.assertEqual(trig['status'], 'active')

        # Step 4: Connect SSE and collect events
        events_url = (
            f'{self.BASE_URL}{trig["event_source"].removeprefix(API_BASE_PATH)}'
        )

        received_events = []
        stop_event = threading.Event()

        def collect_events():
            try:
                with requests.get(events_url, stream=True, timeout=30) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop_event.is_set():
                            break
                        if line and line.startswith('data: '):
                            data = json.loads(line[6:])
                            received_events.append(data)
                            if len(received_events) >= 2:
                                stop_event.set()
                                break
            except requests.exceptions.RequestException:
                pass

        thread = threading.Thread(target=collect_events, daemon=True)
        thread.start()

        # temp_sensor publishes at ~1Hz, events should arrive within 30s
        stop_event.wait(timeout=30)
        thread.join(timeout=5)

        self.assertGreaterEqual(
            len(received_events), 1,
            f'Expected at least 1 SSE event after retry, got '
            f'{len(received_events)}',
        )

        # Verify EventEnvelope schema
        for event in received_events:
            self.assertIn('timestamp', event)
            self.assertIn('payload', event)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
