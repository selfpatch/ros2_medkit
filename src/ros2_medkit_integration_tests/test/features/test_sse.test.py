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

"""Feature tests for SSE (Server-Sent Events) fault stream.

Validates SSE stream headers and keepalive connection handling.

Migrated from:
- test_63_sse_stream_endpoint_returns_correct_headers
- test_64_sse_stream_sends_keepalive
"""

import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
    )


class TestSse(GatewayTestCase):
    """SSE (Server-Sent Events) fault stream tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_sse_stream_endpoint_returns_correct_headers(self):
        """GET /faults/stream returns SSE headers."""
        # Use stream=True and timeout to avoid blocking
        try:
            response = requests.get(
                f'{self.BASE_URL}/faults/stream',
                stream=True,
                timeout=2
            )
            # Check SSE-specific headers
            self.assertEqual(response.status_code, 200)
            content_type = response.headers.get('Content-Type', '')
            self.assertIn('text/event-stream', content_type)
            self.assertEqual(
                response.headers.get('Cache-Control'),
                'no-cache'
            )

            # Close the connection (we just wanted to check headers)
            response.close()
        except requests.exceptions.ReadTimeout:
            # Timeout is expected since SSE keeps connection open
            pass

    def test_sse_stream_sends_keepalive(self):
        """SSE stream can be read and handles concurrent connections."""
        received_data = []
        connection_error = []
        stop_event = threading.Event()

        def read_stream():
            try:
                response = requests.get(
                    f'{self.BASE_URL}/faults/stream',
                    stream=True,
                    timeout=35  # Slightly longer than keepalive interval
                )
                for line in response.iter_lines(decode_unicode=True):
                    if stop_event.is_set():
                        break
                    if line:
                        received_data.append(line)
                response.close()
            except requests.exceptions.Timeout:
                # Timeout is expected when stop_event is set
                pass
            except Exception as exc:
                # Capture connection errors for assertion
                connection_error.append(str(exc))

        # Start reading in background thread
        thread = threading.Thread(target=read_stream)
        thread.daemon = True
        thread.start()

        # Wait briefly to ensure connection is established
        time.sleep(1)
        stop_event.set()
        thread.join(timeout=2)

        # Verify no connection errors occurred
        self.assertEqual(len(connection_error), 0,
                         f'SSE stream connection failed: {connection_error}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly.

        Allow exit code -15 (SIGTERM) which is expected during shutdown
        after SSE stream tests close persistent connections.
        """
        for info in proc_info:
            allowed = {0, -2, -15}  # OK, SIGINT, SIGTERM
            self.assertIn(
                info.returncode, allowed,
                f'Process {info.process_name} exited with code {info.returncode}'
            )
