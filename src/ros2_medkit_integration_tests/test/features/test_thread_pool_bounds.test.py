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

"""Integration tests for the bounded thread pools (issue #440).

The gateway is launched with explicit, small overrides for both thread pools:

- ``server.http_thread_pool_size`` bounds the cpp-httplib request workers.
- ``server.executor_threads`` bounds the main rclcpp MultiThreadedExecutor.

These tests verify that the gateway is fully functional with the bounded pools
(discovery, data sampling, SSE), and - crucially - that regular requests are
still served while an SSE stream holds one HTTP pool thread for its lifetime.
That last check is the practical lower bound on ``http_thread_pool_size``:
it must leave headroom above the number of concurrent SSE streams.
"""

import concurrent.futures
import threading
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

# Bounded pool sizes under test. Both are set to values DISTINCT from the
# shipped defaults (http=3, executor=2) so that a regression which ignored the
# parameters would change observable behaviour. The HTTP pool is larger than the
# single SSE stream opened below so there is headroom for regular requests.
HTTP_THREAD_POOL_SIZE = 5
EXECUTOR_THREADS = 3


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        gateway_params={
            'server.http_thread_pool_size': HTTP_THREAD_POOL_SIZE,
            'server.executor_threads': EXECUTOR_THREADS,
        },
    )


class TestThreadPoolBounds(GatewayTestCase):
    """Gateway behaves correctly with both thread pools bounded small."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_gateway_functional_with_bounded_pools(self):
        """Discovery and data sampling work with bounded executor and HTTP pools.

        Reaching this test already proves the gateway started, served /health,
        and completed discovery (graph events processed by the bounded main
        executor). Here we additionally sample live data, exercising the full
        request path through the bounded HTTP pool.
        """
        apps = self.get_json('/apps').get('items', [])
        app_ids = {a.get('id') for a in apps}
        self.assertIn('temp_sensor', app_ids)

        # A data sample exercises the HTTP handler -> data provider path.
        data = self.poll_endpoint('/apps/temp_sensor/data', timeout=15.0)
        self.assertIn('items', data)

    def test_requests_served_while_sse_stream_open(self):
        """Regular requests succeed while an SSE stream holds a pool thread.

        Each active SSE stream consumes one HTTP pool worker for its entire
        lifetime. With the pool bounded to HTTP_THREAD_POOL_SIZE, an open stream
        must still leave workers free for ordinary traffic. We open one fault
        stream, then fire several concurrent requests and require all of them
        to succeed.
        """
        stop_event = threading.Event()
        connected = threading.Event()
        stream_errors = []

        def read_stream():
            try:
                resp = requests.get(
                    f'{self.BASE_URL}/faults/stream',
                    stream=True,
                    timeout=30,
                )
                self.assertEqual(resp.status_code, 200)
                connected.set()
                for _ in resp.iter_lines(decode_unicode=True):
                    if stop_event.is_set():
                        break
                resp.close()
            except requests.exceptions.Timeout:
                pass
            except Exception as exc:  # noqa: BLE001 - surfaced via assertion
                stream_errors.append(str(exc))
            finally:
                connected.set()

        stream_thread = threading.Thread(target=read_stream, daemon=True)
        stream_thread.start()
        try:
            self.assertTrue(
                connected.wait(timeout=5),
                'SSE stream failed to connect within 5s',
            )
            self.assertEqual(stream_errors, [], f'SSE stream error: {stream_errors}')

            # Fire more concurrent requests than the free workers (pool minus the
            # one SSE thread); they must all complete, not deadlock behind the
            # open stream.
            def hit(path):
                r = requests.get(f'{self.BASE_URL}{path}', timeout=10)
                return r.status_code

            paths = ['/health', '/apps', '/areas', '/functions', '/health', '/apps']
            with concurrent.futures.ThreadPoolExecutor(max_workers=len(paths)) as pool:
                statuses = list(pool.map(hit, paths))

            # The stream must still be open (holding a pool worker) at this point,
            # otherwise "served while an SSE stream is open" was not actually
            # tested - the worker would have been freed before the burst ran.
            self.assertTrue(
                stream_thread.is_alive(),
                'SSE stream closed before the request burst - it was not holding a pool worker',
            )
            self.assertTrue(
                all(s == 200 for s in statuses),
                f'Some requests were not served while an SSE stream was open: {statuses}',
            )
        finally:
            stop_event.set()
            stream_thread.join(timeout=3)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exit cleanly (SIGTERM allowed after closing SSE)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
