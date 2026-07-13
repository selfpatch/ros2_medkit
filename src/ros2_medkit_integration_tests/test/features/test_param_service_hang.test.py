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

"""End-to-end regression coverage for #531.

A discoverable-but-unresponsive node's parameter service must not be able to
hang the gateway's REST API.

Unit/component coverage already proves Ros2ParameterTransport itself bounds
every SyncParametersClient call (see
ros2_medkit_gateway/test/test_ros2_parameter_transport.cpp). What is proven
here is the actual user-facing symptom: against a real gateway process
talking to a real (discoverable but never-replying) demo node over the ROS 2
graph, `/configurations` returns a bounded error instead of hanging, and -
critically - unrelated endpoints like `/health` keep answering even while
several such calls are in flight at once, instead of the whole gateway
freezing until restart (the pre-fix symptom: unbounded parameter round trips
pinned HTTP worker threads forever, eventually exhausting the pool).

Test ordering matters here (hence the numeric ``test_NN_`` prefixes,
overriding unittest's default alphabetical order only incidentally - they
also happen to already sort correctly): test_01 deliberately runs on a cold
Ros2ParameterTransport negative cache so its concurrent calls exercise
genuine ~parameter_service_timeout_sec round trips (the real point of the
worker-pool-exhaustion proof). By the time test_02 runs, test_01's calls
have already negative-cached this node, so test_02 legitimately - and still
correctly - observes a fast cache-hit failure rather than a fresh round
trip; both are "bounded, not hanging", which is what test_02 asserts.
"""

import threading
import time
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

APP_ID = 'unresponsive_param'

# Must exceed server.http_thread_pool_size (6, see gateway_params.yaml) so the
# concurrent /configurations calls can actually pin every HTTP worker at once.
CONCURRENT_STUCK_REQUESTS = 8


def generate_test_description():
    return create_test_launch(
        demo_nodes=[APP_ID],
        fault_manager=False,
    )


class TestParamServiceHang(GatewayTestCase):
    """Proves an unresponsive parameter service cannot hang the REST API."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {APP_ID}

    def test_01_health_stays_responsive_under_stuck_node(self):
        """/health must keep answering while /configurations calls are stuck.

        This is the actual #531 user-facing symptom. Before the fix, an
        unbounded SyncParametersClient round trip held its HTTP worker
        thread forever; enough concurrent requests against an unresponsive
        node's parameter service would eventually pin every worker in the
        pool, and the entire REST API - including endpoints with nothing to
        do with parameters, like /health - would stop answering until the
        gateway process was restarted.

        Reproduces that precondition directly: fire more concurrent
        /configurations requests against the unresponsive node than there
        are HTTP workers (server.http_thread_pool_size defaults to 6), then,
        while they are still in flight, assert /health answers within a
        short, fixed client-side timeout. After the fix each stuck call is
        bounded by parameter_service_timeout_sec, so workers free up in time
        for /health to get one; pre-fix this call would simply never
        return.

        Runs first (see module docstring) so every one of the 8 concurrent
        calls below hits a cold negative cache and genuinely blocks for a
        real round trip - not a cache hit - which is what actually pressures
        the worker pool.
        """
        stuck_threads = []
        # +1 party for this (main) thread: all hammer threads block here
        # until every one of them has started and is about to issue its
        # request, then release together. A plain time.sleep() before
        # starting the /health probe cannot guarantee that - Python thread
        # startup is scheduler-dependent, and a health check that races
        # ahead of the hammer threads reaching the server would under-stress
        # the pool and produce a falsely reassuring near-zero elapsed time.
        release_barrier = threading.Barrier(CONCURRENT_STUCK_REQUESTS + 1)

        def hammer_configurations():
            release_barrier.wait()
            try:
                requests.get(
                    f'{self.BASE_URL}/apps/{APP_ID}/configurations',
                    timeout=20,
                )
            except requests.exceptions.RequestException:
                # The point of this thread is to occupy a worker for a
                # bounded time, not to assert on its own outcome - test_02
                # below covers the bounded-error response shape directly.
                pass

        for _ in range(CONCURRENT_STUCK_REQUESTS):
            thread = threading.Thread(target=hammer_configurations, daemon=True)
            stuck_threads.append(thread)
            thread.start()

        try:
            # Release all hammer threads to fire their requests together.
            release_barrier.wait(timeout=10)

            # Small fixed buffer for the just-released requests to actually
            # reach the server and occupy HTTP workers before probing
            # /health, so the pool is genuinely under pressure rather than
            # /health racing them to a free worker before any of them start
            # blocking. Comfortably smaller than the ~2s bound each stuck
            # call is held to, so it does not eat into the 5s /health budget.
            time.sleep(0.3)

            start = time.monotonic()
            response = requests.get(f'{self.BASE_URL}/health', timeout=5)
            elapsed = time.monotonic() - start
        except requests.exceptions.RequestException as e:
            self.fail(
                f'/health did not respond within 5s while '
                f'{CONCURRENT_STUCK_REQUESTS} concurrent /configurations '
                f'calls were stuck against the unresponsive node: {e}'
            )
        finally:
            for thread in stuck_threads:
                thread.join(timeout=25)

        print(
            f'GET /health returned {response.status_code} in {elapsed:.2f}s '
            f'while {CONCURRENT_STUCK_REQUESTS} concurrent /configurations '
            'calls were stuck against the unresponsive node'
        )
        self.assertEqual(
            response.status_code, 200,
            f'/health returned {response.status_code} while the pool was '
            'under pressure from stuck /configurations calls'
        )
        self.assertLess(
            elapsed, 5.0,
            f'/health took {elapsed:.1f}s to respond while '
            f'{CONCURRENT_STUCK_REQUESTS} concurrent /configurations calls '
            'were stuck - the REST API must stay responsive, not queue '
            'behind the unresponsive node'
        )

    def test_02_configurations_returns_bounded_error_not_hang(self):
        """GET /{app}/configurations against an unresponsive node fails fast.

        Before the fix, the underlying SyncParametersClient calls had no
        timeout, so this request would hang forever once the node's
        list_parameters service accepted the call but never replied. The
        client-side `timeout=15` below would turn that hang into a
        requests.exceptions.Timeout (a test failure/error) rather than a
        passing 503 - the request only reaches the `expected_status=503`
        assertion at all because the gateway bounds the round trip itself
        (parameter_service_timeout_sec, default 2.0s) and returns a SOVD
        error instead of blocking.

        By the time this runs, test_01's concurrent calls have already
        negative-cached this node (see module docstring), so this request
        is expected to return near-instantly via that cache rather than
        paying a fresh round trip - a fast cache hit is exactly the kind of
        "bounded, not hanging" behavior this test is asserting, and is
        itself proof the negative cache introduced by the #531 fix works
        end-to-end, not just in the component test.
        """
        start = time.monotonic()
        data = self.get_json(
            f'/apps/{APP_ID}/configurations',
            timeout=15,
            expected_status=503,
        )
        elapsed = time.monotonic() - start
        print(f'GET /apps/{APP_ID}/configurations returned 503 in {elapsed:.2f}s')

        self.assertLess(
            elapsed, 10.0,
            'GET /configurations against an unresponsive node took '
            f'{elapsed:.1f}s - expected a bounded failure well within the '
            'parameter service timeout, not a near-timeout rescue by the '
            'test client itself'
        )
        # x-medkit-* vendor codes are remapped to the SOVD vendor-error
        # envelope on the wire (see write_generic_error in
        # http/detail/primitives.cpp): error_code stays a generic SOVD code,
        # the precise vendor code moves to vendor_code.
        self.assertEqual(data.get('error_code'), 'vendor-error')
        self.assertEqual(data.get('vendor_code'), 'x-medkit-ros2-node-unavailable')
        self.assertIn('entity_id', data.get('parameters', {}))
        self.assertEqual(data['parameters']['entity_id'], APP_ID)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed).

        Covers the demo node's own shutdown path too: its list_parameters
        callback blocks the executor on rclcpp::ok(), so this also proves
        the node's custom SIGTERM handling (see unresponsive_param_node.cpp)
        actually unblocks it and exits cleanly instead of requiring a
        SIGKILL escalation.
        """
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
