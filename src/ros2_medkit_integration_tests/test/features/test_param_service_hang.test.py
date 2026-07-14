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
here is the actual user-facing symptom against a real gateway process talking
to a real (discoverable but never-replying) demo node over the ROS 2 graph.

The REAL gate is ``test_01_fresh_configurations_request_is_bounded_not_hung``.
It MUST run first, on a cold Ros2ParameterTransport: the shared ``spin_mutex_``
is free and this node is not yet negative-cached, so a single ``/configurations``
GET grabs the free lock and, pre-fix (SyncParametersClient calls with no
timeout), blocks FOREVER inside cache_default_values waiting for a reply the
node never sends - nothing else holds the lock, so there is no acquisition-
timeout rescue and the request simply never returns. The client-side timeout
then fires and fails the test (RED). Post-fix the round trip is bounded by
``parameter_service_timeout_sec`` and the gateway returns a 503 in ~2s (GREEN).
This test was verified RED against the unfixed transport (timeouts removed) and
GREEN with the fix.

``test_02_health_stays_responsive_under_concurrent_config_calls`` is a
SECONDARY smoke test, deliberately NOT the hang gate. Because the shared
``spin_mutex_`` already serialised parameter round trips even pre-fix, ``/health``
stays responsive under concurrent single-node ``/configurations`` calls in BOTH
the fixed and unfixed builds (only one worker ever wedges; the rest time out on
lock acquisition and free up), so this check cannot distinguish fixed from
broken - test_01 does that. By the time test_02 runs, test_01 has already
negative-cached the node, so its concurrent calls return fast via that cache;
what it asserts is only the weaker property that the REST API keeps serving
unrelated endpoints while several parameter calls are in flight.
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

# More concurrent config calls than server.http_thread_pool_size (6, see
# gateway_params.yaml). Post test_01 these are fast negative-cache hits, so this
# no longer pins every worker - it just exercises concurrent request handling.
CONCURRENT_CONFIG_REQUESTS = 8


def generate_test_description():
    return create_test_launch(
        demo_nodes=[APP_ID],
        fault_manager=False,
    )


class TestParamServiceHang(GatewayTestCase):
    """Proves an unresponsive parameter service cannot hang the REST API."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {APP_ID}

    def test_01_fresh_configurations_request_is_bounded_not_hung(self):
        """A single FRESH /configurations GET must return a bounded 503, not hang.

        THIS is the #531 gate (see module docstring). It runs first (the
        ``test_01_`` prefix orders it ahead of the smoke test), so the
        gateway's Ros2ParameterTransport is in its initial state for this
        node: the shared spin_mutex_ is free and the node is not yet
        negative-cached.

        Pre-fix, the underlying SyncParametersClient calls carried NO timeout,
        so this one fresh request grabs the free lock, enters
        cache_default_values -> list_parameters, and blocks FOREVER waiting
        for a reply the unresponsive node never sends. Nothing else holds the
        lock, so there is no acquisition-timeout rescue - the request never
        returns, the client-side timeout below fires, and the test fails (RED).

        Post-fix, every SyncParametersClient call is bounded by
        parameter_service_timeout_sec (default 2.0s): cache_default_values'
        round trip times out, the node is negative-cached, and list_parameters
        short-circuits, so the gateway returns a 503 SOVD error in ~2s (GREEN).

        Running first also means THIS request is what negative-caches the node
        for the following smoke test.
        """
        try:
            start = time.monotonic()
            response = requests.get(
                f'{self.BASE_URL}/apps/{APP_ID}/configurations',
                timeout=15,
            )
            elapsed = time.monotonic() - start
        except requests.exceptions.RequestException as e:
            self.fail(
                'GET /configurations against a fresh unresponsive node did '
                f'not return within the 15s client timeout ({e}) - the gateway '
                'is not bounding the parameter round trip: the request hung '
                'instead of returning a 503 (#531 regression).'
            )

        print(
            f'GET /apps/{APP_ID}/configurations returned '
            f'{response.status_code} in {elapsed:.2f}s (fresh, uncontended)'
        )
        self.assertEqual(
            response.status_code, 503,
            f'expected 503 (node unavailable), got {response.status_code}'
        )
        # Bounded well under the 15s client timeout: a fresh round trip is
        # ~parameter_service_timeout_sec (2s); 10s leaves generous slack for
        # sanitizer CI without letting a near-hang masquerade as a pass.
        self.assertLess(
            elapsed, 10.0,
            'fresh /configurations against an unresponsive node took '
            f'{elapsed:.1f}s - expected a bounded failure within a small '
            'multiple of parameter_service_timeout_sec, not a near-timeout '
            'rescue by the test client itself'
        )
        # x-medkit-* vendor codes are remapped to the SOVD vendor-error
        # envelope on the wire (see write_generic_error in
        # http/detail/primitives.cpp): error_code stays a generic SOVD code,
        # the precise vendor code moves to vendor_code.
        data = response.json()
        self.assertEqual(data.get('error_code'), 'vendor-error')
        self.assertEqual(data.get('vendor_code'), 'x-medkit-ros2-node-unavailable')
        self.assertIn('entity_id', data.get('parameters', {}))
        self.assertEqual(data['parameters']['entity_id'], APP_ID)

    def test_02_health_stays_responsive_under_concurrent_config_calls(self):
        """SMOKE: /health keeps answering with several /configurations in flight.

        Secondary coverage, NOT the #531 gate (test_01 is - see module
        docstring). This does not and cannot prove "no worker ever frees":
        even pre-fix the shared spin_mutex_ serialises parameter round trips,
        so under N concurrent single-node /configurations only one worker
        wedges while the rest time out on lock acquisition and free up, and
        /health is served either way. It asserts only the weaker, still-useful
        property that the REST API stays responsive while several parameter
        calls are in flight.

        By the time this runs, test_01 has already negative-cached the node,
        so these concurrent calls return fast via that cache rather than
        blocking on fresh round trips - which is itself end-to-end proof the
        negative cache works, just not a hang gate.
        """
        config_threads = []
        # +1 party for this (main) thread: all hammer threads block at the
        # barrier until every one has started, then release together, so the
        # concurrent requests actually overlap rather than trickling out under
        # scheduler-dependent thread startup.
        release_barrier = threading.Barrier(CONCURRENT_CONFIG_REQUESTS + 1)

        def hammer_configurations():
            release_barrier.wait()
            try:
                requests.get(
                    f'{self.BASE_URL}/apps/{APP_ID}/configurations',
                    timeout=20,
                )
            except requests.exceptions.RequestException:
                # These threads exist only to put concurrent requests in
                # flight; test_01 already asserted the response shape.
                pass

        for _ in range(CONCURRENT_CONFIG_REQUESTS):
            thread = threading.Thread(target=hammer_configurations, daemon=True)
            config_threads.append(thread)
            thread.start()

        try:
            # Release all hammer threads to fire their requests together.
            release_barrier.wait(timeout=10)

            # Small fixed buffer so the just-released requests actually reach
            # the server and occupy workers before probing /health, rather than
            # /health racing them to a free worker.
            time.sleep(0.3)

            start = time.monotonic()
            response = requests.get(f'{self.BASE_URL}/health', timeout=30)
            elapsed = time.monotonic() - start
        except threading.BrokenBarrierError:
            # release_barrier.wait(timeout=10) tripped its own timeout (or the
            # barrier was otherwise broken): not all hammer threads reached the
            # rendezvous. Turn this into a clear failure instead of an opaque
            # BrokenBarrierError traceback that would ERROR the test.
            self.fail(
                f'hammer threads did not reach the barrier within 10s '
                f'(expected {CONCURRENT_CONFIG_REQUESTS} + 1 parties) - could '
                'not put the concurrent /configurations calls in flight'
            )
        except requests.exceptions.RequestException as e:
            self.fail(
                f'/health did not respond within 30s while '
                f'{CONCURRENT_CONFIG_REQUESTS} concurrent /configurations '
                f'calls were in flight against the unresponsive node: {e}'
            )
        finally:
            for thread in config_threads:
                thread.join(timeout=25)

        print(
            f'GET /health returned {response.status_code} in {elapsed:.2f}s '
            f'while {CONCURRENT_CONFIG_REQUESTS} concurrent /configurations '
            'calls were in flight against the unresponsive node'
        )
        self.assertEqual(
            response.status_code, 200,
            f'/health returned {response.status_code} while '
            f'{CONCURRENT_CONFIG_REQUESTS} concurrent /configurations calls '
            'were in flight'
        )
        # Generous 30s bound: removes sanitizer-CI flake risk (TSan slows this
        # thread/lock-heavy path) while still comfortably under the 120s CTest
        # TIMEOUT. This is a responsiveness smoke check, not the hang gate.
        self.assertLess(
            elapsed, 30.0,
            f'/health took {elapsed:.1f}s to respond while '
            f'{CONCURRENT_CONFIG_REQUESTS} concurrent /configurations calls '
            'were in flight - the REST API must stay responsive'
        )


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
