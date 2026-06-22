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

"""Starvation-guard tests for the bounded HTTP pool (issue #440).

`test_thread_pool_bounds` runs a *well-sized* pool, so it can only ever pass.
This file launches a pool deliberately BELOW the documented budget
(`http_thread_pool_size` < `sse.max_clients` + `data_provider.cold_wait_cap`)
and asserts the guards the PR's safety actually rests on:

- the gateway emits the startup warning that flags the misconfiguration, and
- the bounded `executor_threads` value is actually applied (the gateway logs the
  thread count, so it is observable rather than merely set).

Both checks read the gateway's own process output, so they are deterministic and
do not depend on request timing.
"""

import unittest

import launch_testing

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

# Pool below the shipped budget: 2 < sse.max_clients(2) + cold_wait_cap(4) = 6.
# executor_threads is set to a non-default value so the "applied" assertion is
# meaningful (a regression ignoring the parameter would log a different count).
HTTP_THREAD_POOL_SIZE = 2
SSE_MAX_CLIENTS = 2
EXECUTOR_THREADS = 3


def generate_test_description():
    return create_test_launch(
        demo_nodes=[],          # no discovery needed - this is a config/log test
        fault_manager=False,
        gateway_params={
            'server.http_thread_pool_size': HTTP_THREAD_POOL_SIZE,
            'server.executor_threads': EXECUTOR_THREADS,
            'sse.max_clients': SSE_MAX_CLIENTS,
        },
    )


class TestThreadPoolStarvationGuard(GatewayTestCase):
    """An under-budget pool is flagged, and the executor bound is observable."""

    MIN_EXPECTED_APPS = 0  # skip discovery waiting; the gateway only needs to start

    def test_startup_warns_when_pool_below_budget(self, proc_output):
        """The gateway warns when http_thread_pool_size < max_clients + cold_wait_cap.

        Without this warning the misconfiguration is silent: a burst of SSE
        streams plus cold /data waits can pin every worker. The warning is the
        operator-facing guard, so we assert it is actually emitted.
        """
        proc_output.assertWaitFor(
            'is below sse.max_clients', timeout=15,
        )

    def test_executor_thread_bound_is_applied(self, proc_output):
        """The configured executor_threads value is honoured (and observable).

        The reviewer noted executor_threads was set in tests but never observed.
        The gateway logs the resolved count at startup, so assert it matches the
        value we launched with - a regression that dropped the parameter would
        log a different number.
        """
        proc_output.assertWaitFor(
            f'Main executor bounded to {EXECUTOR_THREADS} threads', timeout=15,
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """All processes exit cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
