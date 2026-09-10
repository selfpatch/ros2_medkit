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
Integration tests for graph-event-driven discovery refresh.

The gateway runs ``refresh_cache()`` whenever rclcpp signals a graph
change (``get_graph_event()->check_and_clear()``), polled every 100 ms.
A long backstop timer (``refresh_interval_ms``, default 30 s) provides
liveness in case a graph event is ever missed.

Scope of these tests:

1. Two demo nodes are launched at startup; both must appear in ``/apps``.
2. A new demo node is spawned mid-test; the gateway must expose it in
   ``/apps`` quickly - the load-bearing assertion that proves the
   graph-event refactor is working.

The gateway runs with ``refresh_interval_ms`` at its maximum (60 s), and the
spawn case checks that it finished inside the window before the first sweep,
so the detection it observes must come from the graph-event poll rather than
the safety backstop.

Kill-detection latency is intentionally not asserted: rclcpp's
graph-event for a departing participant fires only after the executor
sees the DDS leave announcement, which depends on RMW liveliness timers
and graceful-shutdown timing. That cleanup is exercised by the existing
discovery integration tests; here we focus on the path the refactor
actually changed (the refresh trigger).
"""

import os
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    DEFAULT_DOMAIN_ID,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
    DEMO_NODE_REGISTRY,
    get_coverage_env,
)


# The gateway's maximum accepted backstop interval, and it has to be the
# maximum. The backstop timer is created during gateway initialisation and its
# phase relative to a mid-run spawn is arbitrary, so a case that only measures
# elapsed time cannot tell a graph-event refresh from a backstop sweep that
# happened to land nearby. Pushing the FIRST sweep as far out as the parameter
# allows lets the spawn case finish inside a window where no sweep has run yet;
# PRE_BACKSTOP_BUDGET_SEC below is what keeps it inside that window.
BACKSTOP_INTERVAL_MS = 60000

# Demo nodes launched at startup.
INITIAL_NODES = ['temp_sensor', 'rpm_sensor']

# Demo node spawned mid-test to exercise the new-node path.
LATE_NODE_KEY = 'pressure_sensor'

# Detection budgets.
#
# Spawn detection is bounded by:
#   process exec + rclcpp init + DDS announce + 100 ms poll + refresh_cache.
# The poll sits above the latency bound below, so a detection that arrives late
# reports the time it took instead of a bare timeout. Every term in that sum is
# instrumented work, so the budget scales with the sanitizer factor.
SPAWN_DETECTION_TIMEOUT = 15.0 * get_time_scale()

# How long after the launch description was generated the spawn case may still
# measure. launch_testing builds the description in this process and only then
# starts the launch service that forks the gateway, and the gateway arms its
# backstop timer during its own initialisation, so the timer is armed strictly
# after that anchor and its first sweep cannot come earlier than
# BACKSTOP_INTERVAL_MS past it: a wall timer fires late, never early. A
# detection measured inside this budget is therefore one no sweep could have
# served, which is what makes the bound below a statement about the graph-event
# path. The gateway's own startup only widens the window; 50 keeps the
# assertion 10 s clear of the 60 s edge.
#
# This budget deliberately does NOT scale with MEDKIT_TEST_TIME_SCALE. It is
# bounded by the gateway's own backstop interval, which the scale factor does
# not stretch; scaling the budget would let the measurement drift past the
# first sweep, and the assertion would stop being about the graph-event path.
PRE_BACKSTOP_BUDGET_SEC = 50.0

# Taken when generate_test_description() runs, before launch forks the gateway.
# See PRE_BACKSTOP_BUDGET_SEC for why this is the reference point.
_LAUNCH_DESCRIBED_AT = None

# The latency of the graph-event path itself, measured from process spawn. It
# cannot be sub-second: the gateway coalesces graph events behind
# discovery.refresh_debounce_ms, 1000 ms by default, and a spawn that arrives
# mid-window waits for the next one, so detection lands on a multiple of the
# debounce. Measured on a developer machine with the default settings, the
# spread is roughly 1 s to 3.6 s. The budget scales with the sanitizer factor.
GRAPH_EVENT_MAX_LATENCY_SEC = 10.0 * get_time_scale()

# Initial discovery shares the budget with full gateway startup, so it scales
# with the sanitizer factor.
INITIAL_DETECTION_TIMEOUT = 30.0 * get_time_scale()


def generate_test_description():
    global _LAUNCH_DESCRIBED_AT
    _LAUNCH_DESCRIBED_AT = time.monotonic()

    gateway_node = create_gateway_node(
        extra_params={
            'refresh_interval_ms': BACKSTOP_INTERVAL_MS,
        },
    )

    initial_demos = create_demo_nodes(
        nodes=INITIAL_NODES,
        lidar_faulty=False,
    )

    delayed = TimerAction(
        period=2.0,
        actions=initial_demos,
    )

    return (
        LaunchDescription([
            gateway_node,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


def _resolve_demo_executable(name):
    """Resolve a demo executable to its installed absolute path.

    Avoids the cost of going through ``ros2 run``, which spins up a
    Python interpreter and ament_index lookup before exec'ing the
    binary - that overhead would dominate the spawn-detection budget.
    """
    pkg = 'ros2_medkit_integration_tests'
    prefix = get_package_prefix(pkg)
    candidate = os.path.join(prefix, 'lib', pkg, name)
    if not os.path.isfile(candidate):
        raise FileNotFoundError(f'demo executable not found: {candidate}')
    return candidate


def _terminate_process(proc):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGTERM)
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


# @verifies REQ_INTEROP_003
class TestGraphEventDiscovery(GatewayTestCase):
    """Verify graph-event-driven discovery refresh latency."""

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls._extra_proc = None

    @classmethod
    def tearDownClass(cls):
        # Make sure we never leak a stray demo process if a test failed
        # before its own cleanup ran.
        _terminate_process(cls._extra_proc)
        cls._extra_proc = None
        super().tearDownClass()

    @staticmethod
    def _spawn_demo_process(key):
        executable, ros_name, namespace = DEMO_NODE_REGISTRY[key]

        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = str(DEFAULT_DOMAIN_ID)
        env.update(get_coverage_env())

        binary = _resolve_demo_executable(executable)

        return subprocess.Popen(
            [
                binary,
                '--ros-args',
                '-r', f'__ns:={namespace}',
                '-r', f'__node:={ros_name}',
            ],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def test_initial_discovery_picks_up_startup_nodes(self):
        """Both startup demo nodes must be visible in /apps."""
        for key in INITIAL_NODES:
            _, ros_name, _ = DEMO_NODE_REGISTRY[key]
            data = self.poll_endpoint_until(
                '/apps',
                lambda d, n=ros_name: d if any(
                    n in app.get('id', '')
                    for app in d.get('items', [])
                ) else None,
                timeout=INITIAL_DETECTION_TIMEOUT,
                interval=0.1,
            )
            app_ids = [app.get('id', '') for app in data.get('items', [])]
            self.assertTrue(
                any(ros_name in app_id for app_id in app_ids),
                f'Expected {ros_name} in /apps, got {app_ids}',
            )

    def test_new_node_detected_via_graph_event(self):
        """Spawning a node mid-run must propagate within the spawn budget.

        The case runs before the first backstop sweep and checks that it did,
        so the refresh it observes can only have come from a graph event. The
        measured time starts at process spawn, so it also carries the node's
        own startup and the gateway's event debounce.
        """
        # Make sure the initial graph is fully settled before spawning.
        for key in INITIAL_NODES:
            _, ros_name, _ = DEMO_NODE_REGISTRY[key]
            self.poll_endpoint_until(
                '/apps',
                lambda d, n=ros_name: d if any(
                    n in app.get('id', '')
                    for app in d.get('items', [])
                ) else None,
                timeout=INITIAL_DETECTION_TIMEOUT,
                interval=0.1,
            )

        _, late_ros_name, _ = DEMO_NODE_REGISTRY[LATE_NODE_KEY]

        type(self)._extra_proc = self._spawn_demo_process(LATE_NODE_KEY)
        spawn_time = time.monotonic()

        try:
            data = self.poll_endpoint_until(
                '/apps',
                lambda d: d if any(
                    late_ros_name in app.get('id', '')
                    for app in d.get('items', [])
                ) else None,
                timeout=SPAWN_DETECTION_TIMEOUT,
                interval=0.1,
            )
            elapsed = time.monotonic() - spawn_time
            # Establish what was measured before bounding it. Past this budget
            # a backstop sweep could have served the detection, and then the
            # bound below would be reporting on the wrong mechanism.
            since_launch = time.monotonic() - _LAUNCH_DESCRIBED_AT
            self.assertLess(
                since_launch, PRE_BACKSTOP_BUDGET_SEC,
                f'detection landed {since_launch:.3f}s after the launch was described, '
                f'past the {PRE_BACKSTOP_BUDGET_SEC}s window in which no backstop '
                f'sweep can have run ({BACKSTOP_INTERVAL_MS}ms backstop), so this '
                f'run cannot say what triggered the refresh',
            )
            self.assertLess(
                elapsed, GRAPH_EVENT_MAX_LATENCY_SEC,
                f'Spawn detection took {elapsed:.3f}s - expected the '
                f'graph-event poll to serve it, not the backstop sweep '
                f'({BACKSTOP_INTERVAL_MS}ms backstop configured)',
            )
            app_ids = [app.get('id', '') for app in data.get('items', [])]
            self.assertTrue(
                any(late_ros_name in app_id for app_id in app_ids),
                f'Expected {late_ros_name} in /apps, got {app_ids}',
            )
        finally:
            _terminate_process(type(self)._extra_proc)
            type(self)._extra_proc = None


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES
        )
