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

"""A gateway stopped before its standing-fault catch-up runs must exit cleanly.

With a plugin loaded and no fault manager, the entity freeze-frame catch-up
waits a fixed 10 s for the fault-events publisher and then waits for the fault
services. That wait is the first graph wait on the fault transport's private
node. A first graph wait after rclcpp has shut down leaves a node
half-registered with rclcpp's graph listener, and ~NodeGraph then aborts the
process. The catch-up must stop on the shut-down context, and the gateway must
exit cleanly.

The test puts SIGINT before the catch-up deadline and holds the teardown open
past it. An operation call in flight at SIGINT gets no answer once the gateway's
executor stops, so it keeps one HTTP worker busy for its full budget, and the
gateway stops its REST server, which waits for that worker, before it stops the
catch-up.
"""

import os
import re
import signal
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

# kEventsMatchTimeout in entity_freeze_frame_capture.cpp. Not a parameter.
CATCHUP_EVENTS_WAIT_SEC = 10.0
# SIGINT must go in at least this long before the catch-up deadline.
SIGINT_HEADROOM_SEC = 2.0
# The teardown must still be held this long after the catch-up deadline.
HOLD_MARGIN_SEC = 2.0
# Budget of the held operation call. The call starts a few seconds after the
# capture, so it ends well after the catch-up deadline plus the margin.
SERVICE_CALL_TIMEOUT_SEC = 15
# Longer than the SIGINT latency, so the answer arrives after the gateway's
# executor stopped and the call runs to its budget. Shorter than the test, so
# the demo node is idle when launch stops it.
SERVICE_DELAY_SEC = 5.0
# Upper bound for the gateway to exit after the held call ends.
EXIT_TIMEOUT_SEC = 60.0

SLOW_APP = 'slow_calibration_service'
CAPTURE_READY = re.compile(r'\[(\d+\.\d+)\] \[[^\]]*\]: EntityFreezeFrameCapture initialized')
CATCHUP_STOPPED = 'Standing-fault freeze-frame catch-up skipped: rclcpp is shutting down'


def generate_test_description():
    procfs_path = os.path.join(
        get_package_prefix('ros2_medkit_linux_introspection'),
        'lib', 'ros2_medkit_linux_introspection', 'libprocfs_introspection.so')
    # A loaded plugin enables the entity freeze-frame capture. No fault manager
    # runs, so the catch-up waits out its full events deadline.
    gateway_node = create_gateway_node(extra_params={
        'plugins': ['procfs'],
        'plugins.procfs.path': procfs_path,
        'service_call_timeout_sec': SERVICE_CALL_TIMEOUT_SEC,
    })
    slow_service = launch_ros.actions.Node(
        package='ros2_medkit_integration_tests',
        executable='demo_slow_calibration_service',
        name=SLOW_APP,
        output='screen',
        parameters=[{'response_delay_sec': SERVICE_DELAY_SEC}],
    )
    return (
        launch.LaunchDescription([
            gateway_node,
            slow_service,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway_node},
    )


class TestShutdownDuringStandingFaultCatchup(GatewayTestCase):
    """SIGINT lands before the catch-up's first fault-service wait."""

    REQUIRED_APPS = {SLOW_APP}
    REQUIRED_OPERATIONS = {f'/apps/{SLOW_APP}': 'calibrate'}

    @staticmethod
    def _gateway_log(proc_output, gateway_node):
        return ''.join(
            output.text.decode(errors='replace') for output in proc_output[gateway_node])

    def _capture_started_at(self, proc_output, gateway_node):
        proc_output.assertWaitFor(
            'EntityFreezeFrameCapture initialized', process=gateway_node, timeout=30)
        match = CAPTURE_READY.search(self._gateway_log(proc_output, gateway_node))
        self.assertIsNotNone(match, 'No timestamped capture start line in the gateway log')
        return float(match.group(1))

    def test_gateway_exits_cleanly(self, proc_output, proc_info, gateway_node):
        catchup_deadline = (
            self._capture_started_at(proc_output, gateway_node) + CATCHUP_EVENTS_WAIT_SEC)

        held_call = {}

        def call_slow_service():
            started = time.time()
            try:
                held_call['status'] = requests.post(
                    f'{self.BASE_URL}/apps/{SLOW_APP}/operations/calibrate/executions',
                    json={},
                    timeout=SERVICE_CALL_TIMEOUT_SEC + EXIT_TIMEOUT_SEC,
                ).status_code
            except requests.exceptions.RequestException as exc:
                held_call['status'] = repr(exc)
            held_call['started'] = started
            held_call['ended'] = time.time()

        caller = threading.Thread(target=call_slow_service, daemon=True)
        caller.start()
        proc_output.assertWaitFor(
            f'Calling service: /{SLOW_APP}/calibrate', process=gateway_node, timeout=30)

        self.assertLess(
            time.time(), catchup_deadline - SIGINT_HEADROOM_SEC,
            'The catch-up deadline is too close; SIGINT would not precede it')
        os.kill(gateway_node.process_details['pid'], signal.SIGINT)

        proc_info.assertWaitForShutdown(
            process=gateway_node, timeout=SERVICE_CALL_TIMEOUT_SEC + EXIT_TIMEOUT_SEC)
        caller.join(timeout=EXIT_TIMEOUT_SEC)

        self.assertIn(
            proc_info[gateway_node].returncode, ALLOWED_EXIT_CODES,
            f'gateway exited with {proc_info[gateway_node].returncode}')
        # The held call is what kept the catch-up running after shutdown.
        self.assertIn('ended', held_call, 'The held operation call never returned')
        self.assertGreater(
            held_call['ended'], catchup_deadline + HOLD_MARGIN_SEC,
            f'The teardown was not held past the catch-up deadline (call {held_call})')
        # Proves the catch-up waited after shutdown, and stopped on the dead
        # context instead of polling it until the teardown reached it.
        self.assertIn(
            CATCHUP_STOPPED, self._gateway_log(proc_output, gateway_node),
            'The catch-up did not stop on the shut-down context')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}')
