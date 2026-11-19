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
First round trips to a fault manager that has just started.

wait_for_service and count_publishers show that the test found the server. The server
can match the test's reply reader or event subscription later, and a reply or an
event sent before that is lost.
"""

import time

import rclpy
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

# A request with no answer after this long is sent again.
ANSWER_WAIT_SEC = 1.0


def call_until_answered(node, client, request, timeout_sec, answer_wait_sec=ANSWER_WAIT_SEC):
    """Send <request> until <client> gets an answer and return it; <request> must be repeatable."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=answer_wait_sec)
        if future.done():
            return future.result()
        client.remove_pending_request(future)
    raise AssertionError(f'{client.srv_name} did not answer within {timeout_sec} s')


def answer_once_each(node, clients, timeout_sec):
    """Get one answer on each client. The default request names no fault, so it changes nothing."""
    for client in clients:
        call_until_answered(node, client, client.srv_type.Request(), timeout_sec)


def wait_for_event_route(node, report_client, events, prefix, timeout_sec):
    """
    Report a new CRITICAL code until its event reaches <events>.

    Each code confirms once and publishes one event. Events arrive in order, so a caller
    that counts every event can empty <events> afterwards: no earlier probe's event can
    come after the one seen.
    """
    deadline = time.monotonic() + timeout_sec
    index = 0
    while time.monotonic() < deadline:
        index += 1
        request = ReportFault.Request()
        request.fault_code = f'{prefix}{index}'
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_CRITICAL
        request.description = 'event route probe'
        request.source_id = '/event_route_probe'
        # Sent once: a second report of the code would publish a second event.
        client_future = report_client.call_async(request)
        settle = time.monotonic() + ANSWER_WAIT_SEC
        while time.monotonic() < settle:
            if any(e.fault.fault_code == request.fault_code for e in events):
                report_client.remove_pending_request(client_future)
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        report_client.remove_pending_request(client_future)
    raise AssertionError(f'no fault event reached the test within {timeout_sec} s')
