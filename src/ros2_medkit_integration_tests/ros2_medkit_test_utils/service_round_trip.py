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
First round trips to a service server that has just started.

wait_for_service shows that the client found the server. The server can match the
client's reply reader later, and a reply sent before that is lost.
"""

import time

import rclpy

# A request with no answer after this long is sent again.
ANSWER_WAIT_SEC = 1.0


def call_until_answered(node, client, request, timeout_sec, *, spin=True,
                        answer_wait_sec=ANSWER_WAIT_SEC):
    """
    Send <request> until <client> gets an answer and return it; <request> must be repeatable.

    With spin=False another thread spins <node>, and this only waits for the answer.
    """
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        future = client.call_async(request)
        if spin:
            rclpy.spin_until_future_complete(node, future, timeout_sec=answer_wait_sec)
        else:
            settle = time.monotonic() + answer_wait_sec
            while not future.done() and time.monotonic() < settle:
                time.sleep(0.02)
        if future.done():
            return future.result()
        client.remove_pending_request(future)
    raise AssertionError(f'{client.srv_name} did not answer within {timeout_sec} s')


def answer_once_each(node, clients, timeout_sec, *, spin=True):
    """Get one answer on each client. The default request names no fault, so it changes nothing."""
    for client in clients:
        call_until_answered(node, client, client.srv_type.Request(), timeout_sec, spin=spin)
