# Copyright 2026 ahmedsleem109
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

"""Small helper to report faults to the ros2_medkit fault manager."""

import collections
import threading
import time

from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

SEVERITY = {
    'info': Fault.SEVERITY_INFO,
    'warn': Fault.SEVERITY_WARN,
    'error': Fault.SEVERITY_ERROR,
    'critical': Fault.SEVERITY_CRITICAL,
}


class FaultReporter:
    """Sends FAILED / PASSED events to `/fault_manager/report_fault`.

    A node never blocks on it: reports go into a queue and a timer sends them,
    one at a time, oldest first. A report leaves the queue only when the fault
    manager answers it, and is sent again after `ack_timeout_sec` without an
    answer: the service can look ready for seconds after the fault manager
    died, and a request sent then is lost without an error. So a fault seen
    while the fault manager is starting, restarting or gone is reported late,
    in order, rather than lost.
    """

    def __init__(
        self,
        node,
        service='/fault_manager/report_fault',
        callback_group=None,
        ack_timeout_sec=2.0,
        max_pending=100,
    ):
        self._node = node
        self._client = node.create_client(ReportFault, service, callback_group=callback_group)
        self._ack_timeout = ack_timeout_sec
        self._pending = collections.deque(maxlen=max_pending)
        self._lock = threading.Lock()
        self._in_flight = None  # (request, future, sent at)
        self._waiting_logged = False
        node.create_timer(0.1, self._flush, callback_group=callback_group)

    def failed(self, code, severity, description):
        self._queue(code, ReportFault.Request.EVENT_FAILED, SEVERITY[severity], description)

    def passed(self, code):
        self._queue(code, ReportFault.Request.EVENT_PASSED, Fault.SEVERITY_INFO, '')

    def _queue(self, code, event_type, severity, description):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = event_type
        request.severity = severity
        request.description = description
        request.source_id = self._node.get_fully_qualified_name()
        with self._lock:
            if len(self._pending) == self._pending.maxlen:
                dropped = self._pending[0].fault_code
                self._node.get_logger().warning(
                    f'too many fault reports waiting, dropping {dropped}'
                )
            self._pending.append(request)

    def _flush(self):
        with self._lock:
            for _ in range(20):
                if self._in_flight is not None:
                    request, future, sent = self._in_flight
                    if future.done():
                        if self._pending and self._pending[0] is request:
                            self._pending.popleft()
                        self._in_flight = None
                    elif time.monotonic() - sent > self._ack_timeout:
                        self._client.remove_pending_request(future)
                        self._node.get_logger().warning(
                            f'no answer from the fault manager for {request.fault_code}, '
                            'sending again'
                        )
                        self._in_flight = None
                    else:
                        return
                if not self._pending:
                    self._waiting_logged = False
                    return
                if not self._client.service_is_ready():
                    if not self._waiting_logged:
                        self._node.get_logger().warning(
                            f'fault manager not available, '
                            f'{len(self._pending)} fault report(s) waiting'
                        )
                        self._waiting_logged = True
                    return
                request = self._pending[0]
                self._in_flight = (request, self._client.call_async(request), time.monotonic())
