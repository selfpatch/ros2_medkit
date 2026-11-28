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

"""ROS 2 backend: faults to the ros2_medkit fault manager, chunks as a topic.

Works over any rmw (DDS, Zenoh, ...): it is plain rclpy. Needs
`ros2_medkit_msgs`; the introspection topic also needs
`ros2_medkit_policy_msgs` and is skipped, with one warning, without it.

Fault reports never block the caller. They go into a queue that a timer sends
one at a time, oldest first, so a fault seen while the fault manager is
starting, restarting or gone is reported late, in order:

- a report leaves the queue when the fault manager accepts it;
- a report it refuses (`accepted` false, for example while its storage is
  down) is sent again after `retry_sec`, at most `max_attempts` times, then
  dropped with a warning;
- a report still unanswered when the service disappears is sent again when
  a fault manager is back. One still unanswered after `ack_timeout_sec` with
  the service still there (a hung fault manager) is sent again too: if the
  first copy was only late, the fault manager counts the event twice.

Every problem is logged once, when it starts, and once when it is over.
"""

import collections
import threading
import time

from lerobot_medkit.transport import Transport
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

try:
    from ros2_medkit_policy_msgs.msg import PolicyIntrospection
except ImportError:  # ros2_medkit_policy_msgs not built
    PolicyIntrospection = None

SEVERITY = {
    'info': Fault.SEVERITY_INFO,
    'warn': Fault.SEVERITY_WARN,
    'error': Fault.SEVERITY_ERROR,
    'critical': Fault.SEVERITY_CRITICAL,
}


class _Report:
    __slots__ = ('request', 'attempts', 'not_before')

    def __init__(self, request):
        self.request = request
        self.attempts = 0
        self.not_before = 0.0


class RclpyTransport(Transport):
    """Reports through `node`, or through a node of its own named `node_name`.

    With a node of its own, it also spins it on a background thread. If rclpy
    is not initialized yet, it initializes it without rclpy's signal handlers
    (SIGINT and SIGTERM keep their Python behaviour, so `lerobot-rollout` still
    stops on them) and shuts it down again in `close()`. With a given node, the
    caller spins it, and must not call `flush()` from the thread that spins it.

    Threads: `report_fault`, `clear_fault` and `publish_introspection` run on
    the caller's thread (the control loop); `_flush` runs on the thread that
    spins the node. They share the queue under `_lock`. Nothing is logged
    while `_lock` is held.
    """

    def __init__(
        self,
        node=None,
        node_name='lerobot_medkit',
        source_id='',
        report_service='/fault_manager/report_fault',
        introspection_topic='/policy/introspection',
        ack_timeout_sec=30.0,
        retry_sec=1.0,
        max_attempts=5,
        max_pending=100,
    ):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        if max_pending < 1:
            raise ValueError('max_pending must be at least 1')
        self._rclpy = rclpy
        self._own = node is None
        self._owns_context = False
        self._executor = None
        self._thread = None
        if self._own:
            if not rclpy.ok():
                from rclpy.signals import SignalHandlerOptions

                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
                self._owns_context = True
            node = rclpy.create_node(node_name)
        self._node = node
        self._source_id = source_id or node.get_fully_qualified_name()
        self._client = node.create_client(ReportFault, report_service)
        self._ack_timeout = ack_timeout_sec
        self._retry_sec = retry_sec
        self._max_attempts = max_attempts
        self._pending = collections.deque()
        self._max_pending = max_pending
        self._lock = threading.Lock()
        self._in_flight = None  # (report, future, sent at)
        self._problems = set()  # what is currently wrong, each logged once
        self._dropped = 0
        self._timer = node.create_timer(0.1, self._flush)

        self._introspection_pub = None
        if introspection_topic:
            if PolicyIntrospection is None:
                node.get_logger().warning(
                    'ros2_medkit_policy_msgs not found: no introspection on '
                    f'{introspection_topic}'
                )
            else:
                self._introspection_pub = node.create_publisher(
                    PolicyIntrospection, introspection_topic, 10
                )

        if self._own:
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(node)
            self._thread = threading.Thread(
                target=self._spin, name='lerobot_medkit', daemon=True
            )
            self._thread.start()

    @property
    def node(self):
        return self._node

    @property
    def pending(self):
        """Count reports not yet accepted (or dropped)."""
        with self._lock:
            return len(self._pending)

    @property
    def dropped(self):
        """Count reports given up on: refused too often, or pushed out of a full queue."""
        with self._lock:
            return self._dropped

    def report_fault(self, code, severity, description):
        self._queue(code, ReportFault.Request.EVENT_FAILED, SEVERITY[severity], description)

    def clear_fault(self, code):
        self._queue(code, ReportFault.Request.EVENT_PASSED, Fault.SEVERITY_INFO, '')

    def publish_introspection(self, episode, step, chunk, inference_ms, stamp=None):
        if self._introspection_pub is None:
            return
        msg = PolicyIntrospection()
        if stamp is None:
            msg.header.stamp = self._node.get_clock().now().to_msg()
        else:
            msg.header.stamp.sec = int(stamp)
            msg.header.stamp.nanosec = int((stamp - int(stamp)) * 1e9)
        msg.episode = int(episode)
        msg.step = int(step)
        msg.chunk_length, msg.action_dim = (int(n) for n in chunk.shape)
        msg.chunk = chunk.astype('float32').ravel().tolist()
        msg.inference_ms = float(inference_ms)
        self._introspection_pub.publish(msg)

    def flush(self, timeout_sec=2.0):
        """Wait until every queued report is accepted or dropped. False on timeout."""
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self.pending == 0:
                return True
            time.sleep(0.02)
        return self.pending == 0

    def close(self):
        if self._own:
            self.flush(timeout_sec=1.0)
        with self._lock:
            lost = len(self._pending)
        if lost:
            self._node.get_logger().warning(
                f'closing with {lost} fault report(s) the fault manager has not accepted'
            )
        self._timer.cancel()
        if self._own:
            self._executor.shutdown(timeout_sec=1.0)
            if self._thread is not None:
                self._thread.join(timeout=2.0)
            self._node.destroy_node()
            if self._owns_context:
                self._rclpy.try_shutdown()
        else:
            self._node.destroy_timer(self._timer)
            self._node.destroy_client(self._client)
            if self._introspection_pub is not None:
                self._node.destroy_publisher(self._introspection_pub)

    def _spin(self):
        try:
            self._executor.spin()
        except Exception:  # noqa: B902 - log it unless rclpy is shutting down
            if self._rclpy.ok():
                self._node.get_logger().error('lerobot_medkit: executor stopped', exc_info=True)

    def _queue(self, code, event_type, severity, description):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = event_type
        request.severity = severity
        request.description = description
        request.source_id = self._source_id
        messages = []
        with self._lock:
            report = _Report(request)
            if len(self._pending) >= self._max_pending:
                # The oldest report that is not on its way: the one in flight
                # stays, or its answer would take the next report off the queue.
                in_flight = self._in_flight is not None and self._pending[0] is self._in_flight[0]
                self._dropped += 1
                self._problem(messages, 'overflow', True,
                              f'more than {self._max_pending} fault reports waiting: '
                              'dropping the oldest not yet sent')
                if in_flight and len(self._pending) == 1:
                    report = None  # nothing else to drop: drop the new one
                else:
                    del self._pending[1 if in_flight else 0]
            if report is not None:
                self._pending.append(report)
        self._log(messages)

    def _problem(self, messages, name, active, text):
        """Note a problem starting (active) or ending; log only the changes."""
        if active and name not in self._problems:
            self._problems.add(name)
            messages.append(('warning', text))
        elif not active and name in self._problems:
            self._problems.discard(name)
            messages.append(('info', text))

    def _log(self, messages):
        logger = self._node.get_logger()
        for level, text in messages:
            if level == 'warning':
                logger.warning(text)
            else:
                logger.info(text)

    def _forget(self, future):
        remove = getattr(self._client, 'remove_pending_request', None)
        if remove is not None:
            remove(future)

    def _flush(self):
        messages = []
        with self._lock:
            self._flush_locked(messages)
        self._log(messages)

    def _flush_locked(self, messages):
        now = time.monotonic()
        ready = self._client.service_is_ready()
        for _ in range(20):
            if self._in_flight is not None:
                report, future, sent = self._in_flight
                if future.done():
                    self._in_flight = None
                    self._problem(messages, 'hung', False, 'the fault manager answers again')
                    response = future.result()
                    if response is not None and response.accepted:
                        self._pending.popleft()
                        self._problem(messages, 'refused', False,
                                      'the fault manager accepts reports again')
                        continue
                    report.attempts += 1
                    if report.attempts >= self._max_attempts:
                        self._pending.popleft()
                        self._dropped += 1
                        messages.append(('warning', (
                            f'the fault manager refused {report.request.fault_code} '
                            f'{report.attempts} times: dropped')))
                        continue
                    report.not_before = now + self._retry_sec
                    self._problem(messages, 'refused', True,
                                  f'the fault manager refused {report.request.fault_code}, '
                                  f'sending it again every {self._retry_sec:g} s')
                    return
                elif not ready:
                    # The fault manager went away with the request: send it to the next one.
                    self._forget(future)
                    self._in_flight = None
                elif now - sent > self._ack_timeout:
                    self._forget(future)
                    self._in_flight = None
                    self._problem(messages, 'hung', True,
                                  f'no answer from the fault manager for '
                                  f'{self._ack_timeout:g} s, sending again')
                else:
                    return
            if not self._pending:
                self._problem(messages, 'overflow', False, 'fault report queue drained')
                return
            if not ready:
                self._problem(messages, 'unavailable', True,
                              f'fault manager not available, {len(self._pending)} fault '
                              'report(s) waiting')
                return
            self._problem(messages, 'unavailable', False, 'fault manager available')
            report = self._pending[0]
            if now < report.not_before:
                return
            self._in_flight = (report, self._client.call_async(report.request), now)
