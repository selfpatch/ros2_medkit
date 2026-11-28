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


"""RclpyTransport against a stand-in fault manager service. Needs a ROS 2 setup."""

import os
import signal
import subprocess
import sys
import textwrap
import threading
import time

import numpy as np
import pytest

rclpy = pytest.importorskip('rclpy')
pytest.importorskip('ros2_medkit_msgs')

from lerobot_medkit.rclpy_transport import RclpyTransport  # noqa: E402, I100
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from ros2_medkit_msgs.srv import ReportFault  # noqa: E402

SERVICE = f'/test_{os.getpid()}/report_fault'
FAILED, PASSED = ReportFault.Request.EVENT_FAILED, ReportFault.Request.EVENT_PASSED


class FakeFaultManager:
    """Answers after `delay_sec`; refuses the first `refuse` requests."""

    def __init__(self, delay_sec=0.0, refuse=0):
        self.requests = []
        self.delay_sec = delay_sec
        self.refuse = refuse
        self.node = rclpy.create_node(f'fake_fault_manager_{os.getpid()}')
        self.service = self.node.create_service(ReportFault, SERVICE, self._on_report)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # noqa: B902 - a late answer after stop() (the service is gone)
            pass

    def _on_report(self, request, response):
        self.requests.append(
            (request.fault_code, request.event_type, request.severity, request.source_id)
        )
        if self.delay_sec:
            time.sleep(self.delay_sec)
        response.accepted = len(self.requests) > self.refuse
        return response

    def stop(self):
        self.node.destroy_service(self.service)
        self.executor.shutdown(timeout_sec=0.1)
        self.node.destroy_node()


@pytest.fixture(autouse=True)
def ros(monkeypatch):
    # Keep discovery on this host, so parallel runs and other machines do not meet.
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    rclpy.init()
    yield
    rclpy.try_shutdown()


def wait_for(predicate, timeout=10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def transport(**kwargs):
    kwargs.setdefault('introspection_topic', '')
    return RclpyTransport(node_name=f'lm_test_{os.getpid()}', report_service=SERVICE, **kwargs)


def test_reports_in_order_with_source_id():
    manager = FakeFaultManager()
    t = transport()
    try:
        t.report_fault('A_FAULT', 'error', 'broken')
        t.clear_fault('A_FAULT')
        assert t.flush(timeout_sec=10.0)
        node = f'/lm_test_{os.getpid()}'
        assert manager.requests == [('A_FAULT', FAILED, 2, node), ('A_FAULT', PASSED, 0, node)]
    finally:
        t.close()
        manager.stop()


def test_reports_wait_for_a_late_fault_manager():
    t = transport(source_id='/my_robot')
    try:
        t.report_fault('LATE', 'warn', 'before the manager')
        time.sleep(0.5)
        assert t.pending == 1
        manager = FakeFaultManager()
        try:
            assert t.flush(timeout_sec=10.0)
            assert [(r[0], r[3]) for r in manager.requests] == [('LATE', '/my_robot')]
        finally:
            manager.stop()
    finally:
        t.close()


def test_a_slow_answer_is_not_sent_twice():
    manager = FakeFaultManager(delay_sec=1.5)
    t = transport(ack_timeout_sec=30.0)
    try:
        t.report_fault('SLOW_ANSWER', 'warn', '')
        assert t.flush(timeout_sec=10.0)
        time.sleep(0.5)
        assert len(manager.requests) == 1
    finally:
        t.close()
        manager.stop()


def test_a_refused_report_is_sent_again_then_accepted():
    manager = FakeFaultManager(refuse=2)
    t = transport(retry_sec=0.2)
    try:
        t.report_fault('REFUSED', 'warn', '')
        assert t.flush(timeout_sec=10.0)
        assert [r[0] for r in manager.requests] == ['REFUSED'] * 3
    finally:
        t.close()
        manager.stop()


def test_a_report_refused_too_often_is_dropped_and_the_queue_moves_on():
    manager = FakeFaultManager(refuse=3)
    t = transport(retry_sec=0.1, max_attempts=3)
    try:
        t.report_fault('DROPPED', 'warn', '')
        t.report_fault('NEXT', 'warn', '')
        assert t.flush(timeout_sec=10.0)
        assert [r[0] for r in manager.requests] == ['DROPPED'] * 3 + ['NEXT']
    finally:
        t.close()
        manager.stop()


def test_an_unanswered_report_is_sent_again_after_the_ack_timeout():
    manager = FakeFaultManager(delay_sec=4.0)  # a hung fault manager: answers too late
    t = transport(ack_timeout_sec=1.0)
    try:
        t.report_fault('HUNG', 'warn', '')
        assert wait_for(lambda: len(manager.requests) >= 2, timeout=15.0)
        assert [r[0] for r in manager.requests[:2]] == ['HUNG', 'HUNG']
        # The first request is given up on, not left waiting in the client.
        assert len(t._client._pending_requests) <= 1
    finally:
        t.close()
        manager.stop()


def test_a_report_in_flight_when_the_manager_goes_goes_to_the_next_one():
    first = FakeFaultManager(delay_sec=3.0)
    t = transport(ack_timeout_sec=60.0)
    try:
        t.report_fault('MOVED', 'warn', '')
        assert wait_for(lambda: first.requests)
        first.stop()  # gone with the request unanswered
        time.sleep(1.0)
        second = FakeFaultManager()
        try:
            assert t.flush(timeout_sec=15.0)
            assert [r[0] for r in second.requests] == ['MOVED']
        finally:
            second.stop()
    finally:
        t.close()


def test_close_sends_what_is_queued():
    manager = FakeFaultManager()
    t = transport()
    assert wait_for(t._client.service_is_ready)
    t.report_fault('AT_CLOSE', 'warn', '')
    t.close()  # right away: only the flush in close() can deliver it
    try:
        assert [r[0] for r in manager.requests] == ['AT_CLOSE']
    finally:
        manager.stop()


def test_overflow_keeps_the_report_in_flight_and_counts_the_one_dropped():
    manager = FakeFaultManager(delay_sec=2.0)
    t = transport(max_pending=3)
    try:
        t.report_fault('F0', 'warn', '')
        assert wait_for(lambda: manager.requests)  # F0 is in flight
        for code in ('F1', 'F2', 'F3'):
            t.report_fault(code, 'warn', '')
        assert t.flush(timeout_sec=15.0)
        assert [r[0] for r in manager.requests] == ['F0', 'F2', 'F3']
        assert t.dropped == 1
    finally:
        t.close()
        manager.stop()


def test_overflow_keeps_a_refused_report_in_flight_for_its_retry():
    # The report in flight is refused: it must still be queued to be sent again.
    manager = FakeFaultManager(delay_sec=2.0, refuse=1)
    t = transport(max_pending=3, retry_sec=0.2)
    try:
        t.report_fault('F0', 'warn', '')
        assert wait_for(lambda: manager.requests)
        for code in ('F1', 'F2', 'F3'):
            t.report_fault(code, 'warn', '')
        assert t.flush(timeout_sec=15.0)
        assert [r[0] for r in manager.requests] == ['F0', 'F0', 'F2', 'F3']
    finally:
        t.close()
        manager.stop()


def test_queue_keeps_the_newest_reports():
    t = transport(max_pending=3)
    try:
        for i in range(5):
            t.report_fault(f'F{i}', 'warn', '')
        assert t.pending == 3
        manager = FakeFaultManager()
        try:
            assert t.flush(timeout_sec=10.0)
            assert [r[0] for r in manager.requests] == ['F2', 'F3', 'F4']
            assert t.dropped == 2
        finally:
            manager.stop()
    finally:
        t.close()


def test_introspection_is_published_with_the_given_stamp():
    pytest.importorskip('ros2_medkit_policy_msgs')
    from ros2_medkit_policy_msgs.msg import PolicyIntrospection

    received = []
    topic = f'/lm_test_{os.getpid()}/introspection'
    listener = rclpy.create_node(f'lm_listener_{os.getpid()}')
    listener.create_subscription(PolicyIntrospection, topic, received.append, 10)
    t = transport(introspection_topic=topic)
    try:
        chunk = np.arange(12, dtype=np.float32).reshape(4, 3)

        def spin_until_received():
            t.publish_introspection(2, 50, chunk, 7.5, stamp=1234.5)
            rclpy.spin_once(listener, timeout_sec=0.1)
            return received

        assert wait_for(spin_until_received)
        msg = received[0]
        assert (msg.episode, msg.step, msg.chunk_length, msg.action_dim) == (2, 50, 4, 3)
        assert (msg.header.stamp.sec, msg.header.stamp.nanosec) == (1234, 500000000)
        assert list(msg.chunk) == chunk.ravel().tolist()
    finally:
        t.close()
        listener.destroy_node()


CHILD = textwrap.dedent("""
    import sys, time
    from lerobot_medkit.rclpy_transport import RclpyTransport
    t = RclpyTransport(node_name='lm_signal_child', report_service=sys.argv[1],
                       introspection_topic='')
    print('ready', flush=True)
    try:
        time.sleep(30)
    except KeyboardInterrupt:
        t.close()
        print('closed', flush=True)
""")


def run_child_and_signal(sig):
    child = subprocess.Popen([sys.executable, '-c', CHILD, SERVICE], stdout=subprocess.PIPE,
                             text=True)
    assert child.stdout.readline().strip() == 'ready'
    child.send_signal(sig)
    try:
        out, _ = child.communicate(timeout=5)
    except subprocess.TimeoutExpired:
        child.kill()
        raise AssertionError(f'still running 5 s after {sig!r}')
    return child.returncode, out


def test_sigterm_still_ends_a_process_whose_transport_owns_rclpy():
    returncode, _ = run_child_and_signal(signal.SIGTERM)
    assert returncode == -signal.SIGTERM


def test_sigint_reaches_the_program_as_keyboard_interrupt():
    returncode, out = run_child_and_signal(signal.SIGINT)
    assert returncode == 0
    assert 'closed' in out
