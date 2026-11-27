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

"""Harness for the rosbag retention tests: a real fault_manager_node and a ROS client for it.

The fault manager runs as its own process, started with debug logging, and its output is read on a
thread. A test waits on output lines or on polled service state, each with a deadline.
"""

import contextlib
import ctypes
import os
import re
import shutil
import signal
import sqlite3
import subprocess
import tempfile
import threading
import time
import unittest

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ros2_medkit_msgs.srv import ClearFault, GetRosbag, ListRosbags, ReportFault
from std_msgs.msg import UInt8MultiArray

MIB = 1024 * 1024
KIB = 1024
PAYLOAD_TOPIC = '/pin/payload'
SOURCE_ID = '/pin_node'
FM_NODE = 'fault_manager'
# prctl option: the kernel sends the child this signal when the test process dies.
PR_SET_PDEATHSIG = 1


def time_scale():
    """Return MEDKIT_TEST_TIME_SCALE, or 1.0 when unset, unparseable or below 1."""
    try:
        return max(1.0, float(os.environ.get('MEDKIT_TEST_TIME_SCALE', '1')))
    except ValueError:
        return 1.0


SCALE = time_scale()
STARTUP_TIMEOUT_SEC = 60.0 * SCALE
EVENT_TIMEOUT_SEC = 60.0 * SCALE
EXIT_TIMEOUT_SEC = 60.0 * SCALE
SERVICE_TIMEOUT_SEC = 30.0 * SCALE


def _die_with_parent():
    ctypes.CDLL('libc.so.6', use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGKILL)


def _fm_binary():
    return os.path.join(get_package_prefix('ros2_medkit_fault_manager'), 'lib',
                        'ros2_medkit_fault_manager', 'fault_manager_node')


def _ros_value(value):
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)


class FaultManager:
    """A fault_manager_node process whose output lines are read on a thread."""

    def __init__(self, params):
        env = os.environ.copy()
        env['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'
        env['ROS_LOCALHOST_ONLY'] = '1'
        try:
            from ros2_medkit_test_utils.coverage import get_coverage_env
            env.update(get_coverage_env())
        except ImportError:
            pass
        args = [_fm_binary(), '--ros-args', '--log-level', f'{FM_NODE}:=debug']
        for key, value in params.items():
            args += ['-p', f'{key}:={_ros_value(value)}']
        self.proc = subprocess.Popen(
            args, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env,
            preexec_fn=_die_with_parent)
        self.lines = []
        self._eof = False
        self._seen = threading.Condition()
        self._reader = threading.Thread(target=self._read, daemon=True)
        self._reader.start()

    def _read(self):
        for line in self.proc.stdout:
            with self._seen:
                self.lines.append(line)
                self._seen.notify_all()
        with self._seen:
            self._eof = True
            self._seen.notify_all()

    def mark(self):
        """Return the index after the last line read, to search from later."""
        with self._seen:
            return len(self.lines)

    def wait_line(self, pattern, since=0, timeout=None):
        """Return the match of the first line from index `since` on, or None at the deadline."""
        regex = re.compile(pattern)
        deadline = time.monotonic() + (EVENT_TIMEOUT_SEC if timeout is None else timeout)
        index = since
        with self._seen:
            while True:
                while index < len(self.lines):
                    match = regex.search(self.lines[index])
                    index += 1
                    if match:
                        return match
                remaining = deadline - time.monotonic()
                if remaining <= 0 or self._eof:
                    return None
                self._seen.wait(remaining)

    def count_lines(self, pattern, since=0):
        regex = re.compile(pattern)
        with self._seen:
            return sum(1 for line in self.lines[since:] if regex.search(line))

    def output(self):
        with self._seen:
            return ''.join(self.lines)

    def wait_exit(self, timeout=None):
        return self.proc.wait(timeout=EXIT_TIMEOUT_SEC if timeout is None else timeout)

    def terminate(self):
        """SIGTERM the process and return its exit code."""
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGTERM)
        code = self.wait_exit()
        self._reader.join(timeout=10)
        return code

    def kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait()
        self._reader.join(timeout=10)


@contextlib.contextmanager
def running_fault_manager(params, wait_for_subscription=True):
    """Start a fault manager, yield it once ready, end it with SIGTERM and assert exit code 0.

    A sanitizer report inside the process shows as a non-zero exit code. The body may end the
    process itself (a SIGTERM inside it); the exit code is checked either way.
    """
    fm = FaultManager(params)
    try:
        if fm.wait_line(r'FaultManager node started', timeout=STARTUP_TIMEOUT_SEC) is None:
            raise AssertionError('the fault manager never started:\n' + fm.output()[-4000:])
        if wait_for_subscription and params.get('snapshots.rosbag.enabled'):
            subscribed = f"Subscribed to '{PAYLOAD_TOPIC}'"
            if fm.wait_line(subscribed, timeout=STARTUP_TIMEOUT_SEC) is None:
                raise AssertionError('rosbag capture never subscribed:\n' + fm.output()[-4000:])
        yield fm
    except BaseException:
        fm.kill()
        raise
    code = fm.terminate()
    if code != 0:
        raise AssertionError(f'the fault manager exited with {code}:\n' + fm.output()[-4000:])


class FaultManagerClient:
    """The ROS side of a test: a payload publisher and clients of the fault manager services."""

    def __init__(self, node_name):
        self.node = Node(node_name)
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                         depth=500)
        self.publisher = self.node.create_publisher(UInt8MultiArray, PAYLOAD_TOPIC, qos)
        self._clients = {}

    def reset_clients(self):
        """Drop the service clients so the next calls match a freshly started fault manager."""
        for client in self._clients.values():
            self.node.destroy_client(client)
        self._clients = {}

    def destroy(self):
        self.node.destroy_node()

    def _client(self, srv_type, name):
        key = (srv_type, name)
        if key not in self._clients:
            self._clients[key] = self.node.create_client(srv_type, f'/{FM_NODE}/{name}')
        return self._clients[key]

    def wait_for_services(self):
        for srv_type, name in ((ReportFault, 'report_fault'), (ClearFault, 'clear_fault'),
                               (GetRosbag, 'get_rosbag'), (ListRosbags, 'list_rosbags')):
            client = self._client(srv_type, name)
            if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
                raise AssertionError(f'service {name} never appeared')

    def _call(self, srv_type, name, request):
        client = self._client(srv_type, name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SERVICE_TIMEOUT_SEC)
        if future.result() is None:
            raise AssertionError(f'{name} did not answer')
        return future.result()

    def report(self, code):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = 2
        request.description = 'pin'
        request.source_id = SOURCE_ID
        return self._call(ReportFault, 'report_fault', request)

    def clear(self, code):
        request = ClearFault.Request()
        request.fault_code = code
        return self._call(ClearFault, 'clear_fault', request)

    def get_rosbag(self, code='', recording_id=''):
        request = GetRosbag.Request()
        request.fault_code = code
        request.recording_id = recording_id
        return self._call(GetRosbag, 'get_rosbag', request)

    def list_rosbags(self, entity=SOURCE_ID):
        request = ListRosbags.Request()
        request.entity_fqn = entity
        return self._call(ListRosbags, 'list_rosbags', request)

    def wait_for_payload_subscriber(self):
        """Block until the fault manager's subscription on the payload topic is matched."""
        deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
        while self.publisher.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise AssertionError('the fault manager never subscribed to ' + PAYLOAD_TOPIC)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def publish_bytes(self, count, size):
        """Publish `count` messages of `size` random bytes and wait for every reader to ack."""
        for _ in range(count):
            msg = UInt8MultiArray()
            msg.data = list(os.urandom(size))
            self.publisher.publish(msg)
        wait = getattr(self.publisher, 'wait_for_all_acked', None)
        if wait is not None:
            wait(Duration(seconds=SERVICE_TIMEOUT_SEC))


class Ticker:
    """Publishes small messages on the payload topic from a thread until stopped."""

    def __init__(self, client, period_sec=0.01, size=1024):
        self._client = client
        self._period = period_sec
        self._size = size
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        msg = UInt8MultiArray()
        msg.data = list(os.urandom(self._size))
        while not self._stop.wait(self._period):
            self._client.publisher.publish(msg)

    def __enter__(self):
        self._thread.start()
        return self

    def __exit__(self, *exc):
        self._stop.set()
        self._thread.join(timeout=10)


def poll(predicate, timeout=None, interval=0.05):
    """Return True once `predicate()` is true, False at the deadline."""
    deadline = time.monotonic() + (EVENT_TIMEOUT_SEC if timeout is None else timeout)
    waiter = threading.Event()
    while True:
        if predicate():
            return True
        if time.monotonic() >= deadline:
            return False
        waiter.wait(interval)


def dir_size(path):
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            total += os.path.getsize(os.path.join(root, name))
    return total


def bag_dirs(storage_path):
    """Return the recording directories under the storage path, sorted by name."""
    if not os.path.isdir(storage_path):
        return []
    return sorted(name for name in os.listdir(storage_path)
                  if os.path.isdir(os.path.join(storage_path, name)))


def storage_bytes(storage_path):
    return sum(dir_size(os.path.join(storage_path, name)) for name in bag_dirs(storage_path))


def sqlite_rows(db_path):
    """Return (fault_code, file_path, size_bytes, created_at_ns) of every rosbag_files row."""
    conn = sqlite3.connect(db_path, timeout=30)
    try:
        return conn.execute(
            'SELECT fault_code, file_path, size_bytes, created_at_ns FROM rosbag_files '
            'ORDER BY id').fetchall()
    finally:
        conn.close()


def sqlite_exec(db_path, sql, *args):
    conn = sqlite3.connect(db_path, timeout=30)
    try:
        conn.execute(sql, args)
        conn.commit()
    finally:
        conn.close()


def base_params(db_path, storage_path, **overrides):
    """Parameters for a fault manager with rosbag capture on a SQLite file."""
    params = {
        'storage_type': 'sqlite',
        'database_path': db_path,
        'confirmation_threshold': -1,
        'snapshots.recapture_cooldown_sec': 0.0,
        'snapshots.rosbag.enabled': True,
        'snapshots.rosbag.topics': PAYLOAD_TOPIC,
        'snapshots.rosbag.format': 'mcap',
        'snapshots.rosbag.storage_path': storage_path,
        'snapshots.rosbag.duration_sec': 60.0,
        'snapshots.rosbag.duration_after_sec': 0.0,
        'snapshots.rosbag.lazy_start': False,
        'snapshots.rosbag.max_bag_size_mb': 50,
        'snapshots.rosbag.max_total_storage_mb': 500,
        'snapshots.rosbag.max_bags_per_fault': 1,
        'snapshots.rosbag.auto_cleanup': True,
    }
    params.update(overrides)
    return params


MESSAGE_SIZE = 128 * KIB
# Payload bytes plus the mcap and metadata overhead a recording may carry.
SIZE_SLACK = 0.1 * MIB


class RetentionCase(unittest.TestCase):
    """Base of the retention tests: ROS context, scratch directories and recording helpers."""

    @classmethod
    def setUpClass(cls):
        os.environ['ROS_LOCALHOST_ONLY'] = '1'
        rclpy.init()
        cls.client = FaultManagerClient('rosbag_pin_client')

    @classmethod
    def tearDownClass(cls):
        cls.client.destroy()
        rclpy.shutdown()

    def setUp(self):
        self.new_scratch()

    def new_scratch(self):
        """Start a fresh database file and bag directory, removed at the end of the test."""
        tmp = tempfile.mkdtemp(prefix='rosbag_pin_')
        self.addCleanup(shutil.rmtree, tmp, True)
        self.storage = os.path.join(tmp, 'bags')
        self.db = os.path.join(tmp, 'faults.db')

    def storage_params(self):
        return {'storage_type': 'sqlite', 'database_path': self.db}

    def stored_rows(self):
        """Return (fault_code, file_path) of every stored rosbag row."""
        return [(code, path) for code, path, _, _ in sqlite_rows(self.db)]

    def row_paths(self, code):
        return [path for row_code, path in self.stored_rows() if row_code == code]

    @contextlib.contextmanager
    def fm_run(self, **overrides):
        """Start a fault manager on this case's storage and yield it ready for faults."""
        params = base_params(self.db, self.storage, **overrides)
        params.update(self.storage_params())
        self.client.reset_clients()
        with running_fault_manager(params) as fm:
            self.client.wait_for_services()
            if params.get('snapshots.rosbag.enabled'):
                self.client.wait_for_payload_subscriber()
            yield fm

    def check_size(self, path, mib):
        payload = int(mib * MIB)
        actual = dir_size(path)
        self.assertTrue(payload <= actual <= payload + SIZE_SLACK,
                        f'{path} holds {actual} bytes, expected {payload}'
                        f' plus at most {SIZE_SLACK}')

    def record(self, fm, code, mib, burst=()):
        """Publish `mib` MiB, confirm `code` and `burst`, and return the bag path.

        The `burst` faults confirm inside the post-fault window of `code`.
        """
        mark = fm.mark()
        self.client.publish_bytes(int(mib * MIB) // MESSAGE_SIZE, MESSAGE_SIZE)
        self.assertTrue(self.client.report(code).accepted)
        if burst:
            self.assertIsNotNone(
                fm.wait_line(r'Recording [\d.]+s more after fault confirmation', since=mark),
                'the post-fault window never opened:\n' + fm.output()[-3000:])
            for other in burst:
                self.assertTrue(self.client.report(other).accepted)
                self.assertIsNotNone(
                    fm.wait_line(f"Fault '{other}' confirmed during post-fault recording",
                                 since=mark),
                    f'{other} did not land inside the window:\n' + fm.output()[-3000:])
        done = fm.wait_line(rf'Bag file (?:created|completed): (\S*fault_{code}_\d+)', since=mark)
        self.assertIsNotNone(done, f'no recording for {code}:\n' + fm.output()[-3000:])
        path = done.group(1)
        self.assertTrue(os.path.isdir(path), f'{path} is not on disk')
        self.check_size(path, mib)
        return path

    def assert_recording_present(self, code, path):
        self.assertTrue(os.path.isdir(path), f'{path} is gone')
        self.assertIn(path, self.row_paths(code), f'{code} has no row for {path}')
        served = self.client.get_rosbag(code)
        self.assertTrue(served.success, f'GetRosbag({code}) failed: {served.error_message}')

    def assert_recording_gone(self, code, path):
        self.assertFalse(os.path.exists(path), f'{path} is still on disk')
        self.assertNotIn(path, self.row_paths(code), f'{code} still has a row for {path}')

    def check_quota_evicts_oldest_first(self):
        """Five recordings fit a quota for three: the two oldest go, by time and not by name."""
        quota_mib = 7
        order = ['FAULT_E', 'FAULT_A', 'FAULT_D', 'FAULT_B', 'FAULT_C']
        with self.fm_run(**{'snapshots.rosbag.max_bags_per_fault': 0,
                            'snapshots.rosbag.max_total_storage_mb': quota_mib}) as fm:
            paths = {code: self.record(fm, code, 2) for code in order}
            for code in order[:2]:
                self.assert_recording_gone(code, paths[code])
                self.assertFalse(self.client.get_rosbag(code).success)
            for code in order[2:]:
                self.assert_recording_present(code, paths[code])
            self.assertLessEqual(storage_bytes(self.storage), quota_mib * MIB)
            self.assertEqual(sorted(bag_dirs(self.storage)),
                             sorted(os.path.basename(paths[code]) for code in order[2:]))
