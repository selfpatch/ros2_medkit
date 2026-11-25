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

"""The procfs plugin serves the process of the gateway's own ROS domain.

A node with the same name and namespace runs in a second domain on the same
host. Marker nodes started after it show that the plugin has scanned /proc
again: once a marker is served, the scan saw every process started before it.
"""

import os
import signal
import subprocess
import time
import unittest

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    get_test_domain_id,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node, DEMO_NODE_REGISTRY

# Longer than the PID cache TTL (10 s by default) plus a discovery refresh.
SERVED_TIMEOUT = 30.0 * get_time_scale()


def generate_test_description():
    procfs_path = os.path.join(
        get_package_prefix('ros2_medkit_linux_introspection'),
        'lib', 'ros2_medkit_linux_introspection', 'libprocfs_introspection.so')
    gateway = create_gateway_node(extra_params={
        'plugins': ['procfs'],
        'plugins.procfs.path': procfs_path,
    })
    return LaunchDescription([gateway, launch_testing.actions.ReadyToTest()]), {
        'gateway_node': gateway}


class DemoNode:
    """A demo node process in one ROS domain."""

    def __init__(self, key, domain):
        executable, name, namespace = DEMO_NODE_REGISTRY[key]
        path = os.path.join(get_package_prefix('ros2_medkit_integration_tests'),
                            'lib', 'ros2_medkit_integration_tests', executable)
        self.proc = subprocess.Popen(
            [path, '--ros-args', '-r', f'__node:={name}', '-r', f'__ns:={namespace}'],
            env=dict(os.environ, ROS_DOMAIN_ID=str(domain)),
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.pid = self.proc.pid

    def stop(self):
        if self.proc.poll() is None:
            self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()


class TestProcfsDomainIsolation(GatewayTestCase):
    """A same-named node in another domain never answers for the gateway's app."""

    nodes = []

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.own = cls.start('temp_sensor', get_test_domain_id())

    @classmethod
    def tearDownClass(cls):
        for node in cls.nodes:
            node.stop()

    @classmethod
    def start(cls, key, domain):
        node = DemoNode(key, domain)
        cls.nodes.append(node)
        return node

    def procfs(self, app_id):
        """Return (status, served PID) of the procfs route of an app."""
        try:
            r = requests.get(f'{self.BASE_URL}/apps/{app_id}/x-medkit-procfs', timeout=5)
        except requests.exceptions.RequestException as e:
            return str(e), None
        return r.status_code, r.json().get('pid') if r.status_code == 200 else None

    def wait_served(self, app_id, pid, *, own_pid=None):
        """
        Poll until the procfs route of app_id serves pid.

        With own_pid, every answer for temp_sensor meanwhile must be that process
        or a 404 while it is looked up: never another process, never a 503.
        """
        deadline = time.monotonic() + SERVED_TIMEOUT
        last = None
        while time.monotonic() < deadline:
            if own_pid is not None:
                status, served = self.procfs('temp_sensor')
                self.assertNotEqual(status, 503, 'temp_sensor answered 503 while its process runs')
                if status == 200:
                    self.assertEqual(served, own_pid, 'temp_sensor served another process')
            last = self.procfs(app_id)
            if last == (200, pid):
                return
            time.sleep(0.2)
        self.fail(f'{app_id} not served as PID {pid} within {SERVED_TIMEOUT} s; last {last}')

    def test_1_own_process_is_served(self):
        """The procfs route serves the node of the gateway's domain."""
        self.wait_served('temp_sensor', self.own.pid)

    def test_2_same_node_in_another_domain_is_not_served(self):
        """A same-named node started later in a second domain is not served."""
        type(self).other = self.start('temp_sensor', get_test_domain_id(1))
        marker = self.start('rpm_sensor', get_test_domain_id())
        self.wait_served('rpm_sensor', marker.pid, own_pid=self.own.pid)
        self.assertEqual(self.procfs('temp_sensor'), (200, self.own.pid))

    def test_3_exit_of_the_other_node_changes_nothing(self):
        """When the node of the second domain exits, the route still serves its own process."""
        self.other.stop()
        marker = self.start('pressure_sensor', get_test_domain_id())
        self.wait_served('pressure_sensor', marker.pid, own_pid=self.own.pid)
        self.assertEqual(self.procfs('temp_sensor'), (200, self.own.pid))

    def test_4_restarted_node_is_served_without_503(self):
        """After the node restarts, its new process is served at once: no 503 on the old PID."""
        self.own.stop()
        type(self).own = self.start('temp_sensor', get_test_domain_id())
        deadline = time.monotonic() + SERVED_TIMEOUT
        last = None
        while time.monotonic() < deadline:
            last = self.procfs('temp_sensor')
            self.assertNotEqual(last[0], 503, 'temp_sensor answered 503 after it restarted')
            if last == (200, self.own.pid):
                return
            time.sleep(0.2)
        self.fail(f'restarted temp_sensor not served within {SERVED_TIMEOUT} s; last {last}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify the gateway exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
