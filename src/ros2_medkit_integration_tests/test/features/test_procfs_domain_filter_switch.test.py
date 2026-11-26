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

"""With require_same_ros_domain off, a plugin matches a node's process whatever its ROS domain.

The procfs plugin runs with require_same_ros_domain off and the container plugin
with the default, on one synthetic /proc tree. The environ of temp_sensor's process
names another ROS domain, as for a node that sets its domain in code; the environ
of rpm_sensor's process names the gateway's domain. The container plugin logs at
debug level each process it skips for its domain.
"""

import atexit
import os
import re
import shutil
import tempfile
import time
import unittest

from launch import LaunchDescription
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.cgroup_fixtures import (
    HOST_MOUNTINFO,
    v2_cgroup,
    write_file,
    write_process,
)
from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    get_test_domain_id,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node

PROC_ROOT = tempfile.mkdtemp(prefix='medkit_domain_switch_')
atexit.register(shutil.rmtree, PROC_ROOT, True)

HOST_CGROUP = v2_cgroup('/user.slice/user-1000.slice/session-1.scope')
ANSWER_TIMEOUT = 30.0 * get_time_scale()
OWN_DOMAIN = get_test_domain_id()
# Only written to an environ file: no node runs in this domain.
OTHER_DOMAIN = OWN_DOMAIN + 1
FOREIGN_PID = 5401
OWN_PID = 5402


def _plugin_path(name):
    from ament_index_python.packages import get_package_prefix

    return os.path.join(get_package_prefix('ros2_medkit_linux_introspection'),
                        'lib', 'ros2_medkit_linux_introspection', name)


def write_synthetic_process(pid, fqn, domain):
    """Write a process both plugins can read, started in ROS domain domain."""
    write_process(PROC_ROOT, pid, fqn, HOST_CGROUP, mountinfo=HOST_MOUNTINFO)
    # After the command name: state ppid pgrp session tty tpgid flags minflt cminflt
    # majflt cmajflt utime stime cutime cstime priority nice threads itreal starttime.
    write_file(PROC_ROOT, f'proc/{pid}/stat',
               f'{pid} (demo_node) S 1 {pid} {pid} 0 -1 0 0 0 0 0 7 3 0 0 20 0 2 0 100\n')
    write_file(PROC_ROOT, f'proc/{pid}/status', 'VmRSS:\t1024 kB\nVmSize:\t2048 kB\n')
    write_file(PROC_ROOT, f'proc/{pid}/environ', f'PATH=/usr/bin\0ROS_DOMAIN_ID={domain}\0')


def generate_test_description():
    write_synthetic_process(FOREIGN_PID, '/powertrain/engine/temp_sensor', OTHER_DOMAIN)
    write_synthetic_process(OWN_PID, '/powertrain/engine/rpm_sensor', OWN_DOMAIN)
    gateway = create_gateway_node(
        extra_params={
            'plugins': ['procfs', 'container'],
            'plugins.procfs.path': _plugin_path('libprocfs_introspection.so'),
            'plugins.procfs.proc_root': PROC_ROOT,
            'plugins.procfs.pid_cache_ttl_seconds': 1,
            'plugins.procfs.require_same_ros_domain': False,
            'plugins.container.path': _plugin_path('libcontainer_introspection.so'),
            'plugins.container.proc_root': PROC_ROOT,
            'plugins.container.pid_cache_ttl_seconds': 1,
        },
        ros_arguments=['--log-level', 'ros2_medkit_gateway.container_introspection:=debug'],
    )
    return LaunchDescription([
        gateway,
        *create_demo_nodes(['temp_sensor', 'rpm_sensor']),
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway}


class TestProcfsDomainFilterSwitch(GatewayTestCase):
    """require_same_ros_domain off serves a process of another domain; the default skips it."""

    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor'}

    def answer(self, app_id, route):
        """Return (status, served PID, vendor code) of a plugin route of an app."""
        try:
            r = requests.get(f'{self.BASE_URL}/apps/{app_id}/{route}', timeout=5)
        except requests.exceptions.RequestException as e:
            return str(e), None, None
        try:
            body = r.json()
        except ValueError:
            body = {}
        return r.status_code, body.get('pid'), body.get('vendor_code')

    def wait_answer(self, app_id, route, expected):
        """Poll until the plugin route of an app gives the expected answer."""
        deadline = time.monotonic() + ANSWER_TIMEOUT
        last = None
        while time.monotonic() < deadline:
            last = self.answer(app_id, route)
            if last == expected:
                return
            time.sleep(0.2)
        self.fail(f'{app_id} {route}: expected {expected}, last {last}')

    def test_1_switched_off_plugin_serves_process_of_another_domain(self):
        """The procfs plugin with require_same_ros_domain off serves the other domain's process."""
        self.wait_answer('temp_sensor', 'x-medkit-procfs', (200, FOREIGN_PID, None))

    def test_2_default_plugin_skips_it_and_logs_why(self, proc_output, gateway_node):
        """The container plugin skips that process, and logs the PID, FQN and both domains."""
        # Both processes are in the tree before the gateway starts, so the scan that
        # finds rpm_sensor's process has read temp_sensor's too.
        self.wait_answer('rpm_sensor', 'x-medkit-container',
                         (404, None, 'x-medkit-not-containerized'))
        self.assertEqual(self.answer('temp_sensor', 'x-medkit-container'),
                         (404, None, 'x-medkit-pid-lookup-failed'))
        pattern = (rf'\[ros2_medkit_gateway\.container_introspection\]: PID {FOREIGN_PID} '
                   rf'\(/powertrain/engine/temp_sensor\) is not matched: its ROS_DOMAIN_ID is '
                   rf"{OTHER_DOMAIN}, the gateway's domain is {OWN_DOMAIN}")
        deadline = time.monotonic() + ANSWER_TIMEOUT
        while time.monotonic() < deadline:
            text = ''.join(item.text.decode(errors='replace')
                           for item in proc_output[gateway_node])
            if re.search(pattern, text):
                return
            time.sleep(0.2)
        self.fail(f'no debug line for PID {FOREIGN_PID} in the other domain')


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
