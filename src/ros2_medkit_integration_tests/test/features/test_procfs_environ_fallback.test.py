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

"""A process whose environ cannot be read is matched by name, with one warning per plugin.

The procfs and container plugins read a synthetic /proc tree whose processes have
no environ file. Each plugin owns its PID cache, so each logs the warning once per
PID, however often it scans the tree again. A process written to the tree later
and then found by a plugin shows that the plugin has scanned the tree again.
"""

import atexit
import os
import re
import shutil
import tempfile
import time
import unittest

import launch_testing
import requests

from ros2_medkit_test_utils.cgroup_fixtures import (
    HOST_MOUNTINFO,
    v2_cgroup,
    write_file,
    write_process,
)
from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_time_scale
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

PROC_ROOT = tempfile.mkdtemp(prefix='medkit_environ_fallback_')
atexit.register(shutil.rmtree, PROC_ROOT, True)

HOST_CGROUP = v2_cgroup('/user.slice/user-1000.slice/session-1.scope')
PLUGINS = ('procfs_introspection', 'container_introspection')
FOUND_TIMEOUT = 30.0 * get_time_scale()


def _plugin_path(name):
    from ament_index_python.packages import get_package_prefix

    return os.path.join(get_package_prefix('ros2_medkit_linux_introspection'),
                        'lib', 'ros2_medkit_linux_introspection', name)


def write_synthetic_process(pid, fqn):
    """Write a process both plugins can read, with no environ file."""
    write_process(PROC_ROOT, pid, fqn, HOST_CGROUP, mountinfo=HOST_MOUNTINFO)
    # After the command name: state ppid pgrp session tty tpgid flags minflt cminflt
    # majflt cmajflt utime stime cutime cstime priority nice threads itreal starttime.
    write_file(PROC_ROOT, f'proc/{pid}/stat',
               f'{pid} (demo_node) S 1 {pid} {pid} 0 -1 0 0 0 0 0 7 3 0 0 20 0 2 0 100\n')
    write_file(PROC_ROOT, f'proc/{pid}/status', 'VmRSS:\t1024 kB\nVmSize:\t2048 kB\n')


def generate_test_description():
    write_synthetic_process(5101, '/powertrain/engine/temp_sensor')
    return create_test_launch(
        demo_nodes=['temp_sensor', 'rpm_sensor', 'pressure_sensor'],
        fault_manager=False,
        gateway_params={
            'plugins': ['procfs', 'container'],
            'plugins.procfs.path': _plugin_path('libprocfs_introspection.so'),
            'plugins.procfs.proc_root': PROC_ROOT,
            'plugins.procfs.pid_cache_ttl_seconds': 1,
            'plugins.container.path': _plugin_path('libcontainer_introspection.so'),
            'plugins.container.proc_root': PROC_ROOT,
            'plugins.container.pid_cache_ttl_seconds': 1,
        },
    )


class TestProcfsEnvironFallback(GatewayTestCase):
    """Name match and one warning per plugin for a process without a readable environ."""

    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor', 'pressure_sensor'}

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

    def wait_found(self, app_id, pid):
        """Poll until both plugins have found pid for the app in a scan of their cache."""
        deadline = time.monotonic() + FOUND_TIMEOUT
        procfs = container = None
        while time.monotonic() < deadline:
            procfs = self.answer(app_id, 'x-medkit-procfs')
            # The synthetic cgroup is a host one: once the PID is found, not containerized.
            container = self.answer(app_id, 'x-medkit-container')
            if procfs[:2] == (200, pid) and container == (404, None, 'x-medkit-not-containerized'):
                return
            time.sleep(0.2)
        self.fail(f'{app_id} not found as PID {pid}: procfs {procfs}, container {container}')

    @staticmethod
    def warnings(proc_output, gateway_node, pid):
        """Return the number of environ warnings for pid, per plugin."""
        text = ''.join(item.text.decode(errors='replace') for item in proc_output[gateway_node])
        return {
            plugin: len(re.findall(
                rf'\[ros2_medkit_gateway\.{plugin}\]: Cannot read the environment of PID {pid} ',
                text))
            for plugin in PLUGINS
        }

    def test_1_process_without_environ_is_served_by_name(self):
        """The process is matched on its command line alone and served."""
        self.wait_found('temp_sensor', 5101)

    def test_2_warning_is_logged_once_per_plugin(self, proc_output, gateway_node):
        """Each plugin warns once per PID across later scans of the same process."""
        # Each process is written after the previous one is found, so it is found in a
        # later scan, and every scan reads 5101 again.
        write_synthetic_process(5102, '/powertrain/engine/rpm_sensor')
        self.wait_found('rpm_sensor', 5102)
        write_synthetic_process(5103, '/chassis/brakes/pressure_sensor')
        self.wait_found('pressure_sensor', 5103)
        # The warnings of the last scan reach the output after those of the earlier ones.
        deadline = time.monotonic() + FOUND_TIMEOUT
        while time.monotonic() < deadline:
            if all(self.warnings(proc_output, gateway_node, 5103).values()):
                break
            time.sleep(0.2)
        else:
            self.fail(f'no environ warning for PID 5103: '
                      f'{self.warnings(proc_output, gateway_node, 5103)}')
        once = dict.fromkeys(PLUGINS, 1)
        self.assertEqual(self.warnings(proc_output, gateway_node, 5101), once)
        self.assertEqual(self.warnings(proc_output, gateway_node, 5102), once)


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
