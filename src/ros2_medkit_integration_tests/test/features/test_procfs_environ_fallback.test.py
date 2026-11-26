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

The procfs and container plugins read a synthetic /proc tree. A process with no
environ file is matched on its command line alone. Each plugin owns its PID cache
and warns once per PID it serves that way, however often it scans the tree again,
and never for a PID it does not serve. A process of the gateway's ROS domain keeps
its app against a same-named process that cannot be checked. A process written to
the tree later and then found by a plugin shows that the plugin has scanned the
tree again.
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
from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    get_test_domain_id,
    get_time_scale,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

PROC_ROOT = tempfile.mkdtemp(prefix='medkit_environ_fallback_')
atexit.register(shutil.rmtree, PROC_ROOT, True)

HOST_CGROUP = v2_cgroup('/user.slice/user-1000.slice/session-1.scope')
PLUGINS = ('procfs_introspection', 'container_introspection')
FOUND_TIMEOUT = 30.0 * get_time_scale()
# A ROS process that no app maps to.
GHOST_PID = 5104
# Per app: the PID of a process of the gateway's domain, then the PID of a
# same-named process whose environ cannot be read.
PAIRS = (('status_sensor', 5201, 5202), ('actuator', 5301, 5302))
APPS = ['temp_sensor', 'rpm_sensor', 'pressure_sensor', 'controller', 'status_sensor', 'actuator']


def _plugin_path(name):
    from ament_index_python.packages import get_package_prefix

    return os.path.join(get_package_prefix('ros2_medkit_linux_introspection'),
                        'lib', 'ros2_medkit_linux_introspection', name)


def write_synthetic_process(pid, fqn, domain=None):
    """Write a process both plugins can read. Without a domain it has no environ file."""
    write_process(PROC_ROOT, pid, fqn, HOST_CGROUP, mountinfo=HOST_MOUNTINFO)
    # After the command name: state ppid pgrp session tty tpgid flags minflt cminflt
    # majflt cmajflt utime stime cutime cstime priority nice threads itreal starttime.
    write_file(PROC_ROOT, f'proc/{pid}/stat',
               f'{pid} (demo_node) S 1 {pid} {pid} 0 -1 0 0 0 0 0 7 3 0 0 20 0 2 0 100\n')
    write_file(PROC_ROOT, f'proc/{pid}/status', 'VmRSS:\t1024 kB\nVmSize:\t2048 kB\n')
    if domain is not None:
        write_file(PROC_ROOT, f'proc/{pid}/environ', f'PATH=/usr/bin\0ROS_DOMAIN_ID={domain}\0')


def generate_test_description():
    write_synthetic_process(5101, '/powertrain/engine/temp_sensor')
    write_synthetic_process(GHOST_PID, '/nowhere/ghost_node')
    # The pairs are written in opposite orders. In one of them the unreadable process
    # is read last, whether the directory lists entries in creation order, in reverse
    # or by number.
    write_synthetic_process(5201, '/body/door/front_left/status_sensor', get_test_domain_id())
    write_synthetic_process(5202, '/body/door/front_left/status_sensor')
    write_synthetic_process(5302, '/chassis/brakes/actuator')
    write_synthetic_process(5301, '/chassis/brakes/actuator', get_test_domain_id())
    return create_test_launch(
        demo_nodes=APPS,
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

    REQUIRED_APPS = set(APPS)

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

    def wait_warned(self, proc_output, gateway_node, pid):
        """Poll until each plugin has logged the environ warning for pid."""
        deadline = time.monotonic() + FOUND_TIMEOUT
        while time.monotonic() < deadline:
            if all(self.warnings(proc_output, gateway_node, pid).values()):
                return
            time.sleep(0.2)
        self.fail(f'no environ warning for PID {pid}: '
                  f'{self.warnings(proc_output, gateway_node, pid)}')

    def test_1_process_without_environ_is_served_by_name(self):
        """The process is matched on its command line alone and served."""
        self.wait_found('temp_sensor', 5101)

    def test_2_warning_is_logged_once_per_served_pid(self, proc_output, gateway_node):
        """Each plugin warns once per PID it serves by name, and never for the ghost."""
        # Each process is written after the previous one is found, so it is found in a
        # later scan. Every scan reads the ghost, and the PIDs found before are looked
        # up again after it.
        write_synthetic_process(5102, '/powertrain/engine/rpm_sensor')
        self.wait_found('rpm_sensor', 5102)
        self.wait_found('temp_sensor', 5101)
        write_synthetic_process(5103, '/chassis/brakes/pressure_sensor')
        self.wait_found('pressure_sensor', 5103)
        self.wait_found('temp_sensor', 5101)
        self.wait_found('rpm_sensor', 5102)
        # Warnings reach the output in the order they are logged; 5105 is served last.
        write_synthetic_process(5105, '/body/lights/controller')
        self.wait_found('controller', 5105)
        self.wait_warned(proc_output, gateway_node, 5105)
        once = dict.fromkeys(PLUGINS, 1)
        for pid in (5101, 5102, 5103):
            self.assertEqual(self.warnings(proc_output, gateway_node, pid), once, f'PID {pid}')
        self.assertEqual(self.warnings(proc_output, gateway_node, GHOST_PID),
                         dict.fromkeys(PLUGINS, 0))

    def test_3_process_of_the_domain_keeps_its_app(self, proc_output, gateway_node):
        """A same-named process that cannot be checked never replaces one of the domain."""
        # Both processes of a pair are in the tree before the gateway starts, so every
        # answer comes from a scan that has read both.
        for app_id, own, unreadable in PAIRS:
            deadline = time.monotonic() + FOUND_TIMEOUT
            last = None
            while time.monotonic() < deadline:
                last = self.answer(app_id, 'x-medkit-procfs')
                self.assertNotEqual(last[:2], (200, unreadable),
                                    f'{app_id} served the process it cannot check')
                if last[:2] == (200, own):
                    break
                time.sleep(0.2)
            else:
                self.fail(f'{app_id} not served as PID {own}; last {last}')
        for _, _, unreadable in PAIRS:
            self.assertEqual(self.warnings(proc_output, gateway_node, unreadable),
                             dict.fromkeys(PLUGINS, 0), f'PID {unreadable}')


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
