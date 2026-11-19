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

"""Protected mode: a gateway without authentication binds only a loopback address.

Every case starts the real gateway_node (or a launch file) as a subprocess and
reads the outcome from its own output and exit code. Each refusal has a start
on the same host in the same class, so "nothing started" cannot pass for
"refused". A non-loopback start is read from the ready line, because this box
may not own the address; a loopback start is also asked over HTTP.

@verifies REQ_INTEROP_086
"""

import os
import select
import shutil
import signal
import socket
import subprocess
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH, get_test_port
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

# The cases start their gateways one at a time on PORT; RESIDENT_PORT belongs to
# the gateway that keeps the launch alive and answers on loopback.
PORT = get_test_port(0)
RESIDENT_PORT = get_test_port(1)
READY_LINE = 'Medkit Gateway ready on'
REFUSAL = 'Refusing to start: authentication is off'
JWT_SECRET = 'a_secret_for_protected_mode_of_at_least_32_chars'
CLIENT = 'pm:pm_client_secret:admin'

_PARAMS_DIR = tempfile.mkdtemp(prefix='medkit_protected_mode_')

LOOPBACK_HOSTS = (
    '127.0.0.1', '127.255.255.254', '127.0.0.0', '127.1', '::1', 'localhost',
    'LOCALHOST', '::ffff:127.0.0.1',
)
REMOTE_HOSTS = (
    '0.0.0.0', '::', '192.168.1.10', '10.0.0.5', '2001:db8::5', 'gateway.example.test',
    '126.255.255.255', '128.0.0.1',
)


def _write_params(name, auth_enabled, allow_remote=None):
    """Write a params file for every node; auth.jwt_secret and clients go with `enabled`."""
    lines = ['/**:', '  ros__parameters:', '    auth:',
             f'      enabled: {"true" if auth_enabled else "false"}']
    if auth_enabled:
        lines += [f'      jwt_secret: "{JWT_SECRET}"', '      clients:', f'        - "{CLIENT}"']
    if allow_remote is not None:
        lines += ['    server:',
                  f'      allow_unauthenticated_remote: {"true" if allow_remote else "false"}']
    path = os.path.join(_PARAMS_DIR, name)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    return path


FILE_AUTH_OFF = _write_params('off.yaml', False)
FILE_AUTH_ON = _write_params('on.yaml', True)
FILE_OPT_OUT = _write_params('opt-out.yaml', False, allow_remote=True)
FILE_OPT_OUT_FALSE = _write_params('opt-out-false.yaml', False, allow_remote=False)


@pytest.mark.launch_test
def generate_test_description():
    """One loopback gateway keeps the launch alive; every case starts its own subprocess."""
    resident = create_gateway_node(
        port=RESIDENT_PORT, name='protected_mode_resident',
        extra_params={'server.host': '127.0.0.1'})
    return launch.LaunchDescription([
        resident,
        launch_testing.actions.ReadyToTest(),
    ]), {'resident': resident}


def _listening():
    """Return True when something accepts connections on the test port."""
    for family, address in ((socket.AF_INET, '127.0.0.1'), (socket.AF_INET6, '::1')):
        try:
            with socket.socket(family, socket.SOCK_STREAM) as sock:
                sock.settimeout(2)
                if sock.connect_ex((address, PORT)) == 0:
                    return True
        except OSError:
            continue
    return False


class _Process:
    """A subprocess whose output is read until a marker, or until it exits.

    `rc` is the exit code when the process ended before `until` showed, else
    None. Leaving the block kills the process group.
    """

    def __init__(self, testcase, command, env_extra=None, timeout=60, until=READY_LINE):
        self.testcase = testcase
        self.command = command
        self.env = {k: v for k, v in os.environ.items() if not k.startswith('MEDKIT_')}
        self.env.update(env_extra or {})
        self.timeout = timeout
        self.until = until
        self.rc = None
        self.ready = False
        self._chunks = []
        self._process = None

    @property
    def output(self):
        return b''.join(self._chunks).decode(errors='replace')

    def _read_until(self, marker):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            readable, _, _ = select.select([self._process.stdout], [], [], 0.5)
            if not readable:
                continue
            chunk = os.read(self._process.stdout.fileno(), 65536)
            if not chunk:
                return 'exit'
            self._chunks.append(chunk)
            if marker.encode() in b''.join(self._chunks[-2:]):
                return 'marker'
        return 'deadline'

    def __enter__(self):
        self._process = subprocess.Popen(
            self.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=self.env, start_new_session=True)
        outcome = self._read_until(self.until)
        self.ready = READY_LINE in self.output
        if outcome == 'exit':
            self._process.wait(timeout=15)
            self.rc = self._process.returncode
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.rc is None:
            os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            try:
                self._process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
        rest, _ = self._process.communicate()
        if rest:
            self._chunks.append(rest)
        return False

    def assert_refused(self, host):
        t = self.testcase
        t.assertIsNotNone(self.rc, f'the gateway did not exit on {host}: {self.output[-600:]}')
        t.assertNotEqual(self.rc, 0, self.output[-600:])
        t.assertIn(REFUSAL, self.output)
        t.assertIn(f'server.host is {host},', self.output)
        for fix in ('MEDKIT_JWT_SECRET', 'server.host: 127.0.0.1',
                    'server.allow_unauthenticated_remote: true'):
            t.assertIn(fix, self.output)
        t.assertNotIn(READY_LINE, self.output)
        t.assertFalse(_listening(), 'something listens after the refusal')

    def assert_started(self, host, probe='default', expect_open_warning=False):
        t = self.testcase
        t.assertIsNone(self.rc, self.output[-600:])
        t.assertTrue(self.ready, f'no ready line on {host}: {self.output[-600:]}')
        t.assertIn(f'://{host}:{PORT}', self.output)
        t.assertNotIn(REFUSAL, self.output)
        warned = 'reachable without authentication' in self.output
        t.assertEqual(warned, expect_open_warning, self.output[-800:])
        probe = _probe_url(host) if probe == 'default' else probe
        if probe:
            resp = requests.get(f'{probe}{API_BASE_PATH}/health', timeout=15)
            t.assertIn(resp.status_code, (200, 401, 403))


OPT_OUT_ARGS = ['-p', 'server.allow_unauthenticated_remote:=true']


def _gateway(host, params_file=None, extra_args=()):
    # Quoted: `127.1` is a float and `::` is not a scalar to the YAML parser.
    return ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
            '-p', f'server.port:={PORT}', '-p', f'server.host:="{host}"',
            *(['--params-file', params_file] if params_file else []), *extra_args]


def _probe_url(host):
    """Where to ask a gateway bound to `host`, or None when this box may not own the address."""
    if host == '0.0.0.0':
        return f'http://127.0.0.1:{PORT}'
    if host == '::':
        return f'http://[::1]:{PORT}'
    if host in LOOPBACK_HOSTS:
        return f'http://[{host}]:{PORT}' if ':' in host else f'http://{host}:{PORT}'
    return None


class TestAuthOffBindsLoopbackOnly(unittest.TestCase):
    """P1, P2: with authentication off only a loopback host starts."""

    def test_loopback_hosts_start(self):
        for host in LOOPBACK_HOSTS:
            with self.subTest(host=host):
                with _Process(self, _gateway(host, FILE_AUTH_OFF)) as gateway:
                    gateway.assert_started(host, probe=_probe_url(host))

    def test_every_other_host_refuses_and_names_the_fixes(self):
        for host in REMOTE_HOSTS:
            with self.subTest(host=host):
                with _Process(self, _gateway(host, FILE_AUTH_OFF)) as gateway:
                    gateway.assert_refused(host)

    def test_the_default_params_and_no_file_refuse_the_same_way(self):
        with _Process(self, _gateway('0.0.0.0')) as gateway:
            gateway.assert_refused('0.0.0.0')


class TestAuthOnBindsAnywhere(unittest.TestCase):
    """P4, P5: with authentication in force every host starts."""

    def test_auth_from_the_params(self):
        for host in REMOTE_HOSTS:
            with self.subTest(host=host):
                with _Process(self, _gateway(host, FILE_AUTH_ON)) as gateway:
                    gateway.assert_started(host, probe=_probe_url(host))

    def test_auth_from_the_environment_over_a_file_that_says_off(self):
        env = {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': CLIENT}
        for host in REMOTE_HOSTS:
            with self.subTest(host=host):
                with _Process(self, _gateway(host, FILE_AUTH_OFF), env) as gateway:
                    gateway.assert_started(host, probe=_probe_url(host))

    def test_the_closed_gateway_answers_401_on_a_bind_all_address(self):
        env = {'MEDKIT_JWT_SECRET': JWT_SECRET}
        with _Process(self, _gateway('0.0.0.0'), env) as gateway:
            gateway.assert_started('0.0.0.0', probe=None)
            resp = requests.get(f'http://127.0.0.1:{PORT}{API_BASE_PATH}/areas', timeout=15)
            self.assertEqual(resp.status_code, 401)


class TestOptOut(unittest.TestCase):
    """P3, P5: the parameter and MEDKIT_AUTH_DISABLED=1 let an open gateway bind anywhere."""

    def test_the_parameter_opens_every_remote_host_and_warns(self):
        for host in REMOTE_HOSTS:
            with self.subTest(host=host):
                command = _gateway(host, FILE_AUTH_OFF, OPT_OUT_ARGS)
                with _Process(self, command) as gateway:
                    gateway.assert_started(host, probe=_probe_url(host),
                                           expect_open_warning=True)

    def test_the_parameter_from_a_file(self):
        with _Process(self, _gateway('0.0.0.0', FILE_OPT_OUT)) as gateway:
            gateway.assert_started('0.0.0.0', expect_open_warning=True)
            resp = requests.get(f'http://127.0.0.1:{PORT}{API_BASE_PATH}/areas', timeout=15)
            self.assertEqual(resp.status_code, 200)

    def test_an_explicit_false_still_refuses(self):
        with _Process(self, _gateway('0.0.0.0', FILE_OPT_OUT_FALSE)) as gateway:
            gateway.assert_refused('0.0.0.0')

    def test_the_environment_variable_implies_the_opt_out(self):
        for host in REMOTE_HOSTS:
            with self.subTest(host=host):
                with _Process(self, _gateway(host, FILE_AUTH_OFF),
                              {'MEDKIT_AUTH_DISABLED': '1'}) as gateway:
                    gateway.assert_started(host, probe=_probe_url(host),
                                           expect_open_warning=True)

    def test_the_environment_variable_over_a_file_that_asks_for_auth(self):
        env = {'MEDKIT_AUTH_DISABLED': '1'}
        with _Process(self, _gateway('0.0.0.0', FILE_AUTH_ON), env) as gateway:
            gateway.assert_started('0.0.0.0', expect_open_warning=True)
            resp = requests.get(f'http://127.0.0.1:{PORT}{API_BASE_PATH}/areas', timeout=15)
            self.assertEqual(resp.status_code, 200)

    def test_a_value_other_than_one_is_no_opt_out(self):
        for value in ('0', 'true', ''):
            with self.subTest(value=value):
                with _Process(self, _gateway('0.0.0.0', FILE_AUTH_OFF),
                              {'MEDKIT_AUTH_DISABLED': value}) as gateway:
                    gateway.assert_refused('0.0.0.0')


class TestLaunchArgument(unittest.TestCase):
    """P6: bringup.launch.py and gateway.launch.py take allow_unauthenticated_remote."""

    @staticmethod
    def _launch(file, *args):
        return ['ros2', 'launch', 'ros2_medkit_gateway', file, f'server_port:={PORT}',
                'server_host:=0.0.0.0', *args]

    def test_bringup_without_the_argument_refuses(self):
        with _Process(self, self._launch('bringup.launch.py', 'enable_log_bridge:=false',
                                         'enable_action_status_bridge:=false',
                                         'enable_fault_manager:=false'),
                      timeout=90, until=REFUSAL) as launched:
            self.assertIn(REFUSAL, launched.output)
            self.assertNotIn(READY_LINE, launched.output)
            self.assertFalse(_listening())

    def test_bringup_with_the_argument_starts(self):
        with _Process(self, self._launch('bringup.launch.py', 'allow_unauthenticated_remote:=true',
                                         'enable_log_bridge:=false',
                                         'enable_action_status_bridge:=false',
                                         'enable_fault_manager:=false'),
                      timeout=90) as launched:
            launched.assert_started('0.0.0.0', expect_open_warning=True)

    def test_gateway_launch_pair(self):
        command = self._launch('gateway.launch.py')
        with _Process(self, command, timeout=90, until=REFUSAL) as launched:
            self.assertIn(REFUSAL, launched.output)
            self.assertNotIn(READY_LINE, launched.output)
        command = self._launch('gateway.launch.py', 'allow_unauthenticated_remote:=true')
        with _Process(self, command, timeout=90) as launched:
            launched.assert_started('0.0.0.0', expect_open_warning=True)

    def test_bringup_loopback_needs_nothing(self):
        with _Process(self, ['ros2', 'launch', 'ros2_medkit_gateway', 'bringup.launch.py',
                             f'server_port:={PORT}', 'enable_log_bridge:=false',
                             'enable_action_status_bridge:=false', 'enable_fault_manager:=false'],
                      timeout=90) as launched:
            launched.assert_started('127.0.0.1')


@launch_testing.post_shutdown_test()
class TestProtectedModeShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info, resident):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=resident)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(_PARAMS_DIR, ignore_errors=True)

    def test_nothing_left_listening(self):
        self.assertFalse(_listening())
