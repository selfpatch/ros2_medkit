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

"""Protected mode: a gateway that serves anonymous requests binds only a loopback address.

Every case starts the real gateway_node (or a launch file) as a subprocess and
reads the outcome from its own output and exit code. A start counts only when
the process the case started answers HTTP on a port no other process held, and
owns the listener on it. Each refusal has a start on the same host in the same
class, so "nothing started" cannot pass for "refused". A gateway that cannot
bind exits instead of announcing itself.

@verifies REQ_INTEROP_086
"""

import errno
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

# The test owns a band of ten ports. RESIDENT_PORT belongs to the gateway that
# keeps the launch alive; every started gateway takes the next port nobody holds.
PORT = get_test_port(0)
RESIDENT_PORT = get_test_port(1)
BAND = [get_test_port(offset) for offset in range(2, 10)]
READY_LINE = 'Medkit Gateway ready on'
REFUSAL_OFF = 'Refusing to start: authentication is off'
REFUSAL_ON = 'Refusing to start: authentication is on but auth.require_auth_for is'
BIND_REFUSAL = 'Refusing to start: cannot bind the REST API to'
WAIT_LINE = 'a socket bound without listening; waiting up to'
BOUND_LINE = 'bound after waiting'
# A session that ignores proxy settings of the environment: every request goes direct.
_HTTP = requests.Session()
_HTTP.trust_env = False
JWT_SECRET = 'a_secret_for_protected_mode_of_at_least_32_chars'
CLIENT = 'pm:pm_client_secret:admin'

_PARAMS_DIR = tempfile.mkdtemp(prefix='medkit_protected_mode_')

LOOPBACK_HOSTS = (
    '127.0.0.1', '127.255.255.254', '127.0.0.0', '127.1', '::1', 'localhost',
    'LOCALHOST', '::ffff:127.0.0.1',
)
# Refused before any bind, so the machine need not own them.
REMOTE_HOSTS = (
    '0.0.0.0', '::', '192.168.1.10', '10.0.0.5', '2001:db8::5', 'gateway.example.test',
    '126.255.255.255', '128.0.0.1',
)
# The address no interface owns, and a name that does not resolve: the bind fails, with this cause.
UNBINDABLE_HOSTS = {
    '192.0.2.10': 'Cannot assign requested address',
    '2001:db8::5': 'Cannot assign requested address',
    'gateway.example.test': 'cannot resolve the host:',
}


def _own_ipv4():
    """Return this machine's non-loopback IPv4 address, or None when it has none."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(('192.0.2.1', 9))
            address = sock.getsockname()[0]
    except OSError:
        return None
    return None if address.startswith('127.') or address == '0.0.0.0' else address


OWN_IPV4 = _own_ipv4()
NO_OWN_ADDRESS = ('this machine has no non-loopback IPv4 address: the start cases on it '
                  'cannot run, and the test needs one')
# Non-loopback addresses a start case can use here.
BINDABLE_REMOTE_HOSTS = ('0.0.0.0', '::') + ((OWN_IPV4,) if OWN_IPV4 else ())


def _write_params(name, auth_enabled, allow_remote=None, require_auth_for=None):
    """Write a params file for every node; auth.jwt_secret and clients go with `enabled`."""
    lines = ['/**:', '  ros__parameters:', '    auth:',
             f'      enabled: {"true" if auth_enabled else "false"}']
    if auth_enabled:
        lines += [f'      jwt_secret: "{JWT_SECRET}"', '      clients:', f'        - "{CLIENT}"']
    if require_auth_for:
        lines += [f'      require_auth_for: "{require_auth_for}"']
    if allow_remote is not None:
        lines += ['    server:',
                  f'      allow_unauthenticated_remote: {"true" if allow_remote else "false"}']
    path = os.path.join(_PARAMS_DIR, name)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    return path


FILE_AUTH_OFF = _write_params('off.yaml', False)
FILE_AUTH_ON = _write_params('on.yaml', True, require_auth_for='write')
FILE_AUTH_ALL = _write_params('all.yaml', True, require_auth_for='all')
FILE_AUTH_NONE = _write_params('none.yaml', True, require_auth_for='none')
FILE_AUTH_NONE_OPT_OUT = _write_params('none-opt-out.yaml', True, allow_remote=True,
                                       require_auth_for='none')
FILE_AUTH_NONE_UPPER = _write_params('none-upper.yaml', True, require_auth_for='NONE')
FILE_AUTH_NONE_MIXED = _write_params('none-mixed.yaml', True, require_auth_for='None')
FILE_AUTH_WRITE_UPPER = _write_params('write-upper.yaml', True, require_auth_for='WRITE')
FILE_AUTH_ALL_MIXED = _write_params('all-mixed.yaml', True, require_auth_for='All')
FILE_OPT_OUT = _write_params('opt-out.yaml', False, allow_remote=True)
FILE_OPT_OUT_FALSE = _write_params('opt-out-false.yaml', False, allow_remote=False)
# A gateway.launch.py config_file names the node, unlike the `/**` files above.
LAUNCH_CONFIG_OPT_OUT = os.path.join(_PARAMS_DIR, 'launch-config.yaml')
with open(LAUNCH_CONFIG_OPT_OUT, 'w', encoding='utf-8') as _handle:
    _handle.write('ros2_medkit_gateway:\n  ros__parameters:\n    server:\n'
                  '      allow_unauthenticated_remote: true\n')


def _tls_params():
    """Write a self-signed certificate and a params file that turns TLS on with it."""
    cert = os.path.join(_PARAMS_DIR, 'cert.pem')
    key = os.path.join(_PARAMS_DIR, 'key.pem')
    subprocess.run(
        ['openssl', 'req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-days', '1', '-keyout', key,
         '-out', cert, '-subj', '/CN=localhost'],
        check=True, capture_output=True)
    os.chmod(key, 0o600)
    path = os.path.join(_PARAMS_DIR, 'tls.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('/**:\n  ros__parameters:\n    server:\n      tls:\n        enabled: true\n'
                     f'        cert_file: "{cert}"\n        key_file: "{key}"\n')
    return path


TLS_PARAMS = _tls_params()


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


# ---------------------------------------------------------------------------
# Ports and processes
# ---------------------------------------------------------------------------

def _port_is_free(port):
    """Return True when nothing listens on `port` and a wildcard bind succeeds."""
    for family, wildcard in ((socket.AF_INET, '0.0.0.0'), (socket.AF_INET6, '::')):
        try:
            with socket.socket(family, socket.SOCK_STREAM) as sock:
                # As the gateway binds: TIME_WAIT does not make a port busy, a listener does.
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind((wildcard, port))
        except OSError as error:
            if family == socket.AF_INET:
                return False
            if error.errno not in (97, 99):  # no IPv6 here: fine
                return False
    return True


_NEXT = [0]


def _free_port():
    """Return the next port of the band that no process holds."""
    for _ in range(len(BAND)):
        port = BAND[_NEXT[0] % len(BAND)]
        _NEXT[0] += 1
        if _port_is_free(port):
            return port
    raise AssertionError('no free port in the test band')


def _clean_port():
    """Return a port no socket of this machine has used lately: no listener, no TIME_WAIT."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def _listening(port):
    """Return True when something accepts connections on `port`."""
    for family, address in ((socket.AF_INET, '127.0.0.1'), (socket.AF_INET6, '::1')):
        try:
            with socket.socket(family, socket.SOCK_STREAM) as sock:
                sock.settimeout(2)
                if sock.connect_ex((address, port)) == 0:
                    return True
        except OSError:
            continue
    return False


def _listener_inodes(port):
    """Return the socket inodes listening on `port` (IPv4 and IPv6)."""
    inodes = set()
    for table in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            with open(table, encoding='utf-8') as handle:
                rows = handle.read().splitlines()[1:]
        except OSError:
            continue
        for row in rows:
            fields = row.split()
            if fields[3] == '0A' and int(fields[1].split(':')[1], 16) == port:
                inodes.add(fields[9])
    return inodes


def _listener_pids(port):
    """Return the pids that hold a listening socket on `port`."""
    inodes = {f'socket:[{inode}]' for inode in _listener_inodes(port)}
    pids = set()
    for entry in os.listdir('/proc'):
        if not entry.isdigit():
            continue
        try:
            for fd in os.listdir(f'/proc/{entry}/fd'):
                if os.readlink(f'/proc/{entry}/fd/{fd}') in inodes:
                    pids.add(int(entry))
                    break
        except OSError:
            continue
    return pids


def _parent_of(pid):
    """Return the parent pid of `pid`, or None when it is gone."""
    try:
        with open(f'/proc/{pid}/stat', encoding='utf-8') as handle:
            return int(handle.read().rsplit(')', 1)[1].split()[1])
    except (OSError, IndexError, ValueError):
        return None


def _tree(root):
    """Return `root` and every live descendant of it."""
    parents = {}
    for entry in os.listdir('/proc'):
        if entry.isdigit():
            parent = _parent_of(int(entry))
            if parent is not None:
                parents[int(entry)] = parent
    found = {root}
    grew = True
    while grew:
        grew = False
        for pid, parent in parents.items():
            if parent in found and pid not in found:
                found.add(pid)
                grew = True
    return found


def _alive(pid):
    """Return True when `pid` runs (a zombie is not running)."""
    try:
        with open(f'/proc/{pid}/stat', encoding='utf-8') as handle:
            return handle.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


def _probe_url(host, port):
    """Return where to ask a gateway bound to `host`, or None when it cannot be asked here."""
    if host == '0.0.0.0':
        return f'http://127.0.0.1:{port}'
    if host == '::':
        return f'http://[::1]:{port}'
    if host in LOOPBACK_HOSTS or host == OWN_IPV4:
        return f'http://[{host}]:{port}' if ':' in host else f'http://{host}:{port}'
    return None


def _time_wait(port):
    """Return True when a TIME_WAIT socket with local port `port` exists."""
    for table in ('/proc/net/tcp', '/proc/net/tcp6'):
        try:
            with open(table, encoding='utf-8') as handle:
                rows = handle.read().splitlines()[1:]
        except OSError:
            continue
        for row in rows:
            fields = row.split()
            if fields[3] == '06' and int(fields[1].split(':')[1], 16) == port:
                return True
    return False


def _wait_for_time_wait(testcase, port, message, timeout=10):
    """Wait for a TIME_WAIT entry on `port`; it appears when the peer's FIN reaches the closer."""
    deadline = time.monotonic() + timeout
    while not _time_wait(port):
        if time.monotonic() >= deadline:
            testcase.fail(message)
        time.sleep(0.05)


def _nonlocal_bind_allowed():
    """Return True when the kernel lets a process bind an address no interface owns."""
    for name in ('ipv4', 'ipv6'):
        try:
            with open(f'/proc/sys/net/{name}/ip_nonlocal_bind', encoding='utf-8') as handle:
                if handle.read().strip() == '1':
                    return True
        except OSError:
            continue
    return False


# What the stand-in for a released gateway runs: SO_REUSEPORT only, as cpp-httplib's default does.
STAND_IN = r"""
import socket, sys
port, hold = int(sys.argv[1]), sys.argv[2] == 'hold'
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
srv.bind(('127.0.0.1', port))
srv.listen()
print('listening', flush=True)
conn, _ = srv.accept()
conn.close()
print('closed', flush=True)
if hold:
    sys.stdin.read()
"""


class _StandIn:
    """A process that holds a port with SO_REUSEPORT, serves one connection and closes it first.

    With `hold` it then keeps listening until the block ends; without, it exits.
    """

    def __init__(self, testcase, port, hold):
        self.testcase = testcase
        self.port = port
        self.hold = hold
        self._process = None

    @property
    def pid(self):
        return self._process.pid

    def __enter__(self):
        self._process = subprocess.Popen(
            ['python3', '-c', STAND_IN, str(self.port), 'hold' if self.hold else 'exit'],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.testcase.assertEqual(self._process.stdout.readline().strip(), 'listening')
        client = socket.create_connection(('127.0.0.1', self.port), timeout=15)
        self.testcase.assertEqual(self._process.stdout.readline().strip(), 'closed')
        self.testcase.assertEqual(client.recv(1), b'')
        client.close()
        return self

    def wait_exit(self):
        self._process.wait(timeout=30)

    def __exit__(self, exc_type, exc, tb):
        if self._process.poll() is None:
            self._process.stdin.close()
            try:
                self._process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                self._process.kill()
        return False


class _Listener:
    """A process listening on 127.0.0.1:`port` with SO_REUSEPORT only, like a released gateway."""

    SCRIPT = r"""
import socket, sys
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
srv.bind(('127.0.0.1', int(sys.argv[1])))
srv.listen()
print('listening', flush=True)
sys.stdin.read()
"""

    def __init__(self, testcase, port):
        self.testcase = testcase
        self.port = port

    def __enter__(self):
        self._process = subprocess.Popen(['python3', '-c', self.SCRIPT, str(self.port)],
                                         stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
        self.testcase.assertEqual(self._process.stdout.readline().strip(), 'listening')
        return self

    @property
    def pid(self):
        return self._process.pid

    def __exit__(self, exc_type, exc, tb):
        self._process.stdin.close()
        self._process.wait(timeout=15)
        return False


class _Process:
    """A subprocess whose output is read until the ready line, or until it exits.

    `rc` is the exit code when the process ended before the ready line, else
    None. Leaving the block kills the process group.
    """

    def __init__(self, testcase, command, env_extra=None, timeout=60, port=None, until=READY_LINE):
        self.testcase = testcase
        self.command = command
        self.env = {k: v for k, v in os.environ.items() if not k.startswith('MEDKIT_')}
        self.env.update(env_extra or {})
        self.timeout = timeout
        self.until = until
        self.wait_seen_at = None
        self.exit_at = None
        self.port = port if port is not None else PORT
        self.rc = None
        self.ready = False
        self._chunks = []
        self._process = None

    @property
    def pid(self):
        return self._process.pid

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
                if self.exit_at is None:
                    self.exit_at = time.monotonic()
                return 'exit'
            self._chunks.append(chunk)
            if self.wait_seen_at is None and WAIT_LINE.encode() in b''.join(self._chunks[-2:]):
                self.wait_seen_at = time.monotonic()
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

    def wait_exit(self, timeout=90):
        """Read until the process ends; return its exit code."""
        if self.rc is not None:
            return self.rc
        outcome = self._read_until('\0never')
        self.testcase.assertEqual(outcome, 'exit', f'still running: {self.output[-400:]}')
        self._process.wait(timeout=timeout)
        self.rc = self._process.returncode
        return self.rc

    def __exit__(self, exc_type, exc, tb):
        if self.rc is None and self._process.poll() is None:
            os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            try:
                self._process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
        rest, _ = self._process.communicate()
        if rest:
            self._chunks.append(rest)
        return False

    def assert_refused(self, host, reason=REFUSAL_OFF):
        t = self.testcase
        t.assertIsNotNone(self.rc, f'the gateway did not exit on {host}: {self.output[-600:]}')
        t.assertNotEqual(self.rc, 0, self.output[-600:])
        t.assertIn(reason, self.output)
        t.assertIn(f'server.host is {host},', self.output)
        for fix in ('MEDKIT_JWT_SECRET', 'server.host: 127.0.0.1',
                    'server.allow_unauthenticated_remote: true'):
            t.assertIn(fix, self.output)
        t.assertNotIn(READY_LINE, self.output)
        t.assertFalse(_listening(self.port), 'something listens after the refusal')

    def assert_bind_failed(self, host, cause):
        t = self.testcase
        t.assertIsNotNone(self.rc, f'the gateway did not exit on {host}: {self.output[-600:]}')
        t.assertNotEqual(self.rc, 0, self.output[-600:])
        t.assertIn(f'{BIND_REFUSAL} {host}:{self.port}: {cause}', self.output)
        t.assertNotIn(READY_LINE, self.output)

    def assert_started(self, host, expect_open_warning=False, probe='default',
                       expect_wait=False):
        """Assert the started process answers HTTP on a port it alone listens on."""
        t = self.testcase
        t.assertIsNone(self.rc, self.output[-600:])
        t.assertTrue(self.ready, f'no ready line on {host}: {self.output[-600:]}')
        t.assertIn(f'://{host}:{self.port}', self.output)
        t.assertNotIn('Refusing to start', self.output)
        warned = 'reachable without authentication' in self.output
        t.assertEqual(warned, expect_open_warning, self.output[-800:])
        t.assertEqual(WAIT_LINE in self.output, expect_wait, self.output[-800:])
        t.assertEqual(BOUND_LINE in self.output, expect_wait, self.output[-800:])
        owners = _listener_pids(self.port)
        t.assertTrue(owners, f'nobody listens on port {self.port}: {self.output[-600:]}')
        t.assertTrue(owners <= _tree(self.pid),
                     f'port {self.port} is held by {owners}, not by the started process tree')
        probe = _probe_url(host, self.port) if probe == 'default' else probe
        t.assertIsNotNone(probe, f'no way to ask a gateway bound to {host}')
        return _HTTP.get(f'{probe}{API_BASE_PATH}/health', timeout=15, verify=False), probe

    def assert_answers(self, host, **kwargs):
        response, _ = self.assert_started(host, **kwargs)
        self.testcase.assertIn(response.status_code, (200, 401, 403))


def _gateway(host, port, params_file=None, extra_args=()):
    # Quoted: `127.1` is a float and `::` is not a scalar to the YAML parser.
    return ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
            '-p', f'server.port:={port}', '-p', f'server.host:="{host}"',
            *(['--params-file', params_file] if params_file else []), *extra_args]


OPT_OUT_ARGS = ['-p', 'server.allow_unauthenticated_remote:=true']


def _run(testcase, host, params_file=None, env=None, extra_args=(), port=None):
    """Return a started-or-refused process for `host` on a port nobody holds."""
    port = port if port is not None else _free_port()
    return _Process(testcase, _gateway(host, port, params_file, extra_args), env, port=port)


# ---------------------------------------------------------------------------
# Cases
# ---------------------------------------------------------------------------

class TestAuthOffBindsLoopbackOnly(unittest.TestCase):
    """P1, P2: with authentication off only a loopback host starts."""

    def test_loopback_hosts_start(self):
        for host in LOOPBACK_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_OFF) as gateway:
                    gateway.assert_answers(host)

    def test_every_other_host_refuses_and_names_the_fixes(self):
        for host in REMOTE_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_OFF) as gateway:
                    gateway.assert_refused(host)

    def test_the_default_params_and_no_file_refuse_the_same_way(self):
        with _run(self, '0.0.0.0') as gateway:
            gateway.assert_refused('0.0.0.0')

    def test_the_machine_has_a_non_loopback_ipv4_address(self):
        self.assertIsNotNone(OWN_IPV4, NO_OWN_ADDRESS)


class TestAuthOnBindsAnywhere(unittest.TestCase):
    """P4, P5: with authentication in force every host starts."""

    def test_auth_from_the_params(self):
        for host in BINDABLE_REMOTE_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_ON) as gateway:
                    gateway.assert_answers(host)

    def test_auth_from_the_environment_over_a_file_that_says_off(self):
        env = {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': CLIENT}
        for host in BINDABLE_REMOTE_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_OFF, env) as gateway:
                    gateway.assert_answers(host)

    def test_the_closed_gateway_answers_401_on_a_bind_all_address(self):
        with _run(self, '0.0.0.0', env={'MEDKIT_JWT_SECRET': JWT_SECRET}) as gateway:
            _, probe = gateway.assert_started('0.0.0.0')
            self.assertEqual(_HTTP.get(f'{probe}{API_BASE_PATH}/areas', timeout=15).status_code,
                             401)


class TestRequireAuthForNone(unittest.TestCase):
    """D1: authentication on with require_auth_for none leaves every route open."""

    def test_none_on_a_bind_all_address_refuses(self):
        for host in REMOTE_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_NONE) as gateway:
                    gateway.assert_refused(host, reason=f'{REFUSAL_ON} none')
                    self.assertIn('MEDKIT_AUTH_DISABLED=1', gateway.output)

    def test_none_with_the_opt_out_starts_and_warns(self):
        for host in BINDABLE_REMOTE_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_NONE_OPT_OUT) as gateway:
                    gateway.assert_answers(host, expect_open_warning=True)
                    self.assertIn('require_auth_for none', gateway.output)

    def test_none_on_loopback_starts(self):
        with _run(self, '127.0.0.1', FILE_AUTH_NONE) as gateway:
            gateway.assert_answers('127.0.0.1')

    def test_write_on_a_bind_all_address_starts_and_the_write_is_closed(self):
        with _run(self, '0.0.0.0', FILE_AUTH_ON) as gateway:
            _, probe = gateway.assert_started('0.0.0.0')
            self.assertEqual(_HTTP.get(f'{probe}{API_BASE_PATH}/areas', timeout=15).status_code,
                             200)
            self.assertEqual(
                _HTTP.delete(f'{probe}{API_BASE_PATH}/faults', timeout=15).status_code, 401)

    def test_all_on_a_bind_all_address_starts_and_the_read_is_closed(self):
        with _run(self, '0.0.0.0', FILE_AUTH_ALL) as gateway:
            _, probe = gateway.assert_started('0.0.0.0')
            self.assertEqual(_HTTP.get(f'{probe}{API_BASE_PATH}/areas', timeout=15).status_code,
                             401)


class TestRequireAuthForCase(unittest.TestCase):
    """E1: the case of require_auth_for does not change what protected mode decides."""

    def test_none_in_any_case_refuses(self):
        for name, params in (('NONE', FILE_AUTH_NONE_UPPER), ('None', FILE_AUTH_NONE_MIXED)):
            with self.subTest(value=name):
                with _run(self, '0.0.0.0', params) as gateway:
                    gateway.assert_refused('0.0.0.0', reason=f'{REFUSAL_ON} {name}')

    def test_write_in_any_case_starts_with_the_write_closed(self):
        with _run(self, '0.0.0.0', FILE_AUTH_WRITE_UPPER) as gateway:
            _, probe = gateway.assert_started('0.0.0.0')
            self.assertEqual(_HTTP.get(f'{probe}{API_BASE_PATH}/areas', timeout=15).status_code,
                             200)
            deleted = _HTTP.delete(f'{probe}{API_BASE_PATH}/faults', timeout=15)
            self.assertEqual(deleted.status_code, 401)

    def test_all_in_any_case_starts_with_the_read_closed(self):
        with _run(self, '0.0.0.0', FILE_AUTH_ALL_MIXED) as gateway:
            _, probe = gateway.assert_started('0.0.0.0')
            self.assertEqual(_HTTP.get(f'{probe}{API_BASE_PATH}/areas', timeout=15).status_code,
                             401)


class TestOptOut(unittest.TestCase):
    """P3, P5: the parameter and MEDKIT_AUTH_DISABLED=1 let an open gateway bind anywhere."""

    def test_the_parameter_opens_every_bindable_remote_host_and_warns(self):
        for host in BINDABLE_REMOTE_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_OFF, extra_args=OPT_OUT_ARGS) as gateway:
                    gateway.assert_answers(host, expect_open_warning=True)

    def test_the_parameter_from_a_file(self):
        with _run(self, '0.0.0.0', FILE_OPT_OUT) as gateway:
            _, probe = gateway.assert_started('0.0.0.0', expect_open_warning=True)
            self.assertEqual(_HTTP.get(f'{probe}{API_BASE_PATH}/areas', timeout=15).status_code,
                             200)

    def test_an_explicit_false_still_refuses(self):
        with _run(self, '0.0.0.0', FILE_OPT_OUT_FALSE) as gateway:
            gateway.assert_refused('0.0.0.0')

    def test_the_environment_variable_implies_the_opt_out(self):
        for host in BINDABLE_REMOTE_HOSTS:
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_OFF, {'MEDKIT_AUTH_DISABLED': '1'}) as gateway:
                    gateway.assert_answers(host, expect_open_warning=True)

    def test_the_environment_variable_over_a_file_that_asks_for_auth(self):
        with _run(self, '0.0.0.0', FILE_AUTH_ON, {'MEDKIT_AUTH_DISABLED': '1'}) as gateway:
            _, probe = gateway.assert_started('0.0.0.0', expect_open_warning=True)
            self.assertEqual(_HTTP.get(f'{probe}{API_BASE_PATH}/areas', timeout=15).status_code,
                             200)

    def test_a_value_other_than_one_is_no_opt_out(self):
        for value in ('0', 'true', ''):
            with self.subTest(value=value):
                env = {'MEDKIT_AUTH_DISABLED': value}
                with _run(self, '0.0.0.0', FILE_AUTH_OFF, env) as gateway:
                    gateway.assert_refused('0.0.0.0')


class TestBindFailure(unittest.TestCase):
    """D2: a gateway that cannot bind exits and never announces itself, naming the cause."""

    def test_a_port_held_by_another_process_ends_the_gateway(self):
        holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.addCleanup(holder.close)
        holder.bind(('127.0.0.1', 0))
        holder.listen()
        # The holder's port is outside the band, so the pair below needs no care for it.
        held = holder.getsockname()[1]
        with _run(self, '127.0.0.1', FILE_AUTH_OFF, port=held) as gateway:
            gateway.assert_bind_failed('127.0.0.1', 'Address already in use')
        self.assertEqual(_listener_pids(held), {os.getpid()}, 'the holder lost its port')
        with _run(self, '127.0.0.1', FILE_AUTH_OFF) as gateway:
            gateway.assert_answers('127.0.0.1')

    def test_a_port_held_by_another_gateway_ends_the_second(self):
        with _run(self, '127.0.0.1', FILE_AUTH_OFF) as first:
            first.assert_answers('127.0.0.1')
            with _run(self, '127.0.0.1', FILE_AUTH_OFF, port=first.port) as second:
                second.assert_bind_failed('127.0.0.1', 'Address already in use')
            self.assertTrue(_listener_pids(first.port) <= _tree(first.pid))
            self.assertIn(_HTTP.get(f'http://127.0.0.1:{first.port}{API_BASE_PATH}/health',
                                    timeout=15).status_code, (200, 401, 403))

    def test_an_address_the_machine_does_not_own_ends_the_gateway(self):
        if _nonlocal_bind_allowed():
            self.fail('ip_nonlocal_bind is set on this machine: the case cannot be observed')
        for host, cause in UNBINDABLE_HOSTS.items():
            with self.subTest(host=host):
                with _run(self, host, FILE_AUTH_OFF, {'MEDKIT_AUTH_DISABLED': '1'}) as gateway:
                    gateway.assert_bind_failed(host, cause)
                    self.assertFalse(_listening(gateway.port))

    def test_a_bracketed_address_is_not_a_bind_form(self):
        with _run(self, '[::1]', FILE_AUTH_OFF) as gateway:
            gateway.assert_bind_failed('[::1]', 'cannot resolve the host:')

    def test_a_wildcard_listener_ends_a_loopback_gateway_at_once(self):
        for family, wildcard in ((socket.AF_INET, '0.0.0.0'), (socket.AF_INET6, '::')):
            with self.subTest(listener=wildcard):
                holder = socket.socket(family, socket.SOCK_STREAM)
                self.addCleanup(holder.close)
                holder.bind((wildcard, 0))
                holder.listen()
                held = holder.getsockname()[1]
                with _run(self, '127.0.0.1', FILE_AUTH_OFF, port=held) as gateway:
                    gateway.assert_bind_failed('127.0.0.1', 'Address already in use')
                    self.assertNotIn(WAIT_LINE, gateway.output)
                self.assertEqual(_listener_pids(held), {os.getpid()}, 'the holder lost its port')

    def test_a_free_port_on_the_same_address_starts(self):
        self.assertIsNotNone(OWN_IPV4, NO_OWN_ADDRESS)
        with _run(self, OWN_IPV4, FILE_AUTH_OFF, {'MEDKIT_AUTH_DISABLED': '1'}) as gateway:
            gateway.assert_answers(OWN_IPV4, expect_open_warning=True)

    def test_a_held_port_with_tls_ends_the_gateway(self):
        holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.addCleanup(holder.close)
        holder.bind(('127.0.0.1', 0))
        holder.listen()
        held = holder.getsockname()[1]
        with _run(self, '127.0.0.1', TLS_PARAMS, port=held) as gateway:
            gateway.assert_bind_failed('127.0.0.1', 'Address already in use')
        with _run(self, '127.0.0.1', TLS_PARAMS) as gateway:
            gateway.assert_answers('127.0.0.1', probe=f'https://127.0.0.1:{gateway.port}')
            self.assertIn('HTTPS://127.0.0.1', gateway.output)


def _leave_time_wait(testcase, port):
    """Have a stand-in for a released gateway leave a TIME_WAIT entry on 127.0.0.1:`port`."""
    with _StandIn(testcase, port, hold=False) as old:
        old.wait_exit()
    _wait_for_time_wait(testcase, port, 'no TIME_WAIT entry, so a clean port proves nothing')


class TestRestartOverTimeWait(unittest.TestCase):
    """The gateway never shares a port: it waits out TIME_WAIT; a live listener ends the wait."""

    def test_the_first_start_after_a_so_reuseport_gateway_waits_and_is_not_shared(self):
        port = _clean_port()
        _leave_time_wait(self, port)
        command = _gateway('127.0.0.1', port, FILE_AUTH_OFF)
        with _Process(self, command, port=port, timeout=150) as gateway:
            gateway.assert_answers('127.0.0.1', expect_wait=True)
            self.assertIn('waiting up to 70 s', gateway.output)
            # A later process of the same user that binds with SO_REUSEPORT must not share it.
            late = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.addCleanup(late.close)
            late.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            with self.assertRaises(OSError) as raised:
                late.bind(('127.0.0.1', port))
            self.assertEqual(raised.exception.errno, errno.EADDRINUSE)
            self.assertTrue(_listener_pids(port) <= _tree(gateway.pid))

    def test_a_short_wait_ends_with_the_cause_and_the_wait_named(self):
        port = _clean_port()
        _leave_time_wait(self, port)
        command = _gateway('127.0.0.1', port, FILE_AUTH_OFF, ['-p', 'server.bind_wait_sec:=2'])
        with _Process(self, command, port=port) as gateway:
            gateway.wait_exit()
            gateway.assert_bind_failed(
                '127.0.0.1', 'Address already in use (waited 2.')
            waited = gateway.exit_at - gateway.wait_seen_at
            self.assertTrue(1.5 < waited < 5, f'waited {waited:.1f} s')
            self.assertIn('server.bind_wait_sec', gateway.output)

    def test_no_wait_exits_at_once(self):
        port = _clean_port()
        _leave_time_wait(self, port)
        command = _gateway('127.0.0.1', port, FILE_AUTH_OFF, ['-p', 'server.bind_wait_sec:=0'])
        with _Process(self, command, port=port) as gateway:
            gateway.assert_bind_failed('127.0.0.1', 'Address already in use (no live listener')
            self.assertNotIn(WAIT_LINE, gateway.output)

    def test_a_live_listener_that_appears_during_the_wait_ends_it(self):
        port = _clean_port()
        _leave_time_wait(self, port)
        command = _gateway('127.0.0.1', port, FILE_AUTH_OFF)
        with _Process(self, command, port=port, until=WAIT_LINE, timeout=90) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-400:])
            self.assertIn(WAIT_LINE, gateway.output)
            # Over the same TIME_WAIT a SO_REUSEPORT process can bind, as a released gateway does.
            with _Listener(self, port):
                gateway.wait_exit()
                gateway.assert_bind_failed('127.0.0.1', 'Address already in use')
                self.assertNotIn('waited', gateway.output.split(BIND_REFUSAL)[1])
                self.assertEqual(len(_listener_pids(port)), 1)

    def test_a_stop_during_the_wait_ends_the_start_at_once(self):
        for name, number in (('SIGTERM', signal.SIGTERM), ('SIGINT', signal.SIGINT)):
            with self.subTest(signal=name):
                port = _clean_port()
                _leave_time_wait(self, port)
                command = _gateway('127.0.0.1', port, FILE_AUTH_OFF)
                with _Process(self, command, port=port, until=WAIT_LINE, timeout=90) as gateway:
                    self.assertIn(WAIT_LINE, gateway.output)
                    nodes = [pid for pid in _tree(gateway.pid) if _is_gateway_node(pid)]
                    self.assertEqual(len(nodes), 1, f'gateway_node processes: {nodes}')
                    sent = time.monotonic()
                    os.kill(nodes[0], number)
                    self.assertEqual(gateway.wait_exit(), 0, gateway.output[-400:])
                    elapsed = gateway.exit_at - sent
                    self.assertLess(elapsed, 2, f'{name}: stopped after {elapsed:.1f} s')
                    self.assertIn('start stopped during the bind wait', gateway.output)
                    self.assertNotIn(READY_LINE, gateway.output)
                    self.assertNotIn(BOUND_LINE, gateway.output)

    def test_a_live_so_reuseport_listener_is_never_shared(self):
        port = _clean_port()
        with _StandIn(self, port, hold=True) as old:
            with _Process(self, _gateway('127.0.0.1', port, FILE_AUTH_OFF), port=port) as gateway:
                gateway.assert_bind_failed('127.0.0.1', 'Address already in use')
                self.assertNotIn(WAIT_LINE, gateway.output)
            self.assertEqual(_listener_pids(port), {old.pid}, 'the old listener lost its port')

    def test_a_restart_of_the_new_gateway_needs_no_wait(self):
        port = _clean_port()
        with _Process(self, _gateway('127.0.0.1', port, FILE_AUTH_OFF), port=port) as first:
            first.assert_answers('127.0.0.1')
            # The server closes first on `Connection: close`, so its side lands in TIME_WAIT.
            _HTTP.get(f'http://127.0.0.1:{port}{API_BASE_PATH}/health', timeout=15,
                      headers={'Connection': 'close'})
            _wait_for_time_wait(self, port, 'no TIME_WAIT entry after the closed request')
            os.killpg(os.getpgid(first.pid), signal.SIGINT)
            first.wait_exit(timeout=60)
        self.assertTrue(_time_wait(port), 'the TIME_WAIT entry is gone: nothing was tested')
        with _Process(self, _gateway('127.0.0.1', port, FILE_AUTH_OFF), port=port) as second:
            second.assert_answers('127.0.0.1', expect_wait=False)


class TestBindWaitRange(unittest.TestCase):
    """server.bind_wait_sec is 0 to 300; outside that the gateway does not start."""

    def test_outside_the_range_refuses_and_names_the_parameter(self):
        for value in ('-1', '301'):
            with self.subTest(value=value):
                port = _free_port()
                command = _gateway('127.0.0.1', port, FILE_AUTH_OFF,
                                   ['-p', f'server.bind_wait_sec:={value}'])
                with _Process(self, command, port=port) as gateway:
                    self.assertIsNotNone(gateway.rc, gateway.output[-400:])
                    self.assertNotEqual(gateway.rc, 0)
                    self.assertIn('server.bind_wait_sec', gateway.output)
                    self.assertNotIn(READY_LINE, gateway.output)

    def test_the_upper_bound_starts(self):
        port = _free_port()
        command = _gateway('127.0.0.1', port, FILE_AUTH_OFF, ['-p', 'server.bind_wait_sec:=300'])
        with _Process(self, command, port=port) as gateway:
            gateway.assert_answers('127.0.0.1')


class TestLaunchArgument(unittest.TestCase):
    """P6, L1: the launch files take allow_unauthenticated_remote and end with the gateway."""

    @staticmethod
    def _launch(file, port, *args):
        return ['ros2', 'launch', 'ros2_medkit_gateway', file, f'server_port:={port}',
                'server_host:=0.0.0.0', *args]

    @staticmethod
    def _bringup_off():
        return ['enable_log_bridge:=false', 'enable_action_status_bridge:=false',
                'enable_fault_manager:=false']

    def _process(self, file, *args, env=None):
        port = _free_port()
        return _Process(self, self._launch(file, port, *args), env, timeout=90, port=port)

    def test_bringup_without_the_argument_refuses(self):
        with self._process('bringup.launch.py', *self._bringup_off()) as launched:
            # The launch ends, and fails, with the gateway.
            launched.assert_refused('0.0.0.0')

    def test_bringup_with_the_argument_starts(self):
        with self._process('bringup.launch.py', 'allow_unauthenticated_remote:=true',
                           *self._bringup_off()) as launched:
            launched.assert_answers('0.0.0.0', expect_open_warning=True)

    def test_gateway_launch_pair(self):
        with self._process('gateway.launch.py') as launched:
            launched.assert_refused('0.0.0.0')
        with self._process('gateway.launch.py', 'allow_unauthenticated_remote:=true') as launched:
            launched.assert_answers('0.0.0.0', expect_open_warning=True)

    def test_a_config_file_opt_out_survives_the_false_default(self):
        config = f'config_file:={LAUNCH_CONFIG_OPT_OUT}'
        with self._process('gateway.launch.py', config) as launched:
            launched.assert_answers('0.0.0.0', expect_open_warning=True)

    def test_an_explicit_false_revokes_a_config_file_opt_out(self):
        config = f'config_file:={LAUNCH_CONFIG_OPT_OUT}'
        for file in ('gateway.launch.py', 'gateway_https.launch.py'):
            with self.subTest(file=file):
                revoke = 'allow_unauthenticated_remote:=false'
                with self._process(file, config, revoke) as launched:
                    launched.assert_refused('0.0.0.0')

    def test_the_https_launch_ends_with_a_failure_when_the_start_is_refused(self):
        with self._process('gateway_https.launch.py') as launched:
            launched.assert_refused('0.0.0.0')
        with self._process('gateway_https.launch.py', 'allow_unauthenticated_remote:=true',
                           f'cert_dir:={_PARAMS_DIR}/certs') as launched:
            launched.assert_answers('0.0.0.0', expect_open_warning=True,
                                    probe=f'https://127.0.0.1:{launched.port}')

    def test_bringup_loopback_needs_nothing(self):
        port = _free_port()
        command = ['ros2', 'launch', 'ros2_medkit_gateway', 'bringup.launch.py',
                   f'server_port:={port}', *self._bringup_off()]
        with _Process(self, command, timeout=90, port=port) as launched:
            launched.assert_answers('127.0.0.1')

    def test_sigint_to_the_launch_ends_it_cleanly_and_leaves_no_gateway(self):
        with self._process('gateway.launch.py', 'allow_unauthenticated_remote:=true') as launched:
            launched.assert_answers('0.0.0.0', expect_open_warning=True)
            tree = _tree(launched.pid)
            self.assertGreater(len(tree), 1, 'the launch has no child process')
            os.kill(launched.pid, signal.SIGINT)
            self.assertEqual(launched.wait_exit(timeout=60), 0, launched.output[-600:])
            leftovers = [pid for pid in tree if _alive(pid)]
            self.assertEqual(leftovers, [], 'processes of the launch are still alive')

    def test_sigint_to_the_gateway_alone_ends_the_launch_with_zero(self):
        with self._process('gateway.launch.py', 'allow_unauthenticated_remote:=true') as launched:
            launched.assert_answers('0.0.0.0', expect_open_warning=True)
            tree = _tree(launched.pid) - {launched.pid}
            gateways = [pid for pid in tree if _is_gateway_node(pid)]
            self.assertEqual(len(gateways), 1, f'gateway_node processes in the tree: {gateways}')
            os.kill(gateways[0], signal.SIGINT)
            self.assertEqual(launched.wait_exit(timeout=60), 0, launched.output[-600:])
            self.assertFalse(_alive(gateways[0]))


def _is_gateway_node(pid):
    """Return True when `pid` runs the gateway_node executable (checked within a launch tree)."""
    try:
        with open(f'/proc/{pid}/cmdline', 'rb') as handle:
            argv0 = handle.read().split(b'\0')[0]
    except OSError:
        return False
    return os.path.basename(argv0) == b'gateway_node'


@launch_testing.post_shutdown_test()
class TestProtectedModeShutdown(unittest.TestCase):

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(_PARAMS_DIR, ignore_errors=True)

    def test_exit_codes(self, proc_info, resident):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=resident)

    def test_nothing_left_listening(self):
        self.assertFalse(any(_listening(port) for port in [PORT, *BAND]))
