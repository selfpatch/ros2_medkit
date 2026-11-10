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

"""The gateway node reads MEDKIT_JWT_SECRET, MEDKIT_CLIENTS and MEDKIT_AUTH_DISABLED.

The gateways here start with no entrypoint and no launch file, only an
environment, so the rule is proven for every way of starting the node. Each
case pairs the environment with a params file that says the opposite, because
a rule that only agrees with the file proves nothing about precedence.

Three gateways run for the whole file (closed by the environment, opened by the
environment, closed by a file alone). The sweep classes start further gateways
as subprocesses, one per case, because a refusal is observed as an exit code.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import ast
import base64
import hashlib
import hmac
import os
import re
import select
import shutil
import signal
import subprocess
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

CLOSED_PORT = get_test_port(0)
DISABLED_PORT = get_test_port(1)
# Subprocess gateways take this port one at a time.
SWEEP_PORT = get_test_port(2)
FILE_CLOSED_PORT = get_test_port(3)

# How long `ros2 param` may spend discovering a gateway over DDS.
PARAM_DISCOVERY_TIMEOUT = 30
READY_LINE = 'Medkit Gateway ready on'
START_LINE = 'Initializing ROS 2 Medkit Gateway'

CLOSED_BASE_URL = f'http://127.0.0.1:{CLOSED_PORT}{API_BASE_PATH}'
DISABLED_BASE_URL = f'http://127.0.0.1:{DISABLED_PORT}{API_BASE_PATH}'
FILE_CLOSED_BASE_URL = f'http://127.0.0.1:{FILE_CLOSED_PORT}{API_BASE_PATH}'
SWEEP_BASE_URL = f'http://127.0.0.1:{SWEEP_PORT}{API_BASE_PATH}'

JWT_SECRET = 'an_environment_supplied_secret_of_at_least_32_chars'
FILE_JWT_SECRET = 'a_file_supplied_secret_of_at_least_32_characters'
SECRET_32 = 'k' * 32
SECRET_31 = 'j' * 31
CLIENT_ID = 'envclient'
CLIENT_SECRET = 'env_client_secret'
FILE_CLIENT_ID = 'fileclient'
FILE_CLIENT_SECRET = 'a_file_supplied_client_secret'
# The secret half of a two-field entry, and of `id:role:secret` entries, whose
# secret lands in the role field.
MALFORMED_CLIENT_SECRET = 's3cret_that_must_not_be_logged'
SWAPPED_FILE_SECRET = 's3cret_swapped_in_the_file_zz'
SWAPPED_ENV_SECRET = 's3cret_swapped_in_the_env_zz'
PEER_HEADER_PARAM = 'aggregation.peer_auth_header'
PEER_AUTH_HEADER = 'Bearer zz_peer_token_that_must_not_be_served'
SECOND_SECRET = 'second_entry_secret_zz'

# Every value that must never reach a gateway's output.
SECRETS = (
    JWT_SECRET, FILE_JWT_SECRET, SECRET_32, SECRET_31, CLIENT_SECRET,
    FILE_CLIENT_SECRET, MALFORMED_CLIENT_SECRET, SWAPPED_FILE_SECRET,
    SWAPPED_ENV_SECRET, PEER_AUTH_HEADER, SECOND_SECRET,
)

_PARAMS_DIR = tempfile.mkdtemp(prefix='medkit_env_auth_')

# A params file is applied only to the node whose name heads it.
ENV_CLOSED_NODE = 'gateway_env_closed'
ENV_DISABLED_NODE = 'gateway_env_disabled'
FILE_CLOSED_NODE = 'gateway_file_closed'

# A key the environment never touches, with a different value per file; it
# shows that the file reached its gateway at all.
CLOSED_FILE_TOKEN_EXPIRY = 1234
DISABLED_FILE_TOKEN_EXPIRY = 2345


def _write_params(name, node_name, auth_enabled, require_auth_for, token_expiry=None,
                  extra_clients=()):
    """Write a params file stating an auth posture for the environment to override."""
    path = os.path.join(_PARAMS_DIR, name)
    clients = [f'{FILE_CLIENT_ID}:{FILE_CLIENT_SECRET}:viewer', *extra_clients]
    lines = [
        f'{node_name}:',
        '  ros__parameters:',
        '    auth:',
        f'      enabled: {"true" if auth_enabled else "false"}',
        f'      require_auth_for: "{require_auth_for}"',
        f'      jwt_secret: "{FILE_JWT_SECRET}"',
        '      clients:',
        *[f'        - "{c}"' for c in clients],
    ]
    if token_expiry is not None:
        lines.append(f'      token_expiry_seconds: {token_expiry}')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    return path


AUTH_OFF_PARAMS = _write_params(
    'auth-off.yaml', ENV_CLOSED_NODE, False, 'write', CLOSED_FILE_TOKEN_EXPIRY,
    [f'{CLIENT_ID}:{FILE_CLIENT_SECRET}:viewer'])
AUTH_ON_PARAMS = _write_params('auth-on.yaml', ENV_DISABLED_NODE, True, 'write',
                               DISABLED_FILE_TOKEN_EXPIRY)
FILE_CLOSED_PARAMS = _write_params(
    'file-closed.yaml', FILE_CLOSED_NODE, True, 'all', None,
    [f'{CLIENT_ID}:{CLIENT_SECRET}:admin', f'twofield:{MALFORMED_CLIENT_SECRET}',
     f'swapped:admin:{SWAPPED_FILE_SECRET}'])
# For the subprocess gateways, which carry the default node name.
SWEEP_OFF_PARAMS = _write_params('sweep-off.yaml', '/**', False, 'write')
SWEEP_ON_PARAMS = _write_params('sweep-on.yaml', '/**', True, 'all')


@pytest.mark.launch_test
def generate_test_description():
    """Three gateways, differing in what their environments and files say."""
    closed = create_gateway_node(
        port=CLOSED_PORT,
        name=ENV_CLOSED_NODE,
        parameter_files=[AUTH_OFF_PARAMS],
        extra_params={'server.host': '127.0.0.1'},
        extra_env={
            'MEDKIT_JWT_SECRET': JWT_SECRET,
            'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin,'
                              f'envswapped:admin:{SWAPPED_ENV_SECRET}',
        },
    )
    disabled = create_gateway_node(
        port=DISABLED_PORT,
        name=ENV_DISABLED_NODE,
        parameter_files=[AUTH_ON_PARAMS],
        extra_params={'server.host': '127.0.0.1',
                      'aggregation.peer_auth_header': PEER_AUTH_HEADER},
        extra_env={
            'MEDKIT_JWT_SECRET': JWT_SECRET,
            'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin',
            'MEDKIT_AUTH_DISABLED': '1',
        },
    )
    file_closed = create_gateway_node(
        port=FILE_CLOSED_PORT,
        name=FILE_CLOSED_NODE,
        parameter_files=[FILE_CLOSED_PARAMS],
        extra_params={'server.host': '127.0.0.1'},
    )
    return launch.LaunchDescription([
        closed,
        disabled,
        file_closed,
        launch_testing.actions.ReadyToTest(),
    ]), {'closed': closed, 'disabled': disabled, 'file_closed': file_closed}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _ros2_param(*args):
    """Run `ros2 param ...` and return (rc, output).

    Retries while the CLI reports the node missing: the HTTP port answering says
    nothing about whether DDS discovery of the parameter services has finished.
    """
    deadline = time.monotonic() + PARAM_DISCOVERY_TIMEOUT
    while True:
        completed = subprocess.run(
            ['ros2', 'param', *args],
            capture_output=True, text=True, timeout=30, check=False,
        )
        output = completed.stdout + completed.stderr
        if 'Node not found' not in output or time.monotonic() >= deadline:
            return completed.returncode, output
        time.sleep(1)


def _param_value(node, name):
    """Return the parameter's value as Python data, read from `ros2 param get`."""
    rc, output = _ros2_param('get', node, name)
    match = re.search(r'values? (?:is|are): ?(.*)', output)
    assert rc == 0 and match, output
    text = match.group(1).strip()
    if 'values are' in output:
        return ast.literal_eval(text)
    if text in ('True', 'False'):
        return text == 'True'
    return text


def _b64url(data):
    return base64.urlsafe_b64encode(data).rstrip(b'=')


def _signed_with(token, secret):
    """Return True when `token` carries a valid HS256 signature under `secret`."""
    head, body, signature = token.split('.')
    expected = hmac.new(secret.encode(), f'{head}.{body}'.encode(), hashlib.sha256).digest()
    return hmac.compare_digest(_b64url(expected), signature.encode())


def _authorize(base_url, client_id, client_secret):
    return requests.post(
        f'{base_url}/auth/authorize',
        json={'grant_type': 'client_credentials', 'client_id': client_id,
              'client_secret': client_secret},
        timeout=30,
    )


def _status(base_url, client_id, client_secret):
    return _authorize(base_url, client_id, client_secret).status_code


def _assert_no_secret(testcase, output):
    """Assert the output is a real gateway log, then that it holds no secret."""
    testcase.assertIn(
        START_LINE, output,
        f'no gateway startup line in the capture, so a clean scan proves nothing: {output[-400:]}')
    for secret in SECRETS:
        testcase.assertNotIn(secret, output, f'a secret reached the output: {secret[:8]}...')


class _Gateway:
    """A `gateway_node` subprocess on the sweep port, started with this environment.

    ``rc`` is None when the gateway announced itself ready (or was still running
    at the timeout), else its exit code. Leaving the block kills its process
    group: `ros2 run` execs the node as a child, so killing the wrapper alone
    would leave a gateway holding the port.
    """

    def __init__(self, testcase, env_extra=None, extra_args=(), params_file=None, timeout=45):
        self.testcase = testcase
        self.env = {k: v for k, v in os.environ.items() if not k.startswith('MEDKIT_')}
        self.env.update(env_extra or {})
        self.command = [
            'ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
            '-p', f'server.port:={SWEEP_PORT}', '-p', 'server.host:=127.0.0.1',
            *(['--params-file', params_file] if params_file else []),
            *extra_args,
        ]
        self.timeout = timeout
        self.rc = None
        self._chunks = []
        self._process = None

    @property
    def output(self):
        return b''.join(self._chunks).decode(errors='replace')

    def _read_until(self, marker_or_exit):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            readable, _, _ = select.select([self._process.stdout], [], [], 0.5)
            if not readable:
                continue
            chunk = os.read(self._process.stdout.fileno(), 65536)
            if not chunk:
                return False
            self._chunks.append(chunk)
            if marker_or_exit.encode() in b''.join(self._chunks[-2:]):
                return True
        return True

    def __enter__(self):
        self._process = subprocess.Popen(
            self.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=self.env, start_new_session=True,
        )
        if not self._read_until(READY_LINE):
            self._process.wait(timeout=15)
            self.rc = self._process.returncode
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.rc is None:
            os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            try:
                self._process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
        rest, _ = self._process.communicate()
        if rest:
            self._chunks.append(rest)
        if exc_type is None:
            _assert_no_secret(self.testcase, self.output)
        return False


# ---------------------------------------------------------------------------
# Long-running gateways
# ---------------------------------------------------------------------------

class TestEnvironmentClosesTheNode(GatewayTestCase):
    """MEDKIT_JWT_SECRET closes a gateway whose params file says auth is off."""

    BASE_URL = CLOSED_BASE_URL

    def test_00_the_file_reached_this_gateway(self):
        """Premise: every auth key of the file is overridden but the expiry."""
        rc, output = _ros2_param('get', f'/{ENV_CLOSED_NODE}', 'auth.token_expiry_seconds')
        self.assertEqual(rc, 0, output)
        self.assertIn(str(CLOSED_FILE_TOKEN_EXPIRY), output, output)

    def test_01_every_route_is_closed_not_only_the_writes(self):
        """The file says auth off and require_auth_for write; both lose."""
        for path in ('/areas', '/components', '/apps', '/health', '/'):
            with self.subTest(path=path):
                resp = requests.get(f'{CLOSED_BASE_URL}{path}', timeout=15)
                self.assertEqual(resp.status_code, 401, f'anonymous GET {path}: {resp.text[:200]}')

    def test_02_a_write_is_closed_too(self):
        resp = requests.post(f'{CLOSED_BASE_URL}/apps/none/operations/x/executions',
                             json={}, timeout=15)
        self.assertEqual(resp.status_code, 401, resp.text[:200])

    def test_03_the_environment_credential_issues_a_working_token(self):
        """The mirror: a gateway closed to everyone would pass the cases above."""
        token = _authorize(CLOSED_BASE_URL, CLIENT_ID, CLIENT_SECRET)
        self.assertEqual(token.status_code, 200, token.text)
        self.assertEqual(token.json().get('scope'), 'admin', token.text[:300])
        resp = requests.get(
            f'{CLOSED_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {token.json()["access_token"]}'}, timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_04_the_token_is_signed_with_the_environment_secret(self):
        """Not with the file's secret, which the params file also carries."""
        token = _authorize(CLOSED_BASE_URL, CLIENT_ID, CLIENT_SECRET).json()['access_token']
        self.assertTrue(_signed_with(token, JWT_SECRET), 'not signed with MEDKIT_JWT_SECRET')
        self.assertFalse(_signed_with(token, FILE_JWT_SECRET), 'signed with the file secret')

    def test_05_the_file_credential_is_gone(self):
        """MEDKIT_CLIENTS replaces auth.clients; it does not add to it.

        The file names the same id with another secret, and a second id.
        """
        for client_id, secret in ((CLIENT_ID, FILE_CLIENT_SECRET),
                                  (FILE_CLIENT_ID, FILE_CLIENT_SECRET)):
            with self.subTest(client=client_id):
                self.assertNotEqual(_status(CLOSED_BASE_URL, client_id, secret), 200)

    def test_06_a_swapped_entry_registered_nothing(self):
        for secret in (SWAPPED_ENV_SECRET, 'admin'):
            self.assertNotEqual(_authorize(CLOSED_BASE_URL, 'envswapped', secret).status_code, 200)


class TestAuthDisabledWinsOverEverything(GatewayTestCase):
    """MEDKIT_AUTH_DISABLED=1 beats the secret and a file that asks for auth."""

    BASE_URL = DISABLED_BASE_URL

    def test_00_the_file_reached_this_gateway(self):
        rc, output = _ros2_param('get', f'/{ENV_DISABLED_NODE}', 'auth.token_expiry_seconds')
        self.assertEqual(rc, 0, output)
        self.assertIn(str(DISABLED_FILE_TOKEN_EXPIRY), output, output)

    def test_01_every_route_is_answered(self):
        for path in ('/areas', '/health'):
            with self.subTest(path=path):
                resp = requests.get(f'{DISABLED_BASE_URL}{path}', timeout=15)
                self.assertEqual(resp.status_code, 200, resp.text[:200])

    def test_02_the_environment_credential_gets_no_token(self):
        """Authentication is off, and the gateway holds no signing secret."""
        self.assertNotEqual(_status(DISABLED_BASE_URL, CLIENT_ID, CLIENT_SECRET), 200)

    def test_03_no_secret_is_served_with_authentication_off(self):
        secret = _param_value(f'/{ENV_DISABLED_NODE}', 'auth.jwt_secret')
        self.assertEqual(secret, '<set at start>')
        clients = _param_value(f'/{ENV_DISABLED_NODE}', 'auth.clients')
        self.assertEqual(clients, [f'{FILE_CLIENT_ID}:<set at start>:viewer'])

    def test_04_the_peer_auth_header_reads_as_a_sentinel(self):
        self.assertEqual(
            _param_value(f'/{ENV_DISABLED_NODE}', PEER_HEADER_PARAM), '<set at start>')

    def test_05_the_peer_auth_header_cannot_be_set_at_runtime(self):
        _, output = _ros2_param(
            'set', f'/{ENV_DISABLED_NODE}', PEER_HEADER_PARAM, 'Bearer other')
        self.assertIn('read at start', output, output)
        self.assertEqual(
            _param_value(f'/{ENV_DISABLED_NODE}', PEER_HEADER_PARAM), '<set at start>')

    def test_06_introspection_reports_the_open_posture(self):
        self.assertIs(_param_value(f'/{ENV_DISABLED_NODE}', 'auth.enabled'), False)
        self.assertEqual(_param_value(f'/{ENV_DISABLED_NODE}', 'auth.require_auth_for'), 'none')


class TestIntrospectionAgreesWithEnforcement(GatewayTestCase):
    """`ros2 param get` answers what the gateway enforces, and refuses writes."""

    BASE_URL = CLOSED_BASE_URL
    NODE = f'/{ENV_CLOSED_NODE}'

    def test_01_the_closed_gateway_reports_the_posture_it_serves(self):
        self.assertIs(_param_value(self.NODE, 'auth.enabled'), True)
        self.assertEqual(_param_value(self.NODE, 'auth.require_auth_for'), 'all')

    def test_02_secrets_read_back_as_sentinels(self):
        self.assertEqual(_param_value(self.NODE, 'auth.jwt_secret'), '<from MEDKIT_JWT_SECRET>')
        self.assertEqual(
            _param_value(self.NODE, 'auth.clients'),
            [f'{CLIENT_ID}:<from MEDKIT_CLIENTS>:admin'])

    def test_03_every_auth_parameter_is_refused_and_unchanged(self):
        for name, value in (('auth.enabled', 'false'), ('auth.require_auth_for', 'none'),
                            ('auth.jwt_secret', 'another_secret_zz'),
                            ('aggregation.peer_auth_header', 'Bearer other')):
            with self.subTest(parameter=name):
                _, output = _ros2_param('set', self.NODE, name, value)
                self.assertIn('read at start', output, output)
                self.assertNotIn('Set parameter successful', output, output)
        self.assertIs(_param_value(self.NODE, 'auth.enabled'), True)
        self.assertEqual(_param_value(self.NODE, 'auth.jwt_secret'), '<from MEDKIT_JWT_SECRET>')
        resp = requests.get(f'{CLOSED_BASE_URL}/areas', timeout=15)
        self.assertEqual(resp.status_code, 401, resp.text)

    def test_04_a_non_auth_parameter_is_untouched_by_the_guard(self):
        _, output = _ros2_param('set', self.NODE, 'refresh_interval_ms', '3000')
        self.assertIn('Set parameter successful', output, output)
        self.assertEqual(_param_value(self.NODE, 'refresh_interval_ms'), '3000')

    def test_05_a_param_load_applies_what_it_may_and_refuses_the_rest(self):
        """`ros2 param load` sets each key in turn: the auth entry is refused, the rest applies."""
        path = os.path.join(_PARAMS_DIR, 'runtime-load.yaml')
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(
                f'/{ENV_CLOSED_NODE}:\n'
                '  ros__parameters:\n'
                '    auth:\n'
                '      enabled: false\n'
                '    refresh_interval_ms: 4500\n'
            )
        _, output = _ros2_param('load', self.NODE, path)
        self.assertIn('read at start', output, output)
        self.assertEqual(_param_value(self.NODE, 'refresh_interval_ms'), '4500')
        self.assertIs(_param_value(self.NODE, 'auth.enabled'), True)
        self.assertEqual(requests.get(f'{CLOSED_BASE_URL}/areas', timeout=15).status_code, 401)

    def test_06_an_atomic_batch_mixing_the_two_is_refused_whole(self):
        before = _param_value(self.NODE, 'refresh_interval_ms')
        script = (
            'import rclpy\n'
            'from rclpy.node import Node\n'
            'from rclpy.parameter import Parameter\n'
            'from rcl_interfaces.srv import SetParametersAtomically\n'
            'rclpy.init()\n'
            'n = Node("batch_probe")\n'
            f'c = n.create_client(SetParametersAtomically,\n'
            f'                    "{self.NODE}/set_parameters_atomically")\n'
            'assert c.wait_for_service(timeout_sec=30.0), "no service"\n'
            'req = SetParametersAtomically.Request()\n'
            'req.parameters = [\n'
            '    Parameter("refresh_interval_ms", Parameter.Type.INTEGER, 6100)\n'
            '        .to_parameter_msg(),\n'
            '    Parameter("auth.enabled", Parameter.Type.BOOL, False)\n'
            '        .to_parameter_msg(),\n'
            ']\n'
            'f = c.call_async(req)\n'
            'rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)\n'
            'r = f.result()\n'
            'print("SUCCESSFUL", r.result.successful)\n'
            'print("REASON", r.result.reason)\n'
            'rclpy.shutdown()\n'
        )
        completed = subprocess.run(
            ['python3', '-c', script], capture_output=True, text=True, timeout=90, check=False)
        output = completed.stdout + completed.stderr
        self.assertIn('SUCCESSFUL False', output, output[-600:])
        self.assertIn('read at start', output, output[-600:])
        self.assertEqual(_param_value(self.NODE, 'refresh_interval_ms'), before)


class TestAFileSuppliedSecretIsAlsoRedacted(GatewayTestCase):
    """A file-closed gateway with no MEDKIT_* variable: same read-back, same behaviour."""

    BASE_URL = FILE_CLOSED_BASE_URL
    NODE = f'/{FILE_CLOSED_NODE}'

    def test_01_the_file_alone_closes_the_gateway(self):
        resp = requests.get(f'{FILE_CLOSED_BASE_URL}/areas', timeout=15)
        self.assertEqual(resp.status_code, 401)

    def test_02_the_posture_reads_back(self):
        self.assertIs(_param_value(self.NODE, 'auth.enabled'), True)
        self.assertEqual(_param_value(self.NODE, 'auth.require_auth_for'), 'all')

    def test_03_the_file_secret_reads_back_as_a_sentinel(self):
        self.assertEqual(_param_value(self.NODE, 'auth.jwt_secret'), '<set at start>')

    def test_04_the_file_client_secrets_read_back_as_sentinels(self):
        self.assertEqual(
            _param_value(self.NODE, 'auth.clients'),
            [f'{FILE_CLIENT_ID}:<set at start>:viewer', f'{CLIENT_ID}:<set at start>:admin'])

    def test_05_the_credential_still_works_and_signs_with_the_file_secret(self):
        token = _authorize(FILE_CLOSED_BASE_URL, CLIENT_ID, CLIENT_SECRET)
        self.assertEqual(token.status_code, 200, token.text)
        self.assertTrue(_signed_with(token.json()['access_token'], FILE_JWT_SECRET))
        resp = requests.get(
            f'{FILE_CLOSED_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {token.json()["access_token"]}'}, timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_06_the_malformed_entries_registered_no_client(self):
        for client_id, secret in (('twofield', MALFORMED_CLIENT_SECRET),
                                  ('swapped', SWAPPED_FILE_SECRET), ('swapped', 'admin')):
            self.assertNotEqual(_status(FILE_CLOSED_BASE_URL, client_id, secret), 200)


# ---------------------------------------------------------------------------
# One gateway per case
# ---------------------------------------------------------------------------

def _anonymous_status(path='/areas'):
    return requests.get(f'{SWEEP_BASE_URL}{path}', timeout=15).status_code


class TestEnvironmentSweep(unittest.TestCase):
    """The three variables across their input space, one gateway per case."""

    def test_01_no_variable_set_leaves_main_behaviour(self):
        with _Gateway(self) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_anonymous_status('/areas'), 200)
            self.assertEqual(_anonymous_status('/health'), 200)
            self.assertIs(_param_value('/ros2_medkit_gateway', 'auth.enabled'), False)
            self.assertEqual(_param_value('/ros2_medkit_gateway', 'auth.require_auth_for'), 'none')
            self.assertEqual(_param_value('/ros2_medkit_gateway', 'auth.jwt_secret'), '')
            self.assertEqual(_param_value('/ros2_medkit_gateway', 'auth.clients'), [])

    def test_02_a_file_closed_gateway_without_variables_is_unchanged(self):
        with _Gateway(self, params_file=SWEEP_ON_PARAMS) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_anonymous_status('/areas'), 401)
            token = _authorize(SWEEP_BASE_URL, FILE_CLIENT_ID, FILE_CLIENT_SECRET)
            self.assertEqual(token.status_code, 200, token.text)
            self.assertTrue(_signed_with(token.json()['access_token'], FILE_JWT_SECRET))

    def test_03_the_secret_length_boundary(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': SECRET_31}) as gateway:
            self.assertEqual(gateway.rc, 1, gateway.output[-600:])
            self.assertIn('MEDKIT_JWT_SECRET', gateway.output)
        with _Gateway(self, {'MEDKIT_JWT_SECRET': SECRET_32}) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_anonymous_status('/areas'), 401)

    def test_04_an_empty_secret_refuses_to_start_and_names_the_variable(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': ''}, params_file=SWEEP_ON_PARAMS) as gateway:
            self.assertEqual(gateway.rc, 1, gateway.output[-600:])
            self.assertIn('MEDKIT_JWT_SECRET is set but empty', gateway.output)
            self.assertNotIn(READY_LINE, gateway.output)
        with _Gateway(self, {'MEDKIT_JWT_SECRET': '',
                             'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin'}) as gateway:
            self.assertEqual(gateway.rc, 1, gateway.output[-600:])

    def test_05_disabled_wins_over_an_empty_secret(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': '', 'MEDKIT_AUTH_DISABLED': '1'}) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_anonymous_status('/areas'), 200)

    def test_06_only_the_literal_one_disables(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_AUTH_DISABLED': '1'},
                      params_file=SWEEP_ON_PARAMS) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_anonymous_status('/areas'), 200)
        for value in ('0', 'true', ''):
            with self.subTest(value=value):
                with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET,
                                     'MEDKIT_AUTH_DISABLED': value},
                              params_file=SWEEP_OFF_PARAMS) as gateway:
                    self.assertIsNone(gateway.rc, gateway.output[-600:])
                    self.assertEqual(_anonymous_status('/areas'), 401)
        for value in ('0', 'true', ''):
            with self.subTest(value=value, secret=False):
                with _Gateway(self, {'MEDKIT_AUTH_DISABLED': value},
                              params_file=SWEEP_ON_PARAMS) as gateway:
                    self.assertIsNone(gateway.rc, gateway.output[-600:])
                    self.assertEqual(_anonymous_status('/areas'), 401)

    def test_07_the_secret_beats_every_params_source(self):
        """A `-p` argument comes after the file, and the environment still wins."""
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET,
                             'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin'},
                      params_file=SWEEP_OFF_PARAMS,
                      extra_args=('-p', 'auth.enabled:=false',
                                  '-p', 'auth.require_auth_for:=write',
                                  '-p', f'auth.jwt_secret:={SECOND_SECRET}_pad_to_32')) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_anonymous_status('/areas'), 401)
            token = _authorize(SWEEP_BASE_URL, CLIENT_ID, CLIENT_SECRET)
            self.assertEqual(token.status_code, 200, token.text)
            self.assertTrue(_signed_with(token.json()['access_token'], JWT_SECRET))

    def test_08_a_secret_under_rs256_names_both_settings_and_no_value(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET,
                             'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin'},
                      extra_args=('-p', 'auth.jwt_algorithm:=RS256')) as gateway:
            self.assertEqual(gateway.rc, 1, gateway.output[-600:])
            line = next(row for row in gateway.output.splitlines() if 'Refusing to start' in row)
            self.assertIn('MEDKIT_JWT_SECRET (environment)', line)
            self.assertIn('auth.jwt_algorithm is RS256 (parameters)', line)
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET},
                      extra_args=('-p', 'auth.jwt_algorithm:=HS256')) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])

    def test_09_a_refusal_names_where_each_value_came_from(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET},
                      extra_args=('-p', 'auth.token_expiry_seconds:=0')) as gateway:
            self.assertEqual(gateway.rc, 1, gateway.output[-600:])
            line = next(row for row in gateway.output.splitlines() if 'Refusing to start' in row)
            self.assertIn('Token expiry must be positive', line)
            self.assertIn(
                'The environment supplied auth.jwt_secret (MEDKIT_JWT_SECRET); every other '
                'auth.* value came from the parameters.', line)
            self.assertNotIn('MEDKIT_CLIENTS', line)

    def test_10_clients_unset_leaves_the_files_clients(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET},
                      params_file=SWEEP_OFF_PARAMS) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            token = _authorize(SWEEP_BASE_URL, FILE_CLIENT_ID, FILE_CLIENT_SECRET)
            self.assertEqual(token.status_code, 200, token.text)

    def test_11_clients_without_a_secret_are_ignored_with_a_warning(self):
        with _Gateway(self, {'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin'},
                      params_file=SWEEP_ON_PARAMS) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertIn('MEDKIT_CLIENTS is set and MEDKIT_JWT_SECRET is unset', gateway.output)
            self.assertEqual(
                _status(SWEEP_BASE_URL, FILE_CLIENT_ID, FILE_CLIENT_SECRET), 200)
            self.assertNotEqual(_status(SWEEP_BASE_URL, CLIENT_ID, CLIENT_SECRET), 200)

    def test_12_an_empty_clients_value_means_no_clients_and_warns(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': ''},
                      params_file=SWEEP_OFF_PARAMS) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertIn('MEDKIT_CLIENTS is empty', gateway.output)
            self.assertNotEqual(
                _status(SWEEP_BASE_URL, FILE_CLIENT_ID, FILE_CLIENT_SECRET), 200)

    def test_13_a_list_refused_in_full_stops_the_gateway(self):
        with _Gateway(self, {'MEDKIT_JWT_SECRET': JWT_SECRET,
                             'MEDKIT_CLIENTS': f'nocolons,also-bad,third:{MALFORMED_CLIENT_SECRET}'
                                               ':wizard'}) as gateway:
            self.assertEqual(gateway.rc, 1, gateway.output[-600:])
            self.assertIn('every MEDKIT_CLIENTS entry was refused', gateway.output)

    def test_14_one_refused_entry_does_not_take_the_others_with_it(self):
        clients = (f'good:{CLIENT_SECRET}:admin,nocolons,'
                   f'{MALFORMED_CLIENT_SECRET}:x,swapped:admin:{SWAPPED_ENV_SECRET}')
        env = {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': clients}
        with _Gateway(self, env) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_status(SWEEP_BASE_URL, 'good', CLIENT_SECRET), 200)
            self.assertIn('MEDKIT_CLIENTS entry 2 is not', gateway.output)
            self.assertIn('MEDKIT_CLIENTS entry 3 is not', gateway.output)
            self.assertIn('MEDKIT_CLIENTS entry 4 for id "swapped" names an unknown role',
                          gateway.output)

    def test_15_a_duplicate_id_keeps_the_first_entry_and_warns(self):
        clients = f'dup:{CLIENT_SECRET}:admin,dup:{SECOND_SECRET}:viewer'
        env = {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': clients}
        with _Gateway(self, env) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertIn('repeats client id "dup"', gateway.output)
            self.assertEqual(_status(SWEEP_BASE_URL, 'dup', CLIENT_SECRET), 200)
            self.assertNotEqual(_status(SWEEP_BASE_URL, 'dup', SECOND_SECRET), 200)

    def test_16_surrounding_whitespace_is_trimmed(self):
        clients = f' one:{CLIENT_SECRET}:admin ,\ttwo:{SECOND_SECRET}:viewer\t'
        env = {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': clients}
        with _Gateway(self, env) as gateway:
            self.assertIsNone(gateway.rc, gateway.output[-600:])
            self.assertEqual(_status(SWEEP_BASE_URL, 'one', CLIENT_SECRET), 200)
            self.assertEqual(_status(SWEEP_BASE_URL, 'two', SECOND_SECRET), 200)


@launch_testing.post_shutdown_test()
class TestEnvAuthContractShutdown(unittest.TestCase):
    """Every gateway exits cleanly and none printed a secret."""

    def test_exit_codes(self, proc_info, closed, disabled, file_closed):
        for proc in (closed, disabled, file_closed):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc)

    def test_no_secret_reached_the_log(self, proc_output, closed, disabled, file_closed):
        # Concatenated with no separator: a chunk boundary can fall mid-line.
        for proc in (closed, disabled, file_closed):
            text = ''.join(out.text.decode(errors='replace') for out in proc_output[proc])
            self.assertIn(READY_LINE, text, 'the capture holds no gateway startup line')
            _assert_no_secret(self, text)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(_PARAMS_DIR, ignore_errors=True)
