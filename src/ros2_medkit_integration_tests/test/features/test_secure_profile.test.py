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

"""The secure field profile, deployed the way an operator deploys it.

Every case starts a real gateway_node subprocess from the INSTALLED
``config/gateway_params.secure.yaml``. The secrets are only in the environment
(``MEDKIT_JWT_SECRET``, ``MEDKIT_CLIENTS``); no secret is on the command line.
Only what a test host cannot provide is overridden: the port and the paths of the
certificate and key (generated here). Everything else that is security-relevant
comes from the file, and the file's own values are read back and asserted, so a
later edit that weakens it turns this test red.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import base64
import http.client
import json
import os
import select
import shutil
import signal
import ssl
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
import pytest

from ros2_medkit_test_utils.constants import API_BASE_PATH, get_test_port
from ros2_medkit_test_utils.launch_helpers import create_gateway_node
import yaml

SECURE_FILE = os.path.join(
    get_package_share_directory('ros2_medkit_gateway'), 'config', 'gateway_params.secure.yaml')

READY_LINE = 'Medkit Gateway ready on'
NODE = '/ros2_medkit_gateway'
AREAS = f'{API_BASE_PATH}/areas'
HEALTH = f'{API_BASE_PATH}/health'
VERSION_INFO = f'{API_BASE_PATH}/version-info'
WRITE_ROUTE = f'{API_BASE_PATH}/components/test/operations/calibrate/executions'
EXECUTION = f'{API_BASE_PATH}/apps/app1/operations/calibrate/executions/exec1'
JWT_SECRET = 'secure_profile_integration_secret_key_0123456789'
ADMIN = ('admin', 'admin_client_secret_9f3a')
VIEWER = ('viewer', 'viewer_client_secret_7c1d')
CLIENTS = f'{ADMIN[0]}:{ADMIN[1]}:admin,{VIEWER[0]}:{VIEWER[1]}:viewer'
UI_ORIGIN = 'https://medkit-ui.local'

# What the profile documents: limits are upper bounds, switches are exact.
CLIENT_LIMIT = 120
GLOBAL_LIMIT = 600
OPERATION_LIMIT = 30
OPERATION_PATTERNS = ['/api/v1/*/*/operations/*/executions']
# One request per pattern, built from the real route template: method, path and the
# status the gateway answers while the bucket has allowance (the entity does not exist).
OPERATION_REQUESTS = {
    '/api/v1/*/*/operations/*/executions':
        ('POST', f'{API_BASE_PATH}/components/comp1/operations/calibrate/executions', 404),
}

_DIR = tempfile.mkdtemp(prefix='medkit_secure_profile_')
_BAND = [get_test_port(offset) for offset in range(9)]
RESIDENT_PORT = get_test_port(9)
_NEXT = [0]


def _port():
    """Return the next port of the test band; every gateway of the file takes its own."""
    port = _BAND[_NEXT[0] % len(_BAND)]
    _NEXT[0] += 1
    return port


def _openssl(*args):
    subprocess.run(['openssl', *args], check=True, capture_output=True)


def _ca(name):
    key, crt = os.path.join(_DIR, f'{name}-key.pem'), os.path.join(_DIR, f'{name}.pem')
    _openssl('req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-keyout', key, '-out', crt,
             '-days', '1', '-subj', f'/CN=medkit-{name}',
             '-addext', 'basicConstraints=critical,CA:TRUE',
             '-addext', 'keyUsage=critical,keyCertSign,cRLSign')
    return key, crt


def _leaf(name, ca_key, ca_crt, cn, san=False, client=False):
    key = os.path.join(_DIR, f'{name}-key.pem')
    csr = os.path.join(_DIR, f'{name}.csr')
    crt = os.path.join(_DIR, f'{name}.pem')
    ext = os.path.join(_DIR, f'{name}.ext')
    with open(ext, 'w', encoding='utf-8') as handle:
        handle.write('basicConstraints=CA:FALSE\nkeyUsage=digitalSignature,keyEncipherment\n')
        handle.write(f'extendedKeyUsage={"clientAuth" if client else "serverAuth"}\n')
        if san:
            handle.write('subjectAltName=DNS:localhost,IP:127.0.0.1\n')
    _openssl('req', '-newkey', 'rsa:2048', '-nodes', '-keyout', key, '-out', csr,
             '-subj', f'/CN={cn}')
    _openssl('x509', '-req', '-in', csr, '-CA', ca_crt, '-CAkey', ca_key, '-CAcreateserial',
             '-out', crt, '-days', '1', '-extfile', ext)
    os.chmod(key, 0o600)
    return key, crt


CA_KEY, CA_CRT = _ca('ca')
SRV_KEY, SRV_CRT = _leaf('server', CA_KEY, CA_CRT, 'localhost', san=True)
CLI_KEY, CLI_CRT = _leaf('client', CA_KEY, CA_CRT, 'medkit-client', client=True)


def _file_values():
    """Return the installed secure file's `ros__parameters`."""
    with open(SECURE_FILE, encoding='utf-8') as handle:
        return yaml.safe_load(handle)['ros2_medkit_gateway']['ros__parameters']


FILE = _file_values()


@pytest.mark.launch_test
def generate_test_description():
    """One loopback gateway keeps the launch alive; every case starts its own subprocess."""
    resident = create_gateway_node(
        port=RESIDENT_PORT, name='secure_profile_resident',
        extra_params={'server.host': '127.0.0.1'})
    return launch.LaunchDescription([resident, launch_testing.actions.ReadyToTest()]), {}


class _Gateway:
    """A gateway_node started from the secure file; `rc` is its exit code if it ended early."""

    def __init__(self, env=None, extra_args=(), certs=True, timeout=90):
        self.port = _port()
        # Overrides: the port is the test's, the certificate paths are the generated pair
        # (the file's /etc/ros2_medkit/certs does not exist on a test host).
        self.command = ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
                        '--params-file', SECURE_FILE, '-p', f'server.port:={self.port}']
        if certs:
            self.command += ['-p', f'server.tls.cert_file:="{SRV_CRT}"',
                             '-p', f'server.tls.key_file:="{SRV_KEY}"']
        self.command += list(extra_args)
        self.env = {k: v for k, v in os.environ.items() if not k.startswith('MEDKIT_')}
        self.env.update({'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': CLIENTS}
                        if env is None else env)
        self.timeout = timeout
        self.rc = None
        self.ready = False
        self._chunks = []
        self._process = None

    @property
    def output(self):
        return b''.join(self._chunks).decode(errors='replace')

    def __enter__(self):
        self._process = subprocess.Popen(
            self.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=self.env, start_new_session=True)
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            readable, _, _ = select.select([self._process.stdout], [], [], 0.5)
            if not readable:
                continue
            chunk = os.read(self._process.stdout.fileno(), 65536)
            if not chunk:
                self._process.wait(timeout=15)
                self.rc = self._process.returncode
                break
            self._chunks.append(chunk)
            if READY_LINE.encode() in b''.join(self._chunks[-2:]):
                self.ready = True
                break
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._process.poll() is None:
            os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            try:
                self._process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
        rest, _ = self._process.communicate()
        if rest:
            self._chunks.append(rest)
        return False


def _context(max_version=None, cert=None):
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.load_verify_locations(CA_CRT)
    if max_version is not None:
        ctx.maximum_version = max_version
    if cert is not None:
        ctx.load_cert_chain(*cert)
    return ctx


def _call(port, method, path, ctx=None, headers=None, body=None, conn=None):
    """Return (status, headers, parsed body) of one HTTPS request; raise when it fails."""
    own = conn is None
    conn = conn or http.client.HTTPSConnection(
        'localhost', port, context=ctx or _context(), timeout=30)
    try:
        payload = json.dumps(body) if body is not None else None
        send = dict(headers or {})
        if payload is not None:
            send['Content-Type'] = 'application/json'
        conn.request(method, path, body=payload, headers=send)
        resp = conn.getresponse()
        raw = resp.read()
        try:
            parsed = json.loads(raw) if raw else None
        except ValueError:
            parsed = raw
        return resp.status, {k.lower(): v for k, v in resp.getheaders()}, parsed
    finally:
        if own:
            conn.close()


def _basic(client):
    return {'Authorization': 'Basic ' + base64.b64encode(
        f'{client[0]}:{client[1]}'.encode()).decode()}


def _login(port, client, ctx=None):
    """Return the token answer of a client-credentials login with HTTP Basic."""
    status, _, body = _call(port, 'POST', f'{API_BASE_PATH}/auth/authorize', ctx,
                            headers=_basic(client), body={'grant_type': 'client_credentials'})
    assert status == 200, f'login of {client[0]} failed: {status} {body}'
    return body


def _bearer(token):
    return {'Authorization': f'Bearer {token}'}


_REFUSED = (ssl.SSLError, ConnectionError, http.client.HTTPException, OSError)


def _param(verb, name, *value, timeout=45):
    """Run `ros2 param <verb>` without the daemon; retry while discovery has not found the node."""
    deadline = time.monotonic() + timeout
    while True:
        proc = subprocess.run(['ros2', 'param', verb, '--no-daemon', NODE, name, *value],
                              capture_output=True, text=True, timeout=60)
        out = proc.stdout + proc.stderr
        if 'Node not found' not in out or time.monotonic() >= deadline:
            return out
        time.sleep(1)


class _Base(unittest.TestCase):

    def assert_started(self, gw):
        self.assertTrue(gw.ready, f'not started: {gw.output[-1500:]}')
        self.assertIsNone(gw.rc)

    def assert_refused_start(self, gw):
        self.assertIsNotNone(gw.rc, gw.output[-1500:])
        self.assertNotEqual(gw.rc, 0)
        self.assertFalse(gw.ready)


class TestSecureProfile(_Base):
    """One gateway from the file, with the secrets in the environment."""

    @classmethod
    def setUpClass(cls):
        cls.gw = _Gateway().__enter__()
        cls.addClassCleanup(cls.gw.__exit__, None, None, None)
        if not cls.gw.ready:
            raise AssertionError(f'secure gateway did not start: {cls.gw.output[-1500:]}')
        cls.port = cls.gw.port
        cls.admin = _login(cls.port, ADMIN)
        cls.viewer = _login(cls.port, VIEWER)

    def get(self, path, token=None, **kwargs):
        headers = _bearer(token) if token else None
        return _call(self.port, 'GET', path, headers=headers, **kwargs)

    def test_startup_line_names_the_floor_and_no_plain_http_warning(self):
        self.assertIn('minimum protocol TLS 1.3', self.gw.output)
        self.assertIn('client certificates not required', self.gw.output)
        self.assertNotIn('travel in clear text', self.gw.output)

    def test_tls12_is_refused_and_tls13_is_answered(self):
        with self.assertRaises(_REFUSED):
            _call(self.port, 'GET', HEALTH, _context(ssl.TLSVersion.TLSv1_2))
        conn = http.client.HTTPSConnection('localhost', self.port, context=_context(), timeout=30)
        try:
            conn.request('GET', HEALTH)
            conn.getresponse().read()
            self.assertEqual(conn.sock.version(), 'TLSv1.3')
        finally:
            conn.close()

    def test_anonymous_is_refused_on_every_read_and_a_token_is_answered(self):
        for path in (AREAS, HEALTH):
            with self.subTest(path=path):
                self.assertEqual(self.get(path)[0], 401)
        self.assertEqual(self.get(AREAS, self.admin['access_token'])[0], 200)
        self.assertEqual(self.get(HEALTH, self.admin['access_token'])[0], 200)

    def test_viewer_reads_and_is_refused_on_a_write_route(self):
        self.assertEqual(self.get(AREAS, self.viewer['access_token'])[0], 200)
        status, _, _ = _call(self.port, 'POST', WRITE_ROUTE, headers=_bearer(
            self.viewer['access_token']), body={})
        self.assertEqual(status, 403)
        status, _, _ = _call(self.port, 'POST', WRITE_ROUTE, headers=_bearer(
            self.admin['access_token']), body={})
        # The operator requirement passed; the component does not exist.
        self.assertEqual(status, 404)

    def test_refresh_works_and_a_revoked_refresh_token_is_refused(self):
        login = _login(self.port, ADMIN)
        refresh = {'grant_type': 'refresh_token', 'refresh_token': login['refresh_token']}
        status, _, body = _call(self.port, 'POST', f'{API_BASE_PATH}/auth/token',
                                headers=_basic(ADMIN), body=refresh)
        self.assertEqual(status, 200, body)
        self.assertEqual(self.get(AREAS, body['access_token'])[0], 200)
        status, _, body = _call(self.port, 'POST', f'{API_BASE_PATH}/auth/revoke',
                                headers=_basic(ADMIN), body={'token': login['refresh_token']})
        self.assertEqual(status, 200, body)
        status, _, body = _call(self.port, 'POST', f'{API_BASE_PATH}/auth/token',
                                headers=_basic(ADMIN), body=refresh)
        self.assertEqual(status, 400, body)
        self.assertEqual(body.get('error'), 'invalid_grant', body)
        # The same client still logs in: the revocation took one token, not the client.
        self.assertEqual(_login(self.port, ADMIN)['token_type'].lower(), 'bearer')

    def test_a_wrong_client_secret_gets_no_token(self):
        status, _, _ = _call(self.port, 'POST', f'{API_BASE_PATH}/auth/authorize',
                             headers=_basic((ADMIN[0], 'not-the-secret')),
                             body={'grant_type': 'client_credentials'})
        self.assertEqual(status, 401)

    def test_secrets_are_not_served_by_the_parameter_services(self):
        secrets = (JWT_SECRET, ADMIN[1], VIEWER[1])
        jwt = _param('get', 'auth.jwt_secret')
        clients = _param('get', 'auth.clients')
        self.assertIn('<from MEDKIT_JWT_SECRET>', jwt)
        self.assertIn('<from MEDKIT_CLIENTS>', clients)
        for secret in secrets:
            self.assertNotIn(secret, jwt + clients)
        self.assertNotIn(JWT_SECRET, _param('list', ''))

    def test_auth_and_tls_keys_cannot_be_set_at_runtime(self):
        attempts = (('auth.require_auth_for', 'none'), ('server.tls.min_version', '"1.2"'))
        for name, value in attempts:
            with self.subTest(name=name):
                self.assertIn('read at start', _param('set', name, value))
        self.assertIn('all', _param('get', 'auth.require_auth_for'))
        self.assertIn('1.3', _param('get', 'server.tls.min_version'))
        self.assertEqual(self.get(AREAS)[0], 401)

    def test_cors_allows_the_file_origin_only(self):
        def preflight(origin):
            return _call(self.port, 'OPTIONS', AREAS, headers={
                'Origin': origin, 'Access-Control-Request-Method': 'GET'})
        status, headers, _ = preflight(UI_ORIGIN)
        self.assertEqual(status, 204)
        self.assertEqual(headers.get('access-control-allow-origin'), UI_ORIGIN)
        status, headers, _ = preflight('https://evil.example')
        self.assertEqual(status, 403)
        self.assertNotIn('access-control-allow-origin', headers)

    def test_docs_are_off_and_report_it(self):
        status, _, _ = self.get(f'{API_BASE_PATH}/docs', self.admin['access_token'])
        self.assertEqual(status, 501)
        self.assertEqual(self.get(f'{API_BASE_PATH}/docs')[0], 401)


class TestRateLimits(_Base):
    """The limits come from the file; the test crosses them."""

    def test_past_the_per_client_limit_a_token_gets_429_and_anonymous_still_401(self):
        with _Gateway() as gw:
            self.assert_started(gw)
            # The login spends one request of the same per-client bucket.
            token = _login(gw.port, ADMIN)['access_token']
            conn = http.client.HTTPSConnection(
                'localhost', gw.port, context=_context(), timeout=30)
            try:
                seen = []
                for _ in range(2 * CLIENT_LIMIT):
                    exhausted_at = time.monotonic()
                    status, _, _ = _call(gw.port, 'GET', VERSION_INFO,
                                         headers=_bearer(token), conn=conn)
                    if status == 429:
                        break
                    seen.append(status)
                else:
                    self.fail(f'no 429 within {2 * CLIENT_LIMIT} requests')
                self.assertEqual(set(seen), {200})
                # Without a refill the login and the successes add up to the limit.
                self.assertGreaterEqual(len(seen), CLIENT_LIMIT - 1)
                self.assertLess(len(seen), CLIENT_LIMIT + CLIENT_LIMIT // 4)
                self.assertEqual(_call(gw.port, 'GET', VERSION_INFO, conn=conn)[0], 401)
                # The bucket refills CLIENT_LIMIT tokens a minute. A slow run gets
                # back only what the refill explains, then 429 again.
                for earned_back in range(CLIENT_LIMIT // 4):
                    status = _call(gw.port, 'GET', VERSION_INFO,
                                   headers=_bearer(token), conn=conn)[0]
                    if status == 429:
                        break
                    self.assertEqual(status, 200)
                    refill = (time.monotonic() - exhausted_at) * CLIENT_LIMIT / 60
                    self.assertLessEqual(earned_back, refill)
                else:
                    self.fail(f'no second 429 within {CLIENT_LIMIT // 4} requests')
            finally:
                conn.close()

    def test_every_operations_limit_of_the_file_engages_on_its_real_route(self):
        entries = FILE['rate_limiting']['endpoint_limits']
        self.assertEqual([e.rsplit(':', 1)[0] for e in entries], OPERATION_PATTERNS)
        for entry in entries:
            pattern, limit = entry.rsplit(':', 1)
            limit = int(limit)
            method, path, expected = OPERATION_REQUESTS[pattern]
            # The path has the pattern's shape: same segments, `*` one segment each.
            for want, got in zip(pattern.split('/'), path.split('/')):
                self.assertIn(want, ('*', got))
            self.assertEqual(len(pattern.split('/')), len(path.split('/')))
            with self.subTest(pattern=pattern), _Gateway() as gw:
                self.assert_started(gw)
                token = _login(gw.port, ADMIN)['access_token']
                conn = http.client.HTTPSConnection(
                    'localhost', gw.port, context=_context(), timeout=30)
                try:
                    seen = []
                    for _ in range(2 * OPERATION_LIMIT):
                        status, _, _ = _call(gw.port, method, path, headers=_bearer(token),
                                             body={} if method == 'POST' else None, conn=conn)
                        if status == 429:
                            break
                        seen.append(status)
                    else:
                        self.fail(f'no 429 within {2 * OPERATION_LIMIT} requests: {seen[-3:]}')
                    self.assertEqual(set(seen), {expected})
                    self.assertGreaterEqual(len(seen), limit)
                    self.assertLess(len(seen), OPERATION_LIMIT + OPERATION_LIMIT // 4)
                finally:
                    conn.close()


class TestExecutionPolling(_Base):
    """A client that polls an execution does not lose its way to cancel it."""

    def test_status_polls_past_the_start_limit_leave_the_cancel_answered(self):
        with _Gateway() as gw:
            self.assert_started(gw)
            token = _login(gw.port, ADMIN)['access_token']
            conn = http.client.HTTPSConnection(
                'localhost', gw.port, context=_context(), timeout=30)
            try:
                polls = [_call(gw.port, 'GET', EXECUTION, headers=_bearer(token), conn=conn)[0]
                         for _ in range(OPERATION_LIMIT + OPERATION_LIMIT // 2)]
                self.assertEqual(set(polls), {404})
                status, _, _ = _call(gw.port, 'DELETE', EXECUTION, headers=_bearer(token),
                                     conn=conn)
                self.assertEqual(status, 404)
            finally:
                conn.close()


class TestStartRefusals(_Base):

    def test_no_secret_stops_the_start_and_names_it(self):
        with _Gateway(env={}) as gw:
            self.assert_refused_start(gw)
            self.assertIn('JWT secret is required', gw.output)
        with _Gateway() as gw:
            self.assert_started(gw)

    def test_missing_certificate_files_stop_the_start_and_name_the_file(self):
        with _Gateway(certs=False) as gw:
            self.assert_refused_start(gw)
            self.assertIn(FILE['server']['tls']['cert_file'], gw.output)
        with _Gateway() as gw:
            self.assert_started(gw)


class TestMutualTls(_Base):
    """The file's commented switch, turned on with -p and a CA."""

    def test_a_client_without_a_certificate_is_refused_and_one_with_it_gets_in(self):
        args = ('-p', 'server.tls.require_client_cert:=true',
                '-p', f'server.tls.ca_file:="{CA_CRT}"')
        with _Gateway(extra_args=args) as gw:
            self.assert_started(gw)
            self.assertIn('client certificates required', gw.output)
            ctx = _context(cert=(CLI_CRT, CLI_KEY))
            token = _login(gw.port, ADMIN, ctx)['access_token']
            status, _, _ = _call(gw.port, 'GET', AREAS, ctx, headers=_bearer(token))
            self.assertEqual(status, 200)
            with self.assertRaises(_REFUSED):
                _call(gw.port, 'GET', AREAS, _context(), headers=_bearer(token))


class TestTheFileUnderTest(unittest.TestCase):
    """The values the cases above rely on, read from the installed file."""

    def test_the_installed_file_is_closed_encrypted_and_limited(self):
        tls = FILE['server']['tls']
        self.assertIs(tls['enabled'], True)
        self.assertEqual(tls['min_version'], '1.3')
        self.assertNotIn('require_client_cert', tls)
        auth = FILE['auth']
        self.assertIs(auth['enabled'], True)
        self.assertEqual(auth['require_auth_for'], 'all')
        self.assertEqual(auth['jwt_secret'], '')
        self.assertEqual([c for c in auth['clients'] if c], [])
        self.assertEqual([r for r in auth.get('public_routes', []) if r], [])
        cors = FILE['cors']
        self.assertEqual(cors['allowed_origins'], [UI_ORIGIN])
        self.assertIs(cors['allow_credentials'], True)
        limits = FILE['rate_limiting']
        self.assertIs(limits['enabled'], True)
        self.assertLessEqual(limits['global_requests_per_minute'], GLOBAL_LIMIT)
        self.assertLessEqual(limits['client_requests_per_minute'], CLIENT_LIMIT)
        self.assertGreater(limits['client_requests_per_minute'], 0)
        for entry in limits['endpoint_limits']:
            self.assertLessEqual(int(entry.rsplit(':', 1)[1]), OPERATION_LIMIT)
        self.assertIs(FILE['locking']['enabled'], True)
        self.assertIs(FILE['scripts']['allow_uploads'], False)
        self.assertLessEqual(FILE['bulk_data']['max_upload_size'], 25 * 1024 * 1024)
        self.assertIs(FILE['docs']['enabled'], False)
        aggregation = FILE['aggregation']
        self.assertIs(aggregation['enabled'], False)
        self.assertIs(aggregation['require_tls'], True)
        self.assertIs(aggregation['forward_auth'], False)


def _cleanup():
    shutil.rmtree(_DIR, ignore_errors=True)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_cleanup(self):
        _cleanup()
