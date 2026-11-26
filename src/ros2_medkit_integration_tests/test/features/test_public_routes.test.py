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

"""
`auth.public_routes` and the reduced `/health` body.

Five gateways run for the whole file:

- listed: `require_auth_for: all` with `GET /api/v1/health` listed.
- closed: `require_auth_for: all`, empty list.
- write: `require_auth_for: write`, empty list.
- off: authentication off, list set.
- limited: like listed, with a tight limit on `/health`.

The sweep class starts one more gateway per entry as a subprocess, because a
refused entry is observed as an exit code.

@verifies REQ_INTEROP_086
"""

import json
import os
import select
import signal
import subprocess
import tempfile
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node
from ros2_medkit_test_utils.param_client import ParameterClient

LISTED_PORT = get_test_port(0)
CLOSED_PORT = get_test_port(1)
WRITE_PORT = get_test_port(2)
OFF_PORT = get_test_port(3)
LIMITED_PORT = get_test_port(4)
# Subprocess gateways take this port one at a time.
SWEEP_PORT = get_test_port(5)


def _base(port):
    return f'http://127.0.0.1:{port}{API_BASE_PATH}'


LISTED_URL = _base(LISTED_PORT)
CLOSED_URL = _base(CLOSED_PORT)
WRITE_URL = _base(WRITE_PORT)
OFF_URL = _base(OFF_PORT)
LIMITED_URL = _base(LIMITED_PORT)
SWEEP_URL = _base(SWEEP_PORT)

LISTED_NODE = 'gateway_listed'
HEALTH_ROUTE = f'GET {API_BASE_PATH}/health'

JWT_SECRET = 'public_routes_integration_secret_key_0123456789'
CLIENT_ID = 'diagbox'
CLIENT_SECRET = 'diagbox_client_secret'
CLIENTS = [f'{CLIENT_ID}:{CLIENT_SECRET}:admin']

# The whole of the reduced body, and nothing else.
REDUCED_KEYS = {'status', 'timestamp', 'warnings', 'warning_schema_version', 'x-medkit-reduced'}

PATH_LIMIT = 4
MAX_ATTEMPTS = 3 * PATH_LIMIT

READY_LINE = 'Medkit Gateway ready on'
PARAM_DISCOVERY_TIMEOUT = 30

_PARAMS_DIR = tempfile.mkdtemp(prefix='medkit_public_routes_')


def _auth(require_auth_for, **extra):
    return {
        'server.host': '127.0.0.1',
        'auth.enabled': True,
        'auth.require_auth_for': require_auth_for,
        'auth.issuer': 'ros2_medkit_gateway',
        'auth.jwt_secret': JWT_SECRET,
        'auth.clients': CLIENTS,
        **extra,
    }


def generate_test_description():
    listed = create_gateway_node(
        port=LISTED_PORT, name=LISTED_NODE,
        extra_params=_auth('all', **{'auth.public_routes': [HEALTH_ROUTE]}))
    closed = create_gateway_node(
        port=CLOSED_PORT, name='gateway_closed', extra_params=_auth('all'))
    write = create_gateway_node(
        port=WRITE_PORT, name='gateway_write', extra_params=_auth('write'))
    off = create_gateway_node(
        port=OFF_PORT, name='gateway_off',
        extra_params={'server.host': '127.0.0.1', 'auth.enabled': False,
                      'auth.public_routes': [HEALTH_ROUTE]})
    limited = create_gateway_node(
        port=LIMITED_PORT, name='gateway_limited',
        extra_params=_auth(
            'all', **{
                'auth.public_routes': [HEALTH_ROUTE],
                'rate_limiting.enabled': True,
                'rate_limiting.global_requests_per_minute': 1000,
                'rate_limiting.client_requests_per_minute': 100,
                'rate_limiting.endpoint_limits': [f'{API_BASE_PATH}/health:{PATH_LIMIT}'],
            }))
    return launch.LaunchDescription([
        listed, closed, write, off, limited, launch_testing.actions.ReadyToTest(),
    ]), {}


def _token(base_url):
    resp = requests.post(
        f'{base_url}/auth/authorize',
        json={'grant_type': 'client_credentials', 'client_id': CLIENT_ID,
              'client_secret': CLIENT_SECRET},
        timeout=30)
    assert resp.status_code == 200, f'token request failed: {resp.status_code} {resp.text}'
    return {'Authorization': f'Bearer {resp.json()["access_token"]}'}


def _limiter_headers(resp):
    return [h for h in resp.headers
            if h.lower().startswith('x-ratelimit') or h.lower() == 'retry-after']


class TestListedRouteOnClosedGateway(GatewayTestCase):
    """`require_auth_for: all` with `GET /api/v1/health` listed."""

    BASE_URL = LISTED_URL

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.auth = _token(LISTED_URL)

    def test_01_anonymous_health_is_the_reduced_body(self):
        resp = requests.get(f'{LISTED_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        body = resp.json()
        self.assertEqual(set(body), REDUCED_KEYS, body)
        self.assertIs(body['x-medkit-reduced'], True)
        self.assertEqual(body['warnings'], [])
        self.assertEqual(body['status'], 'healthy')

    def test_02_an_invalid_token_gets_the_same_reduced_body(self):
        for header in ('Bearer not-a-token', 'Basic dXNlcjpwYXNz', 'Bearer'):
            with self.subTest(header=header):
                resp = requests.get(
                    f'{LISTED_URL}/health', headers={'Authorization': header}, timeout=15)
                self.assertEqual(resp.status_code, 200, resp.text)
                self.assertEqual(set(resp.json()), REDUCED_KEYS, resp.json())

    def test_03_a_valid_token_gets_the_full_body(self):
        resp = requests.get(f'{LISTED_URL}/health', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        body = resp.json()
        self.assertIn('discovery', body, body)
        self.assertNotIn('x-medkit-reduced', body)

    def test_04_unlisted_routes_and_methods_stay_closed(self):
        self.assertEqual(requests.get(f'{LISTED_URL}/areas', timeout=15).status_code, 401)
        self.assertEqual(requests.post(f'{LISTED_URL}/health', timeout=15).status_code, 401)
        self.assertEqual(
            requests.get(f'{LISTED_URL}/areas', headers=self.auth, timeout=15).status_code, 200)

    def test_05_the_listed_route_shows_the_empty_security_requirement(self):
        spec = requests.get(f'{LISTED_URL}/docs', headers=self.auth, timeout=15).json()
        self.assertEqual(spec['paths']['/health']['get'].get('security'), [])
        self.assertNotEqual(spec['paths']['/areas']['get'].get('security'), [])

    def test_07_the_bearer_scheme_name_is_case_insensitive(self):
        token = self.auth['Authorization'].split(' ', 1)[1]
        upper = {'Authorization': f'BEARER {token}'}
        resp = requests.get(f'{LISTED_URL}/health', headers=upper, timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertIn('discovery', resp.json())
        self.assertNotIn('x-medkit-reduced', resp.json())
        self.assertEqual(
            requests.get(f'{LISTED_URL}/areas', headers=upper, timeout=15).status_code, 200)
        garbage = {'Authorization': 'Bearer not-a-token'}
        reduced = requests.get(f'{LISTED_URL}/health', headers=garbage, timeout=15)
        self.assertEqual(set(reduced.json()), REDUCED_KEYS, reduced.json())
        self.assertEqual(
            requests.get(f'{LISTED_URL}/areas', headers=garbage, timeout=15).status_code, 401)

    def test_06_a_runtime_set_of_the_list_is_refused(self):
        client = ParameterClient('public_routes_probe')
        try:
            node = f'/{LISTED_NODE}'
            deadline = time.monotonic() + PARAM_DISCOVERY_TIMEOUT
            while True:
                try:
                    before = client.get(node, 'auth.public_routes')
                    break
                except Exception:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(1)
            self.assertEqual(before, [HEALTH_ROUTE])
            result = client.set_one(node, 'auth.public_routes', [f'GET {API_BASE_PATH}/areas'])
            self.assertFalse(result.successful, result.reason)
            self.assertIn('read at start', result.reason)
            self.assertEqual(client.get(node, 'auth.public_routes'), before)
        finally:
            client.close()
        self.assertEqual(requests.get(f'{LISTED_URL}/areas', timeout=15).status_code, 401)


class TestDefaultListOnClosedGateway(GatewayTestCase):
    """`require_auth_for: all`, empty list: health is refused like everything else."""

    BASE_URL = CLOSED_URL

    def test_01_anonymous_health_is_refused(self):
        resp = requests.get(f'{CLOSED_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 401, resp.text)

    def test_02_a_valid_token_gets_the_full_body(self):
        resp = requests.get(f'{CLOSED_URL}/health', headers=_token(CLOSED_URL), timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertIn('discovery', resp.json())
        self.assertNotIn('x-medkit-reduced', resp.json())

    def test_03_health_shows_the_protected_security_requirement(self):
        spec = requests.get(
            f'{CLOSED_URL}/docs', headers=_token(CLOSED_URL), timeout=15).json()
        self.assertNotEqual(spec['paths']['/health']['get'].get('security'), [])


class TestWriteModeHealthIsNotCutDown(GatewayTestCase):
    """`require_auth_for: write`, empty list: an anonymous read keeps the full body."""

    BASE_URL = WRITE_URL

    def test_01_anonymous_health_is_the_full_body(self):
        resp = requests.get(f'{WRITE_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertIn('discovery', resp.json())
        self.assertNotIn('x-medkit-reduced', resp.json())


class TestAuthenticationOffWithAList(GatewayTestCase):
    """Authentication off: the list is accepted and the body is whole."""

    BASE_URL = OFF_URL

    def test_01_health_is_the_full_body_for_everyone(self):
        resp = requests.get(f'{OFF_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertIn('discovery', resp.json())
        self.assertNotIn('x-medkit-reduced', resp.json())
        self.assertEqual(requests.get(f'{OFF_URL}/areas', timeout=15).status_code, 200)


class TestRateLimiterOnAListedRoute(GatewayTestCase):
    """The limiter treats a listed route as one that needs no authentication."""

    BASE_URL = LIMITED_URL

    @staticmethod
    def _until_429(send):
        seen = []
        for _ in range(MAX_ATTEMPTS):
            resp = send()
            if resp.status_code == 429:
                return seen, resp
            seen.append(resp.status_code)
        raise AssertionError(f'no 429 within {MAX_ATTEMPTS} requests: {seen}')

    def test_01_anonymous_over_the_limit_gets_the_full_429(self):
        before, refused = self._until_429(
            lambda: requests.get(f'{LIMITED_URL}/health', timeout=15))
        self.assertTrue(set(before) <= {200}, before)
        for name in ('Retry-After', 'X-RateLimit-Limit', 'X-RateLimit-Remaining',
                     'X-RateLimit-Reset'):
            self.assertIn(name, refused.headers, dict(refused.headers))
        self.assertGreater(refused.json()['parameters']['retry_after'], 0)

    def test_02_an_unlisted_route_gives_the_anonymous_caller_no_limiter_state(self):
        for _ in range(2 * PATH_LIMIT):
            resp = requests.get(f'{LIMITED_URL}/areas', timeout=15)
            self.assertEqual(resp.status_code, 401, resp.text)
            self.assertEqual(_limiter_headers(resp), [])


def _write_params(name, enabled, entries):
    path = os.path.join(_PARAMS_DIR, name)
    lines = [
        '/**:',
        '  ros__parameters:',
        '    auth:',
        f'      enabled: {"true" if enabled else "false"}',
        '      require_auth_for: "all"',
        f'      jwt_secret: "{JWT_SECRET}"',
        '      clients:',
        *[f'        - "{c}"' for c in CLIENTS],
        f'      public_routes: {json.dumps(entries)}',
    ]
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    return path


class _Gateway:
    """A `gateway_node` subprocess on the sweep port.

    `rc` is None while the gateway runs, else its exit code. Leaving the block
    kills its process group.
    """

    def __init__(self, params_file, timeout=45):
        self.command = [
            'ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
            '-p', f'server.port:={SWEEP_PORT}', '-p', 'server.host:=127.0.0.1',
            '--params-file', params_file,
        ]
        self.env = {k: v for k, v in os.environ.items() if not k.startswith('MEDKIT_')}
        self.timeout = timeout
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
        outcome = self._read_until(READY_LINE)
        self.ready = outcome == 'marker'
        if outcome == 'exit':
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
        self._process.communicate()
        return False


MALFORMED = (
    'GET',
    f'{API_BASE_PATH}/health',
    f'FOO {API_BASE_PATH}/health',
    f'GET {API_BASE_PATH[1:]}/health',
    f'GET {API_BASE_PATH}/*',
    f'GET {API_BASE_PATH}/health$',
    f'GET {API_BASE_PATH}/health|areas',
    f'GET {API_BASE_PATH}/health\tx',
    f'GET {API_BASE_PATH}/health/',
    f'PATCH {API_BASE_PATH}/health',
    f'GET {API_BASE_PATH}/h\u00e9',
)
VALID_SINGLE = [HEALTH_ROUTE]
VALID_SPELLING = [f' get {API_BASE_PATH}/health ']
VALID_HEAD = [f'HEAD {API_BASE_PATH}/health']
VALID_PAIR = [f'GET {API_BASE_PATH}/version-info', f'GET {API_BASE_PATH}/health']


class TestPublicRoutesAreValidatedAtStart(unittest.TestCase):
    """One gateway per entry, authentication on and off."""

    def _refused(self, enabled, entry, name):
        # The bad entry is second, so the message must name a position that is not 1.
        params = _write_params(name, enabled, [f'GET {API_BASE_PATH}/version-info', entry])
        with _Gateway(params) as gateway:
            self.assertEqual(gateway.rc, 1, gateway.output[-600:])
            self.assertIn('auth.public_routes entry 2', gateway.output)
            self.assertNotIn(READY_LINE, gateway.output)

    def test_01_a_malformed_entry_stops_the_start_with_authentication_on(self):
        for index, entry in enumerate(MALFORMED):
            with self.subTest(entry=entry):
                self._refused(True, entry, f'on-{index}.yaml')

    def test_02_a_malformed_entry_stops_the_start_with_authentication_off(self):
        for index, entry in enumerate(MALFORMED):
            with self.subTest(entry=entry):
                self._refused(False, entry, f'off-{index}.yaml')

    def test_03_valid_entries_start_with_authentication_on_and_off(self):
        for enabled in (True, False):
            cases = (VALID_SINGLE, VALID_PAIR, VALID_SPELLING, VALID_HEAD)
            for index, entries in enumerate(cases):
                with self.subTest(enabled=enabled, entries=entries):
                    params = _write_params(f'ok-{enabled}-{index}.yaml', enabled, entries)
                    with _Gateway(params) as gateway:
                        self.assertIsNone(gateway.rc, gateway.output[-600:])
                        self.assertTrue(gateway.ready, gateway.output[-600:])
                        resp = requests.get(f'{SWEEP_URL}/health', timeout=15)
                        # A HEAD entry opens HEAD only, so an anonymous GET stays closed.
                        expected = 401 if enabled and entries is VALID_HEAD else 200
                        self.assertEqual(resp.status_code, expected, resp.text)

    def test_04_a_blank_first_entry_does_not_shift_the_reported_position(self):
        for enabled in (True, False):
            with self.subTest(enabled=enabled):
                params = _write_params(f'pos-{enabled}.yaml', enabled, ['', '   ', 'GET'])
                with _Gateway(params) as gateway:
                    self.assertEqual(gateway.rc, 1, gateway.output[-600:])
                    self.assertIn('auth.public_routes entry 3', gateway.output)

    def test_05_blank_entries_are_stripped_and_open_nothing(self):
        cases = ([''], ['   '], ['', '   ', f'GET {API_BASE_PATH}/version-info'])
        for index, entries in enumerate(cases):
            with self.subTest(entries=entries):
                params = _write_params(f'blank-{index}.yaml', True, entries)
                with _Gateway(params) as gateway:
                    self.assertIsNone(gateway.rc, gateway.output[-600:])
                    self.assertTrue(gateway.ready, gateway.output[-600:])
                    for route in ('health', 'areas'):
                        resp = requests.get(f'{SWEEP_URL}/{route}', timeout=15)
                        self.assertEqual(resp.status_code, 401, route)

    def test_06_a_blank_entry_beside_a_real_one_still_opens_the_real_one(self):
        params = _write_params('blank-plus-health.yaml', True, ['', HEALTH_ROUTE])
        with _Gateway(params) as gateway:
            self.assertTrue(gateway.ready, gateway.output[-600:])
            resp = requests.get(f'{SWEEP_URL}/health', timeout=15)
            self.assertEqual(resp.status_code, 200, resp.text)
            self.assertEqual(set(resp.json()), REDUCED_KEYS, resp.json())
            self.assertEqual(requests.get(f'{SWEEP_URL}/areas', timeout=15).status_code, 401)

    def test_07_the_entry_spelling_is_trimmed_and_the_method_case_free(self):
        params = _write_params('spelling.yaml', True, VALID_SPELLING)
        with _Gateway(params) as gateway:
            self.assertTrue(gateway.ready, gateway.output[-600:])
            resp = requests.get(f'{SWEEP_URL}/health', timeout=15)
            self.assertEqual(resp.status_code, 200, resp.text)
            self.assertEqual(set(resp.json()), REDUCED_KEYS, resp.json())


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}')
