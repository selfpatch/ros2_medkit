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
Integration tests for JWT authentication and authorization.

Launches the gateway on port 8085 with auth enabled (JWT HS256, four client
roles). Tests authentication flows (authorize, refresh, revoke) and RBAC
enforcement (viewer, operator, configurator, admin).

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import base64
from concurrent.futures import ThreadPoolExecutor
import http.client
import json
import os
import unittest
from urllib.parse import quote_plus

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
from ros2_medkit_test_utils.launch_helpers import (
    create_gateway_node,
    graph_provider_params,
)

AUTH_PORT = get_test_port()
AUTH_BASE_URL = f'http://127.0.0.1:{AUTH_PORT}{API_BASE_PATH}'

# Characters that form-urlencoding changes, in the id and in the secret.
ENCODED_CLIENT = ('svc user', 'p:ss w+rd%&')
# `auth.clients` registers an empty secret for `id::role`.
EMPTY_SECRET_CLIENT = 'nosecret'
REALM_CHALLENGE = 'Basic realm="ros2_medkit_gateway"'
# A Basic value of this many `=` fits the server's 8192-byte header line.
PADDING_RUN = 8000
PADDED_REQUESTS = 16
# Peak RSS growth the padded requests may cause, in KiB.
MAX_PADDING_PEAK_KIB = 16 * 1024


def _basic(client_id, client_secret):
    """Return an Authorization header with RFC 6749 section 2.3.1 Basic credentials."""
    raw = f'{quote_plus(client_id)}:{quote_plus(client_secret)}'.encode()
    return {'Authorization': 'Basic ' + base64.b64encode(raw).decode()}


def _login(client_id, client_secret):
    """Authorize with body credentials and return the token response."""
    resp = requests.post(
        f'{AUTH_BASE_URL}/auth/authorize',
        json={'grant_type': 'client_credentials', 'client_id': client_id,
              'client_secret': client_secret},
        timeout=5)
    assert resp.status_code == 200, f'{client_id} got no token: {resp.status_code} {resp.text}'
    return resp.json()


def _refresh(refresh_token, client=('operator', 'operator_secret')):
    """Refresh with Basic credentials in a form body."""
    return requests.post(
        f'{AUTH_BASE_URL}/auth/token',
        data={'grant_type': 'refresh_token', 'refresh_token': refresh_token},
        headers=_basic(*client), timeout=5)


class _RawAnswer:
    """Status, headers and JSON body of a request sent through http.client."""

    def __init__(self, response):
        self.status_code = response.status
        self.headers = response.headers
        self.text = response.read().decode(errors='replace')

    def json(self):
        return json.loads(self.text)


def _post_raw(path, body, headers):
    """POST with `headers` as (name, value) pairs, so a header can appear twice."""
    conn = http.client.HTTPConnection('127.0.0.1', AUTH_PORT, timeout=5)
    try:
        conn.putrequest('POST', f'{API_BASE_PATH}{path}')
        data = body.encode()
        for name, value in [*headers, ('Content-Length', str(len(data)))]:
            conn.putheader(name, value)
        conn.endheaders(data)
        return _RawAnswer(conn.getresponse())
    finally:
        conn.close()


def _listener_pid(port):
    """Return the pid of the one process listening on `port`."""
    inodes = set()
    for table in ('/proc/net/tcp', '/proc/net/tcp6'):
        with open(table, encoding='utf-8') as handle:
            for row in handle.read().splitlines()[1:]:
                fields = row.split()
                if fields[3] == '0A' and int(fields[1].split(':')[1], 16) == port:
                    inodes.add(f'socket:[{fields[9]}]')
    pids = set()
    for entry in filter(str.isdigit, os.listdir('/proc')):
        try:
            if any(os.readlink(f'/proc/{entry}/fd/{fd}') in inodes
                   for fd in os.listdir(f'/proc/{entry}/fd')):
                pids.add(int(entry))
        except OSError:
            continue
    assert len(pids) == 1, f'listeners on {port}: {pids}'
    return pids.pop()


def _peak_rss_kib(pid):
    """Return VmHWM, the peak resident set size of `pid`, in KiB."""
    with open(f'/proc/{pid}/status', encoding='utf-8') as handle:
        for line in handle:
            if line.startswith('VmHWM:'):
                return int(line.split()[1])
    raise AssertionError(f'no VmHWM for {pid}')


def _reset_peak_rss(pid):
    """Set VmHWM of `pid` to its current resident set size; a later reading counts later growth."""
    with open(f'/proc/{pid}/clear_refs', 'w', encoding='utf-8') as handle:
        handle.write('5')


def _revoke(token, client=('operator', 'operator_secret'), **fields):
    """Revoke with Basic credentials in a form body."""
    return requests.post(
        f'{AUTH_BASE_URL}/auth/revoke', data={'token': token, **fields},
        headers=_basic(*client), timeout=5)


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description with gateway node with auth enabled."""
    gateway_node = create_gateway_node(
        port=AUTH_PORT,
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.jwt_secret': 'test_secret_key_for_jwt_signing_integration_test_12345',
            'auth.jwt_algorithm': 'HS256',
            'auth.token_expiry_seconds': 3600,
            'auth.refresh_token_expiry_seconds': 86400,
            'auth.require_auth_for': 'write',
            'auth.issuer': 'test_gateway',
            'auth.clients': [
                'admin:admin_secret:admin',
                'operator:operator_secret:operator',
                'viewer:viewer_secret:viewer',
                'configurator:configurator_secret:configurator',
                f'{ENCODED_CLIENT[0]}:{ENCODED_CLIENT[1]}:viewer',
                f'{EMPTY_SECRET_CLIENT}::viewer',
            ],
            # The auth-on half of the per-operation `security` pair. This is
            # the only fixture in the suite with authentication enabled AND a
            # plugin that describes a route, so it is the only place the
            # published-requirement direction can be driven.
            **graph_provider_params(),
        },
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


class TestAuthenticationIntegration(GatewayTestCase):
    """Integration tests for authentication endpoints."""

    BASE_URL = AUTH_BASE_URL

    def test_01_health_endpoint_no_auth_required(self):
        """Health endpoint should be accessible without authentication."""
        response = requests.get(f'{self.BASE_URL}/health', timeout=5)
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['status'], 'healthy')

    def test_02_root_endpoint_shows_auth_enabled(self):
        """Root endpoint should indicate auth is enabled."""
        response = requests.get(f'{self.BASE_URL}/', timeout=5)
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertTrue(data['capabilities']['authentication'])
        self.assertIn('auth', data)
        self.assertTrue(data['auth']['enabled'])
        self.assertEqual(data['auth']['algorithm'], 'HS256')

    def test_02b_plugin_operation_states_what_this_gateway_enforces(self):
        """A plugin's declared role reaches the document, and is then filtered.

        The other half of this pair is
        ``test_openapi_contract::test_no_operation_publishes_a_role_when_auth_is_off``:
        the same gateway code, the same plugin, the opposite ``auth.enabled``.
        Splitting it across two fixtures is not a convenience - a single
        gateway has one value of ``auth.enabled``, and the whole point of the
        rule is that the document follows it.

        What this fixture pins is the *second* filter. The graph provider
        declares ``admin`` on this route, and under ``require_auth_for: all``
        that is what the document publishes and what the middleware demands.
        This gateway runs ``write``, under which the middleware admits every
        GET - measured, not assumed: the ``/docs`` fetch on the line below
        carries no token and succeeds. So the honest published requirement here
        is the empty one, and publishing ``admin`` would have promised a check
        an anonymous caller does not meet.

        The role-publishing direction has not been dropped; it moved to
        ``test_auth_policy_contract.test.py``, which runs one gateway per
        ``require_auth_for`` value and can therefore drive both answers for
        this same operation.
        """
        spec = requests.get(f'{self.BASE_URL}/docs', timeout=10).json()
        op = spec['paths'].get(
            '/functions/{function_id}/x-medkit-graph', {}).get('get')
        self.assertIsNotNone(
            op, 'the graph provider route is not documented; the fixture must '
                'load the plugin for this test to mean anything')
        self.assertEqual(
            op.get('security'), [],
            'under require_auth_for=write the middleware admits every GET, so '
            'the plugin route must publish the empty requirement')
        # The scheme is defined either way - that is what lets a single
        # operation name it - and its presence is the auth-on marker that
        # survives a permissive policy.
        self.assertIn(
            'bearerAuth',
            spec.get('components', {}).get('securitySchemes', {}))
        # Anti-vacuous: `security: []` above must mean "this policy admits
        # GETs", not "this gateway publishes no roles at all". The writes are
        # checked under `write`, and they say so.
        roles = {
            tuple(req['bearerAuth'])
            for item in spec['paths'].values()
            for method, operation in item.items()
            if method in ('post', 'put', 'patch', 'delete')
            for req in operation.get('security', [])
            if 'bearerAuth' in req
        }
        self.assertTrue(
            roles, 'no write operation publishes a role; the document has '
                   'stopped naming roles altogether')

    def test_02c_the_auth_routes_keep_their_client_authentication(self):
        """No token is needed on /auth/*, and the Basic requirement survives that rule."""
        spec = requests.get(f'{self.BASE_URL}/docs', timeout=10).json()
        self.assertIn('basic', spec.get('components', {}).get('securitySchemes', {}))
        auth_paths = ('/auth/authorize', '/auth/token', '/auth/revoke')
        for path in auth_paths:
            with self.subTest(path=path):
                self.assertEqual(spec['paths'][path]['post'].get('security'),
                                 [{'basic': []}, {}])
        others = sorted(
            f'{method.upper()} {path}' for path, item in spec['paths'].items()
            for method, operation in item.items()
            if path not in auth_paths and isinstance(operation, dict)
            and any('basic' in req for req in operation.get('security', [])))
        self.assertEqual(others, [], 'only the auth operations take client credentials')

    def test_03_authenticate_valid_credentials(self):
        """@verifies REQ_INTEROP_086 - Authentication with valid credentials."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'admin',
                'client_secret': 'admin_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('access_token', data)
        self.assertIn('refresh_token', data)
        self.assertEqual(data['token_type'], 'Bearer')
        self.assertEqual(data['scope'], 'admin')
        self.assertEqual(data['expires_in'], 3600)

    def test_04_authenticate_invalid_credentials(self):
        """Authentication with invalid credentials should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'admin',
                'client_secret': 'wrong_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
        data = response.json()
        self.assertEqual(data['error'], 'invalid_client')

    def test_05_authenticate_unknown_client(self):
        """Authentication with unknown client should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'unknown_client',
                'client_secret': 'some_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
        data = response.json()
        self.assertEqual(data['error'], 'invalid_client')

    def test_06_authenticate_form_urlencoded(self):
        """@verifies REQ_INTEROP_086 - Authentication with form-urlencoded."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            data='grant_type=client_credentials&client_id=viewer&client_secret=viewer_secret',
            headers={'Content-Type': 'application/x-www-form-urlencoded'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['scope'], 'viewer')

    def test_07_refresh_token(self):
        """@verifies REQ_INTEROP_087 - Token refresh flow."""
        # First, authenticate to get tokens
        auth_response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'operator',
                'client_secret': 'operator_secret',
            },
            timeout=5,
        )
        self.assertEqual(auth_response.status_code, 200)
        tokens = auth_response.json()
        refresh_token = tokens['refresh_token']

        # Use refresh token to get new access token
        refresh_response = requests.post(
            f'{self.BASE_URL}/auth/token',
            json={
                'grant_type': 'refresh_token',
                'refresh_token': refresh_token,
                'client_id': 'operator',
                'client_secret': 'operator_secret',
            },
            timeout=5,
        )
        self.assertEqual(refresh_response.status_code, 200)
        new_tokens = refresh_response.json()
        self.assertIn('access_token', new_tokens)
        self.assertNotEqual(new_tokens['access_token'], tokens['access_token'])
        self.assertEqual(new_tokens['scope'], 'operator')

    def test_08_refresh_invalid_token(self):
        """Refresh with invalid token should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/token',
            json={
                'grant_type': 'refresh_token',
                'refresh_token': 'invalid_token',
                'client_id': 'operator',
                'client_secret': 'operator_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)
        data = response.json()
        self.assertEqual(data['error'], 'invalid_grant')

    def test_09_revoke_token(self):
        """Token revocation flow."""
        # First, authenticate
        auth_response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'admin',
                'client_secret': 'admin_secret',
            },
            timeout=5,
        )
        tokens = auth_response.json()
        refresh_token = tokens['refresh_token']

        # Revoke the refresh token
        revoke_response = requests.post(
            f'{self.BASE_URL}/auth/revoke',
            json={'token': refresh_token, 'client_id': 'admin', 'client_secret': 'admin_secret'},
            timeout=5,
        )
        self.assertEqual(revoke_response.status_code, 200)
        data = revoke_response.json()
        self.assertEqual(data['status'], 'revoked')

        # Try to use revoked refresh token
        refresh_response = requests.post(
            f'{self.BASE_URL}/auth/token',
            json={
                'grant_type': 'refresh_token',
                'refresh_token': refresh_token,
                'client_id': 'admin',
                'client_secret': 'admin_secret',
            },
            timeout=5,
        )
        self.assertEqual(refresh_response.status_code, 400)
        self.assertEqual(refresh_response.json()['error'], 'invalid_grant')

    def test_10_unsupported_grant_type(self):
        """Unsupported grant type should fail."""
        response = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            json={
                'grant_type': 'authorization_code',
                'client_id': 'admin',
                'client_secret': 'admin_secret',
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)
        data = response.json()
        self.assertEqual(data['error'], 'unsupported_grant_type')


class TestAuthorizationIntegration(GatewayTestCase):
    """Integration tests for RBAC authorization."""

    BASE_URL = AUTH_BASE_URL

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready and get tokens."""
        cls._wait_for_gateway_health()

        # Get tokens for different roles
        cls.tokens = {}
        for role, client_id, client_secret in [
            ('admin', 'admin', 'admin_secret'),
            ('operator', 'operator', 'operator_secret'),
            ('viewer', 'viewer', 'viewer_secret'),
            ('configurator', 'configurator', 'configurator_secret'),
        ]:
            response = requests.post(
                f'{cls.BASE_URL}/auth/authorize',
                json={
                    'grant_type': 'client_credentials',
                    'client_id': client_id,
                    'client_secret': client_secret,
                },
                timeout=5,
            )
            if response.status_code == 200:
                cls.tokens[role] = response.json()['access_token']

        missing = [r for r in ('admin', 'operator', 'viewer', 'configurator')
                   if r not in cls.tokens]
        if missing:
            raise AssertionError(
                f'Could not acquire tokens for: {", ".join(missing)}'
            )

    def _auth_header(self, role):
        """Get Authorization header for a role."""
        return {'Authorization': f'Bearer {self.tokens.get(role, "")}'}

    def test_01_read_endpoints_no_auth_required(self):
        """GET endpoints should work without authentication (require_auth_for=write)."""
        # These should work without auth
        endpoints = [
            '/health',
            '/version-info',
            '/areas',
            '/components',
        ]
        for endpoint in endpoints:
            response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=5)
            self.assertEqual(
                response.status_code, 200,
                f'GET {endpoint} failed without auth: {response.status_code}'
            )

    def test_02_write_without_auth_returns_401(self):
        """Write operations without auth should return 401."""
        # Note: These endpoints may not exist but should still require auth
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)

        response = requests.put(
            f'{self.BASE_URL}/components/test/configurations/param',
            json={'value': 123},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)

    def test_03_write_with_invalid_token_returns_401(self):
        """Write operations with invalid token should return 401."""
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            headers={'Authorization': 'Bearer invalid_token'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)

    def test_04_viewer_cannot_write(self):
        """Viewer role should not be able to perform write operations."""
        headers = self._auth_header('viewer')

        # POST operation should be forbidden
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            headers=headers,
            timeout=5,
        )
        self.assertEqual(response.status_code, 403)

        # PUT configuration should be forbidden
        response = requests.put(
            f'{self.BASE_URL}/components/test/configurations/param',
            json={'value': 123},
            headers=headers,
            timeout=5,
        )
        self.assertEqual(response.status_code, 403)

    def test_05_operator_can_trigger_operations(self):
        """Operator role should be able to trigger operations (auth passes)."""
        headers = self._auth_header('operator')

        # POST operation execution - auth should pass for operator role
        response = requests.post(
            f'{self.BASE_URL}/components/test_component/operations/test_op/executions',
            json={},
            headers=headers,
            timeout=5,
        )
        # Auth passes (not 401/403), entity doesn't exist -> 404
        self.assertEqual(
            response.status_code, 404,
            f'Operator should be authorized (not 401/403), got {response.status_code}'
        )

    def test_06_operator_cannot_modify_configurations(self):
        """Operator role should not be able to modify configurations."""
        headers = self._auth_header('operator')

        response = requests.put(
            f'{self.BASE_URL}/components/test/configurations/param',
            json={'value': 123},
            headers=headers,
            timeout=5,
        )
        self.assertEqual(response.status_code, 403)

    def test_07_configurator_can_modify_configurations(self):
        """Configurator role should be able to modify configurations (auth passes)."""
        headers = self._auth_header('configurator')

        # PUT configuration - auth should pass for configurator role
        response = requests.put(
            f'{self.BASE_URL}/components/test_component/configurations/param',
            json={'value': 123},
            headers=headers,
            timeout=5,
        )
        # Auth passes (not 401/403), entity doesn't exist -> 404
        self.assertEqual(
            response.status_code, 404,
            f'Configurator should be authorized (not 401/403), got {response.status_code}'
        )

    def test_08_admin_has_full_access(self):
        """Admin role should have access to all operations."""
        headers = self._auth_header('admin')

        # All write operations should be authorized
        endpoints = [
            ('POST', '/components/test/operations/calibrate', {}),
            ('PUT', '/components/test/configurations/param', {'value': 123}),
            ('DELETE', '/components/test/faults/F001', None),
        ]

        for method, endpoint, body in endpoints:
            if method == 'POST':
                response = requests.post(
                    f'{self.BASE_URL}{endpoint}',
                    json=body,
                    headers=headers,
                    timeout=5,
                )
            elif method == 'PUT':
                response = requests.put(
                    f'{self.BASE_URL}{endpoint}',
                    json=body,
                    headers=headers,
                    timeout=5,
                )
            elif method == 'DELETE':
                response = requests.delete(
                    f'{self.BASE_URL}{endpoint}',
                    headers=headers,
                    timeout=5,
                )

            # Auth passes (not 401/403), entity doesn't exist -> 404
            self.assertEqual(
                response.status_code, 404,
                f'{method} {endpoint}: expected 404 (auth passed, entity not found), '
                f'got {response.status_code}'
            )

    def test_09_www_authenticate_header_on_401(self):
        """401 responses should include WWW-Authenticate header."""
        response = requests.post(
            f'{self.BASE_URL}/components/test/operations/calibrate',
            json={},
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
        self.assertIn('WWW-Authenticate', response.headers)
        self.assertIn('Bearer', response.headers['WWW-Authenticate'])


class TestRefreshNeedsClientAuthentication(GatewayTestCase):
    """POST /auth/token authenticates the client (RFC 6749 2.3.1) and binds the token to it."""

    BASE_URL = AUTH_BASE_URL

    def test_01_refresh_with_basic_credentials(self):
        """@verifies REQ_INTEROP_087 - Basic credentials in the Authorization header."""
        tokens = _login('operator', 'operator_secret')
        resp = _refresh(tokens['refresh_token'])
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(resp.json()['scope'], 'operator')
        self.assertEqual(resp.json()['refresh_token'], tokens['refresh_token'])

    def test_02_refresh_with_body_credentials(self):
        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            data={'grant_type': 'refresh_token', 'refresh_token': tokens['refresh_token'],
                  'client_id': 'operator', 'client_secret': 'operator_secret'},
            timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_03_refresh_without_credentials_is_invalid_client(self):
        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            json={'grant_type': 'refresh_token', 'refresh_token': tokens['refresh_token']},
            timeout=5)
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_client')
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def test_04_a_wrong_basic_secret_is_challenged(self):
        """RFC 6749 5.2: a failed Basic authentication answers 401 with the Basic challenge."""
        tokens = _login('operator', 'operator_secret')
        resp = _refresh(tokens['refresh_token'], client=('operator', 'wrong_secret'))
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_client')
        self.assertEqual(resp.headers.get('WWW-Authenticate'), REALM_CHALLENGE)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def test_05_another_clients_refresh_token_is_invalid_grant(self):
        """The viewer authenticates correctly and presents the operator's refresh token."""
        tokens = _login('operator', 'operator_secret')
        resp = _refresh(tokens['refresh_token'], client=('viewer', 'viewer_secret'))
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_grant')
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200,
                         "the refusal touched the owner's token")

    def test_06_both_authentication_methods_is_invalid_request(self):
        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            data={'grant_type': 'refresh_token', 'refresh_token': tokens['refresh_token'],
                  'client_id': 'operator', 'client_secret': 'operator_secret'},
            headers=_basic('operator', 'operator_secret'), timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_request')
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def test_07_a_revoked_refresh_token_is_invalid_grant(self):
        revoked = _login('operator', 'operator_secret')
        kept = _login('operator', 'operator_secret')
        self.assertEqual(_revoke(revoked['refresh_token']).status_code, 200)
        resp = _refresh(revoked['refresh_token'])
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_grant')
        self.assertEqual(_refresh(kept['refresh_token']).status_code, 200)

    def test_08_basic_that_does_not_decode_is_invalid_client(self):
        """RFC 6749 5.2: a failed Authorization header answers 401 with the Basic challenge."""
        tokens = _login('operator', 'operator_secret')
        malformed = {
            'not base64': 'Basic not*base64!',
            'no colon': 'Basic ' + base64.b64encode(b'operator').decode(),
            'a broken percent escape': 'Basic ' + base64.b64encode(b'operator:secret%zz').decode(),
        }
        for label, header in malformed.items():
            with self.subTest(header=label):
                resp = requests.post(
                    f'{self.BASE_URL}/auth/token',
                    data={'grant_type': 'refresh_token', 'refresh_token': tokens['refresh_token']},
                    headers={'Authorization': header}, timeout=5)
                self.assertEqual(resp.status_code, 401, resp.text)
                self.assertEqual(resp.json()['error'], 'invalid_client')
                self.assertEqual(resp.headers.get('WWW-Authenticate'), REALM_CHALLENGE)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def test_09_basic_credentials_are_form_urlencoded(self):
        """The id and the secret are form-urlencoded before base64 (RFC 6749 2.3.1)."""
        tokens = _login(*ENCODED_CLIENT)
        resp = _refresh(tokens['refresh_token'], client=ENCODED_CLIENT)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(resp.json()['scope'], 'viewer')

        # The same credentials without the encoding: `+` decodes to a space and `%&` is no escape.
        raw = f'{ENCODED_CLIENT[0]}:{ENCODED_CLIENT[1]}'.encode()
        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            data={'grant_type': 'refresh_token', 'refresh_token': tokens['refresh_token']},
            headers={'Authorization': 'Basic ' + base64.b64encode(raw).decode()}, timeout=5)
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_client')
        self.assertEqual(resp.headers.get('WWW-Authenticate'), REALM_CHALLENGE)


class TestRevokeFollowsRfc7009(GatewayTestCase):
    """POST /auth/revoke authenticates the client and revokes only its own refresh tokens."""

    BASE_URL = AUTH_BASE_URL

    def test_01_form_encoded_with_basic_revokes(self):
        """@verifies REQ_INTEROP_086 - the RFC 7009 encoding and client authentication."""
        revoked = _login('operator', 'operator_secret')
        kept = _login('operator', 'operator_secret')
        resp = _revoke(revoked['refresh_token'])
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(resp.json(), {'status': 'revoked'})
        self.assertEqual(_refresh(revoked['refresh_token']).status_code, 400)
        self.assertEqual(_refresh(kept['refresh_token']).status_code, 200)

    def test_02_json_with_body_credentials_revokes(self):
        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/revoke',
            json={'token': tokens['refresh_token'], 'token_type_hint': 'refresh_token',
                  'client_id': 'operator', 'client_secret': 'operator_secret'},
            timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 400)

    def test_03_revoke_without_credentials_is_invalid_client(self):
        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/revoke', data={'token': tokens['refresh_token']}, timeout=5)
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_client')
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200,
                         'a refused revocation revoked the token')
        self.assertEqual(_revoke(tokens['refresh_token']).status_code, 200)

    def test_04_another_clients_refresh_token_is_refused(self):
        """RFC 7009 2.1: the token must have been issued to the requesting client."""
        tokens = _login('operator', 'operator_secret')
        resp = _revoke(tokens['refresh_token'], client=('viewer', 'viewer_secret'))
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_grant')
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200,
                         'another client revoked the token')
        self.assertEqual(_revoke(tokens['refresh_token']).status_code, 200)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 400)

    def test_05_an_access_token_is_an_unsupported_token_type(self):
        """RFC 7009 2.2.1: only refresh tokens are revocable here, with or without a hint."""
        tokens = _login('operator', 'operator_secret')
        for hint in (None, 'access_token', 'refresh_token'):
            with self.subTest(hint=hint):
                fields = {} if hint is None else {'token_type_hint': hint}
                resp = _revoke(tokens['access_token'], **fields)
                self.assertEqual(resp.status_code, 400, resp.text)
                self.assertEqual(resp.json()['error'], 'unsupported_token_type')
        # Nothing was revoked: the access token still clears the middleware and the pair refreshes.
        write = requests.post(
            f'{self.BASE_URL}/components/test_component/operations/test_op/executions',
            json={}, headers={'Authorization': f'Bearer {tokens["access_token"]}'}, timeout=5)
        self.assertEqual(write.status_code, 404, write.text)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def test_06_a_token_that_does_not_verify_answers_200(self):
        """RFC 7009 2.2: an invalid token is not an error, and nothing is revoked."""
        tokens = _login('operator', 'operator_secret')
        header, payload, signature = tokens['refresh_token'].split('.')
        alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_'
        replacement = alphabet[(alphabet.index(signature[0]) + 1) % len(alphabet)]
        forged = '.'.join([header, payload, replacement + signature[1:]])
        for label, token in {'garbage': 'not.a.token', 'a forged signature': forged}.items():
            with self.subTest(token=label):
                resp = _revoke(token)
                self.assertEqual(resp.status_code, 200, resp.text)
                self.assertEqual(resp.json(), {'status': 'revoked'})
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200,
                         'a token with the same jti and a forged signature revoked the real one')

    def test_07_another_content_type_is_invalid_request(self):
        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/revoke', data=f'token={tokens["refresh_token"]}',
            headers={**_basic('operator', 'operator_secret'), 'Content-Type': 'text/plain'},
            timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_request')
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)
        self.assertEqual(_revoke(tokens['refresh_token']).status_code, 200)


class TestAuthorizeAcceptsBasic(GatewayTestCase):
    """POST /auth/authorize takes the client's credentials in either RFC 6749 2.3.1 form."""

    BASE_URL = AUTH_BASE_URL

    def test_01_basic_with_a_form_body(self):
        """@verifies REQ_INTEROP_086 - client_credentials with Basic client authentication."""
        resp = requests.post(
            f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
            headers=_basic('viewer', 'viewer_secret'), timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(resp.json()['scope'], 'viewer')

    def test_02_basic_for_an_unknown_client_is_challenged(self):
        resp = requests.post(
            f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
            headers=_basic('no_such_client', 'viewer_secret'), timeout=5)
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_client')
        self.assertEqual(resp.headers.get('WWW-Authenticate'), REALM_CHALLENGE)

    def test_03_missing_credentials_are_invalid_client(self):
        """RFC 6749 5.2: no client authentication included."""
        resp = requests.post(
            f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
            timeout=5)
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_client')
        resp = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            data={'grant_type': 'client_credentials', 'client_id': 'viewer',
                  'client_secret': 'viewer_secret'},
            timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_04_a_field_that_is_not_a_string_is_invalid_request(self):
        body = {'grant_type': 'client_credentials', 'client_id': 5,
                'client_secret': 'viewer_secret'}
        resp = requests.post(f'{self.BASE_URL}/auth/authorize', json=body, timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_request')
        body['client_id'] = 'viewer'
        resp = requests.post(f'{self.BASE_URL}/auth/authorize', json=body, timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)


class TestAuthResponsesAreNotCached(GatewayTestCase):
    """RFC 6749 5.1: every answer of the three endpoints carries no-store and no-cache."""

    BASE_URL = AUTH_BASE_URL

    def _assert_not_cached(self, resp, label):
        self.assertEqual(resp.headers.get('Cache-Control'), 'no-store', label)
        self.assertEqual(resp.headers.get('Pragma'), 'no-cache', label)

    def test_01_success_and_error_of_each_endpoint(self):
        tokens = _login('operator', 'operator_secret')
        answers = {
            'authorize 200': requests.post(
                f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
                headers=_basic('operator', 'operator_secret'), timeout=5),
            'authorize 401': requests.post(
                f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
                headers=_basic('operator', 'wrong_secret'), timeout=5),
            'token 200': _refresh(tokens['refresh_token']),
            'token 400': _refresh('not.a.token'),
            'revoke 200': _revoke(tokens['refresh_token']),
            'revoke 400': _revoke(tokens['access_token']),
        }
        for label, resp in answers.items():
            with self.subTest(answer=label):
                self.assertEqual(str(resp.status_code), label.split()[1], resp.text)
                self._assert_not_cached(resp, label)


class TestAuthStatusesAreDeclared(GatewayTestCase):
    """Every status the three routes put on the wire is in the served document."""

    BASE_URL = AUTH_BASE_URL

    def test_01_emitted_statuses_are_declared(self):
        tokens = _login('operator', 'operator_secret')
        # 200, 400 and 401 on each route; the recorder sees every request this gateway served.
        requests.post(f'{self.BASE_URL}/auth/authorize', json={'grant_type': 'password'},
                      timeout=5)
        requests.post(f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
                      headers=_basic('operator', 'wrong_secret'), timeout=5)
        _refresh(tokens['refresh_token'])
        _refresh('not.a.token')
        _refresh(tokens['refresh_token'], client=('operator', 'wrong_secret'))
        _revoke(tokens['access_token'])
        _revoke(tokens['refresh_token'], client=('operator', 'wrong_secret'))
        _revoke(tokens['refresh_token'])

        spec = requests.get(f'{self.BASE_URL}/docs', timeout=10).json()
        coverage = requests.get(f'{self.BASE_URL}/x-medkit-status-coverage', timeout=10).json()
        observed = {}
        for entry in coverage['emitted']:
            if entry['path'].startswith('/auth/'):
                observed.setdefault(entry['path'], set()).add(entry['status'])
        for path in ('/auth/authorize', '/auth/token', '/auth/revoke'):
            with self.subTest(path=path):
                self.assertLessEqual({200, 400, 401}, observed.get(path, set()),
                                     f'{path}: the run did not reach its statuses')
                declared = spec['paths'][path]['post']['responses']
                undeclared = sorted(s for s in observed[path] if str(s) not in declared)
                self.assertEqual(undeclared, [], f'{path} emitted undeclared statuses')


class TestClientAuthenticationEdges(GatewayTestCase):
    """Empty Basic credentials, repeated headers and one description for every refusal."""

    BASE_URL = AUTH_BASE_URL

    def test_01_empty_basic_credentials_are_invalid_client(self):
        empty = {'an empty secret': f'{EMPTY_SECRET_CLIENT}:', 'both empty': ':'}
        for label, raw in empty.items():
            with self.subTest(credentials=label):
                resp = requests.post(
                    f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
                    headers={'Authorization': 'Basic ' + base64.b64encode(raw.encode()).decode()},
                    timeout=5)
                self.assertEqual(resp.status_code, 401, resp.text)
                self.assertEqual(resp.json()['error'], 'invalid_client')
                self.assertEqual(resp.headers.get('WWW-Authenticate'), REALM_CHALLENGE)
        # The body path refuses the empty secret as it did before.
        resp = requests.post(
            f'{self.BASE_URL}/auth/authorize',
            data={'grant_type': 'client_credentials', 'client_id': EMPTY_SECRET_CLIENT,
                  'client_secret': ''},
            timeout=5)
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_client')
        resp = requests.post(
            f'{self.BASE_URL}/auth/authorize', data={'grant_type': 'client_credentials'},
            headers=_basic('viewer', 'viewer_secret'), timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_02_more_than_one_authorization_header_is_invalid_request(self):
        basic = _basic('viewer', 'viewer_secret')['Authorization']
        form = [('Content-Type', 'application/x-www-form-urlencoded')]
        for label, values in {'Bearer then Basic': ['Bearer x', basic],
                              'Basic then Basic': [basic, basic]}.items():
            with self.subTest(headers=label):
                resp = _post_raw('/auth/authorize', 'grant_type=client_credentials',
                                 form + [('Authorization', value) for value in values])
                self.assertEqual(resp.status_code, 400, resp.text)
                self.assertEqual(resp.json()['error'], 'invalid_request')
        resp = _post_raw('/auth/authorize', 'grant_type=client_credentials',
                         form + [('Authorization', basic)])
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_03_every_invalid_client_has_one_description(self):
        """The body does not tell whether a client id exists."""
        tokens = _login('operator', 'operator_secret')
        bodies = {
            '/auth/authorize': {'grant_type': 'client_credentials'},
            '/auth/token': {'grant_type': 'refresh_token',
                            'refresh_token': tokens['refresh_token']},
            '/auth/revoke': {'token': tokens['refresh_token']},
        }
        credentials = {
            'an unknown client': _basic('no_such_client', 'operator_secret'),
            'a wrong secret': _basic('operator', 'wrong_secret'),
            'no credentials': {},
        }
        descriptions = set()
        for path, body in bodies.items():
            for label, headers in credentials.items():
                with self.subTest(path=path, credentials=label):
                    resp = requests.post(f'{self.BASE_URL}{path}', data=body, headers=headers,
                                         timeout=5)
                    self.assertEqual(resp.status_code, 401, resp.text)
                    self.assertEqual(resp.json()['error'], 'invalid_client')
                    descriptions.add(resp.json()['error_description'])
        self.assertEqual(len(descriptions), 1, sorted(descriptions))
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def _bodies(self, tokens):
        return {
            '/auth/authorize': {'grant_type': 'client_credentials'},
            '/auth/token': {'grant_type': 'refresh_token',
                            'refresh_token': tokens['refresh_token']},
            '/auth/revoke': {'token': tokens['refresh_token']},
        }

    def test_04_a_body_client_id_unlike_the_basic_one_is_invalid_request(self):
        """Basic does not win over another client_id in the body."""
        tokens = _login('operator', 'operator_secret')
        basic = _basic('operator', 'operator_secret')
        for path, body in self._bodies(tokens).items():
            with self.subTest(path=path, body_client_id='viewer'):
                resp = requests.post(f'{self.BASE_URL}{path}',
                                     data={**body, 'client_id': 'viewer'}, headers=basic,
                                     timeout=5)
                self.assertEqual(resp.status_code, 400, resp.text)
                self.assertEqual(resp.json()['error'], 'invalid_request')
        # The Basic id repeated in the body is no conflict. Revoke runs last.
        for path, body in self._bodies(tokens).items():
            with self.subTest(path=path, body_client_id='operator'):
                resp = requests.post(f'{self.BASE_URL}{path}',
                                     data={**body, 'client_id': 'operator'}, headers=basic,
                                     timeout=5)
                self.assertEqual(resp.status_code, 200, resp.text)

    def test_05_every_invalid_client_carries_the_basic_challenge(self):
        """RFC 7235 3.1: a 401 names its scheme, for credentials in the body too."""
        tokens = _login('operator', 'operator_secret')
        credentials = {
            'Basic with a wrong secret': ({}, _basic('operator', 'wrong_secret')),
            'Basic for an unknown client': ({}, _basic('no_such_client', 'operator_secret')),
            'body with a wrong secret': (
                {'client_id': 'operator', 'client_secret': 'wrong_secret'}, {}),
            'body for an unknown client': (
                {'client_id': 'no_such_client', 'client_secret': 'operator_secret'}, {}),
            'body id without a secret': ({'client_id': 'operator'}, {}),
            'no credentials': ({}, {}),
        }
        for path, body in self._bodies(tokens).items():
            for label, (fields, headers) in credentials.items():
                with self.subTest(path=path, credentials=label):
                    resp = requests.post(f'{self.BASE_URL}{path}', data={**body, **fields},
                                         headers=headers, timeout=5)
                    self.assertEqual(resp.status_code, 401, resp.text)
                    self.assertEqual(resp.json()['error'], 'invalid_client')
                    self.assertEqual(resp.headers.get('WWW-Authenticate'), REALM_CHALLENGE)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def test_06_a_run_of_padding_is_refused_before_it_is_decoded(self):
        """Base64 decoding copies the rest of the value once per trailing `=`."""
        pid = _listener_pid(AUTH_PORT)
        encoded = base64.b64encode(b'viewer:viewer_secret').decode().rstrip('=')
        header = {'Authorization': f'Basic {encoded}' + '=' * PADDING_RUN}

        def send(_):
            return requests.post(f'{self.BASE_URL}/auth/authorize',
                                 data={'grant_type': 'client_credentials'}, headers=header,
                                 timeout=30)

        _reset_peak_rss(pid)
        before = _peak_rss_kib(pid)
        with ThreadPoolExecutor(PADDED_REQUESTS) as pool:
            answers = list(pool.map(send, range(PADDED_REQUESTS)))
        grown = _peak_rss_kib(pid) - before
        for resp in answers:
            self.assertEqual(resp.status_code, 401, resp.text)
            self.assertEqual(resp.json()['error'], 'invalid_client')
        self.assertLess(grown, MAX_PADDING_PEAK_KIB,
                        f'{PADDED_REQUESTS} padded requests raised the peak RSS by {grown} KiB')
        resp = requests.post(f'{self.BASE_URL}/auth/authorize',
                             data={'grant_type': 'client_credentials'},
                             headers={'Authorization': f'Basic {encoded}='}, timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)


class TestAuthBodiesFollowTheirMediaType(GatewayTestCase):
    """Media types, form decoding, repeated parameters and grant_type (RFC 6749 3.1, 3.2)."""

    BASE_URL = AUTH_BASE_URL

    def _authorize(self, content_type, body):
        return requests.post(f'{self.BASE_URL}/auth/authorize', data=body,
                             headers={'Content-Type': content_type}, timeout=5)

    def test_01_the_media_type_is_compared_exactly(self):
        body = json.dumps({'grant_type': 'client_credentials', 'client_id': 'viewer',
                           'client_secret': 'viewer_secret'})
        for content_type in ('application/jsonx', 'text/plain; profile="application/json"'):
            with self.subTest(content_type=content_type):
                resp = self._authorize(content_type, body)
                self.assertEqual(resp.status_code, 400, resp.text)
                self.assertEqual(resp.json()['error'], 'invalid_request')
        resp = self._authorize('Application/JSON', body)
        self.assertEqual(resp.status_code, 200, resp.text)

        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            data=f'grant_type=refresh_token&refresh_token={tokens["refresh_token"]}',
            headers={**_basic('operator', 'operator_secret'),
                     'Content-Type': 'application/x-www-form-urlencoded; charset=UTF-8'},
            timeout=5)
        self.assertEqual(resp.status_code, 200, resp.text)
        resp = requests.post(
            f'{self.BASE_URL}/auth/revoke', data=f'token={tokens["refresh_token"]}',
            headers={**_basic('operator', 'operator_secret'), 'Content-Type': 'application/jsonx'},
            timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)

    def test_02_a_malformed_percent_escape_is_invalid_request(self):
        form = 'application/x-www-form-urlencoded'
        resp = self._authorize(
            form, 'grant_type=client_credentials&client_id=viewer&client_secret=abc%')
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_request')
        resp = self._authorize(
            form, 'grant_type=client_credentials&client_id=viewer&client_secret=viewer%5Fsecret')
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_03_a_repeated_parameter_is_invalid_request(self):
        first = _login('operator', 'operator_secret')
        second = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/revoke',
            data=f'token={first["refresh_token"]}&token={second["refresh_token"]}',
            headers={**_basic('operator', 'operator_secret'),
                     'Content-Type': 'application/x-www-form-urlencoded'},
            timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_request')
        for label, pair in {'first': first, 'second': second}.items():
            self.assertEqual(_refresh(pair['refresh_token']).status_code, 200,
                             f'the {label} token was revoked')

        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            data=('grant_type=client_credentials&grant_type=refresh_token'
                  f'&refresh_token={first["refresh_token"]}'),
            headers={**_basic('operator', 'operator_secret'),
                     'Content-Type': 'application/x-www-form-urlencoded'},
            timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_request')

    def test_04_a_missing_or_empty_grant_type_is_invalid_request(self):
        """RFC 6749 3.1: a parameter without a value is treated as omitted."""
        tokens = _login('operator', 'operator_secret')
        refresh = tokens['refresh_token']
        for label, body in {'missing': f'refresh_token={refresh}',
                            'empty': f'grant_type=&refresh_token={refresh}'}.items():
            with self.subTest(grant_type=label):
                resp = requests.post(
                    f'{self.BASE_URL}/auth/token', data=body,
                    headers={**_basic('operator', 'operator_secret'),
                             'Content-Type': 'application/x-www-form-urlencoded'},
                    timeout=5)
                self.assertEqual(resp.status_code, 400, resp.text)
                self.assertEqual(resp.json()['error'], 'invalid_request')
        resp = requests.post(
            f'{self.BASE_URL}/auth/authorize', json={},
            headers=_basic('viewer', 'viewer_secret'), timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'invalid_request')
        self.assertIn('grant_type', resp.json()['error_description'])
        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            data={'grant_type': 'password', 'refresh_token': refresh},
            headers=_basic('operator', 'operator_secret'), timeout=5)
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json()['error'], 'unsupported_grant_type')
        self.assertEqual(_refresh(refresh).status_code, 200)

    def test_05_a_form_body_over_the_limit_is_413(self):
        """cpp-httplib caps a lower-case application/x-www-form-urlencoded body at 8192 bytes."""
        tokens = _login('operator', 'operator_secret')
        resp = requests.post(
            f'{self.BASE_URL}/auth/token',
            data=(f'grant_type=refresh_token&refresh_token={tokens["refresh_token"]}&pad='
                  + 'a' * 8192),
            headers={**_basic('operator', 'operator_secret'),
                     'Content-Type': 'application/x-www-form-urlencoded'},
            timeout=5)
        self.assertEqual(resp.status_code, 413, resp.text)
        self.assertEqual(resp.json()['error_code'], 'payload-too-large', resp.text)
        self.assertEqual(resp.json()['parameters']['status'], 413, resp.text)
        self.assertEqual(_refresh(tokens['refresh_token']).status_code, 200)


class TestAnswersWrittenBeforeAHandler(GatewayTestCase):
    """The global error handler names the status cpp-httplib answered with."""

    BASE_URL = AUTH_BASE_URL

    def test_01_an_unknown_path_is_resource_not_found(self):
        resp = requests.get(f'{self.BASE_URL}/no_such_route', timeout=5)
        self.assertEqual(resp.status_code, 404, resp.text)
        self.assertEqual(resp.json()['error_code'], 'resource-not-found', resp.text)
        self.assertEqual(resp.json()['parameters']['status'], 404, resp.text)

    def test_02_a_request_line_over_the_limit_is_not_resource_not_found(self):
        """cpp-httplib answers 414 for a request line over 8192 bytes."""
        resp = requests.get(f'{self.BASE_URL}/health?pad=' + 'a' * 8192, timeout=5)
        self.assertEqual(resp.status_code, 414, resp.text)
        self.assertEqual(resp.json()['error_code'], 'invalid-request', resp.text)
        self.assertEqual(resp.json()['parameters']['status'], 414, resp.text)
        self.assertEqual(requests.get(f'{self.BASE_URL}/health', timeout=5).status_code, 200)

    def test_03_an_unparseable_range_is_not_resource_not_found(self):
        """cpp-httplib answers 416 for a Range header it cannot parse."""
        resp = requests.get(f'{self.BASE_URL}/health', headers={'Range': 'bytes=abc'}, timeout=5)
        self.assertEqual(resp.status_code, 416, resp.text)
        self.assertEqual(resp.json()['error_code'], 'invalid-request', resp.text)
        self.assertEqual(resp.json()['parameters']['status'], 416, resp.text)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
