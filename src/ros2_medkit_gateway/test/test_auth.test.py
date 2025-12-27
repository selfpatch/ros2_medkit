#!/usr/bin/env python3
# Copyright 2025 bburda
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

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import requests


@pytest.mark.launch_test
def generate_test_description():
    """Generate launch description with gateway node with auth enabled."""
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='test_gateway_auth',
        parameters=[{
            'server.host': '127.0.0.1',
            'server.port': 8085,  # Use different port to avoid conflicts
            'refresh_interval_ms': 1000,
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
            ],
        }],
        output='screen',
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


class TestAuthenticationIntegration(unittest.TestCase):
    """Integration tests for authentication endpoints."""

    BASE_URL = 'http://127.0.0.1:8085/api/v1'

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready."""
        time.sleep(3)  # Give the gateway time to start

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
            },
            timeout=5,
        )
        self.assertEqual(response.status_code, 401)
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
            json={'token': refresh_token},
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
            },
            timeout=5,
        )
        self.assertEqual(refresh_response.status_code, 401)

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


class TestAuthorizationIntegration(unittest.TestCase):
    """Integration tests for RBAC authorization."""

    BASE_URL = 'http://127.0.0.1:8085/api/v1'

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready and get tokens."""
        time.sleep(1)

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
        """Operator role should be able to trigger operations."""
        headers = self._auth_header('operator')

        # POST operation - may return 404 if component doesn't exist,
        # but should not return 401 or 403
        response = requests.post(
            f'{self.BASE_URL}/components/test_component/operations/test_op',
            json={},
            headers=headers,
            timeout=5,
        )
        # 404 (not found) is acceptable - means auth passed but resource doesn't exist
        self.assertIn(response.status_code, [200, 400, 404, 500])

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
        """Configurator role should be able to modify configurations."""
        headers = self._auth_header('configurator')

        # May return 404 if component doesn't exist, but not 401 or 403
        response = requests.put(
            f'{self.BASE_URL}/components/test_component/configurations/param',
            json={'value': 123},
            headers=headers,
            timeout=5,
        )
        self.assertIn(response.status_code, [200, 400, 404, 500, 503])

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

            # Should not get 401 or 403
            self.assertNotIn(
                response.status_code, [401, 403],
                f'{method} {endpoint} returned auth error: {response.status_code}'
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


@launch_testing.post_shutdown_test()
class TestAuthShutdown(unittest.TestCase):
    """Post-shutdown tests."""

    def test_exit_code(self, proc_info):
        """Check that the gateway exited cleanly."""
        launch_testing.asserts.assertExitCodes(proc_info)
