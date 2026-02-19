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
HTTPS/TLS integration tests for ROS 2 Medkit Gateway.

This test file:
1. Generates self-signed certificates in a temporary directory
2. Starts the gateway with TLS enabled using generated certificates
3. Tests HTTPS endpoints with SSL verification
4. Cleans up certificates after tests

Migrated from: ros2_medkit_gateway/test/test_tls.test.py
"""

import os
import socket
import ssl
import subprocess
import tempfile
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing
import launch_testing.actions
from ros2_medkit_test_utils.coverage import get_coverage_env
import urllib3

# Port for HTTPS testing (different from default HTTP port)
HTTPS_PORT = 8443
HTTPS_BASE_URL = f'https://localhost:{HTTPS_PORT}'


def generate_test_certificates(output_dir: str) -> dict:
    """
    Generate self-signed certificates for testing.

    Parameters
    ----------
    output_dir : str
        Directory to store generated certificates

    Returns
    -------
    dict
        Paths to generated certificate files

    """
    cert_file = os.path.join(output_dir, 'cert.pem')
    key_file = os.path.join(output_dir, 'key.pem')
    ca_file = os.path.join(output_dir, 'ca.pem')

    # Generate CA key and certificate
    subprocess.run([
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', os.path.join(output_dir, 'ca_key.pem'),
        '-out', ca_file,
        '-days', '1', '-nodes',
        '-subj', '/C=US/ST=Test/L=Test/O=Test/CN=TestCA'
    ], check=True, capture_output=True)

    # Generate server key
    subprocess.run([
        'openssl', 'genrsa', '-out', key_file, '2048'
    ], check=True, capture_output=True)

    # Generate CSR
    csr_file = os.path.join(output_dir, 'server.csr')
    subprocess.run([
        'openssl', 'req', '-new', '-key', key_file, '-out', csr_file,
        '-subj', '/C=US/ST=Test/L=Test/O=Test/CN=localhost'
    ], check=True, capture_output=True)

    # Create extension file for SAN
    ext_file = os.path.join(output_dir, 'ext.cnf')
    with open(ext_file, 'w') as f:
        f.write("""basicConstraints=CA:FALSE
keyUsage = digitalSignature, keyEncipherment
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
""")

    # Sign certificate with CA
    subprocess.run([
        'openssl', 'x509', '-req', '-in', csr_file,
        '-CA', ca_file,
        '-CAkey', os.path.join(output_dir, 'ca_key.pem'),
        '-CAcreateserial',
        '-out', cert_file,
        '-days', '1',
        '-extfile', ext_file
    ], check=True, capture_output=True)

    return {
        'cert_file': cert_file,
        'key_file': key_file,
        'ca_file': ca_file,
    }


def create_https_params_file(cert_paths: dict, output_dir: str) -> str:
    """
    Create a temporary parameters YAML file for HTTPS testing.

    Parameters
    ----------
    cert_paths : dict
        Paths to certificate files
    output_dir : str
        Directory to store the params file

    Returns
    -------
    str
        Path to the generated params file

    """
    params_file = os.path.join(output_dir, 'gateway_params_https.yaml')
    params_content = f"""gateway_node:
  ros__parameters:
    server:
      host: "127.0.0.1"
      port: {HTTPS_PORT}
      tls:
        enabled: true
        cert_file: "{cert_paths['cert_file']}"
        key_file: "{cert_paths['key_file']}"
        min_version: "1.2"
    refresh_interval_ms: 1000
    cors:
      allowed_origins: [""]
"""
    with open(params_file, 'w') as f:
        f.write(params_content)

    return params_file


# Create temp dir and certificates at module level for launch description
_temp_dir = tempfile.mkdtemp(prefix='ros2_medkit_https_test_')
_cert_paths = generate_test_certificates(_temp_dir)
_params_file = create_https_params_file(_cert_paths, _temp_dir)


def generate_test_description():
    """Generate the launch description for HTTPS integration tests."""
    coverage_env = get_coverage_env()

    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='gateway_node',
        parameters=[_params_file],
        additional_env=coverage_env,
        output='screen',
    )

    # Allow gateway time to start
    delayed_tests = TimerAction(
        period=3.0,
        actions=[launch_testing.actions.ReadyToTest()]
    )

    return LaunchDescription([
        gateway_node,
        delayed_tests,
    ])


class TestHttpsEndpoints(unittest.TestCase):
    """Integration tests for HTTPS endpoints.

    Does not inherit from GatewayTestCase because all requests require
    custom TLS verify parameters (verify=False or verify=ca_file).
    """

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        cls.base_url = HTTPS_BASE_URL
        cls.ca_file = _cert_paths['ca_file']
        # Suppress InsecureRequestWarning for self-signed cert tests
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    def test_https_health_endpoint_no_verify(self):
        """
        Test HTTPS health endpoint without certificate verification.

        This simulates curl -k behavior for self-signed certificates.
        """
        import requests

        url = f'{self.base_url}/api/v1/health'
        response = requests.get(url, verify=False, timeout=5)

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['status'], 'healthy')

    def test_https_health_endpoint_with_ca_verify(self):
        """Test HTTPS health endpoint with CA certificate verification."""
        import requests

        url = f'{self.base_url}/api/v1/health'
        response = requests.get(url, verify=self.ca_file, timeout=5)

        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['status'], 'healthy')

    def test_https_version_info_endpoint(self):
        """Test HTTPS version-info endpoint returns valid format and data."""
        import requests

        url = f'{self.base_url}/api/v1/version-info'
        response = requests.get(url, verify=False, timeout=5)

        self.assertEqual(response.status_code, 200)
        data = response.json()
        # sovd_info array with version, base_uri, vendor_info
        self.assertIn('sovd_info', data)
        self.assertIsInstance(data['sovd_info'], list)
        self.assertGreater(len(data['sovd_info']), 0)
        info = data['sovd_info'][0]
        self.assertIn('version', info)
        self.assertIn('base_uri', info)

    def test_https_areas_endpoint(self):
        """Test HTTPS areas endpoint."""
        import requests

        url = f'{self.base_url}/api/v1/areas'
        response = requests.get(url, verify=False, timeout=5)

        self.assertEqual(response.status_code, 200)
        data = response.json()
        # Areas endpoint returns {items: [...], total_count: N}
        self.assertIsInstance(data, dict)
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_https_connection_uses_tls(self):
        """Verify that the connection is actually using TLS."""
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE

        with socket.create_connection(('localhost', HTTPS_PORT), timeout=5) as sock:
            with context.wrap_socket(sock, server_hostname='localhost') as ssock:
                # Verify we got a TLS connection
                self.assertIsNotNone(ssock.version())
                self.assertIn('TLS', ssock.version())

    def test_https_tls_version_minimum(self):
        """Verify that TLS 1.2 or higher is used."""
        context = ssl.create_default_context()
        context.check_hostname = False
        context.verify_mode = ssl.CERT_NONE

        with socket.create_connection(('localhost', HTTPS_PORT), timeout=5) as sock:
            with context.wrap_socket(sock, server_hostname='localhost') as ssock:
                version = ssock.version()
                # Should be TLSv1.2 or TLSv1.3
                self.assertTrue(
                    version in ['TLSv1.2', 'TLSv1.3'],
                    'Expected TLS 1.2 or 1.3, got {}'.format(version)
                )


class TestHttpsErrorHandling(unittest.TestCase):
    """Tests for HTTPS error handling scenarios."""

    @classmethod
    def setUpClass(cls):
        """Set up test fixtures."""
        cls.base_url = HTTPS_BASE_URL
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    def test_https_invalid_path_returns_404(self):
        """Test that invalid paths return 404 over HTTPS."""
        import requests

        url = f'{self.base_url}/api/v1/nonexistent'
        response = requests.get(url, verify=False, timeout=5)

        self.assertEqual(response.status_code, 404)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown cleanup tests."""

    def test_cleanup_certificates(self):
        """Clean up temporary certificates after tests."""
        import shutil

        if os.path.exists(_temp_dir):
            shutil.rmtree(_temp_dir, ignore_errors=True)

        # Verify cleanup
        self.assertFalse(
            os.path.exists(_temp_dir),
            'Temp directory {} should be removed'.format(_temp_dir)
        )
