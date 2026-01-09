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
Launch file for ROS 2 Medkit Gateway HTTPS/TLS integration tests.

This launch file:
1. Generates self-signed certificates for testing
2. Starts the ROS 2 Medkit Gateway node with TLS enabled
3. Runs integration tests verifying HTTPS endpoints
4. Cleans up all processes and temporary certificates
"""

import os
import shutil
import subprocess
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests
import urllib3


# Disable SSL warnings for self-signed certificates in tests
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)


def get_coverage_env():
    """Get environment variables for gcov coverage data collection."""
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_gateway')
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_gateway')

        if os.path.exists(build_dir):
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        pass
    return {}


def generate_test_certificates(output_dir: str) -> tuple:
    """
    Generate self-signed certificates for testing.

    Parameters
    ----------
    output_dir : str
        Directory to store generated certificates

    Returns
    -------
    tuple
        (cert_file, key_file, ca_file) paths

    """
    cert_file = os.path.join(output_dir, 'cert.pem')
    key_file = os.path.join(output_dir, 'key.pem')
    ca_file = os.path.join(output_dir, 'ca.pem')

    # Generate CA certificate
    subprocess.run([
        'openssl', 'req', '-x509', '-newkey', 'rsa:2048',
        '-keyout', os.path.join(output_dir, 'ca_key.pem'),
        '-out', ca_file,
        '-days', '1', '-nodes',
        '-subj', '/CN=Test CA'
    ], check=True, capture_output=True)

    # Generate server key
    subprocess.run([
        'openssl', 'genrsa', '-out', key_file, '2048'
    ], check=True, capture_output=True)

    # Generate CSR
    subprocess.run([
        'openssl', 'req', '-new', '-key', key_file,
        '-out', os.path.join(output_dir, 'server.csr'),
        '-subj', '/CN=localhost'
    ], check=True, capture_output=True)

    # Create extension file for SAN
    ext_file = os.path.join(output_dir, 'ext.cnf')
    with open(ext_file, 'w') as f:
        f.write("""
authorityKeyIdentifier=keyid,issuer
basicConstraints=CA:FALSE
keyUsage = digitalSignature, keyEncipherment
subjectAltName = @alt_names

[alt_names]
DNS.1 = localhost
IP.1 = 127.0.0.1
""")

    # Sign server certificate with CA
    subprocess.run([
        'openssl', 'x509', '-req',
        '-in', os.path.join(output_dir, 'server.csr'),
        '-CA', ca_file,
        '-CAkey', os.path.join(output_dir, 'ca_key.pem'),
        '-CAcreateserial',
        '-out', cert_file,
        '-days', '1',
        '-extfile', ext_file
    ], check=True, capture_output=True)

    return cert_file, key_file, ca_file


# Global variables for certificate paths
_cert_dir = tempfile.mkdtemp(prefix='ros2_medkit_tls_test_')
_cert_file, _key_file, _ca_file = generate_test_certificates(_cert_dir)


def generate_test_description():
    """Generate the launch description for TLS integration tests."""
    # Use TLS-specific port to avoid conflicts with other tests
    tls_port = 8443

    # Gateway node with TLS enabled
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        parameters=[{
            'server.host': '127.0.0.1',
            'server.port': tls_port,
            'server.tls.enabled': True,
            'server.tls.cert_file': _cert_file,
            'server.tls.key_file': _key_file,
            'server.tls.min_version': '1.2',
            'refresh_interval_ms': 1000,
        }],
        additional_env=get_coverage_env(),
        output='screen',
    )

    # Delay test start to allow gateway to initialize
    delayed_tests = TimerAction(
        period=3.0,
        actions=[
            launch_testing.actions.ReadyToTest(),
        ],
    )

    return LaunchDescription([
        gateway_node,
        delayed_tests,
    ]), {
        'gateway': gateway_node,
    }


class TestTlsGateway(unittest.TestCase):
    """Integration tests for TLS/HTTPS functionality."""

    BASE_URL = 'https://127.0.0.1:8443'
    API_BASE = f'{BASE_URL}/api/v1'

    @classmethod
    def setUpClass(cls):
        """Wait for gateway to be ready."""
        max_retries = 30
        for _ in range(max_retries):
            try:
                # Use verify=False for self-signed certificate
                response = requests.get(
                    f'{cls.API_BASE}/health',
                    verify=False,
                    timeout=1
                )
                if response.status_code == 200:
                    return
            except requests.exceptions.RequestException:
                pass
            time.sleep(0.5)
        raise RuntimeError('Gateway failed to start with TLS enabled')

    def test_https_health_endpoint(self):
        """Test that health endpoint responds over HTTPS."""
        response = requests.get(
            f'{self.API_BASE}/health',
            verify=False,
            timeout=5
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertEqual(data['status'], 'healthy')

    def test_https_root_endpoint_shows_tls_enabled(self):
        """Test that root endpoint shows TLS capability."""
        response = requests.get(
            f'{self.API_BASE}/',
            verify=False,
            timeout=5
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertTrue(data['capabilities']['tls'])
        self.assertIn('tls', data)
        self.assertTrue(data['tls']['enabled'])
        self.assertEqual(data['tls']['min_version'], '1.2')

    def test_https_version_info_endpoint(self):
        """Test version-info endpoint over HTTPS."""
        response = requests.get(
            f'{self.API_BASE}/version-info',
            verify=False,
            timeout=5
        )
        self.assertEqual(response.status_code, 200)
        data = response.json()
        self.assertIn('version', data)

    def test_https_areas_endpoint(self):
        """Test areas endpoint over HTTPS."""
        response = requests.get(
            f'{self.API_BASE}/areas',
            verify=False,
            timeout=5
        )
        self.assertEqual(response.status_code, 200)
        # Response should be a JSON array
        self.assertIsInstance(response.json(), list)

    def test_https_components_endpoint(self):
        """Test components endpoint over HTTPS."""
        response = requests.get(
            f'{self.API_BASE}/components',
            verify=False,
            timeout=5
        )
        self.assertEqual(response.status_code, 200)
        self.assertIsInstance(response.json(), list)

    def test_https_with_ca_verification(self):
        """Test HTTPS with proper CA certificate verification."""
        # Use the CA certificate for verification
        response = requests.get(
            f'{self.API_BASE}/health',
            verify=_ca_file,
            timeout=5
        )
        self.assertEqual(response.status_code, 200)

    def test_http_connection_rejected(self):
        """Test that plain HTTP connections are rejected when TLS is enabled."""
        # Try to connect with HTTP (should fail)
        with self.assertRaises(requests.exceptions.RequestException):
            requests.get(
                'http://127.0.0.1:8443/api/v1/health',
                timeout=2
            )


@launch_testing.post_shutdown_test()
class TestTlsCleanup(unittest.TestCase):
    """Cleanup tests after shutdown."""

    def test_cleanup_certificates(self):
        """Clean up temporary certificate directory."""
        if os.path.exists(_cert_dir):
            shutil.rmtree(_cert_dir)
        self.assertFalse(os.path.exists(_cert_dir))
