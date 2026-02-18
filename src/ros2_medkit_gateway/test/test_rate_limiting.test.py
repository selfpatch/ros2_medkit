#!/usr/bin/env python3
# Copyright 2026 eclipse0922
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

import os
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests


def get_coverage_env():
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


GATEWAY_HOST = '127.0.0.1'
GATEWAY_PORT = 8087
ALLOWED_ORIGIN = 'http://localhost:5173'
API_BASE_PATH = '/api/v1'


def generate_test_description():
    coverage_env = get_coverage_env()

    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='gateway_rate_limit_integration',
        output='screen',
        parameters=[{
            'server.host': GATEWAY_HOST,
            'server.port': GATEWAY_PORT,
            'rate_limiting.enabled': True,
            'rate_limiting.global_requests_per_minute': 100,
            'rate_limiting.client_requests_per_minute': 2,
            'rate_limiting.client_cleanup_interval_seconds': 60,
            'rate_limiting.client_max_idle_seconds': 120,
            'cors.allowed_origins': [ALLOWED_ORIGIN],
            'cors.allowed_methods': ['GET', 'PUT', 'OPTIONS'],
            'cors.allowed_headers': ['Content-Type', 'Accept', 'Authorization'],
            'cors.allow_credentials': False,
            'cors.max_age_seconds': 3600,
        }],
        additional_env=coverage_env,
    )

    delayed_test = TimerAction(
        period=2.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return LaunchDescription([
        gateway_node,
        delayed_test,
    ])


class TestRateLimitingIntegration(unittest.TestCase):
    BASE_URL = f'http://{GATEWAY_HOST}:{GATEWAY_PORT}{API_BASE_PATH}'

    @classmethod
    def setUpClass(cls):
        max_retries = 30
        for _ in range(max_retries):
            try:
                response = requests.get(f'{cls.BASE_URL}/health', timeout=1)
                if response.status_code == 200:
                    return
            except requests.exceptions.ConnectionError:
                pass
            time.sleep(0.5)
        raise RuntimeError('Gateway failed to start within timeout')

    def _request_until_rate_limited(self):
        headers = {'Origin': ALLOWED_ORIGIN}
        for _ in range(10):
            response = requests.get(f'{self.BASE_URL}/health', headers=headers, timeout=5)
            if response.status_code == 429:
                return response
        self.fail('Expected to receive 429 but did not within 10 requests')

    def test_01_options_preflight_is_not_rate_limited(self):
        for _ in range(5):
            response = requests.options(
                f'{self.BASE_URL}/health',
                headers={
                    'Origin': ALLOWED_ORIGIN,
                    'Access-Control-Request-Method': 'GET',
                    'Access-Control-Request-Headers': 'Content-Type',
                },
                timeout=5,
            )
            self.assertEqual(response.status_code, 204)
            self.assertEqual(response.headers.get('Access-Control-Allow-Origin'), ALLOWED_ORIGIN)
            self.assertIn('GET', response.headers.get('Access-Control-Allow-Methods', ''))
            self.assertNotIn('X-RateLimit-Limit', response.headers)
            self.assertNotIn('Retry-After', response.headers)

    def test_02_over_limit_returns_429_with_rate_headers(self):
        response = self._request_until_rate_limited()

        self.assertEqual(response.status_code, 429)
        self.assertIn('Retry-After', response.headers)
        self.assertIn('X-RateLimit-Limit', response.headers)
        self.assertIn('X-RateLimit-Remaining', response.headers)
        self.assertIn('X-RateLimit-Reset', response.headers)

        self.assertEqual(response.headers.get('X-RateLimit-Remaining'), '0')
        self.assertGreater(int(response.headers.get('Retry-After', '0')), 0)

    def test_03_rate_limited_response_keeps_cors_headers(self):
        response = self._request_until_rate_limited()

        self.assertEqual(response.status_code, 429)
        self.assertEqual(response.headers.get('Access-Control-Allow-Origin'), ALLOWED_ORIGIN)
        exposed = response.headers.get('Access-Control-Expose-Headers', '')
        self.assertIn('Retry-After', exposed)
        self.assertIn('X-RateLimit-Limit', exposed)
        self.assertIn('X-RateLimit-Remaining', exposed)
        self.assertIn('X-RateLimit-Reset', exposed)
