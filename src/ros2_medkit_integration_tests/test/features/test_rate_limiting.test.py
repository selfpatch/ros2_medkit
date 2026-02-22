#!/usr/bin/env python3
# Copyright 2026 eclipse0922, bburda
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
Rate limiting integration tests for ROS 2 Medkit Gateway.

Launches a gateway with token-bucket rate limiting enabled:
- Global: 100 requests/minute
- Per-client: 2 requests/minute (low limit to trigger 429 quickly)

Tests verify:
1. OPTIONS preflight requests bypass rate limiting
2. Exceeding client limit returns 429 with correct headers
3. CORS headers are preserved on 429 responses

"""

import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

# Test configuration
GATEWAY_HOST = '127.0.0.1'
GATEWAY_PORT = 8087
ALLOWED_ORIGIN = 'http://localhost:5173'


def generate_test_description():
    """Generate launch description with rate-limited gateway."""
    gateway_node = create_gateway_node(
        port=GATEWAY_PORT,
        extra_params={
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
        },
    )

    delayed_test = TimerAction(
        period=2.0,
        actions=[launch_testing.actions.ReadyToTest()],
    )

    return LaunchDescription([
        gateway_node,
        delayed_test,
    ])


class TestRateLimitingIntegration(GatewayTestCase):
    """Rate limiting integration tests.

    Uses a very low per-client limit (2 req/min) so tests can trigger 429
    without sending hundreds of requests.
    """

    BASE_URL = f'http://{GATEWAY_HOST}:{GATEWAY_PORT}{API_BASE_PATH}'

    def _request_until_rate_limited(self):
        """Send requests until a 429 is received (max 10 attempts)."""
        headers = {'Origin': ALLOWED_ORIGIN}
        for _ in range(10):
            response = requests.get(
                f'{self.BASE_URL}/health', headers=headers, timeout=5
            )
            if response.status_code == 429:
                return response
        self.fail('Expected to receive 429 but did not within 10 requests')

    def test_01_options_preflight_is_not_rate_limited(self):
        """OPTIONS preflight requests should bypass rate limiting entirely."""
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
            self.assertEqual(
                response.headers.get('Access-Control-Allow-Origin'), ALLOWED_ORIGIN
            )
            self.assertIn('GET', response.headers.get('Access-Control-Allow-Methods', ''))
            self.assertNotIn('X-RateLimit-Limit', response.headers)
            self.assertNotIn('Retry-After', response.headers)

    def test_02_over_limit_returns_429_with_rate_headers(self):
        """Exceeding per-client limit returns 429 with rate limit headers."""
        response = self._request_until_rate_limited()

        self.assertEqual(response.status_code, 429)
        self.assertIn('Retry-After', response.headers)
        self.assertIn('X-RateLimit-Limit', response.headers)
        self.assertIn('X-RateLimit-Remaining', response.headers)
        self.assertIn('X-RateLimit-Reset', response.headers)

        self.assertEqual(response.headers.get('X-RateLimit-Remaining'), '0')
        self.assertGreater(int(response.headers.get('Retry-After', '0')), 0)

    def test_03_rate_limited_response_keeps_cors_headers(self):
        """429 responses should still include CORS headers."""
        response = self._request_until_rate_limited()

        self.assertEqual(response.status_code, 429)
        self.assertEqual(
            response.headers.get('Access-Control-Allow-Origin'), ALLOWED_ORIGIN
        )
        exposed = response.headers.get('Access-Control-Expose-Headers', '')
        self.assertIn('Retry-After', exposed)
        self.assertIn('X-RateLimit-Limit', exposed)
        self.assertIn('X-RateLimit-Remaining', exposed)
        self.assertIn('X-RateLimit-Reset', exposed)


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
