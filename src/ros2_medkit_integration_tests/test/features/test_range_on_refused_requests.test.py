#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""A refusal before routing is never cut by the request's ``Range``.

A client resuming a download sends ``Range: bytes=<offset>-``. When the token
has expired, the 401 comes from the pre-routing handler, before any route runs.
cpp-httplib applies the request's ranges to every error body after the global
error handler, so that 401 came back as an empty 416, and the client never
learned that it had to authenticate again. The same held for a 403 and for a
429.

Two gateways: one with authentication required for every request, one with a
per-client rate limit of two requests a minute.

@verifies REQ_INTEROP_086
"""

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
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

AUTH_PORT = get_test_port(0)
LIMITED_PORT = get_test_port(1)
AUTH_BASE_URL = f'http://127.0.0.1:{AUTH_PORT}{API_BASE_PATH}'
LIMITED_BASE_URL = f'http://127.0.0.1:{LIMITED_PORT}{API_BASE_PATH}'

# The resume of a recording file. Nothing by these names exists, which does not
# matter: every request below is refused before routing.
RECORDING_PATH = '/apps/lidar_sensor/bulk-data/rosbags/fault_LIDAR_RANGE_INVALID_1'
FILE_PATH = f'{RECORDING_PATH}/files/1'
RESUME_RANGES = ('bytes=5000000-', 'bytes=10-', 'bytes=0-3')
STARTUP_TIMEOUT_SEC = 60.0


@pytest.mark.launch_test
def generate_test_description():
    """Launch an authenticated gateway and a rate-limited one."""
    auth_gateway = create_gateway_node(
        port=AUTH_PORT,
        name='gateway_range_auth',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.jwt_secret': 'test_secret_key_for_range_on_refused_requests_1234',
            'auth.jwt_algorithm': 'HS256',
            'auth.token_expiry_seconds': 3600,
            'auth.require_auth_for': 'all',
            'auth.issuer': 'test_gateway',
            'auth.clients': ['viewer:viewer_secret:viewer'],
        },
    )
    limited_gateway = create_gateway_node(
        port=LIMITED_PORT,
        name='gateway_range_limited',
        extra_params={
            'server.host': '127.0.0.1',
            'rate_limiting.enabled': True,
            'rate_limiting.global_requests_per_minute': 100,
            'rate_limiting.client_requests_per_minute': 2,
            'rate_limiting.client_cleanup_interval_seconds': 60,
            'rate_limiting.client_max_idle_seconds': 120,
        },
    )
    return launch.LaunchDescription([
        auth_gateway,
        limited_gateway,
        launch_testing.actions.ReadyToTest(),
    ]), {'auth_gateway': auth_gateway, 'limited_gateway': limited_gateway}


def _wait_until_answering(url):
    """Wait until the gateway answers anything at all on @p url."""
    deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
    while time.monotonic() < deadline:
        try:
            return requests.get(url, timeout=2)
        except requests.exceptions.ConnectionError:
            time.sleep(0.2)
    raise AssertionError(f'{url} never answered')


class TestRangeOnRefusedRequests(unittest.TestCase):
    """Each refusal carries its own status and its whole body."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(f'{AUTH_BASE_URL}/health')
        response = requests.post(
            f'{AUTH_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'viewer',
                'client_secret': 'viewer_secret',
            },
            timeout=5,
        )
        assert response.status_code == 200, response.text
        cls.viewer_token = response.json()['access_token']

    def _assert_whole_refusal(self, response, status, value):
        self.assertEqual(response.status_code, status, f'{value}: {response.text!r}')
        self.assertNotIn('Content-Range', response.headers, value)
        body = response.json()
        self.assertIsInstance(body, dict, value)
        return body

    def test_01_a_resume_without_a_token_gets_the_401(self):
        """A ranged request with no token is answered 401 with the whole body."""
        for value in RESUME_RANGES:
            response = requests.get(
                f'{AUTH_BASE_URL}{FILE_PATH}', headers={'Range': value}, timeout=5)
            body = self._assert_whole_refusal(response, 401, value)
            self.assertIn('error', body, value)
            self.assertIn('Bearer', response.headers.get('WWW-Authenticate', ''), value)

    def test_02_a_resume_with_a_token_that_is_no_longer_valid_gets_the_401(self):
        """An invalid token is the expired-token case: the same 401, uncut."""
        for value in RESUME_RANGES:
            response = requests.get(
                f'{AUTH_BASE_URL}{FILE_PATH}',
                headers={'Range': value, 'Authorization': 'Bearer no.longer.valid'},
                timeout=5)
            body = self._assert_whole_refusal(response, 401, value)
            self.assertEqual(body.get('error'), 'invalid_token', value)

    def test_03_a_ranged_request_the_role_may_not_make_gets_the_403(self):
        """A viewer deleting a recording is refused 403 with the whole body."""
        for value in RESUME_RANGES:
            response = requests.delete(
                f'{AUTH_BASE_URL}{RECORDING_PATH}',
                headers={'Range': value, 'Authorization': f'Bearer {self.viewer_token}'},
                timeout=5)
            body = self._assert_whole_refusal(response, 403, value)
            self.assertIn('error', body, value)

    def test_04_a_ranged_request_over_the_rate_limit_gets_the_429(self):
        """Past the per-client limit, a ranged request is answered 429 with the whole body."""
        _wait_until_answering(f'{LIMITED_BASE_URL}/health')
        statuses = []
        for _ in range(10):
            response = requests.get(
                f'{LIMITED_BASE_URL}{FILE_PATH}', headers={'Range': 'bytes=5000000-'}, timeout=5)
            statuses.append(response.status_code)
            if response.status_code == 429:
                body = self._assert_whole_refusal(response, 429, 'bytes=5000000-')
                self.assertEqual(body.get('error_code'), 'rate-limit-exceeded', body)
                self.assertIn('Retry-After', response.headers)
                return
        self.fail(f'no 429 within 10 requests, got {statuses}')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown checks."""

    def test_exit_codes(self, proc_info):
        """Both gateways exit cleanly."""
        for info in proc_info:
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited {info.returncode}')
