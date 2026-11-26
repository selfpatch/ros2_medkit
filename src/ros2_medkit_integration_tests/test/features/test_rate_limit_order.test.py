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
Who hears the rate limiter, and what an OPTIONS request costs.

Four gateways with real limits. Each case spends the allowance of its own
path, because the limiter keeps one bucket per client and endpoint pattern, so
the cases do not depend on each other or on the order they run in. Nothing here
waits for a bucket to refill: the allowance is spent with requests.

- closed: authentication on for every route, CORS on for one origin.
- write: authentication on for writes only, so GET /health needs no credential.
- open: authentication off, CORS off.
- none: authentication on, `require_auth_for: none`.
"""

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

CLOSED_BASE_URL = f'http://127.0.0.1:{get_test_port(0)}{API_BASE_PATH}'
WRITE_BASE_URL = f'http://127.0.0.1:{get_test_port(1)}{API_BASE_PATH}'
OPEN_BASE_URL = f'http://127.0.0.1:{get_test_port(2)}{API_BASE_PATH}'
NONE_BASE_URL = f'http://127.0.0.1:{get_test_port(3)}{API_BASE_PATH}'

ALLOWED_ORIGIN = 'https://ui.example'
OTHER_ORIGIN = 'https://other.example'
JWT_SECRET = 'rate_limit_order_integration_secret_key_0123456789'
CLIENT_ID = 'diagbox'
CLIENT_SECRET = 'diagbox_client_secret'
VIEWER_ID = 'viewer'
VIEWER_SECRET = 'viewer_client_secret'

# Requests a path admits per minute. Small, so a case spends it in a few calls.
PATH_LIMIT = 4
# Upper bound on the requests a case sends while looking for the first 429.
MAX_ATTEMPTS = 3 * PATH_LIMIT
# The default bucket and the global bucket are far above anything a case sends.
DEFAULT_LIMIT = 100
GLOBAL_LIMIT = 1000

AUTH_PARAMS = {
    'auth.enabled': True,
    'auth.issuer': 'ros2_medkit_gateway',
    'auth.jwt_secret': JWT_SECRET,
    'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin', f'{VIEWER_ID}:{VIEWER_SECRET}:viewer'],
}


def _limits(*paths):
    return {
        'rate_limiting.enabled': True,
        'rate_limiting.global_requests_per_minute': GLOBAL_LIMIT,
        'rate_limiting.client_requests_per_minute': DEFAULT_LIMIT,
        'rate_limiting.endpoint_limits': [f'{API_BASE_PATH}/{p}:{PATH_LIMIT}' for p in paths],
    }


def generate_test_description():
    closed = create_gateway_node(
        port=get_test_port(0),
        name='gateway_closed',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.require_auth_for': 'all',
            'cors.allowed_origins': [ALLOWED_ORIGIN],
            **AUTH_PARAMS,
            **_limits('apps', 'areas', 'components', 'functions', 'faults', 'health',
                      'auth/authorize'),
        },
    )
    write = create_gateway_node(
        port=get_test_port(1),
        name='gateway_write_mode',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.require_auth_for': 'write',
            **AUTH_PARAMS,
            **_limits('health'),
        },
    )
    open_gw = create_gateway_node(
        port=get_test_port(2),
        name='gateway_open',
        extra_params={
            'server.host': '127.0.0.1',
            **_limits('apps', 'areas', 'components'),
        },
    )
    none_mode = create_gateway_node(
        port=get_test_port(3),
        name='gateway_auth_none',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.require_auth_for': 'none',
            **AUTH_PARAMS,
            **_limits('apps'),
        },
    )
    return launch.LaunchDescription([
        closed, write, open_gw, none_mode, launch_testing.actions.ReadyToTest(),
    ]), {}


def _limiter_headers(resp):
    return [h for h in resp.headers
            if h.lower().startswith('x-ratelimit') or h.lower() == 'retry-after']


def _mint_token(base_url, client_id=CLIENT_ID, client_secret=CLIENT_SECRET):
    resp = requests.post(
        f'{base_url}/auth/authorize',
        json={'grant_type': 'client_credentials', 'client_id': client_id,
              'client_secret': client_secret},
        timeout=30,
    )
    assert resp.status_code == 200, f'token request failed: {resp.status_code} {resp.text}'
    return {'Authorization': f'Bearer {resp.json()["access_token"]}'}


def _until_429(send):
    """Send until the first 429. Returns (statuses before it, the 429 response)."""
    seen = []
    for _ in range(MAX_ATTEMPTS):
        resp = send()
        if resp.status_code == 429:
            return seen, resp
        seen.append(resp.status_code)
    raise AssertionError(f'no 429 within {MAX_ATTEMPTS} requests: {seen}')


class TestClosedGatewayLimiter(GatewayTestCase):
    """Authentication on for every route, CORS on."""

    BASE_URL = CLOSED_BASE_URL

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        # Two requests against the 'auth/authorize' bucket.
        cls.auth = _mint_token(CLOSED_BASE_URL)
        cls.viewer = _mint_token(CLOSED_BASE_URL, VIEWER_ID, VIEWER_SECRET)

    def test_01_anonymous_caller_gets_401_and_no_limiter_state(self):
        """Under and over the limit alike, the anonymous answer is the 401."""
        for i in range(2 * PATH_LIMIT):
            resp = requests.get(f'{CLOSED_BASE_URL}/apps', timeout=15)
            with self.subTest(request=i):
                self.assertEqual(resp.status_code, 401, resp.text)
                self.assertIn('WWW-Authenticate', resp.headers)
                self.assertEqual(_limiter_headers(resp), [])
        # The anonymous calls spent the allowance of this path: a credentialed
        # request to it is refused.
        refused = requests.get(f'{CLOSED_BASE_URL}/apps', headers=self.auth, timeout=15)
        self.assertEqual(refused.status_code, 429, refused.text)
        # With a credential the same gateway answers elsewhere: the 401s were
        # not a closed door for everyone.
        ok = requests.get(f'{CLOSED_BASE_URL}/version-info', headers=self.auth, timeout=15)
        self.assertEqual(ok.status_code, 200, ok.text)

    def test_02_garbage_token_over_the_limit_gets_a_bare_429(self):
        """The limiter answers before the token is looked at, and says nothing."""
        garbage = {'Authorization': 'Bearer this-is-not-a-token'}
        before, refused = _until_429(
            lambda: requests.get(f'{CLOSED_BASE_URL}/areas', headers=garbage, timeout=15))
        self.assertEqual(before, [401] * PATH_LIMIT, 'the allowance was not spent on 401s first')
        self.assertEqual(
            _limiter_headers(refused), [],
            f'a bare 429 carried limiter state: {dict(refused.headers)}')
        body = refused.json()
        self.assertEqual(body.get('error_code'), 'rate-limit-exceeded', body)
        self.assertEqual(body.get('parameters'), {}, body)
        self.assertNotRegex(body.get('message', ''), r'\d', body)

    def test_03_valid_token_is_answered_while_allowance_is_left_then_refused_bare(self):
        """The limiter answers before the token is verified, so a valid token is bare too."""
        before, refused = _until_429(
            lambda: requests.get(f'{CLOSED_BASE_URL}/components', headers=self.auth, timeout=15))
        self.assertEqual(before, [200] * PATH_LIMIT)
        self.assertEqual(
            _limiter_headers(refused), [],
            f'a 429 before verification carried limiter state: {dict(refused.headers)}')
        self.assertEqual(refused.json().get('parameters'), {})

    def test_04_accepted_answer_carries_x_ratelimit_headers(self):
        resp = requests.get(f'{CLOSED_BASE_URL}/version-info', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(resp.headers.get('X-RateLimit-Limit'), str(DEFAULT_LIMIT))
        self.assertIn('X-RateLimit-Remaining', resp.headers)

    def test_05_public_auth_route_over_the_limit_gets_a_full_429(self):
        """/auth/authorize needs no credential, so it hears the limiter."""
        bad = {'grant_type': 'client_credentials', 'client_id': CLIENT_ID,
               'client_secret': 'wrong'}
        before, refused = _until_429(
            lambda: requests.post(f'{CLOSED_BASE_URL}/auth/authorize', json=bad, timeout=15))
        self.assertTrue(before, 'no answer before the 429')
        self.assertNotIn(429, before)
        for name in ('Retry-After', 'X-RateLimit-Limit', 'X-RateLimit-Remaining',
                     'X-RateLimit-Reset'):
            self.assertIn(name, refused.headers, dict(refused.headers))

    def test_06_preflights_are_answered_and_not_metered(self):
        for _ in range(3 * PATH_LIMIT):
            resp = requests.options(
                f'{CLOSED_BASE_URL}/functions',
                headers={'Origin': ALLOWED_ORIGIN, 'Access-Control-Request-Method': 'GET'},
                timeout=15)
            self.assertEqual(resp.status_code, 204)
            self.assertEqual(resp.headers.get('Access-Control-Allow-Origin'), ALLOWED_ORIGIN)
            self.assertEqual(_limiter_headers(resp), [])
        # The bucket of that path is untouched: all of its allowance is there.
        for i in range(PATH_LIMIT):
            ok = requests.get(f'{CLOSED_BASE_URL}/functions', headers=self.auth, timeout=15)
            self.assertEqual(ok.status_code, 200, f'request {i}: {ok.text}')
        refused = requests.options(
            f'{CLOSED_BASE_URL}/functions',
            headers={'Origin': OTHER_ORIGIN, 'Access-Control-Request-Method': 'GET'},
            timeout=15)
        self.assertEqual(refused.status_code, 403)

    def test_06b_denied_preflights_do_not_spend_allowance(self):
        for _ in range(3 * PATH_LIMIT):
            resp = requests.options(
                f'{CLOSED_BASE_URL}/health',
                headers={'Origin': OTHER_ORIGIN, 'Access-Control-Request-Method': 'GET'},
                timeout=15)
            self.assertEqual(resp.status_code, 403)
        ok = requests.get(f'{CLOSED_BASE_URL}/health', headers=self.auth, timeout=15)
        self.assertEqual(ok.status_code, 200, ok.text)

    def test_06c_rbac_refusal_carries_no_limiter_state(self):
        """A verified viewer refused on a write route gets the 403 alone."""
        refused = requests.post(
            f'{CLOSED_BASE_URL}/components/test/operations/calibrate',
            json={}, headers=self.viewer, timeout=15)
        self.assertEqual(refused.status_code, 403, refused.text)
        self.assertEqual(_limiter_headers(refused), [])
        ok = requests.get(f'{CLOSED_BASE_URL}/version-info', headers=self.viewer, timeout=15)
        self.assertEqual(ok.status_code, 200, ok.text)
        self.assertIn('X-RateLimit-Limit', ok.headers)

    def test_07_plain_options_is_authenticated_and_metered(self):
        """No Access-Control-Request-Method: an ordinary request to its route."""
        headers = {'Origin': ALLOWED_ORIGIN}
        for i in range(PATH_LIMIT):
            resp = requests.options(f'{CLOSED_BASE_URL}/faults', headers=headers, timeout=15)
            with self.subTest(request=i):
                self.assertEqual(resp.status_code, 401, resp.text)
                self.assertEqual(_limiter_headers(resp), [])
        # The allowance is gone: a credentialed GET of that path is refused.
        refused = requests.get(f'{CLOSED_BASE_URL}/faults', headers=self.auth, timeout=15)
        self.assertEqual(refused.status_code, 429, refused.text)
        # Another path still has its own allowance.
        ok = requests.get(f'{CLOSED_BASE_URL}/version-info', headers=self.auth, timeout=15)
        self.assertEqual(ok.status_code, 200, ok.text)


class TestWriteModeGatewayLimiter(GatewayTestCase):
    """GET /health needs no credential here."""

    BASE_URL = WRITE_BASE_URL

    def test_01_public_route_hears_the_limiter(self):
        before, refused = _until_429(
            lambda: requests.get(f'{WRITE_BASE_URL}/health', timeout=15))
        self.assertTrue(before and set(before) == {200}, before)
        for name in ('Retry-After', 'X-RateLimit-Limit', 'X-RateLimit-Remaining',
                     'X-RateLimit-Reset'):
            self.assertIn(name, refused.headers, dict(refused.headers))
        self.assertEqual(refused.json().get('error_code'), 'rate-limit-exceeded')


class TestOpenGatewayLimiter(GatewayTestCase):
    """Authentication off, CORS off."""

    BASE_URL = OPEN_BASE_URL

    def test_01_over_the_limit_the_full_429_on_any_route(self):
        before, refused = _until_429(
            lambda: requests.get(f'{OPEN_BASE_URL}/components', timeout=15))
        self.assertEqual(before, [200] * PATH_LIMIT)
        for name in ('Retry-After', 'X-RateLimit-Limit', 'X-RateLimit-Remaining',
                     'X-RateLimit-Reset'):
            self.assertIn(name, refused.headers, dict(refused.headers))
        self.assertGreater(refused.json()['parameters']['retry_after'], 0)

    def test_02_options_spends_allowance(self):
        for _ in range(PATH_LIMIT):
            requests.options(f'{OPEN_BASE_URL}/areas', timeout=15)
        refused = requests.get(f'{OPEN_BASE_URL}/areas', timeout=15)
        self.assertEqual(refused.status_code, 429, refused.text)
        # Another path still has its own allowance.
        ok = requests.get(f'{OPEN_BASE_URL}/apps', timeout=15)
        self.assertEqual(ok.status_code, 200, ok.text)


class TestAuthNoneGatewayLimiter(GatewayTestCase):
    """Authentication on, but no route requires it."""

    BASE_URL = NONE_BASE_URL

    def test_01_over_the_limit_the_full_429_with_headers(self):
        garbage = {'Authorization': 'Bearer this-is-not-a-token'}
        before, refused = _until_429(
            lambda: requests.get(f'{NONE_BASE_URL}/apps', headers=garbage, timeout=15))
        self.assertEqual(before, [200] * PATH_LIMIT)
        for name in ('Retry-After', 'X-RateLimit-Limit', 'X-RateLimit-Remaining',
                     'X-RateLimit-Reset'):
            self.assertIn(name, refused.headers, dict(refused.headers))
        self.assertGreater(refused.json()['parameters']['retry_after'], 0)
        ok = requests.get(f'{NONE_BASE_URL}/areas', timeout=15)
        self.assertEqual(ok.status_code, 200, ok.text)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
