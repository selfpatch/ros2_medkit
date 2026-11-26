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

"""A token is worth what the gateway that answers it says.

An aggregator (A) and a peer (B) share one JWT configuration: secret, issuer
and client id. A lists the client as ``admin``, B lists it as ``viewer``.
``aggregation.forward_auth`` carries the caller's token from A to B.

- B accepts a token A minted, directly and when A forwards it.
- B grants the role its own client table lists, not the token's ``role`` claim.
- B refuses a token whose signature does not verify, a client it does not
  list, and an anonymous read.
- ``POST /auth/revoke`` on B refuses A's token on B. A still accepts it.

``aggregation.peer_auth_header`` is a separate hand-minted token, so A's own
health check and entity fetch do not depend on the token under test.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import base64
import hashlib
import hmac
import json
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
    DISCOVERY_INTERVAL,
    DISCOVERY_TIMEOUT,
    GATEWAY_STARTUP_TIMEOUT,
    get_test_domain_id,
    get_test_port,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
)

AGG_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
AGG_BASE_URL = f'http://127.0.0.1:{AGG_PORT}{API_BASE_PATH}'
PEER_BASE_URL = f'http://127.0.0.1:{PEER_PORT}{API_BASE_PATH}'

# Separate DDS domains, so A learns B's entities only over HTTP.
AGG_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

JWT_SECRET = 'forward_auth_cross_gateway_secret_key_0123456789'
JWT_ISSUER = 'ros2_medkit_gateway'
CLIENT_ID = 'aggregator'
CLIENT_SECRET = 'aggregator_client_secret'

PEER_NODES = ['pressure_sensor', 'actuator']
PEER_APP = 'pressure_sensor'
# No gateway here has this app. The role check runs before the entity lookup.
ABSENT_APP = 'an_app_no_gateway_here_has'
STRANGER = 'a_client_no_gateway_here_lists'


def _b64(raw):
    """Encode base64url without padding, as JWT does."""
    return base64.urlsafe_b64encode(raw).rstrip(b'=')


def _mint_peer_credential(lifetime_sec=3600, subject=CLIENT_ID):
    """Sign an HS256 access token with the shared secret.

    Minted here because it is a launch parameter and no gateway runs yet. It
    carries no ``refresh_token_id``, so no refresh record can refuse it.
    """
    header = {'alg': 'HS256', 'typ': 'access'}
    now = int(time.time())
    payload = {
        'iss': JWT_ISSUER,
        'sub': subject,
        'iat': now,
        'exp': now + lifetime_sec,
        'jti': 'forward-auth-peer-credential',
        'role': 'admin',
    }
    signing_input = b'.'.join([
        _b64(json.dumps(header, separators=(',', ':')).encode()),
        _b64(json.dumps(payload, separators=(',', ':')).encode()),
    ])
    signature = hmac.new(JWT_SECRET.encode(), signing_input, hashlib.sha256).digest()
    return (signing_input + b'.' + _b64(signature)).decode()


def _auth_params(role):
    """Return the shared JWT configuration with this gateway's role for the shared client."""
    return {
        'server.host': '127.0.0.1',
        'auth.enabled': True,
        'auth.require_auth_for': 'all',
        'auth.issuer': JWT_ISSUER,
        'auth.jwt_secret': JWT_SECRET,
        'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:{role}'],
    }


@pytest.mark.launch_test
def generate_test_description():
    """Launch an aggregator and a peer, both closed, sharing one JWT configuration."""
    peer_env = {'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)}

    aggregator = create_gateway_node(
        port=AGG_PORT,
        name='forward_auth_aggregator',
        extra_params={
            **_auth_params('admin'),
            'aggregation.enabled': True,
            'aggregation.timeout_ms': 5000,
            'aggregation.announce': False,
            'aggregation.discover': False,
            'aggregation.forward_auth': True,
            'aggregation.peer_auth_header': f'Bearer {_mint_peer_credential()}',
            'aggregation.peer_urls': [f'http://127.0.0.1:{PEER_PORT}'],
            'aggregation.peer_names': ['closed_peer'],
        },
        extra_env={'ROS_DOMAIN_ID': str(AGG_DOMAIN_ID)},
    )

    peer = create_gateway_node(
        port=PEER_PORT,
        name='forward_auth_peer',
        extra_params=_auth_params('viewer'),
        extra_env=peer_env,
    )

    peer_demo_nodes = create_demo_nodes(PEER_NODES, lidar_faulty=False, extra_env=peer_env)

    return launch.LaunchDescription([
        aggregator,
        peer,
        *peer_demo_nodes,
        launch_testing.actions.ReadyToTest(),
    ]), {'aggregator': aggregator, 'peer': peer}


def _poll(predicate, timeout, what):
    """Call `predicate` until it returns a truthy value, and return that value.

    Fails with `what` and the last value seen when `timeout` passes first.
    """
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
        except requests.RequestException as exc:
            last = exc
        else:
            if last:
                return last
        time.sleep(DISCOVERY_INTERVAL)
    raise AssertionError(f'{what} within {timeout}s; last seen: {last!r}')


def _wait_until_answering(url):
    """Wait for any HTTP answer from /health. A refusal also means the process listens."""
    _poll(lambda: requests.get(f'{url}/health', timeout=2).status_code in (200, 401, 403),
          GATEWAY_STARTUP_TIMEOUT, f'{url} did not answer')


def _authorize(url):
    resp = requests.post(
        f'{url}/auth/authorize',
        json={
            'grant_type': 'client_credentials',
            'client_id': CLIENT_ID,
            'client_secret': CLIENT_SECRET,
        },
        timeout=30,
    )
    assert resp.status_code == 200, f'{url} issued no token: {resp.status_code} {resp.text}'
    return resp.json()


def _bearer(token):
    return {'Authorization': f'Bearer {token}'}


def _error_code(resp):
    try:
        return resp.json().get('error')
    except ValueError:
        return None


class TestForwardAuthAcrossGateways(unittest.TestCase):
    """A shared JWT configuration makes one gateway's token good at the other."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(AGG_BASE_URL)
        _wait_until_answering(PEER_BASE_URL)
        cls.agg_auth = _bearer(_authorize(AGG_BASE_URL)['access_token'])

    def test_01_the_peer_accepts_a_token_the_aggregator_minted(self):
        """B holds no refresh record for this token and judges it on signature, expiry, client."""
        resp = requests.get(f'{PEER_BASE_URL}/areas', headers=self.agg_auth, timeout=15)
        self.assertEqual(
            resp.status_code, 200,
            f'the peer answered {resp.status_code} to a token minted by a gateway '
            f'sharing its secret, issuer and client. Body: {resp.text[:300]}')

    def test_02_a_forwarded_read_is_carried_by_the_callers_token(self):
        """A forwards a read on B's app with the caller's token, and B answers it."""
        def merged():
            resp = requests.get(f'{AGG_BASE_URL}/apps', headers=self.agg_auth, timeout=15)
            self.assertEqual(resp.status_code, 200, resp.text)
            return PEER_APP in {item.get('id') for item in resp.json().get('items', [])}

        _poll(merged, DISCOVERY_TIMEOUT,
              f'the aggregator did not list {PEER_APP}. Merging uses '
              f'aggregation.peer_auth_header, so this is a fixture problem')

        resp = requests.get(
            f'{AGG_BASE_URL}/apps/{PEER_APP}/data', headers=self.agg_auth, timeout=15)
        self.assertEqual(
            resp.status_code, 200,
            f'a forwarded read answered {resp.status_code}; the peer refused the '
            f'token the aggregator issued and forwarded. Body: {resp.text[:300]}')

    def test_03_the_peer_refuses_a_token_that_does_not_verify(self):
        """One base64url character of the signature changed; length and alphabet stay valid."""
        good = _authorize(PEER_BASE_URL)['access_token']
        header, payload, signature = good.split('.')

        alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_'
        replacement = alphabet[(alphabet.index(signature[0]) + 1) % len(alphabet)]
        forged = '.'.join([header, payload, replacement + signature[1:]])
        self.assertEqual(len(forged), len(good))
        self.assertNotEqual(forged, good)

        self.assertEqual(
            requests.get(f'{PEER_BASE_URL}/areas', headers=_bearer(good), timeout=15).status_code,
            200, 'the peer refused its own unaltered token')
        resp = requests.get(f'{PEER_BASE_URL}/areas', headers=_bearer(forged), timeout=15)
        self.assertEqual(resp.status_code, 401, f'forged signature answered {resp.status_code}')

    def test_04_the_peer_refuses_an_anonymous_read(self):
        resp = requests.get(f'{PEER_BASE_URL}/areas', timeout=15)
        self.assertEqual(resp.status_code, 401, resp.text)

    def test_05_a_token_naming_a_client_the_peer_lacks_is_refused(self):
        """The signature verifies, so only B's own client list can refuse this."""
        resp = requests.get(
            f'{PEER_BASE_URL}/areas', headers=_bearer(_mint_peer_credential(subject=STRANGER)),
            timeout=15)
        self.assertEqual(
            resp.status_code, 401,
            f'the peer answered {resp.status_code} to a correctly signed token '
            f'naming a client it does not have')
        resp = requests.get(
            f'{PEER_BASE_URL}/areas', headers=_bearer(_mint_peer_credential()), timeout=15)
        self.assertEqual(resp.status_code, 200, 'the same token for a listed client was refused')

    def test_06_a_refresh_token_is_not_an_access_token(self):
        """A refresh token is refused as a bearer token and does not refresh on another gateway."""
        refresh = _authorize(PEER_BASE_URL).get('refresh_token')
        self.assertTrue(refresh, 'the peer issued no refresh token')

        resp = requests.get(f'{PEER_BASE_URL}/areas', headers=_bearer(refresh), timeout=15)
        self.assertEqual(resp.status_code, 401, 'a refresh token was accepted as an access token')
        self.assertIn('Invalid token type', resp.text)

        refreshed = requests.post(
            f'{AGG_BASE_URL}/auth/token',
            json={'grant_type': 'refresh_token', 'refresh_token': refresh},
            timeout=30)
        self.assertEqual(
            refreshed.status_code, 401,
            f'a gateway that never issued this refresh token answered: {refreshed.text}')
        self.assertEqual(
            refreshed.json(),
            {'error': 'invalid_grant', 'error_description': 'Refresh token not found'})
        own = requests.post(
            f'{PEER_BASE_URL}/auth/token',
            json={'grant_type': 'refresh_token', 'refresh_token': refresh},
            timeout=30)
        self.assertEqual(
            own.status_code, 200, f'the issuing gateway refused its own refresh: {own.text}')

    def test_07_revoking_on_the_peer_refuses_the_token_there(self):
        """Revocation of a foreign refresh token holds on B, and only on B."""
        pair = _authorize(AGG_BASE_URL)
        refresh = pair['refresh_token']
        auth = _bearer(pair['access_token'])

        self.assertEqual(
            requests.get(f'{PEER_BASE_URL}/areas', headers=auth, timeout=15).status_code,
            200, 'the peer refused the token before it was revoked')

        revoked = requests.post(
            f'{PEER_BASE_URL}/auth/revoke', json={'token': refresh}, timeout=30)
        self.assertEqual(revoked.status_code, 200, revoked.text)

        on_peer = requests.get(f'{PEER_BASE_URL}/areas', headers=auth, timeout=15)
        self.assertEqual(
            on_peer.status_code, 401, 'the peer went on accepting a token revoked on it')
        self.assertIn('revoked', on_peer.text)

        self.assertEqual(
            requests.get(f'{AGG_BASE_URL}/areas', headers=auth, timeout=15).status_code,
            200, 'revoking on the peer reached the aggregator, which keeps its own records')

        # A still refreshes the pair. The new access token names the record revoked on B.
        refreshed = requests.post(
            f'{AGG_BASE_URL}/auth/token',
            json={'grant_type': 'refresh_token', 'refresh_token': refresh},
            timeout=30)
        self.assertEqual(refreshed.status_code, 200, refreshed.text)
        later = _bearer(refreshed.json()['access_token'])
        self.assertEqual(
            requests.get(f'{AGG_BASE_URL}/areas', headers=later, timeout=15).status_code, 200)
        self.assertEqual(
            requests.get(f'{PEER_BASE_URL}/areas', headers=later, timeout=15).status_code,
            401, 'the peer accepted a later access token minted from a token revoked on it')


class TestTheRoleComesFromTheAnsweringGateway(unittest.TestCase):
    """A token A minted claims admin. B lists the client as viewer, and B decides."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(AGG_BASE_URL)
        _wait_until_answering(PEER_BASE_URL)
        cls.token = _authorize(AGG_BASE_URL)['access_token']
        cls.auth = _bearer(cls.token)

    def _write(self, base_url, app):
        return requests.post(
            f'{base_url}/apps/{app}/operations/probe/executions',
            headers=self.auth, json={}, timeout=15)

    def test_01_the_token_claims_admin(self):
        """The premise: B has to overrule the claim."""
        payload = self.token.split('.')[1]
        payload += '=' * (-len(payload) % 4)
        claims = json.loads(base64.urlsafe_b64decode(payload))
        self.assertEqual(
            claims.get('role'), 'admin', f'the aggregator did not mint an admin token: {claims}')

    def test_02_a_write_is_refused_on_the_peer(self):
        """A viewer on B may not execute an operation on B's own app."""
        resp = self._write(PEER_BASE_URL, PEER_APP)
        self.assertEqual(
            resp.status_code, 403,
            f'the peer answered {resp.status_code} to a write from a client its '
            f'own table lists as viewer. Body: {resp.text[:300]}')
        self.assertEqual(_error_code(resp), 'insufficient_scope', resp.text[:300])

    def test_03_the_identical_write_is_role_refused_on_the_peer_only(self):
        """Same path, same token, two tables: only the grant differs.

        ABSENT_APP is on neither gateway, so A answers it itself and does not
        forward it to B.
        """
        on_peer = self._write(PEER_BASE_URL, ABSENT_APP)
        self.assertEqual(
            on_peer.status_code, 403,
            f'the peer answered {on_peer.status_code} where its own table lists the '
            f'client as viewer. Body: {on_peer.text[:300]}')
        self.assertEqual(_error_code(on_peer), 'insufficient_scope', on_peer.text[:300])

        on_issuer = self._write(AGG_BASE_URL, ABSENT_APP)
        self.assertNotIn(
            on_issuer.status_code, (401, 403),
            f'the aggregator refused its own admin client: {on_issuer.text[:300]}')

    def test_04_a_read_still_works_on_the_peer(self):
        """A viewer may read, so B refuses the method and not the token."""
        resp = requests.get(f'{PEER_BASE_URL}/areas', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_05_a_client_absent_from_the_peers_table_is_refused(self):
        resp = requests.get(
            f'{PEER_BASE_URL}/areas', headers=_bearer(_mint_peer_credential(subject=STRANGER)),
            timeout=15)
        self.assertEqual(
            resp.status_code, 401,
            f'the peer answered {resp.status_code} to a correctly signed token '
            f'naming a client it does not have')


@launch_testing.post_shutdown_test()
class TestForwardAuthAcrossGatewaysShutdown(unittest.TestCase):
    """Both gateways exit cleanly."""

    def test_exit_codes(self, proc_info, aggregator, peer):
        for proc in (aggregator, peer):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc)
