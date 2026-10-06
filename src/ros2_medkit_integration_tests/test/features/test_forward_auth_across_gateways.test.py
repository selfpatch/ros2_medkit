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

"""A peer accepts a token the aggregator minted, which is what forward_auth needs.

``aggregation.forward_auth`` puts the caller's own ``Authorization`` header on
the request the aggregator makes to a peer, and docs/config/aggregation.rst
describes the deployment it is for: peers that are trusted and **share the same
JWT configuration**. The token therefore arrives at a gateway that never issued
it.

Every access token names the refresh record it was minted from, and those
records live in the issuing process's memory. A gateway that read a missing
record as "invalid" refused every forwarded token, so forward_auth could not
work against any peer that authenticated. The record store is a denylist
instead: a record held and marked revoked refuses, and a record that was never
there says nothing.

THE RULES

C1  A token minted by the aggregator is accepted by the peer on a direct read.
    The cross-instance claim in its simplest form.
C2  With forward_auth on, a read the caller aims at a peer-owned app is carried
    to the peer by the caller's own token and answered. The aggregator's own
    credential cannot stand in for it: the documentation says a caller's token
    wins wherever both apply, so a peer that refuses the forwarded token
    refuses this request.
C3  A token whose signature does not verify is refused by the peer. Without it
    C1 and C2 would pass just as well against a peer that ignored the header,
    and would say nothing about the credential.
C4  The peer refuses an anonymous read, so it is closed at all.

``aggregation.peer_auth_header`` carries a SEPARATE, hand-minted token, so the
aggregator's own connections - the health check and the entity fetch behind
merging - succeed whatever happens to the forwarded one. Without it the peer
answers 401 to the health check, is recorded as offline, and C2 would fail for
a reason that has nothing to do with forwarding. That token deliberately omits
the ``refresh_token_id`` claim, which is what distinguishes it from a token
``/auth/authorize`` issues and is exactly the claim this whole case is about.

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
    get_test_domain_id,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import (
    create_demo_nodes,
    create_gateway_node,
)

AGG_PORT = get_test_port(0)
PEER_PORT = get_test_port(1)
AGG_BASE_URL = f'http://127.0.0.1:{AGG_PORT}{API_BASE_PATH}'
PEER_BASE_URL = f'http://127.0.0.1:{PEER_PORT}{API_BASE_PATH}'

# Separate DDS domains, so the aggregator can only learn the peer's entities
# over HTTP - which is the path a credential travels.
AGG_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

# The shared JWT configuration: same secret, same issuer, same client. This is
# what the aggregation documentation means by peers that share a configuration.
JWT_SECRET = 'forward_auth_cross_gateway_secret_key_0123456789'
JWT_ISSUER = 'ros2_medkit_gateway'
CLIENT_ID = 'aggregator'
CLIENT_SECRET = 'aggregator_client_secret'

PEER_NODES = ['pressure_sensor', 'actuator']
PEER_APP = 'pressure_sensor'
# An app id no gateway in this launch has, for the requests whose subject is the
# role check that runs before any entity lookup.
ABSENT_APP = 'an_app_no_gateway_here_has'

TIMEOUT = DISCOVERY_TIMEOUT * get_time_scale()


def _b64(raw):
    """base64url without padding, which is what JWT uses."""
    return base64.urlsafe_b64encode(raw).rstrip(b'=')


def _mint_peer_credential(lifetime_sec=3600, subject=CLIENT_ID):
    """Sign an HS256 access token the peer accepts, for the aggregator's own use.

    Minted here, because it is needed as a LAUNCH parameter and at that point
    no gateway is running to issue one. Built on
    hmac and hashlib from the standard library so a missing JWT package on one
    CI image cannot fail this for a reason unrelated to what it asserts.

    No ``refresh_token_id`` claim: nothing issued this token, so there is no
    record it could name. That is what keeps this credential independent of the
    behaviour under test.
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


def _auth_params(role='admin'):
    """Build the shared JWT configuration with this gateway's own client table.

    `role` is what THIS gateway grants the shared client id. The two gateways
    are given different values on purpose: the role a token grants has to come
    from the table of the gateway answering the request, so a client that is
    admin on the aggregator and viewer on the peer may only read on the peer.
    """
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
            **_auth_params(),
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
        extra_params=_auth_params(role='viewer'),
        extra_env=peer_env,
    )

    peer_demo_nodes = create_demo_nodes(
        PEER_NODES, lidar_faulty=False, extra_env=peer_env)

    return launch.LaunchDescription([
        aggregator,
        peer,
        launch.actions.TimerAction(period=2.0, actions=peer_demo_nodes),
        launch_testing.actions.ReadyToTest(),
    ]), {'aggregator': aggregator, 'peer': peer}


def _wait_until_answering(url):
    """Any HTTP answer, a refusal included, means the process is listening."""
    deadline = time.monotonic() + TIMEOUT
    while time.monotonic() < deadline:
        try:
            resp = requests.get(f'{url}/health', timeout=2)
            if resp.status_code in (200, 401, 403):
                return
        except requests.RequestException:
            pass
        time.sleep(DISCOVERY_INTERVAL)
    raise AssertionError(f'{url} never answered within {TIMEOUT}s')


def _token_from(url):
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
    return resp.json()['access_token']


class TestForwardAuthAcrossGateways(unittest.TestCase):
    """A shared JWT configuration makes one gateway's token good at the other."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(AGG_BASE_URL)
        _wait_until_answering(PEER_BASE_URL)
        cls.agg_token = _token_from(AGG_BASE_URL)
        cls.agg_auth = {'Authorization': f'Bearer {cls.agg_token}'}

    def _wait_for_merged_app(self):
        """Block until the aggregator lists the peer's app, or fail saying so."""
        deadline = time.monotonic() + TIMEOUT
        seen = set()
        while time.monotonic() < deadline:
            resp = requests.get(
                f'{AGG_BASE_URL}/apps', headers=self.agg_auth, timeout=15)
            self.assertEqual(resp.status_code, 200, resp.text)
            seen = {item.get('id') for item in resp.json().get('items', [])}
            if PEER_APP in seen:
                return
            time.sleep(DISCOVERY_INTERVAL)
        self.fail(
            f'the aggregator never merged {PEER_APP}; it saw {sorted(seen)}. '
            f'Merging runs on aggregation.peer_auth_header, so this is a '
            f'problem with the fixture, and says nothing about forwarding.'
        )

    def test_01_the_peer_accepts_a_token_the_aggregator_minted(self):
        """C1. The peer holds no refresh record for this token and never will.

        It was issued by another process; the record that names it lives in
        that process's memory. All the peer can check is the signature, the
        expiry and the client, and all three are good because the two gateways
        share the configuration.
        """
        resp = requests.get(
            f'{PEER_BASE_URL}/areas', headers=self.agg_auth, timeout=15)
        self.assertEqual(
            resp.status_code, 200,
            f'the peer answered {resp.status_code} to a token minted by a gateway '
            f'sharing its secret, issuer and client. Body: {resp.text[:300]}'
        )

    def test_02_a_forwarded_read_is_carried_by_the_callers_token(self):
        """C2. The same acceptance, through the path forward_auth actually uses.

        ``/apps/<peer app>/data`` is owned by the peer, so the aggregator
        forwards it. The caller sent a credential, and the documented rule is
        that a caller's own token wins wherever both apply - so what reaches
        the peer is the token obtained above, not the aggregator's.
        """
        self._wait_for_merged_app()
        resp = requests.get(
            f'{AGG_BASE_URL}/apps/{PEER_APP}/data',
            headers=self.agg_auth,
            timeout=15,
        )
        self.assertEqual(
            resp.status_code, 200,
            f'a forwarded read answered {resp.status_code}; the peer refused the '
            f'token the aggregator issued and forwarded. Body: {resp.text[:300]}'
        )

    def test_03_the_peer_refuses_a_token_that_does_not_verify(self):
        """C3. The control: the peer is checking, not waving tokens through.

        One byte of the signature is changed, and only that. Reversing the
        whole signature would also pass, but a one-byte edit is the minimum
        difference that must be caught: it keeps the length, the alphabet and
        the padding intact, so nothing but the cryptography can reject it. The
        payload still names a client the peer knows and an expiry in the
        future.
        """
        good = _token_from(PEER_BASE_URL)
        header, payload, signature = good.split('.')

        alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_'
        original = signature[0]
        replacement = alphabet[(alphabet.index(original) + 1) % len(alphabet)]
        forged = '.'.join([header, payload, replacement + signature[1:]])
        self.assertEqual(len(forged), len(good), 'the forgery changed the token length')
        self.assertNotEqual(forged, good)

        resp = requests.get(
            f'{PEER_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {forged}'},
            timeout=15,
        )
        self.assertIn(
            resp.status_code, (401, 403),
            f'the peer answered {resp.status_code} to a token with one byte of its '
            f'signature changed, so the acceptances above say nothing about the '
            f'credential'
        )

    def test_04_the_peer_refuses_an_anonymous_read(self):
        """C4. The other half of the control: this peer is closed at all."""
        resp = requests.get(f'{PEER_BASE_URL}/areas', timeout=15)
        self.assertIn(resp.status_code, (401, 403), resp.text)

    def test_05_a_token_naming_a_client_the_peer_lacks_is_refused(self):
        """Sharing a secret is not sharing a client list.

        The signature verifies and the expiry is in the future, so the only
        thing that can refuse this is the peer checking `sub` against its own
        clients. Without that check a gateway would accept any `sub` an issuer
        cared to sign, and "the client list is the grant" would not hold.
        """
        stranger = _mint_peer_credential(subject='a_client_no_gateway_here_lists')
        resp = requests.get(
            f'{PEER_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {stranger}'},
            timeout=15,
        )
        self.assertIn(
            resp.status_code, (401, 403),
            f'the peer answered {resp.status_code} to a correctly signed token '
            f'naming a client it does not have'
        )

    def test_06_a_refresh_token_is_not_an_access_token(self):
        """The asymmetry the denylist model rests on.

        Access tokens are judged on signature, expiry and client, so they cross
        gateways and survive restarts. Refresh tokens are judged against a
        record the issuing process holds in memory, so they do neither - and
        presented as an access token they are refused on their type alone.
        """
        token = requests.post(
            f'{PEER_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        ).json()
        refresh = token.get('refresh_token')
        self.assertTrue(refresh, f'the peer issued no refresh token: {token}')

        resp = requests.get(
            f'{PEER_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {refresh}'},
            timeout=15,
        )
        self.assertIn(
            resp.status_code, (401, 403),
            f'a refresh token read /areas ({resp.status_code}); the two token '
            f'types must not be interchangeable'
        )

        # The aggregator never issued it either, and cannot refresh with it.
        refreshed = requests.post(
            f'{AGG_BASE_URL}/auth/token',
            json={'grant_type': 'refresh_token', 'refresh_token': refresh},
            timeout=30,
        )
        self.assertNotEqual(
            refreshed.status_code, 200,
            'a gateway that never issued this refresh token exchanged it for an '
            'access token; refresh records do not cross gateways'
        )

    def test_07_revoking_on_the_peer_refuses_the_token_there(self):
        """Revocation works on a gateway that never issued the token.

        Under a shared configuration the peer holds no record of anything the
        aggregator minted. Since the records are read as a denylist, a revoke
        that found nothing to mark would be a silent no-op on exactly the
        gateway an operator is trying to lock down.

        Revocation is per gateway: the aggregator is not told, and goes on
        accepting the token it issued.
        """
        token = requests.post(
            f'{AGG_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        ).json()
        access = token['access_token']
        refresh = token['refresh_token']
        auth = {'Authorization': f'Bearer {access}'}

        self.assertEqual(
            requests.get(f'{PEER_BASE_URL}/areas', headers=auth, timeout=15).status_code,
            200, 'the peer refused the token before it was revoked')

        revoked = requests.post(
            f'{PEER_BASE_URL}/auth/revoke',
            json={'token': refresh},
            headers=auth,
            timeout=30,
        )
        self.assertEqual(revoked.status_code, 200, revoked.text)

        self.assertIn(
            requests.get(f'{PEER_BASE_URL}/areas', headers=auth, timeout=15).status_code,
            (401, 403),
            'the peer went on accepting a token revoked on it')

        self.assertEqual(
            requests.get(f'{AGG_BASE_URL}/areas', headers=auth, timeout=15).status_code,
            200,
            'revoking on the peer reached the aggregator, which shares no record '
            'store with it')


class TestTheRoleComesFromTheAnsweringGateway(unittest.TestCase):
    """Sharing a signing configuration does not share the grants.

    The aggregator lists the shared client as `admin` and the peer lists it as
    `viewer`. The token is the aggregator's, and its `role` claim says admin -
    signed, so it cannot be edited in flight. What decides on the peer is the
    peer's own table.

    Without this the grant would travel with the token, and a deployment that
    trusted a peer's signing key would also be trusting whatever role every
    other gateway sharing that key chose to hand out.
    """

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(AGG_BASE_URL)
        _wait_until_answering(PEER_BASE_URL)
        cls.token = _token_from(AGG_BASE_URL)
        cls.auth = {'Authorization': f'Bearer {cls.token}'}

    def test_01_the_token_claims_admin(self):
        """The premise: the claim says admin, so the peer has to overrule it."""
        payload = self.token.split('.')[1]
        payload += '=' * (-len(payload) % 4)
        claims = json.loads(base64.urlsafe_b64decode(payload))
        self.assertEqual(
            claims.get('role'), 'admin',
            f'the aggregator did not mint an admin token: {claims}')

    def test_02_a_write_is_refused_on_the_peer(self):
        """A viewer on the peer may not execute an operation."""
        resp = requests.post(
            f'{PEER_BASE_URL}/apps/{PEER_APP}/operations/probe/executions',
            headers=self.auth,
            json={},
            timeout=15,
        )
        self.assertEqual(
            resp.status_code, 403,
            f'the peer answered {resp.status_code} to a write from a client its '
            f'own table lists as viewer. Body: {resp.text[:300]}'
        )

    def test_03_the_same_call_is_not_role_refused_on_the_issuer(self):
        """The control: admin there, so the refusal above is about the ROLE.

        The role is decided on the method and the path before any entity is
        looked up, so the request names an app no gateway has: on the
        aggregator that answers 404, which is what "the role passed" looks like
        on an app that is not there. The peer's own app is no use here - the
        aggregator has merged it and forwards a call on it to the peer, so the
        answer would be the peer's 403 relayed, and the control would measure
        the peer twice.
        """
        resp = requests.post(
            f'{AGG_BASE_URL}/apps/{ABSENT_APP}/operations/probe/executions',
            headers=self.auth,
            json={},
            timeout=15,
        )
        self.assertNotEqual(
            resp.status_code, 403,
            f'the aggregator refused its own admin client on role grounds: '
            f'{resp.text[:300]}'
        )

    def test_03b_the_identical_request_is_role_refused_on_the_peer(self):
        """Same path, same token, other table: the only variable is the grant."""
        resp = requests.post(
            f'{PEER_BASE_URL}/apps/{ABSENT_APP}/operations/probe/executions',
            headers=self.auth,
            json={},
            timeout=15,
        )
        self.assertEqual(
            resp.status_code, 403,
            f'the peer answered {resp.status_code} where its own table lists the '
            f'client as viewer. Body: {resp.text[:300]}'
        )

    def test_04_a_read_still_works_on_the_peer(self):
        """A viewer may read, so the peer refuses the method and not the token."""
        resp = requests.get(f'{PEER_BASE_URL}/areas', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_05_a_client_absent_from_the_peers_table_is_refused(self):
        """A shared key is not a shared client list."""
        stranger = _mint_peer_credential(subject='a_client_no_gateway_here_lists')
        resp = requests.get(
            f'{PEER_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {stranger}'},
            timeout=15,
        )
        self.assertIn(
            resp.status_code, (401, 403),
            f'the peer answered {resp.status_code} to a correctly signed token '
            f'naming a client it does not have')


@launch_testing.post_shutdown_test()
class TestForwardAuthAcrossGatewaysShutdown(unittest.TestCase):
    """Both gateways exit cleanly."""

    def test_exit_codes(self, proc_info, aggregator, peer):
        for proc in (aggregator, peer):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc
            )
