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
Every route refuses an uncredentialed request under the secure profile.

The gateway's own half of the closed-door acceptance, run against
`config/gateway_params.secure.yaml` - the profile a deployment uses to close a
gateway. It does not check configuration values: it asks the RUNNING gateway
for its route table and then probes every route in it. A test that asserted
`require_auth_for == "all"` would keep passing the day a route is registered
outside the policy, which is the failure this is here to catch.

The route table comes from RouteRegistry via `GET /api/v1/`, so a route added
next year is swept the day it is registered, with nothing here to update.

Two exemptions, and both are named with their reason in EXEMPT below.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
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
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

CLOSED_PORT = get_test_port()
CORS_PORT = get_test_port(1)
CLOSED_BASE_URL = f'http://127.0.0.1:{CLOSED_PORT}{API_BASE_PATH}'
CLOSED_ROOT = f'http://127.0.0.1:{CLOSED_PORT}'
CORS_BASE_URL = f'http://127.0.0.1:{CORS_PORT}{API_BASE_PATH}'
RL_PORT = get_test_port(2)
RL_BASE_URL = f'http://127.0.0.1:{RL_PORT}{API_BASE_PATH}'
PUBLIC_PORT = get_test_port(3)
PUBLIC_BASE_URL = f'http://127.0.0.1:{PUBLIC_PORT}{API_BASE_PATH}'
WRITE_PORT = get_test_port(4)
WRITE_BASE_URL = f'http://127.0.0.1:{WRITE_PORT}{API_BASE_PATH}'
ALLOWED_ORIGIN = 'https://ui.example'

SECURE_PARAMS = os.path.join(
    get_package_share_directory('ros2_medkit_gateway'),
    'config', 'gateway_params.secure.yaml'
)

# At least 32 characters, or the gateway refuses to start under HS256.
JWT_SECRET = 'closed_by_default_integration_secret_key_0123456789'
CLIENT_ID = 'diagbox'
CLIENT_SECRET = 'diagbox_client_secret'

# A path parameter is filled with an id that exists on no gateway. A route that
# refuses for a nonexistent id refuses for a real one, and a probe that turns
# out to reach an OPEN route cannot mutate anything real.
PROBE_ID = 'closed-by-default-probe'


@pytest.mark.launch_test
def generate_test_description():
    """Gateway loaded from the secure profile, the way a deployment closes one."""
    gateway_node = create_gateway_node(
        port=CLOSED_PORT,
        params_file=SECURE_PARAMS,
        extra_params={
            'server.host': '127.0.0.1',
            # TLS is a deployment artefact, not the posture under test; the
            # route sweep below is about who may reach a route, not about how
            # the bytes travel. test_tls_protocol_floor covers the transport.
            'server.tls.enabled': False,
            # Two settings of that profile are turned back off here, and each
            # would otherwise make the sweep prove less than it claims.
            #
            # Rate limiting: the profile allows 120 requests per client per
            # minute and this sweep sends two per route, so most of it would
            # answer 429 where 401 is the point - a refusal for the wrong reason.
            # rl_gateway below is the gateway that exists to test the
            # limiter's interaction with authentication.
            'rate_limiting.enabled': False,
            # Docs: the profile turns the /docs routes off to reduce the
            # surface. They are registered routes and this sweep is about
            # every registered route, so leaving them off would quietly
            # shrink what it covers.
            'docs.enabled': True,
            # The secret and the client cannot come from a committed file - a
            # committed secret is a secret every deployment shares - so they
            # are supplied here the way a deployment supplies them.
            'auth.jwt_secret': JWT_SECRET,
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
        },
    )

    cors_gateway = create_gateway_node(
        port=CORS_PORT,
        name='gateway_with_cors',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.require_auth_for': 'all',
            'auth.issuer': 'ros2_medkit_gateway',
            'auth.jwt_secret': JWT_SECRET,
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
            # CORS on, which is what puts a preflight through the pre-routing handler.
            'cors.allowed_origins': [ALLOWED_ORIGIN],
        },
    )

    # Rate limiting on, and tight, so the limiter can actually be exhausted
    # inside a test. The ordering between the limiter and authentication is
    # only observable against a gateway in this state.
    rl_gateway = create_gateway_node(
        port=RL_PORT,
        name='gateway_with_rate_limit',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.require_auth_for': 'all',
            'auth.issuer': 'ros2_medkit_gateway',
            'auth.jwt_secret': JWT_SECRET,
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
            # Small enough that a test can exhaust it, with room for the
            # credentialed half of the case below to obtain a token and make a
            # request before the allowance is gone.
            'rate_limiting.enabled': True,
            'rate_limiting.global_requests_per_minute': 5,
            'rate_limiting.client_requests_per_minute': 5,
        },
    )

    # The opt-in half. `auth.public_routes` is empty on every gateway above, so
    # without this one nothing here would exercise the knob an operator uses to
    # take a route outside authentication, and "empty by default" would be
    # indistinguishable from "the setting does nothing".
    public_route_gateway = create_gateway_node(
        port=PUBLIC_PORT,
        name='gateway_with_public_route',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.require_auth_for': 'all',
            'auth.issuer': 'ros2_medkit_gateway',
            'auth.jwt_secret': JWT_SECRET,
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
            'auth.public_routes': ['GET /api/v1/health'],
        },
    )

    # Authentication on, but at the OTHER requirement level. Every GET is open
    # here because reads are open, and no route was singled out - which is a
    # different statement from "an operator opened this one", and the two must
    # not be confused by anything that decides what to disclose.
    write_mode_gateway = create_gateway_node(
        port=WRITE_PORT,
        name='gateway_write_mode',
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.require_auth_for': 'write',
            'auth.issuer': 'ros2_medkit_gateway',
            'auth.jwt_secret': JWT_SECRET,
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
        },
    )

    return launch.LaunchDescription([
        gateway_node,
        cors_gateway,
        rl_gateway,
        public_route_gateway,
        write_mode_gateway,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node, 'cors_gateway': cors_gateway,
         'rl_gateway': rl_gateway, 'public_route_gateway': public_route_gateway,
         'write_mode_gateway': write_mode_gateway}


def _is_exempt(method, path):
    """Routes that are deliberately reachable without a credential.

    /auth/* alone, and only because authentication cannot bootstrap through a
    door that already demands the credential it exists to hand out.

    Health is NOT here. `auth.public_routes` is empty as shipped, so a probe
    that wants an uncredentialed answer is a decision an operator makes and
    writes down; TestConfiguredPublicRoute below covers that path.
    """
    del method  # the one exemption is path-shaped: every method under /auth/
    return path.startswith(f'{API_BASE_PATH}/auth/')


class TestClosedByDefault(GatewayTestCase):
    """The gateway refuses every route it serves, bar the named exemptions."""

    BASE_URL = CLOSED_BASE_URL

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        resp = requests.post(
            f'{CLOSED_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=10,
        )
        assert resp.status_code == 200, f'could not obtain a token: {resp.status_code} {resp.text}'
        cls.token = resp.json()['access_token']
        cls.auth = {'Authorization': f'Bearer {cls.token}'}

        # The route table, read from the gateway itself. A hardened gateway
        # does not list its routes anonymously, so this read authenticates.
        root = requests.get(f'{CLOSED_BASE_URL}/', headers=cls.auth, timeout=10)
        assert root.status_code == 200, f'route table unreadable: {root.status_code}'
        cls.endpoints = root.json().get('endpoints', [])
        assert cls.endpoints, 'gateway reported no endpoints - nothing would be proven'

    @staticmethod
    def _fill(path):
        out, depth = [], 0
        for ch in path:
            if ch == '{':
                depth += 1
                if depth == 1:
                    out.append(PROBE_ID)
            elif ch == '}':
                depth -= 1
            elif depth == 0:
                out.append(ch)
        return ''.join(out)

    def test_01_route_table_is_substantial(self):
        """A sweep over three routes would prove almost nothing."""
        self.assertGreater(
            len(self.endpoints), 50,
            f'expected the full gateway surface, got {len(self.endpoints)} routes'
        )

    def test_02_no_route_answers_without_a_credential(self):
        """Sweep EVERY registered route. This is the acceptance."""
        answered = []
        for entry in self.endpoints:
            method, _, raw = entry.partition(' ')
            if not raw:
                continue
            if _is_exempt(method, raw):
                continue
            path = self._fill(raw)
            # A write method needs a body: without Content-Length the server
            # waits for one that never arrives and the probe times out with no
            # status, measuring nothing at all.
            kwargs = {'timeout': 15}
            if method in ('POST', 'PUT', 'PATCH'):
                kwargs['json'] = {}
            try:
                resp = requests.request(method, f'{CLOSED_ROOT}{path}', **kwargs)
            except requests.RequestException as exc:
                answered.append(f'{method} {path} -> transport error {exc}')
                continue
            # 401/403 only. A 404 is an ANSWER: the gateway parsed the request
            # and told an anonymous caller what does not exist here.
            if resp.status_code not in (401, 403):
                answered.append(f'{method} {path} -> {resp.status_code}')

        self.assertEqual(
            [], answered,
            'these routes answered an uncredentialed request:\n  '
            + '\n  '.join(answered)
        )

    def test_03_a_wrong_credential_is_refused_everywhere(self):
        """A token this gateway never issued gets no further than none at all."""
        bad = {'Authorization': 'Bearer not.a.real.token'}
        answered = []
        for entry in self.endpoints:
            method, _, raw = entry.partition(' ')
            if not raw or _is_exempt(method, raw):
                continue
            path = self._fill(raw)
            kwargs = {'timeout': 15, 'headers': bad}
            if method in ('POST', 'PUT', 'PATCH'):
                kwargs['json'] = {}
            try:
                resp = requests.request(method, f'{CLOSED_ROOT}{path}', **kwargs)
            except requests.RequestException as exc:
                answered.append(f'{method} {path} -> transport error {exc}')
                continue
            if resp.status_code not in (401, 403):
                answered.append(f'{method} {path} -> {resp.status_code}')

        self.assertEqual(
            [], answered,
            'these routes accepted a forged credential:\n  ' + '\n  '.join(answered)
        )

    def test_04_reads_are_refused_not_just_writes(self):
        """The require_auth_for="write" hole, pinned directly.

        Under "write" every one of these answers 200 to an anonymous caller,
        and they are the disclosure: the entity tree names the machines.
        """
        for path in ('/', '/areas', '/components', '/apps', '/functions', '/version-info'):
            with self.subTest(path=path):
                resp = requests.get(f'{CLOSED_BASE_URL}{path}', timeout=15)
                self.assertIn(resp.status_code, (401, 403))

    def test_05_health_refuses_like_everything_else(self):
        """Health is not special. It is closed until somebody opens it.

        The route a hardening change is most tempted to leave open, pinned so
        the temptation shows up as a red test. `auth.public_routes` is the way
        to open it, and TestConfiguredPublicRoute holds that end.
        """
        resp = requests.get(f'{CLOSED_BASE_URL}/health', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            f'GET /health answered {resp.status_code} with no credential and an '
            'empty auth.public_routes'
        )

    def test_06_the_full_health_document_names_entities(self):
        """Why the anonymous body has to be cut down when a route is opened.

        With a credential the same route returns discovery state and entity
        cache counts. That is a legitimate operator surface, and it is exactly
        what an anonymous caller must not receive - so if this ever stops being
        true, the narrowing in TestConfiguredPublicRoute has become pointless
        and should be revisited before it becomes dead weight.
        """
        body = requests.get(
            f'{CLOSED_BASE_URL}/health', headers=self.auth, timeout=15
        ).json()
        self.assertIn('discovery', body)
        self.assertIn('x-medkit-entity-cache', body)
        self.assertNotIn(
            'x-medkit-reduced', body,
            'an authenticated caller was served the cut-down body'
        )

    def test_07_a_valid_credential_gets_through(self):
        """Otherwise the sweeps above would pass on a gateway that serves nobody."""
        resp = requests.get(f'{CLOSED_BASE_URL}/areas', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200)


class TestConfiguredPublicRoute(GatewayTestCase):
    """`auth.public_routes` opens exactly what it names, and nothing near it.

    The gateway under this class runs `require_auth_for: all` with one entry,
    `GET /api/v1/health`. Everything here is about the edge of that entry: a
    setting that opened the route it names AND its neighbours would pass a test
    that only checked the route it names.
    """

    BASE_URL = PUBLIC_BASE_URL

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        token = requests.post(
            f'{PUBLIC_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=15,
        ).json()['access_token']
        cls.auth = {'Authorization': f'Bearer {token}'}

    def test_01_the_named_route_answers_without_a_credential(self):
        """The knob does something. Without this the rest proves only refusal."""
        resp = requests.get(f'{PUBLIC_BASE_URL}/health', timeout=15)
        self.assertEqual(
            resp.status_code, 200,
            'auth.public_routes named GET /api/v1/health and it still refused'
        )

    def test_02_the_route_next_door_is_untouched(self):
        """An entry opens one route, not the surface around it.

        The failure this catches is a prefix or wildcard match creeping into
        the comparison: `/health` opening `/healthz`, or worse, one entry
        opening every GET.
        """
        for path in ('/', '/areas', '/components', '/apps', '/version-info'):
            with self.subTest(path=path):
                resp = requests.get(f'{PUBLIC_BASE_URL}{path}', timeout=15)
                self.assertIn(
                    resp.status_code, (401, 403),
                    f'{path} answered {resp.status_code} on a gateway whose only '
                    'public route is GET /api/v1/health'
                )

    def test_03_the_method_is_part_of_the_entry(self):
        """An entry names a method, and the method is part of the match.

        cpp-httplib dispatches HEAD into the GET handler table, so a comparison
        that dropped the method would hand the status document to an anonymous
        HEAD. The write methods have no handler here and answer the same either
        way, so HEAD is the one that can show the difference.
        """
        head = requests.head(f'{PUBLIC_BASE_URL}/health', timeout=15)
        self.assertIn(
            head.status_code, (401, 403),
            f'HEAD /health answered {head.status_code} for an entry that named GET'
        )

    def test_04_the_anonymous_body_is_liveness_and_says_so(self):
        """Opening the route must not publish the entity inventory.

        An allowlist, not a denylist: listing the fields known to leak today
        would pass the day a new section is added, and a probe needs no more
        than "am I alive".
        """
        body = requests.get(f'{PUBLIC_BASE_URL}/health', timeout=15).json()
        self.assertEqual(
            set(body),
            {'status', 'timestamp', 'warnings', 'warning_schema_version',
             'x-medkit-reduced'},
            f'an anonymous /health returned more than liveness: {body}'
        )
        self.assertEqual(body['status'], 'healthy')
        # The array is the leak vector: a linking warning reads like
        # "App 'engine_ecu' cannot bind to '/nav/controller'", naming an entity
        # and a ROS node FQN.
        self.assertEqual(body['warnings'], [])
        # And the empty array must not read as "nothing is wrong". A monitor
        # that cannot tell withheld from clean would clear a real warning.
        self.assertIs(
            body['x-medkit-reduced'], True,
            'the cut-down body did not say it was cut down, so an empty '
            'warnings array reads as a clean bill of health'
        )

    def test_05_a_credential_still_gets_the_whole_document(self):
        """Opening a route for probes must not cost the operator surface."""
        body = requests.get(
            f'{PUBLIC_BASE_URL}/health', headers=self.auth, timeout=15
        ).json()
        self.assertIn('discovery', body)
        self.assertNotIn('x-medkit-reduced', body)

    def test_06_a_forged_credential_is_an_anonymous_caller(self):
        """A token this gateway never issued must not unlock the full body."""
        body = requests.get(
            f'{PUBLIC_BASE_URL}/health',
            headers={'Authorization': 'Bearer not.a.real.token'},
            timeout=15,
        ).json()
        self.assertIs(body.get('x-medkit-reduced'), True, body)
        self.assertNotIn('discovery', body)


class TestWriteModeHealthIsNotCutDown(GatewayTestCase):
    """The reduction follows `auth.public_routes`, not `auth.enabled`.

    Under `require_auth_for: "write"` every GET is answered without a
    credential because reads are open at that level, and no operator singled
    out a route. Cutting the body there withheld the discovery, entity-cache
    and linking sections from a configuration nobody hardened, while
    docs/config/server.rst promises the cut only on a route named in
    `auth.public_routes`.

    Two conditions have to be separable for this to mean anything, so this
    gateway holds one of them (authentication on, anonymous caller) and NOT the
    other (no public_routes entry).
    """

    BASE_URL = WRITE_BASE_URL

    def test_01_authentication_really_is_on(self):
        """Without this the class passes against a gateway with auth off.

        A write refused for want of a credential is what proves the gateway is
        authenticating at all, which is the premise every assertion below
        rests on.
        """
        resp = requests.post(
            f'{WRITE_BASE_URL}/apps/{PROBE_ID}/operations/{PROBE_ID}/executions',
            json={},
            timeout=15,
        )
        self.assertIn(
            resp.status_code, (401, 403),
            f'a write answered {resp.status_code} on a gateway running '
            f'auth.enabled true, so authentication is not on and nothing below '
            f'is about the reduction'
        )

    def test_02_an_anonymous_health_carries_the_sections(self):
        body = requests.get(f'{WRITE_BASE_URL}/health', timeout=15).json()
        self.assertNotIn(
            'x-medkit-reduced', body,
            f'/health was cut down under require_auth_for "write", where no '
            f'route was named in auth.public_routes: {body}'
        )
        self.assertIn(
            'discovery', body,
            f'the discovery section was withheld from an anonymous caller on a '
            f'gateway with no public_routes entry: {body}'
        )
        self.assertIn('x-medkit-entity-cache', body, body)

    def test_03_a_reduced_body_is_still_reachable_where_it_is_configured(self):
        """The control, on the gateway that DOES name a route.

        Without it, test_02 would pass just as well against a gateway that had
        stopped reducing anything at all, and the whole `public_routes`
        behaviour would be untested by this pair.
        """
        body = requests.get(f'{PUBLIC_BASE_URL}/health', timeout=15).json()
        self.assertIs(body.get('x-medkit-reduced'), True, body)


class TestNothingAnswersBeforeAuth(GatewayTestCase):
    """What the CORS preflight may and may not do without a credential.

    Preflight is answered anonymously on purpose, and it is the second named
    exemption after /auth/*. A browser never puts Authorization on a preflight
    - asking permission before sending the real request is the whole point of
    the mechanism - so demanding one would not harden anything, it would make
    browser clients impossible. The control below is what pins that.

    What must hold instead: the preflight discloses only CORS policy, and the
    REAL request that follows is still refused without a credential.

    This gateway enables CORS for a real origin, which the rest of the file
    deliberately does not, because that is the configuration in which any of
    this is reachable at all.
    """

    BASE_URL = CORS_BASE_URL

    def _preflight(self, extra=None):
        headers = {'Origin': ALLOWED_ORIGIN, 'Access-Control-Request-Method': 'GET'}
        headers.update(extra or {})
        return requests.options(f'{CORS_BASE_URL}/apps', headers=headers, timeout=15)

    def test_01_an_anonymous_preflight_is_answered(self):
        """The exemption, stated as a test so it stays explicit.

        A browser cannot put a credential on a preflight - asking permission
        before sending the real request is the whole point of the mechanism -
        so a 401 or 403 here means no browser client can reach this gateway at
        all.
        """
        resp = self._preflight()
        self.assertEqual(
            resp.status_code, 204,
            f'an anonymous preflight got {resp.status_code}; a browser cannot '
            'authenticate a preflight, so refusing it makes browser clients '
            'impossible'
        )
        self.assertEqual(resp.headers.get('Access-Control-Allow-Origin'), ALLOWED_ORIGIN)

    def test_02_the_preflight_discloses_only_cors_policy(self):
        """Why the exemption is safe: there is nothing in the response.

        If a preflight ever grew a body, the exemption would start leaking and
        this fails, so it cannot pass unnoticed.
        """
        resp = self._preflight()
        self.assertEqual(
            resp.content, b'',
            f'the preflight returned a body: {resp.content[:200]!r}'
        )

    def test_03_a_preflight_from_an_unknown_origin_is_refused(self):
        """The exemption is scoped to origins the operator configured."""
        resp = requests.options(
            f'{CORS_BASE_URL}/apps',
            headers={'Origin': 'https://not-configured.example',
                     'Access-Control-Request-Method': 'GET'},
            timeout=15,
        )
        self.assertEqual(resp.status_code, 403)

    def test_04_the_real_request_after_a_preflight_still_needs_a_credential(self):
        """The property that actually matters.

        A preflight being answered must not carry any implication for the GET
        that follows it, which is where the data is.
        """
        resp = requests.get(
            f'{CORS_BASE_URL}/apps', headers={'Origin': ALLOWED_ORIGIN}, timeout=15
        )
        self.assertIn(
            resp.status_code, (401, 403),
            f'a cross-origin GET got {resp.status_code} with no credential'
        )

    def test_04b_a_plain_options_without_the_preflight_header_is_refused(self):
        """The exemption is for preflights, not for the OPTIONS method.

        A browser preflight always carries Access-Control-Request-Method. An
        OPTIONS without it is an ordinary request that any client could send,
        and it has no reason to skip the credential check. The helper above
        always sends both headers, so this boundary needs its own case.
        """
        resp = requests.options(
            f'{CORS_BASE_URL}/apps',
            headers={'Origin': ALLOWED_ORIGIN},
            timeout=15,
        )
        self.assertIn(
            resp.status_code, (401, 403),
            f'a plain OPTIONS with no Access-Control-Request-Method got '
            f'{resp.status_code}; the preflight exemption is too wide'
        )

    def test_05_an_authenticated_cross_origin_request_works(self):
        """The mirror: CORS is live and a credentialed browser call succeeds."""
        headers = {'Origin': ALLOWED_ORIGIN}
        headers.update(self.cors_auth)
        resp = requests.get(f'{CORS_BASE_URL}/apps', headers=headers, timeout=15)
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(resp.headers.get('Access-Control-Allow-Origin'), ALLOWED_ORIGIN)

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        resp = requests.post(
            f'{CORS_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        )
        assert resp.status_code == 200, f'token request failed: {resp.status_code}'
        cls.cors_auth = {'Authorization': f'Bearer {resp.json()["access_token"]}'}


class TestLimiterSpeaksOnlyToCallersItMay(GatewayTestCase):
    """Who hears the limiter, and what it tells them.

    Three answers on one gateway, and the difference between them is the
    contract:

    - no Authorization header on a protected route: 401, whatever the
      allowance. `process` short-circuits on the missing header before it
      extracts anything, so the refusal is free and 429 would disclose limiter
      state to a caller holding nothing at all.
    - an Authorization header with the allowance gone: a bare 429 - no
      `X-RateLimit-*`, no `Retry-After` - and the token inside is never
      verified. Nothing about this caller has been checked, so the allowance
      and the reset time stay behind the check they would otherwise precede.
    - a credential the gateway accepted: the full limiter state, which is what
      a client needs to pace itself.

    The limit here is deliberately tiny so the exhausted state is reachable in
    a test at all.
    """

    BASE_URL = RL_BASE_URL

    # The anonymous refusal from test_01, taken while the allowance is still
    # there. test_03 compares the exhausted refusal against it, and by the time
    # test_03 runs the bucket is long gone - a refusal it fetched for itself
    # would be a second exhausted one, and comparing two exhausted responses
    # says nothing about whether they drifted apart.
    unexhausted_refusal = None

    # A token this gateway accepts, taken in test_01 while the allowance was
    # there. test_07 needs one and cannot mint it: by then /auth/authorize is
    # over the limit like everything else.
    accepted_bearer = None

    def test_01_the_limiter_state_reaches_only_a_credentialed_caller(self):
        """X-RateLimit-* says how much allowance is left and when it resets.

        Reporting it to a caller holding no credential is the disclosure that
        answering 429 before authentication would have made, arriving by
        another door. A caller who authenticates is entitled to it, and that
        half is checked first - both because the allowance is still there, and
        because without it this would pass against a gateway that had simply
        stopped emitting the headers.
        """
        token = requests.post(
            f'{RL_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        )
        self.assertEqual(token.status_code, 200, token.text)
        authed = requests.get(
            f'{RL_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {token.json()["access_token"]}'},
            timeout=15,
        )
        self.assertEqual(authed.status_code, 200, authed.text)
        self.assertNotEqual(
            [h for h in authed.headers if h.lower().startswith('x-ratelimit')], [],
            f'a credentialed caller was told nothing about the limiter: '
            f'{dict(authed.headers)}'
        )

        anonymous = requests.get(f'{RL_BASE_URL}/apps', timeout=15)
        self.assertIn(anonymous.status_code, (401, 403), anonymous.text)
        leaked = [h for h in anonymous.headers if h.lower().startswith('x-ratelimit')]
        self.assertEqual(
            leaked, [],
            f'an uncredentialed caller was told the limiter state: {leaked}'
        )

        # The allowance the credentialed request above reported is what makes
        # this refusal an unexhausted one. test_03 compares against it, and a
        # stash taken past the limit would make that comparison say nothing.
        remaining = int(authed.headers.get('X-RateLimit-Remaining', '0'))
        self.assertGreater(
            remaining, 0,
            f'the bucket was already spent when test_01 ran, so the refusal it '
            f'stashes is not an unexhausted one (remaining={remaining})'
        )
        type(self).unexhausted_refusal = anonymous
        type(self).accepted_bearer = token.json()['access_token']

    def test_02_an_exhausted_anonymous_caller_still_gets_401(self):
        responses = []
        # Comfortably past a limit of 5/minute.
        for _ in range(12):
            responses.append(requests.get(f'{RL_BASE_URL}/apps', timeout=15))
        seen = [r.status_code for r in responses]

        self.assertNotIn(
            429, seen,
            f'an anonymous caller was rate-limited where a refusal is due: {seen}'
        )
        self.assertTrue(
            all(code in (401, 403) for code in seen),
            f'expected only 401/403 for an uncredentialed caller, got {seen}'
        )

    def test_03_the_exhausted_refusal_is_shaped_like_every_other_401(self):
        """Status alone is not the contract; the body and the challenge are.

        The refusal an exhausted anonymous caller gets is produced on a
        different line from the one an unexhausted caller gets, so nothing
        stops the two drifting apart. A client that reads the error document to
        tell a missing credential from an expired one gets nothing to read from
        a bare 401 - and a 401 that is visibly a different shape is itself the
        disclosure that the limiter, not the credential, decided it.

        The unexhausted half comes from test_01, which took it while the
        allowance was still there. Fetching one here would get a second
        exhausted refusal: 5 per minute refills one every 12 seconds and the
        tests above spent 15.
        """
        fresh = type(self).unexhausted_refusal
        self.assertIsNotNone(
            fresh,
            'test_01 did not stash an unexhausted refusal, so this compares '
            'nothing'
        )
        exhausted = None
        for _ in range(8):
            exhausted = requests.get(f'{RL_BASE_URL}/apps', timeout=15)

        self.assertEqual(
            fresh.json().keys(), exhausted.json().keys(),
            f'the two refusals are different documents: {fresh.json()} vs '
            f'{exhausted.json()}'
        )
        for label, resp in (('first', fresh), ('exhausted', exhausted)):
            with self.subTest(response=label):
                self.assertIn(resp.status_code, (401, 403), resp.text)
                self.assertIn(
                    'WWW-Authenticate', resp.headers,
                    f'the {label} refusal carries no challenge header: '
                    f'{dict(resp.headers)}'
                )
                body = resp.json()
                # The shape AuthMiddleware produces: an OAuth-style error and
                # a human-readable description, not the SOVD `error_code`
                # envelope the handlers use for their own refusals.
                self.assertTrue(
                    body.get('error'),
                    f'the {label} refusal carries no error: {body}'
                )
                self.assertTrue(
                    body.get('error_description'),
                    f'the {label} refusal carries no error_description: {body}'
                )

    def test_04_an_exhausted_caller_with_a_header_gets_a_bare_429(self):
        """A credential the gateway never has to verify, and a 429 that says nothing.

        Verifying a signature is the expensive half of the auth path - under
        RS256 it reads a key file - and an over-limit caller is one the gateway
        has already decided to refuse. So a request carrying an Authorization
        header on a protected route while the allowance is gone answers 429 and
        the token in it is never looked at.

        The header here is deliberately not a token. A gateway that verified
        first would answer 401 with a decode error; 429 is only reachable by
        the limiter having spoken first.

        And the 429 carries no limiter state. Nothing about this caller has
        been checked - presenting a header is not presenting a credential -
        so `X-RateLimit-*` and `Retry-After` would tell an unverified caller
        the allowance, the reset time and the retry delay, which is the
        disclosure answering before authentication was meant to avoid.
        """
        # The tests above spent well past the 5/minute allowance; one more
        # burst removes any doubt about which side of the limit this sits on.
        for _ in range(8):
            requests.get(f'{RL_BASE_URL}/apps', timeout=15)

        refused = requests.get(
            f'{RL_BASE_URL}/apps',
            headers={'Authorization': 'Bearer this-is-not-a-token'},
            timeout=15,
        )
        self.assertEqual(
            refused.status_code, 429,
            f'an exhausted caller presenting a header got {refused.status_code} '
            f'where 429 is due, so the gateway verified a token it had already '
            f'decided to refuse. Body: {refused.text[:300]}'
        )

        leaked = [h for h in refused.headers
                  if h.lower().startswith('x-ratelimit') or h.lower() == 'retry-after']
        self.assertEqual(
            leaked, [],
            f'the pre-validation 429 carried limiter state to a caller nothing '
            f'has verified: {leaked}'
        )
        self.assertEqual(
            refused.json().get('parameters'), {},
            f'the bare 429 body carried limiter state: {refused.text[:300]}'
        )

    def test_06_a_non_preflight_options_is_metered_like_any_other_request(self):
        """OPTIONS is not a free method; a PREFLIGHT is an exempt request.

        A real preflight carries Access-Control-Request-Method and is answered
        before the limiter is reached. An OPTIONS without it is an ordinary
        request to a route: it runs the token verifier, so it has to spend
        allowance, or a flood of them costs a signature check each for free.
        """
        for _ in range(10):
            requests.get(f'{RL_BASE_URL}/apps', timeout=15)

        resp = requests.options(
            f'{RL_BASE_URL}/apps',
            headers={'Authorization': 'Bearer this-is-not-a-token'},
            timeout=15,
        )
        self.assertEqual(
            resp.status_code, 429,
            f'an exhausted OPTIONS carrying a header answered {resp.status_code}; '
            f'the limiter did not meter it. Body: {resp.text[:300]}'
        )
        leaked = [h for h in resp.headers
                  if h.lower().startswith('x-ratelimit') or h.lower() == 'retry-after']
        self.assertEqual(leaked, [], f'the bare 429 carried limiter state: {leaked}')

    def test_07_a_valid_credential_over_the_limit_gets_the_same_bare_429(self):
        """A token the gateway WOULD accept, over the limit.

        The refusal is the limiter's, so it lands the same way whether the
        credential is good or garbage - and a client holding a valid token
        therefore gets no Retry-After on a protected route. That is the
        consequence a closed-profile client has to be built around, and this
        is where it is pinned.
        """
        # Minted by test_01, while the allowance was still there. Obtaining one
        # here would spend allowance on a call that is not the subject, and by
        # this point in the class /auth/authorize is over the limit itself.
        bearer = type(self).accepted_bearer
        self.assertIsNotNone(
            bearer,
            'test_01 did not stash a token, so this cannot test the '
            'credentialed path'
        )

        for _ in range(10):
            requests.get(f'{RL_BASE_URL}/apps', timeout=15)

        resp = requests.get(
            f'{RL_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {bearer}'},
            timeout=15,
        )
        self.assertEqual(
            resp.status_code, 429,
            f'an over-limit caller holding a VALID token answered {resp.status_code}'
        )
        leaked = [h for h in resp.headers
                  if h.lower().startswith('x-ratelimit') or h.lower() == 'retry-after']
        self.assertEqual(
            leaked, [],
            f'a valid credential over the limit was told the limiter state: {leaked}'
        )

    def test_05_the_bare_429_needs_only_a_header_not_a_bearer(self):
        """The test is the header's PRESENCE; its value is never parsed here.

        A gateway that reached this path by looking for "Bearer " would answer
        401 to the scheme below, and the comment in rest_server would be
        describing something the code does not do.
        """
        for _ in range(8):
            requests.get(f'{RL_BASE_URL}/apps', timeout=15)

        for header in ('Basic dXNlcjpwYXNz', 'not-even-a-scheme'):
            with self.subTest(authorization=header):
                resp = requests.get(
                    f'{RL_BASE_URL}/apps',
                    headers={'Authorization': header},
                    timeout=15,
                )
                self.assertEqual(
                    resp.status_code, 429,
                    f'an exhausted caller sending {header!r} got '
                    f'{resp.status_code}; the limiter path keys on the header '
                    f'being there, not on its scheme'
                )


@launch_testing.post_shutdown_test()
class TestClosedByDefaultShutdown(unittest.TestCase):
    """Gateway exits cleanly."""

    def test_exit_codes(self, proc_info, gateway_node, cors_gateway, rl_gateway,
                        public_route_gateway, write_mode_gateway):
        for proc in (gateway_node, cors_gateway, rl_gateway, public_route_gateway,
                     write_mode_gateway):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc
            )
