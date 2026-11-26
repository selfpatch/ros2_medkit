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

"""An aggregator does not take a reduced `/health` from a peer as healthy.

Three gateway peers list `GET /api/v1/health` in `auth.public_routes`: one
under `require_auth_for: all`, one under `write`, and one under `all` that also
lists `GET /api/v1/areas`. A caller without a token the peer accepts gets the
reduced body from all three. Two more peers are stand-ins in this process with a
reduced `/health`: the discovery reads of one answer 503, 429, nothing, and 200;
those of the other answer 200 and then 403. Three aggregators point at the five
peers: one sends no
`aggregation.peer_auth_header`, one sends a token the peers refuse, one sends a
token they accept.

- Reduced `/health` and a discovery read answering 401 or 403: the peer is
  offline and the aggregator logs that its credential was not accepted.
- Reduced `/health` and every discovery read answering 200: the peer stays
  online, logged as reads that are open.
- Reduced `/health` and discovery reads without a usable answer: neither is
  logged, and the peer stays online.
- An accepted credential: every gateway peer is online and none is logged.
- A refused peer stays offline through a failed `/health` and through reads
  without a usable answer after it; a full `/health` brings it back online. A
  fourth aggregator probes a stand-in whose answers follow that sequence.

@verifies REQ_INTEROP_086
"""

import base64
import hashlib
import hmac
import http.server
import json
import re
import threading
import time
import unittest

import launch
import launch_testing
import launch_testing.actions
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
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

ALL_PEER_PORT = get_test_port(0)
WRITE_PEER_PORT = get_test_port(1)
MISSING_AGG_PORT = get_test_port(2)
REJECTED_AGG_PORT = get_test_port(3)
ACCEPTED_AGG_PORT = get_test_port(4)
MIXED_PEER_PORT = get_test_port(5)
STAND_IN_PORT = get_test_port(6)
FORBIDDING_PORT = get_test_port(7)
SEQUENCE_PORT = get_test_port(8)
SEQUENCE_AGG_PORT = get_test_port(9)


def _base(port):
    return f'http://127.0.0.1:{port}{API_BASE_PATH}'


# The aggregators learn the peers over HTTP only.
AGG_DOMAIN_ID = get_test_domain_id(0)
PEER_DOMAIN_ID = get_test_domain_id(1)

ALL_PEER = 'all_peer'
WRITE_PEER = 'write_peer'
MIXED_PEER = 'mixed_peer'
STAND_IN_PEER = 'stand_in_peer'
FORBIDDING_PEER = 'forbidding_peer'
SEQUENCE_PEER = 'sequence_peer'
GATEWAY_PEERS = (ALL_PEER, WRITE_PEER, MIXED_PEER)
HEALTH_ROUTE = f'GET {API_BASE_PATH}/health'
AREAS_ROUTE = f'GET {API_BASE_PATH}/areas'
JWT_SECRET = 'peer_reduced_health_secret_key_0123456789abcdef'
JWT_ISSUER = 'ros2_medkit_gateway'
CLIENT_ID = 'aggregator'
REFUSED_NOTICE = "did not accept this gateway's credential"
# test_00 waits for the peers; the aggregators re-check them every second after that.
NOTICE_TIMEOUT = GATEWAY_STARTUP_TIMEOUT


def _b64(raw):
    return base64.urlsafe_b64encode(raw).rstrip(b'=')


def _bearer(secret):
    """Return an HS256 access token for CLIENT_ID, signed with `secret`."""
    header = {'alg': 'HS256', 'typ': 'access'}
    now = int(time.time())
    payload = {'iss': JWT_ISSUER, 'sub': CLIENT_ID, 'iat': now, 'exp': now + 3600,
               'jti': 'peer-reduced-health', 'role': 'admin'}
    signing_input = b'.'.join([
        _b64(json.dumps(header, separators=(',', ':')).encode()),
        _b64(json.dumps(payload, separators=(',', ':')).encode()),
    ])
    signature = hmac.new(secret.encode(), signing_input, hashlib.sha256).digest()
    return 'Bearer ' + (signing_input + b'.' + _b64(signature)).decode()


REJECTED_CREDENTIAL = _bearer('a_secret_the_peers_do_not_hold_0123456789')
ACCEPTED_CREDENTIAL = _bearer(JWT_SECRET)


class _StandInPeer(http.server.BaseHTTPRequestHandler):
    """A peer with a reduced `/health` whose discovery reads get no usable answer."""

    # Path -> status. `None` closes the connection without an answer.
    ANSWERS = {f'{API_BASE_PATH}/areas': 503, f'{API_BASE_PATH}/components': 429,
               f'{API_BASE_PATH}/apps': None}
    # (path, Authorization) of every request, in order.
    seen = []
    seen_changed = threading.Condition()

    def do_GET(self):  # noqa: N802 - the name BaseHTTPRequestHandler dispatches to
        with self.seen_changed:
            self.seen.append((self.path, self.headers.get('Authorization', '')))
            self.seen_changed.notify_all()
        if self.path == f'{API_BASE_PATH}/health':
            self._send(200, {'status': 'healthy', 'timestamp': 0, 'x-medkit-reduced': True})
        elif self.path in self.ANSWERS and self.ANSWERS[self.path] is None:
            self.close_connection = True
        elif self.path in self.ANSWERS:
            self._send(self.ANSWERS[self.path], {'error_code': 'stand-in', 'message': 'stand-in'})
        else:
            self._send(200, {'items': []})

    def _send(self, status, body):
        data = json.dumps(body).encode()
        self.send_response(status)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


class _ForbiddingPeer(_StandInPeer):
    """A peer with a reduced `/health` whose second discovery read answers 403."""

    ANSWERS = {f'{API_BASE_PATH}/components': 403}
    seen = []
    seen_changed = threading.Condition()


class _SequencePeer(_StandInPeer):
    """A peer whose answers follow the health checks of the one aggregator that probes it.

    Check 1: reduced `/health`, `/areas` 401. Check 2: `/health` 503. Then reduced `/health`,
    `/areas` 429 and the other reads 200, until `full` is set; from then on a full `/health`.
    """

    full = threading.Event()
    # Phase of every /health served, in order.
    phases = []
    changed = threading.Condition()

    def do_GET(self):  # noqa: N802 - the name BaseHTTPRequestHandler dispatches to
        if self.path == f'{API_BASE_PATH}/health':
            with self.changed:
                served = len(self.phases)
                phase = ('refused' if served == 0 else 'failed' if served == 1
                         else 'full' if self.full.is_set() else 'inconclusive')
                self.phases.append(phase)
                self.changed.notify_all()
            if phase == 'failed':
                self._send(503, {'error_code': 'stand-in', 'message': 'stand-in'})
            else:
                self._send(200, {'status': 'healthy', 'timestamp': 0,
                                 'x-medkit-reduced': phase != 'full'})
            return
        with self.changed:
            phase = self.phases[-1] if self.phases else 'refused'
        status = {'refused': 401, 'inconclusive': 429}.get(phase, 200)
        if self.path == f'{API_BASE_PATH}/areas' and status != 200:
            self._send(status, {'error_code': 'stand-in', 'message': 'stand-in'})
        else:
            self._send(200, {'items': []})


def _peer(name, port, require_auth_for, public_routes=(HEALTH_ROUTE,)):
    return create_gateway_node(
        port=port, name=name,
        extra_params={
            'server.host': '127.0.0.1',
            'auth.enabled': True,
            'auth.require_auth_for': require_auth_for,
            'auth.issuer': JWT_ISSUER,
            'auth.jwt_secret': JWT_SECRET,
            'auth.clients': [f'{CLIENT_ID}:aggregator_client_secret:admin'],
            'auth.public_routes': list(public_routes),
        },
        extra_env={'ROS_DOMAIN_ID': str(PEER_DOMAIN_ID)})


def _aggregator(name, port, credential, peers=None):
    if peers is None:
        peers = {ALL_PEER: ALL_PEER_PORT, WRITE_PEER: WRITE_PEER_PORT,
                 MIXED_PEER: MIXED_PEER_PORT, STAND_IN_PEER: STAND_IN_PORT,
                 FORBIDDING_PEER: FORBIDDING_PORT}
    params = {
        'server.host': '127.0.0.1',
        'refresh_interval_ms': 1000,
        'aggregation.enabled': True,
        'aggregation.announce': False,
        'aggregation.discover': False,
        'aggregation.peer_urls': [f'http://127.0.0.1:{port}' for port in peers.values()],
        'aggregation.peer_names': list(peers),
    }
    if credential is not None:
        params['aggregation.peer_auth_header'] = credential
    return create_gateway_node(port=port, name=name, extra_params=params,
                               extra_env={'ROS_DOMAIN_ID': str(AGG_DOMAIN_ID)})


def generate_test_description():
    for port, handler in ((STAND_IN_PORT, _StandInPeer), (FORBIDDING_PORT, _ForbiddingPeer),
                          (SEQUENCE_PORT, _SequencePeer)):
        server = http.server.ThreadingHTTPServer(('127.0.0.1', port), handler)
        threading.Thread(target=server.serve_forever, daemon=True).start()
    missing = _aggregator('agg_missing', MISSING_AGG_PORT, None)
    rejected = _aggregator('agg_rejected', REJECTED_AGG_PORT, REJECTED_CREDENTIAL)
    accepted = _aggregator('agg_accepted', ACCEPTED_AGG_PORT, ACCEPTED_CREDENTIAL)
    sequence = _aggregator('agg_sequence', SEQUENCE_AGG_PORT, REJECTED_CREDENTIAL,
                           {SEQUENCE_PEER: SEQUENCE_PORT})
    return launch.LaunchDescription([
        _peer('peer_all', ALL_PEER_PORT, 'all'),
        _peer('peer_write', WRITE_PEER_PORT, 'write'),
        _peer('peer_mixed', MIXED_PEER_PORT, 'all', (HEALTH_ROUTE, AREAS_ROUTE)),
        missing, rejected, accepted, sequence,
        launch_testing.actions.ReadyToTest(),
    ]), {'missing': missing, 'rejected': rejected, 'accepted': accepted, 'sequence': sequence}


def _poll(predicate, what):
    """Return the first truthy value of `predicate` within DISCOVERY_TIMEOUT."""
    deadline = time.monotonic() + DISCOVERY_TIMEOUT
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
    raise AssertionError(f'{what} within {DISCOVERY_TIMEOUT}s; last seen: {last!r}')


def _notices(proc_output, process, peer=None):
    """Return the refused-credential lines `process` logged, for `peer` or for any peer."""
    marker = f"Peer '{peer}' {REFUSED_NOTICE}" if peer else REFUSED_NOTICE
    text = ''.join(out.text.decode(errors='replace') for out in proc_output[process])
    return [line[line.find('Peer '):] for line in text.splitlines() if marker in line]


def _peer_status(agg_port):
    """Return {peer name: status} from the aggregator's /health.

    The first peer checks, and their log lines, come before the REST server binds.
    """
    body = _poll(lambda: requests.get(f'{_base(agg_port)}/health', timeout=5).json(),
                 f'the aggregator on {agg_port} did not answer /health')
    return {peer['name']: peer['status'] for peer in body.get('peers', [])}


class TestPeerReducedHealth(unittest.TestCase):

    def test_00_the_peers_serve_the_reduced_body_to_a_stranger(self):
        """Premise: the peers are up, list /health, and differ on reads."""
        for port in (ALL_PEER_PORT, WRITE_PEER_PORT, MIXED_PEER_PORT):
            body = _poll(lambda port=port: requests.get(f'{_base(port)}/health', timeout=5).json(),
                         f'peer on {port} did not answer /health')
            self.assertIs(body.get('x-medkit-reduced'), True, body)
        reads = {(ALL_PEER_PORT, 'areas'): 401, (WRITE_PEER_PORT, 'areas'): 200,
                 (MIXED_PEER_PORT, 'areas'): 200, (MIXED_PEER_PORT, 'components'): 401}
        for (port, collection), status in reads.items():
            with self.subTest(port=port, collection=collection):
                resp = requests.get(f'{_base(port)}/{collection}', timeout=5)
                self.assertEqual(resp.status_code, status)

    def _assert_refused(self, proc_output, process, agg_port):
        # Logged after the status is stored, so the line marks a finished check.
        for peer, outcome in ((ALL_PEER, 'Marked offline'), (MIXED_PEER, 'Marked offline'),
                              (FORBIDDING_PEER, 'Marked offline'), (WRITE_PEER, 'stays online')):
            line = re.compile(f"Peer '{peer}' {re.escape(REFUSED_NOTICE)}.*{outcome}")
            proc_output.assertWaitFor(line, process=process, timeout=NOTICE_TIMEOUT)
        self.assertEqual(_peer_status(agg_port), {
            ALL_PEER: 'offline', WRITE_PEER: 'online', MIXED_PEER: 'offline',
            STAND_IN_PEER: 'online', FORBIDDING_PEER: 'offline'})

    def test_01_no_credential_takes_the_all_peer_offline(self, proc_output, missing):
        self._assert_refused(proc_output, missing, MISSING_AGG_PORT)

    def test_02_a_refused_credential_takes_the_all_peer_offline(self, proc_output, rejected):
        self._assert_refused(proc_output, rejected, REJECTED_AGG_PORT)

    def test_03_an_accepted_credential_keeps_the_gateway_peers_online(
            self, proc_output, accepted):
        # The stand-ins answer every credential alike, so only the gateway peers tell.
        _poll(lambda: all(_peer_status(ACCEPTED_AGG_PORT).get(peer) == 'online'
                          for peer in GATEWAY_PEERS),
              'the aggregator with an accepted credential never saw every gateway peer online')
        for peer in GATEWAY_PEERS:
            self.assertEqual(_notices(proc_output, accepted, peer), [])

    def test_04_reads_without_a_usable_answer_change_nothing(self, proc_output, missing, rejected,
                                                             accepted):
        """503, 429 and no answer are neither a refusal nor open reads."""
        aggregators = {'': (missing, MISSING_AGG_PORT),
                       REJECTED_CREDENTIAL: (rejected, REJECTED_AGG_PORT),
                       ACCEPTED_CREDENTIAL: (accepted, ACCEPTED_AGG_PORT)}

        def probed_and_checked_again():
            # A /health after a probe of /areas from the same aggregator starts a new check, so
            # the check that probed has finished and logged.
            for credential in aggregators:
                paths = [path for path, auth in _StandInPeer.seen if auth == credential]
                areas = f'{API_BASE_PATH}/areas'
                if areas not in paths or f'{API_BASE_PATH}/health' not in paths[
                        paths.index(areas):]:
                    return False
            return True

        with _StandInPeer.seen_changed:
            self.assertTrue(_StandInPeer.seen_changed.wait_for(probed_and_checked_again,
                                                               timeout=NOTICE_TIMEOUT),
                            f'not every aggregator probed the stand-in: {_StandInPeer.seen[-12:]}')
        for credential, (process, port) in aggregators.items():
            with self.subTest(aggregator=port):
                self.assertEqual(_notices(proc_output, process, STAND_IN_PEER), [])
                self.assertEqual(_peer_status(port).get(STAND_IN_PEER), 'online')

    def _wait_for_checks(self, count, phase):
        # A /health starts a new check, so `count` of them mean `count - 1` checks have finished.
        with _SequencePeer.changed:
            self.assertTrue(
                _SequencePeer.changed.wait_for(
                    lambda: _SequencePeer.phases.count(phase) >= count, timeout=NOTICE_TIMEOUT),
                f'the stand-in served no {count} {phase} checks: {_SequencePeer.phases}')

    def test_05_only_a_conclusive_answer_replaces_a_refusal(self, proc_output, sequence):
        """A failed /health and inconclusive reads after it leave a refused peer offline."""
        self._wait_for_checks(2, 'inconclusive')
        self.assertEqual(_SequencePeer.phases[:2], ['refused', 'failed'])
        self.assertEqual(_peer_status(SEQUENCE_AGG_PORT).get(SEQUENCE_PEER), 'offline')
        notices = _notices(proc_output, sequence, SEQUENCE_PEER)
        self.assertEqual(len(notices), 1, notices)
        self.assertIn('Marked offline', notices[0])

        # A full /health shows the credential accepted.
        _SequencePeer.full.set()
        self._wait_for_checks(2, 'full')
        self.assertEqual(_peer_status(SEQUENCE_AGG_PORT).get(SEQUENCE_PEER), 'online')
        self.assertEqual(_notices(proc_output, sequence, SEQUENCE_PEER), notices)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
