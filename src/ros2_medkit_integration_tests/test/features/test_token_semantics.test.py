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

"""Token semantics on running gateways.

- ``POST /auth/revoke`` revokes refresh tokens only. An access token answers
  ``unsupported_token_type`` and changes nothing.
- A refresh token past its expiry, or issued by a clock running ahead, can
  still be revoked. The access tokens minted from it are refused from then on.
- The role a token grants comes from this gateway's client table. Any role
  claim, or none, is accepted.
- ``auth.max_refresh_tokens_per_client`` drops a client's oldest live refresh
  record at login, never a revoked one. Outside 1 to 100000 the gateway does
  not start.
- A revoked refresh record minted here is kept for one access lifetime.
  Revoked records count toward the cap; when they fill it, a login answers
  429 with ``Retry-After``.
- An open stream ends at its next write once its token is revoked or expired.
  A download ends at its next chunk and frees its worker.
- A revocation of a token minted elsewhere goes to a per-client pool of its
  own, kept until the token's ``exp`` plus one access lifetime. A full pool
  answers 503 with ``Retry-After`` and never blocks a login.
- Only the exact client secret authenticates.
- RS256 key files are read at start. Removing them before the first request
  changes nothing.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import atexit
import base64
from concurrent.futures import ThreadPoolExecutor
import hashlib
import hmac
import json
import os
import select
import shutil
import signal
import subprocess
import tempfile
import threading
import time
import unittest
import uuid

import launch
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    GATEWAY_STARTUP_TIMEOUT,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node

LONG_PORT = get_test_port(0)
SHORT_PORT = get_test_port(1)
CAPPED_PORT = get_test_port(2)
# Subprocess gateways, one per port.
RS256_PORT = get_test_port(3)
REFUSED_PORT = get_test_port(4)
LOWEST_CAP_PORT = get_test_port(5)
HIGHEST_CAP_PORT = get_test_port(6)
EXPIRY_PORT = get_test_port(7)
BRIEF_PORT = get_test_port(8)
FILL_PORT = get_test_port(9)


def _base_url(port):
    return f'http://127.0.0.1:{port}{API_BASE_PATH}'


LONG_BASE_URL = _base_url(LONG_PORT)
SHORT_BASE_URL = _base_url(SHORT_PORT)
CAPPED_BASE_URL = _base_url(CAPPED_PORT)
EXPIRY_BASE_URL = _base_url(EXPIRY_PORT)
BRIEF_BASE_URL = _base_url(BRIEF_PORT)
FILL_BASE_URL = _base_url(FILL_PORT)

JWT_SECRET = 'token_semantics_shared_secret_0123456789abcdef'
JWT_ISSUER = 'token_semantics_issuer'
CLIENT_ID = 'semantics'
# Mixed case, so a case change is a different secret.
CLIENT_SECRET = 'Token_Semantics_Secret_42'
CAP_PARAMETER = 'auth.max_refresh_tokens_per_client'
CAP = 3
# No gateway here has this app. The role check runs before the entity lookup.
ABSENT_APP = 'an_app_no_gateway_here_has'

# Access and refresh lifetime of the short gateway. An access token minted just
# before the refresh token expires outlives it by about this long.
SHORT_EXPIRY_SEC = 5
# How far ahead of this clock a skewed issuer stamps `iat`.
SKEW_SEC = 3

# The brief gateway: a revoked record minted there is kept for BRIEF_ACCESS_SEC.
BRIEF_ACCESS_SEC = 2
BRIEF_CAP = 2
FULL_CAP_CLIENT = ('cap_full', 'cap_full_secret')
MIXED_CAP_CLIENT = ('cap_mixed', 'cap_mixed_secret')
# Foreign revocations on the brief gateway: tokens signed here for a listed
# subject. Each client's pool holds BRIEF_CAP of them.
POOL_CLIENT = ('pool_foreign', 'pool_foreign_secret')
DROPPED_CLIENT = ('cap_dropped', 'cap_dropped_secret')
FOREIGN_LIFETIME_SEC = 3

# Downloads on the capped gateway: two workers, bulk data in BULK_DIR. The
# payload is larger than the loopback socket buffers, so it cannot be in
# flight whole when the token is revoked.
DOWNLOADER = ('downloader', 'downloader_secret')
BULK_CATEGORY = 'calibration'
BULK_DIR = tempfile.mkdtemp(prefix='token_semantics_bulk_')
atexit.register(shutil.rmtree, BULK_DIR, True)
DOWNLOAD_BYTES = 32 * 1024 * 1024

# Stream checks. The long gateway sends a fault-stream keepalive every
# LONG_KEEPALIVE_SEC; the expiry gateway every EXPIRY_KEEPALIVE_SEC, with
# access tokens of EXPIRY_ACCESS_SEC. A cyclic stream writes a frame per sample:
# the 0.2 s `normal` interval plus at most the 1 s topic sample timeout.
LONG_KEEPALIVE_SEC = 2
EXPIRY_KEEPALIVE_SEC = 1
EXPIRY_ACCESS_SEC = 8
CYCLIC_FRAME_SEC = 1.2
STREAM_MARGIN_SEC = 2.0 * get_time_scale()
DEMO_APP = 'temp_sensor'
SYSTEM_TOPICS = {'/parameter_events', '/rosout'}

POLL_INTERVAL = 0.1
READY_LINE = 'Medkit Gateway ready on'

# A token part ending in a run of base64url fills (`%3d`). Decoding copies the rest of
# the part once per fill. Header values are percent-decoded, so the bearer run is `%253d`.
BODY_FILL_TOKEN = '%3d' * 2700 + '.e30.sig'
HEADER_FILL_TOKEN = '%253d' * 1600 + '.e30.sig'
# The same lengths in JWS compact form, decoded in one pass: they take a fill's request path.
HEADER_WARM_TOKEN = 'A' * 4800 + '.e30.sig'
BODY_WARM_TOKEN = 'A' * 8100 + '.e30.sig'
FILL_REQUESTS = 16
# Peak RSS growth one batch of fill tokens may cause, in KiB.
MAX_FILL_PEAK_KIB = 16 * 1024
# What a token outside JWS compact form is told, before it is decoded.
REFUSED_SHAPE = 'JWT decode error: invalid token supplied'
BASE64URL = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_'


def _auth_params(**overrides):
    params = {
        'server.host': '127.0.0.1',
        'auth.enabled': True,
        'auth.require_auth_for': 'all',
        'auth.issuer': JWT_ISSUER,
        'auth.jwt_secret': JWT_SECRET,
        'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:operator'],
    }
    params.update(overrides)
    return params


def _without_asan_quarantine():
    """Return this run's ASAN_OPTIONS with no quarantine; inert without ASan."""
    options = [os.environ.get('ASAN_OPTIONS', ''), 'quarantine_size_mb=0']
    return {'ASAN_OPTIONS': ':'.join(option for option in options if option)}


@pytest.mark.launch_test
def generate_test_description():
    """Six closed HS256 gateways and one demo node for a cyclic subscription to sample."""
    # Streams hold an HTTP worker each, so the pool and the SSE cap are raised together.
    stream_params = {'sse.max_clients': 8, 'server.http_thread_pool_size': 16}
    long_lived = create_gateway_node(
        port=LONG_PORT, name='token_semantics_long',
        extra_params=_auth_params(**stream_params,
                                  **{'sse.keepalive_interval_sec': LONG_KEEPALIVE_SEC}))
    short_lived = create_gateway_node(
        port=SHORT_PORT, name='token_semantics_short',
        extra_params=_auth_params(**{
            'auth.token_expiry_seconds': SHORT_EXPIRY_SEC,
            'auth.refresh_token_expiry_seconds': SHORT_EXPIRY_SEC,
        }))
    capped = create_gateway_node(
        port=CAPPED_PORT, name='token_semantics_capped',
        extra_params=_auth_params(**{
            CAP_PARAMETER: CAP,
            # A download holds one worker; a stream can hold the other.
            'server.http_thread_pool_size': 2,
            'sse.keepalive_interval_sec': 1,
            'bulk_data.storage_dir': BULK_DIR,
            'bulk_data.categories': [BULK_CATEGORY],
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:operator',
                             f'{DOWNLOADER[0]}:{DOWNLOADER[1]}:operator'],
        }))
    expiring = create_gateway_node(
        port=EXPIRY_PORT, name='token_semantics_expiry',
        extra_params=_auth_params(**stream_params, **{
            'sse.keepalive_interval_sec': EXPIRY_KEEPALIVE_SEC,
            'auth.token_expiry_seconds': EXPIRY_ACCESS_SEC,
            'auth.refresh_token_expiry_seconds': 60,
        }))
    brief = create_gateway_node(
        port=BRIEF_PORT, name='token_semantics_brief',
        extra_params=_auth_params(**{
            'auth.token_expiry_seconds': BRIEF_ACCESS_SEC,
            'auth.refresh_token_expiry_seconds': 60,
            CAP_PARAMETER: BRIEF_CAP,
            'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:operator',
                             f'{FULL_CAP_CLIENT[0]}:{FULL_CAP_CLIENT[1]}:operator',
                             f'{MIXED_CAP_CLIENT[0]}:{MIXED_CAP_CLIENT[1]}:operator',
                             f'{POOL_CLIENT[0]}:{POOL_CLIENT[1]}:operator',
                             f'{DROPPED_CLIENT[0]}:{DROPPED_CLIENT[1]}:operator'],
        }))
    # Its peak RSS is measured. ASan reuses freed memory only without a quarantine, so
    # the peak then counts what requests hold, as it does without ASan.
    fill = create_gateway_node(
        port=FILL_PORT, name='token_semantics_fill',
        extra_params=_auth_params(**{'server.http_thread_pool_size': FILL_REQUESTS}),
        extra_env=_without_asan_quarantine())
    return launch.LaunchDescription([
        long_lived,
        short_lived,
        capped,
        expiring,
        brief,
        fill,
        *create_demo_nodes([DEMO_APP], lidar_faulty=False),
        launch_testing.actions.ReadyToTest(),
    ]), {'long_lived': long_lived, 'short_lived': short_lived, 'capped': capped,
         'expiring': expiring, 'brief': brief}


def _wait_until_answering(url):
    """Wait for any HTTP answer from /health. A refusal also means the process listens."""
    deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
    while time.monotonic() < deadline:
        try:
            if requests.get(f'{url}/health', timeout=2).status_code in (200, 401, 403):
                return
        except requests.RequestException:
            pass
        time.sleep(POLL_INTERVAL)
    raise AssertionError(f'{url} did not answer within {GATEWAY_STARTUP_TIMEOUT}s')


def _authorize(url, secret=CLIENT_SECRET, client_id=CLIENT_ID):
    return requests.post(
        f'{url}/auth/authorize',
        json={'grant_type': 'client_credentials', 'client_id': client_id,
              'client_secret': secret},
        timeout=30)


def _pair(url, client=(CLIENT_ID, CLIENT_SECRET)):
    resp = _authorize(url, secret=client[1], client_id=client[0])
    assert resp.status_code == 200, f'{url} issued no token: {resp.status_code} {resp.text}'
    return resp.json()


def _refresh(url, refresh_token, client=(CLIENT_ID, CLIENT_SECRET)):
    return requests.post(
        f'{url}/auth/token',
        json={'grant_type': 'refresh_token', 'refresh_token': refresh_token,
              'client_id': client[0], 'client_secret': client[1]},
        timeout=30)


def _revoke(url, token, client=(CLIENT_ID, CLIENT_SECRET)):
    return requests.post(
        f'{url}/auth/revoke',
        json={'token': token, 'client_id': client[0], 'client_secret': client[1]}, timeout=30)


def _bearer(access_token):
    return {'Authorization': f'Bearer {access_token}'}


def _read(url, access_token):
    return requests.get(f'{url}/areas', headers=_bearer(access_token), timeout=15)


def _listener_pid(port):
    """Return the pid of the one process listening on `port`."""
    inodes = set()
    for table in ('/proc/net/tcp', '/proc/net/tcp6'):
        with open(table, encoding='utf-8') as handle:
            for row in handle.read().splitlines()[1:]:
                fields = row.split()
                if fields[3] == '0A' and int(fields[1].split(':')[1], 16) == port:
                    inodes.add(f'socket:[{fields[9]}]')
    pids = set()
    for entry in filter(str.isdigit, os.listdir('/proc')):
        try:
            if any(os.readlink(f'/proc/{entry}/fd/{fd}') in inodes
                   for fd in os.listdir(f'/proc/{entry}/fd')):
                pids.add(int(entry))
        except OSError:
            continue
    assert len(pids) == 1, f'listeners on {port}: {pids}'
    return pids.pop()


def _peak_rss_kib(pid):
    """Return VmHWM, the peak resident set size of `pid`, in KiB."""
    with open(f'/proc/{pid}/status', encoding='utf-8') as handle:
        for line in handle:
            if line.startswith('VmHWM:'):
                return int(line.split()[1])
    raise AssertionError(f'no VmHWM for {pid}')


def _reset_peak_rss(pid):
    """Set VmHWM of `pid` to its current resident set size; a later reading counts later growth."""
    with open(f'/proc/{pid}/clear_refs', 'w', encoding='utf-8') as handle:
        handle.write('5')


def _exp(token):
    """Return the `exp` claim of a JWT."""
    payload = token.split('.')[1]
    payload += '=' * (-len(payload) % 4)
    return json.loads(base64.urlsafe_b64decode(payload))['exp']


def _jti(token):
    """Return the `jti` claim of a JWT."""
    payload = token.split('.')[1]
    payload += '=' * (-len(payload) % 4)
    return json.loads(base64.urlsafe_b64decode(payload))['jti']


class _Stream:
    """An SSE response read on its own thread.

    ``ended`` is set, and ``ended_at`` (monotonic) and ``ended_wall``
    recorded, when the server ends the response. ``close()`` by the test sets
    none of them.
    """

    def __init__(self, url, access_token):
        self.response = requests.get(url, headers=_bearer(access_token), stream=True,
                                     timeout=(5, 120))
        self.keepalives = 0
        self.last_line_at = None
        self.ended_at = None
        self.ended_wall = None
        self.line_seen = threading.Event()
        self.ended = threading.Event()
        self.closed = threading.Event()
        if self.response.status_code == 200:
            threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        try:
            for line in self.response.iter_lines():
                if line.startswith(b':'):
                    self.keepalives += 1
                self.last_line_at = time.monotonic()
                self.line_seen.set()
        except requests.RequestException:
            pass
        except Exception:
            # close() shuts the response under a read in progress, which can raise anything.
            if not self.closed.is_set():
                raise
        finally:
            if not self.closed.is_set():
                self.ended_at = time.monotonic()
                self.ended_wall = time.time()
                self.ended.set()

    def wait_for(self, condition, timeout):
        """Wait until `condition()` holds after a line; False at the deadline or the end."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            if self.ended.is_set():
                return False
            self.line_seen.wait(timeout=POLL_INTERVAL)
            self.line_seen.clear()
        return condition()

    def wait_for_line_after(self, moment, timeout):
        return self.wait_for(lambda: self.last_line_at is not None and self.last_line_at > moment,
                             timeout)

    def close(self):
        self.closed.set()
        self.response.close()


def _open_stream(test, url, access_token):
    """Open a _Stream and fail `test` unless it answers 200."""
    stream = _Stream(url, access_token)
    test.addCleanup(stream.close)
    # The body is read only on a refusal: an open stream has no end to read to.
    if stream.response.status_code != 200:
        test.fail(f'{url}: {stream.response.status_code} {stream.response.text[:300]}')
    return stream


def _b64(raw):
    """Encode base64url without padding, as JWT does."""
    return base64.urlsafe_b64encode(raw).rstrip(b'=')


def _b64decode(text):
    return base64.urlsafe_b64decode(text + '=' * (-len(text) % 4))


def _set_unused_bits(part):
    """Return `part` with the unused low bits of its last character set."""
    unused = {2: 4, 3: 2}[len(part) % 4]
    last = BASE64URL.index(part[-1]) | ((1 << unused) - 1)
    return part[:-1] + BASE64URL[last]


def _with_unused_bits_set(token):
    """Return the token with the unused low bits of its signature's last character set."""
    signing_input, signature = token.rsplit('.', 1)
    return f'{signing_input}.{_set_unused_bits(signature)}'


def _signed(signing_input):
    """Append an HS256 signature over the text of `signing_input`."""
    signature = hmac.new(JWT_SECRET.encode(), signing_input.encode(), hashlib.sha256).digest()
    return f'{signing_input}.{_b64(signature).decode()}'


def _sign(payload, typ='access'):
    """Sign an HS256 JWT with the gateways' shared secret."""
    header = {'alg': 'HS256', 'typ': typ}
    signing_input = b'.'.join([
        _b64(json.dumps(header, separators=(',', ':')).encode()),
        _b64(json.dumps(payload, separators=(',', ':')).encode()),
    ])
    signature = hmac.new(JWT_SECRET.encode(), signing_input, hashlib.sha256).digest()
    return (signing_input + b'.' + _b64(signature)).decode()


def _claims(**overrides):
    now = int(time.time())
    claims = {'iss': JWT_ISSUER, 'sub': CLIENT_ID, 'iat': now, 'exp': now + 600,
              'jti': uuid.uuid4().hex}
    claims.update(overrides)
    return claims


class _Gateway:
    """A gateway_node subprocess, read until its ready line or its exit.

    ``rc`` is the exit code when it ended before the ready line, else None.
    Leaving the block ends its process group.
    """

    def __init__(self, port, params):
        self.base_url = _base_url(port)
        self.command = ['ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
                        '-p', f'server.port:={port}']
        for key, value in params.items():
            self.command += ['-p', f'{key}:={json.dumps(value)}']
        self.rc = None
        self.ready = False
        self._chunks = []
        self._process = None

    @property
    def output(self):
        return b''.join(self._chunks).decode(errors='replace')

    def __enter__(self):
        env = {k: v for k, v in os.environ.items() if not k.startswith('MEDKIT_')}
        self._process = subprocess.Popen(
            self.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, env=env,
            start_new_session=True)
        deadline = time.monotonic() + GATEWAY_STARTUP_TIMEOUT
        while time.monotonic() < deadline and not self.ready:
            readable, _, _ = select.select([self._process.stdout], [], [], POLL_INTERVAL)
            if not readable:
                continue
            chunk = os.read(self._process.stdout.fileno(), 65536)
            if not chunk:
                self.rc = self._process.wait(timeout=30)
                break
            self._chunks.append(chunk)
            self.ready = READY_LINE in self.output
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._process.poll() is None:
            os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            try:
                self._process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
        rest, _ = self._process.communicate()
        if rest:
            self._chunks.append(rest)
        return False


class TestRevocation(unittest.TestCase):
    """Only a refresh token is revocable, and an expired one still is."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(LONG_BASE_URL)
        _wait_until_answering(SHORT_BASE_URL)

    def test_01_revoking_an_access_token_changes_nothing(self):
        """RFC 7009 2.2.1 refuses the token type, and both tokens keep working."""
        pair = _pair(LONG_BASE_URL)

        resp = _revoke(LONG_BASE_URL, pair['access_token'])
        self.assertEqual(resp.status_code, 400, resp.text)
        self.assertEqual(resp.json().get('error'), 'unsupported_token_type', resp.text)

        self.assertEqual(_read(LONG_BASE_URL, pair['access_token']).status_code, 200,
                         'revoking an access token withdrew it')
        refreshed = _refresh(LONG_BASE_URL, pair['refresh_token'])
        self.assertEqual(refreshed.status_code, 200,
                         f'revoking an access token withdrew its refresh token: {refreshed.text}')

        # The same route does revoke the refresh token, so it is not a dead route.
        resp = _revoke(LONG_BASE_URL, pair['refresh_token'])
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(resp.json(), {'status': 'revoked'})
        refused = _read(LONG_BASE_URL, pair['access_token'])
        self.assertEqual(refused.status_code, 401)
        self.assertIn('revoked', refused.text)

    def test_02_an_expired_refresh_token_is_revocable(self):
        """Revoke a refresh token past its expiry; a late access token from it is refused."""
        pair = _pair(SHORT_BASE_URL)
        refresh = pair['refresh_token']

        # Refresh until the gateway refuses the refresh token as expired. The last success is
        # minted just before that and outlives it by about SHORT_EXPIRY_SEC.
        late = None
        refusal = None
        deadline = time.monotonic() + (SHORT_EXPIRY_SEC + 30) * get_time_scale()
        while time.monotonic() < deadline:
            resp = _refresh(SHORT_BASE_URL, refresh)
            if resp.status_code == 200:
                late = resp.json()['access_token']
            else:
                refusal = resp
                break
            time.sleep(POLL_INTERVAL)
        self.assertIsNotNone(refusal, 'the refresh token never expired')
        self.assertIsNotNone(late, 'no refresh succeeded before the refresh token expired')
        self.assertEqual(refusal.status_code, 400, refusal.text)
        self.assertEqual(refusal.json().get('error'), 'invalid_grant', refusal.text)
        self.assertIn('expired', refusal.text)

        self.assertEqual(_read(SHORT_BASE_URL, late).status_code, 200,
                         'the late access token was refused before any revocation')

        resp = _revoke(SHORT_BASE_URL, refresh)
        self.assertEqual(resp.status_code, 200, resp.text)

        refused = _read(SHORT_BASE_URL, late)
        self.assertEqual(
            refused.status_code, 401,
            'an access token minted from a revoked, expired refresh token was accepted')
        self.assertIn('revoked', refused.text)

    def test_03_a_revocation_holds_when_the_issuer_clock_runs_ahead(self):
        """Two pairs stamped SKEW_SEC ahead; one refresh token is revoked at once."""
        ahead = int(time.time()) + SKEW_SEC
        pairs = []
        for _ in range(2):
            jti = uuid.uuid4().hex
            refresh = _sign(_claims(iat=ahead, exp=ahead + 600, jti=jti), typ='refresh')
            access = _sign(_claims(iat=ahead, exp=ahead + 600, refresh_token_id=jti))
            pairs.append((refresh, access))
        (revoked_refresh, revoked_access), (_, kept_access) = pairs

        resp = _revoke(LONG_BASE_URL, revoked_refresh)
        self.assertEqual(resp.status_code, 200, resp.text)

        # Wait for `iat` to pass: the access token of the unrevoked pair becomes valid.
        deadline = time.monotonic() + (SKEW_SEC + 30) * get_time_scale()
        kept = _read(LONG_BASE_URL, kept_access)
        while kept.status_code != 200 and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
            kept = _read(LONG_BASE_URL, kept_access)
        self.assertEqual(kept.status_code, 200, f'the unrevoked pair stayed invalid: {kept.text}')

        refused = _read(LONG_BASE_URL, revoked_access)
        self.assertEqual(
            refused.status_code, 401,
            'an access token whose refresh token was revoked ahead of its iat was accepted')
        self.assertIn('revoked', refused.text)


class TestMalformedTokensAreRefusedBeforeDecoding(unittest.TestCase):
    """A token outside JWS compact form costs no memory at any entry point that decodes."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(LONG_BASE_URL)
        _wait_until_answering(FILL_BASE_URL)
        cls.pid = _listener_pid(FILL_PORT)

    def _batch(self, send, token):
        """Send FILL_REQUESTS requests at once; return the set of their statuses."""
        with ThreadPoolExecutor(FILL_REQUESTS) as pool:
            answers = list(pool.map(lambda _: send(token), range(FILL_REQUESTS)))
        return {resp.status_code for resp in answers}

    def test_01_a_run_of_fills_is_refused_before_it_is_decoded(self):
        entry_points = {
            'bearer header': (lambda token: _read(FILL_BASE_URL, token),
                              HEADER_WARM_TOKEN, HEADER_FILL_TOKEN, 401),
            # The client authenticates first; an invalid refresh token is then invalid_grant.
            'refresh_token body': (lambda token: _refresh(FILL_BASE_URL, token),
                                   BODY_WARM_TOKEN, BODY_FILL_TOKEN, 400),
            'revoke token body': (lambda token: _revoke(FILL_BASE_URL, token),
                                  BODY_WARM_TOKEN, BODY_FILL_TOKEN, 200),
        }
        for name, (send, warm, fill, status) in entry_points.items():
            with self.subTest(entry_point=name):
                # A path's first batch costs allocator and ASan bookkeeping once; spend it here.
                self.assertEqual(self._batch(send, warm), {status})
                _reset_peak_rss(self.pid)
                before = _peak_rss_kib(self.pid)
                self.assertEqual(self._batch(send, fill), {status})
                grown = _peak_rss_kib(self.pid) - before
                print(f'{name}: {FILL_REQUESTS} requests raised the peak RSS by {grown} KiB')
                self.assertLess(grown, MAX_FILL_PEAK_KIB,
                                f'{FILL_REQUESTS} requests raised the peak RSS by {grown} KiB')
        # A token in compact form is still decoded and accepted.
        pair = _pair(FILL_BASE_URL)
        self.assertEqual(_read(FILL_BASE_URL, pair['access_token']).status_code, 200)
        self.assertEqual(_refresh(FILL_BASE_URL, pair['refresh_token']).status_code, 200)

    def test_02_a_part_that_is_empty_or_of_no_base64url_length_is_refused(self):
        """Each part is non-empty and no unpadded base64url text has a length of 1 mod 4."""
        pair = _pair(LONG_BASE_URL)
        head, body, signature = pair['access_token'].split('.')
        shapes = {
            'a one-character part': f'A.{body}.{signature}',
            'a five-character part': f'{head}.AAAAA.{signature}',
            'an empty header': f'.{body}.{signature}',
            'an empty payload': f'{head}..{signature}',
            'an empty signature': f'{head}.{body}.',
        }
        for shape, token in shapes.items():
            with self.subTest(shape=shape, entry_point='bearer header'):
                resp = _read(LONG_BASE_URL, token)
                self.assertEqual(resp.status_code, 401, resp.text)
                self.assertEqual(resp.json().get('error_description'), REFUSED_SHAPE, resp.text)
            with self.subTest(shape=shape, entry_point='refresh_token body'):
                # The client authenticates first; an invalid refresh token is then invalid_grant.
                resp = _refresh(LONG_BASE_URL, token)
                self.assertEqual(resp.status_code, 400, resp.text)
                self.assertEqual(resp.json().get('error'), 'invalid_grant', resp.text)
                self.assertEqual(resp.json().get('error_description'), REFUSED_SHAPE, resp.text)
            with self.subTest(shape=shape, entry_point='revoke token body'):
                resp = _revoke(LONG_BASE_URL, token)
                self.assertEqual((resp.status_code, resp.json()), (200, {'status': 'revoked'}))
        # The same parts in the right shape still decode.
        self.assertEqual(_read(LONG_BASE_URL, pair['access_token']).status_code, 200)

    def test_03_a_part_whose_unused_bits_are_not_zero_is_refused(self):
        """Unpadded base64url leaves the unused bits of a part's last character zero."""
        pair = _pair(LONG_BASE_URL)
        tokens = {name: _with_unused_bits_set(pair[name])
                  for name in ('access_token', 'refresh_token')}
        for name, token in tokens.items():
            # Another spelling of the same bytes, so only the spelling can refuse it.
            self.assertNotEqual(token, pair[name])
            self.assertEqual(_b64decode(token.rsplit('.', 1)[1]),
                             _b64decode(pair[name].rsplit('.', 1)[1]))
        with self.subTest(entry_point='bearer header'):
            resp = _read(LONG_BASE_URL, tokens['access_token'])
            self.assertEqual(resp.status_code, 401, resp.text)
            self.assertEqual(resp.json().get('error_description'), REFUSED_SHAPE, resp.text)
        with self.subTest(entry_point='refresh_token body'):
            # The client authenticates first; an invalid refresh token is then invalid_grant.
            resp = _refresh(LONG_BASE_URL, tokens['refresh_token'])
            self.assertEqual(resp.status_code, 400, resp.text)
            self.assertEqual(resp.json().get('error'), 'invalid_grant', resp.text)
            self.assertEqual(resp.json().get('error_description'), REFUSED_SHAPE, resp.text)
        with self.subTest(entry_point='revoke token body'):
            resp = _revoke(LONG_BASE_URL, tokens['refresh_token'])
            self.assertEqual((resp.status_code, resp.json()), (200, {'status': 'revoked'}))
            # The refused spelling revoked nothing: the canonical one still refreshes.
            resp = _refresh(LONG_BASE_URL, pair['refresh_token'])
            self.assertEqual(resp.status_code, 200, resp.text)
        with self.subTest(part='a payload of length 2 mod 4, 4 unused bits'):
            claims = _claims(pad='')
            while len(_b64(json.dumps(claims, separators=(',', ':')).encode())) % 4 != 2:
                claims['pad'] += 'x'
            canonical = _sign(claims)
            head, body, _ = canonical.split('.')
            altered = _signed(f'{head}.{_set_unused_bits(body)}')
            self.assertEqual(_b64decode(_set_unused_bits(body)), _b64decode(body))
            self.assertEqual(_read(LONG_BASE_URL, canonical).status_code, 200)
            resp = _read(LONG_BASE_URL, altered)
            self.assertEqual(resp.status_code, 401, resp.text)
            self.assertEqual(resp.json().get('error_description'), REFUSED_SHAPE, resp.text)


class TestRoleClaimIsNotConsulted(unittest.TestCase):
    """Any role claim, or none, decodes; the client table grants operator."""

    VARIANTS = {'an unknown name': 'root', 'not a string': 7, 'absent': None,
                'the real role': 'operator'}

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(LONG_BASE_URL)

    def test_01_every_role_claim_gets_the_tables_role(self):
        for label, role in self.VARIANTS.items():
            with self.subTest(role=label):
                token = _sign(_claims() if role is None else _claims(role=role))
                auth = {'Authorization': f'Bearer {token}'}

                read = requests.get(f'{LONG_BASE_URL}/areas', headers=auth, timeout=15)
                self.assertEqual(read.status_code, 200, read.text[:300])

                # Configurator and above.
                forbidden = requests.put(
                    f'{LONG_BASE_URL}/apps/{ABSENT_APP}/configurations/p',
                    headers=auth, json={'data': 1}, timeout=15)
                self.assertEqual(forbidden.status_code, 403, forbidden.text[:300])
                self.assertEqual(forbidden.json().get('error'), 'insufficient_scope')

                # Operator and above, so the role is operator, not a default.
                allowed = requests.post(
                    f'{LONG_BASE_URL}/apps/{ABSENT_APP}/operations/probe/executions',
                    headers=auth, json={}, timeout=15)
                self.assertNotIn(allowed.status_code, (401, 403), allowed.text[:300])


class TestClientSecret(unittest.TestCase):
    """Only the exact secret authenticates."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(LONG_BASE_URL)

    def test_01_wrong_secrets_are_refused(self):
        wrong = {
            'a prefix': CLIENT_SECRET[:5],
            'one byte short': CLIENT_SECRET[:-1],
            'one byte long': CLIENT_SECRET + '2',
            'last byte changed': CLIENT_SECRET[:-1] + '3',
            'first byte changed': 't' + CLIENT_SECRET[1:],
            'other case': CLIENT_SECRET.swapcase(),
        }
        for label, secret in wrong.items():
            with self.subTest(secret=label):
                self.assertNotEqual(secret, CLIENT_SECRET)
                resp = _authorize(LONG_BASE_URL, secret)
                self.assertEqual(resp.status_code, 401, resp.text)
                self.assertEqual(resp.json().get('error'), 'invalid_client', resp.text)

    def test_02_an_empty_secret_is_refused_before_the_comparison(self):
        """The handler answers a missing secret with 401 invalid_client (RFC 6749 5.2)."""
        resp = _authorize(LONG_BASE_URL, '')
        self.assertEqual(resp.status_code, 401, resp.text)
        self.assertEqual(resp.json().get('error'), 'invalid_client', resp.text)

    def test_03_the_exact_secret_authenticates(self):
        resp = _authorize(LONG_BASE_URL)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(_read(LONG_BASE_URL, resp.json()['access_token']).status_code, 200)


class TestRefreshRecordIsBoundToItsClient(unittest.TestCase):
    """A token signed with the shared key, naming one client and another's record, is refused."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(CAPPED_BASE_URL)

    def test_01_a_record_of_another_client_neither_refreshes_nor_revokes(self):
        pair = _pair(CAPPED_BASE_URL)
        forged = _sign(_claims(sub=DOWNLOADER[0], jti=_jti(pair['refresh_token'])), typ='refresh')

        refused = _refresh(CAPPED_BASE_URL, forged, DOWNLOADER)
        self.assertEqual(refused.status_code, 400, refused.text)
        self.assertEqual(refused.json().get('error'), 'invalid_grant', refused.text)

        resp = _revoke(CAPPED_BASE_URL, forged, DOWNLOADER)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(_refresh(CAPPED_BASE_URL, pair['refresh_token']).status_code, 200,
                         "another client's token revoked this client's record")
        self.assertEqual(_read(CAPPED_BASE_URL, pair['access_token']).status_code, 200)


class TestRefreshTokenCap(unittest.TestCase):
    """The capped gateway keeps at most CAP live refresh records per client."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(CAPPED_BASE_URL)

    def test_01_a_login_past_the_cap_drops_the_oldest_live_record(self):
        pairs = [_pair(CAPPED_BASE_URL) for _ in range(CAP + 1)]

        oldest = _refresh(CAPPED_BASE_URL, pairs[0]['refresh_token'])
        self.assertEqual(oldest.status_code, 400,
                         f'the oldest refresh token survived a login past the cap: {oldest.text}')
        for index, pair in enumerate(pairs[1:], start=2):
            resp = _refresh(CAPPED_BASE_URL, pair['refresh_token'])
            self.assertEqual(resp.status_code, 200, f'login {index}: {resp.text}')

        # A dropped record stops refreshing only. Its access token runs to its own expiry.
        self.assertEqual(_read(CAPPED_BASE_URL, pairs[0]['access_token']).status_code, 200)

    def test_02_the_cap_never_drops_a_revoked_record(self):
        pairs = [_pair(CAPPED_BASE_URL) for _ in range(CAP + 1)]
        withdrawn = pairs[1]

        self.assertEqual(_read(CAPPED_BASE_URL, withdrawn['access_token']).status_code, 200)
        resp = _revoke(CAPPED_BASE_URL, withdrawn['refresh_token'])
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(_read(CAPPED_BASE_URL, withdrawn['access_token']).status_code, 401)

        for _ in range(5):
            latest = _pair(CAPPED_BASE_URL)

        refused = _read(CAPPED_BASE_URL, withdrawn['access_token'])
        self.assertEqual(refused.status_code, 401,
                         'the cap dropped a revoked record and its access token came back')
        self.assertIn('revoked', refused.text)
        self.assertEqual(_read(CAPPED_BASE_URL, latest['access_token']).status_code, 200)
        self.assertEqual(_refresh(CAPPED_BASE_URL, latest['refresh_token']).status_code, 200)


class TestRefreshTokenCapRange(unittest.TestCase):
    """The cap is 1 to 100000; outside that the gateway does not start."""

    def test_01_outside_the_range_refuses_and_names_the_parameter(self):
        for value in (0, 100001):
            with self.subTest(value=value):
                with _Gateway(REFUSED_PORT, _auth_params(**{CAP_PARAMETER: value})) as gateway:
                    self.assertIsNotNone(gateway.rc, f'still running: {gateway.output[-600:]}')
                    self.assertNotEqual(gateway.rc, 0, gateway.output[-600:])
                    self.assertIn(CAP_PARAMETER, gateway.output)
                    self.assertNotIn(READY_LINE, gateway.output)

    def test_02_both_bounds_start(self):
        for port, value in ((LOWEST_CAP_PORT, 1), (HIGHEST_CAP_PORT, 100000)):
            with self.subTest(value=value):
                with _Gateway(port, _auth_params(**{CAP_PARAMETER: value})) as gateway:
                    self.assertTrue(gateway.ready, gateway.output[-600:])
                    pair = _pair(gateway.base_url)
                    read = _read(gateway.base_url, pair['access_token'])
                    self.assertEqual(read.status_code, 200)
                    self.assertEqual(
                        _refresh(gateway.base_url, pair['refresh_token']).status_code, 200)


class TestRs256KeysAreReadAtStart(unittest.TestCase):
    """Both key files removed before the first request: signing and verifying still work."""

    def setUp(self):
        self.key_dir = tempfile.mkdtemp(prefix='medkit_token_semantics_')
        self.addCleanup(shutil.rmtree, self.key_dir, True)
        self.private_key = os.path.join(self.key_dir, 'private.pem')
        self.public_key = os.path.join(self.key_dir, 'public.pem')
        subprocess.run(['openssl', 'genrsa', '-out', self.private_key, '2048'],
                       check=True, capture_output=True, timeout=60)
        subprocess.run(['openssl', 'rsa', '-in', self.private_key, '-pubout',
                        '-out', self.public_key],
                       check=True, capture_output=True, timeout=60)

    def test_01_removed_key_files_change_nothing(self):
        params = _auth_params(**{
            'auth.jwt_algorithm': 'RS256',
            'auth.jwt_secret': self.private_key,
            'auth.jwt_public_key': self.public_key,
        })
        with _Gateway(RS256_PORT, params) as gateway:
            self.assertTrue(gateway.ready, gateway.output[-600:])
            os.remove(self.private_key)
            os.remove(self.public_key)
            self.assertFalse(os.path.exists(self.private_key))
            self.assertFalse(os.path.exists(self.public_key))

            resp = _authorize(gateway.base_url)
            self.assertEqual(resp.status_code, 200, f'authorize after removal: {resp.text}')
            pair = resp.json()
            self.assertEqual(_read(gateway.base_url, pair['access_token']).status_code, 200,
                             'a token issued after the files were removed does not verify')

            refreshed = _refresh(gateway.base_url, pair['refresh_token'])
            self.assertEqual(
                refreshed.status_code, 200, f'refresh after removal: {refreshed.text}')
            self.assertEqual(
                _read(gateway.base_url, refreshed.json()['access_token']).status_code, 200)

            # Verification still runs: one base64url character of the signature changed is refused.
            header, payload, signature = pair['access_token'].split('.')
            alphabet = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_'
            replacement = alphabet[(alphabet.index(signature[0]) + 1) % len(alphabet)]
            forged = '.'.join([header, payload, replacement + signature[1:]])
            self.assertEqual(_read(gateway.base_url, forged).status_code, 401)


class TestRevokedRecordHorizon(unittest.TestCase):
    """On the brief gateway a revoked record minted there is kept for one access lifetime."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(BRIEF_BASE_URL)

    def test_01_a_revoked_refresh_token_stays_refused_after_its_record_is_gone(self):
        pair = _pair(BRIEF_BASE_URL)
        resp = _revoke(BRIEF_BASE_URL, pair['refresh_token'])
        self.assertEqual(resp.status_code, 200, resp.text)
        refused = _refresh(BRIEF_BASE_URL, pair['refresh_token'])
        self.assertEqual(refused.status_code, 400)
        self.assertIn('revoked', refused.text)

        # Every read of the access token is refused, as revoked and then as expired.
        deadline = time.monotonic() + (BRIEF_ACCESS_SEC + 30) * get_time_scale()
        read = _read(BRIEF_BASE_URL, pair['access_token'])
        while 'expired' not in read.text and time.monotonic() < deadline:
            self.assertEqual(read.status_code, 401, 'a revoked access token was accepted')
            self.assertIn('revoked', read.text)
            time.sleep(POLL_INTERVAL)
            read = _read(BRIEF_BASE_URL, pair['access_token'])
        self.assertEqual(read.status_code, 401)
        self.assertIn('expired', read.text)

        # A login sweeps; once the record is gone the refresh token is unknown, and still refused.
        deadline = time.monotonic() + (BRIEF_ACCESS_SEC + 30) * get_time_scale()
        while True:
            latest = _pair(BRIEF_BASE_URL)
            refused = _refresh(BRIEF_BASE_URL, pair['refresh_token'])
            self.assertEqual(refused.status_code, 400, 'a revoked refresh token refreshed')
            if 'not found' in refused.text or time.monotonic() >= deadline:
                break
            time.sleep(POLL_INTERVAL)
        self.assertIn('not found', refused.text,
                      'the revoked record outlived the access lifetime it has to cover')
        self.assertEqual(_refresh(BRIEF_BASE_URL, latest['refresh_token']).status_code, 200)


class TestRevokedRecordsCountTowardTheCap(unittest.TestCase):
    """On the brief gateway each client holds at most BRIEF_CAP refresh records."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(BRIEF_BASE_URL)

    def test_01_a_live_record_makes_room_when_one_is_revoked(self):
        first = _pair(BRIEF_BASE_URL, MIXED_CAP_CLIENT)
        second = _pair(BRIEF_BASE_URL, MIXED_CAP_CLIENT)
        self.assertEqual(
            _revoke(BRIEF_BASE_URL, first['refresh_token'], MIXED_CAP_CLIENT).status_code, 200)

        third = _pair(BRIEF_BASE_URL, MIXED_CAP_CLIENT)
        dropped = _refresh(BRIEF_BASE_URL, second['refresh_token'], MIXED_CAP_CLIENT)
        self.assertEqual(dropped.status_code, 400,
                         f'the live record survived a login at the cap: {dropped.text}')
        self.assertEqual(
            _refresh(BRIEF_BASE_URL, third['refresh_token'], MIXED_CAP_CLIENT).status_code, 200)
        withdrawn = _read(BRIEF_BASE_URL, first['access_token'])
        self.assertEqual(withdrawn.status_code, 401)
        self.assertIn('revoked', withdrawn.text)

    def test_02_revoked_records_filling_the_cap_refuse_a_login_until_they_expire(self):
        pairs = [_pair(BRIEF_BASE_URL, FULL_CAP_CLIENT) for _ in range(BRIEF_CAP)]
        for pair in pairs:
            self.assertEqual(
                _revoke(BRIEF_BASE_URL, pair['refresh_token'], FULL_CAP_CLIENT).status_code, 200)

        refused = _authorize(BRIEF_BASE_URL, secret=FULL_CAP_CLIENT[1],
                             client_id=FULL_CAP_CLIENT[0])
        self.assertEqual(refused.status_code, 429, refused.text)
        retry_after = int(refused.headers.get('Retry-After', '0'))
        self.assertGreaterEqual(retry_after, 1)
        self.assertLessEqual(retry_after, BRIEF_ACCESS_SEC + 1)
        body = refused.json()
        self.assertEqual(body.get('error'), 'temporarily_unavailable', body)
        self.assertIn(CAP_PARAMETER, body.get('error_description', ''))

        # Once the revoked records' horizon passes, a login sweeps them and succeeds.
        deadline = time.monotonic() + (retry_after + 30) * get_time_scale()
        resp = refused
        while resp.status_code == 429 and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
            resp = _authorize(BRIEF_BASE_URL, secret=FULL_CAP_CLIENT[1],
                              client_id=FULL_CAP_CLIENT[0])
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertEqual(_read(BRIEF_BASE_URL, resp.json()['access_token']).status_code, 200)


class TestOpenStreamsEndWithTheirToken(unittest.TestCase):
    """A stream ends at its next write once its token no longer validates."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(LONG_BASE_URL)
        _wait_until_answering(EXPIRY_BASE_URL)

    def _open(self, url, access_token):
        return _open_stream(self, url, access_token)

    def _assert_revocation_ends(self, gone, stays, gone_refresh, bound, label):
        timeout = (LONG_KEEPALIVE_SEC + CYCLIC_FRAME_SEC) * 4 + STREAM_MARGIN_SEC
        self.assertTrue(gone.wait_for(lambda: gone.last_line_at is not None, timeout))
        self.assertTrue(stays.wait_for(lambda: stays.last_line_at is not None, timeout))

        revoked_at = time.monotonic()
        resp = _revoke(LONG_BASE_URL, gone_refresh)
        self.assertEqual(resp.status_code, 200, resp.text)

        self.assertTrue(gone.ended.wait(timeout=bound),
                        f'{label}: still open {bound:.1f}s after its token was revoked')
        took = gone.ended_at - revoked_at
        print(f'{label}: closed {took:.2f}s after revocation (bound {bound:.1f}s)')
        self.assertGreaterEqual(took, 0.0, f'{label}: ended before the revocation')
        self.assertLessEqual(took, bound)

        self.assertTrue(stays.wait_for_line_after(revoked_at + bound, timeout=bound * 3),
                        f'{label}: the stream with a valid token did not stay open')
        self.assertFalse(stays.ended.is_set())

    def test_01_revocation_ends_an_open_fault_stream(self):
        gone, stays = _pair(LONG_BASE_URL), _pair(LONG_BASE_URL)
        self._assert_revocation_ends(
            self._open(f'{LONG_BASE_URL}/faults/stream', gone['access_token']),
            self._open(f'{LONG_BASE_URL}/faults/stream', stays['access_token']),
            gone['refresh_token'], LONG_KEEPALIVE_SEC + STREAM_MARGIN_SEC, 'fault stream')

    def test_02_revocation_ends_an_open_cyclic_subscription_stream(self):
        gone, stays = _pair(LONG_BASE_URL), _pair(LONG_BASE_URL)
        resource = self._data_resource(stays['access_token'])
        streams = []
        for pair in (gone, stays):
            created = requests.post(
                f'{LONG_BASE_URL}/apps/{DEMO_APP}/cyclic-subscriptions',
                headers=_bearer(pair['access_token']),
                json={'resource': resource, 'interval': 'normal', 'duration': 60}, timeout=15)
            self.assertEqual(created.status_code, 201, created.text)
            sub_id = created.json()['id']
            streams.append(self._open(
                f'{LONG_BASE_URL}/apps/{DEMO_APP}/cyclic-subscriptions/{sub_id}/events',
                pair['access_token']))
        self._assert_revocation_ends(streams[0], streams[1], gone['refresh_token'],
                                     CYCLIC_FRAME_SEC + STREAM_MARGIN_SEC, 'cyclic stream')

    def test_03_expiry_ends_an_open_fault_stream(self):
        first = _pair(EXPIRY_BASE_URL)
        exp = _exp(first['access_token'])
        gone = self._open(f'{EXPIRY_BASE_URL}/faults/stream', first['access_token'])

        # A second token minted three keepalives later outlives the first by about that much.
        self.assertTrue(gone.wait_for(lambda: gone.keepalives >= 3,
                                      timeout=EXPIRY_ACCESS_SEC * get_time_scale()))
        stays = self._open(f'{EXPIRY_BASE_URL}/faults/stream',
                           _pair(EXPIRY_BASE_URL)['access_token'])

        bound = EXPIRY_KEEPALIVE_SEC + STREAM_MARGIN_SEC
        self.assertTrue(gone.ended.wait(timeout=max(0.0, exp - time.time()) + bound),
                        'the stream stayed open after its token expired')
        # Expiry is a wall-clock event, and the wall clock can step against the monotonic one.
        took = gone.ended_wall - exp
        print(f'expired token: closed {took:.2f}s after expiry (bound {bound:.1f}s)')
        # The token is valid through its `exp` second; one second absorbs the wall-clock reads.
        self.assertGreaterEqual(took, -1.0, 'the stream ended before its token expired')
        self.assertLessEqual(took, bound)
        self.assertTrue(stays.wait_for_line_after(gone.ended_at, timeout=bound * 2),
                        'the stream with a valid token did not stay open')

    def _data_resource(self, access_token):
        """Return the data resource of the demo app, once the gateway serves one."""
        deadline = time.monotonic() + 60 * get_time_scale()
        while time.monotonic() < deadline:
            resp = requests.get(f'{LONG_BASE_URL}/apps/{DEMO_APP}/data',
                                headers=_bearer(access_token), timeout=15)
            if resp.status_code == 200:
                for item in resp.json().get('items', []):
                    if item['id'] not in SYSTEM_TOPICS:
                        return f'/api/v1/apps/{DEMO_APP}/data{item["id"]}'
            time.sleep(POLL_INTERVAL)
        self.fail(f'{DEMO_APP} served no data item')


def _foreign_pair(client_id, lifetime):
    """Sign a refresh token and an access token minted from it, for `client_id`."""
    now = int(time.time())
    jti = uuid.uuid4().hex
    refresh = _sign(_claims(sub=client_id, iat=now, exp=now + lifetime, jti=jti), typ='refresh')
    access = _sign(_claims(sub=client_id, iat=now, exp=now + lifetime, refresh_token_id=jti))
    return refresh, access


class TestForeignRevocationPool(unittest.TestCase):
    """On the brief gateway each client holds BRIEF_CAP foreign revocations, apart from logins."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(BRIEF_BASE_URL)

    def test_01_a_full_pool_refuses_a_revocation_and_never_a_login(self):
        recorded = [_foreign_pair(POOL_CLIENT[0], FOREIGN_LIFETIME_SEC) for _ in range(BRIEF_CAP)]
        extra_refresh, extra_access = _foreign_pair(POOL_CLIENT[0], 600)
        for refresh, _ in recorded:
            resp = _revoke(BRIEF_BASE_URL, refresh, POOL_CLIENT)
            self.assertEqual(resp.status_code, 200, resp.text)

        full = _revoke(BRIEF_BASE_URL, extra_refresh, POOL_CLIENT)
        self.assertEqual(full.status_code, 503, full.text)
        retry_after = int(full.headers.get('Retry-After', '0'))
        self.assertGreaterEqual(retry_after, 1)
        self.assertLessEqual(retry_after, FOREIGN_LIFETIME_SEC + BRIEF_ACCESS_SEC + 1)
        body = full.json()
        self.assertEqual(body.get('error'), 'temporarily_unavailable', body)
        self.assertIn(CAP_PARAMETER, body.get('error_description', ''))

        # Nothing was dropped to make room, and the refused token was not recorded.
        for _, access in recorded:
            refused = _read(BRIEF_BASE_URL, access)
            self.assertEqual(refused.status_code, 401)
            self.assertIn('revoked', refused.text)
        self.assertEqual(_read(BRIEF_BASE_URL, extra_access).status_code, 200)

        # A full pool does not block a login.
        login = _authorize(BRIEF_BASE_URL, secret=POOL_CLIENT[1], client_id=POOL_CLIENT[0])
        self.assertEqual(login.status_code, 200, login.text)
        self.assertEqual(_read(BRIEF_BASE_URL, login.json()['access_token']).status_code, 200)

        # Once the oldest record is swept, the revocation is accepted.
        deadline = time.monotonic() + (retry_after + 30) * get_time_scale()
        resp = full
        while resp.status_code == 503 and time.monotonic() < deadline:
            time.sleep(POLL_INTERVAL)
            resp = _revoke(BRIEF_BASE_URL, extra_refresh, POOL_CLIENT)
        self.assertEqual(resp.status_code, 200, resp.text)
        refused = _read(BRIEF_BASE_URL, extra_access)
        self.assertEqual(refused.status_code, 401)
        self.assertIn('revoked', refused.text)

    def test_02_a_token_the_cap_dropped_is_revoked_without_locking_its_client_out(self):
        pairs = [_pair(BRIEF_BASE_URL, DROPPED_CLIENT) for _ in range(BRIEF_CAP * 2)]
        dropped, kept = pairs[:BRIEF_CAP], pairs[BRIEF_CAP:]
        for pair in dropped:
            gone = _refresh(BRIEF_BASE_URL, pair['refresh_token'], DROPPED_CLIENT)
            self.assertEqual(gone.status_code, 400, 'the cap kept a record it had to drop')
            self.assertIn('not found', gone.text)
            resp = _revoke(BRIEF_BASE_URL, pair['refresh_token'], DROPPED_CLIENT)
            self.assertEqual(resp.status_code, 200, resp.text)
        for pair in kept:
            self.assertEqual(
                _refresh(BRIEF_BASE_URL, pair['refresh_token'], DROPPED_CLIENT).status_code, 200)

        login = _authorize(BRIEF_BASE_URL, secret=DROPPED_CLIENT[1], client_id=DROPPED_CLIENT[0])
        self.assertEqual(login.status_code, 200, login.text)
        self.assertEqual(_read(BRIEF_BASE_URL, login.json()['access_token']).status_code, 200)


class TestForeignRecordHorizon(unittest.TestCase):
    """On the short gateway a foreign record outlives this gateway's own refresh lifetime."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(SHORT_BASE_URL)

    def test_01_a_foreign_record_is_kept_until_its_token_expires(self):
        refresh, access = _foreign_pair(CLIENT_ID, 600)
        _, unrevoked_access = _foreign_pair(CLIENT_ID, 600)
        resp = _revoke(SHORT_BASE_URL, refresh)
        self.assertEqual(resp.status_code, 200, resp.text)
        # min(exp, revocation + refresh lifetime) + access lifetime: the record must outlive it.
        cut_off = int(time.time()) + SHORT_EXPIRY_SEC + SHORT_EXPIRY_SEC

        deadline = time.monotonic() + (cut_off - time.time() + 30) * get_time_scale()
        while time.time() < cut_off + 1 and time.monotonic() < deadline:
            self.assertEqual(_read(SHORT_BASE_URL, access).status_code, 401)
            time.sleep(POLL_INTERVAL)
        self.assertGreaterEqual(time.time(), cut_off + 1)

        _pair(SHORT_BASE_URL)  # a login sweeps
        refused = _read(SHORT_BASE_URL, access)
        self.assertEqual(refused.status_code, 401,
                         "the foreign record was cut at this gateway's refresh lifetime")
        self.assertIn('revoked', refused.text)
        self.assertEqual(_read(SHORT_BASE_URL, unrevoked_access).status_code, 200)


class TestDownloadsEndWithTheirToken(unittest.TestCase):
    """On the capped gateway a download ends at its next chunk once its token is revoked."""

    @classmethod
    def setUpClass(cls):
        _wait_until_answering(CAPPED_BASE_URL)
        token = _pair(CAPPED_BASE_URL, DOWNLOADER)['access_token']
        app_url = f'{CAPPED_BASE_URL}/apps/{DEMO_APP}'
        deadline = time.monotonic() + 60 * get_time_scale()
        while requests.get(app_url, headers=_bearer(token), timeout=15).status_code != 200:
            assert time.monotonic() < deadline, f'{DEMO_APP} was not discovered'
            time.sleep(POLL_INTERVAL)
        cls.payload = os.urandom(DOWNLOAD_BYTES)
        upload = requests.post(
            f'{app_url}/bulk-data/{BULK_CATEGORY}', headers=_bearer(token),
            files={'file': ('payload.bin', cls.payload, 'application/octet-stream')}, timeout=120)
        assert upload.status_code == 201, f'upload: {upload.status_code} {upload.text}'
        cls.url = f'{app_url}/bulk-data/{BULK_CATEGORY}/{upload.json()["id"]}'

    def test_01_revocation_ends_a_download_and_frees_its_worker(self):
        gone, stays = _pair(CAPPED_BASE_URL, DOWNLOADER), _pair(CAPPED_BASE_URL, DOWNLOADER)
        download = requests.get(self.url, headers=_bearer(gone['access_token']), stream=True,
                                timeout=(5, 60))
        self.addCleanup(download.close)
        self.assertEqual(download.status_code, 200)
        chunks = download.iter_content(chunk_size=64 * 1024)
        received = len(next(chunks))

        revoked_at = time.monotonic()
        self.assertEqual(
            _revoke(CAPPED_BASE_URL, gone['refresh_token'], DOWNLOADER).status_code, 200)
        # The download holds one worker and this stream the other: only the download's can
        # answer /health.
        _open_stream(self, f'{CAPPED_BASE_URL}/faults/stream', stays['access_token'])

        # Reading resumes here. The token is checked before each write, so the download ends
        # once the client has the bytes in the socket buffers and the one chunk past its check.
        resumed_at = time.monotonic()
        try:
            for chunk in chunks:
                received += len(chunk)
        except requests.RequestException:
            pass
        ended = time.monotonic() - resumed_at
        self.assertLess(received, DOWNLOAD_BYTES,
                        'the download completed after its token was revoked')

        asked = time.monotonic()
        health = requests.get(f'{CAPPED_BASE_URL}/health', headers=_bearer(stays['access_token']),
                              timeout=30)
        answered = time.monotonic() - asked
        print(f'download: revoked, then reading resumed {resumed_at - revoked_at:.3f}s later and '
              f'ended {ended:.3f}s after that with {received} of {DOWNLOAD_BYTES} bytes; '
              f'/health answered in {answered:.3f}s (bound {STREAM_MARGIN_SEC:.1f}s)')
        self.assertEqual(health.status_code, 200, health.text)
        self.assertLessEqual(ended, STREAM_MARGIN_SEC)
        self.assertLessEqual(answered, STREAM_MARGIN_SEC)

    def test_02_a_download_with_a_valid_token_completes(self):
        token = _pair(CAPPED_BASE_URL, DOWNLOADER)['access_token']
        resp = requests.get(self.url, headers=_bearer(token), timeout=120)
        self.assertEqual(resp.status_code, 200)
        self.assertEqual(len(resp.content), DOWNLOAD_BYTES)
        self.assertEqual(hashlib.sha256(resp.content).digest(),
                         hashlib.sha256(self.payload).digest())


@launch_testing.post_shutdown_test()
class TestTokenSemanticsShutdown(unittest.TestCase):
    """Every launched gateway exits cleanly."""

    def test_exit_codes(self, proc_info, long_lived, short_lived, capped, expiring, brief):
        for proc in (long_lived, short_lived, capped, expiring, brief):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc)
