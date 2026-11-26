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

"""TLS protocol floor, mutual TLS and the plain-HTTP warning.

Every case starts a real gateway_node subprocess and talks to it over its real
protocol. A refusal is always paired with an acceptance on the same gateway or
the same class. TLS 1.1 is not tested: the OpenSSL of the host does not
offer it at its default security level.

@verifies REQ_INTEROP_086
"""

import hashlib
import http.client
import json
import os
import re
import select
import shutil
import signal
import ssl
import subprocess
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_testing
import launch_testing.actions
import pytest

from ros2_medkit_test_utils.constants import API_BASE_PATH, get_test_port
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

READY_LINE = 'Medkit Gateway ready on'
HEALTH = f'{API_BASE_PATH}/health'
APPS = f'{API_BASE_PATH}/apps'
JWT_SECRET = 'tls_floor_mtls_integration_secret_key_0123456789'
CLIENT_ID = 'diagbox'
CLIENT_SECRET = 'diagbox_client_secret'
PLAIN_WARNING = 'travel in clear text'

_DIR = tempfile.mkdtemp(prefix='medkit_tls_floor_mtls_')
RESIDENT_PORT = get_test_port(9)
_BAND = [get_test_port(offset) for offset in range(9)]
_NEXT = [0]


def _port():
    """Return the next port of the test band; every gateway of the file takes its own."""
    port = _BAND[_NEXT[0] % len(_BAND)]
    _NEXT[0] += 1
    return port


def _openssl(*args):
    subprocess.run(['openssl', *args], check=True, capture_output=True)


def _ca(name):
    key, crt = os.path.join(_DIR, f'{name}-key.pem'), os.path.join(_DIR, f'{name}.pem')
    _openssl('req', '-x509', '-newkey', 'rsa:2048', '-nodes', '-keyout', key, '-out', crt,
             '-days', '1', '-subj', f'/CN=medkit-{name}',
             '-addext', 'basicConstraints=critical,CA:TRUE',
             '-addext', 'keyUsage=critical,keyCertSign,cRLSign')
    return key, crt


def _leaf(name, ca_key, ca_crt, cn, san=False, client=False):
    key = os.path.join(_DIR, f'{name}-key.pem')
    csr = os.path.join(_DIR, f'{name}.csr')
    crt = os.path.join(_DIR, f'{name}.pem')
    ext = os.path.join(_DIR, f'{name}.ext')
    with open(ext, 'w', encoding='utf-8') as handle:
        handle.write('basicConstraints=CA:FALSE\nkeyUsage=digitalSignature,keyEncipherment\n')
        usage = 'clientAuth' if client else 'serverAuth'
        handle.write(f'extendedKeyUsage={usage}\n')
        if san:
            handle.write('subjectAltName=DNS:localhost,IP:127.0.0.1\n')
    _openssl('req', '-newkey', 'rsa:2048', '-nodes', '-keyout', key, '-out', csr,
             '-subj', f'/CN={cn}')
    _openssl('x509', '-req', '-in', csr, '-CA', ca_crt, '-CAkey', ca_key, '-CAcreateserial',
             '-out', crt, '-days', '1', '-extfile', ext)
    os.chmod(key, 0o600)
    return key, crt


CA_KEY, CA_CRT = _ca('ca')
SRV_KEY, SRV_CRT = _leaf('server', CA_KEY, CA_CRT, 'localhost', san=True)
CLI_KEY, CLI_CRT = _leaf('client', CA_KEY, CA_CRT, 'medkit-client', client=True)
OTHER_KEY, OTHER_CRT = _ca('other')
OTHER_CLI_KEY, OTHER_CLI_CRT = _leaf(
    'other-client', OTHER_KEY, OTHER_CRT, 'other-client', client=True)
EMPTY_CA = os.path.join(_DIR, 'empty-ca.pem')
open(EMPTY_CA, 'w', encoding='utf-8').close()


@pytest.mark.launch_test
def generate_test_description():
    """One loopback gateway keeps the launch alive; every case starts its own subprocess."""
    resident = create_gateway_node(
        port=RESIDENT_PORT, name='tls_resident', extra_params={'server.host': '127.0.0.1'})
    return launch.LaunchDescription([resident, launch_testing.actions.ReadyToTest()]), {}


def _tls(min_version='1.2', ca_file=None, require=None):
    params = {'server.tls.enabled': True, 'server.tls.cert_file': SRV_CRT,
              'server.tls.key_file': SRV_KEY, 'server.tls.min_version': min_version}
    if ca_file is not None:
        params['server.tls.ca_file'] = ca_file
    if require is not None:
        params['server.tls.require_client_cert'] = require
    return params


def _auth_params():
    return {'auth.enabled': True, 'auth.require_auth_for': 'all',
            'auth.issuer': 'ros2_medkit_gateway',
            'auth.jwt_secret': JWT_SECRET, 'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin']}


class _Gateway:
    """A gateway_node subprocess. `rc` is its exit code once it ended before the ready line."""

    def __init__(self, params, host='127.0.0.1', timeout=60, env=None, command=None, port=None):
        self.port = port or _port()
        self.node = f'tls_gw_{self.port}'
        self.command = command or [
            'ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
            '-r', f'__node:={self.node}',
            '-p', f'server.port:={self.port}', '-p', f'server.host:="{host}"',
            '-p', 'refresh_interval_ms:=1000']
        for key, value in params.items():
            self.command += ['-p', f'{key}:={json.dumps(value)}']
        self.env = {k: v for k, v in os.environ.items() if not k.startswith('MEDKIT_')}
        self.env.update(env or {})
        self.timeout = timeout
        self.rc = None
        self.ready = False
        self._chunks = []
        self._process = None

    @property
    def output(self):
        return b''.join(self._chunks).decode(errors='replace')

    def __enter__(self):
        self._process = subprocess.Popen(
            self.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=self.env, start_new_session=True)
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            readable, _, _ = select.select([self._process.stdout], [], [], 0.5)
            if not readable:
                continue
            chunk = os.read(self._process.stdout.fileno(), 65536)
            if not chunk:
                self._process.wait(timeout=15)
                self.rc = self._process.returncode
                break
            self._chunks.append(chunk)
            if READY_LINE.encode() in b''.join(self._chunks[-2:]):
                self.ready = True
                break
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._process.poll() is None:
            os.killpg(os.getpgid(self._process.pid), signal.SIGTERM)
            try:
                self._process.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self._process.pid), signal.SIGKILL)
        rest, _ = self._process.communicate()
        if rest:
            self._chunks.append(rest)
        return False


def _context(max_version=None, cert=None, ca=CA_CRT):
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ctx.load_verify_locations(ca)
    if max_version is not None:
        ctx.maximum_version = max_version
    if cert is not None:
        ctx.load_cert_chain(*cert)
    return ctx


def _get(port, ctx, path=HEALTH, headers=None):
    """Return (status, version) of one HTTPS request; raise when it fails."""
    conn = http.client.HTTPSConnection('localhost', port, context=ctx, timeout=20)
    try:
        conn.request('GET', path, headers=headers or {})
        resp = conn.getresponse()
        resp.read()
        return resp.status, conn.sock.version()
    finally:
        conn.close()


class _SessionConnection(http.client.HTTPSConnection):
    """An HTTPS connection that offers `session` for resumption."""

    def __init__(self, port, ctx, session):
        super().__init__('localhost', port, context=ctx, timeout=20)
        self._offered = session

    def connect(self):
        http.client.HTTPConnection.connect(self)
        self.sock = self._context.wrap_socket(self.sock, server_hostname=self.host,
                                              session=self._offered)


def _session_get(port, ctx, session=None):
    """Return (status, session, session_reused) of one GET on a new connection."""
    conn = _SessionConnection(port, ctx, session)
    try:
        conn.request('GET', HEALTH)
        resp = conn.getresponse()
        resp.read()
        return resp.status, conn.sock.session, conn.sock.session_reused
    finally:
        conn.close()


_REFUSED = (ssl.SSLError, ConnectionError, http.client.HTTPException, OSError)


class _Base(unittest.TestCase):

    def assert_refused(self, port, ctx, **kwargs):
        with self.assertRaises(_REFUSED):
            _get(port, ctx, **kwargs)

    def assert_started(self, gw):
        self.assertTrue(gw.ready, f'not started: {gw.output[-800:]}')
        self.assertIsNone(gw.rc)


class TestFloor(_Base):

    def test_floor_13_refuses_tls12_and_serves_tls13(self):
        with _Gateway(_tls('1.3')) as gw:
            self.assert_started(gw)
            self.assert_refused(gw.port, _context(ssl.TLSVersion.TLSv1_2))
            status, version = _get(gw.port, _context())
            self.assertEqual((status, version), (200, 'TLSv1.3'))
            self.assertIn('minimum protocol TLS 1.3', gw.output)

    def test_floor_12_serves_tls12(self):
        with _Gateway(_tls('1.2')) as gw:
            self.assert_started(gw)
            status, version = _get(gw.port, _context(ssl.TLSVersion.TLSv1_2))
            self.assertEqual((status, version), (200, 'TLSv1.2'))
            self.assertIn('minimum protocol TLS 1.2', gw.output)

    def test_a_floor_other_than_12_or_13_stops_the_start(self):
        for value in ('1.1', 'abc'):
            with self.subTest(value=value), _Gateway(_tls(value)) as gw:
                self.assertIsNotNone(gw.rc, gw.output[-800:])
                self.assertNotEqual(gw.rc, 0)
                self.assertIn('server.tls.min_version', gw.output)
                self.assertFalse(gw.ready)
        for value in ('1.2', '1.3'):
            with self.subTest(value=value), _Gateway(_tls(value)) as gw:
                self.assert_started(gw)


class TestMutualTls(_Base):

    def test_a_client_certificate_is_required_and_checked_against_the_ca(self):
        with _Gateway(_tls(ca_file=CA_CRT, require=True)) as gw:
            self.assert_started(gw)
            self.assertIn('client certificates required', gw.output)
            status, _ = _get(gw.port, _context(cert=(CLI_CRT, CLI_KEY)))
            self.assertEqual(status, 200)
            self.assert_refused(gw.port, _context())
            self.assert_refused(gw.port, _context(cert=(OTHER_CLI_CRT, OTHER_CLI_KEY)))

    def test_a_session_made_with_a_client_certificate_resumes(self):
        with _Gateway(_tls(ca_file=CA_CRT, require=True)) as gw:
            self.assert_started(gw)
            for version in (ssl.TLSVersion.TLSv1_2, ssl.TLSVersion.TLSv1_3):
                with self.subTest(version=version.name):
                    ctx = _context(cert=(CLI_CRT, CLI_KEY))
                    ctx.minimum_version = version
                    ctx.maximum_version = version
                    status, session, reused = _session_get(gw.port, ctx)
                    self.assertEqual((status, reused), (200, False))
                    self.assertIsNotNone(session)
                    status, _, reused = _session_get(gw.port, ctx, session)
                    self.assertEqual((status, reused), (200, True))

    def test_the_ca_file_is_required_and_must_be_readable(self):
        for name, ca_file in (('empty', ''), ('missing', os.path.join(_DIR, 'nope.pem')),
                              ('not a bundle', EMPTY_CA)):
            with self.subTest(case=name), _Gateway(_tls(ca_file=ca_file, require=True)) as gw:
                self.assertIsNotNone(gw.rc, gw.output[-800:])
                self.assertNotEqual(gw.rc, 0)
                self.assertIn('server.tls.ca_file', gw.output)
                self.assertFalse(gw.ready)
        with _Gateway(_tls(ca_file=CA_CRT, require=True)) as gw:
            self.assert_started(gw)

    def test_without_the_switch_the_ca_file_changes_nothing_for_clients(self):
        with _Gateway(_tls(ca_file=CA_CRT)) as gw:
            self.assert_started(gw)
            self.assertIn('client certificates not required', gw.output)
            status, _ = _get(gw.port, _context())
            self.assertEqual(status, 200)

    def test_a_certificate_does_not_replace_the_token(self):
        params = {**_tls(ca_file=CA_CRT, require=True), **_auth_params()}
        with _Gateway(params) as gw:
            self.assert_started(gw)
            ctx = _context(cert=(CLI_CRT, CLI_KEY))
            status, _ = _get(gw.port, ctx, path=APPS)
            self.assertEqual(status, 401)
            conn = http.client.HTTPSConnection('localhost', gw.port, context=ctx, timeout=20)
            conn.request('POST', f'{API_BASE_PATH}/auth/authorize',
                         body=json.dumps({'grant_type': 'client_credentials',
                                          'client_id': CLIENT_ID,
                                          'client_secret': CLIENT_SECRET}),
                         headers={'Content-Type': 'application/json'})
            resp = conn.getresponse()
            self.assertEqual(resp.status, 200)
            token = json.loads(resp.read())['access_token']
            conn.close()
            status, _ = _get(gw.port, ctx, path=APPS, headers={'Authorization': f'Bearer {token}'})
            self.assertEqual(status, 200)

    def test_the_dev_cert_script_makes_a_server_and_a_client_pair_mutual_tls_accepts(self):
        script = os.path.join(get_package_share_directory('ros2_medkit_gateway'),
                              'scripts', 'generate_dev_certs.sh')
        out = os.path.join(_DIR, 'script-certs')
        subprocess.run([script, out], check=True, capture_output=True)
        params = _tls(ca_file=os.path.join(out, 'ca.pem'), require=True)
        params['server.tls.cert_file'] = os.path.join(out, 'cert.pem')
        params['server.tls.key_file'] = os.path.join(out, 'key.pem')
        with _Gateway(params) as gw:
            self.assert_started(gw)
            ca = os.path.join(out, 'ca.pem')
            ctx = _context(ca=ca, cert=(os.path.join(out, 'client.pem'),
                                        os.path.join(out, 'client-key.pem')))
            status, _ = _get(gw.port, ctx)
            self.assertEqual(status, 200)
            self.assert_refused(gw.port, _context(ca=ca))

    def test_require_client_cert_without_tls_stops_the_start(self):
        with _Gateway({'server.tls.require_client_cert': True}) as gw:
            self.assertIsNotNone(gw.rc, gw.output[-800:])
            self.assertNotEqual(gw.rc, 0)
            self.assertIn('server.tls.require_client_cert', gw.output)
            self.assertIn('server.tls.enabled', gw.output)
            self.assertFalse(gw.ready)
        with _Gateway({'server.tls.require_client_cert': False}) as gw:
            self.assert_started(gw)

    def test_a_ca_file_without_a_ca_certificate_stops_the_start(self):
        with _Gateway(_tls(ca_file=SRV_CRT, require=True)) as gw:
            self.assertIsNotNone(gw.rc, gw.output[-800:])
            self.assertNotEqual(gw.rc, 0)
            self.assertIn('server.tls.ca_file', gw.output)
            self.assertFalse(gw.ready)
        with _Gateway(_tls(ca_file=CA_CRT, require=True)) as gw:
            self.assert_started(gw)


def _param(gw, verb, name, *value, timeout=30):
    """Run `ros2 param <verb>` without the daemon; retry while discovery has not found the node."""
    deadline = time.monotonic() + timeout
    while True:
        proc = subprocess.run(['ros2', 'param', verb, '--no-daemon', f'/{gw.node}', name, *value],
                              capture_output=True, text=True, timeout=60)
        out = proc.stdout + proc.stderr
        if 'Node not found' not in out or time.monotonic() >= deadline:
            return out
        time.sleep(1)


class TestRuntimeSet(_Base):

    def test_server_tls_keys_cannot_be_set_at_runtime(self):
        with _Gateway(_tls('1.2', ca_file=CA_CRT)) as gw:
            self.assert_started(gw)
            out = _param(gw, 'set', 'server.tls.require_client_cert', 'true')
            self.assertIn('read at start', out)
            self.assertIn('False', _param(gw, 'get', 'server.tls.require_client_cert'))
            out = _param(gw, 'set', 'server.tls.min_version', '"1.3"')
            self.assertIn('read at start', out)
            self.assertIn('1.2', _param(gw, 'get', 'server.tls.min_version'))
            status, version = _get(gw.port, _context(ssl.TLSVersion.TLSv1_2))
            self.assertEqual((status, version), (200, 'TLSv1.2'))


class TestLaunchMutualTls(_Base):

    def test_the_launch_makes_a_client_certificate_that_the_gateway_accepts(self):
        cert_dir = os.path.join(_DIR, 'launch-certs')
        port = _port()
        command = ['ros2', 'launch', 'ros2_medkit_gateway', 'gateway_https.launch.py',
                   'require_client_cert:=true', f'server_port:={port}', f'cert_dir:={cert_dir}']
        with _Gateway({}, command=command, port=port) as gw:
            self.assert_started(gw)
            match = re.search(r'--cert (\S+) --key (\S+)', gw.output)
            self.assertIsNotNone(match, gw.output[-1500:])
            cert, key = match.groups()
            ca = os.path.join(cert_dir, 'ca.pem')
            status, _ = _get(port, _context(ca=ca, cert=(cert, key)))
            self.assertEqual(status, 200)
            self.assert_refused(port, _context(ca=ca))

    def _launch(self, cert_dir, *args):
        port = _port()
        command = ['ros2', 'launch', 'ros2_medkit_gateway', 'gateway_https.launch.py',
                   f'server_port:={port}', f'cert_dir:={cert_dir}', *args]
        return _Gateway({}, command=command, port=port), port

    def _config(self, name, tls_lines, node='ros2_medkit_gateway'):
        path = os.path.join(_DIR, name)
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(f'{node}:\n  ros__parameters:\n    server:\n      tls:\n')
            handle.write(''.join(f'        {line}\n' for line in tls_lines))
        return path

    def _assert_printed_pair_is_accepted(self, gw, port, cert_dir):
        match = re.search(r'--cert (\S+) --key (\S+)', gw.output)
        self.assertIsNotNone(match, gw.output[-1500:])
        ca = os.path.join(cert_dir, 'ca.pem')
        status, _ = _get(port, _context(ca=ca, cert=match.groups()))
        self.assertEqual(status, 200)
        self.assert_refused(port, _context(ca=ca))
        self.assertNotIn('curl -k', gw.output)

    def test_a_config_file_that_turns_mtls_on_gets_a_client_certificate(self):
        cert_dir = os.path.join(_DIR, 'launch-certs-config')
        config = self._config('mtls-on.yaml', ['require_client_cert: true'])
        gw, port = self._launch(cert_dir, f'config_file:={config}')
        with gw:
            self.assert_started(gw)
            self._assert_printed_pair_is_accepted(gw, port, cert_dir)

    def test_the_launch_keeps_the_ca_file_a_config_file_sets(self):
        cert_dir = os.path.join(_DIR, 'launch-certs-own-ca')
        config = self._config(
            'mtls-own-ca.yaml', ['require_client_cert: true', f'ca_file: "{CA_CRT}"'], node='/**')
        gw, port = self._launch(cert_dir, f'config_file:={config}')
        with gw:
            self.assert_started(gw)
            dev_ca = os.path.join(cert_dir, 'ca.pem')
            status, _ = _get(port, _context(ca=dev_ca, cert=(CLI_CRT, CLI_KEY)))
            self.assertEqual(status, 200)
            self.assert_refused(port, _context(ca=dev_ca, cert=(OTHER_CLI_CRT, OTHER_CLI_KEY)))
            self.assertIn(f'signed by {CA_CRT}', gw.output)
            self.assertFalse(os.path.exists(os.path.join(cert_dir, 'client.pem')))

    def _cert_dir(self, name, files):
        """Return a new cert_dir holding `files`, a {name in cert_dir: source} map."""
        cert_dir = os.path.join(_DIR, name)
        os.makedirs(cert_dir)
        for target, source in files.items():
            shutil.copy(source, os.path.join(cert_dir, target))
        return cert_dir

    def test_a_configured_ca_in_cert_dir_is_never_written_over(self):
        """Making the dev server pair would write cert_dir/ca.pem, the configured CA."""
        cert_dir = self._cert_dir('launch-certs-configured-ca-no-pair', {'ca.pem': OTHER_CRT})
        ca = os.path.join(cert_dir, 'ca.pem')
        with open(ca, 'rb') as handle:
            digest = hashlib.sha256(handle.read()).hexdigest()
        config = self._config('mtls-ca-in-cert-dir.yaml',
                              ['require_client_cert: true', f'ca_file: "{ca}"'])
        gw, _ = self._launch(cert_dir, f'config_file:={config}')
        with gw:
            self.assertFalse(gw.ready, gw.output[-1500:])
            self.assertIsNotNone(gw.rc, gw.output[-1500:])
            self.assertNotEqual(gw.rc, 0)
            self.assertIn(ca, gw.output)
        with open(ca, 'rb') as handle:
            self.assertEqual(hashlib.sha256(handle.read()).hexdigest(), digest)
        self.assertFalse(os.path.exists(os.path.join(cert_dir, 'cert.pem')))

    def test_a_configured_ca_that_generation_would_write_stops_the_launch(self):
        """The same file as one the launch writes or removes, by path or by a hard link."""
        at_cert_pem = self._cert_dir('launch-certs-ca-at-cert-pem', {'cert.pem': OTHER_CRT})
        linked = self._cert_dir('launch-certs-ca-hard-linked', {'ca.pem': OTHER_CRT})
        link = os.path.join(_DIR, 'hard-linked-ca.pem')
        os.link(os.path.join(linked, 'ca.pem'), link)
        cases = {
            'cert_dir/cert.pem, no key.pem': (at_cert_pem, os.path.join(at_cert_pem, 'cert.pem')),
            'a hard link to cert_dir/ca.pem': (linked, link),
        }
        for name, (cert_dir, ca) in cases.items():
            with self.subTest(case=name):
                listing = sorted(os.listdir(cert_dir))
                with open(ca, 'rb') as handle:
                    digest = hashlib.sha256(handle.read()).hexdigest()
                config = self._config(f'mtls-{os.path.basename(cert_dir)}.yaml',
                                      ['require_client_cert: true', f'ca_file: "{ca}"'])
                gw, _ = self._launch(cert_dir, f'config_file:={config}')
                with gw:
                    self.assertFalse(gw.ready, gw.output[-1500:])
                    self.assertIsNotNone(gw.rc, gw.output[-1500:])
                    self.assertNotEqual(gw.rc, 0)
                    self.assertIn(ca, gw.output)
                with open(ca, 'rb') as handle:
                    self.assertEqual(hashlib.sha256(handle.read()).hexdigest(), digest)
                self.assertEqual(sorted(os.listdir(cert_dir)), listing)

    def test_a_configured_ca_in_cert_dir_signs_no_client_pair(self):
        """A CA the params files name is external, even at cert_dir/ca.pem and without its key."""
        cert_dir = self._cert_dir('launch-certs-configured-ca-with-pair', {
            'ca.pem': OTHER_CRT, 'cert.pem': SRV_CRT, 'key.pem': SRV_KEY})
        ca = os.path.join(cert_dir, 'ca.pem')
        config = self._config('mtls-ca-in-cert-dir-pair.yaml',
                              ['require_client_cert: true', f'ca_file: "{ca}"'])
        gw, port = self._launch(cert_dir, f'config_file:={config}')
        with gw:
            self.assert_started(gw)
            self.assertIn(f'signed by {ca}', gw.output)
            self.assertFalse(os.path.exists(os.path.join(cert_dir, 'client.pem')))
            status, _ = _get(port, _context(cert=(OTHER_CLI_CRT, OTHER_CLI_KEY)))
            self.assertEqual(status, 200)
            self.assert_refused(port, _context(cert=(CLI_CRT, CLI_KEY)))

    def test_a_client_key_that_is_not_the_certificates_is_made_again(self):
        cert_dir = self._cert_dir('launch-certs-foreign-client-key', {
            'ca.pem': CA_CRT, 'ca_key.pem': CA_KEY, 'cert.pem': SRV_CRT, 'key.pem': SRV_KEY,
            'client.pem': CLI_CRT, 'client-key.pem': OTHER_CLI_KEY})
        gw, port = self._launch(cert_dir, 'require_client_cert:=true')
        with gw:
            self.assert_started(gw)
            self._assert_printed_pair_is_accepted(gw, port, cert_dir)

    def test_a_client_pair_from_another_ca_is_made_again(self):
        fresh = os.path.join(_DIR, 'launch-certs-stale-fresh-ca')
        kept = os.path.join(_DIR, 'launch-certs-stale-kept-ca')
        os.makedirs(fresh)
        os.makedirs(kept)
        # `kept` holds a CA the launch reuses; `fresh` gets a new one.
        for src, name in ((CA_CRT, 'ca.pem'), (CA_KEY, 'ca_key.pem'), (SRV_CRT, 'cert.pem'),
                          (SRV_KEY, 'key.pem')):
            shutil.copy(src, os.path.join(kept, name))
        for cert_dir in (fresh, kept):
            with self.subTest(cert_dir=os.path.basename(cert_dir)):
                shutil.copy(OTHER_CLI_CRT, os.path.join(cert_dir, 'client.pem'))
                shutil.copy(OTHER_CLI_KEY, os.path.join(cert_dir, 'client-key.pem'))
                gw, port = self._launch(cert_dir, 'require_client_cert:=true')
                with gw:
                    self.assert_started(gw)
                    self._assert_printed_pair_is_accepted(gw, port, cert_dir)

    def test_without_the_switch_the_launch_prints_no_client_certificate(self):
        cert_dir = os.path.join(_DIR, 'launch-certs-plain')
        port = _port()
        command = ['ros2', 'launch', 'ros2_medkit_gateway', 'gateway_https.launch.py',
                   f'server_port:={port}', f'cert_dir:={cert_dir}']
        with _Gateway({}, command=command, port=port) as gw:
            self.assert_started(gw)
            self.assertNotIn('--cert', gw.output)
            status, _ = _get(port, _context(ca=os.path.join(cert_dir, 'ca.pem')))
            self.assertEqual(status, 200)


class TestPlainHttpWarning(_Base):

    def test_warns_only_for_secrets_over_plain_http_on_a_network_address(self):
        write = {**_auth_params(), 'auth.require_auth_for': 'write'}
        none = {**_auth_params(), 'auth.require_auth_for': 'none',
                'server.allow_unauthenticated_remote': True}
        env = {'MEDKIT_JWT_SECRET': JWT_SECRET,
               'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin'}
        cases = (
            ('auth on, 0.0.0.0, no TLS', _auth_params(), '0.0.0.0', None, True),
            ('require_auth_for write', write, '0.0.0.0', None, True),
            ('authentication from the environment only', {}, '0.0.0.0', env, True),
            ('require_auth_for none', none, '0.0.0.0', None, False),
            ('auth on, 0.0.0.0, TLS on', {**_auth_params(), **_tls()}, '0.0.0.0', None, False),
            ('auth on, loopback, no TLS', _auth_params(), '127.0.0.1', None, False),
            ('auth off, 0.0.0.0, no TLS',
             {'auth.enabled': False, 'server.allow_unauthenticated_remote': True},
             '0.0.0.0', None, False),
        )
        for name, params, host, env_extra, warns in cases:
            with self.subTest(case=name), _Gateway(params, host=host, env=env_extra) as gw:
                self.assert_started(gw)
                self.assertEqual(gw.output.count(PLAIN_WARNING), 1 if warns else 0,
                                 gw.output[-1500:])
                if warns:
                    self.assertIn('0.0.0.0', gw.output.split(PLAIN_WARNING)[0].splitlines()[-1])
                    self.assertIn('server.tls', gw.output)


def tearDownModule():
    shutil.rmtree(_DIR, ignore_errors=True)
