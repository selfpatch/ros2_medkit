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

"""The gateway node reads the three auth environment variables itself.

Where the rule lives decides which deployments it covers. A container
entrypoint can only apply it to what the entrypoint runs, and a launch file
only to launches that include it - so `docker run <image> ros2 launch ...`,
`docker run <image> bash`, and a plain `ros2 run ros2_medkit_gateway
gateway_node` each ran on whatever rule reached them. The node reads the
variables where it reads its parameters, which covers every path by
construction, and this is the test of that: the gateways below are started with
no entrypoint and no launch file, only an environment.

Each case pairs the environment against a params file that says the OPPOSITE,
because a rule that only agrees with the file proves nothing about precedence.

THE RULES

C1  MEDKIT_JWT_SECRET, against a file with `auth.enabled: false`: the gateway
    refuses an anonymous read.
C2  Every route, writes and reads alike. `require_auth_for` has to become
    "all": at
    the file's "write" an anonymous GET answers 200 with authentication
    switched on, and the entity tree, the fault history and the operation list
    are the disclosure.
C3  MEDKIT_CLIENTS reaches the gateway: the credential in it obtains a token
    from POST /auth/authorize, and that token reads. Without this a gateway
    closed to everyone including its operator would pass C1 and C2.
C4  MEDKIT_AUTH_DISABLED=1 on top of the same secret, against a file with
    `auth.enabled: true`: the gateway answers an anonymous read.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import os
import select
import shutil
import signal
import subprocess
import tempfile
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
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

CLOSED_PORT = get_test_port(0)
DISABLED_PORT = get_test_port(1)
# For the gateways below that are expected to refuse to start. They never bind
# it; a port of their own keeps a case that unexpectedly DOES come up from
# colliding with the two gateways this file launches.
REFUSE_PORT = get_test_port(2)
FILE_CLOSED_PORT = get_test_port(3)

# How long `ros2 param` may spend discovering a gateway over DDS before a call
# against it is treated as a real failure.
PARAM_DISCOVERY_TIMEOUT = 30
CLOSED_BASE_URL = f'http://127.0.0.1:{CLOSED_PORT}{API_BASE_PATH}'
DISABLED_BASE_URL = f'http://127.0.0.1:{DISABLED_PORT}{API_BASE_PATH}'
FILE_CLOSED_BASE_URL = f'http://127.0.0.1:{FILE_CLOSED_PORT}{API_BASE_PATH}'

# Never valid as a credential - it is the secret half of a two-field entry the
# gateway must refuse - and distinctive enough to grep a whole log for.
MALFORMED_CLIENT_SECRET = 's3cret_that_must_not_be_logged'
# The secret of an entry written `id:role:secret`, which parses as a role
# nobody has; a warning that quoted the role field would print it.
SWAPPED_FILE_SECRET = 's3cret_swapped_in_the_file_zz'
SWAPPED_ENV_SECRET = 's3cret_swapped_in_the_env_zz'
# What this gateway would present to its peers; served through the parameter
# services as a sentinel and never as itself.
PEER_AUTH_HEADER = 'Bearer zz_peer_token_that_must_not_be_served'
FILE_JWT_SECRET = 'a_file_supplied_secret_of_at_least_32_characters'

JWT_SECRET = 'an_environment_supplied_secret_of_at_least_32_chars'
CLIENT_ID = 'envclient'
CLIENT_SECRET = 'env_client_secret'

# Written once at import time, so both gateways load a real file from disk
# - the file, and not inline parameters, is the thing the environment has to
# outrank.
_PARAMS_DIR = tempfile.mkdtemp(prefix='medkit_env_auth_')

# The node names the launch below gives its gateways. A params file is keyed by
# the node it is for: rcl applies an entry only to the node whose name heads
# it, so a file written under any other name is read and applies to nothing,
# and a test built on it measures the defaults.
ENV_CLOSED_NODE = 'gateway_env_closed'
ENV_DISABLED_NODE = 'gateway_env_disabled'
FILE_CLOSED_NODE = 'gateway_file_closed'


# A key the environment never touches and the inline parameters never set, with
# a different value per file. Every other key in these files is overwritten by
# the environment or equals the default, so this is the one that shows the
# file reached its gateway at all.
CLOSED_FILE_TOKEN_EXPIRY = 1234
DISABLED_FILE_TOKEN_EXPIRY = 2345


def _write_params(name, node_name, auth_enabled, token_expiry):
    """Write a params file stating a posture, so the environment has one to override."""
    path = os.path.join(_PARAMS_DIR, name)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(
            f'{node_name}:\n'
            '  ros__parameters:\n'
            '    auth:\n'
            f'      enabled: {"true" if auth_enabled else "false"}\n'
            '      require_auth_for: "write"\n'
            '      jwt_secret: "a_file_supplied_secret_of_at_least_32_characters"\n'
            f'      clients: ["{CLIENT_ID}:a_file_supplied_client_secret:viewer"]\n'
            f'      token_expiry_seconds: {token_expiry}\n'
        )
    return path


AUTH_OFF_PARAMS = _write_params('auth-off.yaml', ENV_CLOSED_NODE, auth_enabled=False,
                                token_expiry=CLOSED_FILE_TOKEN_EXPIRY)
AUTH_ON_PARAMS = _write_params('auth-on.yaml', ENV_DISABLED_NODE, auth_enabled=True,
                               token_expiry=DISABLED_FILE_TOKEN_EXPIRY)


def _write_file_closed_params():
    """Write a file that closes the gateway on its own, with one bad client.

    Everything here comes from the file and nothing from the environment, which
    is the case the sentinel and the malformed-entry warning are about.
    """
    path = os.path.join(_PARAMS_DIR, 'file-closed.yaml')
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(
            f'{FILE_CLOSED_NODE}:\n'
            '  ros__parameters:\n'
            '    auth:\n'
            '      enabled: true\n'
            '      require_auth_for: "all"\n'
            f'      jwt_secret: "{FILE_JWT_SECRET}"\n'
            '      clients:\n'
            f'        - "{CLIENT_ID}:{CLIENT_SECRET}:admin"\n'
            f'        - "twofield:{MALFORMED_CLIENT_SECRET}"\n'
            f'        - "swapped:admin:{SWAPPED_FILE_SECRET}"\n'
        )
    return path


FILE_CLOSED_PARAMS = _write_file_closed_params()


@pytest.mark.launch_test
def generate_test_description():
    """Two gateways, differing only in what their environments say."""
    closed = create_gateway_node(
        port=CLOSED_PORT,
        name=ENV_CLOSED_NODE,
        params_file=AUTH_OFF_PARAMS,
        # `auth.public_routes: [""]` is the empty-sequence idiom, and it is
        # written here on purpose: a gateway that read the blank entry as a
        # route would refuse to start, and one that opened something on it
        # would answer /health anonymously below.
        extra_params={'server.host': '127.0.0.1', 'auth.public_routes': ['']},
        extra_env={
            'MEDKIT_JWT_SECRET': JWT_SECRET,
            'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin,'
                              f'envswapped:admin:{SWAPPED_ENV_SECRET}',
        },
    )

    disabled = create_gateway_node(
        port=DISABLED_PORT,
        name=ENV_DISABLED_NODE,
        params_file=AUTH_ON_PARAMS,
        extra_params={'server.host': '127.0.0.1',
                      'aggregation.peer_auth_header': PEER_AUTH_HEADER},
        extra_env={
            'MEDKIT_JWT_SECRET': JWT_SECRET,
            'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin',
            'MEDKIT_AUTH_DISABLED': '1',
        },
    )

    file_closed = create_gateway_node(
        port=FILE_CLOSED_PORT,
        name=FILE_CLOSED_NODE,
        params_file=FILE_CLOSED_PARAMS,
        extra_params={'server.host': '127.0.0.1'},
    )

    return launch.LaunchDescription([
        closed,
        disabled,
        file_closed,
        launch_testing.actions.ReadyToTest(),
    ]), {'closed': closed, 'disabled': disabled, 'file_closed': file_closed}


class TestEnvironmentClosesTheNode(GatewayTestCase):
    """MEDKIT_JWT_SECRET closes a gateway whose params file says auth is off."""

    BASE_URL = CLOSED_BASE_URL

    def test_00_the_file_reached_this_gateway(self):
        """The premise of every case below: the file was applied at all.

        Every auth key the file sets is overwritten by the environment, so a
        file that never reached the node would leave C1-C4 passing against the
        defaults. The expiry is the one key the environment leaves alone.
        """
        rc, output = _ros2_param('get', f'/{ENV_CLOSED_NODE}', 'auth.token_expiry_seconds')
        self.assertEqual(rc, 0, output)
        self.assertIn(
            str(CLOSED_FILE_TOKEN_EXPIRY), output,
            f'the params file did not reach {ENV_CLOSED_NODE}: {output}')

    def test_01_an_anonymous_read_is_refused(self):
        """C1. The params file says `auth.enabled: false` and is overruled."""
        resp = requests.get(f'{CLOSED_BASE_URL}/areas', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            f'MEDKIT_JWT_SECRET was in the environment and an anonymous GET /areas '
            f'answered {resp.status_code}. Body: {resp.text[:300]}'
        )

    def test_02_every_route_is_closed_not_only_the_writes(self):
        """C2. `require_auth_for` became "all", against a file saying "write".

        A read is what "closed" has to mean here: at "write" every one of these
        answers 200 with authentication switched on, which is the posture an
        operator would be told was closed.
        """
        for path in ('/areas', '/components', '/apps', '/health', '/'):
            with self.subTest(path=path):
                resp = requests.get(f'{CLOSED_BASE_URL}{path}', timeout=15)
                self.assertIn(
                    resp.status_code, (401, 403),
                    f'an anonymous GET {path} answered {resp.status_code}; '
                    f'require_auth_for is still the file\'s "write"'
                )

    def test_03_the_environment_credential_issues_a_working_token(self):
        """C3. The mirror: a gateway closed to everyone would pass C1 and C2.

        The file names the same client id with a different secret and the
        `viewer` role, so this also pins WHICH credential reached the gateway:
        the environment's secret authenticates, and the role that comes back is
        the environment's `admin`.
        """
        token = requests.post(
            f'{CLOSED_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        )
        self.assertEqual(token.status_code, 200, token.text)
        self.assertEqual(
            token.json().get('scope'), 'admin',
            f"the token came back with the file's role, so auth.clients was not "
            f'replaced: {token.text[:300]}'
        )

        resp = requests.get(
            f'{CLOSED_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {token.json()["access_token"]}'},
            timeout=15,
        )
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_04_the_file_credential_is_gone(self):
        """MEDKIT_CLIENTS replaces auth.clients; it does not add to it.

        Leaving the file's credentials standing under a secret they were not
        issued against is the surprise: an operator who closed a container with
        a new secret would still be handing out tokens to whoever knew the old
        file.
        """
        token = requests.post(
            f'{CLOSED_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': 'a_file_supplied_client_secret',
            },
            timeout=30,
        )
        self.assertNotEqual(
            token.status_code, 200,
            "the params file's client secret still obtains a token"
        )


class TestAuthDisabledWinsOverEverything(GatewayTestCase):
    """MEDKIT_AUTH_DISABLED=1 beats the secret and a file that asks for auth."""

    BASE_URL = DISABLED_BASE_URL

    def test_00_the_file_reached_this_gateway(self):
        """The premise: the file was applied, so C4 overrules something."""
        rc, output = _ros2_param('get', f'/{ENV_DISABLED_NODE}', 'auth.token_expiry_seconds')
        self.assertEqual(rc, 0, output)
        self.assertIn(
            str(DISABLED_FILE_TOKEN_EXPIRY), output,
            f'the params file did not reach {ENV_DISABLED_NODE}: {output}')

    def test_01_an_anonymous_read_is_answered(self):
        """C4. Both the file (`auth.enabled: true`) and MEDKIT_JWT_SECRET lose."""
        resp = requests.get(f'{DISABLED_BASE_URL}/areas', timeout=15)
        self.assertEqual(
            resp.status_code, 200,
            f'MEDKIT_AUTH_DISABLED=1 was set and an anonymous GET /areas answered '
            f'{resp.status_code}. Body: {resp.text[:300]}'
        )

    def test_02_nothing_asks_for_a_credential(self):
        resp = requests.get(f'{DISABLED_BASE_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        self.assertNotIn(
            'x-medkit-reduced', resp.json(),
            'the body was cut down on a gateway running without authentication'
        )

    def test_03_no_secret_is_served_with_authentication_off(self):
        """The file's secrets are unused here and still not served.

        A gateway running open carries the secrets of the file that asked for
        auth, and the parameter services answer anyone on the domain; a peer
        sharing that file signs with the same secret.
        """
        rc, secret = _ros2_param('get', f'/{ENV_DISABLED_NODE}', 'auth.jwt_secret')
        self.assertEqual(rc, 0, secret)
        self.assertNotIn(
            'a_file_supplied_secret_of_at_least_32_characters', secret,
            'the file signing secret is served by an open gateway')
        rc, clients = _ros2_param('get', f'/{ENV_DISABLED_NODE}', 'auth.clients')
        self.assertEqual(rc, 0, clients)
        self.assertNotIn(
            'a_file_supplied_client_secret', clients,
            'a file client secret is served by an open gateway')

    def test_04_the_peer_auth_header_is_never_served(self):
        """What this gateway presents to its peers is a bearer, and stays here."""
        rc, header = _ros2_param('get', f'/{ENV_DISABLED_NODE}', 'aggregation.peer_auth_header')
        self.assertEqual(rc, 0, header)
        self.assertNotIn(
            'zz_peer_token', header,
            'aggregation.peer_auth_header is served as itself')
        self.assertIn('<set at start>', header, header)

    def test_05_the_peer_auth_header_cannot_be_set_at_runtime(self):
        """Read once at construction, like auth.*, so a set would change nothing."""
        _, output = _ros2_param(
            'set', f'/{ENV_DISABLED_NODE}', 'aggregation.peer_auth_header', 'Bearer other')
        self.assertIn(
            'read at start', output,
            f'a runtime set of aggregation.peer_auth_header was not refused: {output}')


def _ros2_param(*args):
    """Run `ros2 param ...` against this test's domain and return (rc, output).

    Retries while the CLI reports the node missing. `ros2 param` builds its own
    participant and has to discover the gateway's parameter services over DDS,
    which is not instantaneous - the HTTP port answering says nothing about
    whether that has happened yet, so the first call in a class would otherwise
    fail with "Node not found" while the node is plainly there.
    """
    deadline = time.monotonic() + PARAM_DISCOVERY_TIMEOUT
    output = ''
    while True:
        completed = subprocess.run(
            ['ros2', 'param', *args],
            capture_output=True, text=True, timeout=30, check=False,
        )
        output = completed.stdout + completed.stderr
        if 'Node not found' not in output or time.monotonic() >= deadline:
            return completed.returncode, output
        time.sleep(1)


class TestIntrospectionAgreesWithEnforcement(GatewayTestCase):
    """`ros2 param get` answers what the gateway enforces, and refuses writes.

    The environment decides the posture, and the parameters carried whatever
    the params file said - so on a gateway refusing every request,
    `auth.enabled` read back `false`, which is the file's value and the
    opposite of the truth. An operator reading the parameters to find out how a
    container is running was told the wrong thing by the gateway itself.

    A write is refused, because the auth configuration is consumed once at
    construction: a `param set` that reported success would change nothing and
    say it had.
    """

    BASE_URL = CLOSED_BASE_URL

    def test_01_the_closed_gateway_reports_the_posture_it_serves(self):
        rc, enabled = _ros2_param('get', '/gateway_env_closed', 'auth.enabled')
        self.assertEqual(rc, 0, enabled)
        self.assertIn(
            'True', enabled,
            f'the params file says auth.enabled false and the gateway refuses '
            f'every anonymous read; introspection answered: {enabled}'
        )

        rc, level = _ros2_param('get', '/gateway_env_closed', 'auth.require_auth_for')
        self.assertEqual(rc, 0, level)
        self.assertIn(
            'all', level,
            'the environment forced require_auth_for to "all" and the '
            f'parameter still reports the value from the file: {level}'
        )

    def test_02_the_disabled_gateway_reports_the_posture_it_serves(self):
        """The other shape: a file asking for auth, an environment refusing it."""
        rc, enabled = _ros2_param('get', '/gateway_env_disabled', 'auth.enabled')
        self.assertEqual(rc, 0, enabled)
        self.assertIn(
            'False', enabled,
            'MEDKIT_AUTH_DISABLED=1 turned authentication off and the '
            f'parameter still reports true from the file: {enabled}'
        )

    def test_03_no_secret_is_readable_through_the_parameters(self):
        """A sentinel names the variable; the value stays out of reach.

        The parameter services answer anything that can reach the node, so a
        secret written back here would be readable by a caller holding no HTTP
        credential at all - which is a wider audience than the file it was
        kept out of.
        """
        rc, secret = _ros2_param('get', '/gateway_env_closed', 'auth.jwt_secret')
        self.assertEqual(rc, 0, secret)
        self.assertNotIn(JWT_SECRET, secret, 'the signing secret is readable through ros2 param')
        self.assertIn('<from MEDKIT_JWT_SECRET>', secret, secret)

        rc, clients = _ros2_param('get', '/gateway_env_closed', 'auth.clients')
        self.assertEqual(rc, 0, clients)
        self.assertNotIn(CLIENT_SECRET, clients, 'a client secret is readable through ros2 param')
        self.assertIn(CLIENT_ID, clients, f'the client id should still be visible: {clients}')
        self.assertIn('<from MEDKIT_CLIENTS>', clients, clients)

    def test_04_an_auth_parameter_cannot_be_set_at_runtime(self):
        """Refused, and the gateway goes on refusing anonymous reads."""
        _, output = _ros2_param(
            'set', '/gateway_env_closed', 'auth.enabled', 'false')
        self.assertIn(
            'read at start', output,
            f'setting auth.enabled did not report the reason it is refused: {output}'
        )
        self.assertNotIn('Set parameter successful', output, output)

        resp = requests.get(f'{CLOSED_BASE_URL}/areas', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            f'after a refused param set the gateway answered {resp.status_code}'
        )

    def test_06_a_param_load_applies_what_it_may_and_refuses_the_rest(self):
        """`ros2 param load` is how a whole file is pushed at a running node.

        It sets each parameter in turn, so the guard has to refuse the auth
        entry and leave the rest applied - a file mixing the two must not
        become an all-or-nothing gamble on which key came first.
        """
        path = os.path.join(_PARAMS_DIR, 'runtime-load.yaml')
        # Keyed on the absolute node name: `ros2 param load` matches the
        # file's keys against the name it was given, slash included.
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(
                f'/{ENV_CLOSED_NODE}:\n'
                '  ros__parameters:\n'
                '    refresh_interval_ms: 4500\n'
                '    auth:\n'
                '      enabled: false\n'
            )

        _, output = _ros2_param('load', '/gateway_env_closed', path)
        self.assertIn(
            'read at start', output,
            f'the auth entry in a loaded file was not refused: {output}')

        rc, refresh = _ros2_param('get', '/gateway_env_closed', 'refresh_interval_ms')
        self.assertEqual(rc, 0, refresh)
        self.assertIn(
            '4500', refresh,
            f'the non-auth entry was not applied: {refresh}')

        rc, enabled = _ros2_param('get', '/gateway_env_closed', 'auth.enabled')
        self.assertEqual(rc, 0, enabled)
        self.assertIn('True', enabled, f'auth.enabled was changed by the load: {enabled}')

        resp = requests.get(f'{CLOSED_BASE_URL}/areas', timeout=15)
        self.assertIn(resp.status_code, (401, 403), resp.text)

    def test_07_an_atomic_batch_mixing_the_two_is_refused_whole(self):
        """set_parameters_atomically is all-or-nothing, and auth.* refuses.

        The guard returns one result for the batch, so a batch carrying an
        auth parameter must apply none of it - including the harmless one
        beside it, which would otherwise be a way to have half a refused
        request take effect.
        """
        before = _ros2_param('get', '/gateway_env_closed', 'refresh_interval_ms')[1]

        script = (
            'import rclpy\n'
            'from rclpy.node import Node\n'
            'from rclpy.parameter import Parameter\n'
            'from rcl_interfaces.srv import SetParametersAtomically\n'
            'rclpy.init()\n'
            'n = Node("batch_probe")\n'
            'c = n.create_client(SetParametersAtomically,\n'
            '                    "/gateway_env_closed/set_parameters_atomically")\n'
            'assert c.wait_for_service(timeout_sec=30.0), "no service"\n'
            'req = SetParametersAtomically.Request()\n'
            'req.parameters = [\n'
            '    Parameter("refresh_interval_ms", Parameter.Type.INTEGER, 6100)\n'
            '        .to_parameter_msg(),\n'
            '    Parameter("auth.enabled", Parameter.Type.BOOL, False)\n'
            '        .to_parameter_msg(),\n'
            ']\n'
            'f = c.call_async(req)\n'
            'rclpy.spin_until_future_complete(n, f, timeout_sec=30.0)\n'
            'r = f.result()\n'
            'print("SUCCESSFUL", r.result.successful)\n'
            'print("REASON", r.result.reason)\n'
            'rclpy.shutdown()\n'
        )
        completed = subprocess.run(
            ['python3', '-c', script],
            capture_output=True, text=True, timeout=90, check=False,
        )
        output = completed.stdout + completed.stderr
        self.assertIn('SUCCESSFUL False', output, f'the batch was accepted: {output[-600:]}')
        self.assertIn('read at start', output, f'the batch gave no reason: {output[-600:]}')

        after = _ros2_param('get', '/gateway_env_closed', 'refresh_interval_ms')[1]
        self.assertEqual(
            before, after,
            'the harmless half of a refused atomic batch was applied')
        self.assertNotIn('6100', after, after)

    def test_05_a_non_auth_parameter_is_untouched_by_the_guard(self):
        """The guard names `auth.` and must not close the rest of the surface."""
        _, output = _ros2_param(
            'set', '/gateway_env_closed', 'refresh_interval_ms', '3000')
        self.assertNotIn(
            'read at start', output,
            f'the auth guard refused a parameter outside auth.*: {output}'
        )
        # The absence of the refusal is not the claim; a gateway that refused
        # every set for some other reason would satisfy it. The set has to work.
        self.assertIn(
            'Set parameter successful', output,
            f'setting a parameter outside auth.* did not succeed: {output}'
        )


class TestAMisconfiguredEnvironmentRefusesToStart(GatewayTestCase):
    """Two environments with no working reading, each refused at startup.

    Both would otherwise produce a gateway that runs and serves nobody: a
    client list where every entry was rejected leaves a gateway closed to its
    own operator, and MEDKIT_JWT_SECRET under RS256 hands a shared secret to a
    code path that wants a file path. Refusing is the same answer a missing
    secret already gets, and for the same reason - a gateway nobody can use is
    a misconfiguration, not a posture.

    Driven as subprocesses, because the subject is a process that must NOT come
    up: a launch action that dies is a fixture failure, and the exit code is the
    assertion here.
    """

    BASE_URL = CLOSED_BASE_URL

    @staticmethod
    def _run_gateway(env_extra, extra_args=(), timeout=45):
        """Start a gateway with this environment; return (rc, output).

        ``rc`` is None when the gateway announced itself ready, or was still
        running at the timeout - either is what "it started" looks like here,
        and the control case needs that to be distinguishable from a refusal.
        The ready line ends the wait, so a control returns the moment the
        gateway is up and does not sit out a timer on it.
        """
        env = dict(os.environ)
        env.update(env_extra)
        command = [
            'ros2', 'run', 'ros2_medkit_gateway', 'gateway_node', '--ros-args',
            '-p', f'server.port:={REFUSE_PORT}', '-p', 'server.host:=127.0.0.1',
            *extra_args,
        ]
        # Its own process group, and killed as a group on the way out. `ros2
        # run` execs the node as a CHILD, so killing the wrapper alone leaves a
        # gateway running with PPID 1 holding this port and its DDS domain - and
        # a later test drawing the same port then talks to it and fails looking
        # like a regression. The control case here is a gateway that comes up
        # and stays up, so this path is taken on every run.
        process = subprocess.Popen(
            command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=env, start_new_session=True,
        )
        chunks = []
        started = False
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([process.stdout], [], [], min(remaining, 0.5))
            if not readable:
                continue
            chunk = os.read(process.stdout.fileno(), 65536)
            if not chunk:
                break  # end of output: the process group is gone
            chunks.append(chunk)
            # The line can straddle two reads, so the last two are searched.
            if b'Medkit Gateway ready on' in b''.join(chunks[-2:]):
                started = True
                break

        def drain():
            rest, _ = process.communicate()
            if rest:
                chunks.append(rest)
            return b''.join(chunks).decode(errors='replace')

        if started or process.poll() is None and deadline <= time.monotonic():
            os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            return None, drain()

        process.wait(timeout=15)
        return process.returncode, drain()

    def test_01_a_client_list_refused_in_full_stops_the_gateway(self):
        rc, output = self._run_gateway({
            'MEDKIT_JWT_SECRET': JWT_SECRET,
            'MEDKIT_CLIENTS': 'nocolons,also-bad,third:entry:wizard',
        })
        self.assertNotEqual(
            rc, 0,
            f'a gateway whose every client entry was refused started anyway, '
            f'closed to everyone including its operator. Output: {output[-600:]}'
        )
        self.assertIn('every MEDKIT_CLIENTS entry was refused', output, output[-600:])

    def test_02_an_empty_client_list_starts_and_warns(self):
        """The control. An EMPTY value is a decision, and must not be refused.

        Without this, test_01 would pass against a gateway that refused any
        MEDKIT_CLIENTS it could not use a credential from, which is a different
        and wrong rule.
        """
        rc, output = self._run_gateway(
            {'MEDKIT_JWT_SECRET': JWT_SECRET, 'MEDKIT_CLIENTS': ''},
            timeout=20,
        )
        self.assertIsNone(
            rc,
            f'an empty MEDKIT_CLIENTS stopped the gateway; empty is a decision, '
            f'not a list that failed to parse. Output: {output[-600:]}'
        )
        self.assertIn(
            'Medkit Gateway ready on', output,
            f'the gateway neither refused nor came up: {output[-600:]}')
        self.assertNotIn('every MEDKIT_CLIENTS entry was refused', output, output[-600:])
        self.assertIn('no client can obtain a token', output, output[-600:])

    def test_03_a_secret_under_rs256_stops_the_gateway(self):
        rc, output = self._run_gateway(
            {'MEDKIT_JWT_SECRET': JWT_SECRET,
             'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin'},
            extra_args=('-p', 'auth.jwt_algorithm:=RS256'),
        )
        self.assertNotEqual(
            rc, 0,
            f'MEDKIT_JWT_SECRET under RS256 started a gateway. Output: {output[-600:]}'
        )
        self.assertIn('MEDKIT_JWT_SECRET is set and auth.jwt_algorithm is RS256', output,
                      output[-600:])
        self.assertNotIn(
            JWT_SECRET, output,
            'the refusal echoed the secret it was refusing')

    def test_04_a_short_secret_names_the_variable_it_came_from(self):
        """The message has to send the operator to the right knob.

        A FATAL naming `auth.jwt_secret` sends them to a params file the
        gateway did not read, which is the whole point of the environment
        contract being that the file loses.
        """
        rc, output = self._run_gateway({
            'MEDKIT_JWT_SECRET': 'too-short',
            'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin',
        })
        self.assertNotEqual(rc, 0, output[-600:])
        self.assertNotIn('too-short', output, 'the refusal echoed the secret')

        # The startup WARN naming MEDKIT_JWT_SECRET is printed on every closed
        # gateway, so finding that string anywhere proves nothing. The claim is
        # about the REFUSAL line: it has to say the secret came from the
        # environment, because that is where the operator has to go and fix it.
        refusal = [line for line in output.splitlines() if 'Refusing to start' in line]
        self.assertTrue(refusal, f'no refusal line in the output: {output[-600:]}')
        self.assertIn(
            'MEDKIT_JWT_SECRET', refusal[0],
            f'the refusal does not name the variable the secret came from: {refusal[0]}'
        )

    def test_05_a_refused_file_value_is_not_blamed_on_the_environment(self):
        """The refusal names which values the environment supplied, and no more.

        A gateway closed by MEDKIT_JWT_SECRET still reads its expiries from the
        parameters. The refusal line names the values the environment supplied
        (the secret, the clients, or both) and says every other auth.* value
        came from the parameters, so the validator's own message is what points
        at the refused key.
        """
        rc, output = self._run_gateway(
            {'MEDKIT_JWT_SECRET': JWT_SECRET,
             'MEDKIT_CLIENTS': f'{CLIENT_ID}:{CLIENT_SECRET}:admin'},
            extra_args=('-p', 'auth.token_expiry_seconds:=0'),
        )
        self.assertNotEqual(rc, 0, output[-600:])
        refusal = [line for line in output.splitlines() if 'Refusing to start' in line]
        self.assertTrue(refusal, f'no refusal line in the output: {output[-600:]}')
        self.assertIn(
            'Token expiry must be positive', refusal[0],
            f'the refusal does not say which value was refused: {refusal[0]}')
        self.assertIn(
            'came from the parameters', refusal[0],
            f'the refusal does not send the operator to the parameters: {refusal[0]}')
        self.assertNotIn(
            'MEDKIT_AUTH_DISABLED', refusal[0],
            f'the refusal names a variable the value did not come from: {refusal[0]}')

    def test_06_a_refusal_names_only_the_variable_that_was_set(self):
        """The secret without the clients: the clause names one variable."""
        rc, output = self._run_gateway(
            {'MEDKIT_JWT_SECRET': JWT_SECRET},
            extra_args=('-p', 'auth.token_expiry_seconds:=0'),
        )
        self.assertNotEqual(rc, 0, output[-600:])
        refusal = [line for line in output.splitlines() if 'Refusing to start' in line]
        self.assertTrue(refusal, f'no refusal line in the output: {output[-600:]}')
        self.assertIn(
            'The environment supplied auth.jwt_secret (MEDKIT_JWT_SECRET); every other '
            'auth.* value came from the parameters.',
            refusal[0], refusal[0])
        self.assertNotIn(
            'MEDKIT_CLIENTS', refusal[0],
            f'the refusal names a variable that was not set: {refusal[0]}')


class TestAFileSuppliedSecretIsAlsoRedacted(GatewayTestCase):
    """A secret is redacted wherever it came from and in every auth state.

    A `jwt_secret:=` launch argument or a value in a params file would reach
    the parameter services like any other parameter, and those answer any
    participant on the domain - a wider audience than the file the operator put
    it in. The gateway takes these from its overrides once at construction,
    declares the parameters with a sentinel, and the on-set guard makes them
    immutable, so nothing downstream needs the value.
    """

    BASE_URL = FILE_CLOSED_BASE_URL

    def test_01_the_gateway_is_closed_by_its_file_alone(self):
        """The premise: no MEDKIT_* variable is set for this gateway."""
        resp = requests.get(f'{FILE_CLOSED_BASE_URL}/areas', timeout=15)
        self.assertIn(resp.status_code, (401, 403), resp.text)

    def test_02_the_file_secret_reads_back_as_a_sentinel(self):
        rc, secret = _ros2_param('get', '/gateway_file_closed', 'auth.jwt_secret')
        self.assertEqual(rc, 0, secret)
        self.assertNotIn(
            FILE_JWT_SECRET, secret,
            'a params-file signing secret is readable through ros2 param')
        self.assertIn('<set at start>', secret, secret)

    def test_03_the_file_client_secrets_read_back_as_sentinels(self):
        rc, clients = _ros2_param('get', '/gateway_file_closed', 'auth.clients')
        self.assertEqual(rc, 0, clients)
        self.assertNotIn(
            CLIENT_SECRET, clients,
            'a params-file client secret is readable through ros2 param')
        self.assertIn(CLIENT_ID, clients, f'the client id should still show: {clients}')
        self.assertIn('<set at start>', clients, clients)

    def test_04_the_credential_still_works(self):
        """Redaction is about what is readable, not about what is configured."""
        token = requests.post(
            f'{FILE_CLOSED_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        )
        self.assertEqual(token.status_code, 200, token.text)
        resp = requests.get(
            f'{FILE_CLOSED_BASE_URL}/areas',
            headers={'Authorization': f'Bearer {token.json()["access_token"]}'},
            timeout=15,
        )
        self.assertEqual(resp.status_code, 200, resp.text)

    def test_05_the_malformed_entry_did_not_register_a_client(self):
        """The two-field entry is dropped, and the rest of the list stands.

        test_04 above shows the good entry registered; this shows the bad one
        did not become a client under some other reading of its fields.
        """
        token = requests.post(
            f'{FILE_CLOSED_BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': 'twofield',
                'client_secret': MALFORMED_CLIENT_SECRET,
            },
            timeout=30,
        )
        self.assertNotEqual(
            token.status_code, 200,
            'a two-field auth.clients entry was registered as a client')


class TestPublicRoutesAreValidatedWhateverThePosture(GatewayTestCase):
    """A malformed auth.public_routes stops the gateway with auth OFF too.

    A list validated only while authentication is on leaves a typo unnoticed
    until somebody closes the gateway - the worst moment to discover that a
    route they believe is reachable is not. The key is a configuration error
    whenever it is set.

    `[""]` is the opposite case and must not be refused: it is how a ROS 2 YAML
    file writes an empty sequence, and both shipped profiles use that idiom.
    """

    BASE_URL = CLOSED_BASE_URL

    def test_01_a_malformed_entry_stops_a_gateway_with_auth_off(self):
        rc, output = TestAMisconfiguredEnvironmentRefusesToStart._run_gateway(
            {}, extra_args=('-p', 'auth.enabled:=false',
                            '-p', 'auth.public_routes:=["nonsense"]'))
        self.assertNotEqual(
            rc, 0,
            f'a malformed auth.public_routes started a gateway because auth was '
            f'off. Output: {output[-600:]}'
        )
        self.assertIn('auth.public_routes entry "nonsense" is invalid', output, output[-600:])

    def test_02_a_blank_entry_is_accepted_with_auth_off(self):
        """The control: the empty-sequence idiom is not a typo."""
        rc, output = TestAMisconfiguredEnvironmentRefusesToStart._run_gateway(
            {}, extra_args=('-p', 'auth.enabled:=false',
                            '-p', 'auth.public_routes:=[""]'),
            timeout=20)
        self.assertIsNone(
            rc,
            f'auth.public_routes: [""] stopped the gateway; that is how a ROS 2 '
            f'YAML file writes an empty sequence. Output: {output[-600:]}'
        )
        self.assertIn(
            'Medkit Gateway ready on', output,
            f'the gateway neither refused nor came up: {output[-600:]}')
        self.assertNotIn('auth.public_routes entry', output, output[-600:])

    def test_03_a_blank_entry_exempts_nothing(self):
        """And it opens no route.

        The closed gateway in this launch carries `auth.public_routes: [""]`,
        so it came up with the blank entry and still refuses /health.
        """
        resp = requests.get(f'{CLOSED_BASE_URL}/health', timeout=15)
        self.assertIn(resp.status_code, (401, 403), resp.text)


@launch_testing.post_shutdown_test()
class TestEnvAuthContractShutdown(unittest.TestCase):
    """Both gateways exit cleanly."""

    def test_exit_codes(self, proc_info, closed, disabled, file_closed):
        for proc in (closed, disabled, file_closed):
            launch_testing.asserts.assertExitCodes(
                proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=proc
            )

    def test_no_secret_reached_the_log(self, proc_output, closed, disabled, file_closed):
        """Nothing the gateway printed carries a secret it was given.

        Container logs are shipped, aggregated and kept, so a secret echoed
        once at startup outlives the process and reaches an audience nobody
        chose. Checked post-shutdown, over the whole captured output rather
        than a line somebody expected to be the risky one.

        Concatenated with no separator: proc_output yields raw stream chunks
        and a chunk boundary can fall mid-line, so joining with a newline
        splices one into the text being searched.
        """
        for proc in (closed, disabled, file_closed):
            text = ''.join(
                output.text.decode(errors='replace') for output in proc_output[proc]
            )
            self.assertNotIn(
                JWT_SECRET, text,
                'MEDKIT_JWT_SECRET was echoed into the gateway log'
            )
            self.assertNotIn(
                CLIENT_SECRET, text,
                'a MEDKIT_CLIENTS client secret was echoed into the gateway log'
            )
            self.assertNotIn(
                FILE_JWT_SECRET, text,
                'a params-file jwt_secret was echoed into the gateway log'
            )
            # The malformed entry is the interesting one: a warning that quoted
            # the entry to show what was wrong with it would publish the secret
            # inside it.
            self.assertNotIn(
                MALFORMED_CLIENT_SECRET, text,
                'the secret inside a malformed auth.clients entry was echoed '
                'into the gateway log'
            )
            # A swapped entry puts the secret in the role field; a warning that
            # quoted the role it could not recognise would print it.
            self.assertNotIn(
                SWAPPED_FILE_SECRET, text,
                'the role field of a swapped auth.clients entry was echoed into the log')
            self.assertNotIn(
                SWAPPED_ENV_SECRET, text,
                'the role field of a swapped MEDKIT_CLIENTS entry was echoed into the log')
            self.assertNotIn(
                PEER_AUTH_HEADER, text,
                'the peer auth header was echoed into the gateway log')
            # The blank entry is not an exemption and gets no line of its own;
            # the two spaces are what an empty entry leaves in that message.
            self.assertNotIn(
                'auth.public_routes:  is answered', text,
                'a blank auth.public_routes entry was logged as an exemption')

    @classmethod
    def tearDownClass(cls):
        """Drop the params files, which exist only for this run."""
        shutil.rmtree(_PARAMS_DIR, ignore_errors=True)
