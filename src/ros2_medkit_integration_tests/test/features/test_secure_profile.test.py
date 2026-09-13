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

"""Boot the gateway from the SECURE profile and check that it is closed.

``config/gateway_params.yaml`` is the open profile and
``config/gateway_params.secure.yaml`` is the closed one. Every other test in
this suite builds its parameters inline, which is fine for testing behaviour
but means nothing loads either file - so the secure profile could be edited
back to ``auth.enabled: false`` and the whole suite would stay green, because
each test supplies the values it needs itself.

This file closes that gap for the secure profile. It launches with the
installed copy of that file, overriding only the port, the signing secret and
the client - the three things a real deployment must supply and the file
deliberately leaves empty - and then checks that what it ships is closed.
``test_open_default_profile`` is the mirror for the other file.

TLS is turned off here and only here. The secure file has it on, which is
correct, but a certificate is a deployment artefact and generating one would
test the certificate rather than the posture. ``test_tls_protocol_floor``
covers TLS itself against real handshakes.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import os
import socket
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
)
from ros2_medkit_test_utils.coverage import get_coverage_env
import yaml

PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{PORT}{API_BASE_PATH}'

SECURE_PARAMS = os.path.join(
    get_package_share_directory('ros2_medkit_gateway'),
    'config', 'gateway_params.secure.yaml'
)

JWT_SECRET = 'secure_profile_integration_secret_key_01234567890'
CLIENT_ID = 'secure'
CLIENT_SECRET = 'secure_client_secret'


@pytest.mark.launch_test
def generate_test_description():
    """Launch the gateway with the secure params file, plus the required secrets."""
    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[
            SECURE_PARAMS,
            {
                'server.host': '127.0.0.1',
                'server.port': PORT,
                'refresh_interval_ms': 1000,
                # A certificate is a deployment artefact, not part of the
                # posture under test here.
                'server.tls.enabled': False,
                # What the secure file leaves empty on purpose.
                'auth.jwt_secret': JWT_SECRET,
                'auth.clients': [f'{CLIENT_ID}:{CLIENT_SECRET}:admin'],
            },
        ],
        additional_env=dict(get_coverage_env()),
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


def _wait_listening(port, timeout=90.0):
    """Block until the gateway accepts a connection.

    launch_testing starts the tests when the process is spawned, not when it is
    serving. Without this the first request is refused by a gateway that simply
    has not opened its socket yet, which looks nothing like the posture this
    file is about.

    The timeout is generous because this gateway loads the full secure profile,
    which does more work at startup than the inline parameter sets the rest of
    the suite uses.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection(('127.0.0.1', port), timeout=2):
                return
        except OSError:
            time.sleep(0.25)
    raise AssertionError(f'gateway on port {port} never started listening within {timeout}s')


class TestSecureProfile(unittest.TestCase):
    """What config/gateway_params.yaml actually produces."""

    @classmethod
    def setUpClass(cls):
        _wait_listening(PORT)
        resp = requests.post(
            f'{BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        )
        assert resp.status_code == 200, f'token request failed: {resp.status_code} {resp.text}'
        cls.auth = {'Authorization': f'Bearer {resp.json()["access_token"]}'}

    def test_01_the_secure_file_turns_authentication_on(self):
        """Reverting auth.enabled in the secure file must fail here.

        No other test would notice: they all pass auth.enabled themselves.
        """
        resp = requests.get(f'{BASE_URL}/areas', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            'the secure profile served /areas to an anonymous caller'
        )

    def test_02_the_secure_file_covers_reads_not_just_writes(self):
        """Pins require_auth_for: "all" as the secure profile's value.

        Under "write" every one of these answers 200 without a credential.
        """
        for path in ('/', '/areas', '/components', '/apps', '/functions', '/version-info'):
            with self.subTest(path=path):
                resp = requests.get(f'{BASE_URL}{path}', timeout=15)
                self.assertIn(resp.status_code, (401, 403))

    def test_03_health_refuses_in_the_secure_file_too(self):
        """The secure file opens nothing, health included.

        `auth.public_routes` is absent from the secure profile, so the file
        that a closed deployment starts from leaves no route reachable without
        a credential. An operator who wants a probe route adds the entry
        themselves - that path is covered in test_closed_by_default.

        Pinned here separately from the sweep above because health is the route
        a hardening change is most tempted to leave open, and a secure profile
        that quietly did so would still pass every other test in this class.
        """
        resp = requests.get(f'{BASE_URL}/health', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            f'the secure profile answered GET /health with {resp.status_code} '
            'to a caller holding no credential'
        )

    def test_04_a_configured_client_still_works(self):
        """The mirror: a gateway that refused everyone would pass the rest."""
        resp = requests.get(f'{BASE_URL}/areas', headers=self.auth, timeout=15)
        self.assertEqual(resp.status_code, 200)

    def test_05_the_secure_file_is_the_one_under_test(self):
        """Guard against this test silently drifting off the real file.

        If the installed config stops declaring the values this file exists to
        check, the assertions above would still pass for the wrong reason.

        Read through a YAML parse and addressed by key path. A substring search
        cannot do this job: ``enabled: true`` occurs under several different
        parents in this file, so a text guard for it stays green with
        ``auth.enabled: false`` - the one drift this test exists to catch. The
        parse also settles ``public_routes`` for free, because a commented
        example is not a key and a key written in flow style still is one.
        """
        with open(SECURE_PARAMS, encoding='utf-8') as handle:
            document = yaml.safe_load(handle)
        params = document['ros2_medkit_gateway']['ros__parameters']
        auth = params['auth']
        self.assertIs(
            auth['enabled'], True,
            f"the secure profile sets auth.enabled to {auth['enabled']!r}"
        )
        self.assertEqual(
            auth['require_auth_for'], 'all',
            'the secure profile sets auth.require_auth_for to '
            f'{auth["require_auth_for"]!r}'
        )
        self.assertIs(
            params['server']['tls']['enabled'], True,
            'the secure profile sets server.tls.enabled to '
            f"{params['server']['tls']['enabled']!r}"
        )
        # No route is opened by the file itself. An entry here would be a
        # public route in every deployment that uses this profile, which is
        # exactly what it exists to stop.
        self.assertNotIn(
            'public_routes', auth,
            'the secure profile declares public routes: '
            f'{auth.get("public_routes")!r}'
        )


@launch_testing.post_shutdown_test()
class TestSecureProfileShutdown(unittest.TestCase):
    """Gateway exits cleanly."""

    def test_exit_codes(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=gateway_node
        )
