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

"""The default profile answers a caller who holds no credential.

``config/gateway_params.yaml`` is what every existing launch, every existing
config file and the web UI get when they name no profile of their own, and none
of them sends an ``Authorization`` header. Closing that file is therefore a
breaking change for all three at once, and it is a one-word edit.

The mirror of ``test_secure_profile``: that file pins the closed profile
closed, this one pins the open profile open, and between them a flip in either
direction has to be deliberate. Nothing else in the suite would notice - every
other test supplies the auth parameters it needs inline, so both files could
say anything and stay green.

What this does NOT assert is that leaving it open is correct. It asserts that
the file has not changed underneath a deployment that already trusts it.

@verifies REQ_INTEROP_086
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
import yaml

PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{PORT}{API_BASE_PATH}'

DEFAULT_PARAMS = os.path.join(
    get_package_share_directory('ros2_medkit_gateway'),
    'config', 'gateway_params.yaml'
)


@pytest.mark.launch_test
def generate_test_description():
    """Launch the gateway from the default params file, overriding only the port."""
    gateway_node = create_gateway_node(
        port=PORT,
        params_file=DEFAULT_PARAMS,
        # Nothing about auth or TLS is set here: whatever the file says is
        # exactly what this test is about. The host is narrowed because a test
        # has no business binding every interface on the machine it runs on.
        extra_params={'server.host': '127.0.0.1'},
    )

    return launch.LaunchDescription([
        gateway_node,
        launch_testing.actions.ReadyToTest(),
    ]), {'gateway_node': gateway_node}


class TestOpenDefaultProfile(GatewayTestCase):
    """The default profile serves an uncredentialed caller."""

    BASE_URL = BASE_URL

    def test_01_an_anonymous_read_of_areas_succeeds(self):
        """A GET with no Authorization header is answered, not refused.

        ``/areas`` rather than ``/health``: health has its own anonymous
        handling (a reduced body on a route opened through
        ``auth.public_routes``), so it can answer 200 for a reason that has
        nothing to do with the profile. An entity collection has no such path -
        a 200 here means the request was authorised.
        """
        resp = requests.get(f'{BASE_URL}/areas', timeout=15)
        self.assertEqual(
            resp.status_code, 200,
            f'the default profile answered an anonymous GET /areas with '
            f'{resp.status_code}; every existing launch and the web UI send no '
            f'credential. Body: {resp.text[:300]}'
        )

    def test_02_an_anonymous_read_of_health_succeeds_in_full(self):
        """Health answers, and answers whole.

        ``x-medkit-reduced`` marks the cut-down body an anonymous caller gets
        when authentication is on. Its absence is what separates "auth is off"
        from "auth is on and this route was opened".
        """
        resp = requests.get(f'{BASE_URL}/health', timeout=15)
        self.assertEqual(resp.status_code, 200, resp.text)
        body = resp.json()
        self.assertEqual(body.get('status'), 'healthy', body)
        self.assertNotIn(
            'x-medkit-reduced', body,
            'health came back marked reduced, which means authentication is on '
            f'in the default profile: {body}'
        )

    def test_03_the_file_under_test_is_the_open_one(self):
        """Guard against this test drifting off the file it names.

        Read by key path through a YAML parse. The three values are the whole
        of the profile's posture, and each is one word away from its opposite.
        """
        with open(DEFAULT_PARAMS, encoding='utf-8') as handle:
            document = yaml.safe_load(handle)
        params = document['ros2_medkit_gateway']['ros__parameters']
        auth = params['auth']
        self.assertIs(
            auth['enabled'], False,
            f"the default profile sets auth.enabled to {auth['enabled']!r}"
        )
        self.assertEqual(
            auth['require_auth_for'], 'write',
            'the default profile sets auth.require_auth_for to '
            f'{auth["require_auth_for"]!r}'
        )
        self.assertIs(
            params['server']['tls']['enabled'], False,
            'the default profile sets server.tls.enabled to '
            f"{params['server']['tls']['enabled']!r}"
        )


@launch_testing.post_shutdown_test()
class TestOpenDefaultProfileShutdown(unittest.TestCase):
    """Gateway exits cleanly."""

    def test_exit_codes(self, proc_info, gateway_node):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES, process=gateway_node
        )
