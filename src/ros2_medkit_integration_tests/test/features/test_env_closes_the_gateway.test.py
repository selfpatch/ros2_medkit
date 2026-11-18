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

"""MEDKIT_JWT_SECRET closes a gateway started through gateway.launch.py.

The rule lives in the gateway node (see test_env_auth_contract). This file
checks that the variables reach a node started by `gateway.launch.py`, with
no launch argument naming auth and against the open default profile, so
anything that closes the gateway came from the environment.

@verifies REQ_INTEROP_086, REQ_INTEROP_087
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

PORT = get_test_port()
BASE_URL = f'http://127.0.0.1:{PORT}{API_BASE_PATH}'

JWT_SECRET = 'an_environment_supplied_secret_of_at_least_32_chars'
CLIENT_ID = 'envclosed'
CLIENT_SECRET = 'env_closed_client_secret'


@pytest.mark.launch_test
def generate_test_description():
    """Include gateway.launch.py with the closing variables in its environment."""
    launch_file = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'), 'launch', 'gateway.launch.py')

    return launch.LaunchDescription([
        launch.actions.SetEnvironmentVariable('MEDKIT_JWT_SECRET', JWT_SECRET),
        launch.actions.SetEnvironmentVariable(
            'MEDKIT_CLIENTS', f'{CLIENT_ID}:{CLIENT_SECRET}:admin'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file),
            launch_arguments={
                'server_port': str(PORT),
                'server_host': '127.0.0.1',
            }.items(),
        ),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class TestEnvClosesTheGateway(GatewayTestCase):
    """The environment variable alone is enough to close the gateway."""

    BASE_URL = BASE_URL

    def test_01_an_anonymous_read_is_refused(self):
        """A read, not /health: at require_auth_for "write" a read answers 200."""
        resp = requests.get(f'{BASE_URL}/areas', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            f'MEDKIT_JWT_SECRET was in the environment and an anonymous GET '
            f'/areas answered {resp.status_code}. Body: {resp.text[:300]}'
        )

    def test_02_health_is_refused_too(self):
        """No route is exempt but the auth routes, health included."""
        resp = requests.get(f'{BASE_URL}/health', timeout=15)
        self.assertIn(
            resp.status_code, (401, 403),
            f'an anonymous GET /health answered {resp.status_code}'
        )

    def test_03_the_environment_credential_issues_a_working_token(self):
        """The mirror: a gateway that refused everyone would pass the two above."""
        token = requests.post(
            f'{BASE_URL}/auth/authorize',
            json={
                'grant_type': 'client_credentials',
                'client_id': CLIENT_ID,
                'client_secret': CLIENT_SECRET,
            },
            timeout=30,
        )
        self.assertEqual(token.status_code, 200, token.text)
        access = token.json()['access_token']
        resp = requests.get(
            f'{BASE_URL}/areas',
            headers={'Authorization': f'Bearer {access}'},
            timeout=15,
        )
        self.assertEqual(resp.status_code, 200, resp.text)


@launch_testing.post_shutdown_test()
class TestEnvClosesTheGatewayShutdown(unittest.TestCase):
    """Every process exits cleanly."""

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
