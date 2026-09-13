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

"""MEDKIT_JWT_SECRET in the environment closes a gateway started by the launch file.

The container image is where this matters. `docker run <img> ros2 launch
ros2_medkit_gateway bringup.launch.py` execs a command instead of the node, so
the entrypoint's own `-p` arguments never reach the gateway and the environment
is the only channel the variable has. A launch file that took the secret and
left `auth.enabled` at the open profile's `false` produced exactly the failure
worth a test: the operator is told the container is closed, and it serves the
entity tree, the fault history and every operation to anyone who reaches the
port.

Driven through `gateway.launch.py` itself, with no launch arguments, because
the defect lived in the argument-versus-environment precedence inside that
file. Supplying the parameters directly would test the gateway, which was never
wrong here.

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
    """Include gateway.launch.py with the closing variables in its environment.

    No launch argument names auth or TLS. The config file it defaults to is the
    open profile, so anything that closes this gateway came from the
    environment - which is what the case is about.
    """
    launch_file = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'), 'launch', 'gateway.launch.py')

    # SetEnvironmentVariable, because gateway.launch.py reads os.environ when
    # the launch description is evaluated and that happens in this process.
    # Confining it to this file is what launch_testing already does: each test
    # file runs in a process of its own, so the variable reaches this gateway
    # and no other.
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
        """The claim the image documentation makes, checked on the launch path.

        ``/areas`` and not ``/health``: a read is what "closed" has to mean
        here. Under ``require_auth_for: "write"`` - the open profile's value,
        and what the gateway keeps if only ``auth.enabled`` is asserted - this
        request answers 200 with authentication switched on.
        """
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
        """The mirror: a gateway that refused everyone would pass the two above.

        It also pins MEDKIT_CLIENTS reaching the gateway - without it the
        container is closed to its operator as well as to everyone else.
        """
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
    """Every process exits cleanly.

    Swept without naming one: the gateway is created inside the included launch
    file, so this file holds no handle to it.
    """

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
