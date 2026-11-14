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

"""A per-entity clear that cannot clear every record says so.

DELETE /{entity}/faults lists the records under the entity and sends one
ClearFault per record. While no fault manager answers, nothing can be listed
and the route answers 503. When the fault manager refuses one record, the route
answers 500 and names its code in ``failed_codes``; the records it could clear
stay cleared.

The fault manager clears a record it holds whatever its status, so it refuses
only a record that is gone between the list and the clear. No test can hold
that window open, so refusing_fault_manager.py stands in for it: it lists two
records and refuses the clear of one.

@verifies REQ_INTEROP_015
"""

import os
import sys
import tempfile
import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import (
    ALLOWED_EXIT_CODES,
    API_BASE_PATH,
    get_test_port,
    get_time_scale,
)
from ros2_medkit_test_utils.launch_helpers import create_gateway_node

PORT = get_test_port(0)
BASE_URL = f'http://localhost:{PORT}{API_BASE_PATH}'

TIME_SCALE = get_time_scale()
HTTP_TIMEOUT_SEC = 30.0 * TIME_SCALE
DISCOVERY_TIMEOUT_SEC = 60.0 * TIME_SCALE

OWNER_A = 'owner-a'
OWNER_B = 'owner-b'
HOST_COMPONENT = 'shared-code-hub'
ACCEPTED_CODE = 'CLEAR_ACCEPTED'
REFUSED_CODE = 'CLEAR_REFUSED'

FAULT_MANAGER_GATE = os.path.join(
    tempfile.gettempdir(), f'test_faults_entity_clear_failure_gate_{os.getpid()}')


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'fault_owner_identity_manifest.yaml',
    )
    gateway = create_gateway_node(
        port=PORT,
        extra_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            'fault_manager.service_timeout_sec': 2.0,
        },
    )
    gate = ExecuteProcess(
        cmd=['sh', '-c', f'while [ ! -e "{FAULT_MANAGER_GATE}" ]; do sleep 0.2; done'],
        name='fault_manager_gate',
        output='screen',
    )
    fault_manager = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'refusing_fault_manager.py'),
        ],
        name='refusing_fault_manager',
        output='screen',
    )
    return (
        LaunchDescription([
            gateway,
            gate,
            RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=[fault_manager])),
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


def _poll(predicate, *, what, timeout=DISCOVERY_TIMEOUT_SEC):
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        try:
            last = predicate()
            if last:
                return last
        except requests.exceptions.RequestException as exc:
            last = exc
        time.sleep(0.3)
    raise AssertionError(f'timed out after {timeout}s waiting for {what}; last={last!r}')


def _get(path):
    return requests.get(f'{BASE_URL}{path}', timeout=HTTP_TIMEOUT_SEC)


def _delete(path):
    return requests.delete(f'{BASE_URL}{path}', timeout=HTTP_TIMEOUT_SEC)


def _statuses(path):
    """Code -> status of every item GET {path}?status=all lists."""
    response = _get(f'{path}?status=all')
    response.raise_for_status()
    return {item['fault_code']: item['status'] for item in response.json().get('items', [])}


class TestEntityClearFailure(unittest.TestCase):
    """The bulk clear answers 503 before the fault manager and 500 for a refused record."""

    @classmethod
    def setUpClass(cls):
        cls.addClassCleanup(lambda: os.path.exists(FAULT_MANAGER_GATE)
                            and os.unlink(FAULT_MANAGER_GATE))
        for path in (f'/components/{HOST_COMPONENT}', f'/apps/{OWNER_A}', f'/apps/{OWNER_B}'):
            _poll(lambda p=path: _get(p).status_code == 200, what=f'{path} to be discovered')

    def test_01_without_a_fault_manager_the_bulk_clear_is_503(self):
        response = _delete(f'/components/{HOST_COMPONENT}/faults')
        self.assertEqual(response.status_code, 503, response.text)

    def test_02_a_refused_record_clear_is_500_naming_its_code(self):
        with open(FAULT_MANAGER_GATE, 'w', encoding='utf-8') as gate:
            gate.write('open')
        _poll(lambda: len(_statuses(f'/components/{HOST_COMPONENT}/faults')) == 2,
              what='the fault manager to list both records')

        response = _delete(f'/components/{HOST_COMPONENT}/faults')
        self.assertEqual(response.status_code, 500, response.text)
        body = response.json()
        self.assertEqual(body['parameters']['failed_codes'], [REFUSED_CODE], body)
        self.assertEqual(
            _statuses(f'/components/{HOST_COMPONENT}/faults'),
            {ACCEPTED_CODE: 'CLEARED', REFUSED_CODE: 'CONFIRMED'},
            'the record the fault manager accepted must stay cleared')

        # One app's collection: the same answer for the one record it holds.
        response = _delete(f'/apps/{OWNER_B}/faults')
        self.assertEqual(response.status_code, 500, response.text)
        self.assertEqual(response.json()['parameters']['failed_codes'], [REFUSED_CODE])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES,
                          f'{info.process_name} exited with code {info.returncode}')
