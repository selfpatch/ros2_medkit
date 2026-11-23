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

"""Entity freeze frames read over HTTP, and the fault manager calls that make them."""

import os
import time

from ament_index_python.packages import get_package_prefix
from launch.actions import ExecuteProcess
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ClearFault, ListFaults, ReportFault

from ros2_medkit_test_utils.constants import API_BASE_PATH
from ros2_medkit_test_utils.launch_helpers import create_gateway_node
from ros2_medkit_test_utils.service_round_trip import answer_once_each, call_until_answered

TIMEOUT_SEC = 60.0
ALL_STATUSES = ['PREFAILED', 'PREPASSED', 'CONFIRMED', 'HEALED', 'CLEARED']


def route_plugin_gateway(name, port, store_path):
    """Return a gateway with the route-data test plugin and a freeze-frame store file."""
    plugin = os.path.join(get_package_prefix('ros2_medkit_gateway'), 'lib',
                          'ros2_medkit_gateway', 'libtest_route_data_plugin.so')
    return create_gateway_node(
        name=name,
        port=port,
        extra_params={
            'plugins': ['test_route_data'],
            'plugins.test_route_data.path': plugin,
            'entity_freeze_frame.storage.path': store_path,
        },
    )


def gate_process(name, path):
    """Return a process that exits once ``path`` exists, to start another on its exit."""
    return ExecuteProcess(
        cmd=['sh', '-c', f'while [ ! -e "{path}" ]; do sleep 0.2; done'],
        name=name, output='screen')


def open_gate(path):
    with open(path, 'w', encoding='utf-8') as gate:
        gate.write('open')


def detail_frames(port, entity_path, code):
    """Return the freeze frames of a fault detail by entity id, or None when it is not served."""
    try:
        response = requests.get(
            f'http://localhost:{port}{API_BASE_PATH}/{entity_path}/faults/{code}', timeout=5)
    except requests.exceptions.RequestException:
        return None
    if response.status_code != 200:
        return None
    body = response.json()
    snapshots = body.get('environment_data', {}).get('snapshots', [])
    return {
        'frames': {s.get('name'): s for s in snapshots if s.get('type') == 'freeze_frame'},
        'occurrence': body.get('x-medkit', {}).get('occurrence_count'),
    }


def wait_frames(port, entity_path, code, entities, *, ready=lambda frame: True,
                timeout=TIMEOUT_SEC):
    """Poll until the detail serves a frame of each of ``entities`` that is ``ready``."""
    deadline = time.monotonic() + timeout
    last = None
    while time.monotonic() < deadline:
        last = detail_frames(port, entity_path, code)
        if last and all(e in last['frames'] and ready(last['frames'][e]) for e in entities):
            return last
        time.sleep(0.2)
    raise AssertionError(
        f'{entity_path}/faults/{code} on port {port}: no frames of {entities}; last {last}')


def captured_at(frame):
    return frame.get('x-medkit', {}).get('captured_at')


def capture_origin(frame):
    return frame.get('x-medkit', {}).get('capture_origin')


class FaultManagerCalls:
    """Report, clear and list through the fault manager, each call answered."""

    def __init__(self, node, timeout=TIMEOUT_SEC):
        self.node = node
        self.timeout = timeout
        self.report_client = node.create_client(ReportFault, '/fault_manager/report_fault')
        self.clear_client = node.create_client(ClearFault, '/fault_manager/clear_fault')
        self.list_client = node.create_client(ListFaults, '/fault_manager/list_faults')
        self.sentinels = 0

    def ready(self):
        """Wait for the services and one answer from each."""
        clients = (self.report_client, self.clear_client, self.list_client)
        for client in clients:
            assert client.wait_for_service(timeout_sec=self.timeout), client.srv_name
        answer_once_each(self.node, clients, self.timeout)

    def report(self, code, source):
        request = ReportFault.Request()
        request.fault_code = code
        request.event_type = ReportFault.Request.EVENT_FAILED
        request.severity = Fault.SEVERITY_ERROR
        request.description = 'freeze frame test fault'
        request.source_id = source
        answer = call_until_answered(self.node, self.report_client, request, self.timeout)
        assert answer.accepted, f'{code} from {source} was not accepted'

    def clear(self, code, source):
        request = ClearFault.Request()
        request.fault_code = code
        request.source_id = source
        answer = call_until_answered(self.node, self.clear_client, request, self.timeout)
        assert answer.success, answer.message

    def listing(self):
        request = ListFaults.Request()
        request.statuses = ALL_STATUSES
        request.include_muted = True
        return call_until_answered(self.node, self.list_client, request, self.timeout)

    def records(self, code):
        """Return {owner: Fault} of every record of ``code``."""
        return {f.owner: f for f in self.listing().faults if f.fault_code == code}

    def wait_event_route(self, port, app, prefix):
        """
        Report fresh codes from ``app`` until one is framed on the gateway at ``port``.

        A frame of a code confirmed now shows that the gateway receives confirm events.
        """
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            self.sentinels += 1
            code = f'{prefix}_{self.sentinels}'
            self.report(code, app)
            wait_until = time.monotonic() + 5.0
            while time.monotonic() < wait_until:
                found = detail_frames(port, f'apps/{app}', code)
                if found and app in found['frames']:
                    return
                time.sleep(0.2)
        raise AssertionError(f'no {prefix} code was framed on port {port}')
