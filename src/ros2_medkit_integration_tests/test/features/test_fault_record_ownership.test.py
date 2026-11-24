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

"""A fault record belongs to every App that reported it, over the whole HTTP stack.

A record is in an entity's scope when any of its reporting sources is. A shared
code reported by Apps A and B therefore shows on A and on B, and a DELETE
through either clears the one record for both. A per_source code keeps one
record per App. An App that reported nothing lists nothing, and a trigger on an
App fires for a shared record that App reported.

@verifies REQ_INTEROP_012
@verifies REQ_INTEROP_013
@verifies REQ_INTEROP_015
"""

import atexit
import json
import os
import shutil
import tempfile
import threading
import time
import unittest

import launch_testing
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


OWNER_A = 'owner-a'
OWNER_B = 'owner-b'
OWNER_C = 'owner-c'
NEAR_A = 'owner-a2'
HUB = 'ownership-hub'
CELL = 'ownership-cell'
SHARED_CODE = 'OWNERSHIP_SHARED'
PER_SOURCE_CODE = 'OWNERSHIP_PER_SOURCE'
TRIGGER_CODE = 'OWNERSHIP_TRIGGER'
FAULT_TIMEOUT = 30.0
CONFIG_DIR = tempfile.mkdtemp(prefix='medkit_record_ownership_')
atexit.register(shutil.rmtree, CONFIG_DIR, True)
MANIFEST = os.path.join(CONFIG_DIR, 'manifest.yaml')
THRESHOLDS = os.path.join(CONFIG_DIR, 'fault_thresholds.yaml')
MANIFEST_TEXT = f"""
manifest_version: "1.0"
metadata:
  name: "record-ownership"
  version: "1.0.0"
config:
  unmanifested_nodes: warn
  inherit_runtime_resources: true
areas:
  - id: {CELL}
    name: "Cell"
    namespace: /ownership_cell
components:
  - id: {HUB}
    name: "Hub"
    type: "controller"
    area: {CELL}
apps:
""" + ''.join(f"""  - id: {app}
    name: "{app}"
    external: true
    is_located_on: {HUB}
""" for app in (OWNER_A, OWNER_B, OWNER_C, NEAR_A))


def generate_test_description():
    with open(MANIFEST, 'w') as f:
        f.write(MANIFEST_TEXT)
    with open(THRESHOLDS, 'w') as f:
        f.write(f'{PER_SOURCE_CODE}:\n  per_source: true\n')
    return create_test_launch(
        demo_nodes=[],
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,
            'fault_thresholds.config_file': THRESHOLDS,
            'snapshots.rosbag.enabled': False,
        },
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': MANIFEST,
            'discovery.manifest_strict_validation': False,
        },
    )


class TestFaultRecordOwnership(GatewayTestCase):
    """Each App that reported a record lists it, and only those Apps do."""

    MIN_EXPECTED_APPS = 4
    REQUIRED_APPS = {OWNER_A, OWNER_B, OWNER_C, NEAR_A}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._node = Node('record_ownership_reporter')
        cls._client = cls._node.create_client(ReportFault, '/fault_manager/report_fault')
        super().setUpClass()
        assert cls._client.wait_for_service(timeout_sec=15.0), 'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()
        rclpy.shutdown()

    def _report(self, code, source):
        """Report a FAILED event and wait for the fault manager's answer."""
        req = ReportFault.Request()
        req.fault_code = code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = f'reported by {source}'
        req.source_id = source
        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
        self.assertTrue(future.done() and future.result().accepted,
                        f'{code} from {source} not accepted')

    def _entries(self, entity, code, status=None):
        query = f'?status={status}' if status else ''
        items = self.get_json(f'/{entity}/faults{query}').get('items', [])
        return [i for i in items if i.get('fault_code') == code]

    @staticmethod
    def _sources_of(listing, code):
        return sorted(s for i in listing.get('items', []) if i.get('fault_code') == code
                      for s in i.get('reporting_sources', []))

    def _wait_unlisted(self, entity, code):
        """Wait until the entity's default fault list no longer shows the code."""
        self.poll_endpoint_until(
            f'/{entity}/faults',
            lambda d: d if not any(i.get('fault_code') == code for i in d.get('items', []))
            else None,
            timeout=FAULT_TIMEOUT)

    def _wait_sources(self, code, sources):
        """Wait until the global list holds one record of the code from exactly these sources."""
        self.poll_endpoint_until(
            '/faults?status=all',
            lambda d: d if [sorted(i.get('reporting_sources', [])) for i in d.get('items', [])
                            if i.get('fault_code') == code] == [sorted(sources)] else None,
            timeout=FAULT_TIMEOUT)

    def test_01_a_shared_record_belongs_to_every_app_that_reported_it(self):
        """A and B list the shared record, C and A2 do not, and one DELETE clears it."""
        self._report(SHARED_CODE, OWNER_A)
        self._report(SHARED_CODE, OWNER_B)
        self._wait_sources(SHARED_CODE, [OWNER_A, OWNER_B])

        for app in (OWNER_A, OWNER_B):
            entries = self._entries(f'apps/{app}', SHARED_CODE)
            self.assertEqual(len(entries), 1, f'{app}: {entries}')
            self.assertEqual(sorted(entries[0]['reporting_sources']), [OWNER_A, OWNER_B])
            detail = self.get_json(f'/apps/{app}/faults/{SHARED_CODE}')
            self.assertEqual(detail['item']['code'], SHARED_CODE, detail)
        for app in (OWNER_C, NEAR_A):
            self.assertEqual(self._entries(f'apps/{app}', SHARED_CODE, 'all'), [], app)
            self.get_json(f'/apps/{app}/faults/{SHARED_CODE}', expected_status=404)
        self.assertEqual(len(self._entries(f'components/{HUB}', SHARED_CODE)), 1)

        # One record: clearing it through A clears it for B as well.
        self.delete_request(f'/apps/{OWNER_A}/faults/{SHARED_CODE}')
        self._wait_unlisted(f'apps/{OWNER_B}', SHARED_CODE)
        cleared = self._entries(f'apps/{OWNER_B}', SHARED_CODE, 'all')
        self.assertEqual([i['status'] for i in cleared], ['CLEARED'], cleared)
        self.assertEqual(self._entries(f'apps/{OWNER_A}', SHARED_CODE), [])

    def test_02_a_per_source_code_keeps_one_record_per_app(self):
        """Each App lists its own record of a per_source code; a DELETE clears only that one."""
        for source in (OWNER_A, OWNER_B, OWNER_C):
            self._report(PER_SOURCE_CODE, source)
        self.poll_endpoint_until(
            '/faults?status=all',
            lambda d: d if self._sources_of(d, PER_SOURCE_CODE) == [OWNER_A, OWNER_B, OWNER_C]
            else None,
            timeout=FAULT_TIMEOUT)

        for app in (OWNER_A, OWNER_B, OWNER_C):
            entries = self._entries(f'apps/{app}', PER_SOURCE_CODE)
            self.assertEqual([e['reporting_sources'] for e in entries], [[app]],
                             f'{app}: {entries}')
        self.assertEqual(self._entries(f'apps/{NEAR_A}', PER_SOURCE_CODE, 'all'), [])
        hub = self._entries(f'components/{HUB}', PER_SOURCE_CODE)
        self.assertEqual(len(hub), 1, hub)
        self.assertEqual(len(hub[0]['x-medkit']['records']), 3, hub)

        self.delete_request(f'/apps/{OWNER_A}/faults/{PER_SOURCE_CODE}')
        self._wait_unlisted(f'apps/{OWNER_A}', PER_SOURCE_CODE)
        for app in (OWNER_B, OWNER_C):
            self.assertEqual(len(self._entries(f'apps/{app}', PER_SOURCE_CODE)), 1, app)

    def _open_stream(self, events_url):
        """Read an SSE stream on a thread. Returns (events, stop, thread)."""
        events = []
        stop = threading.Event()

        def read():
            try:
                with requests.get(events_url, stream=True, timeout=(5, 30)) as resp:
                    for line in resp.iter_lines(decode_unicode=True):
                        if stop.is_set():
                            return
                        if line and line.startswith('data:'):
                            events.append(json.loads(line[len('data:'):]))
            except requests.exceptions.RequestException:
                pass

        thread = threading.Thread(target=read, daemon=True)
        thread.start()
        return events, stop, thread

    def _trigger_on(self, app):
        body = {
            'resource': f'{API_BASE_PATH}/apps/{app}/faults',
            'trigger_condition': {'condition_type': 'OnChange'},
            'multishot': True,
            'lifetime': 120,
        }
        r = requests.post(f'{self.BASE_URL}/apps/{app}/triggers', json=body, timeout=5)
        self.assertEqual(r.status_code, 201, r.text)
        trigger = r.json()
        self.addCleanup(requests.delete,
                        f'{self.BASE_URL}/apps/{app}/triggers/{trigger["id"]}', timeout=5)
        return trigger

    def _wait_events(self, events, code, count):
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            matched = [e for e in list(events) if e.get('payload', {}).get('fault_code') == code]
            if len(matched) >= count:
                return matched
            time.sleep(0.1)
        self.fail(f'{count} event(s) of {code} expected, got {list(events)}')

    def test_03_a_trigger_on_an_app_fires_for_a_shared_record_it_reported(self):
        """B confirms the shared record and A joins it: A's trigger fires, C's does not."""
        streams = {}
        for app in (OWNER_A, OWNER_C):
            trigger = self._trigger_on(app)
            events, stop, thread = self._open_stream(
                f'{self.BASE_URL}{trigger["event_source"].removeprefix(API_BASE_PATH)}')
            self.addCleanup(thread.join, 5)
            self.addCleanup(stop.set)
            streams[app] = events

        # B alone: the record is not A's yet. A joins: every later event is A's too.
        self._report(TRIGGER_CODE, OWNER_B)
        self._wait_sources(TRIGGER_CODE, [OWNER_B])
        self._report(TRIGGER_CODE, OWNER_A)
        first = self._wait_events(streams[OWNER_A], TRIGGER_CODE, 1)
        self.assertEqual(sorted(first[0]['payload']['reporting_sources']),
                         [OWNER_A, OWNER_B], first)

        # A second event reaching A means C's trigger saw the same changes.
        self._report(TRIGGER_CODE, OWNER_B)
        self._wait_events(streams[OWNER_A], TRIGGER_CODE, 2)
        self.assertEqual([e for e in list(streams[OWNER_C])
                          if e.get('payload', {}).get('fault_code') == TRIGGER_CODE], [])


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for name in proc_info.process_names():
            self.assertIn(proc_info[name].returncode, ALLOWED_EXIT_CODES)
