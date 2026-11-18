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

"""Fault records of one code under SOVD entities, over the whole HTTP stack.

A fault record is the pair (``fault_code``, owner). A per_source code has one
record per reporting source; any other code has one shared record that every
source reports into. An entity shows each code once:

* An App shows the records of its own source and of its sub-sources
  (``<app>/<suffix>``) as one entry. A Component or an Area shows every record
  of the code under it as one entry.
* An entry of several records is their merge, and ``x-medkit.records`` names
  each record: its App, its source and its status. A shared record is named by
  its ``reporting_sources``, never by an owner.
* An entry of one record is that record, as the global list shows it. A shared
  record looks as it always has: no owner field anywhere.
* DELETE of a code clears every record of it under the entity.
* The global list ``GET /faults`` is not an entity: one item per record.

Every scenario runs for a per_source code and for a shared one.

@verifies REQ_INTEROP_012
@verifies REQ_INTEROP_013
@verifies REQ_INTEROP_014
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

from ament_index_python.packages import get_package_share_directory
import launch_testing
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


OWNER_A = 'owner-a'
OWNER_B = 'owner-b'
# Its id extends OWNER_A's, so a prefix match would give it OWNER_A's sub-sources.
NEAR_A = 'owner-a2'
SUB_X = f'{OWNER_A}/x'
SUB_Y = f'{OWNER_A}/y'
HOST_COMPONENT = 'shared-code-hub'
CELL_AREA = 'shared-code-cell'
# per_source codes. SHARED_CODE keeps its name for the recording cases.
SHARED_CODE = 'SHARED_OVERFLOW'
PER_SOURCE = {
    'app': 'APP_PER_SOURCE',
    'sub': 'SUB_PER_SOURCE',
    'merge': 'MERGE_PER_SOURCE',
    'bulk': 'BULK_PER_SOURCE',
    'global': 'GLOBAL_PER_SOURCE',
    # Reported only in test_08, so every frame of it comes from that test's reports.
    'stream': 'STREAM_PER_SOURCE',
}
# Default codes: one shared record whatever the number of sources.
SHARED = {
    'pair': 'PAIR_SHARED',
    'solo': 'SOLO_SHARED',
    'sub': 'SUB_SHARED',
    'merge': 'MERGE_SHARED',
    'bulk': 'BULK_SHARED',
}
PRIME_CODE = 'OWNER_IDENTITY_PRIME'
STREAM_CODE = PER_SOURCE['stream']
FAULT_TIMEOUT = 30.0
REPORT_TIMEOUT = 2.0
CONFIG_DIR = tempfile.mkdtemp(prefix='medkit_owner_identity_')
atexit.register(shutil.rmtree, CONFIG_DIR, True)
THRESHOLDS_PATH = os.path.join(CONFIG_DIR, 'fault_thresholds.yaml')
# Keys a shared record never carries.
OWNER_KEYS = ('source_id', 'owner', 'x-medkit')


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'fault_owner_identity_manifest.yaml',
    )
    with open(THRESHOLDS_PATH, 'w') as f:
        for code in (SHARED_CODE, *PER_SOURCE.values()):
            f.write(f'{code}:\n  per_source: true\n')
    return create_test_launch(
        demo_nodes=[],
        fault_manager=True,
        # Negative threshold: a record confirms on its second FAILED event, and
        # each owner debounces on its own reports.
        fault_manager_params={
            'confirmation_threshold': -2,
            'fault_thresholds.config_file': THRESHOLDS_PATH,
        },
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
            # Locking off, so the per-code DELETE publishes no lock contract and
            # test_07 reads a 409 on it only if the route declares one itself.
            'locking.enabled': False,
            # The fault stream writes at least once a second, so closing it in
            # test_08 returns at once instead of after a 30 s keepalive.
            'sse.keepalive_interval_sec': 1,
        },
    )


class TestFaultsOwnerIdentity(GatewayTestCase):
    """Each entity shows a code once; the global list shows each record."""

    MIN_EXPECTED_APPS = 3
    REQUIRED_APPS = {OWNER_A, OWNER_B, NEAR_A}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._reporter = Node('fault_owner_identity_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault'
        )
        super().setUpClass()
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report(self, source_id, *, fault_code=SHARED_CODE,
                event_type=ReportFault.Request.EVENT_FAILED, times=4):
        """Report under one source, one request at a time.

        Each report waits for its answer, so a burst cannot overrun the
        service's request queue, which drops the oldest requests. The reply
        path is not discovery-matched by wait_for_service, so an early answer
        can be lost even though the record was written: a report that gets no
        answer in time is not an error. The behavioural assertion is the HTTP
        poll, which retries.
        """
        for _ in range(times):
            req = ReportFault.Request()
            req.fault_code = fault_code
            req.event_type = event_type
            req.severity = Fault.SEVERITY_ERROR
            req.description = f'reported by {source_id}'
            req.source_id = source_id
            future = self._report_client.call_async(req)
            rclpy.spin_until_future_complete(self._reporter, future, timeout_sec=REPORT_TIMEOUT)

    def _records(self, code, *, status='all'):
        """Items of the global list for `code`."""
        body = self.get_json(f'/faults?status={status}')
        return [item for item in body.get('items', []) if item.get('fault_code') == code]

    def _wait_records(self, code, statuses):
        """Wait until the global list holds one record of `code` per expected status.

        `statuses` maps a record's first reporting source to the status it must
        reach. Returns those records keyed the same way.
        """
        deadline = time.monotonic() + FAULT_TIMEOUT
        found = {}
        while time.monotonic() < deadline:
            found = {item['reporting_sources'][0]: item for item in self._records(code)}
            if all(found.get(src, {}).get('status') == want for src, want in statuses.items()):
                return found
            time.sleep(0.2)
        self.fail(f'{code}: records {statuses} not reached; global list has {found}')

    def _raise(self, code, sources, *, confirmed=True):
        times = 4 if confirmed else 1
        for source in sources:
            self._report(source, fault_code=code, times=times)

    def _entries(self, entity, code, *, status=None):
        query = f'?status={status}' if status else ''
        body = self.get_json(f'/{entity}/faults{query}')
        return [item for item in body.get('items', []) if item.get('fault_code') == code]

    def _assert_record_view_of_shared(self, item, sources):
        for key in OWNER_KEYS:
            self.assertNotIn(key, item, f'a shared record carries no {key}: {item}')
        self.assertEqual(sorted(item['reporting_sources']), sorted(sources), item)

    def _assert_detail_of_shared(self, detail, sources):
        self.assertNotIn('owner', detail['x-medkit'], detail)
        self.assertNotIn('records', detail['x-medkit'], detail)
        self.assertEqual(sorted(detail['x-medkit']['reporting_sources']), sorted(sources))

    def _wait_status(self, code, source, want):
        deadline = time.monotonic() + FAULT_TIMEOUT
        status = None
        while time.monotonic() < deadline:
            status = next((item['status'] for item in self._records(code)
                           if source in item['reporting_sources']), None)
            if status == want:
                return
            time.sleep(0.2)
        self.fail(f'{code} of {source}: status {status}, expected {want}')

    def _raise_both(self):
        self._report(OWNER_A)
        self._report(OWNER_B)
        self.wait_for_fault(f'/apps/{OWNER_A}', SHARED_CODE, max_wait=FAULT_TIMEOUT)
        self.wait_for_fault(f'/apps/{OWNER_B}', SHARED_CODE, max_wait=FAULT_TIMEOUT)

    # Method order is alphabetical. Each case raises codes of its own.

    def test_01_a_shared_code_shows_no_owner(self):
        """A shared record looks the same on an App, a Component and the global list.

        @verifies REQ_INTEROP_012
        @verifies REQ_INTEROP_013
        """
        pair, solo = SHARED['pair'], SHARED['solo']
        self._raise(pair, [OWNER_A, OWNER_B])
        self._raise(solo, [OWNER_A])
        self.poll_endpoint_until(
            '/faults', lambda d: d if sum(
                1 for i in d.get('items', [])
                if i.get('fault_code') == pair and sorted(i.get('reporting_sources', []))
                == [OWNER_A, OWNER_B]) == 1 else None, timeout=FAULT_TIMEOUT)
        self.wait_for_fault(f'/apps/{OWNER_A}', solo, max_wait=FAULT_TIMEOUT)

        records = self._records(pair)
        self.assertEqual(len(records), 1, records)
        self._assert_record_view_of_shared(records[0], [OWNER_A, OWNER_B])

        entries = self._entries(f'components/{HOST_COMPONENT}', pair)
        self.assertEqual(len(entries), 1, entries)
        self._assert_record_view_of_shared(entries[0], [OWNER_A, OWNER_B])
        self._assert_detail_of_shared(
            self.get_json(f'/components/{HOST_COMPONENT}/faults/{pair}'), [OWNER_A, OWNER_B])

        # An App holds a shared record only when every source is its own.
        self.assertEqual(self._entries(f'apps/{OWNER_A}', pair), [])
        self.get_json(f'/apps/{OWNER_A}/faults/{pair}', expected_status=404)
        entries = self._entries(f'apps/{OWNER_A}', solo)
        self.assertEqual(len(entries), 1, entries)
        self._assert_record_view_of_shared(entries[0], [OWNER_A])
        self._assert_detail_of_shared(self.get_json(f'/apps/{OWNER_A}/faults/{solo}'), [OWNER_A])

    def test_02_an_app_addresses_the_record_of_its_source(self):
        """Two sources of a per_source code: each App sees, reads and clears its own.

        @verifies REQ_INTEROP_012
        @verifies REQ_INTEROP_013
        @verifies REQ_INTEROP_015
        """
        code = PER_SOURCE['app']
        self._raise(code, [OWNER_A])
        self._raise(code, [OWNER_B], confirmed=False)
        records = self._wait_records(code, {OWNER_A: 'CONFIRMED', OWNER_B: 'PREFAILED'})

        entries = self._entries(f'apps/{OWNER_A}', code)
        self.assertEqual(len(entries), 1, entries)
        self.assertNotIn('source_id', entries[0], 'vendor fields live inside x-medkit only')
        self.assertEqual(entries[0]['reporting_sources'], [OWNER_A])
        self.assertEqual(entries[0]['occurrence_count'], records[OWNER_A]['occurrence_count'])
        self.assertEqual(entries[0]['x-medkit']['records'],
                         [{'app_id': OWNER_A, 'source_id': OWNER_A, 'status': 'CONFIRMED'}])

        detail = self.get_json(f'/apps/{OWNER_A}/faults/{code}')
        self.assertEqual(detail['x-medkit']['owner'], OWNER_A)
        self.assertEqual(detail['x-medkit']['reporting_sources'], [OWNER_A])
        self.assertEqual(detail['x-medkit']['status_raw'], 'CONFIRMED')
        self.assertEqual(detail['x-medkit']['occurrence_count'],
                         records[OWNER_A]['occurrence_count'])
        # B's record is PREFAILED: a detail of B that merged both records or
        # served A's says CONFIRMED.
        detail_b = self.get_json(f'/apps/{OWNER_B}/faults/{code}')
        self.assertEqual(detail_b['x-medkit']['owner'], OWNER_B)
        self.assertEqual(detail_b['x-medkit']['status_raw'], 'PREFAILED')

        self.delete_request(f'/apps/{OWNER_A}/faults/{code}')
        self._wait_status(code, OWNER_A, 'CLEARED')
        self._wait_status(code, OWNER_B, 'PREFAILED')

    def test_03_sub_sources_of_one_app_are_one_entry(self):
        """Records under <app>/<suffix> merge into the App's one entry, not a neighbour's.

        @verifies REQ_INTEROP_012
        @verifies REQ_INTEROP_013
        @verifies REQ_INTEROP_015
        """
        code, shared = PER_SOURCE['sub'], SHARED['sub']
        self._raise(code, [SUB_X])
        self._raise(code, [SUB_Y], confirmed=False)
        self._raise(shared, [SUB_X, SUB_Y])
        records = self._wait_records(code, {SUB_X: 'CONFIRMED', SUB_Y: 'PREFAILED'})
        self.wait_for_fault(f'/apps/{OWNER_A}', shared, max_wait=FAULT_TIMEOUT)
        total = sum(r['occurrence_count'] for r in records.values())

        entries = self._entries(f'apps/{OWNER_A}', code)
        self.assertEqual(len(entries), 1, entries)
        entry = entries[0]
        self.assertEqual(entry['status'], 'CONFIRMED')
        self.assertEqual(entry['occurrence_count'], total)
        self.assertEqual(entry['reporting_sources'], [SUB_X, SUB_Y])
        self.assertNotIn('source_id', entry)
        self.assertEqual(entry['x-medkit']['records'], [
            {'app_id': OWNER_A, 'source_id': SUB_X, 'status': 'CONFIRMED'},
            {'app_id': OWNER_A, 'source_id': SUB_Y, 'status': 'PREFAILED'},
        ])

        detail = self.get_json(f'/apps/{OWNER_A}/faults/{code}')
        self.assertEqual(detail['item']['status']['aggregatedStatus'], 'active')
        self.assertEqual(detail['x-medkit']['occurrence_count'], total)
        self.assertEqual(detail['x-medkit']['status_raw'], 'CONFIRMED')
        self.assertNotIn('owner', detail['x-medkit'])
        self.assertEqual(len(detail['x-medkit']['records']), 2, detail)

        shared_entries = self._entries(f'apps/{OWNER_A}', shared)
        self.assertEqual(len(shared_entries), 1, shared_entries)
        self._assert_record_view_of_shared(shared_entries[0], [SUB_X, SUB_Y])

        for listed in (code, shared):
            self.assertEqual(self._entries(f'apps/{NEAR_A}', listed, status='all'), [],
                             f'{NEAR_A} must not list a record under {OWNER_A}')

        self.delete_request(f'/apps/{OWNER_A}/faults/{code}')
        self._wait_status(code, SUB_X, 'CLEARED')
        self._wait_status(code, SUB_Y, 'CLEARED')

    def test_04_a_component_and_an_area_merge_every_app(self):
        """Three Apps and a sub-source: one entry per code on the Component and the Area.

        @verifies REQ_INTEROP_012
        @verifies REQ_INTEROP_013
        @verifies REQ_INTEROP_015
        """
        code, shared = PER_SOURCE['merge'], SHARED['merge']
        sources = [OWNER_A, OWNER_B, NEAR_A, SUB_X]
        self._raise(code, sources)
        self._raise(shared, sources)
        records = self._wait_records(code, dict.fromkeys(sources, 'CONFIRMED'))
        self.poll_endpoint_until(
            f'/components/{HOST_COMPONENT}/faults', lambda d: d if any(
                i.get('fault_code') == shared and len(i.get('reporting_sources', [])) == 4
                for i in d.get('items', [])) else None, timeout=FAULT_TIMEOUT)
        total = sum(r['occurrence_count'] for r in records.values())
        expected_records = sorted([
            {'app_id': OWNER_A, 'source_id': OWNER_A, 'status': 'CONFIRMED'},
            {'app_id': OWNER_A, 'source_id': SUB_X, 'status': 'CONFIRMED'},
            {'app_id': OWNER_B, 'source_id': OWNER_B, 'status': 'CONFIRMED'},
            {'app_id': NEAR_A, 'source_id': NEAR_A, 'status': 'CONFIRMED'},
        ], key=lambda r: r['source_id'])

        for entity in (f'components/{HOST_COMPONENT}', f'areas/{CELL_AREA}'):
            entries = self._entries(entity, code)
            self.assertEqual(len(entries), 1, f'{entity}: {entries}')
            self.assertEqual(entries[0]['occurrence_count'], total, entity)
            self.assertEqual(entries[0]['reporting_sources'], sorted(sources), entity)
            self.assertEqual(sorted(entries[0]['x-medkit']['records'],
                                    key=lambda r: r['source_id']), expected_records, entity)
            shared_entries = self._entries(entity, shared)
            self.assertEqual(len(shared_entries), 1, f'{entity}: {shared_entries}')
            self._assert_record_view_of_shared(shared_entries[0], sources)

        detail = self.get_json(f'/components/{HOST_COMPONENT}/faults/{code}')
        self.assertEqual(detail['x-medkit']['occurrence_count'], total)
        self.assertEqual(len(detail['x-medkit']['records']), 4, detail)
        self._assert_detail_of_shared(
            self.get_json(f'/areas/{CELL_AREA}/faults/{shared}'), sources)

        self.delete_request(f'/components/{HOST_COMPONENT}/faults/{code}')
        for source in sources:
            self._wait_status(code, source, 'CLEARED')
        self.delete_request(f'/areas/{CELL_AREA}/faults/{shared}')
        self._wait_status(shared, OWNER_B, 'CLEARED')

    def test_05_the_bulk_clear_takes_every_record_under_the_entity(self):
        """DELETE /apps/{id}/faults clears its records and its sub-sources', nobody else's.

        @verifies REQ_INTEROP_014
        """
        code, shared = PER_SOURCE['bulk'], SHARED['bulk']
        self._raise(code, [OWNER_A, SUB_X, OWNER_B])
        self._raise(shared, [OWNER_A])
        self._wait_records(code, {OWNER_A: 'CONFIRMED', SUB_X: 'CONFIRMED', OWNER_B: 'CONFIRMED'})
        self._wait_records(shared, {OWNER_A: 'CONFIRMED'})

        self.delete_request(f'/apps/{OWNER_A}/faults')

        self._wait_status(code, OWNER_A, 'CLEARED')
        self._wait_status(code, SUB_X, 'CLEARED')
        self._wait_status(shared, OWNER_A, 'CLEARED')
        self._wait_status(code, OWNER_B, 'CONFIRMED')

    def test_06_the_global_list_shows_one_item_per_record(self):
        """GET /faults: a record of a per_source code names its App and source.

        @verifies REQ_INTEROP_012
        """
        code = PER_SOURCE['global']
        self._raise(code, [OWNER_A, OWNER_B])
        records = self._wait_records(code, {OWNER_A: 'CONFIRMED', OWNER_B: 'CONFIRMED'})

        self.assertEqual(len(self._records(code)), 2)
        for source, item in records.items():
            self.assertNotIn('source_id', item, 'vendor fields live inside x-medkit only')
            self.assertEqual(item['x-medkit']['records'],
                             [{'app_id': source, 'source_id': source, 'status': 'CONFIRMED'}])
            self.assertNotIn('owner', item)

    def test_07_the_openapi_document_declares_the_record_contract(self):
        """The served spec says what the routes now do.

        @verifies REQ_INTEROP_013
        """
        spec = self.poll_endpoint_until('/docs', lambda d: d if 'openapi' in d else None)

        item = spec['components']['schemas']['FaultListItem']
        self.assertIn('x-medkit', item['properties'],
                      'the published fault item must declare x-medkit')
        self.assertNotIn('source_id', item['properties'],
                         'a fault item carries vendor fields only inside x-medkit')

        route = spec['paths']['/apps/{app_id}/faults/{fault_code}']
        # With locking off the DELETE carries no lock contract, so a 409 on
        # either method would be the route's own.
        root = self.get_json('')
        self.assertIs(
            root['capabilities']['locking'], False,
            'this launch must run with locking off, or the DELETE 409 below may be the lock one',
        )
        self.assertNotIn('x-medkit-lock-guarded', route['delete'])
        for method in ('get', 'delete'):
            self.assertNotIn(
                '409', route[method]['responses'],
                f'the per-entity fault {method} never answers 409 for a code',
            )

    def test_076_every_recording_download_declares_its_409(self):
        """The download routes answer 409 on an ambiguous code and say so.

        A GET carries no lock marker, so a 409 on these operations is there
        only because the route declares the ambiguous-fault refusal.

        @verifies REQ_INTEROP_072
        """
        spec = self.poll_endpoint_until('/docs', lambda d: d if 'openapi' in d else None)

        downloads = {
            path: item['get'] for path, item in spec['paths'].items()
            if path.endswith('/bulk-data/{category_id}/{file_id}') and 'get' in item
        }
        # One per entity type that serves bulk data, so the loop below cannot
        # pass by matching nothing.
        self.assertEqual(len(downloads), 6, sorted(downloads))
        for path, operation in sorted(downloads.items()):
            self.assertIn(
                '409', operation['responses'],
                f'GET {path} answers 409 x-medkit-ambiguous-fault and has to declare it',
            )

    def test_077_a_bare_code_bag_url_refuses_to_pick_an_owner(self):
        """A recording URL carrying a bare fault code names no single record.

        Two owners of one code in the component's scope means the compatibility
        URL (the pre-recording-id form, which carries a fault code) addresses
        neither record, and each owner keeps its own recordings. Serving the
        lowest-sorting owner's bytes under that URL would never say whose they
        were, so it answers the same 409 the fault routes do. Rosbag capture is
        off in this launch, so the assertion is on the refusal the resolution
        makes before any bag is looked up.

        @verifies REQ_INTEROP_072
        """
        self._raise_both()

        response = requests.get(
            f'{self.BASE_URL}/components/{HOST_COMPONENT}/bulk-data/rosbags/{SHARED_CODE}',
            timeout=10,
        )

        self.assertEqual(response.status_code, 409, response.text)
        body = response.json()
        self.assertEqual(body.get('vendor_code'), 'x-medkit-ambiguous-fault', body)
        self.assertEqual(
            sorted(body['parameters']['owners']), [OWNER_A, OWNER_B], body)

        # One owner in scope is one record, so that entity's own URL is not
        # ambiguous - it 404s only because no bag was captured.
        single = requests.get(
            f'{self.BASE_URL}/apps/{OWNER_A}/bulk-data/rosbags/{SHARED_CODE}', timeout=10
        )
        self.assertNotEqual(
            single.status_code, 409,
            'one owner in scope must not read as ambiguous',
        )

    def test_08_stream_frames_name_the_record_they_describe(self):
        """Each frame carries the entity hint of its own record's owner."""
        frames = []
        stop_event = threading.Event()
        response = requests.get(
            f'{self.BASE_URL}/faults/stream', stream=True, timeout=(5, 60)
        )
        self.assertEqual(response.status_code, 200)
        pump = threading.Thread(
            target=self._pump_stream, args=(response, frames, stop_event), daemon=True
        )
        pump.start()
        try:
            self._prime_stream(frames)
            self._report(OWNER_A, fault_code=STREAM_CODE)
            self._report(OWNER_B, fault_code=STREAM_CODE)

            seen = {}
            deadline = time.monotonic() + FAULT_TIMEOUT
            while time.monotonic() < deadline and len(seen) < 2:
                for frame in list(frames):
                    data = frame.get('data')
                    if data is None:
                        continue
                    payload = json.loads(data)
                    fault = payload.get('fault', {})
                    if fault.get('fault_code') != STREAM_CODE:
                        continue
                    hint = payload.get('x-medkit')
                    self.assertIsNotNone(
                        hint, f'a frame for {STREAM_CODE} carried no entity hint: {payload}'
                    )
                    self.assertEqual(
                        hint['entity_id'], fault.get('source_id'),
                        'the hint must name the record the frame describes',
                    )
                    seen[hint['entity_id']] = hint
                time.sleep(0.2)

            self.assertEqual(
                sorted(seen), [OWNER_A, OWNER_B],
                f'both owners must appear on the stream, saw: {sorted(seen)}',
            )
            for hint in seen.values():
                self.assertEqual(hint['entity_type'], 'apps')
        finally:
            stop_event.set()
            response.close()
            pump.join(timeout=5)

    @staticmethod
    def _pump_stream(response, frames, stop_event):
        """Collect SSE frames as dicts of field -> value."""
        current = {}
        try:
            for line in response.iter_lines(decode_unicode=True):
                if stop_event.is_set():
                    break
                if line is None:
                    continue
                if line == '':
                    if current:
                        frames.append(current)
                        current = {}
                    continue
                if line.startswith(':'):
                    continue  # keepalive comment
                key, _, value = line.partition(':')
                current[key.strip()] = value.strip()
        except Exception:  # noqa: BLE001 - closed socket on test teardown
            pass

    def _prime_stream(self, frames):
        """Block until the fault event pipeline demonstrably reaches the stream.

        /fault_manager/events is reliable but volatile: an event published
        before the gateway's subscription has matched the fault manager's
        publisher is lost outright. Repeating a sacrificial record until one of
        its frames arrives proves service -> fault manager -> events -> SSE.
        """
        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            self._report(OWNER_A, fault_code=PRIME_CODE, times=2)
            settle = min(time.monotonic() + 1.0, deadline)
            while time.monotonic() < settle:
                for frame in list(frames):
                    data = frame.get('data')
                    if data is None:
                        continue
                    if json.loads(data).get('fault', {}).get('fault_code') == PRIME_CODE:
                        return
                time.sleep(0.1)
        raise AssertionError(
            f'no event for priming fault {PRIME_CODE} on /faults/stream within '
            f'{FAULT_TIMEOUT}s, events pipeline never went live'
        )


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    """All processes exited cleanly (SIGTERM allowed for SSE teardown)."""

    def test_exit_codes(self, proc_info):
        for process_name in proc_info.process_names():
            self.assertIn(
                proc_info[process_name].returncode, ALLOWED_EXIT_CODES,
                f'{process_name} exited with {proc_info[process_name].returncode}',
            )
