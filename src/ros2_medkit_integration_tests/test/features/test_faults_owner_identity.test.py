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

"""Two owners of one fault code, over the whole HTTP stack.

A fault record is the pair (``fault_code``, reporting source). The source is the
``source_id`` the reporter used and it owns the record, so two apps reporting one
code are two records with their own status, timestamps and clear.

The unit layers cover the pieces separately: the record selection and its 404 /
409 mapping (``SelectScopedFaultTest``), the scope resolution
(``ResolveEntitySourceFqnsTest``), the wire fields (``FaultListItemSchema``) and
the owner reaching the service request (``FaultManagerTest.*SendsTheOwningSource*``).
None of them runs a real fault manager behind a real gateway, which is where the
model either holds or collapses back to one record per code.

The topology is two external apps under one component, so both records land in
one component's fault scope. What this pins:

* ``GET /faults`` lists two items with the same code and distinct ``source_id``.
* Each app's own detail route serves its own record and nothing else.
* ``DELETE`` on one app's record leaves the other app's record CONFIRMED. Under
  a code-keyed store one clear took both.
* The component owns both, so its per-code detail route names neither: ``409``
  ``x-medkit-ambiguous-fault`` listing both owners, instead of serving whichever
  record the store happened to list first.
* ``DELETE /components/<host>/faults`` clears both, one record at a time.
* Every stream frame carries the entity hint for the record it describes, and
  the two records resolve to their own owners.
* The served OpenAPI declares what the routes now do.

@verifies REQ_INTEROP_012
@verifies REQ_INTEROP_013
@verifies REQ_INTEROP_014
@verifies REQ_INTEROP_015
"""

import json
import os
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
HOST_COMPONENT = 'shared-code-hub'
SHARED_CODE = 'SHARED_OVERFLOW'
PRIME_CODE = 'OWNER_IDENTITY_PRIME'
FAULT_TIMEOUT = 30.0


def generate_test_description():
    manifest_path = os.path.join(
        get_package_share_directory('ros2_medkit_gateway'),
        'config', 'examples', 'fault_owner_identity_manifest.yaml',
    )
    return create_test_launch(
        demo_nodes=[],
        fault_manager=True,
        # Negative threshold: a record confirms after a couple of FAILED events,
        # and each owner debounces on its own reports.
        fault_manager_params={'confirmation_threshold': -2},
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': manifest_path,
            'discovery.manifest_strict_validation': False,
        },
    )


class TestFaultsOwnerIdentity(GatewayTestCase):
    """Two sources reporting one code are two independently addressable records."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {OWNER_A, OWNER_B}

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
        """Report under one owner, fire and forget.

        Same shape as the production fault_reporter client: the reply path is
        not discovery-matched by wait_for_service, so an early round trip can
        lose its response even though the record was written. The behavioural
        assertion is the HTTP poll, which retries.
        """
        for _ in range(times):
            req = ReportFault.Request()
            req.fault_code = fault_code
            req.event_type = event_type
            req.severity = Fault.SEVERITY_ERROR
            req.description = f'reported by {source_id}'
            req.source_id = source_id
            self._report_client.call_async(req)
            rclpy.spin_once(self._reporter, timeout_sec=0.1)

    def _raise_both(self):
        self._report(OWNER_A)
        self._report(OWNER_B)
        self.wait_for_fault(f'/apps/{OWNER_A}', SHARED_CODE, max_wait=FAULT_TIMEOUT)
        self.wait_for_fault(f'/apps/{OWNER_B}', SHARED_CODE, max_wait=FAULT_TIMEOUT)

    def _records_of_shared_code(self):
        body = self.get_json('/faults')
        items = body.get('items', body.get('faults', []))
        return [item for item in items if item.get('fault_code') == SHARED_CODE]

    def _status_on(self, app_id):
        """Status of that app's own record, or None when it holds none."""
        body = self.get_json(f'/apps/{app_id}/faults?status=all')
        for item in body.get('items', []):
            if item.get('fault_code') == SHARED_CODE:
                return item.get('status')
        return None

    # Method order is alphabetical, and the clearing cases consume the records
    # the reading cases assert on, so the names carry the order.

    def test_01_the_global_list_shows_one_item_per_owner(self):
        """One code, two sources, two items - each naming its own owner.

        @verifies REQ_INTEROP_012
        """
        self._raise_both()

        records = self._records_of_shared_code()

        self.assertEqual(
            len(records), 2,
            f'two sources reporting {SHARED_CODE} must be two records, got: {records}',
        )
        owners = sorted(r.get('source_id') for r in records)
        self.assertEqual(owners, [OWNER_A, OWNER_B])
        for record in records:
            self.assertEqual(
                record.get('reporting_sources'), [record.get('source_id')],
                'a record names exactly its own owner',
            )

    def test_02_each_app_detail_serves_its_own_record(self):
        """The app route addresses the record that app owns.

        @verifies REQ_INTEROP_013
        """
        self._raise_both()

        for owner in (OWNER_A, OWNER_B):
            detail = self.get_json(f'/apps/{owner}/faults/{SHARED_CODE}')
            self.assertEqual(detail['item']['code'], SHARED_CODE)
            self.assertEqual(
                detail['x-medkit']['owner'], owner,
                f"/apps/{owner} served another owner's record",
            )
            self.assertNotIn(
                'source_id', detail['x-medkit'],
                "the detail must not reuse the list's source_id key for the owner",
            )
            self.assertEqual(detail['x-medkit']['reporting_sources'], [owner])

    def test_03_the_component_detail_refuses_to_pick_an_owner(self):
        """Both records are in the component's scope, so the code names neither.

        @verifies REQ_INTEROP_013
        """
        self._raise_both()

        response = requests.get(
            f'{self.BASE_URL}/components/{HOST_COMPONENT}/faults/{SHARED_CODE}',
            timeout=10,
        )

        self.assertEqual(response.status_code, 409, response.text)
        body = response.json()
        self.assertEqual(body.get('vendor_code', body.get('error_code')),
                         'x-medkit-ambiguous-fault', body)
        owners = sorted(body.get('parameters', body).get('owners', []))
        self.assertEqual(owners, [OWNER_A, OWNER_B], body)

    def test_04_the_component_list_still_shows_both(self):
        """Ambiguous to address by code is not invisible: the list has both.

        @verifies REQ_INTEROP_012
        """
        self._raise_both()

        body = self.get_json(f'/components/{HOST_COMPONENT}/faults')
        owners = sorted(
            item.get('source_id') for item in body.get('items', [])
            if item.get('fault_code') == SHARED_CODE
        )

        self.assertEqual(owners, [OWNER_A, OWNER_B], body)

    def test_05_clearing_one_owners_record_leaves_the_other(self):
        """The clear addresses one record. Under a code key it took both.

        @verifies REQ_INTEROP_015
        """
        self._raise_both()

        self.delete_request(f'/apps/{OWNER_A}/faults/{SHARED_CODE}')

        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            if self._status_on(OWNER_A) == 'CLEARED':
                break
            time.sleep(0.2)
        self.assertEqual(self._status_on(OWNER_A), 'CLEARED')
        self.assertEqual(
            self._status_on(OWNER_B), 'CONFIRMED',
            f"clearing {OWNER_A}'s record must not touch {OWNER_B}'s record of the same code",
        )

    def test_06_the_component_bulk_clear_takes_every_record(self):
        """Per-entity DELETE clears each in-scope record with its own owner.

        @verifies REQ_INTEROP_014
        """
        self._raise_both()

        self.delete_request(f'/components/{HOST_COMPONENT}/faults')

        deadline = time.monotonic() + FAULT_TIMEOUT
        while time.monotonic() < deadline:
            if (self._status_on(OWNER_A) == 'CLEARED'
                    and self._status_on(OWNER_B) == 'CLEARED'):
                break
            time.sleep(0.2)
        self.assertEqual(self._status_on(OWNER_A), 'CLEARED')
        self.assertEqual(
            self._status_on(OWNER_B), 'CLEARED',
            'the bulk clear stopped after one record of the shared code',
        )

    def test_07_the_openapi_document_declares_the_record_contract(self):
        """The served spec says what the routes now do.

        @verifies REQ_INTEROP_013
        """
        spec = self.poll_endpoint_until('/docs', lambda d: d if 'openapi' in d else None)

        item = spec['components']['schemas']['FaultListItem']
        self.assertIn(
            'source_id', item['properties'],
            'the published fault item must declare the owner it carries',
        )

        route = spec['paths']['/apps/{app_id}/faults/{fault_code}']
        # The GET is the discriminating half: it carries no lock marker, so its
        # 409 is there only because the route declares the ambiguous-fault one.
        # The DELETE is lock-guarded, and that marker declares a 409 of its own
        # for the locked-entity refusal, so its status alone cannot tell the two
        # causes apart - the document has one response object per status. The
        # DELETE assertion is a presence check, not a proof of which 409.
        for method in ('get', 'delete'):
            self.assertIn(
                '409', route[method]['responses'],
                f'the per-entity fault {method} answers 409 on an ambiguous code '
                'and has to declare it',
            )

    def test_075_the_detail_names_the_owner_and_not_the_lists_source_id(self):
        """x-medkit.owner is the record's owner, source_id belongs to a list.

        @verifies REQ_INTEROP_013
        """
        self._raise_both()

        detail = self.get_json(f'/apps/{OWNER_A}/faults/{SHARED_CODE}')

        self.assertEqual(detail['x-medkit']['owner'], OWNER_A)
        self.assertNotIn('source_id', detail['x-medkit'])

        # The list-level key is the other meaning, and it is still there.
        listing = self.get_json(f'/apps/{OWNER_A}/faults')
        self.assertIn('source_id', listing['x-medkit'])

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
        makes before any bag is looked up, which is exactly the branch Z6 names.

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
            self._report(OWNER_A, fault_code=SHARED_CODE)
            self._report(OWNER_B, fault_code=SHARED_CODE)

            seen = {}
            deadline = time.monotonic() + FAULT_TIMEOUT
            while time.monotonic() < deadline and len(seen) < 2:
                for frame in list(frames):
                    data = frame.get('data')
                    if data is None:
                        continue
                    payload = json.loads(data)
                    fault = payload.get('fault', {})
                    if fault.get('fault_code') != SHARED_CODE:
                        continue
                    hint = payload.get('x-medkit')
                    self.assertIsNotNone(
                        hint, f'a frame for {SHARED_CODE} carried no entity hint: {payload}'
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
