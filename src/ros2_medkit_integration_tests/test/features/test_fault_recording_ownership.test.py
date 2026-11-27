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

"""A fault recording follows the record it was captured for, over the whole HTTP stack.

An entity lists a recording when one of the records it covers is in the
entity's scope, and serves every recording it lists on every route: by id, its
manifest, its files and the address by fault code, which serves the newest
recording among the entity's records of that code. An entity holding none of
those records lists nothing and answers 404 on every route.

@verifies REQ_INTEROP_071
@verifies REQ_INTEROP_072
"""

import atexit
import os
import shutil
import tempfile
import time
import unittest

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
OWNER_C = 'owner-c'
NEAR_A = 'owner-a2'
SUB_X = f'{OWNER_A}/x'
SUB_Y = f'{OWNER_A}/y'
HUB = 'recording-hub'
CELL = 'recording-cell'
SHARED_CODE = 'RECORDING_SHARED'
SEPARATE_CODE = 'RECORDING_SEPARATE'
BURST_CODE = 'RECORDING_BURST'
SUB_CODE = 'RECORDING_SUB'
PER_SOURCE_CODES = (SEPARATE_CODE, BURST_CODE, SUB_CODE)
# The post-fault window, long enough for a second confirmation to join a recording.
POST_FAULT_SEC = 2.0
RECORDING_TIMEOUT = 40.0
APPS = (OWNER_A, OWNER_B, OWNER_C, NEAR_A)
ENTITIES = tuple(f'apps/{app}' for app in APPS) + (f'components/{HUB}', f'areas/{CELL}')
CONFIG_DIR = tempfile.mkdtemp(prefix='medkit_recording_ownership_')
atexit.register(shutil.rmtree, CONFIG_DIR, True)
MANIFEST = os.path.join(CONFIG_DIR, 'manifest.yaml')
THRESHOLDS = os.path.join(CONFIG_DIR, 'fault_thresholds.yaml')
MANIFEST_TEXT = f"""
manifest_version: "1.0"
metadata:
  name: "recording-ownership"
  version: "1.0.0"
config:
  unmanifested_nodes: warn
  inherit_runtime_resources: true
areas:
  - id: {CELL}
    name: "Cell"
    namespace: /recording_cell
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
""" for app in APPS)


def generate_test_description():
    with open(MANIFEST, 'w') as f:
        f.write(MANIFEST_TEXT)
    with open(THRESHOLDS, 'w') as f:
        for code in PER_SOURCE_CODES:
            f.write(f'{code}:\n  per_source: true\n')
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,
            'fault_thresholds.config_file': THRESHOLDS,
            'snapshots.rosbag.duration_after_sec': POST_FAULT_SEC,
        },
        gateway_params={
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': MANIFEST,
            'discovery.manifest_strict_validation': False,
        },
    )


class TestFaultRecordingOwnership(GatewayTestCase):
    """Every recording an entity lists downloads through it, and no other entity's does."""

    MIN_EXPECTED_APPS = 4
    REQUIRED_APPS = set(APPS)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._node = Node('recording_ownership_reporter')
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

    def _listed(self, entity, code):
        """Ids of the recordings of `code` the entity lists, newest first."""
        r = requests.get(f'{self.BASE_URL}/{entity}/bulk-data/rosbags', timeout=10)
        self.assertEqual(r.status_code, 200, f'{entity}: {r.text}')
        items = [i for i in r.json().get('items', [])
                 if code in i.get('x-medkit', {}).get('fault_codes', [])]
        items.sort(key=lambda i: i.get('creation_date', ''), reverse=True)
        return [i['id'] for i in items]

    def _wait_listed(self, code, count):
        """Wait until the component lists `count` recordings of `code`, windows closed."""
        deadline = time.monotonic() + RECORDING_TIMEOUT
        while time.monotonic() < deadline:
            ids = self._listed(f'components/{HUB}', code)
            if len(ids) >= count:
                self.assertEqual(len(ids), count, ids)
                return ids
            time.sleep(0.5)
        self.fail(f'{code}: {count} recording(s) expected under {HUB}')

    def _get(self, path):
        return requests.get(f'{self.BASE_URL}{path}', timeout=15)

    def _routes(self, entity, recording_id):
        base = f'/{entity}/bulk-data/rosbags/{recording_id}'
        return (base, f'{base}/manifest', f'{base}/files/0')

    def _assert_serves(self, entity, recording_id):
        for path in self._routes(entity, recording_id):
            self.assertEqual(self._get(path).status_code, 200, path)

    def _assert_refuses(self, entity, recording_id):
        for path in self._routes(entity, recording_id):
            self.assertEqual(self._get(path).status_code, 404, path)

    def _assert_by_code(self, entity, code, recording_id):
        """Assert the address by fault code serves `recording_id`, or 404 when it is None."""
        r = self._get(f'/{entity}/bulk-data/rosbags/{code}')
        if recording_id is None:
            self.assertEqual(r.status_code, 404, f'{entity} {code}: {r.text[:200]}')
            return
        self.assertEqual(r.status_code, 200, f'{entity} {code}: {r.text[:200]}')
        self.assertIn(recording_id, r.headers.get('Content-Disposition', ''), f'{entity} {code}')

    def _assert_every_listing_downloads(self, code):
        """Each id a listing shows downloads through the entity that listed it."""
        for entity in ENTITIES:
            for recording_id in self._listed(entity, code):
                self._assert_serves(entity, recording_id)

    def test_01_a_shared_record_s_recording_follows_every_reporter(self):
        """Listed and served under A, B, the component and the area; C and A2 list none."""
        self._report(SHARED_CODE, OWNER_A)
        self._report(SHARED_CODE, OWNER_B)
        [recording] = self._wait_listed(SHARED_CODE, 1)

        for entity in (f'apps/{OWNER_A}', f'apps/{OWNER_B}', f'components/{HUB}', f'areas/{CELL}'):
            self.assertEqual(self._listed(entity, SHARED_CODE), [recording], entity)
            self._assert_serves(entity, recording)
            self._assert_by_code(entity, SHARED_CODE, recording)
        for app in (OWNER_C, NEAR_A):
            self.assertEqual(self._listed(f'apps/{app}', SHARED_CODE), [], app)
            self._assert_refuses(f'apps/{app}', recording)
            self._assert_by_code(f'apps/{app}', SHARED_CODE, None)
        self._assert_every_listing_downloads(SHARED_CODE)

    def test_02_per_source_records_keep_their_own_recordings(self):
        """A serves A's recording, B serves B's, the component both; neither App the other's."""
        self._report(SEPARATE_CODE, OWNER_A)
        [of_a] = self._wait_listed(SEPARATE_CODE, 1)
        self._report(SEPARATE_CODE, OWNER_B)
        of_b = next(i for i in self._wait_listed(SEPARATE_CODE, 2) if i != of_a)

        for app, own, other in ((OWNER_A, of_a, of_b), (OWNER_B, of_b, of_a)):
            entity = f'apps/{app}'
            self.assertEqual(self._listed(entity, SEPARATE_CODE), [own], entity)
            self._assert_serves(entity, own)
            self._assert_refuses(entity, other)
            self._assert_by_code(entity, SEPARATE_CODE, own)
        self.assertEqual(sorted(self._listed(f'components/{HUB}', SEPARATE_CODE)),
                         sorted([of_a, of_b]))
        self._assert_serves(f'components/{HUB}', of_a)
        self._assert_serves(f'components/{HUB}', of_b)
        self._assert_by_code(f'components/{HUB}', SEPARATE_CODE, of_b)
        for app in (OWNER_C, NEAR_A):
            self.assertEqual(self._listed(f'apps/{app}', SEPARATE_CODE), [], app)
            self._assert_refuses(f'apps/{app}', of_a)
            self._assert_refuses(f'apps/{app}', of_b)
        self._assert_every_listing_downloads(SEPARATE_CODE)

    def test_03_two_records_sharing_one_recording_both_serve_it(self):
        """A and B confirm inside one post-fault window: one recording, served by each."""
        self._report(BURST_CODE, OWNER_A)
        self._report(BURST_CODE, OWNER_B)
        [recording] = self._wait_listed(BURST_CODE, 1)

        for entity in (f'apps/{OWNER_A}', f'apps/{OWNER_B}', f'components/{HUB}'):
            self.assertEqual(self._listed(entity, BURST_CODE), [recording], entity)
            self._assert_serves(entity, recording)
            self._assert_by_code(entity, BURST_CODE, recording)
        self._assert_refuses(f'apps/{OWNER_C}', recording)
        self._assert_every_listing_downloads(BURST_CODE)

    def test_04_records_below_an_app_are_the_app_s(self):
        """Records of owner-a/x and owner-a/y are A's; the near-miss A2 holds neither."""
        self._report(SUB_CODE, SUB_X)
        [older] = self._wait_listed(SUB_CODE, 1)
        self._report(SUB_CODE, SUB_Y)
        newer = next(i for i in self._wait_listed(SUB_CODE, 2) if i != older)

        entity = f'apps/{OWNER_A}'
        self.assertEqual(sorted(self._listed(entity, SUB_CODE)), sorted([older, newer]))
        self._assert_serves(entity, older)
        self._assert_serves(entity, newer)
        self._assert_by_code(entity, SUB_CODE, newer)
        for app in (NEAR_A, OWNER_B):
            self.assertEqual(self._listed(f'apps/{app}', SUB_CODE), [], app)
            self._assert_refuses(f'apps/{app}', older)
            self._assert_refuses(f'apps/{app}', newer)
            self._assert_by_code(f'apps/{app}', SUB_CODE, None)
        self._assert_every_listing_downloads(SUB_CODE)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for name in proc_info.process_names():
            self.assertIn(proc_info[name].returncode, ALLOWED_EXIT_CODES)
