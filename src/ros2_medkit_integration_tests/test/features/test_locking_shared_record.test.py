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

"""A clear of a shared fault record honours the lock of every app that reported it.

A shared record that apps A and B reported is in the scope of both. While
another client locks B, the per-code DELETE through A or through the component
hosting both answers the same lock refusal a DELETE through B answers, and the
record stays. The bulk DELETE through A skips it. A record only A reported is
not B's and clears as usual. The client holding B's lock clears the record
through A, and so does anyone after the unlock.

@verifies REQ_INTEROP_015
"""

import atexit
import os
import shutil
import tempfile
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
HUB = 'lock-hub'
CELL = 'lock-cell'
LOCKER = 'client_1'
CLEARER = 'client_2'
FAULT_TIMEOUT = 30.0
CONFIG_DIR = tempfile.mkdtemp(prefix='medkit_locking_shared_record_')
atexit.register(shutil.rmtree, CONFIG_DIR, True)
MANIFEST = os.path.join(CONFIG_DIR, 'manifest.yaml')
MANIFEST_TEXT = f"""
manifest_version: "1.0"
metadata:
  name: "locking-shared-record"
  version: "1.0.0"
config:
  unmanifested_nodes: warn
areas:
  - id: {CELL}
    name: "Cell"
    namespace: /lock_cell
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
""" for app in (OWNER_A, OWNER_B, OWNER_C))


def generate_test_description():
    with open(MANIFEST, 'w') as f:
        f.write(MANIFEST_TEXT)
    return create_test_launch(
        demo_nodes=[],
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,
            'snapshots.rosbag.enabled': False,
        },
        gateway_params={
            'locking.enabled': True,
            'locking.default_max_expiration': 3600,
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': MANIFEST,
            'discovery.manifest_strict_validation': False,
        },
    )


class TestLockingSharedRecord(GatewayTestCase):
    """A lock on one reporter of a shared record protects it on every route."""

    MIN_EXPECTED_APPS = 3
    REQUIRED_APPS = {OWNER_A, OWNER_B, OWNER_C}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls._node = Node('locking_shared_record_reporter')
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

    def _raise(self, code, sources):
        """Report `code` from each source and wait for its one record to name them all."""
        for source in sources:
            self._report(code, source)
        self.poll_endpoint_until(
            '/faults?status=all',
            lambda d: d if [sorted(i.get('reporting_sources', [])) for i in d.get('items', [])
                            if i.get('fault_code') == code] == [sorted(sources)] else None,
            timeout=FAULT_TIMEOUT)

    def _status(self, code):
        items = [i for i in self.get_json('/faults?status=all').get('items', [])
                 if i.get('fault_code') == code]
        self.assertEqual(len(items), 1, items)
        return items[0]['status']

    def _lock(self, entity, client):
        resp = requests.post(f'{self.BASE_URL}/{entity}/locks', json={'lock_expiration': 300},
                             headers={'X-Client-Id': client}, timeout=10)
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']
        self.addCleanup(requests.delete, f'{self.BASE_URL}/{entity}/locks/{lock_id}',
                        headers={'X-Client-Id': client}, timeout=10)
        return lock_id

    def _delete(self, path, client):
        return requests.delete(f'{self.BASE_URL}{path}', headers={'X-Client-Id': client},
                               timeout=10)

    def test_01_a_lock_on_one_reporter_refuses_the_clear_through_another(self):
        """B locked by client 1: client 2 clears the shared record through neither A nor hub."""
        shared, only_a, bulk_only_a = 'LOCK_SHARED', 'LOCK_ONLY_A', 'LOCK_BULK_ONLY_A'
        self._raise(shared, [OWNER_A, OWNER_B])
        self._raise(only_a, [OWNER_A])
        self._raise(bulk_only_a, [OWNER_A])
        lock_id = self._lock(f'apps/{OWNER_B}', LOCKER)

        direct = self._delete(f'/apps/{OWNER_B}/faults/{shared}', CLEARER)
        self.assertEqual(direct.status_code, 409, direct.text)
        for entity in (f'apps/{OWNER_A}', f'components/{HUB}'):
            refused = self._delete(f'/{entity}/faults/{shared}', CLEARER)
            self.assertEqual(refused.status_code, 409, f'{entity}: {refused.text}')
            body = refused.json()
            for key in ('error_code', 'vendor_code'):
                self.assertEqual(body.get(key), direct.json().get(key), f'{entity}: {body}')
            self.assertEqual(body['parameters']['entity_id'], OWNER_B, body)
            self.assertEqual(self._status(shared), 'CONFIRMED', entity)

        # A record only A reported is not B's.
        own = self._delete(f'/apps/{OWNER_A}/faults/{only_a}', CLEARER)
        self.assertEqual(own.status_code, 204, own.text)
        self.assertEqual(self._status(only_a), 'CLEARED')

        # The bulk clear through A takes A's own record and skips the shared one.
        bulk = self._delete(f'/apps/{OWNER_A}/faults', CLEARER)
        self.assertEqual(bulk.status_code, 204, bulk.text)
        self.assertEqual(self._status(bulk_only_a), 'CLEARED')
        self.assertEqual(self._status(shared), 'CONFIRMED')

        unlocked = self._delete(f'/apps/{OWNER_B}/locks/{lock_id}', LOCKER)
        self.assertEqual(unlocked.status_code, 204, unlocked.text)
        cleared = self._delete(f'/apps/{OWNER_A}/faults/{shared}', CLEARER)
        self.assertEqual(cleared.status_code, 204, cleared.text)
        self.assertEqual(self._status(shared), 'CLEARED')

    def test_02_the_lock_holder_clears_the_shared_record_through_another_reporter(self):
        """Client 1 holds B's lock, so its clear through A takes the record."""
        shared = 'LOCK_HOLDER_SHARED'
        self._raise(shared, [OWNER_A, OWNER_B])
        self._lock(f'apps/{OWNER_B}', LOCKER)

        cleared = self._delete(f'/apps/{OWNER_A}/faults/{shared}', LOCKER)
        self.assertEqual(cleared.status_code, 204, cleared.text)
        self.assertEqual(self._status(shared), 'CLEARED')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for name in proc_info.process_names():
            self.assertIn(proc_info[name].returncode, ALLOWED_EXIT_CODES)
