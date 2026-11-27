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

"""Integration test: global DELETE /faults honours the lock of every entity that owns a record.

A record belongs to an entity when one of its reporting sources equals one of
the entity's owned sources or is a path child of one: an app's node FQN, an
external app's bare id, an external component's bare id. The global clear skips the record when
any such entity is locked by another client. A name that only starts with an
owned source is not a path child, so its record is cleared.

Hybrid discovery with a manifest: the only way to declare external apps and
components, and two apps bound to one node path (spelled two ways).
"""

import os
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

LIDAR_FQN = '/perception/lidar/lidar_sensor'
EXTERNAL_APP = 'plc-app'
EXTERNAL_COMPONENT = 'plc-rack'
TWIN_SOURCE = '/scope/t/twin'
UNOWNED_SOURCE = '/nobody/owns_this'

MANIFEST_YAML = """\
manifest_version: "1.0"
metadata:
  name: locking-faults-owners
  version: "1.0.0"
config:
  unmanifested_nodes: ignore
areas:
  - id: perception
    name: Perception
    namespace: /perception
  - id: yard
    name: Yard
    namespace: /yard
  - id: twins
    name: Twins
    namespace: /scope
components:
  - id: lidar-unit
    name: Lidar unit
    area: perception
  - id: plc-unit
    name: PLC unit
    area: yard
  - id: plc-rack
    name: PLC rack
    area: yard
    external: true
  - id: twin-unit
    name: Twin unit
    area: twins
apps:
  - id: lidar_sensor
    name: Lidar sensor
    is_located_on: lidar-unit
    ros_binding:
      node_name: lidar_sensor
      namespace: /perception/lidar
  - id: plc-app
    name: PLC app
    external: true
    is_located_on: plc-unit
  - id: twin-a
    name: Twin A
    is_located_on: twin-unit
    ros_binding:
      node_name: twin
      namespace: /scope/t
  # The same node path spelled with a trailing slash: an identical binding
  # would be rejected as a duplicate (R010).
  - id: twin-b
    name: Twin B
    is_located_on: twin-unit
    ros_binding:
      node_name: twin
      namespace: /scope/t/
"""


def _write_manifest():
    fd, path = tempfile.mkstemp(prefix='medkit_locking_faults_owners_', suffix='.yaml')
    os.write(fd, MANIFEST_YAML.encode('utf-8'))
    os.close(fd)
    return path


def generate_test_description():
    """Launch a hybrid gateway with locking, the fault manager and the lidar node."""
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        gateway_params={
            'locking.enabled': True,
            'locking.default_max_expiration': 3600,
            'discovery.mode': 'hybrid',
            'discovery.manifest_path': _write_manifest(),
            'discovery.manifest_strict_validation': False,
        },
        fault_manager=True,
        fault_manager_params={'confirmation_threshold': -2},
        lidar_faulty=False,
    )


class TestLockingFaultsOwners(GatewayTestCase):
    """Global fault clear honours the lock of every entity whose scope holds the owner."""

    MIN_EXPECTED_APPS = 4
    REQUIRED_APPS = {'lidar_sensor', EXTERNAL_APP, 'twin-a', 'twin-b'}

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._reporter = Node('locking_faults_owners_reporter')
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault')
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report(self, fault_code, source):
        """Report FAILED until two reports are answered, so the fault confirms.

        A reply can be lost while the fault manager is still matching this
        client, even though the request arrived; resending is harmless.
        """
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'locking test fault'
        req.source_id = source
        answered = 0
        for _ in range(8):
            future = self._report_client.call_async(req)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self._reporter, timeout_sec=0.1)
            if future.done():
                self.assertTrue(future.result().accepted)
                answered += 1
                if answered == 2:
                    return
        self.fail(f'report_fault {fault_code} answered {answered} of 2 times')

    def _lock(self, entity_path, client_id):
        resp = requests.post(
            f'{self.BASE_URL}{entity_path}/locks',
            json={'lock_expiration': 300},
            headers={'X-Client-Id': client_id},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']
        self.addCleanup(
            requests.delete,
            f'{self.BASE_URL}{entity_path}/locks/{lock_id}',
            headers={'X-Client-Id': client_id},
            timeout=10,
        )

    def _global_codes(self):
        resp = requests.get(f'{self.BASE_URL}/faults', timeout=10)
        self.assertEqual(resp.status_code, 200, resp.text)
        return {f['fault_code'] for f in resp.json().get('items', [])}

    def _assert_global_clear(self, entity_path, spared, cleared):
        """Clear globally as another client: `spared` codes stay, `cleared` codes go.

        `cleared` holds a control fault from a source no entity owns, which
        proves the clear ran. The per-entity DELETE of a spared fault must
        refuse too.
        """
        for fault_code in spared:
            resp = requests.delete(
                f'{self.BASE_URL}{entity_path}/faults/{fault_code}',
                headers={'X-Client-Id': 'client_b'},
                timeout=10,
            )
            self.assertEqual(resp.status_code, 409, resp.text)

        resp = requests.delete(
            f'{self.BASE_URL}/faults',
            headers={'X-Client-Id': 'client_b'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 204, resp.text)

        codes = self._global_codes()
        for fault_code in cleared:
            self.assertNotIn(
                fault_code, codes,
                f'global clear kept {fault_code}, which no locked entity owns')
        for fault_code in spared:
            self.assertIn(
                fault_code, codes,
                f'global clear removed {fault_code} while {entity_path} is locked by another '
                f'client')

    # @verifies REQ_INTEROP_100
    def test_global_clear_skips_subpath_source_of_locked_app(self):
        """Owners below the locked app's FQN are spared, a name sharing only its prefix is not."""
        self._report('LOCK_SUBPATH', f'{LIDAR_FQN}/diagnostics')
        self._report('LOCK_SUBPATH2', f'{LIDAR_FQN}/diagnostics/deep')
        # Starts with the app's FQN but is not a path child of it.
        self._report('LOCK_NAME_PREFIX', f'{LIDAR_FQN}_extra/diag')
        self._report('LOCK_SUBPATH_CONTROL', UNOWNED_SOURCE)
        self.wait_for_fault('/apps/lidar_sensor', 'LOCK_SUBPATH')
        self.wait_for_fault('/apps/lidar_sensor', 'LOCK_SUBPATH2')

        self._lock('/apps/lidar_sensor', 'client_a')
        self._assert_global_clear('/apps/lidar_sensor', ['LOCK_SUBPATH', 'LOCK_SUBPATH2'],
                                  ['LOCK_NAME_PREFIX', 'LOCK_SUBPATH_CONTROL'])

    # @verifies REQ_INTEROP_100
    def test_global_clear_by_the_lock_holder_clears_its_locked_records(self):
        """The lock holder's own global clear removes the records its locks cover."""
        self._report('LOCK_HOLDER', f'{LIDAR_FQN}/holder')
        self._report('LOCK_HOLDER_TWIN', TWIN_SOURCE)
        self.wait_for_fault('/apps/lidar_sensor', 'LOCK_HOLDER')
        self.wait_for_fault('/apps/twin-a', 'LOCK_HOLDER_TWIN')

        self._lock('/apps/lidar_sensor', 'client_a')
        self._lock('/apps/twin-a', 'client_a')
        resp = requests.delete(
            f'{self.BASE_URL}/faults',
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 204, resp.text)

        codes = self._global_codes()
        for fault_code in ('LOCK_HOLDER', 'LOCK_HOLDER_TWIN'):
            self.assertNotIn(
                fault_code, codes,
                f'the global clear of the lock holder kept {fault_code}')

    # @verifies REQ_INTEROP_100
    def test_global_clear_skips_bare_id_source_of_locked_external_app(self):
        """A source equal to a locked external app's id is covered by that app's lock."""
        self._report('LOCK_EXTERNAL', EXTERNAL_APP)
        self._report('LOCK_EXTERNAL_CONTROL', UNOWNED_SOURCE)
        self.wait_for_fault(f'/apps/{EXTERNAL_APP}', 'LOCK_EXTERNAL')

        self._lock(f'/apps/{EXTERNAL_APP}', 'client_a')
        self._assert_global_clear(f'/apps/{EXTERNAL_APP}', ['LOCK_EXTERNAL'],
                                  ['LOCK_EXTERNAL_CONTROL'])

    # @verifies REQ_INTEROP_100
    def test_global_clear_skips_bare_id_source_of_locked_external_component(self):
        """A source equal to a locked external component's id, or below it, is covered."""
        self._report('LOCK_RACK', EXTERNAL_COMPONENT)
        self._report('LOCK_RACK_SLOT', f'{EXTERNAL_COMPONENT}/slot_1')
        self._report('LOCK_RACK_CONTROL', UNOWNED_SOURCE)
        self.wait_for_fault(f'/components/{EXTERNAL_COMPONENT}', 'LOCK_RACK')
        self.wait_for_fault(f'/components/{EXTERNAL_COMPONENT}', 'LOCK_RACK_SLOT')

        self._lock(f'/components/{EXTERNAL_COMPONENT}', 'client_a')
        self._assert_global_clear(f'/components/{EXTERNAL_COMPONENT}',
                                  ['LOCK_RACK', 'LOCK_RACK_SLOT'], ['LOCK_RACK_CONTROL'])

    def _assert_twin_lock_spares(self, twin):
        code = f'LOCK_{twin.upper().replace("-", "_")}'
        control = f'{code}_CONTROL'
        self._report(code, TWIN_SOURCE)
        self._report(control, UNOWNED_SOURCE)
        self.wait_for_fault(f'/apps/{twin}', code)

        self._lock(f'/apps/{twin}', 'client_a')
        self._assert_global_clear(f'/apps/{twin}', [code], [control])

    # @verifies REQ_INTEROP_100
    def test_global_clear_skips_source_of_locked_twin_a(self):
        """Two apps own one source: the lock of the first is honoured."""
        self._assert_twin_lock_spares('twin-a')

    # @verifies REQ_INTEROP_100
    def test_global_clear_skips_source_of_locked_twin_b(self):
        """Two apps own one source: the lock of the second is honoured."""
        self._assert_twin_lock_spares('twin-b')


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
