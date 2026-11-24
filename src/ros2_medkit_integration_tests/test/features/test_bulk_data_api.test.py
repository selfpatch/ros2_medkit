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

"""Feature tests for bulk-data API.

Validates bulk-data category listing, descriptor listing, empty results,
unknown categories, download 404, and nested entity paths.

NOTE: Tests that require rosbag downloads (test_127, test_129, test_137)
are in scenario tests, not here.

"""

import atexit
import os
import shutil
import signal
import tempfile
import time
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from rclpy.node import Node
import requests
from ros2_medkit_msgs.msg import Fault
from ros2_medkit_msgs.srv import ReportFault

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, get_time_scale, ROSBAG_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

# The fault manager writes bag directories here, so a test can reach the files of a recording.
ROSBAG_DIR = tempfile.mkdtemp(prefix='medkit_bulk_data_rosbags_')
atexit.register(shutil.rmtree, ROSBAG_DIR, True)
CONFIG_DIR = tempfile.mkdtemp(prefix='medkit_bulk_data_config_')
atexit.register(shutil.rmtree, CONFIG_DIR, True)
THRESHOLDS_PATH = os.path.join(CONFIG_DIR, 'fault_thresholds.yaml')

LIDAR_FQN = '/perception/lidar/lidar_sensor'
REPORTER_NODE = 'bulk_data_reporter'
# Reported from two sources and read back as two records, so one record per source.
PER_SOURCE_CODES = ('BULK_TWICE',)


def generate_test_description():
    with open(THRESHOLDS_PATH, 'w') as f:
        for code in PER_SOURCE_CODES:
            f.write(f'{code}:\n  per_source: true\n')
    return create_test_launch(
        demo_nodes=['lidar_sensor'],
        fault_manager=True,
        # The stalled fault manager case waits this long on every call it makes.
        gateway_params={'fault_manager.service_timeout_sec': 1.0 * get_time_scale()},
        fault_manager_params={
            'snapshots.rosbag.storage_path': ROSBAG_DIR,
            'fault_thresholds.config_file': THRESHOLDS_PATH,
        },
        lidar_faulty=True,
    )


class TestBulkDataApi(GatewayTestCase):
    """Bulk-data API tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'lidar_sensor'}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Runtime discovery makes this node an app, so it owns records that the
        # host component lists and lidar_sensor does not.
        cls._reporter = Node(REPORTER_NODE)
        cls._report_client = cls._reporter.create_client(
            ReportFault, '/fault_manager/report_fault')
        super().setUpClass()
        assert cls._report_client.wait_for_service(timeout_sec=15.0), \
            'report_fault service not available'

    @classmethod
    def tearDownClass(cls):
        cls._reporter.destroy_node()
        rclpy.shutdown()

    def _report(self, fault_code, source):
        """Report one FAILED event; resend while the reply is lost.

        A reply can be lost while the fault manager is still matching this
        client, even though the request arrived; resending is harmless.
        """
        req = ReportFault.Request()
        req.fault_code = fault_code
        req.event_type = ReportFault.Request.EVENT_FAILED
        req.severity = Fault.SEVERITY_ERROR
        req.description = 'bulk-data scope test fault'
        req.source_id = source
        for _ in range(6):
            future = self._report_client.call_async(req)
            deadline = time.monotonic() + 5.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self._reporter, timeout_sec=0.1)
            if future.done():
                self.assertTrue(future.result().accepted)
                return
        self.fail(f'report_fault {fault_code} never answered')

    def _bag_fault_codes(self, endpoint, wanted):
        """Fault codes on the rosbag descriptors of ``endpoint`` once ``wanted`` are all listed."""
        deadline = time.monotonic() + ROSBAG_TIMEOUT
        codes = set()
        while time.monotonic() < deadline:
            data = self.get_json(endpoint)
            codes = {code for item in data.get('items', [])
                     for code in item.get('x-medkit', {}).get('fault_codes', [])}
            if wanted <= codes:
                return codes
            time.sleep(0.5)
        self.fail(f'{endpoint} never listed bags of {sorted(wanted - codes)}; '
                  f'listed {sorted(codes)}')

    def test_bulk_data_list_categories_success(self):
        """GET /apps/{app}/bulk-data returns categories.

        @verifies REQ_INTEROP_071
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/bulk-data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        # Should include rosbags category
        self.assertIn('rosbags', data['items'])

    def test_bulk_data_list_categories_all_entity_types(self):
        """Bulk-data endpoint works for apps and components.

        As a ros2_medkit extension, these entity types support bulk-data.
        Uses the host-derived default component (SOVD-aligned entity model).

        @verifies REQ_INTEROP_071
        """
        # Get host component ID dynamically
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertGreater(len(components), 0, 'Expected at least one component')
        comp_id = components[0]['id']

        supported_endpoints = [
            '/apps/lidar_sensor/bulk-data',
            f'/components/{comp_id}/bulk-data',
        ]

        for endpoint in supported_endpoints:
            response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=10)
            self.assertEqual(
                response.status_code, 200,
                f'Expected 200 for {endpoint}, got {response.status_code}'
            )

            data = response.json()
            self.assertIn('items', data)
            self.assertIsInstance(data['items'], list)

    def test_bulk_data_list_categories_entity_not_found(self):
        """Bulk-data returns 404 for nonexistent entity.

        @verifies REQ_INTEROP_071
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_app/bulk-data',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)

    def test_bulk_data_list_descriptors_structure(self):
        """GET /apps/{app}/bulk-data/rosbags returns BulkDataDescriptor[].

        @verifies REQ_INTEROP_072
        """
        # Poll until rosbag descriptors appear (fault triggers capture)
        data = self.poll_endpoint_until(
            '/apps/lidar_sensor/bulk-data/rosbags',
            lambda d: d if d.get('items') else None,
            timeout=10.0,
            interval=1.0,
        )
        self.assertIsInstance(data['items'], list)
        self.assertGreater(
            len(data['items']), 0, 'Expected at least one rosbag descriptor',
        )

        descriptor = data['items'][0]
        self.assertIn('id', descriptor)
        self.assertIn('name', descriptor)
        self.assertIn('size', descriptor)
        self.assertIn('mimetype', descriptor)  # SOVD uses 'mimetype'
        self.assertIn('creation_date', descriptor)
        # Verify x-medkit extension
        self.assertIn('x-medkit', descriptor)
        x_medkit = descriptor['x-medkit']
        # fault_codes, plural: one recording covers every fault of a burst, and
        # the descriptor is per recording rather than per fault.
        self.assertIn('fault_codes', x_medkit)
        self.assertIsInstance(x_medkit['fault_codes'], list)
        self.assertGreater(len(x_medkit['fault_codes']), 0)
        # The descriptor id IS the recording id - that is what addresses the bag.
        self.assertIn('recording_id', x_medkit)
        self.assertEqual(descriptor['id'], x_medkit['recording_id'])
        self.assertTrue(
            x_medkit['recording_id'].startswith('fault_'),
            f'recording_id should be the bag directory name, '
            f'got: {x_medkit["recording_id"]}',
        )

    def test_bulk_data_list_descriptors_empty_result(self):
        """Bulk-data returns empty or non-empty array for component rosbags.

        @verifies REQ_INTEROP_072
        """
        # Use the host-derived default component
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertGreater(len(components), 0, 'Expected at least one component')
        comp_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/bulk-data/rosbags',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_bulk_data_component_aggregates_child_apps(self):
        """GET /components/{id}/bulk-data/rosbags aggregates from hosted apps.

        Synthetic / runtime-discovered components have an empty fqn /
        namespace_path, so the legacy fall-through path returned zero source
        filters and produced empty descriptor lists. The handler now resolves
        hosted apps via the entity cache (mirrors the FUNCTION branch).

        @verifies REQ_INTEROP_072
        """
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        # Find the component that hosts lidar_sensor (the only demo app here).
        comp_id = None
        for c in components:
            hosts = self.get_json(
                f"/components/{c['id']}/hosts").get('items', [])
            if any(h.get('id') == 'lidar_sensor' for h in hosts):
                comp_id = c['id']
                break
        self.assertIsNotNone(
            comp_id, 'No component hosts lidar_sensor')

        # Poll - rosbag capture is fault-triggered and runs after launch.
        data = self.poll_endpoint_until(
            f'/components/{comp_id}/bulk-data/rosbags',
            lambda d: d if d.get('items') else None,
            timeout=15.0,
            interval=1.0,
        )
        self.assertGreater(
            len(data['items']), 0,
            'Component bulk-data aggregation returned zero descriptors',
        )

    def test_bulk_data_unknown_category_returns_404(self):
        """Bulk-data returns 404 for unknown category.

        @verifies REQ_INTEROP_072
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/bulk-data/unknown_category',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)

    def test_bulk_data_descriptor_size_is_the_download_length(self):
        """The descriptor's size is the number of bytes the download sends.

        A client sizes a buffer or a progress bar from the listing, so the
        listing has to promise what the transfer delivers. Nothing else here
        connects the two: the structure test above only checks that a size
        field exists, and a wrong number passes that.

        The test recording is far below snapshots.rosbag.max_bag_size_mb, so it
        is held in one storage file. That is the case where the descriptor size
        and the download length are defined to be equal; a split recording is
        reported at its total and is deliberately larger than its download.

        @verifies REQ_INTEROP_073
        """
        data = self.poll_endpoint_until(
            '/apps/lidar_sensor/bulk-data/rosbags',
            lambda d: d if d.get('items') else None,
            timeout=10.0,
            interval=1.0,
        )
        self.assertGreater(
            len(data['items']), 0, 'Expected at least one rosbag descriptor',
        )
        descriptor = data['items'][0]

        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/bulk-data/rosbags/'
            f'{descriptor["id"]}',
            timeout=30,
        )
        self.assertEqual(response.status_code, 200)

        body = response.content
        self.assertGreater(len(body), 0, 'Download served an empty body')
        self.assertEqual(
            descriptor['size'], len(body),
            f'Listing promised {descriptor["size"]} bytes and the download '
            f'sent {len(body)}',
        )
        self.assertEqual(
            int(response.headers['Content-Length']), len(body),
            'Content-Length disagrees with the body it described',
        )
        # The only bag here written by rosbag2 itself. A parse that misses the
        # real metadata.yaml layout omits the field, and the size asserts above
        # still pass on the fault manager's figure.
        self.assertEqual(
            descriptor['x-medkit'].get('storage_files'), 1,
            'storage_files is missing or wrong for a real rosbag2 bag',
        )

    def test_bulk_data_symlinked_storage_file_is_not_followed(self):
        """A storage file replaced by a symlink to a host file is not sized or served.

        The link keeps the name metadata.yaml gives and points outside the bag.
        The listing and the fault's snapshot entry must report the stored total,
        and the download must not send the link target.

        @verifies REQ_INTEROP_073
        """
        listing_endpoint = '/apps/lidar_sensor/bulk-data/rosbags'
        data = self.poll_endpoint_until(
            listing_endpoint,
            lambda d: d if d.get('items') else None,
            timeout=10.0,
            interval=1.0,
        )
        descriptor = data['items'][0]
        recording_id = descriptor['id']
        fault_code = descriptor['x-medkit']['fault_codes'][0]

        bag_dir = os.path.join(ROSBAG_DIR, recording_id)
        storage_names = [
            n for n in os.listdir(bag_dir) if n.endswith(('.mcap', '.db3'))
        ]
        self.assertEqual(len(storage_names), 1, f'Expected one storage file in {bag_dir}')
        storage_file = os.path.join(bag_dir, storage_names[0])
        stored_total = sum(
            os.path.getsize(os.path.join(bag_dir, n)) for n in os.listdir(bag_dir)
        )

        def listed_size():
            items = self.get_json(listing_endpoint)['items']
            return next(i['size'] for i in items if i['id'] == recording_id)

        def fault_snapshot_size():
            detail = self.get_json(f'/apps/lidar_sensor/faults/{fault_code}')
            return next(
                s['size_bytes']
                for s in detail['environment_data']['snapshots']
                if s.get('type') == 'rosbag'
                and s['bulk_data_uri'].endswith(f'/{recording_id}')
            )

        # Control: the intact bag is sized by its storage file on both paths.
        served = os.path.getsize(storage_file)
        self.assertEqual(listed_size(), served)
        self.assertEqual(fault_snapshot_size(), served)

        outside_dir = tempfile.mkdtemp(prefix='medkit_bulk_data_outside_')
        target = os.path.join(outside_dir, 'host_file' + os.path.splitext(storage_file)[1])
        target_bytes = b'o' * 12345
        with open(target, 'wb') as f:
            f.write(target_bytes)
        original = os.path.join(outside_dir, 'original')
        shutil.move(storage_file, original)
        try:
            os.symlink(target, storage_file)
            self.assertEqual(
                listed_size(), stored_total,
                'listing did not fall back to the stored total for a symlinked file',
            )
            self.assertEqual(
                fault_snapshot_size(), stored_total,
                'fault snapshot did not fall back to the stored total for a symlinked file',
            )
            response = requests.get(
                f'{self.BASE_URL}{listing_endpoint}/{recording_id}', timeout=30,
            )
            self.assertNotEqual(response.content, target_bytes, 'download served the link target')
            self.assertEqual(response.status_code, 500, 'bag with no regular storage file')
        finally:
            if os.path.lexists(storage_file):
                os.remove(storage_file)
            shutil.move(original, storage_file)
            shutil.rmtree(outside_dir, ignore_errors=True)

    def test_bulk_data_download_not_found(self):
        """Bulk-data download returns 404 for invalid UUID.

        @verifies REQ_INTEROP_073
        """
        response = requests.get(
            f'{self.BASE_URL}/apps/lidar_sensor/bulk-data/rosbags/nonexistent-uuid',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        # Response should be JSON error
        data = response.json()
        self.assertIn('error_code', data)

    def test_bulk_data_nested_entity_path(self):
        """Bulk-data endpoints work for component entities.

        Components support bulk-data. Uses the host-derived default component.

        @verifies REQ_INTEROP_071
        """
        # Use the host-derived default component
        comp_data = self.get_json('/components')
        components = comp_data.get('items', [])
        self.assertGreater(len(components), 0, 'Expected at least one component')
        comp_id = components[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/components/{comp_id}/bulk-data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

    def test_bulk_data_rosbag_list_follows_fault_scope(self):
        """An entity lists the bag of a record when the record's owner is in its scope.

        ``BULK_SUBPATH`` and ``BULK_SUBPATH2`` are owned by path children of the
        lidar node FQN, one and two levels down, which the lidar app owns.
        ``BULK_FOREIGN`` is owned by another app, so it belongs to the host
        component and not to the lidar app. Each listing must agree with the
        download of the same entity, which applies that rule.

        @verifies REQ_INTEROP_072
        """
        self.poll_endpoint(f'/apps/{REPORTER_NODE}', timeout=30.0)
        comp_id = next(
            (c['id'] for c in self.get_json('/components').get('items', [])
             if any(h.get('id') == 'lidar_sensor'
                    for h in self.get_json(f"/components/{c['id']}/hosts").get('items', []))),
            None)
        self.assertIsNotNone(comp_id, 'No component hosts lidar_sensor')

        self._report('BULK_SUBPATH', f'{LIDAR_FQN}/diag')
        self._report('BULK_SUBPATH2', f'{LIDAR_FQN}/diag/deep')
        self._report('BULK_FOREIGN', f'/{REPORTER_NODE}')

        owned = {'BULK_SUBPATH', 'BULK_SUBPATH2'}
        component_bags = f'/components/{comp_id}/bulk-data/rosbags'
        self._bag_fault_codes(component_bags, owned | {'BULK_FOREIGN'})
        app_codes = self._bag_fault_codes('/apps/lidar_sensor/bulk-data/rosbags', owned)
        self.assertNotIn(
            'BULK_FOREIGN', app_codes,
            'lidar_sensor lists the bag of a record it does not own')

        # The listings agree with the downloads (fault code resolves to the
        # record's newest recording).
        downloads = {
            '/apps/lidar_sensor/bulk-data/rosbags/BULK_SUBPATH': 200,
            '/apps/lidar_sensor/bulk-data/rosbags/BULK_SUBPATH2': 200,
            '/apps/lidar_sensor/bulk-data/rosbags/BULK_FOREIGN': 404,
            f'{component_bags}/BULK_SUBPATH': 200,
            f'{component_bags}/BULK_SUBPATH2': 200,
            f'{component_bags}/BULK_FOREIGN': 200,
        }
        for endpoint, status in downloads.items():
            with self.subTest(endpoint=endpoint):
                response = requests.get(f'{self.BASE_URL}{endpoint}', timeout=10)
                self.assertEqual(response.status_code, status, response.text[:200])

    def test_bulk_data_code_url_serves_the_newer_recording_of_two_owners_below_one_app(self):
        """Two path children of one node own records of one code: the code URL serves the newer.

        Every recording the listing shows downloads by its id.

        @verifies REQ_INTEROP_073
        """
        endpoint = '/apps/lidar_sensor/bulk-data/rosbags'

        def twice(listing):
            return [item['id'] for item in listing.get('items', [])
                    if 'BULK_TWICE' in item.get('x-medkit', {}).get('fault_codes', [])]

        self._report('BULK_TWICE', f'{LIDAR_FQN}/diag')
        [older] = self.poll_endpoint_until(endpoint, lambda d: twice(d), timeout=ROSBAG_TIMEOUT)
        # Listed means its window has closed, so the next confirmation gets its own recording.
        self._report('BULK_TWICE', f'{LIDAR_FQN}/diag/deep')
        ids = self.poll_endpoint_until(
            endpoint, lambda d: twice(d) if len(twice(d)) == 2 else None, timeout=ROSBAG_TIMEOUT)
        newer = next(i for i in ids if i != older)

        response = requests.get(f'{self.BASE_URL}{endpoint}/BULK_TWICE', timeout=30)
        self.assertEqual(response.status_code, 200, response.text[:300])
        self.assertIn(newer, response.headers.get('Content-Disposition', ''))

        for recording_id in ids:
            with self.subTest(recording_id=recording_id):
                download = requests.get(f'{self.BASE_URL}{endpoint}/{recording_id}', timeout=30)
                self.assertEqual(download.status_code, 200, download.text[:200])

    def test_rosbag_routes_declare_their_503(self):
        """The listing and download routes answer 503 when the fault manager is silent, and say so.

        @verifies REQ_INTEROP_072
        """
        spec = self.poll_endpoint_until('/docs', lambda d: d if 'openapi' in d else None)

        routes = {
            path: item['get'] for path, item in spec['paths'].items()
            if path.endswith(('/bulk-data/{category_id}', '/bulk-data/{category_id}/{file_id}'))
            and 'get' in item
        }
        # A list and a download per entity type that serves bulk data, so the
        # loop below cannot pass by matching nothing.
        self.assertEqual(len(routes), 12, sorted(routes))
        for path, operation in sorted(routes.items()):
            self.assertIn('503', operation['responses'],
                          f'GET {path} answers 503 when the fault manager fails')

    def test_zz_rosbag_routes_answer_503_while_the_fault_manager_is_stalled(
            self, proc_info, fault_manager_node):
        """The rosbag listing and download report an unanswering fault manager.

        SIGSTOP keeps the fault manager's services on the graph while they
        never answer. A 200 with no items would tell the client there are no
        recordings, and a 404 that the recording does not exist. Named to run
        last; the process resumes on every exit path.

        @verifies REQ_INTEROP_072
        @verifies REQ_INTEROP_073
        """
        self._report('BULK_STALL', f'{LIDAR_FQN}/diag')
        listing = '/apps/lidar_sensor/bulk-data/rosbags'
        self._bag_fault_codes(listing, {'BULK_STALL'})
        recording_id = next(
            item['id'] for item in self.get_json(listing).get('items', [])
            if 'BULK_STALL' in item.get('x-medkit', {}).get('fault_codes', []))
        download = f'{self.BASE_URL}{listing}/{recording_id}'
        self.assertEqual(requests.get(download, timeout=30).status_code, 200)

        pid = proc_info[fault_manager_node].pid
        self.addCleanup(os.kill, pid, signal.SIGCONT)
        os.kill(pid, signal.SIGSTOP)
        try:
            listed = requests.get(f'{self.BASE_URL}{listing}', timeout=60)
            downloaded = requests.get(download, timeout=60)
        finally:
            os.kill(pid, signal.SIGCONT)

        self.assertEqual(listed.status_code, 503, listed.text[:300])
        self.assertEqual(downloaded.status_code, 503, downloaded.text[:300])
        # Both routes recover once the fault manager answers again.
        self.poll_endpoint(listing, timeout=30.0)
        self.assertEqual(requests.get(download, timeout=30).status_code, 200)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
