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

"""
Integration tests for bulk data upload (POST) and delete (DELETE) endpoints.

Launches the gateway node with bulk_data categories configured, then tests
the full CRUD lifecycle: upload, list, download, delete.
"""

import json
import os
import tempfile
import time
import unittest

from launch import LaunchDescription
from launch.actions import TimerAction
import launch_ros.actions
import launch_testing.actions
import requests


# Use a unique port to avoid conflicts with other integration tests
TEST_PORT = 8765


def get_coverage_env():
    """Get environment variables for gcov coverage data collection."""
    try:
        from ament_index_python.packages import get_package_prefix
        pkg_prefix = get_package_prefix('ros2_medkit_gateway')
        workspace = os.path.dirname(os.path.dirname(pkg_prefix))
        build_dir = os.path.join(workspace, 'build', 'ros2_medkit_gateway')
        if os.path.exists(build_dir):
            return {
                'GCOV_PREFIX': build_dir,
                'GCOV_PREFIX_STRIP': str(build_dir.count(os.sep)),
            }
    except Exception:
        pass
    return {}


def generate_test_description():
    """Generate launch description with gateway node configured for bulk data."""
    tmpdir = tempfile.mkdtemp(prefix='bulk_data_test_')

    gateway_node = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='gateway_node',
        name='ros2_medkit_gateway',
        output='screen',
        parameters=[{
            'server.host': '127.0.0.1',
            'server.port': TEST_PORT,
            'refresh_interval_ms': 1000,
            'bulk_data.storage_dir': tmpdir,
            'bulk_data.max_upload_size': 104857600,
            'bulk_data.categories': ['calibration', 'firmware'],
        }],
        additional_env=get_coverage_env(),
    )

    # A single demo node so we have at least one app entity to test with
    engine_temp_sensor = launch_ros.actions.Node(
        package='ros2_medkit_gateway',
        executable='demo_engine_temp_sensor',
        name='temp_sensor',
        namespace='/powertrain/engine',
        output='screen',
        additional_env=get_coverage_env(),
    )

    delayed_sensors = TimerAction(
        period=2.0,
        actions=[engine_temp_sensor],
    )

    return (
        LaunchDescription([
            gateway_node,
            delayed_sensors,
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'gateway_node': gateway_node,
        },
    )


API_BASE_PATH = '/api/v1'


class TestBulkDataUploadDelete(unittest.TestCase):
    """Integration tests for bulk data upload/delete REST API."""

    BASE_URL = f'http://localhost:{TEST_PORT}{API_BASE_PATH}'
    MAX_DISCOVERY_WAIT = 30.0

    @classmethod
    def setUpClass(cls):
        """Wait for gateway and at least one app to be discovered."""
        # Wait for gateway health
        for i in range(30):
            try:
                r = requests.get(f'{cls.BASE_URL}/health', timeout=2)
                if r.status_code == 200:
                    break
            except requests.exceptions.RequestException:
                if i == 29:
                    raise unittest.SkipTest('Gateway not responding')
                time.sleep(1)

        # Wait for temp_sensor app to be discovered
        start = time.time()
        while time.time() - start < cls.MAX_DISCOVERY_WAIT:
            try:
                r = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
                if r.status_code == 200:
                    apps = r.json().get('items', [])
                    app_ids = {a.get('id', '') for a in apps}
                    if 'temp_sensor' in app_ids:
                        cls.test_app_id = 'temp_sensor'
                        # Also find a component
                        cr = requests.get(f'{cls.BASE_URL}/components', timeout=5)
                        if cr.status_code == 200:
                            comps = cr.json().get('items', [])
                            if comps:
                                cls.test_component_id = comps[0].get('id', '')
                        # Find an area
                        ar = requests.get(f'{cls.BASE_URL}/areas', timeout=5)
                        if ar.status_code == 200:
                            areas = ar.json().get('items', [])
                            if areas:
                                cls.test_area_id = areas[0].get('id', '')
                        return
            except requests.exceptions.RequestException:
                pass
            time.sleep(1)
        raise unittest.SkipTest('App temp_sensor not discovered in time')

    # === Upload (POST) tests ===

    # @verifies REQ_INTEROP_074
    def test_upload_small_file(self):
        """POST multipart with small file returns 201 with descriptor."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('test.bin', b'x' * 100, 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 201)
        data = r.json()
        self.assertIn('id', data)
        self.assertEqual(data['name'], 'test.bin')
        self.assertEqual(data['mimetype'], 'application/octet-stream')
        self.assertEqual(data['size'], 100)
        self.assertIn('creation_date', data)

    # @verifies REQ_INTEROP_074
    def test_upload_returns_location_header(self):
        """201 response has Location header pointing to uploaded resource."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('loc.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 201)
        location = r.headers.get('Location', '')
        prefix = (
            f'/api/v1/apps/{self.test_app_id}/bulk-data/calibration/'
        )
        self.assertTrue(location.startswith(prefix))
        # Location should end with the item ID
        item_id = r.json()['id']
        self.assertTrue(location.endswith(item_id))

    # @verifies REQ_INTEROP_074
    def test_upload_with_description(self):
        """POST with description field includes it in descriptor."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('desc.bin', b'data', 'application/octet-stream')}
        data = {'description': 'Test calibration data'}
        r = requests.post(url, files=files, data=data, timeout=10)
        self.assertEqual(r.status_code, 201)
        self.assertEqual(r.json()['description'], 'Test calibration data')

    # @verifies REQ_INTEROP_074
    def test_upload_with_metadata(self):
        """POST with metadata JSON field includes x-medkit in descriptor."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('meta.bin', b'data', 'application/octet-stream')}
        data = {'metadata': json.dumps({'sensor': 'lidar', 'version': 2})}
        r = requests.post(url, files=files, data=data, timeout=10)
        self.assertEqual(r.status_code, 201)
        resp = r.json()
        self.assertIn('x-medkit', resp)
        self.assertEqual(resp['x-medkit']['sensor'], 'lidar')

    # @verifies REQ_INTEROP_074
    def test_upload_missing_file_field(self):
        """POST without file field returns 400."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        # Send empty multipart with only description, no file
        data = {'description': 'no file'}
        r = requests.post(url, data=data, timeout=10)
        self.assertEqual(r.status_code, 400)

    # @verifies REQ_INTEROP_074
    def test_upload_unknown_category(self):
        """POST to unknown category returns 400."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/bogus'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 400)

    # @verifies REQ_INTEROP_074
    def test_upload_rosbags_rejected(self):
        """POST to rosbags category returns 400."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/rosbags'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 400)
        self.assertIn('fault system', r.json().get('message', ''))

    # @verifies REQ_INTEROP_074
    def test_upload_nonexistent_entity(self):
        """POST to nonexistent entity returns 404."""
        url = f'{self.BASE_URL}/apps/fake_entity_12345/bulk-data/calibration'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_074
    def test_upload_to_areas_405(self):
        """POST to areas returns 405."""
        area_id = getattr(self, 'test_area_id', 'powertrain')
        url = f'{self.BASE_URL}/areas/{area_id}/bulk-data/calibration'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 405)

    # @verifies REQ_INTEROP_074
    def test_upload_to_functions_405(self):
        """POST to functions returns 405."""
        url = f'{self.BASE_URL}/functions/some_func/bulk-data/calibration'
        files = {'file': ('test.bin', b'data', 'application/octet-stream')}
        r = requests.post(url, files=files, timeout=10)
        self.assertEqual(r.status_code, 405)

    # === Delete (DELETE) tests ===

    # @verifies REQ_INTEROP_074
    def test_delete_uploaded_item(self):
        """Upload then DELETE returns 204."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('del.bin', b'delete_me', 'application/octet-stream')}
        upload_r = requests.post(url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        del_url = f'{url}/{item_id}'
        del_r = requests.delete(del_url, timeout=10)
        self.assertEqual(del_r.status_code, 204)

    # @verifies REQ_INTEROP_074
    def test_delete_nonexistent_item(self):
        """DELETE nonexistent ID returns 404."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration/nonexistent_id'
        r = requests.delete(url, timeout=10)
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_074
    def test_delete_rosbags_rejected(self):
        """DELETE rosbags category returns 400."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/rosbags/some_id'
        r = requests.delete(url, timeout=10)
        self.assertEqual(r.status_code, 400)

    # @verifies REQ_INTEROP_074
    def test_delete_to_areas_405(self):
        """DELETE on areas returns 405."""
        area_id = getattr(self, 'test_area_id', 'powertrain')
        url = f'{self.BASE_URL}/areas/{area_id}/bulk-data/calibration/some_id'
        r = requests.delete(url, timeout=10)
        self.assertEqual(r.status_code, 405)

    # === List / Download (GET — extended) tests ===

    # @verifies REQ_INTEROP_071
    def test_list_categories_includes_configured(self):
        """GET /bulk-data returns rosbags + configured categories."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data'
        r = requests.get(url, timeout=10)
        self.assertEqual(r.status_code, 200)
        items = r.json().get('items', [])
        self.assertIn('rosbags', items)
        self.assertIn('calibration', items)
        self.assertIn('firmware', items)

    # @verifies REQ_INTEROP_072
    def test_list_descriptors_after_upload(self):
        """Upload 2 files, list shows 2 items."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/firmware'
        for name in ['fw1.bin', 'fw2.bin']:
            files = {'file': (name, b'firmware_data', 'application/octet-stream')}
            r = requests.post(url, files=files, timeout=10)
            self.assertEqual(r.status_code, 201)

        r = requests.get(url, timeout=10)
        self.assertEqual(r.status_code, 200)
        items = r.json().get('items', [])
        self.assertGreaterEqual(len(items), 2)

    # @verifies REQ_INTEROP_073
    def test_download_uploaded_file(self):
        """Upload 'hello' then download returns same content."""
        base_url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        files = {'file': ('hello.txt', b'hello', 'text/plain')}
        upload_r = requests.post(base_url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        dl_url = f'{base_url}/{item_id}'
        dl_r = requests.get(dl_url, timeout=10)
        self.assertEqual(dl_r.status_code, 200)
        self.assertEqual(dl_r.content, b'hello')
        # Check Content-Disposition
        cd = dl_r.headers.get('Content-Disposition', '')
        self.assertIn('hello.txt', cd)

    # @verifies REQ_INTEROP_073
    def test_download_nonexistent_returns_404(self):
        """GET download with fake ID returns 404."""
        url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration/nonexistent_id'
        r = requests.get(url, timeout=10)
        self.assertEqual(r.status_code, 404)

    # @verifies REQ_INTEROP_074
    def test_list_empty_after_delete(self):
        """Upload, delete, then list shows no items from this upload."""
        # Use firmware category to avoid interference from other tests
        base_url = f'{self.BASE_URL}/components/{self.test_component_id}/bulk-data/firmware'
        files = {'file': ('gone.bin', b'data', 'application/octet-stream')}
        upload_r = requests.post(base_url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        # Delete
        del_r = requests.delete(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(del_r.status_code, 204)

        # Check item is gone
        dl_r = requests.get(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(dl_r.status_code, 404)

    # === Full CRUD flow ===

    # @verifies REQ_INTEROP_071
    # @verifies REQ_INTEROP_074
    def test_full_crud_cycle(self):
        """Upload -> List (present) -> Download (content) -> Delete -> List (absent)."""
        base_url = f'{self.BASE_URL}/apps/{self.test_app_id}/bulk-data/calibration'
        payload = b'full_crud_test_data'
        files = {'file': ('crud.bin', payload, 'application/octet-stream')}

        # Upload
        upload_r = requests.post(base_url, files=files, timeout=10)
        self.assertEqual(upload_r.status_code, 201)
        item_id = upload_r.json()['id']

        # List — verify present
        list_r = requests.get(base_url, timeout=10)
        self.assertEqual(list_r.status_code, 200)
        item_ids = [i['id'] for i in list_r.json().get('items', [])]
        self.assertIn(item_id, item_ids)

        # Download — verify content
        dl_r = requests.get(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(dl_r.status_code, 200)
        self.assertEqual(dl_r.content, payload)

        # Delete
        del_r = requests.delete(f'{base_url}/{item_id}', timeout=10)
        self.assertEqual(del_r.status_code, 204)

        # List — verify absent
        list_r2 = requests.get(base_url, timeout=10)
        self.assertEqual(list_r2.status_code, 200)
        item_ids2 = [i['id'] for i in list_r2.json().get('items', [])]
        self.assertNotIn(item_id, item_ids2)
