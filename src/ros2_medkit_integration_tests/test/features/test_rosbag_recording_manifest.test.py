#!/usr/bin/env python3
# Copyright 2026 mfaferek93
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

"""A recording split across several storage files, fetched whole over HTTP.

A recording past ``snapshots.rosbag.max_bag_size_mb`` is several storage files.
The download route serves one of them, so a client needs the file list, a way
to fetch each file, and something to check the bytes against. This suite
records a real split recording and reads it back through the manifest route and
the file-by-index route, comparing every size and sha256 with the files the
fault manager wrote.

The fault is the lidar's own range check, driven by its parameters. The split
comes from a topic this test publishes: the lidar's scan alone stays far below
1 MB, the smallest split size there is.

The storage quota is 1 MB, below the size of the recording, so the fault
manager's own sweep runs on it and has to keep it.
"""

import hashlib
import json
import os
import tempfile
import threading
import time
import unittest

import launch_testing
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, ROSBAG_TIMEOUT
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

from std_msgs.msg import String


APP_ENDPOINT = '/apps/lidar_sensor'
OTHER_APP_ENDPOINT = '/apps/temp_sensor'
LIDAR_NAMESPACE = '/perception/lidar'
FAULT_CODE = 'LIDAR_RANGE_INVALID'
NOT_FOUND_VENDOR_CODE = 'x-medkit-recording-not-found'

# 200 KB at 20 Hz fills several 1 MB storage files inside the recording window.
BLOB_TOPIC = '/manifest_probe/blob'
BLOB_BYTES = 200 * 1024
BLOB_PERIOD_SEC = 0.05
# Sent after the capture subscribed and before the fault is raised: 4 MB, which
# is several storage files of the 1 MB split.
BLOBS_BEFORE_FAULT = 20
SUBSCRIBE_TIMEOUT_SEC = 30.0

# Below the size of the recording, so the sweep after the store has to keep it.
MAX_TOTAL_STORAGE_MB = 1
ALONE_EXCEEDS_WARNING = 'alone exceeds snapshots.rosbag.max_total_storage_mb'

ROSBAG_STORAGE_PATH = tempfile.mkdtemp(prefix='rosbag_e2e_manifest_')


def generate_test_description():
    return create_test_launch(
        demo_nodes=['lidar_sensor', 'temp_sensor'],
        lidar_faulty=False,
        fault_manager=True,
        fault_manager_params={
            'confirmation_threshold': -1,
            'snapshots.rosbag.topics': 'all',
            'snapshots.rosbag.duration_sec': 3.0,
            'snapshots.rosbag.duration_after_sec': 1.0,
            'snapshots.rosbag.format': 'mcap',
            'snapshots.rosbag.storage_path': ROSBAG_STORAGE_PATH,
            'snapshots.rosbag.max_bag_size_mb': 1,
            'snapshots.rosbag.max_total_storage_mb': MAX_TOTAL_STORAGE_MB,
        },
    )


def sha256_hex(data):
    return hashlib.sha256(data).hexdigest()


def metadata_file_order(bag_dir):
    """Storage file names in the order metadata.yaml lists them."""
    names = []
    in_list = False
    with open(os.path.join(bag_dir, 'metadata.yaml'), encoding='utf-8') as file:
        for line in file:
            entry = line.strip()
            if entry.startswith('relative_file_paths:'):
                in_list = True
            elif in_list and entry.startswith('- '):
                names.append(os.path.basename(entry[2:].strip()))
            elif in_list:
                break
    return names


class TestRosbagRecordingManifest(GatewayTestCase):
    """One split recording, read back file by file."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'lidar_sensor', 'temp_sensor'}

    recording_id = None

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        rclpy.init()
        cls._node = Node('rosbag_manifest_test_client')
        cls._param_client = cls._node.create_client(
            SetParameters, f'{LIDAR_NAMESPACE}/lidar_sensor/set_parameters',
        )
        assert cls._param_client.wait_for_service(timeout_sec=20.0), \
            'lidar_sensor parameter services not available'
        cls._blob_pub = cls._node.create_publisher(String, BLOB_TOPIC, 10)

    @classmethod
    def tearDownClass(cls):
        cls._node.destroy_node()
        rclpy.shutdown()
        super().tearDownClass()

    # ------------------------------------------------------------------
    # Making the recording
    # ------------------------------------------------------------------

    def _break_the_range(self):
        """Make min_range >= max_range, which the lidar reports on its own."""
        request = SetParameters.Request()
        request.parameters = [
            Parameter('min_range', Parameter.Type.DOUBLE, 10.0).to_parameter_msg(),
            Parameter('max_range', Parameter.Type.DOUBLE, 5.0).to_parameter_msg(),
        ]
        future = self._param_client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
        result = future.result()
        self.assertIsNotNone(result, 'set_parameters call to lidar_sensor timed out')
        for outcome in result.results:
            self.assertTrue(outcome.successful, outcome.reason)

    def _listed_recording(self):
        """Return the listing item of the range fault's recording, or None."""
        listing = self.get_json(f'{APP_ENDPOINT}/bulk-data/rosbags')
        for item in listing.get('items', []):
            if FAULT_CODE in item.get('x-medkit', {}).get('fault_codes', []):
                return item
        return None

    def _wait_for_capture_subscription(self):
        """Wait until the fault manager's capture has subscribed to the blob topic."""
        deadline = time.monotonic() + SUBSCRIBE_TIMEOUT_SEC
        while self._blob_pub.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                self.fail(f'the capture never subscribed to {BLOB_TOPIC}')
            rclpy.spin_once(self._node, timeout_sec=0.1)

    def _record(self):
        """Record one split recording of the range fault and return its listing item."""
        self._wait_for_capture_subscription()
        stop = threading.Event()
        enough_sent = threading.Event()

        def publish_blobs():
            message = String()
            message.data = 'x' * BLOB_BYTES
            sent = 0
            while not stop.is_set():
                self._blob_pub.publish(message)
                sent += 1
                if sent >= BLOBS_BEFORE_FAULT:
                    enough_sent.set()
                stop.wait(BLOB_PERIOD_SEC)

        publisher = threading.Thread(target=publish_blobs, daemon=True)
        publisher.start()
        try:
            # The buffer holds enough for the recording to split once these are sent.
            self.assertTrue(enough_sent.wait(timeout=SUBSCRIBE_TIMEOUT_SEC),
                            'the blob publisher stalled')
            self._break_the_range()
            self.wait_for_fault(APP_ENDPOINT, FAULT_CODE)

            deadline = time.time() + ROSBAG_TIMEOUT
            item = None
            while time.time() < deadline:
                item = self._listed_recording()
                if item is not None:
                    return item
                time.sleep(0.5)
            self.fail(f'no recording of {FAULT_CODE} was listed')
        finally:
            stop.set()
            publisher.join(timeout=5.0)

    def _manifest(self):
        response = self.get_raw(
            f'{APP_ENDPOINT}/bulk-data/rosbags/{self.recording_id}/manifest')
        return response, json.loads(response.content)

    def _file_url(self, index, recording_id=None):
        return (f'{APP_ENDPOINT}/bulk-data/rosbags/'
                f'{recording_id or self.recording_id}/files/{index}')

    def _assert_plain_404(self, endpoint, **kwargs):
        body = self.get_raw(endpoint, expected_status=404, **kwargs).json()
        self.assertNotIn(
            'vendor_code', body,
            f'{endpoint} claims the recording does not exist: {body}')

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_the_listing_advertises_the_manifest(self, proc_output, fault_manager_node):
        """A finalised recording is listed with ``x-medkit.manifest`` and survives its quota sweep.

        @verifies REQ_INTEROP_072
        """
        item = self._record()
        type(self).recording_id = item['id']
        self.assertIs(item['x-medkit'].get('manifest'), True,
                      f'the listing does not advertise a manifest: {item}')
        self.assertGreater(item['size'], MAX_TOTAL_STORAGE_MB * 1024 * 1024,
                           'the recording does not exceed the quota, so the sweep proves nothing')

        # The sweep runs right after the rows are stored. It keeps the recording
        # and says that it alone is over the quota.
        proc_output.assertWaitFor(
            f"Recording '{self.recording_id}' {ALONE_EXCEEDS_WARNING}",
            process=fault_manager_node, timeout=ROSBAG_TIMEOUT)
        self.assertIsNotNone(self._listed_recording(),
                             'the quota sweep deleted the recording it had just stored')
        self.assertTrue(
            os.path.isfile(os.path.join(ROSBAG_STORAGE_PATH, self.recording_id, 'manifest.json')))

    def test_02_the_manifest_matches_the_files_on_disk(self):
        """Order, sizes and hashes are those of the files the fault manager wrote.

        @verifies REQ_INTEROP_073
        """
        self.assertIsNotNone(self.recording_id, 'test_01 must run first')
        response, manifest = self._manifest()
        bag_dir = os.path.join(ROSBAG_STORAGE_PATH, self.recording_id)

        self.assertIn('application/json', response.headers.get('Content-Type', ''))
        with open(os.path.join(bag_dir, 'manifest.json'), 'rb') as file:
            self.assertEqual(response.content, file.read(),
                             'the route did not serve the manifest byte for byte')

        self.assertEqual(manifest['manifest_version'], 1)
        self.assertEqual(manifest['recording_id'], self.recording_id)
        self.assertEqual(manifest['format'], 'mcap')
        self.assertIsInstance(manifest['created_at_ns'], int)

        files = manifest['files']
        self.assertGreater(len(files), 1, 'the recording never split')

        self.assertEqual(
            [entry['name'] for entry in files], metadata_file_order(bag_dir),
            'the manifest does not list the files in metadata.yaml order')

        total = 0
        for entry in files:
            path = os.path.join(bag_dir, entry['name'])
            with open(path, 'rb') as file:
                data = file.read()
            self.assertEqual(entry['size'], len(data), entry['name'])
            self.assertEqual(entry['sha256'], sha256_hex(data), entry['name'])
            total += len(data)
        self.assertEqual(manifest['total_bytes'], total)
        self.assertNotIn('metadata.yaml', [entry['name'] for entry in files])

    def test_02b_the_listing_and_the_fault_report_the_files_total(self):
        """A split recording is reported at the sum of its storage files.

        @verifies REQ_INTEROP_072
        """
        self.assertIsNotNone(self.recording_id, 'test_01 must run first')
        _, manifest = self._manifest()
        bag_dir = os.path.join(ROSBAG_STORAGE_PATH, self.recording_id)
        directory_total = sum(
            os.path.getsize(os.path.join(bag_dir, name)) for name in os.listdir(bag_dir)
            if os.path.isfile(os.path.join(bag_dir, name)))
        # metadata.yaml and manifest.json make the stored total larger.
        self.assertGreater(directory_total, manifest['total_bytes'])

        item = self._listed_recording()
        self.assertIsNotNone(item, 'the recording is no longer listed')
        self.assertEqual(item['size'], manifest['total_bytes'],
                         'the listing does not report the sum of the storage files')

        snapshot = self.poll_endpoint_until(
            f'{APP_ENDPOINT}/faults/{FAULT_CODE}',
            lambda d: next(
                (s for s in d.get('environment_data', {}).get('snapshots', [])
                 if s.get('type') == 'rosbag'
                 and s.get('bulk_data_uri', '').endswith(f'/rosbags/{self.recording_id}')),
                None,
            ),
            timeout=10.0,
            interval=0.5,
        )
        self.assertIsNotNone(snapshot, 'the fault lists no rosbag snapshot for the recording')
        self.assertEqual(snapshot.get('size_bytes'), manifest['total_bytes'],
                         'the fault does not report the sum of the storage files')

    def test_02c_the_download_serves_the_first_file_and_counts_them_all(self):
        """The recording download hands over the first file the manifest lists.

        @verifies REQ_INTEROP_073
        """
        self.assertIsNotNone(self.recording_id, 'test_01 must run first')
        _, manifest = self._manifest()
        first = manifest['files'][0]

        item = self._listed_recording()
        self.assertIsNotNone(item, 'the recording is no longer listed')
        self.assertEqual(item['x-medkit'].get('storage_files'), len(manifest['files']),
                         'the listing does not count every storage file')

        response = self.get_raw(f'{APP_ENDPOINT}/bulk-data/rosbags/{self.recording_id}',
                                timeout=30)
        self.assertEqual(int(response.headers['Content-Length']), first['size'])
        self.assertEqual(sha256_hex(response.content), first['sha256'],
                         'the download did not serve the first storage file')
        self.assertLess(int(response.headers['Content-Length']), item['size'])

    def test_03_every_file_downloads_whole_and_verifies(self):
        """Each index serves exactly the file the manifest describes.

        @verifies REQ_INTEROP_073
        """
        _, manifest = self._manifest()
        for index, entry in enumerate(manifest['files']):
            response = self.get_raw(self._file_url(index), timeout=30)
            self.assertEqual(int(response.headers['Content-Length']), entry['size'])
            self.assertEqual(response.headers.get('Accept-Ranges'), 'bytes')
            self.assertIn('application/octet-stream',
                          response.headers.get('Content-Type', ''))
            self.assertEqual(len(response.content), entry['size'])
            self.assertEqual(sha256_hex(response.content), entry['sha256'],
                             f'file {index} does not match its manifest entry')
            self.assertEqual(response.content[:5], b'\x89MCAP')

    def test_04_a_range_resumes_a_file(self):
        """Open-ended, closed and suffix ranges return the bytes they name.

        @verifies REQ_INTEROP_073
        """
        _, manifest = self._manifest()
        size = manifest['files'][0]['size']
        whole = self.get_raw(self._file_url(0), timeout=30).content

        resumed = self.get_raw(self._file_url(0), expected_status=206, timeout=30,
                               headers={'Range': 'bytes=1000-'})
        self.assertEqual(int(resumed.headers['Content-Length']), size - 1000)
        self.assertEqual(resumed.headers['Content-Range'], f'bytes 1000-{size - 1}/{size}')
        self.assertEqual(resumed.content, whole[1000:])

        window = self.get_raw(self._file_url(0), expected_status=206,
                              headers={'Range': 'bytes=1000-1999'})
        self.assertEqual(window.headers['Content-Range'], f'bytes 1000-1999/{size}')
        self.assertEqual(int(window.headers['Content-Length']), 1000)
        self.assertEqual(window.content, whole[1000:2000])

        suffix = self.get_raw(self._file_url(0), expected_status=206,
                              headers={'Range': 'bytes=-500'})
        self.assertEqual(suffix.headers['Content-Range'],
                         f'bytes {size - 500}-{size - 1}/{size}')
        self.assertEqual(suffix.content, whole[-500:])

        last = self.get_raw(self._file_url(0), expected_status=206,
                            headers={'Range': f'bytes={size - 1}-'})
        self.assertEqual(last.content, whole[-1:])

    def test_05_fixed_chunks_read_the_last_file_to_its_end(self):
        """A last byte past the end is the end of the file, so fixed chunks need no size.

        @verifies REQ_INTEROP_073
        """
        _, manifest = self._manifest()
        index = len(manifest['files']) - 1
        size = manifest['files'][index]['size']
        chunk = 1024 * 1024
        whole = self.get_raw(self._file_url(index), timeout=30).content

        joined = b''
        first = 0
        while first < size:
            part = self.get_raw(self._file_url(index), expected_status=206, timeout=30,
                                headers={'Range': f'bytes={first}-{first + chunk - 1}'})
            last = min(first + chunk, size) - 1
            self.assertEqual(part.headers['Content-Range'], f'bytes {first}-{last}/{size}')
            self.assertEqual(int(part.headers['Content-Length']), last - first + 1)
            joined += part.content
            first += chunk
        self.assertEqual(joined, whole)
        self.assertEqual(sha256_hex(joined), manifest['files'][index]['sha256'])

        clamped = self.get_raw(self._file_url(index), expected_status=206,
                               headers={'Range': f'bytes=0-{size}'})
        self.assertEqual(clamped.headers['Content-Range'], f'bytes 0-{size - 1}/{size}')
        self.assertEqual(clamped.content, whole)

    def test_05b_a_range_outside_the_file_is_416(self):
        """Only a range with no byte inside the file is refused.

        @verifies REQ_INTEROP_073
        """
        _, manifest = self._manifest()
        size = manifest['files'][0]['size']
        for value in (f'bytes={size}-', f'bytes={size + 100000}-', f'bytes={size}-{size + 10}'):
            response = self.get_raw(self._file_url(0), expected_status=416,
                                    headers={'Range': value})
            self.assertEqual(response.headers.get('Content-Range'), f'bytes */{size}', value)
            self.assertEqual(response.json()['parameters']['size'], size, value)

        # One part past the end is left out, the other is served.
        mixed = self.get_raw(self._file_url(0), expected_status=206,
                             headers={'Range': f'bytes=0-9,{size}-'})
        self.assertEqual(mixed.headers['Content-Range'], f'bytes 0-9/{size}')
        self.assertEqual(len(mixed.content), 10)

    def test_06_only_an_index_addresses_a_file(self):
        """An index past the list, a word and a file name are all plain 404s.

        @verifies REQ_INTEROP_073
        """
        _, manifest = self._manifest()
        files = manifest['files']
        self._assert_plain_404(self._file_url(len(files)))
        self._assert_plain_404(self._file_url('first'))
        self._assert_plain_404(self._file_url(files[0]['name']))
        self._assert_plain_404(self._file_url('-1'))

    def test_07_an_unknown_recording_says_so(self):
        """Only "no such recording" carries the vendor code.

        @verifies REQ_INTEROP_073
        """
        unknown = 'fault_LIDAR_RANGE_INVALID_1'
        for endpoint in (f'{APP_ENDPOINT}/bulk-data/rosbags/{unknown}/manifest',
                         self._file_url(0, unknown)):
            body = self.get_raw(endpoint, expected_status=404).json()
            self.assertEqual(body.get('error_code'), 'vendor-error', body)
            self.assertEqual(body.get('vendor_code'), NOT_FOUND_VENDOR_CODE, body)

        # A resuming client asks with a Range. The answer has to be the same
        # whole error document, not a slice of it.
        for value in ('bytes=10-', 'bytes=5000000-'):
            body = self.get_raw(self._file_url(0, unknown), expected_status=404,
                                headers={'Range': value}).json()
            self.assertEqual(body.get('vendor_code'), NOT_FOUND_VENDOR_CODE, body)

    def test_08_a_fault_code_is_not_a_recording_id(self):
        """The new routes take a recording id only; the download route keeps its fallback.

        @verifies REQ_INTEROP_073
        """
        body = self.get_raw(
            f'{APP_ENDPOINT}/bulk-data/rosbags/{FAULT_CODE}/manifest',
            expected_status=404).json()
        self.assertEqual(body.get('vendor_code'), NOT_FOUND_VENDOR_CODE, body)

        legacy = self.get_raw(f'{APP_ENDPOINT}/bulk-data/rosbags/{FAULT_CODE}', timeout=30)
        self.assertEqual(legacy.content[:5], b'\x89MCAP')

    def test_08b_the_download_route_resolves_ranges_the_same_way(self):
        """The one-file download route clamps, refuses and reports ranges like the file route.

        @verifies REQ_INTEROP_073
        """
        endpoint = f'{APP_ENDPOINT}/bulk-data/rosbags/{self.recording_id}'
        whole = self.get_raw(endpoint, timeout=30).content
        size = len(whole)

        clamped = self.get_raw(endpoint, expected_status=206, timeout=30,
                               headers={'Range': f'bytes=1000-{size + 1000}'})
        self.assertEqual(clamped.headers['Content-Range'], f'bytes 1000-{size - 1}/{size}')
        self.assertEqual(int(clamped.headers['Content-Length']), size - 1000)
        self.assertEqual(clamped.content, whole[1000:])

        refused = self.get_raw(endpoint, expected_status=416,
                               headers={'Range': f'bytes={size + 1000}-'})
        self.assertEqual(refused.headers.get('Content-Range'), f'bytes */{size}')
        self.assertEqual(refused.json()['parameters']['size'], size)

    def test_09_another_entity_does_not_reach_the_recording(self):
        """A recording that exists is never reported as missing.

        @verifies REQ_INTEROP_073
        """
        self._assert_plain_404(
            f'{OTHER_APP_ENDPOINT}/bulk-data/rosbags/{self.recording_id}/manifest')
        self._assert_plain_404(
            f'{OTHER_APP_ENDPOINT}/bulk-data/rosbags/{self.recording_id}/files/0')
        self._assert_plain_404(
            f'/apps/no_such_app/bulk-data/rosbags/{self.recording_id}/manifest')

    def test_10_the_manifest_does_not_change(self):
        """Two reads return the same bytes: they are the recording's identity.

        @verifies REQ_INTEROP_073
        """
        first, _ = self._manifest()
        second, _ = self._manifest()
        self.assertEqual(sha256_hex(first.content), sha256_hex(second.content))

    def test_11_a_missing_first_file_fails_the_download(self):
        """Without its first file the download answers 500 and serves no later file.

        @verifies REQ_INTEROP_073
        """
        self.assertIsNotNone(self.recording_id, 'test_01 must run first')
        _, manifest = self._manifest()
        bag_dir = os.path.join(ROSBAG_STORAGE_PATH, self.recording_id)
        first = os.path.join(bag_dir, manifest['files'][0]['name'])
        # The bag keeps a later .mcap, so a directory scan would find one.
        aside = first + '.aside'
        os.replace(first, aside)
        try:
            self.get_raw(f'{APP_ENDPOINT}/bulk-data/rosbags/{self.recording_id}',
                         expected_status=500, timeout=30)
            item = self._listed_recording()
            self.assertIsNotNone(item, 'the recording is no longer listed')
            self.assertEqual(item['x-medkit'].get('storage_files'), len(manifest['files']))
        finally:
            os.replace(aside, first)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown checks."""

    def test_exit_codes(self, proc_info):
        """Gateway and fault manager exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)

    def test_cleanup_temp_directory(self):
        """Remove this run's bag storage."""
        import shutil
        if os.path.exists(ROSBAG_STORAGE_PATH):
            shutil.rmtree(ROSBAG_STORAGE_PATH, ignore_errors=True)
