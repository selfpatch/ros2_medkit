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

"""Integration tests for multi-collection cyclic subscriptions.

Validates that cyclic subscriptions can be created for data, faults,
and configurations collections, and that error cases (unsupported
collection, invalid URI, entity mismatch, unsupported protocol,
path traversal) return 400.

"""

import time
import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    """Launch gateway with demo nodes for multi-collection subscription testing."""
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=True,
        gateway_params={
            'sse.max_clients': 10,
            'sse.max_subscriptions': 50,
        },
    )


class TestMultiCollectionSubscriptions(GatewayTestCase):
    """Test cyclic subscriptions across different resource collections."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    app_id = None
    topic_id = None
    data_resource_uri = None

    @classmethod
    def setUpClass(cls):
        """Wait for gateway, discover temp sensor, and find a data topic."""
        super().setUpClass()

        # Find the temp_sensor app
        try:
            r = requests.get(f'{cls.BASE_URL}/apps', timeout=5)
            if r.status_code == 200:
                apps = r.json().get('items', [])
                for app in apps:
                    if 'temp_sensor' in app.get('id', '').lower() or \
                       'engine' in app.get('id', '').lower():
                        cls.app_id = app['id']
                        break
        except requests.exceptions.RequestException:
            pass

        if not cls.app_id:
            raise AssertionError('Demo temp sensor app not discovered')

        # Wait for data availability on the discovered app
        # Skip ROS 2 system topics that don't have continuous data flow
        system_topics = {'/parameter_events', '/rosout'}
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            try:
                r = requests.get(
                    f'{cls.BASE_URL}/apps/{cls.app_id}/data', timeout=5,
                )
                if r.status_code == 200:
                    items = r.json().get('items', [])
                    for item in items:
                        if item['id'] not in system_topics:
                            cls.topic_id = item['id']
                            cls.data_resource_uri = (
                                f'/api/v1/apps/{cls.app_id}/data{cls.topic_id}'
                            )
                            break
                    if cls.topic_id:
                        break
            except requests.exceptions.RequestException:
                pass
            time.sleep(1.0)

        if not cls.topic_id:
            raise AssertionError(
                f'No data items available for app {cls.app_id}'
            )

    def _create_subscription(self, resource, interval='normal', duration=60,
                             protocol=None):
        """Create a subscription and return the raw response."""
        body = {
            'resource': resource,
            'interval': interval,
            'duration': duration,
        }
        if protocol is not None:
            body['protocol'] = protocol
        return requests.post(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            json=body,
            timeout=5,
        )

    def _delete_subscription(self, sub_id):
        """Delete a subscription, ignoring errors for cleanup."""
        try:
            requests.delete(
                f'{self.BASE_URL}/apps/{self.app_id}'
                f'/cyclic-subscriptions/{sub_id}',
                timeout=5,
            )
        except requests.exceptions.RequestException:
            pass

    # ===================================================================
    # Happy path: data, faults, configurations subscriptions
    # ===================================================================

    def test_data_subscription_create(self):
        """Create a subscription on data collection (regression test).

        @verifies REQ_INTEROP_089
        """
        r = self._create_subscription(self.data_resource_uri)
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        self.assertIn('id', data)
        self.assertTrue(data['id'].startswith('sub_'))
        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'normal')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)
        self.assertTrue(data['event_source'].endswith('/events'))

    def test_faults_subscription_create(self):
        """Create a subscription on faults collection.

        @verifies REQ_INTEROP_089
        """
        resource = f'/api/v1/apps/{self.app_id}/faults'
        r = self._create_subscription(resource, interval='fast')
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'fast')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)

    def test_configurations_subscription_create(self):
        """Create a subscription on configurations collection.

        @verifies REQ_INTEROP_089
        """
        resource = f'/api/v1/apps/{self.app_id}/configurations'
        r = self._create_subscription(resource, interval='slow')
        self.assertEqual(r.status_code, 201, f'Create failed: {r.text}')

        data = r.json()
        self.addCleanup(self._delete_subscription, data['id'])

        self.assertEqual(data['protocol'], 'sse')
        self.assertEqual(data['interval'], 'slow')
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)

    # ===================================================================
    # CRUD operations: list, get, update, delete
    # ===================================================================

    def test_list_subscriptions(self):
        """List subscriptions returns items array.

        @verifies REQ_INTEROP_025
        """
        # Create a subscription first
        r = self._create_subscription(self.data_resource_uri, duration=60)
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']
        self.addCleanup(self._delete_subscription, sub_id)

        # List
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}/cyclic-subscriptions',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertIn('items', data)
        ids = [s['id'] for s in data['items']]
        self.assertIn(sub_id, ids)

    def test_get_subscription(self):
        """Get single subscription returns SOVD fields.

        @verifies REQ_INTEROP_026
        """
        r = self._create_subscription(self.data_resource_uri, duration=60)
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']
        self.addCleanup(self._delete_subscription, sub_id)

        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        data = r.json()
        self.assertEqual(data['id'], sub_id)
        self.assertIn('observed_resource', data)
        self.assertIn('event_source', data)
        self.assertIn('protocol', data)
        self.assertIn('interval', data)

    def test_update_subscription_interval(self):
        """Update subscription interval.

        @verifies REQ_INTEROP_027
        """
        r = self._create_subscription(
            self.data_resource_uri, interval='normal', duration=60,
        )
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']
        self.addCleanup(self._delete_subscription, sub_id)

        r = requests.put(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            json={'interval': 'fast'},
            timeout=5,
        )
        self.assertEqual(r.status_code, 200)
        self.assertEqual(r.json()['interval'], 'fast')

    def test_delete_subscription(self):
        """Delete subscription returns 204.

        @verifies REQ_INTEROP_028
        """
        r = self._create_subscription(self.data_resource_uri, duration=60)
        self.assertEqual(r.status_code, 201)
        sub_id = r.json()['id']

        r = requests.delete(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 204)

        # Verify it's gone
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/{sub_id}',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_get_nonexistent_subscription_returns_404(self):
        """Get nonexistent subscription returns 404."""
        r = requests.get(
            f'{self.BASE_URL}/apps/{self.app_id}'
            f'/cyclic-subscriptions/nonexistent_sub',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    # ===================================================================
    # Error cases
    # ===================================================================

    def test_unsupported_collection_returns_400(self):
        """Subscription on unknown collection returns 400."""
        resource = f'/api/v1/apps/{self.app_id}/unknown_collection'
        r = self._create_subscription(resource)
        self.assertEqual(r.status_code, 400)

    def test_invalid_resource_uri_returns_400(self):
        """Invalid resource URI format returns 400."""
        r = self._create_subscription('/invalid/uri')
        self.assertEqual(r.status_code, 400)

    def test_entity_mismatch_returns_400(self):
        """Resource URI referencing different entity returns 400."""
        resource = '/api/v1/apps/OTHER_nonexistent_entity/data/topic'
        r = self._create_subscription(resource)
        self.assertEqual(r.status_code, 400)

    def test_unsupported_protocol_returns_400(self):
        """Unsupported protocol returns 400."""
        r = self._create_subscription(
            self.data_resource_uri, protocol='mqtt',
        )
        self.assertEqual(r.status_code, 400)

    def test_path_traversal_returns_400(self):
        """Resource path with '..' returns 400."""
        resource = f'/api/v1/apps/{self.app_id}/data/../../../etc/passwd'
        r = self._create_subscription(resource)
        self.assertEqual(r.status_code, 400)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            self.assertIn(
                info.returncode, ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}'
            )
