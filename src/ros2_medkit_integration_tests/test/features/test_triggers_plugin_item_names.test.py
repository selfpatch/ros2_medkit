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

"""A data trigger on a plugin item addresses the item as GET of the same URI does.

The item names carry characters that a client percent-encodes in a URI path
(a space, a quote), like PLC symbols such as ``"DB1".temp``.
"""

import os
import unittest
from urllib.parse import quote

from ament_index_python.packages import get_package_prefix
import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES, API_BASE_PATH
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch

PLUGIN_APP = 'test_plc_app'
ITEMS = ['temperature', '"DB1".temp', 'Motor Speed']
DOT_DOT_SPELLINGS = ['%2E%2E', '%2e%2e', '.%2E', '%2E.']


def _get_plugin_path(so_name):
    pkg_prefix = get_package_prefix('ros2_medkit_gateway')
    return os.path.join(pkg_prefix, 'lib', 'ros2_medkit_gateway', so_name)


def generate_test_description():
    return create_test_launch(
        demo_nodes=[],
        fault_manager=False,
        gateway_params={
            'plugins': ['test_data_provider'],
            'plugins.test_data_provider.path':
                _get_plugin_path('libtest_data_provider_plugin.so'),
        },
    )


class TestTriggersPluginItemNames(GatewayTestCase):

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {PLUGIN_APP}

    def _post_trigger(self, resource):
        return requests.post(
            f'{self.BASE_URL}/apps/{PLUGIN_APP}/triggers',
            json={'resource': resource,
                  'trigger_condition': {'condition_type': 'OnChange'}},
            timeout=5,
        )

    def _delete_trigger(self, trigger_id):
        requests.delete(
            f'{self.BASE_URL}/apps/{PLUGIN_APP}/triggers/{trigger_id}',
            timeout=5)

    def test_encoded_item_name_is_accepted(self):
        """Check that the URI a client would GET is a valid trigger resource.

        @verifies REQ_INTEROP_029
        """
        for item in ITEMS:
            with self.subTest(item=item):
                resource = f'{API_BASE_PATH}/apps/{PLUGIN_APP}/data/{quote(item, safe="")}'
                get = requests.get(self.BASE_URL.removesuffix(API_BASE_PATH) + resource,
                                   timeout=5)
                self.assertEqual(get.status_code, 200, get.text)
                r = self._post_trigger(resource)
                self.assertEqual(r.status_code, 201, r.text)
                body = r.json()
                try:
                    # The resource is echoed as the client sent it.
                    self.assertEqual(body['observed_resource'], resource)
                    self.assertEqual(body['status'], 'active')
                finally:
                    self._delete_trigger(body['id'])

    def test_encoded_dot_dot_segment_is_refused(self):
        """Check that a '..' segment is refused however it is spelled.

        @verifies REQ_INTEROP_029
        """
        for collection in ['data', 'faults']:
            for spelling in DOT_DOT_SPELLINGS:
                with self.subTest(collection=collection, spelling=spelling):
                    r = self._post_trigger(
                        f'{API_BASE_PATH}/apps/{PLUGIN_APP}/{collection}/{spelling}')
                    if r.status_code == 201:
                        self._delete_trigger(r.json()['id'])
                    self.assertEqual(r.status_code, 400, r.text)
                    self.assertEqual(r.json().get('vendor_code'),
                                     'x-medkit-invalid-resource-uri', r.text)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES)
