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

"""A data subscription on a plugin item reads the item GET of the same URI reads.

The item names carry characters that a client percent-encodes in a URI path
(a space, a quote), like PLC symbols such as ``"DB1".temp``.
"""

import json
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
PLAIN_ITEM = 'temperature'
QUOTED_ITEM = '"DB1".temp'
SPACED_ITEM = 'Motor Speed'


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
            'sse.max_clients': 10,
            'sse.max_subscriptions': 50,
        },
    )


class TestPluginItemSubscription(GatewayTestCase):

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {PLUGIN_APP}

    def _item_uri(self, item):
        return f'{API_BASE_PATH}/apps/{PLUGIN_APP}/data/{quote(item, safe="")}'

    def _first_event(self, item, expect_status=201):
        resource = self._item_uri(item)
        r = requests.post(
            f'{self.BASE_URL}/apps/{PLUGIN_APP}/cyclic-subscriptions',
            json={'resource': resource, 'interval': 'fast', 'duration': 30},
            timeout=5,
        )
        self.assertEqual(r.status_code, expect_status, r.text)
        if expect_status != 201:
            return r
        body = r.json()
        # The resource is echoed as the client sent it.
        self.assertEqual(body['observed_resource'], resource)
        sub_id = body['id']
        try:
            with requests.get(self.BASE_URL.removesuffix(API_BASE_PATH) +
                              body['event_source'], stream=True,
                              timeout=15) as stream:
                for line in stream.iter_lines(decode_unicode=True):
                    if line and line.startswith('data:'):
                        return json.loads(line[len('data:'):].strip())
        finally:
            requests.delete(
                f'{self.BASE_URL}/apps/{PLUGIN_APP}/cyclic-subscriptions/'
                f'{sub_id}', timeout=5)
        self.fail('SSE stream ended without an event')

    def _assert_event_matches_get(self, item):
        get = requests.get(
            f'{self.BASE_URL}/apps/{PLUGIN_APP}/data/{quote(item, safe="")}',
            timeout=5)
        self.assertEqual(get.status_code, 200, get.text)
        event = self._first_event(item)
        self.assertEqual(
            event.get('payload', event), get.json(),
            f'event {event} differs from GET {get.json()} for {item!r}')

    def test_plain_item_matches_get(self):
        """Check one case.

        @verifies REQ_INTEROP_089
        """
        self._assert_event_matches_get(PLAIN_ITEM)

    def test_quoted_item_matches_get(self):
        """Check one case.

        @verifies REQ_INTEROP_089
        """
        self._assert_event_matches_get(QUOTED_ITEM)

    def test_spaced_item_matches_get(self):
        """Check one case.

        @verifies REQ_INTEROP_089
        """
        self._assert_event_matches_get(SPACED_ITEM)

    def test_encoded_dot_dot_segment_is_refused(self):
        """Check one case.

        @verifies REQ_INTEROP_089
        """
        r = requests.post(
            f'{self.BASE_URL}/apps/{PLUGIN_APP}/cyclic-subscriptions',
            json={'resource':
                  f'{API_BASE_PATH}/apps/{PLUGIN_APP}/data/%2E%2E',
                  'interval': 'fast', 'duration': 30},
            timeout=5,
        )
        self.assertEqual(r.status_code, 400, r.text)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(info.returncode, ALLOWED_EXIT_CODES)
