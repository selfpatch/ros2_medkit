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

"""Rosbag routes while the fault manager does not answer.

"There are no recordings" and "the fault manager did not answer" are different
facts, and a client that deletes or gives up on the first must not be told it
when the second is true. No fault manager runs here at all, so every rosbag
route has to answer 503: the listing instead of an empty collection, and the
recording routes instead of the 404 that says a recording does not exist.
"""

import unittest

import launch_testing

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


APP_ENDPOINT = '/apps/temp_sensor'
RECORDING = 'fault_ENGINE_OVERHEAT_1785441426087'


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        # Every request below waits this long for a service that never appears.
        gateway_params={'fault_manager.service_timeout_sec': 1.0},
    )


class TestRosbagRecordingUnavailable(GatewayTestCase):
    """No fault manager on the graph."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def _assert_503(self, endpoint):
        body = self.get_raw(endpoint, expected_status=503, timeout=15).json()
        self.assertNotIn('vendor_code', body, body)

    def test_the_listing_is_not_an_empty_collection(self):
        """A listing that could not be read is refused.

        @verifies REQ_INTEROP_072
        """
        self._assert_503(f'{APP_ENDPOINT}/bulk-data/rosbags')

    def test_the_manifest_route_does_not_claim_absence(self):
        """503, not the 404 that proves the recording is gone.

        @verifies REQ_INTEROP_073
        """
        self._assert_503(f'{APP_ENDPOINT}/bulk-data/rosbags/{RECORDING}/manifest')

    def test_the_file_route_does_not_claim_absence(self):
        """503, not the 404 that proves the recording is gone.

        @verifies REQ_INTEROP_073
        """
        self._assert_503(f'{APP_ENDPOINT}/bulk-data/rosbags/{RECORDING}/files/0')

    def test_the_category_is_still_listed(self):
        """The category exists whether or not its backend answers right now.

        @verifies REQ_INTEROP_071
        """
        categories = self.get_json(f'{APP_ENDPOINT}/bulk-data')
        self.assertIn('rosbags', categories['items'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Post-shutdown checks."""

    def test_exit_codes(self, proc_info):
        """The gateway exits cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=ALLOWED_EXIT_CODES)
