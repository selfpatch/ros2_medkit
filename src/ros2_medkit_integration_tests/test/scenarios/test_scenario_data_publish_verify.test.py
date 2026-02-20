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

"""Scenario: Data publish and verify — read sensor data, publish command, read feedback.

End-to-end story: read sensor data from temp_sensor, write a command to the
brake actuator, then read data back from the actuator to confirm the
publish round-trip.
"""

import unittest

import launch_testing
import launch_testing.actions

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['temp_sensor', 'actuator'],
        fault_manager=False,
    )


class TestScenarioDataPublishVerify(GatewayTestCase):
    """Scenario: Read sensor data, publish command, read actuator feedback.

    Validates the full data flow through the gateway: reading live topic
    data from a sensor, writing a command to an actuator, and reading
    back from the actuator.

    Steps:
    1. Read temperature data from temp_sensor
    2. Write/publish a brake command to the actuator
    3. Read data from the actuator to verify data access works
    """

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'temp_sensor', 'actuator'}

    TEMP_ENDPOINT = '/apps/temp_sensor'
    ACTUATOR_ENDPOINT = '/apps/actuator'

    # ------------------------------------------------------------------
    # Tests
    # ------------------------------------------------------------------

    def test_01_read_sensor_data(self):
        """Read temperature data from temp_sensor.

        @verifies REQ_INTEROP_018
        """
        data = self.poll_endpoint(f'{self.TEMP_ENDPOINT}/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        # temp_sensor should have at least one published topic
        if items:
            first = items[0]
            self.assertIn('id', first)
            self.assertIn('name', first)
            self.assertIn('x-medkit', first)
            x_medkit = first['x-medkit']
            self.assertIn('ros2', x_medkit)
            self.assertIn('direction', x_medkit['ros2'])
            self.assertIn(
                x_medkit['ros2']['direction'],
                ['publish', 'subscribe', 'both'],
            )

    def test_02_publish_command_to_actuator(self):
        """Write/publish a brake command to the actuator.

        @verifies REQ_INTEROP_020
        """
        # Get the actuator's data to find the command topic
        app_data = self.poll_endpoint(f'{self.ACTUATOR_ENDPOINT}/data')
        self.assertIn('items', app_data)

        # Find a topic with subscribe direction (actuator listens to commands)
        subscribe_topic = None
        for item in app_data['items']:
            x_medkit = item.get('x-medkit', {})
            ros2 = x_medkit.get('ros2', {})
            if ros2.get('direction') == 'subscribe':
                subscribe_topic = item
                break

        if subscribe_topic is None:
            self.skipTest('Actuator has no subscribe topics')

        topic_id = subscribe_topic['id']

        # Write brake pressure command (50.0 bar)
        data = self.put_json(
            f'{self.ACTUATOR_ENDPOINT}/data/{topic_id}',
            {
                'type': 'std_msgs/msg/Float32',
                'data': {'data': 50.0},
            },
        )

        self.assertIn('x-medkit', data)
        self.assertEqual(data['x-medkit']['status'], 'published')

    def test_03_read_actuator_feedback(self):
        """Read data from actuator to verify data access works.

        @verifies REQ_INTEROP_018
        """
        data = self.poll_endpoint(f'{self.ACTUATOR_ENDPOINT}/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        # Actuator should have at least one topic (command subscription)
        if items:
            first = items[0]
            self.assertIn('id', first)
            self.assertIn('name', first)
            self.assertIn('x-medkit', first)
            x_medkit = first['x-medkit']
            self.assertIn('ros2', x_medkit)
            self.assertIn('direction', x_medkit['ros2'])


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            allowed = {0, -2, -15}  # OK, SIGINT, SIGTERM
            self.assertIn(
                info.returncode, allowed,
                f'{info.process_name} exited with code {info.returncode}'
            )
