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

"""Feature tests for data write (PUT) endpoints.

Validates publishing data to topics, validation errors for missing fields
and invalid formats, nonexistent components, and invalid JSON bodies.

Migrated from:
- test_25_publish_brake_command
- test_26_publish_validation_missing_type
- test_27_publish_validation_missing_data
- test_28_publish_validation_invalid_type_format
- test_29_publish_nonexistent_component
- test_30_publish_invalid_json_body
"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def generate_test_description():
    return create_test_launch(
        demo_nodes=['actuator', 'controller'],
        fault_manager=False,
    )


class TestDataWrite(GatewayTestCase):
    """Data write endpoint tests for publishing to topics."""

    MIN_EXPECTED_APPS = 2
    REQUIRED_APPS = {'actuator', 'controller'}

    def test_publish_brake_command(self):
        """PUT /components/{component_id}/data/{topic_name} publishes data.

        Uses synthetic 'chassis' component.

        @verifies REQ_INTEROP_020
        """
        topic_path = self.encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32', 'data': {'data': 50.0}},
            timeout=10,
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD write response format with x-medkit extension
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/chassis/brakes/command')
        self.assertEqual(x_medkit['ros2']['type'], 'std_msgs/msg/Float32')
        self.assertEqual(x_medkit['status'], 'published')

    def test_publish_validation_missing_type(self):
        """PUT returns 400 when type is missing.

        @verifies REQ_INTEROP_020
        """
        topic_path = self.encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            json={'data': {'data': 50.0}},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('type', data['message'].lower())

    def test_publish_validation_missing_data(self):
        """PUT returns 400 when data is missing.

        @verifies REQ_INTEROP_020
        """
        topic_path = self.encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('data', data['message'].lower())

    def test_publish_validation_invalid_type_format(self):
        """PUT returns 400 for invalid message type format.

        @verifies REQ_INTEROP_020
        """
        invalid_types = [
            'InvalidType',  # No slashes
            'std_msgs/Float32',  # Missing /msg/
            'std_msgs/srv/Empty',  # Wrong middle part (srv instead of msg)
            'a/b/c/d',  # Too many slashes (no /msg/)
            'a/msg/b/c',  # Too many slashes (3 instead of 2)
            '/msg/Type',  # Missing package (starts with /)
            'package/msg/',  # Missing type (ends with /)
        ]

        topic_path = self.encode_topic_path('/chassis/brakes/command')
        for invalid_type in invalid_types:
            response = requests.put(
                f'{self.BASE_URL}/components/chassis/data/{topic_path}',
                json={'type': invalid_type, 'data': {'data': 50.0}},
                timeout=5,
            )
            self.assertEqual(
                response.status_code, 400, f'Expected 400 for type: {invalid_type}'
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertEqual(data['message'], 'Invalid message type format')

    def test_publish_nonexistent_component(self):
        """PUT returns 404 for unknown component.

        @verifies REQ_INTEROP_020
        """
        topic_path = self.encode_topic_path('/some/topic/path')
        response = requests.put(
            f'{self.BASE_URL}/components/nonexistent_component/data/{topic_path}',
            json={'type': 'std_msgs/msg/Float32', 'data': {'data': 50.0}},
            timeout=5,
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_component')

    def test_publish_invalid_json_body(self):
        """PUT returns 400 for invalid JSON body.

        @verifies REQ_INTEROP_020
        """
        topic_path = self.encode_topic_path('/chassis/brakes/command')
        response = requests.put(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}',
            data='not valid json',
            headers={'Content-Type': 'application/json'},
            timeout=5,
        )
        self.assertEqual(response.status_code, 400)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertIn('json', data['message'].lower())


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly (SIGTERM allowed)."""
        for info in proc_info:
            allowed = {0, -2, -15}  # OK, SIGINT, SIGTERM
            self.assertIn(
                info.returncode, allowed,
                f'Process {info.process_name} exited with code {info.returncode}'
            )
