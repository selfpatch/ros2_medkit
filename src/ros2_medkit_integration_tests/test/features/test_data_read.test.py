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

"""Feature tests for data read endpoints (app data, component topic data).

Validates reading topic data from apps and components, data structure,
error handling, data categories/groups, and area/function data.

"""

import unittest

import launch_testing
import launch_testing.actions
import requests

from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import (
    ACTUATOR_NODES,
    create_test_launch,
    SENSOR_NODES,
    SERVICE_NODES,
)


def generate_test_description():
    return create_test_launch(
        demo_nodes=SENSOR_NODES + ACTUATOR_NODES + SERVICE_NODES,
        fault_manager=False,
    )


class TestDataRead(GatewayTestCase):
    """Data read endpoint tests for apps, components, areas, and functions."""

    MIN_EXPECTED_APPS = 5
    REQUIRED_APPS = {'temp_sensor', 'rpm_sensor', 'pressure_sensor'}
    REQUIRED_AREAS = {'powertrain', 'chassis', 'body'}

    # ------------------------------------------------------------------
    # App data (test_07-12)
    # ------------------------------------------------------------------

    def test_app_data_powertrain_engine(self):
        """GET /apps/{app_id}/data for engine temperature sensor app.

        Apps are ROS 2 nodes. The temp_sensor app publishes temperature data.

        @verifies REQ_INTEROP_018
        """
        data = self.poll_endpoint('/apps/temp_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        if len(items) > 0:
            for topic_data in items:
                self.assertIn('id', topic_data)
                self.assertIn('name', topic_data)
                # direction is now in x-medkit.ros2
                self.assertIn('x-medkit', topic_data)
                x_medkit = topic_data['x-medkit']
                self.assertIn('ros2', x_medkit)
                self.assertIn('direction', x_medkit['ros2'])
                direction = x_medkit['ros2']['direction']
                self.assertIn(direction, ['publish', 'subscribe', 'both'])

    def test_app_data_chassis_brakes(self):
        """GET /apps/{app_id}/data for brakes pressure sensor app.

        @verifies REQ_INTEROP_018
        """
        data = self.poll_endpoint('/apps/pressure_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        if len(items) > 0:
            for topic_data in items:
                self.assertIn('id', topic_data)
                self.assertIn('name', topic_data)
                self.assertIn('x-medkit', topic_data)
                x_medkit = topic_data['x-medkit']
                self.assertIn('ros2', x_medkit)
                self.assertIn('direction', x_medkit['ros2'])

    def test_app_data_body_door(self):
        """GET /apps/{app_id}/data for door status sensor app.

        @verifies REQ_INTEROP_018
        """
        data = self.poll_endpoint('/apps/status_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        if len(items) > 0:
            for topic_data in items:
                self.assertIn('id', topic_data)
                self.assertIn('name', topic_data)
                self.assertIn('x-medkit', topic_data)
                x_medkit = topic_data['x-medkit']
                self.assertIn('ros2', x_medkit)
                self.assertIn('direction', x_medkit['ros2'])

    def test_app_data_structure(self):
        """GET /apps/{app_id}/data response structure.

        @verifies REQ_INTEROP_018
        """
        data = self.poll_endpoint('/apps/temp_sensor/data')
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list, 'Response should have items array')

        if len(items) > 0:
            first_item = items[0]
            self.assertIn('id', first_item, "Each item should have 'id' field")
            self.assertIn('name', first_item, "Each item should have 'name' field")
            self.assertIn('x-medkit', first_item, "Each item should have 'x-medkit' field")
            x_medkit = first_item['x-medkit']
            self.assertIn('ros2', x_medkit, 'x-medkit should have ros2 section')
            self.assertIn('direction', x_medkit['ros2'], 'x-medkit.ros2 should have direction')
            self.assertIsInstance(
                first_item['name'], str, "'name' should be a string"
            )
            self.assertIn(x_medkit['ros2']['direction'], ['publish', 'subscribe', 'both'])

    def test_app_nonexistent_error(self):
        """GET /apps/{app_id}/data returns 404 for nonexistent app.

        @verifies REQ_INTEROP_018
        """
        response = self.get_raw('/apps/nonexistent_app/data', expected_status=404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertIn('entity_id', data['parameters'])
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_app')

    def test_app_no_topics(self):
        """GET /apps/{app_id}/data returns empty array for app with no topics.

        The calibration app typically has only services, no topics.
        Uses poll_endpoint to avoid race conditions with service-only nodes
        that can transiently disappear from the ROS 2 graph between
        discovery cycles (observed on Jazzy).

        @verifies REQ_INTEROP_018
        """
        data = self.poll_endpoint('/apps/calibration/data')
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list, 'Response should have items array')

    # ------------------------------------------------------------------
    # Component topic data (test_17-24)
    # ------------------------------------------------------------------

    def test_component_topic_temperature(self):
        """GET /components/{component_id}/data/{topic_name} for temperature topic.

        Uses synthetic 'powertrain' component which aggregates apps in that namespace.

        @verifies REQ_INTEROP_019
        """
        topic_path = self.encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # SOVD ReadValue format with x-medkit extension
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/powertrain/engine/temperature')
        self.assertIn('timestamp', x_medkit)
        self.assertIn('status', x_medkit)
        self.assertIsInstance(x_medkit['timestamp'], int)
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

    def test_component_topic_rpm(self):
        """GET /components/{component_id}/data/{topic_name} for RPM topic.

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        topic_path = self.encode_topic_path('/powertrain/engine/rpm')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/powertrain/engine/rpm')
        self.assertIn('timestamp', x_medkit)
        self.assertIn('status', x_medkit)
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

    def test_component_topic_pressure(self):
        """GET /components/{component_id}/data/{topic_name} for pressure topic.

        Uses synthetic 'chassis' component.

        @verifies REQ_INTEROP_019
        """
        topic_path = self.encode_topic_path('/chassis/brakes/pressure')
        response = requests.get(
            f'{self.BASE_URL}/components/chassis/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/chassis/brakes/pressure')
        self.assertIn('timestamp', x_medkit)
        self.assertIn('status', x_medkit)
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

    def test_component_topic_data_structure(self):
        """GET /components/{component_id}/data/{topic_name} response structure.

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        topic_path = self.encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        # Verify SOVD ReadValue structure with x-medkit extension
        self.assertIn('id', data, "Response should have 'id' field")
        self.assertIn('data', data, "Response should have 'data' field")
        self.assertIn('x-medkit', data, "Response should have 'x-medkit' field")

        # Verify x-medkit fields
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit, 'x-medkit should have ros2 section')
        self.assertIn('topic', x_medkit['ros2'], 'x-medkit.ros2 should have topic')
        self.assertIn('timestamp', x_medkit, 'x-medkit should have timestamp')
        self.assertIn('status', x_medkit, 'x-medkit should have status')

        # Verify field types
        self.assertIsInstance(x_medkit['ros2']['topic'], str, "'ros2.topic' should be a string")
        self.assertIsInstance(
            x_medkit['timestamp'], int, "'timestamp' should be an integer (nanoseconds)"
        )
        self.assertIn(x_medkit['status'], ['data', 'metadata_only'])

        # Verify topic path format
        self.assertTrue(
            x_medkit['ros2']['topic'].startswith('/'),
            "Topic should be an absolute path starting with '/'",
        )

    def test_component_nonexistent_topic_metadata_only(self):
        """Nonexistent topic returns 200 with metadata_only status.

        The gateway returns metadata about the topic even if no data is available.
        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        topic_path = self.encode_topic_path('/some/nonexistent/topic')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('id', data)
        self.assertIn('data', data)
        self.assertIn('x-medkit', data)

        x_medkit = data['x-medkit']
        self.assertEqual(x_medkit['entity_id'], 'powertrain')
        self.assertEqual(x_medkit['status'], 'metadata_only')

    def test_component_topic_nonexistent_component_error(self):
        """GET endpoint returns 404 for nonexistent component.

        @verifies REQ_INTEROP_019
        """
        topic_path = self.encode_topic_path('/some/topic')
        response = requests.get(
            f'{self.BASE_URL}/components/nonexistent_component/data/{topic_path}',
            timeout=5,
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertIn('entity_id', data['parameters'])
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_component')

    def test_component_topic_with_slashes(self):
        """GET with percent-encoded slashes in topic path.

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        topic_path = self.encode_topic_path('/powertrain/engine/temperature')
        response = requests.get(
            f'{self.BASE_URL}/components/powertrain/data/{topic_path}', timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('x-medkit', data)
        x_medkit = data['x-medkit']
        self.assertIn('ros2', x_medkit)
        self.assertEqual(x_medkit['ros2']['topic'], '/powertrain/engine/temperature')

    def test_component_topic_valid_names(self):
        """Valid topic names work correctly.

        Uses synthetic 'powertrain' component.

        @verifies REQ_INTEROP_019
        """
        valid_topic_names = [
            'topic_name',
            'topic_name_123',
            'TopicName',
            'topic123',
        ]

        for valid_topic in valid_topic_names:
            response = requests.get(
                f'{self.BASE_URL}/components/powertrain/data/{valid_topic}', timeout=10
            )
            self.assertIn(
                response.status_code,
                [200, 404],
                f'Expected 200 or 404 for valid topic name: {valid_topic}, '
                f'got {response.status_code}',
            )

    # ------------------------------------------------------------------
    # Data categories and groups (test_96-97)
    # ------------------------------------------------------------------

    def test_list_data_categories(self):
        """GET /apps/{id}/data-categories returns 501 Not Implemented.

        TODO: Data categories are not yet implemented in the gateway.
        """
        app_id = 'temp_sensor'

        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/data-categories',
            timeout=10
        )

        self.assertEqual(
            response.status_code, 501,
            f'Expected 501 Not Implemented, got {response.status_code}'
        )

        data = response.json()
        self.assertIn('error_code', data)

    def test_list_data_groups(self):
        """GET /apps/{id}/data-groups returns 501 Not Implemented.

        TODO: Data groups are not yet implemented in the gateway.
        """
        app_id = 'temp_sensor'

        response = requests.get(
            f'{self.BASE_URL}/apps/{app_id}/data-groups',
            timeout=10
        )

        self.assertEqual(
            response.status_code, 501,
            f'Expected 501 Not Implemented, got {response.status_code}'
        )

        data = response.json()
        self.assertIn('error_code', data)

    # ------------------------------------------------------------------
    # Area data endpoints (test_109-112)
    # ------------------------------------------------------------------

    def test_list_area_data(self):
        """GET /areas/{area_id}/data returns aggregated topics for area.

        Areas aggregate data from all components and apps in their hierarchy.
        The powertrain area should include topics from engine sensors.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/powertrain/data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)

        if len(items) > 0:
            for topic_data in items:
                self.assertIn('id', topic_data)
                self.assertIn('name', topic_data)
                self.assertIn('x-medkit', topic_data)
                x_medkit = topic_data['x-medkit']
                self.assertIn('ros2', x_medkit)
                self.assertIn('direction', x_medkit['ros2'])
                direction = x_medkit['ros2']['direction']
                self.assertIn(direction, ['publish', 'subscribe', 'both'])

        # Should include aggregated_from in x-medkit
        self.assertIn('x-medkit', data)
        self.assertIn('aggregated_from', data['x-medkit'])
        self.assertIn('powertrain', data['x-medkit']['aggregated_from'])

    def test_list_area_data_nonexistent(self):
        """GET /areas/{area_id}/data returns 404 for nonexistent area.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/nonexistent_area/data',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertIn('entity_id', data['parameters'])
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_area')

    def test_list_area_data_root(self):
        """GET /areas/root/data returns all topics system-wide.

        The root area aggregates all entities in the system.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/root/data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        items = data['items']
        self.assertIsInstance(items, list)
        self.assertGreater(len(items), 0, 'Root area should have aggregated topics')

    def test_list_area_data_empty(self):
        """GET /areas/{area_id}/data returns 200 with empty items for area with no data.

        Some areas may exist but have no direct topics - they should return 200
        with empty items, not 404.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/areas/chassis/data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)
        self.assertIn('x-medkit', data)
        self.assertIn('aggregated_from', data['x-medkit'])
        self.assertIn('chassis', data['x-medkit']['aggregated_from'])

    # ------------------------------------------------------------------
    # Function data endpoints (test_113-115)
    # ------------------------------------------------------------------

    def test_list_function_data(self):
        """GET /functions/{function_id}/data returns data items for function.

        Functions are logical groupings and may aggregate data from multiple sources.

        @verifies REQ_INTEROP_018
        """
        funcs_response = requests.get(f'{self.BASE_URL}/functions', timeout=10)
        if funcs_response.status_code != 200:
            self.fail('No functions available for testing')

        funcs = funcs_response.json().get('items', [])
        if len(funcs) == 0:
            self.fail('No functions available for testing')

        func_id = funcs[0]['id']

        response = requests.get(
            f'{self.BASE_URL}/functions/{func_id}/data',
            timeout=10
        )
        self.assertEqual(response.status_code, 200)

        data = response.json()
        self.assertIn('items', data)
        self.assertIsInstance(data['items'], list)

        self.assertIn('x-medkit', data)
        self.assertIn('aggregated_from', data['x-medkit'])
        self.assertIn(func_id, data['x-medkit']['aggregated_from'])

    def test_list_function_data_nonexistent(self):
        """GET /functions/{function_id}/data returns 404 for nonexistent function.

        @verifies REQ_INTEROP_018
        """
        response = requests.get(
            f'{self.BASE_URL}/functions/nonexistent_function/data',
            timeout=10
        )
        self.assertEqual(response.status_code, 404)

        data = response.json()
        self.assertIn('error_code', data)
        self.assertEqual(data['message'], 'Entity not found')
        self.assertIn('parameters', data)
        self.assertIn('entity_id', data['parameters'])
        self.assertEqual(data['parameters'].get('entity_id'), 'nonexistent_function')

    def test_list_function_data_invalid_id(self):
        """GET /functions/{function_id}/data rejects invalid function IDs.

        @verifies REQ_INTEROP_018
        """
        invalid_ids = [
            'func;drop',
            'func<script>',
            'func name',
        ]

        for invalid_id in invalid_ids:
            response = requests.get(
                f'{self.BASE_URL}/functions/{invalid_id}/data',
                timeout=10
            )
            self.assertEqual(
                response.status_code,
                400,
                f'Expected 400 for function_id: {invalid_id}'
            )

            data = response.json()
            self.assertIn('error_code', data)
            self.assertIn('invalid', data['message'].lower())


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
