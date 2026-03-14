#!/usr/bin/env python3
# Copyright 2026 selfpatch GmbH
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

"""Integration tests for topic-based beacon discovery plugin.

Validates the full beacon pipeline: demo_beacon_publisher publishes
MedkitDiscoveryHint messages on /ros2_medkit/discovery, the gateway's
TopicBeaconPlugin receives them, stores hints in BeaconHintStore,
maps them via BeaconEntityMapper, and serves enriched metadata at the
vendor extension endpoint GET /{entity_type}/{id}/x-medkit-topic-beacon.

The gateway runs in hybrid mode with the topic_beacon plugin loaded
and allow_new_entities=False, so the beacon enriches existing
runtime-discovered entities only.
"""

import os
import unittest

import launch_testing
import launch_testing.asserts
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_test_launch


def _get_beacon_plugin_path():
    """Get path to topic_beacon_plugin.so."""
    from ament_index_python.packages import get_package_prefix

    pkg_prefix = get_package_prefix('ros2_medkit_topic_beacon')
    return os.path.join(
        pkg_prefix, 'lib', 'ros2_medkit_topic_beacon', 'libtopic_beacon_plugin.so'
    )


def generate_test_description():
    """Launch gateway with topic_beacon plugin + beacon publisher demo node.

    Uses temp_sensor as the runtime-discovered entity that the beacon
    publisher enriches. The beacon publisher is added separately (not via
    DEMO_NODE_REGISTRY) so we can set custom parameters: entity_id pointing
    at temp_sensor, a transport_type, and a faster publish rate for tests.
    """
    plugin_path = _get_beacon_plugin_path()
    launch_description, context = create_test_launch(
        demo_nodes=['temp_sensor'],
        fault_manager=False,
        gateway_params={
            'discovery.mode': 'hybrid',
            'plugins': ['topic_beacon'],
            'plugins.topic_beacon.path': plugin_path,
            'plugins.topic_beacon.beacon_ttl_sec': 3.0,
            'plugins.topic_beacon.beacon_expiry_sec': 10.0,
            'plugins.topic_beacon.allow_new_entities': False,
        },
    )
    # Add beacon publisher with test-specific parameters.
    # Not using create_test_launch's demo_nodes because the registry entry
    # has default (empty) entity_id which gets rejected by the validator.
    from launch_ros.actions import Node as LaunchNode

    beacon_node = LaunchNode(
        package='ros2_medkit_integration_tests',
        executable='demo_beacon_publisher',
        name='beacon_publisher',
        parameters=[{
            'beacon_entity_id': 'temp_sensor',
            'beacon_transport_type': 'shared_memory',
            'beacon_rate_hz': 2.0,
            'beacon_process_name': 'test_beacon',
        }],
    )
    launch_description.add_action(beacon_node)
    return launch_description, context


class TestBeaconTopic(GatewayTestCase):
    """Topic beacon discovery plugin integration tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'temp_sensor'}

    def test_01_beacon_enrichment(self):
        """Verify entity metadata includes beacon data via vendor endpoint."""
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/x-medkit-topic-beacon',
            lambda d: d if d.get('status') == 'active' else None,
            timeout=30,
        )
        self.assertEqual(data['entity_id'], 'temp_sensor')
        self.assertEqual(data['transport_type'], 'shared_memory')
        self.assertGreater(data['process_id'], 0)
        self.assertIn('age_sec', data)
        self.assertIn('hostname', data)
        self.assertEqual(data['process_name'], 'test_beacon')

    def test_02_beacon_capabilities_registered(self):
        """Verify app capabilities include x-medkit-topic-beacon."""
        data = self.get_json('/apps/temp_sensor')
        capabilities = data.get('capabilities', [])
        cap_names = [c['name'] for c in capabilities]
        self.assertIn(
            'x-medkit-topic-beacon',
            cap_names,
            f'x-medkit-topic-beacon not in capabilities: {cap_names}',
        )
        # Verify href points to the correct path
        beacon_cap = next(
            c for c in capabilities if c['name'] == 'x-medkit-topic-beacon'
        )
        self.assertIn('/apps/temp_sensor/x-medkit-topic-beacon', beacon_cap['href'])

    def test_03_beacon_vendor_endpoint_not_found(self):
        """Verify 404 for entity without beacon data."""
        r = requests.get(
            f'{self.BASE_URL}/apps/nonexistent_entity/x-medkit-topic-beacon',
            timeout=5,
        )
        self.assertEqual(r.status_code, 404)

    def test_04_beacon_metadata_fields(self):
        """Verify all expected beacon metadata fields are present."""
        data = self.poll_endpoint_until(
            '/apps/temp_sensor/x-medkit-topic-beacon',
            lambda d: d if d.get('status') == 'active' else None,
            timeout=30,
        )
        expected_fields = [
            'entity_id', 'status', 'age_sec', 'stable_id',
            'display_name', 'transport_type', 'negotiated_format',
            'process_id', 'process_name', 'hostname',
            'component_id', 'function_ids', 'depends_on', 'metadata',
        ]
        for field in expected_fields:
            self.assertIn(
                field,
                data,
                f'Missing field {field!r} in beacon response',
            )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify gateway exits cleanly."""

    def test_exit_codes(self, proc_info):
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'Process {info.process_name} exited with {info.returncode}',
            )
