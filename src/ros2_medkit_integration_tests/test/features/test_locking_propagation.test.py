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

"""Integration test for lock parent propagation.

Verifies that locking a component blocks mutating operations on its
child apps (SOVD locking rule: locking a parent entity locks children
with the same scope).

Uses manifest_only discovery mode for stable component/app IDs.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
import launch_testing
import requests

from ros2_medkit_test_utils.constants import ALLOWED_EXIT_CODES
from ros2_medkit_test_utils.gateway_test_case import GatewayTestCase
from ros2_medkit_test_utils.launch_helpers import create_demo_nodes, create_gateway_node


def generate_test_description():
    """Launch gateway in manifest_only mode with locking enabled."""
    pkg_share = get_package_share_directory('ros2_medkit_gateway')
    manifest_path = os.path.join(
        pkg_share, 'config', 'examples', 'demo_nodes_manifest.yaml'
    )

    gateway = create_gateway_node(extra_params={
        'discovery.mode': 'manifest_only',
        'discovery.manifest_path': manifest_path,
        'discovery.manifest_strict_validation': False,
        'locking.enabled': True,
        'locking.default_max_expiration': 3600,
    })

    demo_nodes = create_demo_nodes(
        ['temp_sensor'],
        lidar_faulty=False,
    )

    delayed = TimerAction(period=2.0, actions=demo_nodes)

    return (
        LaunchDescription([
            gateway,
            delayed,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'gateway_node': gateway},
    )


class TestLockingPropagation(GatewayTestCase):
    """Lock parent propagation integration tests."""

    MIN_EXPECTED_APPS = 1
    REQUIRED_APPS = {'engine-temp-sensor'}

    # @verifies REQ_INTEROP_100
    def test_component_lock_blocks_child_app(self):
        """Lock on component blocks mutating operations on child app."""
        # Lock temp-sensor-hw component (parent of engine-temp-sensor app)
        resp = requests.post(
            f'{self.BASE_URL}/components/temp-sensor-hw/locks',
            json={'lock_expiration': 300},
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']

        # Client B tries to PUT data on engine-temp-sensor (child app) - 409
        resp = requests.put(
            f'{self.BASE_URL}/apps/engine-temp-sensor/data/engine_temperature',
            json={'type': 'std_msgs/msg/Float64', 'data': {'data': 99.0}},
            headers={'X-Client-Id': 'client_b'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 409, resp.text)

        # Clean up
        requests.delete(
            f'{self.BASE_URL}/components/temp-sensor-hw/locks/{lock_id}',
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )

    # @verifies REQ_INTEROP_100
    def test_component_lock_allows_owner_on_child(self):
        """Lock owner can still access child app."""
        resp = requests.post(
            f'{self.BASE_URL}/components/temp-sensor-hw/locks',
            json={'lock_expiration': 300},
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )
        self.assertEqual(resp.status_code, 201, resp.text)
        lock_id = resp.json()['id']

        # Owner (client_a) should NOT get 409 on child app
        resp = requests.put(
            f'{self.BASE_URL}/apps/engine-temp-sensor/data/engine_temperature',
            json={'type': 'std_msgs/msg/Float64', 'data': {'data': 99.0}},
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )
        self.assertNotEqual(resp.status_code, 409, resp.text)

        # Clean up
        requests.delete(
            f'{self.BASE_URL}/components/temp-sensor-hw/locks/{lock_id}',
            headers={'X-Client-Id': 'client_a'},
            timeout=10,
        )


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        """Check all processes exited cleanly."""
        for info in proc_info:
            self.assertIn(
                info.returncode,
                ALLOWED_EXIT_CODES,
                f'{info.process_name} exited with code {info.returncode}',
            )
