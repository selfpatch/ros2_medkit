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

"""Shared constants for ros2_medkit integration tests."""

import os

API_BASE_PATH = '/api/v1'
DEFAULT_PORT = int(os.environ.get('GATEWAY_TEST_PORT', '8080'))
DEFAULT_DOMAIN_ID = int(os.environ.get('ROS_DOMAIN_ID', '0'))
DEFAULT_BASE_URL = f'http://localhost:{DEFAULT_PORT}{API_BASE_PATH}'


def get_test_port(offset=0):
    """Return the assigned test port plus an optional offset.

    Each integration test gets a unique ``GATEWAY_TEST_PORT`` from CMake.
    Tests that launch multiple gateway instances use *offset* to get
    additional non-colliding ports (e.g. ``get_test_port(1)``).
    """
    return DEFAULT_PORT + offset


def get_test_domain_id(offset=0):
    """Return a DDS domain ID for this test, optionally with an offset.

    Each integration test gets a unique ``ROS_DOMAIN_ID`` from CMake
    (stride of 1, range 140-229). For offset 0, returns the assigned
    domain ID directly.

    For offset in 1..3 (multi-gateway tests needing extra DDS domains),
    returns one of the three secondary domains 230, 231, 232 so each
    offset produces a distinct domain. These three secondaries sit
    outside the per-package allocation in
    ``ROS2MedkitTestDomain.cmake`` and are shared across every
    multi-gateway integration test; CTest serialises those tests via a
    ``RESOURCE_LOCK`` (see ``CMakeLists.txt``) so two of them never
    hold the same secondary domain simultaneously.

    DDS max domain ID is 232 (UDP port formula: 7400 + 250 * domain_id).
    Offsets above 3 would exceed that ceiling and are rejected up front.
    """
    if offset == 0:
        return DEFAULT_DOMAIN_ID
    if not 1 <= offset <= 3:
        raise ValueError(
            f'secondary DDS domain offset {offset} out of range 1..3 '
            '(only domains 230-232 are available; DDS max is 232). '
            'Add a new offset only after extending the allocation scheme.'
        )
    return 229 + offset


# Gateway startup
GATEWAY_STARTUP_TIMEOUT = 30.0
GATEWAY_STARTUP_INTERVAL = 0.5

# Discovery
DISCOVERY_TIMEOUT = 60.0
DISCOVERY_INTERVAL = 0.5  # seconds between discovery polls

# Operations
ACTION_TIMEOUT = 30.0

# Faults
FAULT_TIMEOUT = 30.0
ROSBAG_TIMEOUT = 30.0
SNAPSHOT_TIMEOUT = 30.0

# Shutdown
ALLOWED_EXIT_CODES = {0, -2, -15}  # OK, SIGINT, SIGTERM
