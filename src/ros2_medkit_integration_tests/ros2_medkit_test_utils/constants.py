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

    For offset > 0 (multi-gateway tests needing a second DDS domain),
    derives a unique secondary domain from the primary via modular
    mapping into the 230-232 range (DDS max domain is 232). Each
    test's unique primary ensures a deterministic secondary. With 3
    buckets and currently 2 multi-gateway tests, collisions are avoided.
    """
    if offset == 0:
        return DEFAULT_DOMAIN_ID
    return 230 + ((DEFAULT_DOMAIN_ID - 140) % 3)


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
