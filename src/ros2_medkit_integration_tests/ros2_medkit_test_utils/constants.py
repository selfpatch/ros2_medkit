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


def _secondary_domains():
    """Return the extra DDS domains this test was given, beyond its own.

    The domain wrapper allocates them when the test starts and publishes them as
    ``MEDKIT_SECONDARY_DOMAINS``; it holds every one of them, through an open
    socket, for as long as the test runs. Absent (a test run by hand outside
    CTest), there is nothing to fall back on that would be safe, so say so.
    """
    raw = os.environ.get('MEDKIT_SECONDARY_DOMAINS', '')
    return [int(part) for part in raw.split(',') if part.strip()]


def get_test_domain_id(offset=0):
    """Return a DDS domain ID for this test, optionally with an offset.

    Every test gets a ``ROS_DOMAIN_ID`` of its own when it starts, from the
    wrapper described in ``ROS2MedkitTestDomain.cmake``. For offset 0, returns
    that domain.

    Offsets above 0 are for multi-gateway tests that run a second or third
    gateway at the same time. Those domains are allocated to this test alone -
    ``DOMAINS <n>`` on its ``medkit_add_launch_test`` call is what asks for them -
    so no other test can be on them while this one runs.
    """
    if offset == 0:
        return DEFAULT_DOMAIN_ID
    secondary = _secondary_domains()
    if not secondary:
        raise RuntimeError(
            'MEDKIT_SECONDARY_DOMAINS is not set, so no secondary DDS domain can be '
            'handed out. The domain wrapper sets it for a test registered with '
            'DOMAINS greater than 1. To run this test by hand, run it through '
            'medkit_run_with_domain.py --domains <n>, or set ROS_DOMAIN_ID and '
            'MEDKIT_SECONDARY_DOMAINS yourself to domains nobody else is using.'
        )
    if not 1 <= offset <= len(secondary):
        raise ValueError(
            f'secondary DDS domain offset {offset} out of range 1..{len(secondary)} '
            f'(this test was given {secondary}). Raise DOMAINS on its '
            'medkit_add_launch_test call in CMakeLists.txt before adding a new offset.'
        )
    return secondary[offset - 1]


# Gateway startup
GATEWAY_STARTUP_TIMEOUT = 30.0
GATEWAY_STARTUP_INTERVAL = 0.5

# Discovery
DISCOVERY_TIMEOUT = 60.0
DISCOVERY_INTERVAL = 0.5  # seconds between discovery polls

# Parameter service readiness
# A node's ROS 2 parameter service can lag its graph discovery, and the
# configurations endpoint returns 503 until it responds. This lag is small
# without instrumentation but grows under the TSan job, where the service was
# still 503 more than 15s after discovery finished. Generous on purpose: a
# larger timeout costs nothing on the passing path (the poll returns the moment
# the endpoint answers 200) and only bounds how long a genuinely dead service
# waits before failing, well inside the sanitizer jobs' 360s ctest budget.
PARAM_SERVICE_TIMEOUT = 90.0

# Operations
ACTION_TIMEOUT = 30.0

# Faults
FAULT_TIMEOUT = 30.0
ROSBAG_TIMEOUT = 30.0
SNAPSHOT_TIMEOUT = 30.0

# Shutdown
ALLOWED_EXIT_CODES = {0, -2, -15}  # OK, SIGINT, SIGTERM
