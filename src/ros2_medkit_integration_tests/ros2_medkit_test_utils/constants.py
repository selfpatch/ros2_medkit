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

API_BASE_PATH = '/api/v1'
DEFAULT_PORT = 8080
DEFAULT_BASE_URL = f'http://localhost:{DEFAULT_PORT}{API_BASE_PATH}'

# Gateway startup
GATEWAY_STARTUP_TIMEOUT = 30.0
GATEWAY_STARTUP_INTERVAL = 0.5

# Discovery
DISCOVERY_TIMEOUT = 60.0
DISCOVERY_INTERVAL = 1.0

# Operations
ACTION_TIMEOUT = 30.0

# Faults
FAULT_TIMEOUT = 30.0
ROSBAG_TIMEOUT = 30.0
SNAPSHOT_TIMEOUT = 30.0
