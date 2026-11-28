#!/bin/bash
# Copyright 2026 ahmedsleem109
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

# Start a campaign of N attempts.  Usage: ./scripts/campaign.sh [N]   (default 10)
set -eu
NUM="${1:-10}"
CONTAINER="${EXAMPLE_CONTAINER:-lerobot_so101_example}"
ROS="source /opt/example_ws/install/setup.bash"
docker exec "$CONTAINER" bash -c "$ROS && \
  ros2 param set /episode_runner num_episodes $NUM >/dev/null && \
  ros2 service call /episode_runner/start_campaign std_srvs/srv/Trigger" | grep -E "success|message"
echo "Attempts are appended to results/attempts.jsonl"
