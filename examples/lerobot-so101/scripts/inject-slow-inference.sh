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

# Slow the policy down (default +500 ms), until ./scripts/restore-normal.sh. With
# lerobot-medkit the delay is added to each model inference and the plugin raises
# POLICY_INFERENCE_SLOW; without it, to every step, which the policy node reports
# on /diagnostics and the diagnostic bridge raises as POLICY_INFERENCE_SLOW. The
# simulation runs in lockstep, so the robot waits for the policy: attempts get
# slower, their outcome does not change.
# Usage: ./scripts/inject-slow-inference.sh [delay_ms]
set -eu
DELAY="${1:-500}"
CONTAINER="${EXAMPLE_CONTAINER:-lerobot_so101_example}"
ROS="source /opt/example_ws/install/setup.bash"
docker exec "$CONTAINER" bash -c "$ROS && ros2 param set /lerobot_policy inference_delay_ms $DELAY"
echo "Each policy step now takes $DELAY ms more. Undo with ./scripts/restore-normal.sh"
