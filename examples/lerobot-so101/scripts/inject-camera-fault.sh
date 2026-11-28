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

# Freeze one simulated camera from a given step of every following attempt,
# until ./scripts/restore-normal.sh. The camera monitor reports it on /diagnostics
# and the diagnostic bridge raises CAMERA_<CAMERA>_FROZEN.
# Usage: ./scripts/inject-camera-fault.sh [wrist|overhead] [start_step]
set -eu
CAMERA="${1:-overhead}"
START="${2:-40}"
CONTAINER="${EXAMPLE_CONTAINER:-lerobot_so101_example}"
ROS="source /opt/example_ws/install/setup.bash"
case "$CAMERA" in wrist|overhead) ;; *) echo "Usage: $0 [wrist|overhead] [start_step]"; exit 1 ;; esac
docker exec "$CONTAINER" bash -c "$ROS && \
  ros2 param set /so101/so101_sim camera_fault_camera $CAMERA && \
  ros2 param set /so101/so101_sim camera_fault_start_step $START && \
  ros2 param set /so101/so101_sim camera_fault frozen"
echo "$CAMERA camera frozen from step $START of each attempt. Undo with ./scripts/restore-normal.sh"
