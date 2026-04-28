#!/usr/bin/env bash
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

# Verifies that the neutral core layer has no ROS includes.
# Run from the package root directory.

set -euo pipefail

CORE_DIRS=(
  "include/ros2_medkit_gateway/core"
  "src/core"
)

FORBIDDEN_PATTERN='#include[[:space:]]*[<"](rclcpp|rcl_interfaces|rosidl|action_msgs|std_msgs|sensor_msgs|geometry_msgs|builtin_interfaces|ros2_medkit_msgs|rcutils|rmw)[/_]'

violations=0
for dir in "${CORE_DIRS[@]}"; do
  if [ ! -d "$dir" ]; then
    continue
  fi
  while IFS= read -r match; do
    echo "FORBIDDEN ROS INCLUDE: $match"
    violations=$((violations + 1))
  done < <(grep -rEn "$FORBIDDEN_PATTERN" "$dir" 2>/dev/null || true)
done

if [ "$violations" -gt 0 ]; then
  echo ""
  echo "Core purity check failed: $violations ROS-include violation(s) detected."
  echo "The core/ layer is the middleware-neutral business-logic layer."
  echo "ROS-specific code belongs under src/ (or a future ros2/ subdirectory)."
  exit 1
fi

echo "Core purity check OK."
exit 0
