#!/bin/bash
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

set -e

# Source ROS 2 and the built workspace
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO}/setup.bash"
# shellcheck disable=SC1091
source "${COLCON_WS}/install/setup.bash"

# Default to FastDDS (can be overridden via RMW_IMPLEMENTATION env var)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# MEDKIT_JWT_SECRET, MEDKIT_CLIENTS and MEDKIT_AUTH_DISABLED are read by the
# gateway node itself, so the rule holds on every path out of this script.
# Nothing here interprets them; export carries them into what is exec'd.
export MEDKIT_JWT_SECRET MEDKIT_CLIENTS MEDKIT_AUTH_DISABLED

# Dispatch on the first argument:
#   - empty, or starts with "-": run the gateway node with the three config
#     layers first and the caller's arguments after them. rclcpp applies node
#     entries in order of first appearance, so a caller's `-p` wins over the files.
#   - a full command (`ros2 launch ...`, `bash`): exec it as-is.
if [ -z "$1" ] || [ "${1#-}" != "$1" ]; then
  exec ros2 run ros2_medkit_gateway gateway_node \
    --ros-args \
    --params-file /etc/ros2_medkit/base.yaml \
    --params-file /etc/ros2_medkit/container.yaml \
    --params-file /etc/ros2_medkit/params.yaml \
    "$@"
fi
exec "$@"
