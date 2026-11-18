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

# Dispatch on the first argument:
#   - empty, or starts with "-" (the default CMD "--ros-args --params-file ..."
#     or an override like --ros-args -p server.port:=9090): run the gateway node
#     directly, so `docker run <img>` and arg-only overrides keep working.
#   - a full command (e.g. `ros2 launch ros2_medkit_gateway bringup.launch.py`
#     or `bash`): exec it as-is, so the image can launch the whole bringup stack.
if [ -z "$1" ] || [ "${1#-}" != "$1" ]; then
  exec ros2 run ros2_medkit_gateway gateway_node "$@"
fi
exec "$@"
