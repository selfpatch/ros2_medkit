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

# Closing the image is opt-in, through the environment.
#
# The packaged params file is config/gateway_params.yaml, the same one a source
# install gets, and it leaves authentication off. So `docker run <image>` is the
# gateway a reader of the quickstart expects, and the web UI - which sends no
# Authorization header - talks to it.
#
# MEDKIT_JWT_SECRET, MEDKIT_CLIENTS and MEDKIT_AUTH_DISABLED are passed through
# and nothing here interprets them. The gateway node reads them itself when it
# reads its parameters, so the rule holds on every path out of this script -
# the `ros2 run` below, `docker run <img> ros2 launch ... bringup.launch.py`,
# and `docker run <img> bash` followed by anything. A copy of the rule here
# would cover only the first, and the exec paths would quietly run on different
# terms.
#
# Exported, which is the step that matters: a variable passed with `docker run
# -e` is already in this shell's environment, and `export` is what carries it
# into the environment of what this script execs.
export MEDKIT_JWT_SECRET MEDKIT_CLIENTS MEDKIT_AUTH_DISABLED

# Dispatch on the first argument:
#   - empty, or starts with "-" (an override like --ros-args -p
#     server.port:=9090): run the gateway node with the image's three config
#     layers first and the caller's arguments after them.
#   - a full command (e.g. `ros2 launch ros2_medkit_gateway bringup.launch.py`
#     or `bash`): exec it as-is, so the image can launch the whole bringup stack.
#
# The three --params-file arguments belong here and not in the Dockerfile CMD,
# because a caller passing arguments REPLACES the CMD. As part of the CMD they
# would be dropped by any `docker run <img> --ros-args ...`, leaving a gateway
# on the packaged loopback bind and reachable from nothing outside the
# container. Supplied here, an override changes the one key it names.
#
# The caller's arguments follow, and their own --ros-args opens a second group.
# rclcpp applies the merged node entries in order of first appearance, and the
# caller's `-p` is the first one on this command line, so its entry comes after
# these files and wins over them - which is what lets an override change the
# key it names while the layers supply everything else.
if [ -z "$1" ] || [ "${1#-}" != "$1" ]; then
  exec ros2 run ros2_medkit_gateway gateway_node \
    --ros-args \
    --params-file /etc/ros2_medkit/base.yaml \
    --params-file /etc/ros2_medkit/container.yaml \
    --params-file /etc/ros2_medkit/params.yaml \
    "$@"
fi
exec "$@"
