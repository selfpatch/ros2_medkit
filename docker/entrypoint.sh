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
# Setting MEDKIT_JWT_SECRET is the statement "this container runs closed". It
# turns authentication on, sets require_auth_for to "all" and hands the gateway
# that secret, whatever any params file says: these are passed after the file
# and a later -p wins. That is the point. Reading the file and deferring to it
# let a file carrying a secret but auth.enabled false run open with the variable
# set, which is the one outcome this must not produce. MEDKIT_CLIENTS carries
# the client credentials, without which nothing can obtain a token.
#
# MEDKIT_AUTH_DISABLED=1 forces authentication off and wins over both.

AUTH_ARGS=()
if [ "${MEDKIT_AUTH_DISABLED:-0}" = "1" ]; then
  AUTH_ARGS+=(-p auth.enabled:=false)
  echo "ros2_medkit: MEDKIT_AUTH_DISABLED=1 - starting WITHOUT authentication." >&2
  echo "             Every route is readable by anyone who can reach this port." >&2
elif [ -n "${MEDKIT_JWT_SECRET:-}" ]; then
  AUTH_ARGS+=(-p auth.enabled:=true)
  AUTH_ARGS+=(-p auth.require_auth_for:=all)
  AUTH_ARGS+=(-p "auth.jwt_secret:=${MEDKIT_JWT_SECRET}")
  if [ -n "${MEDKIT_CLIENTS:-}" ]; then
    AUTH_ARGS+=(-p "auth.clients:=[${MEDKIT_CLIENTS}]")
  else
    echo "ros2_medkit: MEDKIT_JWT_SECRET is set but MEDKIT_CLIENTS is not, so no" >&2
    echo "             client can obtain a token. Pass" >&2
    echo "             MEDKIT_CLIENTS=<id>:<secret>:admin as well." >&2
  fi
  echo "ros2_medkit: MEDKIT_JWT_SECRET is set - authentication is ON and every" >&2
  echo "             route needs a credential." >&2
fi
# Exported so the other dispatch branch works too: `docker run <img> ros2 launch
# ... bringup.launch.py` execs a command instead of the node, so it never sees
# AUTH_ARGS. gateway.launch.py reads these variables and applies the same rule.
export MEDKIT_JWT_SECRET MEDKIT_CLIENTS MEDKIT_AUTH_DISABLED

# Dispatch on the first argument:
#   - empty, or starts with "-" (the default CMD "--ros-args --params-file ..."
#     or an override like --ros-args -p server.port:=9090): run the gateway node
#     directly, so `docker run <img>` and arg-only overrides keep working.
#   - a full command (e.g. `ros2 launch ros2_medkit_gateway bringup.launch.py`
#     or `bash`): exec it as-is, so the image can launch the whole bringup stack.
if [ -z "$1" ] || [ "${1#-}" != "$1" ]; then
  exec ros2 run ros2_medkit_gateway gateway_node "$@" "${AUTH_ARGS[@]}"
fi
exec "$@"
