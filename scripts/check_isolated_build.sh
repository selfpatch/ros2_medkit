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

# Reproduce the ROS build-farm topology locally on Jazzy: build + install the
# gateway into a throwaway prefix, then build each plugin LIBRARY against ONLY
# the installed gateway with httplib genuinely WITHHELD (a poisoned shadow
# <httplib.h> injected via -isystem ahead of /usr/include). A leaked <httplib.h>
# in a plugin-facing public header (masked in normal CI by the colcon overlay +
# system libcpp-httplib-dev) trips the poison's #error here, exactly like the
# build farm / Docker Publish. The leak is distro-independent, so Jazzy alone
# catches this class. Run before pushing changes to gateway public headers or
# plugins.
set -euo pipefail

DISTRO="${ROS_DISTRO:-jazzy}"
# The ament setup scripts reference optional env vars, so relax nounset while
# sourcing (otherwise `set -u` aborts on AMENT_TRACE_SETUP_FILES et al.).
set +u
# shellcheck disable=SC1090
source "/opt/ros/${DISTRO}/setup.bash"
set -u

# Cap build parallelism. The gateway has many heavy TUs (PCH + httplib +
# nlohmann + jwt); compiling all of them at once on a high-core / low-memory
# host (e.g. a CI container) can exhaust RAM and get OOM-killed. Build one
# package at a time (--parallel-workers 1) and cap per-package compile jobs.
# Override MEDKIT_ISO_JOBS for faster builds on high-memory hosts.
NPROC="$(nproc)"
MEDKIT_ISO_JOBS="${MEDKIT_ISO_JOBS:-$(( NPROC < 6 ? NPROC : 6 ))}"
export MAKEFLAGS="-j${MEDKIT_ISO_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="${MEDKIT_ISO_JOBS}"

ISO="$(mktemp -d /tmp/medkit_iso.XXXXXX)"
POISON="$(mktemp -d /tmp/medkit_poison.XXXXXX)"
trap 'rm -rf "${ISO}" "${POISON}"' EXIT

# A shadow httplib.h that fails loudly if any plugin TU includes <httplib.h>.
cat > "${POISON}/httplib.h" <<'POISONEOF'
#error "httplib.h leaked into a plugin build: a plugin-facing gateway public header transitively includes <httplib.h>. See ros2_medkit_gateway/http/handler_result.hpp."
POISONEOF

echo "==> Stage 1: build + install gateway into ${ISO} (httplib available here)"
colcon build --parallel-workers 1 --build-base "${ISO}/build" --install-base "${ISO}/install" \
  --packages-up-to ros2_medkit_gateway \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

# Stage 2 sources ONLY the isolated gateway install (NOT the dev overlay
# install/, which would re-expose the gateway build-tree httplib include path).
# Each plugin overlays into the SAME isolated prefix so it resolves the gateway
# and ros2_medkit_cmake from the install, never from the source overlay.
set +u
# shellcheck disable=SC1091
source "${ISO}/install/setup.bash"
set -u

for plugin in ros2_medkit_graph_provider ros2_medkit_sovd_service_interface ros2_medkit_opcua; do
  echo "==> Stage 2: isolated lib build ${plugin} (httplib poisoned)"
  colcon build --parallel-workers 1 --build-base "${ISO}/build" --install-base "${ISO}/install" \
    --packages-select "${plugin}" \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
                 "-DCMAKE_CXX_FLAGS=-isystem ${POISON}"
done

echo "OK: all plugin libraries build against the installed gateway with httplib withheld."
