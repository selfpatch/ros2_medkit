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

# Asserts that plugin-facing public gateway headers do NOT transitively depend
# on <httplib.h>. Plugins consume these headers through the provider/DTO
# interface; if any pulls httplib, a plugin built against the INSTALLED gateway
# (ROS build-farm / Docker topology, where the gateway's vendored httplib is not
# on the include path) fails to compile. Preprocessor-only scan (g++ -M -MG):
# fast and distro-independent.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INCLUDE_DIR="$(cd "${SCRIPT_DIR}/../include" && pwd)"
CXX="${CXX:-g++}"

# Plugin-facing surface: ALL provider interfaces (globbed, so new providers are
# covered automatically) + the explicit plugin base headers a GatewayPlugin
# subclass includes. Deliberately excludes gateway-internal HTTP-plumbing
# headers (rest_server.hpp, handler_context.hpp, plugin_manager.hpp, ...): no
# plugin includes those, and the build farm propagates httplib to the gateway's
# own consumers via its declared <depend>libcpp-httplib-dev</depend>.
mapfile -t HEADERS < <(
  cd "${INCLUDE_DIR}" && \
  ls ros2_medkit_gateway/core/providers/*.hpp 2>/dev/null
)
HEADERS+=(
  "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
  "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
  "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
  "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"
)

status=0
for header in "${HEADERS[@]}"; do
  if [[ ! -f "${INCLUDE_DIR}/${header}" ]]; then
    echo "SKIP (not found): ${header}"
    continue
  fi
  # -M: full dependency list incl. system headers; -MG: also emit unresolved
  # angle-bracket includes as bare tokens (covers the httplib-withheld case).
  # -MG tolerates ALL missing includes, so a non-zero exit means a genuine
  # error (e.g. a syntax error in the header). Fail loud rather than silently
  # reporting "ok" on empty/partial output.
  if ! deps="$("${CXX}" -std=c++17 -I "${INCLUDE_DIR}" -M -MG -x c++ \
                 <(printf '#include "%s"\n' "${header}") 2>&1)"; then
    echo "FAIL: ${header} could not be preprocessed:"
    echo "${deps}"
    status=1
    continue
  fi
  # Match both the resolved path form (".../httplib.h") and the bare -MG token.
  if grep -qE '(^|[[:space:]/])httplib\.h([[:space:]]|$)' <<<"${deps}"; then
    echo "FAIL: ${header} transitively depends on <httplib.h>"
    status=1
  else
    echo "ok:   ${header}"
  fi
done

if [[ "${status}" -ne 0 ]]; then
  echo ""
  echo "Plugin-facing public headers must not depend on httplib. See"
  echo "http/handler_result.hpp for the httplib-free handler vocabulary."
fi
exit "${status}"
