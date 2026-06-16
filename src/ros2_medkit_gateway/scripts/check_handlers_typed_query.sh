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

# Asserts that HTTP handlers read query parameters ONLY through the typed
# TypedRequest::query<T>() contract, never via raw req.query_param() or
# get_param_value().
#
# Why: the typed path derives the OpenAPI `parameters` from the same query-DTO
# descriptor (dto::QueryParamWriter<T>) that the handler parses. A raw read is
# invisible to the spec, so the parameter never reaches the generated clients -
# exactly the regression that left every query parameter out of the published
# 0.5.0 clients. Forcing the typed path makes that drift impossible: a handler
# can only read fields that exist on its query DTO, and those fields are what the
# route declares.
#
# To add a query parameter: define a query DTO (struct + dto_fields), declare it
# on the route with .query<T>(), and read it with req.query<T>(). See
# include/ros2_medkit_gateway/dto/query.hpp.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Handler layers to scan. Both the HTTP and the core handler layers are covered,
# on the source side (src/.../handlers, the .cpp bodies) AND the include side
# (include/.../handlers, where inline handler methods live in .hpp) - a raw read
# hidden in a header or a second handler layer would otherwise bypass the gate.
# The plugin HTTP shim (src/plugins/plugin_http_types.cpp) is intentionally NOT
# scanned: it is the sanctioned raw-httplib boundary the typed wrapper is built on.
scan_dirs=()
for rel in \
  "../src/http/handlers" \
  "../src/core/http/handlers" \
  "../include/ros2_medkit_gateway/http/handlers" \
  "../include/ros2_medkit_gateway/core/http/handlers"; do
  abs="${SCRIPT_DIR}/${rel}"
  if [[ -d "${abs}" ]]; then
    scan_dirs+=("$(cd "${abs}" && pwd)")
  fi
done

if [[ ${#scan_dirs[@]} -eq 0 ]]; then
  echo "ERROR: no handler directories found to scan under ${SCRIPT_DIR}/../src or ../include."
  exit 1
fi

# Path reads (req.path), header reads (req.header), and fan-out (raw_for_framework
# for path/Authorization) are legitimate; only query-string reads are banned.
# Match the accessor name directly (not "req." + name) so whitespace tricks like
# `req . query_param (` cannot slip past, and ban has_param() too - it is the
# presence half of a raw query read.
pattern='(query_param|get_param_value|has_param)[[:space:]]*\('

# Run the scan with grep's own status preserved: 0 = matches, 1 = none, >=2 = a
# real error (unreadable file, bad path). A blanket `|| true` would mask exit 2
# as "no matches", so an unscannable handler file would pass the gate silently;
# treat >=2 as a hard failure of the gate itself instead.
set +e
raw_hits="$(grep -rnE "${pattern}" "${scan_dirs[@]}" --include='*.cpp' --include='*.hpp')"
grep_rc=$?
set -e
if [[ ${grep_rc} -ge 2 ]]; then
  echo "ERROR: grep failed (exit ${grep_rc}) while scanning handler dirs; cannot certify the gate."
  exit 1
fi

# Heuristic: drop matches that sit in a whole-line // comment. Inline trailing
# comments and block/string occurrences are not stripped (a grep-level lint
# cannot parse C++), but those are rare and a stray mention in a comment is
# better flagged than a real read silently missed.
hits="$(printf '%s' "${raw_hits}" | grep -vE '^[^:]*:[0-9]+:[[:space:]]*//' || true)"
if [[ -n "${hits}" ]]; then
  echo "ERROR: handlers must read query parameters via TypedRequest::query<T>(), not raw query reads."
  echo "Offending lines:"
  echo "${hits}"
  echo
  echo "Fix: define a query DTO (struct + dto_fields), declare it on the route with"
  echo ".query<T>(), and read it via req.query<T>(). See dto/query.hpp."
  exit 1
fi

echo "OK: handlers read query parameters only via the typed query<T>() contract."
