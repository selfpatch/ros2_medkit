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
HANDLERS_DIR="$(cd "${SCRIPT_DIR}/../src/http/handlers" && pwd)"

# Path reads (req.path), header reads (req.header), and fan-out (raw_for_framework
# for path/Authorization) are legitimate; only query-string reads are banned.
pattern='\.query_param\(|->query_param\(|get_param_value\('

hits="$(grep -rnE "${pattern}" "${HANDLERS_DIR}" --include='*.cpp' || true)"
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
