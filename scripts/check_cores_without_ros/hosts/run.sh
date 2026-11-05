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

# Builds gateway_core in two hosts that bring their own httplib::httplib (see CMakeLists.txt).
# Run from the repository root with CXX set to the compiler under test.
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
vendored="$here/../../../src/ros2_medkit_gateway/src/vendored/cpp_httplib"
empty="$(mktemp -d)"
trap 'rm -rf "$empty"' EXIT

# Version 0.26 has no multipart API the handlers use: the target must be ignored. Its include
# directory holds no httplib.h, so using it fails the build. GLOBAL so that CMake 3.16 accepts it.
cmake -S "$here" -B /tmp/host_new -DHOST_HTTPLIB_VERSION=0.26.0 -DHOST_HTTPLIB_INCLUDE="$empty" -DHOST_HTTPLIB_GLOBAL=ON
cmake --build /tmp/host_new --parallel "$(nproc)"
/tmp/host_new/check_gateway_core_host

# An unversioned target from find_package() is not GLOBAL: CMake 3.16 and 3.17 cannot alias it.
cmake -S "$here" -B /tmp/host_unversioned -DHOST_HTTPLIB_INCLUDE="$vendored"
cmake --build /tmp/host_unversioned --parallel "$(nproc)"
/tmp/host_unversioned/check_gateway_core_host
echo "Host variants OK."
