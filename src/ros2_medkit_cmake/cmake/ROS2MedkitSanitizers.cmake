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

include_guard(GLOBAL)

# Sanitizer support for ros2_medkit packages.
# Include this module early in CMakeLists.txt (alongside ROS2MedkitCcache).
#
# Activate from the command line:
#   colcon build --cmake-args -DSANITIZER=asan,ubsan
#   colcon build --cmake-args -DSANITIZER=tsan
#
# Supported sanitizers: asan, tsan, ubsan (comma-separated).
# ASan and TSan cannot be combined (incompatible runtimes).
# When SANITIZER is empty or unset, this module is a no-op.

set(SANITIZER "" CACHE STRING "Comma-separated list of sanitizers: asan, tsan, ubsan")

if(SANITIZER STREQUAL "")
  return()
endif()

# Parse comma-separated list
string(REPLACE "," ";" _SANITIZER_LIST "${SANITIZER}")

# Validate: asan + tsan is not allowed
list(FIND _SANITIZER_LIST "asan" _HAS_ASAN)
list(FIND _SANITIZER_LIST "tsan" _HAS_TSAN)
if(NOT _HAS_ASAN EQUAL -1 AND NOT _HAS_TSAN EQUAL -1)
  message(FATAL_ERROR "Cannot combine ASan and TSan - they use incompatible runtimes")
endif()

# Validate: only known sanitizers
foreach(_SAN IN LISTS _SANITIZER_LIST)
  if(NOT _SAN MATCHES "^(asan|tsan|ubsan)$")
    message(FATAL_ERROR "Unknown sanitizer '${_SAN}'. Supported: asan, tsan, ubsan")
  endif()
endforeach()

# Build the -fsanitize= flag value (asan -> address, tsan -> thread, ubsan -> undefined)
set(_FSANITIZE_FLAGS "")
foreach(_SAN IN LISTS _SANITIZER_LIST)
  if(_SAN STREQUAL "asan")
    list(APPEND _FSANITIZE_FLAGS "address")
  elseif(_SAN STREQUAL "tsan")
    list(APPEND _FSANITIZE_FLAGS "thread")
  elseif(_SAN STREQUAL "ubsan")
    list(APPEND _FSANITIZE_FLAGS "undefined")
  endif()
endforeach()
list(JOIN _FSANITIZE_FLAGS "," _FSANITIZE_VALUE)

# -O1: sanitizers produce fewer false positives and run faster than -O0.
# This intentionally overrides Debug's -O0 for sanitizer builds.
add_compile_options(-fsanitize=${_FSANITIZE_VALUE} -fno-omit-frame-pointer -O1)
add_link_options(-fsanitize=${_FSANITIZE_VALUE})

message(STATUS "Sanitizers enabled: ${SANITIZER} (-fsanitize=${_FSANITIZE_VALUE})")
