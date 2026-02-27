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

# Shared linting configuration for ros2_medkit packages.
# Include this in CMakeLists.txt BEFORE the if(BUILD_TESTING) block
# (alongside include(ROS2MedkitCcache) - CMAKE_MODULE_PATH is already set).
#
# Provides:
#   option ENABLE_CLANG_TIDY (default OFF)
#   function ros2_medkit_clang_tidy([HEADER_FILTER <regex>] [TIMEOUT <seconds>])

option(ENABLE_CLANG_TIDY "Register clang-tidy as a CTest target" OFF)

# Capture at include-time: inside a function CMAKE_CURRENT_LIST_DIR resolves to the caller.
set(_ROS2_MEDKIT_CLANG_TIDY_CONFIG "${CMAKE_CURRENT_LIST_DIR}/../.clang-tidy")

function(ros2_medkit_clang_tidy)
  if(NOT ENABLE_CLANG_TIDY)
    return()
  endif()

  cmake_parse_arguments(ARG "" "HEADER_FILTER;TIMEOUT" "" ${ARGN})

  find_package(ament_cmake_clang_tidy REQUIRED)

  set(_args "${CMAKE_CURRENT_BINARY_DIR}" CONFIG_FILE "${_ROS2_MEDKIT_CLANG_TIDY_CONFIG}")

  if(ARG_HEADER_FILTER)
    list(APPEND _args HEADER_FILTER "${ARG_HEADER_FILTER}")
  endif()

  if(ARG_TIMEOUT)
    list(APPEND _args TIMEOUT "${ARG_TIMEOUT}")
  endif()

  ament_clang_tidy(${_args})
endfunction()
