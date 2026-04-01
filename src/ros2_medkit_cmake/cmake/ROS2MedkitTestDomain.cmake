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

# ROS_DOMAIN_ID allocation for tests.
#
# DDS domain IDs are limited to 0-232 (UDP port formula: 7400 + 250 * domain_id).
# colcon test runs CTest per package in parallel, so different packages MUST use
# non-overlapping domain ID ranges to prevent DDS cross-contamination.
#
# Allocated ranges:
#   ros2_medkit_fault_manager:       10 -  29 (20 slots)
#   ros2_medkit_gateway:             30 -  89 (60 slots)
#   ros2_medkit_diagnostic_bridge:   90 -  99 (10 slots)
#   ros2_medkit_param_beacon:       100 - 109 (10 slots)
#   ros2_medkit_topic_beacon:       110 - 119 (10 slots)
#   ros2_medkit_graph_provider:     120 - 129 (10 slots)
#   ros2_medkit_linux_introspection: 130 - 139 (10 slots)
#   ros2_medkit_integration_tests:  140 - 229 (90 slots)
#   multi-domain tests (secondary):  230 - 232 (3 slots, via get_test_domain_id)
#
# To add a new package: pick the next free range and update this comment.

# Initialize the domain ID counter for a package.
# Must be called once per CMakeLists.txt before using medkit_set_test_domain.
#
# Usage:
#   medkit_init_test_domains(START 30 END 89)
#
macro(medkit_init_test_domains)
  cmake_parse_arguments(_MTID "" "START;END" "" ${ARGN})
  if(NOT DEFINED _MTID_START OR NOT DEFINED _MTID_END)
    message(FATAL_ERROR "medkit_init_test_domains requires START and END arguments")
  endif()
  set(_MEDKIT_DOMAIN_COUNTER ${_MTID_START})
  set(_MEDKIT_DOMAIN_END ${_MTID_END})
  set(_MEDKIT_DOMAIN_START ${_MTID_START})
endmacro()

# Assign the next available ROS_DOMAIN_ID to a test target.
# The test must already be defined via ament_add_gtest or add_launch_test.
# Uses macro (not function) so the counter persists between calls.
#
# Usage:
#   ament_add_gtest(test_foo test_foo.cpp)
#   medkit_set_test_domain(test_foo)
#
macro(medkit_set_test_domain TEST_NAME)
  if(NOT DEFINED _MEDKIT_DOMAIN_COUNTER)
    message(FATAL_ERROR
      "medkit_set_test_domain called before medkit_init_test_domains")
  endif()
  if(${_MEDKIT_DOMAIN_COUNTER} GREATER ${_MEDKIT_DOMAIN_END})
    message(FATAL_ERROR
      "Domain ID range exhausted (${_MEDKIT_DOMAIN_START}-${_MEDKIT_DOMAIN_END}). "
      "Increase END or allocate a new range in ROS2MedkitTestDomain.cmake.")
  endif()
  set_tests_properties(${TEST_NAME} PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=${_MEDKIT_DOMAIN_COUNTER}"
  )
  math(EXPR _MEDKIT_DOMAIN_COUNTER "${_MEDKIT_DOMAIN_COUNTER} + 1")
endmacro()
