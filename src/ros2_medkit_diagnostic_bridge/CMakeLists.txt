# Copyright 2026 mfaferek93
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

cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_diagnostic_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

# Code coverage option
option(ENABLE_COVERAGE "Enable code coverage reporting" OFF)
if(ENABLE_COVERAGE)
  message(STATUS "Code coverage enabled")
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_fault_reporter REQUIRED)

# Library target (for testing)
add_library(diagnostic_bridge_lib SHARED
  src/diagnostic_bridge_node.cpp
)

target_include_directories(diagnostic_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(diagnostic_bridge_lib
  rclcpp
  diagnostic_msgs
  ros2_medkit_msgs
  ros2_medkit_fault_reporter
)

# Executable
add_executable(diagnostic_bridge_node src/main.cpp)
target_link_libraries(diagnostic_bridge_node diagnostic_bridge_lib)
ament_target_dependencies(diagnostic_bridge_node rclcpp)

# Install executable
install(TARGETS diagnostic_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install library
install(TARGETS diagnostic_bridge_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch and config
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Export for downstream packages
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp diagnostic_msgs ros2_medkit_msgs ros2_medkit_fault_reporter)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  # Unit tests
  ament_add_gtest(test_diagnostic_bridge test/test_diagnostic_bridge.cpp)
  target_link_libraries(test_diagnostic_bridge diagnostic_bridge_lib)
  ament_target_dependencies(test_diagnostic_bridge rclcpp diagnostic_msgs ros2_medkit_msgs)

  if(ENABLE_COVERAGE)
    target_compile_options(test_diagnostic_bridge PRIVATE --coverage -O0 -g)
    target_link_options(test_diagnostic_bridge PRIVATE --coverage)
  endif()

  # Integration tests
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  add_launch_test(
    test/test_integration.test.py
    TARGET test_integration
    TIMEOUT 60
  )
endif()

ament_package()
