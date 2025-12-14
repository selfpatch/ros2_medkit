# Copyright 2025 mfaferek93
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
project(ros2_medkit_fault_manager)

set(CMAKE_CXX_STANDARD 23)
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
find_package(ros2_medkit_msgs REQUIRED)

# Library target (shared between executable and tests)
add_library(fault_manager_lib STATIC
  src/fault_manager_node.cpp
  src/fault_storage.cpp
)

target_include_directories(fault_manager_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fault_manager_lib
  rclcpp
  ros2_medkit_msgs
)

if(ENABLE_COVERAGE)
  target_compile_options(fault_manager_lib PRIVATE --coverage -O0 -g)
  target_link_options(fault_manager_lib PRIVATE --coverage)
endif()

# Executable target
add_executable(fault_manager_node src/main.cpp)
target_link_libraries(fault_manager_node fault_manager_lib)

# Apply coverage flags to executable
if(ENABLE_COVERAGE)
  target_compile_options(fault_manager_node PRIVATE --coverage -O0 -g)
  target_link_options(fault_manager_node PRIVATE --coverage)
endif()

# Install targets
install(TARGETS fault_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

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
  ament_add_gtest(test_fault_manager test/test_fault_manager.cpp)
  target_link_libraries(test_fault_manager fault_manager_lib)
  ament_target_dependencies(test_fault_manager rclcpp ros2_medkit_msgs)

  # Apply coverage flags to test target
  if(ENABLE_COVERAGE)
    target_compile_options(test_fault_manager PRIVATE --coverage -O0 -g)
    target_link_options(test_fault_manager PRIVATE --coverage)
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
