# Copyright 2026 ahmedsleem109
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
project(ros2_medkit_policy_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitCoverage)
include(ROS2MedkitLinting)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)

include(ROS2MedkitCompat)

find_package(rclcpp REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_fault_reporter REQUIRED)
find_package(ros2_medkit_policy_msgs REQUIRED)

add_library(policy_bridge_lib SHARED
  src/policy_bridge_node.cpp
)

target_include_directories(policy_bridge_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

medkit_target_dependencies(policy_bridge_lib
  rclcpp
  ros2_medkit_msgs
  ros2_medkit_fault_reporter
  ros2_medkit_policy_msgs
)

add_executable(policy_bridge_node src/main.cpp)
target_link_libraries(policy_bridge_node policy_bridge_lib)
medkit_target_dependencies(policy_bridge_node rclcpp)

install(TARGETS policy_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS policy_bridge_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp ros2_medkit_msgs ros2_medkit_fault_reporter ros2_medkit_policy_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  medkit_lint_config(.clang-format ament_cmake_clang_format_CONFIG_FILE)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cpplint ament_cmake_clang_tidy)
  ament_lint_auto_find_test_dependencies()

  ros2_medkit_clang_tidy()

  include(ROS2MedkitTestDomain)

  # The core has no ROS dependency: its tests need only the headers and create
  # no ROS entities.
  ament_add_gtest(test_policy_monitor test/test_policy_monitor.cpp)
  target_include_directories(test_policy_monitor PRIVATE include)
  medkit_test_needs_no_domain(test_policy_monitor)
  ament_add_gtest(test_fault_sync test/test_fault_sync.cpp)
  target_include_directories(test_fault_sync PRIVATE include)
  medkit_test_needs_no_domain(test_fault_sync)

  # The node against a ReportFault server in the same process. Each wait is
  # at most 3 s (x MEDKIT_TEST_TIME_SCALE). Worst case for the whole binary:
  # 7 set-ups x 3 s + 3.8 + 12.5 + 9.2 + 6.2 s = 53 s at scale 1, 158 s at
  # scale 3 (the sanitizer jobs triple TIMEOUT to 270 s).
  medkit_add_gtest(test_policy_bridge_node test/test_policy_bridge_node.cpp TIMEOUT 90)
  target_link_libraries(test_policy_bridge_node policy_bridge_lib)
  medkit_target_dependencies(test_policy_bridge_node rclcpp ros2_medkit_msgs ros2_medkit_policy_msgs)

  # Launch tests through the real fault manager. Each file states its
  # worst-case sum of waits next to its TIMEOUT.
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )
  medkit_add_launch_test(test_integration test/test_integration.test.py TIMEOUT 240)
  medkit_add_launch_test(test_fault_manager_lifecycle test/test_fault_manager_lifecycle.test.py TIMEOUT 200)
  # 40 runs of up to 1 s + 80 runs of up to 0.25 s, each stop within a few
  # seconds; the sibling bridge has the same test with the same limit.
  medkit_add_launch_test(test_sigint_exit test/test_sigint_exit.test.py TIMEOUT 120)

  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
