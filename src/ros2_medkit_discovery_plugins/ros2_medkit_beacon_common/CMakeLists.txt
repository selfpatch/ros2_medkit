# Copyright 2026 selfpatch GmbH
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
project(ros2_medkit_beacon_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(beacon_common_lib STATIC
  src/beacon_validator.cpp
  src/beacon_hint_store.cpp
  src/beacon_entity_mapper.cpp
)

set_target_properties(beacon_common_lib PROPERTIES
  POSITION_INDEPENDENT_CODE ON  # Required: linked into MODULE .so targets
)

target_include_directories(beacon_common_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(beacon_common_lib
  ros2_medkit_msgs
  ros2_medkit_gateway
  rclcpp
)

target_link_libraries(beacon_common_lib nlohmann_json::nlohmann_json)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Install library and export for downstream packages (topic_beacon)
install(TARGETS beacon_common_lib
  EXPORT beacon_common_libTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(beacon_common_libTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(ros2_medkit_msgs ros2_medkit_gateway rclcpp nlohmann_json)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_beacon_validator test/test_beacon_validator.cpp)
  target_link_libraries(test_beacon_validator beacon_common_lib)
  ament_target_dependencies(test_beacon_validator ros2_medkit_gateway)

  ament_add_gtest(test_beacon_hint_store test/test_beacon_hint_store.cpp)
  target_link_libraries(test_beacon_hint_store beacon_common_lib)
  ament_target_dependencies(test_beacon_hint_store ros2_medkit_gateway)

  ament_add_gtest(test_beacon_entity_mapper test/test_beacon_entity_mapper.cpp)
  target_link_libraries(test_beacon_entity_mapper beacon_common_lib)
  ament_target_dependencies(test_beacon_entity_mapper ros2_medkit_gateway)
endif()

ament_package()
