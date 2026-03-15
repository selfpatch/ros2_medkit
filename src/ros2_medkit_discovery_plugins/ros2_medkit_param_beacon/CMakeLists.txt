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

cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_param_beacon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared cmake modules (multi-distro compat)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../../cmake")
include(ROS2MedkitCompat)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_beacon_common REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# cpp-httplib via multi-distro compatibility macro
medkit_find_cpp_httplib()

# Enable OpenSSL support for cpp-httplib
add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)

# MODULE target: loaded via dlopen at runtime by PluginManager.
add_library(param_beacon_plugin MODULE
  src/param_beacon_plugin.cpp
)

target_include_directories(param_beacon_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

medkit_target_dependencies(param_beacon_plugin
  ros2_medkit_gateway
  ros2_medkit_beacon_common
  rclcpp
)

target_link_libraries(param_beacon_plugin
  nlohmann_json::nlohmann_json
  ros2_medkit_beacon_common::beacon_common_lib
  cpp_httplib_target
  OpenSSL::SSL OpenSSL::Crypto
)

install(TARGETS param_beacon_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # uncrustify/cpplint conflict with project-wide clang-format (120 cols vs 100)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_format
  )
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_clang_format REQUIRED)
  file(GLOB_RECURSE _format_files
    "include/*.hpp" "src/*.cpp" "test/*.cpp"
  )
  ament_clang_format(${_format_files}
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../../.clang-format")

  find_package(ament_cmake_gmock REQUIRED)

  # Include plugin source in test (plugin is MODULE/dlopen, test instantiates directly)
  ament_add_gmock(test_param_beacon_plugin
    test/test_param_beacon_plugin.cpp
    src/param_beacon_plugin.cpp
    TIMEOUT 30)
  target_include_directories(test_param_beacon_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  medkit_target_dependencies(test_param_beacon_plugin
    ros2_medkit_gateway ros2_medkit_beacon_common rclcpp)
  target_link_libraries(test_param_beacon_plugin
    nlohmann_json::nlohmann_json
    ros2_medkit_beacon_common::beacon_common_lib
    cpp_httplib_target
    OpenSSL::SSL OpenSSL::Crypto)
  set_tests_properties(test_param_beacon_plugin PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=70")
endif()

ament_package()
