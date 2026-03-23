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
project(ros2_medkit_graph_provider)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared cmake modules (multi-distro compat)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# cpp-httplib via multi-distro compatibility macro
medkit_find_cpp_httplib()

# Enable OpenSSL support for cpp-httplib
add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)

# MODULE target: loaded via dlopen at runtime by PluginManager.
# Symbols from gateway_lib are resolved from the host process at runtime.
# We only need headers at compile time (via medkit_target_dependencies).
add_library(ros2_medkit_graph_provider_plugin MODULE
  src/graph_provider_plugin.cpp
  src/graph_provider_plugin_exports.cpp
)

target_include_directories(ros2_medkit_graph_provider_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

medkit_target_dependencies(ros2_medkit_graph_provider_plugin
  ros2_medkit_msgs
  ros2_medkit_gateway
  rclcpp
  diagnostic_msgs
)

# Allow unresolved symbols - they resolve from the host process at runtime
target_link_options(ros2_medkit_graph_provider_plugin PRIVATE
  -Wl,--unresolved-symbols=ignore-all
)

target_link_libraries(ros2_medkit_graph_provider_plugin
  nlohmann_json::nlohmann_json
  cpp_httplib_target
  OpenSSL::SSL OpenSSL::Crypto
)

install(TARGETS ros2_medkit_graph_provider_plugin
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

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_graph_provider_plugin
    test/test_graph_provider_plugin.cpp
    src/graph_provider_plugin.cpp
  )
  target_include_directories(test_graph_provider_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  medkit_target_dependencies(test_graph_provider_plugin
    ros2_medkit_gateway
    rclcpp
    diagnostic_msgs
    ros2_medkit_msgs
  )
  target_link_libraries(test_graph_provider_plugin
    nlohmann_json::nlohmann_json
    cpp_httplib_target
    OpenSSL::SSL OpenSSL::Crypto
  )
  set_tests_properties(test_graph_provider_plugin
    PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=77"
  )
endif()

ament_package()
