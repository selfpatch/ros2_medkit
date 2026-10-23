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

# ROS-free gateway layer: business logic, HTTP server, REST server, handlers, OpenAPI,
# plugin manager, aggregation and scripts. Plain CMake, no ament. Included by the ament
# build and by scripts/check_cores_without_ros. Defines the STATIC target `gateway_core`.
if(CMAKE_VERSION VERSION_LESS 3.16)
  message(FATAL_ERROR "gateway_core needs CMake 3.16 or newer")
endif()

set(_gwc_root "${CMAKE_CURRENT_LIST_DIR}/..")

# Version string from package.xml, so every build reports the same one.
file(READ "${_gwc_root}/package.xml" _gwc_package_xml)
string(REGEX MATCH "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" _ "${_gwc_package_xml}")
set(_gwc_version "${CMAKE_MATCH_1}")

find_package(Threads REQUIRED)

# Targets from the including build win; otherwise find the library; otherwise use the vendored copy.
if(NOT TARGET SQLite::SQLite3)
  find_package(SQLite3 REQUIRED)
endif()
if(NOT TARGET OpenSSL::SSL)
  find_package(OpenSSL REQUIRED)
endif()
if(NOT TARGET nlohmann_json::nlohmann_json)
  find_package(nlohmann_json REQUIRED)
endif()
# yaml-cpp 0.6 exports `yaml-cpp`, newer versions also `yaml-cpp::yaml-cpp`.
if(NOT TARGET yaml-cpp::yaml-cpp AND NOT TARGET yaml-cpp)
  find_package(yaml-cpp REQUIRED)
endif()
if(TARGET yaml-cpp::yaml-cpp)
  set(_gwc_yaml_cpp yaml-cpp::yaml-cpp)
else()
  set(_gwc_yaml_cpp yaml-cpp)
endif()

if(NOT TARGET cpp_httplib_target)
  if(TARGET httplib::httplib)
    add_library(cpp_httplib_target ALIAS httplib::httplib)
  else()
    add_library(cpp_httplib_target INTERFACE)
    target_include_directories(cpp_httplib_target SYSTEM INTERFACE "${_gwc_root}/src/vendored/cpp_httplib")
    target_link_libraries(cpp_httplib_target INTERFACE Threads::Threads)
  endif()
endif()
if(NOT TARGET tl::expected)
  add_library(tl_expected_iface INTERFACE)
  target_include_directories(tl_expected_iface SYSTEM INTERFACE
    $<BUILD_INTERFACE:${_gwc_root}/src/vendored/tl_expected/include>)
  add_library(tl::expected ALIAS tl_expected_iface)
endif()
if(NOT TARGET jwt-cpp::jwt-cpp)
  add_library(jwt_cpp_iface INTERFACE)
  target_include_directories(jwt_cpp_iface SYSTEM INTERFACE "${_gwc_root}/src/vendored/jwt_cpp/include")
  add_library(jwt-cpp::jwt-cpp ALIAS jwt_cpp_iface)
endif()
if(TARGET ros2_medkit_fault_detection::ros2_medkit_fault_detection)
  set(_gwc_fault_detection ros2_medkit_fault_detection::ros2_medkit_fault_detection)
else()
  if(NOT TARGET ros2_medkit_fault_detection)
    add_library(ros2_medkit_fault_detection INTERFACE)
    target_include_directories(ros2_medkit_fault_detection INTERFACE
      "${_gwc_root}/../ros2_medkit_fault_detection/include")
  endif()
  set(_gwc_fault_detection ros2_medkit_fault_detection)
endif()

file(GLOB_RECURSE _gwc_core_sources CONFIGURE_DEPENDS "${_gwc_root}/src/core/*.cpp")
file(GLOB _gwc_handler_sources CONFIGURE_DEPENDS "${_gwc_root}/src/http/handlers/*.cpp")
file(GLOB _gwc_openapi_sources CONFIGURE_DEPENDS "${_gwc_root}/src/openapi/*.cpp")

add_library(gateway_core STATIC
  ${_gwc_core_sources}
  ${_gwc_handler_sources}
  ${_gwc_openapi_sources}
  "${_gwc_root}/src/http/http_server.cpp"
  "${_gwc_root}/src/http/rest_server.cpp"
  "${_gwc_root}/src/http/fan_out_helpers.cpp"
  "${_gwc_root}/src/plugins/plugin_manager.cpp"
  "${_gwc_root}/src/plugins/plugin_loader.cpp"
  "${_gwc_root}/src/plugins/plugin_http_types.cpp"
  "${_gwc_root}/src/aggregation/aggregation_manager.cpp"
  "${_gwc_root}/src/script_manager.cpp"
  "${_gwc_root}/src/default_script_provider.cpp"
)

target_include_directories(gateway_core
  PUBLIC
    $<BUILD_INTERFACE:${_gwc_root}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    # Private implementation headers of the openapi module (not installed)
    $<BUILD_INTERFACE:${_gwc_root}/src/openapi>
)
target_include_directories(gateway_core SYSTEM PRIVATE "${_gwc_root}/src/vendored/mdns")

target_link_libraries(gateway_core
  PUBLIC
    cpp_httplib_target
    nlohmann_json::nlohmann_json
    ${_gwc_yaml_cpp}
    OpenSSL::SSL
    OpenSSL::Crypto
    tl::expected
    ${_gwc_fault_detection}
    jwt-cpp::jwt-cpp
    SQLite::SQLite3
    Threads::Threads
    ${CMAKE_DL_LIBS}
)

set_target_properties(gateway_core PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
)

target_compile_definitions(gateway_core PUBLIC
  CPPHTTPLIB_OPENSSL_SUPPORT
  GATEWAY_VERSION_STRING="${_gwc_version}")
# Emitted-status recorder (test builds only). Public: make_error() is inline and shared by every
# translation unit, so all of them must see the same definition.
if(GATEWAY_CORE_STATUS_RECORDER)
  target_compile_definitions(gateway_core PUBLIC MEDKIT_STATUS_RECORDER)
endif()

# Swagger UI: definitions and generated asset source, for every including build.
option(ENABLE_SWAGGER_UI "Embed Swagger UI assets in gateway binary" OFF)
if(ENABLE_SWAGGER_UI)
  target_compile_definitions(gateway_core PUBLIC ENABLE_SWAGGER_UI)
  include("${_gwc_root}/cmake/SwaggerUI.cmake")
  target_compile_definitions(gateway_core PRIVATE SWAGGER_UI_VERSION="${SWAGGER_UI_VERSION}")
endif()

# Heavy headers shared across the layer. Disable with -DCMAKE_DISABLE_PRECOMPILE_HEADERS=ON.
target_precompile_headers(gateway_core PRIVATE
  <string>
  <vector>
  <memory>
  <optional>
  <functional>
  <unordered_map>
  <algorithm>
  <sstream>
  <chrono>
  <mutex>
  <nlohmann/json.hpp>
  <httplib.h>
  <tl/expected.hpp>
)
