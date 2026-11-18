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

# A plain-CMake host that differs from the main check: it supplies its own httplib::httplib,
# sets no global C++ standard and compiles its own code as C++14.
#   HOST_HTTPLIB_VERSION  httplib_VERSION to set; empty leaves it undefined
#   HOST_HTTPLIB_INCLUDE  include directory of the imported httplib::httplib
#   HOST_HTTPLIB_GLOBAL   ON makes the imported target GLOBAL
cmake_minimum_required(VERSION 3.16)
project(check_gateway_core_host LANGUAGES CXX)

set(HOST_HTTPLIB_VERSION "" CACHE STRING "")
set(HOST_HTTPLIB_INCLUDE "" CACHE PATH "")
option(HOST_HTTPLIB_GLOBAL "" OFF)

if(HOST_HTTPLIB_GLOBAL)
  add_library(httplib::httplib INTERFACE IMPORTED GLOBAL)
else()
  add_library(httplib::httplib INTERFACE IMPORTED)
endif()
set_target_properties(httplib::httplib PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${HOST_HTTPLIB_INCLUDE}")
if(NOT HOST_HTTPLIB_VERSION STREQUAL "")
  set(httplib_VERSION "${HOST_HTTPLIB_VERSION}")
endif()

include(${CMAKE_CURRENT_LIST_DIR}/../../../src/ros2_medkit_gateway/cmake/gateway_core.cmake)

add_executable(check_gateway_core_host consumer.cpp)
target_compile_features(check_gateway_core_host PRIVATE cxx_std_14)
set_target_properties(check_gateway_core_host PROPERTIES CXX_STANDARD 14 CXX_EXTENSIONS OFF)
target_link_libraries(check_gateway_core_host PRIVATE gateway_core)
