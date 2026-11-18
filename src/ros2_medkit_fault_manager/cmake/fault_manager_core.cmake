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

# ROS-free fault manager core: storage records, the three backends, threshold resolver,
# correlation engine and audit log. Plain CMake, no ament. Included by the ament build and by
# scripts/check_cores_without_ros.
if(CMAKE_VERSION VERSION_LESS 3.16)
  message(FATAL_ERROR "fault_manager_core needs CMake 3.16 or newer")
endif()

set(_fmc_root "${CMAKE_CURRENT_LIST_DIR}/..")

# The ament build provides the namespaced imported target; a plain project gets the header by path.
if(TARGET ros2_medkit_fault_detection::ros2_medkit_fault_detection)
  set(_fmc_fault_detection ros2_medkit_fault_detection::ros2_medkit_fault_detection)
else()
  if(NOT TARGET ros2_medkit_fault_detection)
    add_library(ros2_medkit_fault_detection INTERFACE)
    target_include_directories(ros2_medkit_fault_detection INTERFACE
      "${_fmc_root}/../ros2_medkit_fault_detection/include")
  endif()
  set(_fmc_fault_detection ros2_medkit_fault_detection)
endif()

# A plain project finds the libraries itself; the ament build has already done so.
if(NOT TARGET SQLite::SQLite3)
  find_package(SQLite3 REQUIRED)
endif()
if(NOT TARGET OpenSSL::Crypto)
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
  set(_fmc_yaml_cpp yaml-cpp::yaml-cpp)
else()
  set(_fmc_yaml_cpp yaml-cpp)
endif()

set(_fmc_sources
  "${_fmc_root}/src/fault_storage.cpp"
  "${_fmc_root}/src/sqlite_fault_storage.cpp"
  "${_fmc_root}/src/fault_audit_log.cpp"
  "${_fmc_root}/src/threshold_resolver.cpp"
  "${_fmc_root}/src/correlation/types.cpp"
  "${_fmc_root}/src/correlation/config_parser.cpp"
  "${_fmc_root}/src/correlation/pattern_matcher.cpp"
  "${_fmc_root}/src/correlation/correlation_engine.cpp"
)
file(GLOB _fmc_core_sources CONFIGURE_DEPENDS "${_fmc_root}/src/core/*.cpp")
list(APPEND _fmc_sources ${_fmc_core_sources})

if(POSTGRES_SUPPORT)
  list(APPEND _fmc_sources "${_fmc_root}/src/postgres_fault_storage.cpp")
endif()

add_library(fault_manager_core STATIC ${_fmc_sources})
set_target_properties(fault_manager_core PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(fault_manager_core PUBLIC cxx_std_17)
target_include_directories(fault_manager_core PUBLIC
  $<BUILD_INTERFACE:${_fmc_root}/include>)
target_link_libraries(fault_manager_core PUBLIC
  ${_fmc_fault_detection}
  SQLite::SQLite3
  OpenSSL::Crypto
  nlohmann_json::nlohmann_json
  ${_fmc_yaml_cpp})

if(POSTGRES_SUPPORT)
  # The backend uses pqxx::params (libpqxx 7.5). The ament build already defines `pqxx`.
  if(NOT TARGET pqxx AND NOT TARGET libpqxx::pqxx)
    find_package(libpqxx 7.5 QUIET)
    if(NOT libpqxx_FOUND)
      message(FATAL_ERROR "fault_manager_core with POSTGRES_SUPPORT needs libpqxx 7.5 or newer")
    endif()
  endif()
  if(NOT TARGET PostgreSQL::PostgreSQL)
    find_package(PostgreSQL REQUIRED)
  endif()
  # libpq directly as well: the connection string is checked with PQconninfoParse.
  if(TARGET pqxx)
    target_link_libraries(fault_manager_core PUBLIC pqxx PostgreSQL::PostgreSQL)
  else()
    target_link_libraries(fault_manager_core PUBLIC libpqxx::pqxx PostgreSQL::PostgreSQL)
  endif()
  target_compile_definitions(fault_manager_core PUBLIC POSTGRES_SUPPORT)
endif()
