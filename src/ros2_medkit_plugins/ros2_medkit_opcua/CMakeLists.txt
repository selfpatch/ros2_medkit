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

cmake_minimum_required(VERSION 3.16)
project(ros2_medkit_opcua)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Shared cmake modules (multi-distro compat)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitWarnings)

find_package(ament_cmake REQUIRED)
find_package(ros2_medkit_gateway REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)

# yaml-cpp via the medkit compat helper: jazzy exports the native
# ``yaml-cpp::yaml-cpp`` cmake target via ``yaml_cpp_vendor``; humble may fall
# back to the system library. The macro handles both cases and fails with a
# clear error if ``yaml_cpp_vendor`` is missing.
medkit_find_yaml_cpp()

# ---- open62541pp via FetchContent ----
# open62541pp v0.16.0 is MPLv2 licensed. Linking from Apache 2.0 code to MPLv2
# libraries is permitted - MPLv2 is a weak per-file copyleft that only requires
# source disclosure for modifications to the MPLv2 files themselves. We ship
# the library unmodified via FetchContent pinned to a specific upstream tag.
include(FetchContent)
fetchcontent_declare(
  open62541pp
  GIT_REPOSITORY https://github.com/open62541pp/open62541pp.git
  GIT_TAG 122ea4c842193918b97fd2f2def4fbecabb33ffe  # v0.16.0 pinned by SHA
  GIT_SUBMODULES_RECURSE ON
)
# Build open62541 as part of open62541pp, with -fPIC for shared library
set(UAPP_INTERNAL_OPEN62541 ON CACHE BOOL "" FORCE)
set(CMAKE_POSITION_INDEPENDENT_CODE ON CACHE BOOL "" FORCE)

# open62541 + open62541pp are third-party code pulled via FetchContent.
# ROS2MedkitWarnings promotes -Wswitch-enum, -Wnull-dereference, and other
# warnings to errors project-wide, and those fire on upstream C sources that
# we do not maintain. Temporarily clear directory-level COMPILE_OPTIONS before
# the FetchContent add_subdirectory so the third-party subdir inherits none of
# the strict flags, then restore them for our own targets defined below.
get_directory_property(_medkit_saved_compile_options COMPILE_OPTIONS)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "")
fetchcontent_makeavailable(open62541pp)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "${_medkit_saved_compile_options}")
unset(_medkit_saved_compile_options)

# Mark open62541pp's interface include directories as SYSTEM so consumers
# (our plugin and tests) do not propagate strict warnings into third-party
# header code. CMake propagates SYSTEM-ness only when the interface property
# is explicitly marked; we do that here on the open62541pp target so downstream
# target_link_libraries picks it up automatically.
if(TARGET open62541pp)
  get_target_property(_op62_includes open62541pp INTERFACE_INCLUDE_DIRECTORIES)
  if(_op62_includes)
    set_target_properties(open62541pp PROPERTIES
      INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_op62_includes}")
  endif()
  unset(_op62_includes)
endif()

# ---- MODULE target: loaded via dlopen at runtime by PluginManager ----
# Symbols from ros2_medkit_gateway are resolved from the host process at runtime.
add_library(ros2_medkit_opcua_plugin MODULE
  src/opcua_plugin.cpp
  src/opcua_plugin_exports.cpp
  src/opcua_client.cpp
  src/node_map.cpp
  src/opcua_poller.cpp
)

target_include_directories(ros2_medkit_opcua_plugin PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

medkit_target_dependencies(ros2_medkit_opcua_plugin
  ros2_medkit_msgs
  ros2_medkit_gateway
  rclcpp
  std_msgs
)

# Allow unresolved symbols - they resolve from the host process at runtime
target_link_options(ros2_medkit_opcua_plugin PRIVATE
  -Wl,--unresolved-symbols=ignore-all
)

target_link_libraries(ros2_medkit_opcua_plugin
  open62541pp::open62541pp
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp
  OpenSSL::SSL OpenSSL::Crypto
)

install(TARGETS ros2_medkit_opcua_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ---- Tests ----
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

  include(ROS2MedkitTestDomain)
  medkit_init_test_domains(START 220 END 229)

  ament_add_gtest(test_opcua_plugin
    test/test_opcua_plugin.cpp
    src/opcua_plugin.cpp
    src/opcua_client.cpp
    src/node_map.cpp
    src/opcua_poller.cpp
  )
  target_include_directories(test_opcua_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  medkit_target_dependencies(test_opcua_plugin
    ros2_medkit_msgs
    ros2_medkit_gateway
    rclcpp
    std_msgs
  )
  target_link_libraries(test_opcua_plugin
    open62541pp::open62541pp
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
    OpenSSL::SSL OpenSSL::Crypto
  )
  medkit_set_test_domain(test_opcua_plugin)

  ament_add_gtest(test_node_map
    test/test_node_map.cpp
    src/node_map.cpp
  )
  target_include_directories(test_node_map PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  medkit_target_dependencies(test_node_map
    ros2_medkit_gateway
    rclcpp
  )
  target_link_libraries(test_node_map
    open62541pp::open62541pp
    nlohmann_json::nlohmann_json
    yaml-cpp::yaml-cpp
  )
  medkit_set_test_domain(test_node_map)

  ament_add_gtest(test_opcua_client
    test/test_opcua_client.cpp
    src/opcua_client.cpp
  )
  target_include_directories(test_opcua_client PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  medkit_target_dependencies(test_opcua_client
    ros2_medkit_gateway
  )
  target_link_libraries(test_opcua_client
    open62541pp::open62541pp
    nlohmann_json::nlohmann_json
  )
  medkit_set_test_domain(test_opcua_client)

  # Issue #386: pure-function state machine tests. Header-only target -
  # no opcua dependency at link time so it runs fast and is sanitizer
  # clean independent of the open62541pp build flavour.
  ament_add_gtest(test_alarm_state_machine
    test/test_alarm_state_machine.cpp
  )
  target_include_directories(test_alarm_state_machine PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  medkit_set_test_domain(test_alarm_state_machine)

  # ---- test_alarm_server fixture ------------------------------------------
  # Standalone OPC-UA server emitting AlarmConditionType events for
  # integration testing of native alarm subscriptions (issue #386).
  #
  # The plugin's main open62541pp build is configured with
  # UA_NAMESPACE_ZERO=REDUCED and UA_ENABLE_SUBSCRIPTIONS_ALARMS_CONDITIONS=OFF
  # because the runtime client path uses neither. Re-enabling alarms there
  # would force every consumer of open62541pp to pull in the full namespace 0
  # (~5MB of generated source) and the EXPERIMENTAL A&C subsystem.
  #
  # Instead we build a second open62541 statically via ExternalProject_Add
  # using the source already on disk from FetchContent, pinned to FULL ns0
  # and alarms ON. The fixture binary links only against this private copy.
  # Defaults ON because the fixture is part of the standard test suite for
  # issue #386. The earlier `-j` race on open62541 namespace0_generated.c is
  # fixed by forcing a serial sub-build (BUILD_COMMAND below).
  option(MEDKIT_OPCUA_BUILD_ALARM_SERVER
    "Build the OPC-UA AlarmCondition test fixture server (issue #386)" ON)
  if(MEDKIT_OPCUA_BUILD_ALARM_SERVER)
    find_package(Threads REQUIRED)
    include(ExternalProject)
    set(_alarm_o62_src "${open62541pp_SOURCE_DIR}/3rdparty/open62541")
    set(_alarm_o62_install "${CMAKE_BINARY_DIR}/_alarm_open62541")
    externalproject_add(alarm_open62541_ep
      SOURCE_DIR "${_alarm_o62_src}"
      PREFIX "${CMAKE_BINARY_DIR}/_alarm_open62541_ep"
      INSTALL_DIR "${_alarm_o62_install}"
      CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_INSTALL_PREFIX=${_alarm_o62_install}
        -DBUILD_SHARED_LIBS=OFF
        -DUA_ENABLE_SUBSCRIPTIONS_EVENTS=ON
        -DUA_ENABLE_SUBSCRIPTIONS_ALARMS_CONDITIONS=ON
        -DUA_NAMESPACE_ZERO=FULL
        -DUA_BUILD_EXAMPLES=OFF
        -DUA_BUILD_TOOLS=OFF
        -DUA_FORCE_WERROR=OFF
        -DCMAKE_POSITION_INDEPENDENT_CODE=ON
        -DCMAKE_C_FLAGS=-w
      # Serial build avoids a -j race in open62541's Ninja-style codegen
      # where a parallel writer can clobber namespace0_generated.c.o.d
      # before the .d file is materialized. The build is one-shot, the
      # 30-60s overhead is well worth predictability.
      BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR>
      BUILD_BYPRODUCTS "${_alarm_o62_install}/lib/libopen62541.a"
      UPDATE_DISCONNECTED 1
    )
    set(_alarm_o62_lib "${_alarm_o62_install}/lib/libopen62541.a")
    set(_alarm_o62_include "${_alarm_o62_install}/include")
    file(MAKE_DIRECTORY "${_alarm_o62_include}")

    add_executable(test_alarm_server
      test/fixtures/test_alarm_server/test_alarm_server.cpp)
    add_dependencies(test_alarm_server alarm_open62541_ep)
    target_include_directories(test_alarm_server SYSTEM PRIVATE
      "${_alarm_o62_include}")
    target_link_libraries(test_alarm_server PRIVATE
      "${_alarm_o62_lib}" Threads::Threads)
    set_target_properties(test_alarm_server PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
    target_compile_options(test_alarm_server PRIVATE -w)

    install(TARGETS test_alarm_server
      RUNTIME DESTINATION lib/${PROJECT_NAME})
    install(PROGRAMS
      test/fixtures/test_alarm_server/smoke_test.py
      test/fixtures/test_alarm_server/run_ctest.py
      DESTINATION lib/${PROJECT_NAME})

    # CTest wrapper that boots test_alarm_server on an ephemeral port and runs
    # the asyncua-based smoke test against it. Skips with CTest exit 77 when
    # asyncua is not installed, so iterating on plugin code without the python
    # dep does not fail the suite. CI installs asyncua and observes a real
    # pass / fail.
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_test(NAME test_alarm_server_smoke
      COMMAND "${Python3_EXECUTABLE}"
              "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/test_alarm_server/run_ctest.py"
              "${CMAKE_BINARY_DIR}/test_alarm_server"
              "${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures/test_alarm_server/smoke_test.py")
    set_tests_properties(test_alarm_server_smoke PROPERTIES
      LABELS "integration"
      SKIP_RETURN_CODE 77
      TIMEOUT 60)
  endif()

  ros2_medkit_relax_vendor_warnings()
endif()

ament_package()
