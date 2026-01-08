cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_gateway)

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

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)

# Find cpp-httplib using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(cpp_httplib REQUIRED IMPORTED_TARGET cpp-httplib)

# Find OpenSSL (required by jwt-cpp for RS256)
find_package(OpenSSL REQUIRED)

# Fetch header-only libraries
include(FetchContent)

# Fetch tl::expected (C++11/14/17 compatible std::expected alternative)
# Disable tests and examples to avoid clang-tidy analyzing external code
set(EXPECTED_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(EXPECTED_BUILD_PACKAGE OFF CACHE BOOL "" FORCE)
fetchcontent_declare(
  tl_expected
  GIT_REPOSITORY https://github.com/TartanLlama/expected.git
  GIT_TAG        v1.3.1
  SYSTEM         # Treat as system headers to suppress warnings
  EXCLUDE_FROM_ALL  # Don't build tests/examples
)
fetchcontent_getproperties(tl_expected)
if(NOT tl_expected_POPULATED)
  fetchcontent_populate(tl_expected)
  # Only add include directory, don't add_subdirectory to avoid install conflicts
endif()

# Create interface library for tl::expected manually
add_library(tl_expected_iface INTERFACE)
target_include_directories(tl_expected_iface SYSTEM INTERFACE
  ${tl_expected_SOURCE_DIR}/include
)
add_library(tl::expected ALIAS tl_expected_iface)

# Fetch jwt-cpp header-only library
# Disable tests and examples to avoid clang-tidy analyzing external code
set(JWT_CPP_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(JWT_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
fetchcontent_declare(
  jwt-cpp
  GIT_REPOSITORY https://github.com/Thalhammer/jwt-cpp.git
  GIT_TAG        v0.7.0
  SYSTEM         # Treat as system headers to suppress warnings
  EXCLUDE_FROM_ALL  # Don't build tests/examples
)
fetchcontent_getproperties(jwt-cpp)
if(NOT jwt-cpp_POPULATED)
  fetchcontent_populate(jwt-cpp)
  # Only add include directory, don't add_subdirectory to avoid install conflicts
endif()

# Create interface library for jwt-cpp manually
add_library(jwt_cpp_iface INTERFACE)
target_include_directories(jwt_cpp_iface SYSTEM INTERFACE
  ${jwt-cpp_SOURCE_DIR}/include
)
add_library(jwt-cpp::jwt-cpp ALIAS jwt_cpp_iface)

# Include directories
include_directories(include)

# Gateway library (shared between executable and tests for coverage)
add_library(gateway_lib STATIC
  src/config.cpp
  src/gateway_node.cpp
  src/rest_server.cpp
  src/discovery_manager.cpp
  src/ros2_cli_wrapper.cpp
  src/output_parser.cpp
  src/data_access_manager.cpp
  src/type_introspection.cpp
  src/native_topic_sampler.cpp
  src/operation_manager.cpp
  src/configuration_manager.cpp
  src/fault_manager.cpp
  # Auth module (subfolder)
  src/auth/auth_config.cpp
  src/auth/auth_models.cpp
  src/auth/auth_manager.cpp
  src/auth/auth_middleware.cpp
  src/auth/auth_requirement_policy.cpp
)

ament_target_dependencies(gateway_lib
  rclcpp
  std_msgs
  std_srvs
  rcl_interfaces
  ament_index_cpp
  action_msgs
  ros2_medkit_msgs
)

target_link_libraries(gateway_lib
  PkgConfig::cpp_httplib
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp
  OpenSSL::SSL
  OpenSSL::Crypto
  tl::expected
  jwt-cpp::jwt-cpp
)

# Apply coverage flags to library
if(ENABLE_COVERAGE)
  target_compile_options(gateway_lib PRIVATE --coverage -O0 -g)
  target_link_options(gateway_lib PRIVATE --coverage)
endif()

# Gateway node executable
add_executable(gateway_node src/main.cpp)
target_link_libraries(gateway_node gateway_lib)

# Apply coverage flags to gateway_node target
if(ENABLE_COVERAGE)
  target_compile_options(gateway_node PRIVATE --coverage -O0 -g)
  target_link_options(gateway_node PRIVATE --coverage)
endif()

# Install
install(TARGETS gateway_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts with execute permissions explicitly set
install(PROGRAMS scripts/get_type_schema.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Testing
if(BUILD_TESTING)
  # Linting and code quality checks
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)

  # Use custom clang-format and clang-tidy configs from repo root
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")

  # Limit clang-tidy to only report issues from our source files (not FetchContent deps)
  set(ament_cmake_clang_tidy_HEADER_FILTER "^${CMAKE_CURRENT_SOURCE_DIR}/(include|src|test)/")

  # Exclude linters that don't work well with FetchContent dependencies:
  # - uncrustify/cpplint: conflicts with clang-format
  # - copyright/lint_cmake: flags generated/fetched files in install/
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
  )
  ament_lint_auto_find_test_dependencies()

  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gateway_node test/test_gateway_node.cpp)
  target_link_libraries(test_gateway_node gateway_lib)

  ament_add_gtest(test_auth_manager test/test_auth_manager.cpp)
  target_link_libraries(test_auth_manager gateway_lib)

  ament_add_gtest(test_operation_manager test/test_operation_manager.cpp)
  target_link_libraries(test_operation_manager gateway_lib)

  ament_add_gtest(test_configuration_manager test/test_configuration_manager.cpp)
  target_link_libraries(test_configuration_manager gateway_lib)

  # Add NativeTopicSampler tests
  find_package(std_msgs REQUIRED)
  ament_add_gtest(test_native_topic_sampler test/test_native_topic_sampler.cpp)
  target_link_libraries(test_native_topic_sampler gateway_lib)
  ament_target_dependencies(test_native_topic_sampler std_msgs)

  # Add DiscoveryManager tests
  ament_add_gtest(test_discovery_manager test/test_discovery_manager.cpp)
  target_link_libraries(test_discovery_manager gateway_lib)
  ament_target_dependencies(test_discovery_manager std_msgs)

  # Apply coverage flags to test targets
  if(ENABLE_COVERAGE)
    target_compile_options(test_gateway_node PRIVATE --coverage -O0 -g)
    target_link_options(test_gateway_node PRIVATE --coverage)
    target_compile_options(test_auth_manager PRIVATE --coverage -O0 -g)
    target_link_options(test_auth_manager PRIVATE --coverage)
    target_compile_options(test_operation_manager PRIVATE --coverage -O0 -g)
    target_link_options(test_operation_manager PRIVATE --coverage)
    target_compile_options(test_configuration_manager PRIVATE --coverage -O0 -g)
    target_link_options(test_configuration_manager PRIVATE --coverage)
    target_compile_options(test_native_topic_sampler PRIVATE --coverage -O0 -g)
    target_link_options(test_native_topic_sampler PRIVATE --coverage)
    target_compile_options(test_discovery_manager PRIVATE --coverage -O0 -g)
    target_link_options(test_discovery_manager PRIVATE --coverage)
  endif()

  # Integration testing
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Install test directory
  install(DIRECTORY test
    DESTINATION share/${PROJECT_NAME}
  )

  # Add integration tests
  add_launch_test(
    test/test_integration.test.py
    TARGET test_integration
    TIMEOUT 120
  )

  # Add CORS integration tests
  add_launch_test(
    test/test_cors.test.py
    TARGET test_cors
    TIMEOUT 60
  )

  # Add authentication integration tests
  add_launch_test(
    test/test_auth.test.py
    TARGET test_auth
    TIMEOUT 120
  )

  # Demo automotive nodes
  add_executable(demo_engine_temp_sensor
    test/demo_nodes/engine_temp_sensor.cpp
  )
  ament_target_dependencies(demo_engine_temp_sensor
    rclcpp
    rcl_interfaces
    sensor_msgs
  )
  install(TARGETS demo_engine_temp_sensor
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(demo_rpm_sensor
    test/demo_nodes/rpm_sensor.cpp
  )
  ament_target_dependencies(demo_rpm_sensor
    rclcpp
    std_msgs
  )
  install(TARGETS demo_rpm_sensor
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(demo_brake_pressure_sensor
    test/demo_nodes/brake_pressure_sensor.cpp
  )
  ament_target_dependencies(demo_brake_pressure_sensor
    rclcpp
    std_msgs
  )
  install(TARGETS demo_brake_pressure_sensor
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(demo_door_status_sensor
    test/demo_nodes/door_status_sensor.cpp
  )
  ament_target_dependencies(demo_door_status_sensor
    rclcpp
    std_msgs
  )
  install(TARGETS demo_door_status_sensor
    DESTINATION lib/${PROJECT_NAME}
  )

  # demo actuators
  add_executable(demo_brake_actuator
    test/demo_nodes/brake_actuator.cpp
  )
  ament_target_dependencies(demo_brake_actuator
    rclcpp
    std_msgs
  )
  install(TARGETS demo_brake_actuator
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(demo_light_controller
    test/demo_nodes/light_controller.cpp
  )
  ament_target_dependencies(demo_light_controller
    rclcpp
    std_msgs
  )
  install(TARGETS demo_light_controller
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(demo_calibration_service
    test/demo_nodes/calibration_service.cpp
  )
  ament_target_dependencies(demo_calibration_service
    rclcpp
    std_srvs
  )
  install(TARGETS demo_calibration_service
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(demo_long_calibration_action
    test/demo_nodes/long_calibration_action.cpp
  )
  ament_target_dependencies(demo_long_calibration_action
    rclcpp
    rclcpp_action
    example_interfaces
  )
  install(TARGETS demo_long_calibration_action
    DESTINATION lib/${PROJECT_NAME}
  )

  add_executable(demo_lidar_sensor
    test/demo_nodes/lidar_sensor.cpp
  )
  ament_target_dependencies(demo_lidar_sensor
    rclcpp
    rcl_interfaces
    sensor_msgs
    std_srvs
    ros2_medkit_msgs
  )
  install(TARGETS demo_lidar_sensor
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()
