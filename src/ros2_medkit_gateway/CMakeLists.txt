cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_gateway)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
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

# Find cpp-httplib using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(cpp_httplib REQUIRED IMPORTED_TARGET cpp-httplib)

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
)

ament_target_dependencies(gateway_lib
  rclcpp
  std_msgs
  std_srvs
  rcl_interfaces
  ament_index_cpp
  action_msgs
)

target_link_libraries(gateway_lib
  PkgConfig::cpp_httplib
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp
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

  # Use custom clang-format and clang-tidy configs from repo root
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")

  # Exclude uncrustify (conflicts with clang-format) and cpplint (conflicts with clang-format include order)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()

  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gateway_node test/test_gateway_node.cpp)
  target_link_libraries(test_gateway_node gateway_lib)

  # Apply coverage flags to test target
  if(ENABLE_COVERAGE)
    target_compile_options(test_gateway_node PRIVATE --coverage -O0 -g)
    target_link_options(test_gateway_node PRIVATE --coverage)
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
endif()

ament_package()
