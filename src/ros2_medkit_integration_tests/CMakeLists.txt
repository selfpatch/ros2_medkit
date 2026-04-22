cmake_minimum_required(VERSION 3.14)
project(ros2_medkit_integration_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# Shared cmake modules (multi-distro compat, ccache, linting)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitWarnings)

find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# === Demo node executables ===

# Shared include directory for the demo_node_main.hpp helper. Keeps the
# graceful-shutdown boilerplate out of every demo binary.
set(_demo_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(demo_engine_temp_sensor demo_nodes/engine_temp_sensor.cpp)
target_include_directories(demo_engine_temp_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_engine_temp_sensor rclcpp rcl_interfaces sensor_msgs)

add_executable(demo_rpm_sensor demo_nodes/rpm_sensor.cpp)
target_include_directories(demo_rpm_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_rpm_sensor rclcpp std_msgs)

add_executable(demo_brake_pressure_sensor demo_nodes/brake_pressure_sensor.cpp)
target_include_directories(demo_brake_pressure_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_brake_pressure_sensor rclcpp std_msgs)

add_executable(demo_door_status_sensor demo_nodes/door_status_sensor.cpp)
target_include_directories(demo_door_status_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_door_status_sensor rclcpp std_msgs)

add_executable(demo_brake_actuator demo_nodes/brake_actuator.cpp)
target_include_directories(demo_brake_actuator PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_brake_actuator rclcpp std_msgs)

add_executable(demo_light_controller demo_nodes/light_controller.cpp)
target_include_directories(demo_light_controller PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_light_controller rclcpp std_msgs)

add_executable(demo_calibration_service demo_nodes/calibration_service.cpp)
target_include_directories(demo_calibration_service PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_calibration_service rclcpp std_srvs)

add_executable(demo_long_calibration_action demo_nodes/long_calibration_action.cpp)
target_include_directories(demo_long_calibration_action PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_long_calibration_action rclcpp rclcpp_action example_interfaces)

add_executable(demo_lidar_sensor demo_nodes/lidar_sensor.cpp)
target_include_directories(demo_lidar_sensor PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_lidar_sensor rclcpp rcl_interfaces sensor_msgs std_srvs ros2_medkit_msgs)

add_executable(demo_beacon_publisher demo_nodes/beacon_publisher.cpp)
target_include_directories(demo_beacon_publisher PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_beacon_publisher rclcpp ros2_medkit_msgs diagnostic_msgs)

add_executable(demo_param_beacon_node demo_nodes/param_beacon_node.cpp)
target_include_directories(demo_param_beacon_node PRIVATE ${_demo_include_dir})
medkit_target_dependencies(demo_param_beacon_node rclcpp)

install(TARGETS
  demo_engine_temp_sensor
  demo_rpm_sensor
  demo_brake_pressure_sensor
  demo_door_status_sensor
  demo_brake_actuator
  demo_light_controller
  demo_calibration_service
  demo_long_calibration_action
  demo_lidar_sensor
  demo_beacon_publisher
  demo_param_beacon_node
  DESTINATION lib/${PROJECT_NAME}
)

# === Python test utilities ===

ament_python_install_package(ros2_medkit_test_utils)

# === Install launch files ===

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# === Integration tests ===

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Each integration test gets a unique HTTP port via GATEWAY_TEST_PORT env var,
  # allowing parallel CTest execution without port conflicts.
  # Stride of 10 per test so multi-gateway tests can use get_test_port(offset).
  # Each test also gets a unique ROS_DOMAIN_ID to prevent DDS cross-contamination
  # between tests (e.g., stale DDS participants from previous tests leaking into
  # subsequent test's graph discovery). Multi-gateway tests (e.g. peer aggregation)
  # use get_test_domain_id(offset) for additional domains in the 230-232 range.
  set(_INTEG_PORT 9100)

  include(ROS2MedkitTestDomain)
  medkit_init_test_domains(START 140 END 229)

  # Feature tests (atomic, independent, 120s timeout)
  file(GLOB FEATURE_TESTS test/features/*.test.py)
  foreach(test_file ${FEATURE_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_launch_test(${test_file}
      TARGET ${test_name}
      TIMEOUT 120
    )
    set_tests_properties(${test_name} PROPERTIES
      LABELS "integration;feature"
      ENVIRONMENT "GATEWAY_TEST_PORT=${_INTEG_PORT};ROS_DOMAIN_ID=${_MEDKIT_DOMAIN_COUNTER}"
    )
    math(EXPR _MEDKIT_DOMAIN_COUNTER "${_MEDKIT_DOMAIN_COUNTER} + 1")
    math(EXPR _INTEG_PORT "${_INTEG_PORT} + 10")
  endforeach()

  # Scenario tests (E2E stories, 300s timeout)
  file(GLOB SCENARIO_TESTS test/scenarios/*.test.py)
  foreach(test_file ${SCENARIO_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_launch_test(${test_file}
      TARGET ${test_name}
      TIMEOUT 300
    )
    set_tests_properties(${test_name} PROPERTIES
      LABELS "integration;scenario"
      ENVIRONMENT "GATEWAY_TEST_PORT=${_INTEG_PORT};ROS_DOMAIN_ID=${_MEDKIT_DOMAIN_COUNTER}"
    )
    math(EXPR _MEDKIT_DOMAIN_COUNTER "${_MEDKIT_DOMAIN_COUNTER} + 1")
    math(EXPR _INTEG_PORT "${_INTEG_PORT} + 10")
  endforeach()

  # Verify domain range not exhausted
  if(_MEDKIT_DOMAIN_COUNTER GREATER _MEDKIT_DOMAIN_END)
    message(FATAL_ERROR "Integration test domain range exhausted (${_MEDKIT_DOMAIN_START}-${_MEDKIT_DOMAIN_END})")
  endif()

  # Serialise multi-gateway tests that share the secondary DDS domain pool
  # (230-232, see ros2_medkit_test_utils.constants.get_test_domain_id). CTest
  # will not run two tests holding this resource lock concurrently, so they
  # cannot collide on those shared domains. Every integration test that
  # calls get_test_domain_id(offset>0) must be listed here.
  set_tests_properties(
    test_peer_aggregation
    test_cross_ecu_fanout
    test_daisy_chain_aggregation
    test_leaf_collision_aggregation
    PROPERTIES RESOURCE_LOCK medkit_secondary_dds_domains
  )
endif()

ament_package()
