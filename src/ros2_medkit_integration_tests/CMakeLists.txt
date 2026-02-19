cmake_minimum_required(VERSION 3.14)
project(ros2_medkit_integration_tests)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)

# === Demo node executables ===

add_executable(demo_engine_temp_sensor demo_nodes/engine_temp_sensor.cpp)
ament_target_dependencies(demo_engine_temp_sensor rclcpp rcl_interfaces sensor_msgs)

add_executable(demo_rpm_sensor demo_nodes/rpm_sensor.cpp)
ament_target_dependencies(demo_rpm_sensor rclcpp std_msgs)

add_executable(demo_brake_pressure_sensor demo_nodes/brake_pressure_sensor.cpp)
ament_target_dependencies(demo_brake_pressure_sensor rclcpp std_msgs)

add_executable(demo_door_status_sensor demo_nodes/door_status_sensor.cpp)
ament_target_dependencies(demo_door_status_sensor rclcpp std_msgs)

add_executable(demo_brake_actuator demo_nodes/brake_actuator.cpp)
ament_target_dependencies(demo_brake_actuator rclcpp std_msgs)

add_executable(demo_light_controller demo_nodes/light_controller.cpp)
ament_target_dependencies(demo_light_controller rclcpp std_msgs)

add_executable(demo_calibration_service demo_nodes/calibration_service.cpp)
ament_target_dependencies(demo_calibration_service rclcpp std_srvs)

add_executable(demo_long_calibration_action demo_nodes/long_calibration_action.cpp)
ament_target_dependencies(demo_long_calibration_action rclcpp rclcpp_action example_interfaces)

add_executable(demo_lidar_sensor demo_nodes/lidar_sensor.cpp)
ament_target_dependencies(demo_lidar_sensor rclcpp rcl_interfaces sensor_msgs std_srvs ros2_medkit_msgs)

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
  DESTINATION lib/${PROJECT_NAME}
)

# === Python test utilities ===

ament_python_install_package(ros2_medkit_test_utils)

# === Install launch files ===

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

# === Integration tests ===

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Feature tests (atomic, independent, 120s timeout)
  file(GLOB FEATURE_TESTS test/features/*.test.py)
  foreach(test_file ${FEATURE_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_launch_test(${test_file}
      TARGET ${test_name}
      TIMEOUT 120
    )
    set_tests_properties(${test_name} PROPERTIES LABELS "integration;feature")
  endforeach()

  # Scenario tests (E2E stories, 300s timeout)
  file(GLOB SCENARIO_TESTS test/scenarios/*.test.py)
  foreach(test_file ${SCENARIO_TESTS})
    get_filename_component(test_name ${test_file} NAME_WE)
    add_launch_test(${test_file}
      TARGET ${test_name}
      TIMEOUT 300
    )
    set_tests_properties(${test_name} PROPERTIES LABELS "integration;scenario")
  endforeach()
endif()

ament_package()
