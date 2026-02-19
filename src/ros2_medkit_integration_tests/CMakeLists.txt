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
