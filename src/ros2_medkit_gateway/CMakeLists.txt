cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Find cpp-httplib using pkg-config
find_package(PkgConfig REQUIRED)
pkg_check_modules(cpp_httplib REQUIRED IMPORTED_TARGET cpp-httplib)

# Gateway node executable
add_executable(gateway_node src/gateway_node.cpp)
ament_target_dependencies(gateway_node rclcpp)
target_link_libraries(gateway_node PkgConfig::cpp_httplib nlohmann_json::nlohmann_json)
target_include_directories(gateway_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(gateway_node PUBLIC c_std_99 cxx_std_17)

# Install targets
install(TARGETS gateway_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Exclude uncrustify since we use clang-format
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gateway_node test/test_gateway_node.cpp)
  target_link_libraries(test_gateway_node PkgConfig::cpp_httplib nlohmann_json::nlohmann_json)
  ament_target_dependencies(test_gateway_node rclcpp)
endif()

ament_package()
