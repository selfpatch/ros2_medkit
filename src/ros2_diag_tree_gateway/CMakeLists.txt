cmake_minimum_required(VERSION 3.8)
project(ros2_diag_tree_gateway)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add httplib dependency
include(FetchContent)
fetchcontent_declare(
  httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG v0.14.3
)
fetchcontent_makeavailable(httplib)

# Gateway node executable
add_executable(gateway_node src/gateway_node.cpp)
ament_target_dependencies(gateway_node rclcpp)
target_link_libraries(gateway_node httplib::httplib)
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
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_gateway_node test/test_gateway_node.cpp)
  target_link_libraries(test_gateway_node httplib::httplib)
  ament_target_dependencies(test_gateway_node rclcpp)
endif()

ament_package()
