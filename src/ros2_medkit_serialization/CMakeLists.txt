cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_serialization)

set(CMAKE_CXX_STANDARD 17)
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
find_package(dynmsg REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

# Library
add_library(${PROJECT_NAME}
  src/json_serializer.cpp
  src/type_cache.cpp
  src/service_action_types.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  dynmsg
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_cpp
  rcpputils
  yaml_cpp_vendor
)

target_link_libraries(${PROJECT_NAME}
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp
)

# Apply coverage flags to library
if(ENABLE_COVERAGE)
  target_compile_options(${PROJECT_NAME} PRIVATE --coverage -O0 -g)
  target_link_options(${PROJECT_NAME} PRIVATE --coverage)
endif()

# Export the library
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  dynmsg
  rosidl_typesupport_introspection_cpp
  rosidl_runtime_cpp
  rcpputils
  yaml_cpp_vendor
  nlohmann_json
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(test_msgs REQUIRED)

  # Use custom clang-format and clang-tidy configs from repo root
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  set(ament_cmake_clang_tidy_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")

  # Limit clang-tidy to only report issues from our source files
  set(ament_cmake_clang_tidy_HEADER_FILTER "^${CMAKE_CURRENT_SOURCE_DIR}/(include|src|test)/")

  # Exclude linters that conflict with clang-format
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_tidy
  )
  ament_lint_auto_find_test_dependencies()

  # Configure clang-tidy manually with increased timeout
  ament_clang_tidy(
    "${CMAKE_CURRENT_BINARY_DIR}"
    CONFIG_FILE "${ament_cmake_clang_tidy_CONFIG_FILE}"
    HEADER_FILTER "${ament_cmake_clang_tidy_HEADER_FILTER}"
    TIMEOUT 300
  )

  # Unit tests
  ament_add_gtest(test_type_cache test/test_type_cache.cpp)
  target_link_libraries(test_type_cache ${PROJECT_NAME})
  ament_target_dependencies(test_type_cache std_msgs std_srvs)

  ament_add_gtest(test_json_serializer test/test_json_serializer.cpp)
  target_link_libraries(test_json_serializer ${PROJECT_NAME})
  ament_target_dependencies(test_json_serializer std_msgs std_srvs geometry_msgs sensor_msgs test_msgs)

  ament_add_gtest(test_service_action_types test/test_service_action_types.cpp)
  target_link_libraries(test_service_action_types ${PROJECT_NAME})
  ament_target_dependencies(test_service_action_types std_srvs)

  # Apply coverage flags to test targets
  if(ENABLE_COVERAGE)
    target_compile_options(test_type_cache PRIVATE --coverage -O0 -g)
    target_link_options(test_type_cache PRIVATE --coverage)
    target_compile_options(test_json_serializer PRIVATE --coverage -O0 -g)
    target_link_options(test_json_serializer PRIVATE --coverage)
    target_compile_options(test_service_action_types PRIVATE --coverage -O0 -g)
    target_link_options(test_service_action_types PRIVATE --coverage)
  endif()
endif()

ament_package()
