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
find_package(rcutils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
# yaml-cpp target resolution: Jazzy provides yaml-cpp::yaml-cpp via find_package,
# Humble's yaml_cpp_vendor bundles yaml-cpp without the namespaced cmake target.
find_package(yaml-cpp QUIET)
if(NOT TARGET yaml-cpp::yaml-cpp)
  find_library(YAML_CPP_LIB yaml-cpp)
  find_path(YAML_CPP_INCLUDE yaml-cpp/yaml.h)
  if(YAML_CPP_LIB AND YAML_CPP_INCLUDE)
    add_library(yaml-cpp::yaml-cpp IMPORTED INTERFACE)
    set_target_properties(yaml-cpp::yaml-cpp PROPERTIES
      INTERFACE_LINK_LIBRARIES "${YAML_CPP_LIB}"
      INTERFACE_INCLUDE_DIRECTORIES "${YAML_CPP_INCLUDE}"
    )
  else()
    message(FATAL_ERROR "Could not find yaml-cpp library")
  endif()
endif()
find_package(nlohmann_json REQUIRED)

# Library
add_library(${PROJECT_NAME}
  src/json_serializer.cpp
  src/type_cache.cpp
  src/service_action_types.cpp
  src/message_cleanup.cpp
  # Vendored dynmsg sources (C++ API only)
  src/vendored/dynmsg/typesupport.cpp
  src/vendored/dynmsg/message_reading_cpp.cpp
  src/vendored/dynmsg/msg_parser_cpp.cpp
  src/vendored/dynmsg/string_utils.cpp
  src/vendored/dynmsg/vector_utils.cpp
  src/vendored/dynmsg/yaml_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rcutils
  rosidl_runtime_c
  rosidl_typesupport_introspection_c
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
  rcutils
  rosidl_runtime_c
  rosidl_typesupport_introspection_c
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

  # Exclude conflicting linters and those we configure manually
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_tidy
    ament_cmake_clang_format
  )
  ament_lint_auto_find_test_dependencies()

  # Configure clang-format manually for non-vendored files only
  find_package(ament_cmake_clang_format REQUIRED)
  set(_clang_format_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")
  ament_clang_format(
    CONFIG_FILE "${_clang_format_config}"
    "include/ros2_medkit_serialization/json_serializer.hpp"
    "include/ros2_medkit_serialization/message_cleanup.hpp"
    "include/ros2_medkit_serialization/serialization_error.hpp"
    "include/ros2_medkit_serialization/service_action_types.hpp"
    "include/ros2_medkit_serialization/type_cache.hpp"
    "src/json_serializer.cpp"
    "src/message_cleanup.cpp"
    "src/service_action_types.cpp"
    "src/type_cache.cpp"
    "test/test_json_serializer.cpp"
    "test/test_service_action_types.cpp"
    "test/test_type_cache.cpp"
  )

  # Configure clang-tidy manually for non-vendored files only
  set(_clang_tidy_config "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-tidy")
  ament_clang_tidy(
    "${CMAKE_CURRENT_BINARY_DIR}"
    CONFIG_FILE "${_clang_tidy_config}"
    HEADER_FILTER "^${CMAKE_CURRENT_SOURCE_DIR}/include/ros2_medkit_serialization/[^v].*"
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
