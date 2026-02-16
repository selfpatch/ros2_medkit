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
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_serialization REQUIRED)

# GenericClient compat shim dependencies (needed explicitly for Humble,
# transitively available on Iron+/Jazzy but harmless to declare unconditionally)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

# Find cpp-httplib — prefer pkg-config (system package on Jazzy/Noble),
# fall back to CMake config (built from source on Humble/Jammy)
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(cpp_httplib IMPORTED_TARGET cpp-httplib)
endif()
if(cpp_httplib_FOUND)
  add_library(cpp_httplib_target ALIAS PkgConfig::cpp_httplib)
else()
  find_package(httplib REQUIRED)
  add_library(cpp_httplib_target ALIAS httplib::httplib)
endif()

# Find OpenSSL (required by jwt-cpp for RS256 and optional TLS support)
find_package(OpenSSL REQUIRED)

# Enable OpenSSL support for cpp-httplib SSLServer
# This enables the httplib::SSLServer class for HTTPS support
add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)

# Vendored header-only libraries (no network access on ROS build farm)
# tl::expected v1.3.1 (CC0) - https://github.com/TartanLlama/expected
add_library(tl_expected_iface INTERFACE)
target_include_directories(tl_expected_iface SYSTEM INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/tl_expected/include
)
add_library(tl::expected ALIAS tl_expected_iface)

# jwt-cpp v0.7.0 (MIT) - https://github.com/Thalhammer/jwt-cpp
add_library(jwt_cpp_iface INTERFACE)
target_include_directories(jwt_cpp_iface SYSTEM INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/jwt_cpp/include
)
add_library(jwt-cpp::jwt-cpp ALIAS jwt_cpp_iface)

# Include directories
include_directories(include)

# Gateway library (shared between executable and tests for coverage)
add_library(gateway_lib STATIC
  src/config.cpp
  src/gateway_node.cpp
  src/data_access_manager.cpp
  src/type_introspection.cpp
  src/native_topic_sampler.cpp
  src/operation_manager.cpp
  src/configuration_manager.cpp
  src/fault_manager.cpp
  # Entity resource model
  src/models/entity_types.cpp
  src/models/entity_capabilities.cpp
  src/models/thread_safe_entity_cache.cpp
  src/models/aggregation_service.cpp
  # Discovery module
  src/discovery/discovery_enums.cpp
  src/discovery/discovery_manager.cpp
  src/discovery/runtime_discovery.cpp
  src/discovery/hybrid_discovery.cpp
  # Discovery models (with .cpp serialization)
  src/discovery/models/app.cpp
  src/discovery/models/function.cpp
  # Manifest parser, validator and manager
  src/discovery/manifest/manifest_parser.cpp
  src/discovery/manifest/manifest_validator.cpp
  src/discovery/manifest/manifest_manager.cpp
  src/discovery/manifest/runtime_linker.cpp
  # HTTP module (subfolder)
  src/http/http_server.cpp
  src/http/rest_server.cpp
  # HTTP handlers - discovery (unified)
  src/http/handlers/discovery_handlers.cpp
  # HTTP handlers - utilities
  src/http/handlers/capability_builder.cpp
  # HTTP handlers - resource handlers (unified across entity types)
  src/http/handlers/data_handlers.cpp
  # HTTP handlers - other
  src/http/handlers/handler_context.cpp
  src/http/handlers/health_handlers.cpp
  src/http/handlers/operation_handlers.cpp
  src/http/handlers/config_handlers.cpp
  src/http/handlers/fault_handlers.cpp
  src/http/handlers/bulkdata_handlers.cpp
  src/http/handlers/sse_fault_handler.cpp
  src/http/handlers/auth_handlers.cpp
  # HTTP utilities
  src/http/x_medkit.cpp
  src/http/entity_path_utils.cpp
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
  ros2_medkit_serialization
  rosidl_typesupport_cpp
  rosidl_typesupport_introspection_cpp
)

target_link_libraries(gateway_lib
  cpp_httplib_target
  nlohmann_json::nlohmann_json
  yaml-cpp::yaml-cpp
  OpenSSL::SSL
  OpenSSL::Crypto
  tl::expected
  jwt-cpp::jwt-cpp
)

# Gateway node executable
add_executable(gateway_node src/main.cpp)
target_link_libraries(gateway_node gateway_lib)

# Apply coverage flags to main targets
if(ENABLE_COVERAGE)
  foreach(_target gateway_lib gateway_node)
    target_compile_options(${_target} PRIVATE --coverage -O0 -g)
    target_link_options(${_target} PRIVATE --coverage)
  endforeach()
endif()

# Install
install(TARGETS gateway_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts with execute permissions explicitly set
install(PROGRAMS scripts/get_type_schema.py scripts/generate_dev_certs.sh
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

  # Limit clang-tidy to only report issues from our source files (not vendored deps)
  set(ament_cmake_clang_tidy_HEADER_FILTER "^${CMAKE_CURRENT_SOURCE_DIR}/(include|src|test)/")

  # Exclude linters that don't work well with vendored dependencies:
  # - uncrustify/cpplint: conflicts with clang-format
  # - copyright/clang_format: we configure manually to skip src/vendored/
  # - clang_tidy: we configure it manually below with increased timeout
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_cpplint
    ament_cmake_clang_tidy
    ament_cmake_copyright
    ament_cmake_clang_format
  )
  ament_lint_auto_find_test_dependencies()

  # Configure copyright check to exclude vendored dependencies
  find_package(ament_cmake_copyright REQUIRED)
  set(VENDORED_FILES
    "src/vendored/jwt_cpp/include/jwt-cpp/base.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/jwt.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/boost-json/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/boost-json/traits.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/danielaparker-jsoncons/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/danielaparker-jsoncons/traits.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/kazuho-picojson/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/kazuho-picojson/traits.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/nlohmann-json/defaults.h"
    "src/vendored/jwt_cpp/include/jwt-cpp/traits/nlohmann-json/traits.h"
    "src/vendored/jwt_cpp/include/picojson/picojson.h"
    "src/vendored/tl_expected/include/tl/expected.hpp"
  )
  ament_copyright(EXCLUDE ${VENDORED_FILES})

  # Configure clang-format to only check our source files (not vendored)
  find_package(ament_cmake_clang_format REQUIRED)
  file(GLOB_RECURSE _format_files
    "include/*.h" "include/*.hpp"
    "src/*.cpp" "src/*.h" "src/*.hpp"
    "test/*.cpp" "test/*.h" "test/*.hpp"
  )
  list(FILTER _format_files EXCLUDE REGEX ".*/vendored/.*")
  ament_clang_format(${_format_files}
    CONFIG_FILE "${ament_cmake_clang_format_CONFIG_FILE}"
  )

  # Configure clang-tidy manually with increased timeout (1500s instead of default 300s)
  # This is needed because the project has many files and clang-tidy analysis takes time
  ament_clang_tidy(
    "${CMAKE_CURRENT_BINARY_DIR}"
    CONFIG_FILE "${ament_cmake_clang_tidy_CONFIG_FILE}"
    HEADER_FILTER "${ament_cmake_clang_tidy_HEADER_FILTER}"
    TIMEOUT 1500
  )

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

  # Add TLS configuration tests
  ament_add_gtest(test_tls_config test/test_tls_config.cpp)
  target_link_libraries(test_tls_config gateway_lib)

  # Add FaultManager tests
  ament_add_gtest(test_fault_manager test/test_fault_manager.cpp)
  target_link_libraries(test_fault_manager gateway_lib)
  ament_target_dependencies(test_fault_manager rclcpp ros2_medkit_msgs)

  # Add discovery models tests
  ament_add_gtest(test_discovery_models test/test_discovery_models.cpp)
  target_link_libraries(test_discovery_models gateway_lib)

  # Add manifest parser tests
  ament_add_gtest(test_manifest_parser test/test_manifest_parser.cpp)
  target_link_libraries(test_manifest_parser gateway_lib)

  # Add manifest validator tests
  ament_add_gtest(test_manifest_validator test/test_manifest_validator.cpp)
  target_link_libraries(test_manifest_validator gateway_lib)

  # Add manifest manager tests
  ament_add_gtest(test_manifest_manager test/test_manifest_manager.cpp)
  target_link_libraries(test_manifest_manager gateway_lib)

  # Add runtime linker tests
  ament_add_gtest(test_runtime_linker test/test_runtime_linker.cpp)
  target_link_libraries(test_runtime_linker gateway_lib)

  # Add capability builder tests
  ament_add_gtest(test_capability_builder test/test_capability_builder.cpp)
  target_link_libraries(test_capability_builder gateway_lib)

  # Add handler context tests
  ament_add_gtest(test_handler_context test/test_handler_context.cpp)
  target_link_libraries(test_handler_context gateway_lib)

  # Add x-medkit extension tests
  ament_add_gtest(test_x_medkit test/test_x_medkit.cpp)
  target_link_libraries(test_x_medkit gateway_lib)

  # Add auth config tests
  ament_add_gtest(test_auth_config test/test_auth_config.cpp)
  target_link_libraries(test_auth_config gateway_lib)

  # Add data access manager tests
  ament_add_gtest(test_data_access_manager test/test_data_access_manager.cpp)
  target_link_libraries(test_data_access_manager gateway_lib)
  ament_target_dependencies(test_data_access_manager rclcpp std_msgs)

  # Add entity resource model tests
  ament_add_gtest(test_entity_resource_model test/test_entity_resource_model.cpp)
  target_link_libraries(test_entity_resource_model gateway_lib)

  # Add entity path utils tests
  ament_add_gtest(test_entity_path_utils test/test_entity_path_utils.cpp)
  target_link_libraries(test_entity_path_utils gateway_lib)

  # Add fault handlers tests (SOVD response building)
  ament_add_gtest(test_fault_handlers test/test_fault_handlers.cpp)
  target_link_libraries(test_fault_handlers gateway_lib)

  # Add bulk data handlers tests
  ament_add_gtest(test_bulkdata_handlers test/test_bulkdata_handlers.cpp)
  target_link_libraries(test_bulkdata_handlers gateway_lib)

  # Apply coverage flags to test targets
  if(ENABLE_COVERAGE)
    set(_test_targets
      test_gateway_node
      test_auth_manager
      test_operation_manager
      test_configuration_manager
      test_native_topic_sampler
      test_discovery_manager
      test_tls_config
      test_fault_manager
      test_discovery_models
      test_manifest_parser
      test_manifest_validator
      test_capability_builder
      test_handler_context
      test_auth_config
      test_data_access_manager
      test_entity_path_utils
      test_fault_handlers
      test_bulkdata_handlers
    )
    foreach(_target ${_test_targets})
      target_compile_options(${_target} PRIVATE --coverage -O0 -g)
      target_link_options(${_target} PRIVATE --coverage)
    endforeach()
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
    TIMEOUT 600
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

  # Add TLS/HTTPS integration tests
  add_launch_test(
    test/test_tls.test.py
    TARGET test_tls
    TIMEOUT 120
  )

  # Add manifest-only discovery integration tests
  add_launch_test(
    test/test_discovery_manifest.test.py
    TARGET test_discovery_manifest
    TIMEOUT 120
  )

  # Add hybrid discovery integration tests
  add_launch_test(
    test/test_discovery_hybrid.test.py
    TARGET test_discovery_hybrid
    TIMEOUT 120
  )

  # Add heuristic apps discovery integration tests
  add_launch_test(
    test/test_discovery_heuristic_apps.test.py
    TARGET test_discovery_heuristic_apps
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
