cmake_minimum_required(VERSION 3.8)
project(ros2_medkit_gateway)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Shared cmake modules (multi-distro compat, ccache, linting)
find_package(ros2_medkit_cmake REQUIRED)
include(ROS2MedkitCompat)
include(ROS2MedkitCcache)
include(ROS2MedkitSanitizers)
include(ROS2MedkitLinting)

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

# Swagger UI embedding option
option(ENABLE_SWAGGER_UI "Embed Swagger UI assets in gateway binary" OFF)

# Extract version from package.xml so version.hpp stays in sync automatically
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" _package_xml)
string(REGEX MATCH "<version>([0-9]+\\.[0-9]+\\.[0-9]+)</version>" _ "${_package_xml}")
set(GATEWAY_VERSION "${CMAKE_MATCH_1}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
medkit_find_yaml_cpp()
find_package(ament_index_cpp REQUIRED)
find_package(action_msgs REQUIRED)
find_package(ros2_medkit_msgs REQUIRED)
find_package(ros2_medkit_serialization REQUIRED)
find_package(SQLite3 REQUIRED)

# GenericClient compat shim dependencies (needed explicitly for Humble,
# transitively available on Iron+/Jazzy but harmless to declare unconditionally)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)

medkit_find_cpp_httplib(VENDORED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/cpp_httplib")

# Find OpenSSL (required by jwt-cpp for RS256 and optional TLS support)
find_package(OpenSSL REQUIRED)

# Enable OpenSSL support for cpp-httplib SSLServer
# This enables the httplib::SSLServer class for HTTPS support
add_compile_definitions(CPPHTTPLIB_OPENSSL_SUPPORT)

# Vendored header-only libraries (no network access on ROS build farm)
# tl::expected v1.3.1 (CC0) - https://github.com/TartanLlama/expected
add_library(tl_expected_iface INTERFACE)
target_include_directories(tl_expected_iface SYSTEM INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/tl_expected/include>
  $<INSTALL_INTERFACE:include/ros2_medkit_gateway/vendored>
)
add_library(tl::expected ALIAS tl_expected_iface)

# jwt-cpp v0.7.0 (MIT) - https://github.com/Thalhammer/jwt-cpp
add_library(jwt_cpp_iface INTERFACE)
target_include_directories(jwt_cpp_iface SYSTEM INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/vendored/jwt_cpp/include
)
add_library(jwt-cpp::jwt-cpp ALIAS jwt_cpp_iface)

# mjansson/mdns (Public Domain) - https://github.com/mjansson/mdns
# Header-only C library for mDNS/DNS-SD, used for peer gateway discovery
include_directories(SYSTEM src/vendored/mdns)

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
  src/fault_manager_paths.cpp
  src/log_manager.cpp
  src/subscription_manager.cpp
  src/lock_manager.cpp
  src/resource_change_notifier.cpp
  src/resource_sampler.cpp
  src/subscription_transport.cpp
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
  src/discovery/merge_pipeline.cpp
  src/discovery/layers/manifest_layer.cpp
  src/discovery/layers/runtime_layer.cpp
  src/discovery/layers/plugin_layer.cpp
  # Host info provider (default component from host system)
  src/discovery/host_info_provider.cpp
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
  src/http/rate_limiter.cpp
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
  src/http/handlers/log_handlers.cpp
  src/http/handlers/lock_handlers.cpp
  src/http/handlers/bulkdata_handlers.cpp
  src/http/handlers/sse_fault_handler.cpp
  src/http/handlers/cyclic_subscription_handlers.cpp
  src/http/handlers/sse_transport_provider.cpp
  src/http/handlers/auth_handlers.cpp
  src/http/handlers/docs_handlers.cpp
  # Bulk data storage
  src/bulk_data_store.cpp
  # HTTP utilities
  src/http/x_medkit.cpp
  src/http/entity_path_utils.cpp
  # Auth module (subfolder)
  src/auth/auth_config.cpp
  src/auth/auth_models.cpp
  src/auth/auth_manager.cpp
  src/auth/auth_middleware.cpp
  src/auth/auth_requirement_policy.cpp
  # Updates module
  src/updates/update_manager.cpp
  # Scripts module
  src/script_manager.cpp
  src/default_script_provider.cpp
  # HTTP handlers - scripts
  src/http/handlers/script_handlers.cpp
  # HTTP handlers - updates
  src/http/handlers/update_handlers.cpp
  # Plugin framework
  src/plugins/plugin_context.cpp
  src/plugins/plugin_loader.cpp
  src/plugins/plugin_manager.cpp
  src/plugins/plugin_http_types.cpp
  # OpenAPI documentation module
  src/openapi/schema_builder.cpp
  src/openapi/path_builder.cpp
  src/openapi/openapi_spec_builder.cpp
  src/openapi/path_resolver.cpp
  src/openapi/capability_generator.cpp
  src/openapi/route_registry.cpp
  # Trigger persistence
  src/sqlite_trigger_store.cpp
  # Trigger management
  src/trigger_manager.cpp
  # Trigger fault subscriber (ROS 2 topic -> notifier bridge)
  src/trigger_fault_subscriber.cpp
  # Trigger topic subscriber (data triggers -> ROS 2 subscriptions)
  src/trigger_topic_subscriber.cpp
  # Trigger HTTP handlers
  src/http/handlers/trigger_handlers.cpp
  # Aggregation module
  src/aggregation/peer_client.cpp
  src/aggregation/entity_merger.cpp
  src/aggregation/aggregation_manager.cpp
  src/aggregation/mdns_discovery.cpp
  src/aggregation/sse_stream_proxy.cpp
)

medkit_target_dependencies(gateway_lib
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
  SQLite::SQLite3
  ${CMAKE_DL_LIBS}
)

# Precompiled headers - heavy headers used across most translation units.
# Speeds up full builds by ~30-40%. Disable with -DCMAKE_DISABLE_PRECOMPILE_HEADERS=ON.
target_precompile_headers(gateway_lib PRIVATE
  <string>
  <vector>
  <memory>
  <optional>
  <functional>
  <unordered_map>
  <algorithm>
  <sstream>
  <chrono>
  <mutex>
  <rclcpp/rclcpp.hpp>
  <nlohmann/json.hpp>
  <httplib.h>
  <tl/expected.hpp>
)
set_target_properties(gateway_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Pass version from package.xml to version.hpp
target_compile_definitions(gateway_lib PUBLIC GATEWAY_VERSION_STRING="${GATEWAY_VERSION}")

# Gateway node executable
add_executable(gateway_node src/main.cpp)
target_link_libraries(gateway_node gateway_lib)
# Export symbols to dlopen'd plugins (PluginContext::send_json, send_error, etc.)
set_target_properties(gateway_node PROPERTIES ENABLE_EXPORTS ON)

# Apply Swagger UI compile definition and embed assets
if(ENABLE_SWAGGER_UI)
  target_compile_definitions(gateway_lib PRIVATE ENABLE_SWAGGER_UI)
  target_compile_definitions(gateway_node PRIVATE ENABLE_SWAGGER_UI)
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/SwaggerUI.cmake)
  target_compile_definitions(gateway_lib PRIVATE SWAGGER_UI_VERSION="${SWAGGER_UI_VERSION}")
endif()

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
  # Use custom clang-format config from repo root
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")

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
    "src/vendored/mdns/mdns.h"
    "src/vendored/cpp_httplib/httplib.h"
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

  ros2_medkit_clang_tidy(
    HEADER_FILTER "^${CMAKE_CURRENT_SOURCE_DIR}/(include|src|test)/"
    TIMEOUT 3000
  )

  include(ROS2MedkitTestDomain)
  medkit_init_test_domains(START 30 END 89)

  # Add GTest
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp_action REQUIRED)
  find_package(example_interfaces REQUIRED)

  ament_add_gtest(test_gateway_node test/test_gateway_node.cpp)
  target_link_libraries(test_gateway_node gateway_lib)
  medkit_set_test_domain(test_gateway_node)

  ament_add_gtest(test_auth_manager test/test_auth_manager.cpp)
  target_link_libraries(test_auth_manager gateway_lib)

  ament_add_gtest(test_generic_client_compat test/test_generic_client_compat.cpp)
  target_link_libraries(test_generic_client_compat gateway_lib)
  medkit_target_dependencies(test_generic_client_compat rclcpp std_srvs)
  medkit_set_test_domain(test_generic_client_compat)

  ament_add_gtest(test_operation_manager test/test_operation_manager.cpp)
  target_link_libraries(test_operation_manager gateway_lib)
  medkit_set_test_domain(test_operation_manager)

  ament_add_gtest(test_configuration_manager test/test_configuration_manager.cpp)
  target_link_libraries(test_configuration_manager gateway_lib)
  medkit_set_test_domain(test_configuration_manager)

  # Add NativeTopicSampler tests
  find_package(std_msgs REQUIRED)
  ament_add_gtest(test_native_topic_sampler test/test_native_topic_sampler.cpp)
  target_link_libraries(test_native_topic_sampler gateway_lib)
  medkit_target_dependencies(test_native_topic_sampler std_msgs)
  medkit_set_test_domain(test_native_topic_sampler)

  # Add DiscoveryManager tests
  ament_add_gtest(test_discovery_manager test/test_discovery_manager.cpp)
  target_link_libraries(test_discovery_manager gateway_lib)
  medkit_target_dependencies(test_discovery_manager std_msgs)
  medkit_set_test_domain(test_discovery_manager)

  # Add RuntimeDiscoveryStrategy tests
  ament_add_gtest(test_runtime_discovery test/test_runtime_discovery.cpp)
  target_link_libraries(test_runtime_discovery gateway_lib)
  medkit_set_test_domain(test_runtime_discovery)

  # Add TLS configuration tests
  ament_add_gtest(test_tls_config test/test_tls_config.cpp)
  target_link_libraries(test_tls_config gateway_lib)

  # Add FaultManager tests
  ament_add_gtest(test_fault_manager test/test_fault_manager.cpp)
  target_link_libraries(test_fault_manager gateway_lib)
  medkit_target_dependencies(test_fault_manager rclcpp ros2_medkit_msgs)
  medkit_set_test_domain(test_fault_manager)

  # Add discovery models tests
  ament_add_gtest(test_discovery_models test/test_discovery_models.cpp)
  target_link_libraries(test_discovery_models gateway_lib)

  # Add host info provider tests
  ament_add_gtest(test_host_info_provider test/test_host_info_provider.cpp)
  target_link_libraries(test_host_info_provider gateway_lib)

  # Add discovery handlers tests
  ament_add_gtest(test_discovery_handlers test/test_discovery_handlers.cpp)
  target_link_libraries(test_discovery_handlers gateway_lib)
  medkit_set_test_domain(test_discovery_handlers)

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

  # Add merge pipeline tests
  ament_add_gtest(test_merge_pipeline test/test_merge_pipeline.cpp)
  target_link_libraries(test_merge_pipeline gateway_lib)

  # Add capability builder tests
  ament_add_gtest(test_capability_builder test/test_capability_builder.cpp)
  target_link_libraries(test_capability_builder gateway_lib)

  # Add handler context tests
  ament_add_gtest(test_handler_context test/test_handler_context.cpp)
  target_link_libraries(test_handler_context gateway_lib)
  medkit_set_test_domain(test_handler_context)

  # Add x-medkit extension tests
  ament_add_gtest(test_x_medkit test/test_x_medkit.cpp)
  target_link_libraries(test_x_medkit gateway_lib)

  # Add rate limiter tests
  ament_add_gtest(test_rate_limiter test/test_rate_limiter.cpp)
  target_link_libraries(test_rate_limiter gateway_lib)

  # Add auth config tests
  ament_add_gtest(test_auth_config test/test_auth_config.cpp)
  target_link_libraries(test_auth_config gateway_lib)

  # Add data access manager tests
  ament_add_gtest(test_data_access_manager test/test_data_access_manager.cpp)
  target_link_libraries(test_data_access_manager gateway_lib)
  medkit_target_dependencies(test_data_access_manager rclcpp std_msgs)
  medkit_set_test_domain(test_data_access_manager)

  # Add entity resource model tests
  ament_add_gtest(test_entity_resource_model test/test_entity_resource_model.cpp)
  target_link_libraries(test_entity_resource_model gateway_lib)

  # Function/Area resource collection aggregation tests
  ament_add_gtest(test_function_resource_collections test/test_function_resource_collections.cpp)
  target_link_libraries(test_function_resource_collections gateway_lib)

  # Add entity path utils tests
  ament_add_gtest(test_entity_path_utils test/test_entity_path_utils.cpp)
  target_link_libraries(test_entity_path_utils gateway_lib)

  # Add fault handlers tests (SOVD response building)
  ament_add_gtest(test_fault_handlers test/test_fault_handlers.cpp)
  target_link_libraries(test_fault_handlers gateway_lib)

  # Add bulk data store tests
  ament_add_gtest(test_bulk_data_store test/test_bulk_data_store.cpp)
  target_link_libraries(test_bulk_data_store gateway_lib)

  # Add bulk data handlers tests
  ament_add_gtest(test_bulkdata_handlers test/test_bulkdata_handlers.cpp)
  target_link_libraries(test_bulkdata_handlers gateway_lib)

  # Add subscription manager tests
  ament_add_gtest(test_subscription_manager test/test_subscription_manager.cpp)
  target_link_libraries(test_subscription_manager gateway_lib)

  # Add resource sampler registry tests
  ament_add_gtest(test_resource_sampler_registry test/test_resource_sampler_registry.cpp)
  target_link_libraries(test_resource_sampler_registry gateway_lib)

  # Add transport registry tests
  ament_add_gtest(test_transport_registry test/test_transport_registry.cpp)
  target_link_libraries(test_transport_registry gateway_lib)

  # Add cyclic subscription handler tests
  ament_add_gtest(test_cyclic_subscription_handlers test/test_cyclic_subscription_handlers.cpp)
  target_link_libraries(test_cyclic_subscription_handlers gateway_lib)

  # Add SSE transport provider tests
  ament_add_gtest(test_sse_transport_provider test/test_sse_transport_provider.cpp)
  target_link_libraries(test_sse_transport_provider gateway_lib)

  # Add SSE fault handler tests
  ament_add_gtest(test_sse_fault_handler test/test_sse_fault_handler.cpp)
  target_link_libraries(test_sse_fault_handler gateway_lib)
  medkit_target_dependencies(test_sse_fault_handler rclcpp ros2_medkit_msgs)
  medkit_set_test_domain(test_sse_fault_handler)

  # Add update manager tests
  ament_add_gtest(test_update_manager test/test_update_manager.cpp)
  target_link_libraries(test_update_manager gateway_lib)

  # Add script manager tests
  ament_add_gtest(test_script_manager test/test_script_manager.cpp)
  target_link_libraries(test_script_manager gateway_lib)

  # Add default script provider tests
  ament_add_gtest(test_default_script_provider test/test_default_script_provider.cpp)
  target_link_libraries(test_default_script_provider gateway_lib)

  # Add script handler tests
  ament_add_gtest(test_script_handlers test/test_script_handlers.cpp)
  target_link_libraries(test_script_handlers gateway_lib)
  medkit_set_test_domain(test_script_handlers)

  # Add data handler tests
  ament_add_gtest(test_data_handlers test/test_data_handlers.cpp)
  target_link_libraries(test_data_handlers gateway_lib)

  # Add auth handler tests
  ament_add_gtest(test_auth_handlers test/test_auth_handlers.cpp)
  target_link_libraries(test_auth_handlers gateway_lib)

  # Add health handler tests
  ament_add_gtest(test_health_handlers test/test_health_handlers.cpp)
  target_link_libraries(test_health_handlers gateway_lib)
  medkit_set_test_domain(test_health_handlers)

  # Add operation handler tests
  ament_add_gtest(test_operation_handlers test/test_operation_handlers.cpp)
  target_link_libraries(test_operation_handlers gateway_lib)
  medkit_target_dependencies(test_operation_handlers rclcpp rclcpp_action std_srvs example_interfaces)
  medkit_set_test_domain(test_operation_handlers)

  # Demo update backend plugin (.so for integration tests)
  add_library(test_update_backend MODULE
    test/demo_nodes/test_update_backend.cpp
  )
  target_include_directories(test_update_backend PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_update_backend
    nlohmann_json::nlohmann_json
    tl::expected
    cpp_httplib_target
    OpenSSL::SSL
    OpenSSL::Crypto
  )
  install(TARGETS test_update_backend
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  # Test gateway plugin (.so for plugin loader tests)
  add_library(test_gateway_plugin MODULE
    test/demo_nodes/test_gateway_plugin.cpp
  )
  target_link_libraries(test_gateway_plugin gateway_lib)
  install(TARGETS test_gateway_plugin
    LIBRARY DESTINATION lib/${PROJECT_NAME}
  )

  # Minimal test plugins for error-path coverage
  foreach(_plugin test_bad_version_plugin test_no_symbols_plugin test_null_factory_plugin test_version_only_plugin)
    add_library(${_plugin} MODULE test/demo_nodes/${_plugin}.cpp)
    target_include_directories(${_plugin} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(${_plugin} nlohmann_json::nlohmann_json)
    install(TARGETS ${_plugin} LIBRARY DESTINATION lib/${PROJECT_NAME})
  endforeach()

  # Minimal plugin with no provider query functions (only required exports)
  add_library(test_minimal_plugin MODULE test/demo_nodes/test_minimal_plugin.cpp)
  target_include_directories(test_minimal_plugin PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(test_minimal_plugin nlohmann_json::nlohmann_json cpp_httplib_target OpenSSL::SSL OpenSSL::Crypto)
  install(TARGETS test_minimal_plugin LIBRARY DESTINATION lib/${PROJECT_NAME})

  # Add plugin loader tests
  ament_add_gtest(test_plugin_loader test/test_plugin_loader.cpp)
  target_link_libraries(test_plugin_loader gateway_lib)
  medkit_target_dependencies(test_plugin_loader ament_index_cpp)

  # Plugin manager tests
  ament_add_gtest(test_plugin_manager test/test_plugin_manager.cpp)
  target_link_libraries(test_plugin_manager gateway_lib)

  # Plugin HTTP types tests
  ament_add_gtest(test_plugin_http_types test/test_plugin_http_types.cpp)
  target_link_libraries(test_plugin_http_types gateway_lib)

  # Log manager tests
  # Dedicated ROS_DOMAIN_ID to prevent cross-talk with concurrent integration tests
  ament_add_gtest(test_log_manager test/test_log_manager.cpp)
  target_link_libraries(test_log_manager gateway_lib)
  medkit_set_test_domain(test_log_manager)

  # Log handlers tests
  ament_add_gtest(test_log_handlers test/test_log_handlers.cpp)
  target_link_libraries(test_log_handlers gateway_lib)

  # Route descriptions tests (OpenAPI doc builder)
  ament_add_gtest(test_route_descriptions test/test_route_descriptions.cpp)
  target_link_libraries(test_route_descriptions gateway_lib)

  # Schema builder tests (OpenAPI schema generation)
  ament_add_gtest(test_schema_builder test/test_schema_builder.cpp)
  target_link_libraries(test_schema_builder gateway_lib)

  # Path builder tests (OpenAPI path item generation)
  ament_add_gtest(test_path_builder test/test_path_builder.cpp)
  target_link_libraries(test_path_builder gateway_lib)

  # OpenAPI spec builder tests (full document assembly)
  ament_add_gtest(test_openapi_spec_builder test/test_openapi_spec_builder.cpp)
  target_link_libraries(test_openapi_spec_builder gateway_lib)

  # Path resolver tests (OpenAPI path classification)
  ament_add_gtest(test_path_resolver test/test_path_resolver.cpp)
  target_link_libraries(test_path_resolver gateway_lib)

  # Capability generator tests (OpenAPI spec generation engine)
  ament_add_gtest(test_capability_generator test/test_capability_generator.cpp)
  target_link_libraries(test_capability_generator gateway_lib)
  medkit_set_test_domain(test_capability_generator)

  # Route registry tests (OpenAPI route registration and path conversion)
  ament_add_gtest(test_route_registry test/test_route_registry.cpp)
  target_link_libraries(test_route_registry gateway_lib)

  # Docs handlers tests (OpenAPI /docs endpoint handlers)
  ament_add_gtest(test_docs_handlers test/test_docs_handlers.cpp)
  target_link_libraries(test_docs_handlers gateway_lib)
  medkit_set_test_domain(test_docs_handlers)

  # Lock manager tests
  ament_add_gtest(test_lock_manager test/test_lock_manager.cpp)
  target_link_libraries(test_lock_manager gateway_lib)

  # Lock handlers tests
  ament_add_gtest(test_lock_handlers test/test_lock_handlers.cpp)
  target_link_libraries(test_lock_handlers gateway_lib)
  medkit_set_test_domain(test_lock_handlers)

  # Condition evaluator tests (trigger condition interface + built-in evaluators)
  ament_add_gtest(test_condition_evaluator test/test_condition_evaluator.cpp)
  target_link_libraries(test_condition_evaluator gateway_lib)

  # Resource change notifier tests (async notification hub for triggers)
  ament_add_gtest(test_resource_change_notifier test/test_resource_change_notifier.cpp TIMEOUT 300)
  target_link_libraries(test_resource_change_notifier gateway_lib)

  # Trigger store tests (SQLite persistence for triggers)
  ament_add_gtest(test_trigger_store test/test_trigger_store.cpp)
  target_link_libraries(test_trigger_store gateway_lib)

  # Trigger manager tests (CRUD, condition eval, hierarchy matching)
  ament_add_gtest(test_trigger_manager test/test_trigger_manager.cpp)
  target_link_libraries(test_trigger_manager gateway_lib)

  # Trigger handler tests (REST endpoints for triggers)
  ament_add_gtest(test_trigger_handlers test/test_trigger_handlers.cpp)
  target_link_libraries(test_trigger_handlers gateway_lib)

  # Peer client tests (aggregation module)
  ament_add_gtest(test_peer_client test/test_peer_client.cpp)
  target_link_libraries(test_peer_client gateway_lib)

  # Entity merger tests (aggregation module)
  ament_add_gtest(test_entity_merger test/test_entity_merger.cpp)
  target_link_libraries(test_entity_merger gateway_lib)

  # Aggregation manager tests (aggregation module)
  ament_add_gtest(test_aggregation_manager test/test_aggregation_manager.cpp)
  target_link_libraries(test_aggregation_manager gateway_lib)

  # Stream proxy tests (SSE parsing, aggregation module)
  ament_add_gtest(test_stream_proxy test/test_stream_proxy.cpp)
  target_link_libraries(test_stream_proxy gateway_lib)

  # mDNS discovery tests (aggregation module)
  ament_add_gtest(test_mdns_discovery test/test_mdns_discovery.cpp)
  target_link_libraries(test_mdns_discovery gateway_lib)

  # Network utility tests (URL parsing, local address collection)
  ament_add_gtest(test_network_utils test/test_network_utils.cpp)
  target_link_libraries(test_network_utils gateway_lib)

  # Apply coverage flags to test targets
  if(ENABLE_COVERAGE)
    set(_test_targets
      test_gateway_node
      test_auth_manager
      test_generic_client_compat
      test_operation_manager
      test_configuration_manager
      test_native_topic_sampler
      test_discovery_manager
      test_runtime_discovery
      test_tls_config
      test_fault_manager
      test_discovery_models
      test_host_info_provider
      test_discovery_handlers
      test_manifest_parser
      test_manifest_validator
      test_manifest_manager
      test_capability_builder
      test_handler_context
      test_x_medkit
      test_rate_limiter
      test_auth_config
      test_data_access_manager
      test_entity_resource_model
      test_function_resource_collections
      test_entity_path_utils
      test_fault_handlers
      test_bulk_data_store
      test_bulkdata_handlers
      test_subscription_manager
      test_resource_sampler_registry
      test_transport_registry
      test_cyclic_subscription_handlers
      test_sse_transport_provider
      test_sse_fault_handler
      test_update_manager
      test_script_manager
      test_default_script_provider
      test_script_handlers
      test_data_handlers
      test_auth_handlers
      test_health_handlers
      test_operation_handlers
      test_plugin_loader
      test_plugin_manager
      test_plugin_http_types
      test_log_manager
      test_log_handlers
      test_merge_pipeline
      test_runtime_linker
      test_route_descriptions
      test_schema_builder
      test_path_builder
      test_openapi_spec_builder
      test_path_resolver
      test_capability_generator
      test_route_registry
      test_docs_handlers
      test_lock_manager
      test_lock_handlers
      test_condition_evaluator
      test_resource_change_notifier
      test_trigger_store
      test_trigger_manager
      test_trigger_handlers
      test_peer_client
      test_entity_merger
      test_aggregation_manager
      test_stream_proxy
      test_mdns_discovery
      test_network_utils
    )
    foreach(_target ${_test_targets})
      target_compile_options(${_target} PRIVATE --coverage -O0 -g)
      target_link_options(${_target} PRIVATE --coverage)
    endforeach()
  endif()

endif()

# Export include directories for downstream packages (plugins)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY src/vendored/tl_expected/include/ DESTINATION include/ros2_medkit_gateway/vendored)
# Vendored httplib.h installed to share/ (NOT include/) so it does not pollute
# downstream include paths. Used as VENDORED_DIR fallback by packages whose tests
# need httplib (e.g. graph_provider) on platforms where the system package is too old.
install(FILES src/vendored/cpp_httplib/httplib.h
  DESTINATION share/${PROJECT_NAME}/vendored/cpp_httplib)
ament_export_include_directories(include include/ros2_medkit_gateway/vendored)

ament_package()
