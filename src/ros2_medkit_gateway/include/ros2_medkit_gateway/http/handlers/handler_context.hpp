// Copyright 2025 bburda
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <httplib.h>

#include <iomanip>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/auth/auth_config.hpp"
#include "ros2_medkit_gateway/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/models/entity_capabilities.hpp"
#include "ros2_medkit_gateway/models/entity_types.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Entity type enumeration for SOVD entities
 * @note This is the legacy EntityType enum. For new code, prefer using SovdEntityType.
 */
enum class EntityType { COMPONENT, APP, AREA, FUNCTION, UNKNOWN };

/**
 * @brief Convert legacy EntityType to SovdEntityType
 */
inline SovdEntityType to_sovd_entity_type(EntityType type) {
  switch (type) {
    case EntityType::COMPONENT:
      return SovdEntityType::COMPONENT;
    case EntityType::APP:
      return SovdEntityType::APP;
    case EntityType::AREA:
      return SovdEntityType::AREA;
    case EntityType::FUNCTION:
      return SovdEntityType::FUNCTION;
    case EntityType::UNKNOWN:
    default:
      return SovdEntityType::UNKNOWN;
  }
}

/**
 * @brief Information about a resolved entity
 */
struct EntityInfo {
  EntityType type{EntityType::UNKNOWN};
  std::string id;
  std::string namespace_path;
  std::string fqn;         ///< Fully qualified name (for ROS 2 nodes)
  std::string id_field;    ///< JSON field name for ID ("component_id", "app_id", etc.)
  std::string error_name;  ///< Human-readable name for errors ("Component", "App", etc.)

  /**
   * @brief Get SovdEntityType equivalent
   */
  SovdEntityType sovd_type() const {
    return to_sovd_entity_type(type);
  }

  /**
   * @brief Check if entity supports a resource collection
   */
  bool supports_collection(ResourceCollection col) const {
    return EntityCapabilities::for_type(sovd_type()).supports_collection(col);
  }
};

// Forward declarations
class GatewayNode;
class AuthManager;

namespace handlers {

/**
 * @brief Shared context for all HTTP handlers
 *
 * Provides access to gateway node, configuration, and common utilities
 * that handlers need to process requests.
 */
class HandlerContext {
 public:
  HandlerContext(GatewayNode * node, const CorsConfig & cors_config, const AuthConfig & auth_config,
                 const TlsConfig & tls_config, AuthManager * auth_manager)
    : node_(node)
    , cors_config_(cors_config)
    , auth_config_(auth_config)
    , tls_config_(tls_config)
    , auth_manager_(auth_manager) {
  }

  /// Get gateway node
  GatewayNode * node() const {
    return node_;
  }

  /// Get CORS configuration
  const CorsConfig & cors_config() const {
    return cors_config_;
  }

  /// Get authentication configuration
  const AuthConfig & auth_config() const {
    return auth_config_;
  }

  /// Get TLS configuration
  const TlsConfig & tls_config() const {
    return tls_config_;
  }

  /// Get authentication manager (may be nullptr if auth disabled)
  AuthManager * auth_manager() const {
    return auth_manager_;
  }

  /**
   * @brief Validate entity ID (component_id, area_id, etc.)
   * @param entity_id The ID to validate
   * @return Empty if valid, error message otherwise
   */
  tl::expected<void, std::string> validate_entity_id(const std::string & entity_id) const;

  /**
   * @brief Get namespace path for a component
   * @param component_id Component ID
   * @return Namespace path or error message
   * @deprecated Use get_entity_info instead for unified entity handling
   */
  tl::expected<std::string, std::string> get_component_namespace_path(const std::string & component_id) const;

  /**
   * @brief Get information about any entity (Component, App, Area, Function)
   *
   * If expected_type is specified, searches ONLY in that collection.
   * If expected_type is UNKNOWN (default), searches all types in order:
   * Component, App, Area, Function - returns the first match found.
   *
   * @param entity_id Entity ID to look up
   * @param expected_type Optional: restrict search to specific entity type
   * @return EntityInfo with resolved details, or EntityInfo with UNKNOWN type if not found
   */
  EntityInfo get_entity_info(const std::string & entity_id,
                             SovdEntityType expected_type = SovdEntityType::UNKNOWN) const;

  /**
   * @brief Validate that entity supports a resource collection
   *
   * Checks EntityCapabilities based on SOVD spec (Table 8).
   * Returns error if the entity type doesn't support the collection.
   *
   * @param entity Entity information (from get_entity_info)
   * @param collection Resource collection to validate
   * @return std::nullopt if valid, error message if invalid
   */
  static std::optional<std::string> validate_collection_access(const EntityInfo & entity,
                                                               ResourceCollection collection);

  /**
   * @brief Validate entity exists and matches expected route type
   *
   * Unified validation helper that:
   * 1. Validates entity ID format
   * 2. Looks up entity in the expected collection (based on route path)
   * 3. Sends appropriate error responses:
   *    - 400 with "invalid-parameter" if ID format is invalid
   *    - 400 with "invalid-parameter" if entity exists but wrong type for route
   *    - 404 with "entity-not-found" if entity doesn't exist
   *
   * @param req HTTP request (used to extract expected type from path)
   * @param res HTTP response (error responses sent here)
   * @param entity_id Entity ID to validate
   * @return EntityInfo if valid, std::nullopt if error response was sent
   */
  std::optional<EntityInfo> validate_entity_for_route(const httplib::Request & req, httplib::Response & res,
                                                      const std::string & entity_id) const;

  /**
   * @brief Set CORS headers on response if origin is allowed
   * @param res HTTP response
   * @param origin Origin header value
   */
  void set_cors_headers(httplib::Response & res, const std::string & origin) const;

  /**
   * @brief Check if origin is allowed by CORS config
   * @param origin Origin header value
   * @return true if allowed
   */
  bool is_origin_allowed(const std::string & origin) const;

  /**
   * @brief Send JSON error response following SOVD GenericError schema
   *
   * Creates a response with the following structure:
   * {
   *   "error_code": "entity-not-found",
   *   "message": "Entity not found",
   *   "parameters": { ... }  // optional
   * }
   *
   * For vendor-specific errors (x-medkit-*), the response includes:
   * {
   *   "error_code": "vendor-error",
   *   "vendor_code": "x-medkit-ros2-service-unavailable",
   *   "message": "..."
   * }
   *
   * @param res HTTP response object
   * @param status HTTP status code
   * @param error_code SOVD error code (use constants from error_codes.hpp)
   * @param message Human-readable error message
   * @param parameters Optional additional parameters for context
   */
  static void send_error(httplib::Response & res, httplib::StatusCode status, const std::string & error_code,
                         const std::string & message, const nlohmann::json & parameters = {});

  /**
   * @brief Send JSON success response
   */
  static void send_json(httplib::Response & res, const nlohmann::json & data);

  /**
   * @brief Get logger for handlers
   */
  static rclcpp::Logger logger() {
    return rclcpp::get_logger("rest_server");
  }

 private:
  GatewayNode * node_;
  CorsConfig cors_config_;
  AuthConfig auth_config_;
  TlsConfig tls_config_;
  AuthManager * auth_manager_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
