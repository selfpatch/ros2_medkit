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
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/auth/auth_config.hpp"
#include "ros2_medkit_gateway/auth/auth_manager.hpp"
#include "ros2_medkit_gateway/config.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Entity type enumeration for SOVD entities
 */
enum class EntityType { COMPONENT, APP, AREA, FUNCTION, UNKNOWN };

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
   * Searches through all entity types in order: Component, App, Area, Function.
   * Returns the first match found.
   *
   * @param entity_id Entity ID to look up
   * @return EntityInfo with resolved details, or EntityInfo with UNKNOWN type if not found
   */
  EntityInfo get_entity_info(const std::string & entity_id) const;

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
   * @brief Send JSON error response with simple message
   */
  static void send_error(httplib::Response & res, httplib::StatusCode status, const std::string & error);

  /**
   * @brief Send JSON error response with additional fields
   */
  static void send_error(httplib::Response & res, httplib::StatusCode status, const std::string & error,
                         const nlohmann::json & extra_fields);

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
