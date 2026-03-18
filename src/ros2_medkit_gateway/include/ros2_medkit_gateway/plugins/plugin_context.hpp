// Copyright 2026 bburda
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

#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/models/entity_types.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"

#include <functional>
#include <httplib.h>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <tl/expected.hpp>
#include <unordered_map>
#include <vector>

namespace rclcpp {
class Node;
}

namespace ros2_medkit_gateway {

class ResourceSamplerRegistry;

/**
 * @brief Entity information exposed to plugins
 */
struct PluginEntityInfo {
  SovdEntityType type{SovdEntityType::UNKNOWN};
  std::string id;
  std::string namespace_path;  ///< ROS 2 namespace (for component fault filtering)
  std::string fqn;             ///< Fully qualified ROS 2 node name
};

/**
 * @brief Context interface providing plugins access to gateway data and utilities
 *
 * Passed to plugins during lifecycle via set_context(). Replaces the old set_node()
 * by providing both ROS 2 node access and gateway-level abstractions.
 *
 * @note This interface is versioned alongside PLUGIN_API_VERSION. New methods may
 * be added in future versions (entity data access, configuration queries, etc.).
 *
 * @par Thread Safety
 * All methods are safe to call from any thread. Entity and fault queries use
 * the gateway's thread-safe caches internally.
 */
class PluginContext {
 public:
  virtual ~PluginContext() = default;

  // ---- ROS 2 access (replaces set_node) ----

  /// Get ROS 2 node pointer for subscriptions, service clients, etc.
  virtual rclcpp::Node * node() const = 0;

  // ---- Entity access (read-only) ----

  /// Look up an entity by ID. Returns nullopt if not found.
  virtual std::optional<PluginEntityInfo> get_entity(const std::string & id) const = 0;

  /// Get all Apps belonging to a Component (for aggregation endpoints)
  virtual std::vector<PluginEntityInfo> get_child_apps(const std::string & component_id) const = 0;

  // ---- Fault access ----

  /// List faults for a given entity. Returns JSON array of fault objects.
  /// Empty array if entity has no faults or fault manager is unavailable.
  virtual nlohmann::json list_entity_faults(const std::string & entity_id) const = 0;

  // ---- HTTP handler utilities (for entity-scoped routes) ----

  /**
   * @brief Validate entity exists and matches route type, sending SOVD error if not
   *
   * Use this in register_routes() handlers to validate entity IDs from path params.
   * On failure, an appropriate SOVD GenericError response is sent automatically.
   *
   * @param req HTTP request (extracts expected entity type from path)
   * @param res HTTP response (error sent here on failure)
   * @param entity_id Entity ID from path parameter (e.g., req.matches[1])
   * @return Entity info if valid, nullopt if error was sent
   */
  virtual std::optional<PluginEntityInfo> validate_entity_for_route(const httplib::Request & req,
                                                                    httplib::Response & res,
                                                                    const std::string & entity_id) const = 0;

  /// Send SOVD-compliant JSON error response
  static void send_error(httplib::Response & res, int status, const std::string & error_code,
                         const std::string & message, const nlohmann::json & parameters = {}) {
    res.status = status;
    nlohmann::json error_json;
    if (is_vendor_error_code(error_code)) {
      error_json["error_code"] = ERR_VENDOR_ERROR;
      error_json["vendor_code"] = error_code;
    } else {
      error_json["error_code"] = error_code;
    }
    error_json["message"] = message;
    if (!parameters.empty()) {
      error_json["parameters"] = parameters;
    }
    res.set_content(error_json.dump(2), "application/json");
  }

  /// Send JSON success response
  static void send_json(httplib::Response & res, const nlohmann::json & data) {
    res.set_content(data.dump(2), "application/json");
  }

  // ---- Capability registration ----

  /**
   * @brief Register a custom capability for all entities of a given type
   *
   * The capability will appear in the entity's capabilities array with an
   * auto-generated href. For example, registering "x-medkit-traces" for
   * SovdEntityType::APP produces: {"name": "x-medkit-traces", "href": "/api/v1/apps/{id}/x-medkit-traces"}
   *
   * The plugin must also register a matching route in register_routes().
   *
   * @param entity_type Entity type to add the capability to
   * @param capability_name Capability name (use x- prefix for vendor extensions)
   */
  virtual void register_capability(SovdEntityType entity_type, const std::string & capability_name) = 0;

  /**
   * @brief Register a custom capability for a specific entity
   *
   * Like register_capability(entity_type, name) but scoped to a single entity.
   *
   * @param entity_id Specific entity ID
   * @param capability_name Capability name (use x- prefix for vendor extensions)
   */
  virtual void register_entity_capability(const std::string & entity_id, const std::string & capability_name) = 0;

  // ---- Capability query (used by discovery handlers) ----

  /// Get plugin-registered capabilities for an entity type
  virtual std::vector<std::string> get_type_capabilities(SovdEntityType entity_type) const = 0;

  /// Get plugin-registered capabilities for a specific entity
  virtual std::vector<std::string> get_entity_capabilities(const std::string & entity_id) const = 0;

  // ---- Entity bulk access ----

  /// Get a snapshot of all discovered entities (areas, components, apps, functions).
  /// Returns an IntrospectionInput populated from the current entity cache.
  virtual IntrospectionInput get_entity_snapshot() const {
    return {};
  }

  // ---- All-faults access ----

  /// List all faults across all entities. Returns JSON with "faults" array.
  /// Empty object if fault manager is unavailable.
  virtual nlohmann::json list_all_faults() const {
    return nlohmann::json::object();
  }

  // ---- Resource sampler registration ----

  /// Register a cyclic subscription sampler for a custom collection.
  virtual void register_sampler(
      const std::string & /*collection*/,
      std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> /*fn*/) {
  }
};

// Forward declarations
class GatewayNode;
class FaultManager;

/// Factory for creating the concrete gateway plugin context
std::unique_ptr<PluginContext> make_gateway_plugin_context(GatewayNode * node, FaultManager * fault_manager,
                                                           ResourceSamplerRegistry * sampler_registry = nullptr);

}  // namespace ros2_medkit_gateway
