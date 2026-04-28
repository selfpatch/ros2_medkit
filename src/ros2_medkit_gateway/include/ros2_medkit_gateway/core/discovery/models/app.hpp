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

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"

#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief SOVD App entity - represents a software application
 *
 * In the ROS 2 context, an App typically corresponds to a ROS node.
 * Apps are located on Components and can depend on other Apps.
 *
 * Apps can be:
 * - Manifest-defined: Declared in YAML manifest with ROS binding
 * - Runtime-discovered: Automatically created from ROS nodes (future)
 */
struct App {
  // === Required fields ===
  std::string id;    ///< Unique identifier
  std::string name;  ///< Human-readable name

  // === Optional SOVD fields ===
  std::string translation_id;     ///< For i18n support
  std::string description;        ///< Detailed description
  std::vector<std::string> tags;  ///< Tags for filtering

  // === Relationships ===
  std::string component_id;             ///< is-located-on relationship
  std::vector<std::string> depends_on;  ///< depends-on relationship (App IDs)

  // === ROS binding (for manifest) ===
  /**
   * @brief ROS 2 binding configuration for manifest-defined apps
   *
   * Specifies how to bind this App to a ROS 2 node at runtime.
   */
  struct RosBinding {
    std::string node_name;          ///< ROS node name to bind to
    std::string namespace_pattern;  ///< Namespace (can be "*" for wildcard)
    std::string topic_namespace;    ///< Alternative: bind by topic prefix

    bool is_empty() const {
      return node_name.empty() && topic_namespace.empty();
    }

    json to_json() const {
      json j;
      if (!node_name.empty()) {
        j["nodeName"] = node_name;
        if (!namespace_pattern.empty()) {
          j["namespace"] = namespace_pattern;
        }
      }
      if (!topic_namespace.empty()) {
        j["topicNamespace"] = topic_namespace;
      }
      return j;
    }
  };
  std::optional<RosBinding> ros_binding;

  // === Runtime state (populated after linking) ===
  std::optional<std::string> bound_fqn;  ///< Bound ROS node FQN
  bool is_online{false};                 ///< Whether the bound node is running
  bool external{false};                  ///< True if not a ROS node

  /// Get effective FQN: bound_fqn if available, otherwise derived from ros_binding.
  /// Returns empty string if neither is available. Used by handlers and samplers
  /// to resolve the ROS node FQN in manifest_only mode where runtime linking
  /// doesn't set bound_fqn.
  /// @note namespace_pattern must be empty or an exact namespace path (e.g. "/perception").
  /// Glob patterns like "**" or "prefix*" are not supported and will produce empty FQN.
  std::string effective_fqn() const {
    if (bound_fqn.has_value() && !bound_fqn->empty()) {
      return *bound_fqn;
    }
    if (ros_binding.has_value() && !ros_binding->node_name.empty()) {
      const auto & ns = ros_binding->namespace_pattern;
      // Wildcard or glob patterns cannot produce valid FQNs
      if (ns.find('*') != std::string::npos) {
        return "";
      }
      if (ns.empty()) {
        return "/" + ros_binding->node_name;
      }
      if (ns.back() != '/') {
        return ns + "/" + ros_binding->node_name;
      }
      return ns + ros_binding->node_name;
    }
    return "";
  }

  // === Resources (populated from bound node) ===
  ComponentTopics topics;
  std::vector<ServiceInfo> services;
  std::vector<ActionInfo> actions;

  // === Discovery metadata ===
  std::string source = "manifest";        ///< "manifest" or "runtime"
  std::string original_id;                ///< Pre-rename ID when collision-prefixed by aggregation
  std::vector<std::string> contributors;  ///< Aggregation provenance: "local" and/or "peer:<name>"

  // === Serialization methods ===

  /**
   * @brief Serialize to full JSON representation
   */
  json to_json() const;

  /**
   * @brief Create SOVD EntityReference format
   * @param base_url Base URL for href (e.g., "/api/v1")
   */
  json to_entity_reference(const std::string & base_url) const;

  /**
   * @brief Create SOVD Entity Capabilities format
   * @param base_url Base URL for capability URIs
   */
  json to_capabilities(const std::string & base_url) const;
};

}  // namespace ros2_medkit_gateway
