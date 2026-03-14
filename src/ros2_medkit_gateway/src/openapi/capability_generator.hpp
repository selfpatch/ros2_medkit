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

#include <nlohmann/json.hpp>
#include <optional>
#include <string>

#include "path_resolver.hpp"
#include "schema_builder.hpp"

#include "ros2_medkit_gateway/models/entity_types.hpp"

namespace ros2_medkit_gateway {

class GatewayNode;
class PluginManager;

namespace handlers {
class HandlerContext;
}  // namespace handlers

namespace openapi {

/// Main engine that generates context-aware OpenAPI specs for any valid
/// gateway path. Uses PathResolver to classify the path, then dispatches
/// to the appropriate combination of SchemaBuilder + PathBuilder +
/// OpenApiSpecBuilder to produce the full document.
class CapabilityGenerator {
 public:
  CapabilityGenerator(handlers::HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr);

  /// Generate OpenAPI spec for the given base path (without /docs suffix).
  /// Returns nullopt if the path is not valid or resolvable.
  std::optional<nlohmann::json> generate(const std::string & base_path) const;

 private:
  nlohmann::json generate_root() const;
  nlohmann::json generate_entity_collection(const ResolvedPath & resolved) const;
  nlohmann::json generate_specific_entity(const ResolvedPath & resolved) const;
  nlohmann::json generate_resource_collection(const ResolvedPath & resolved) const;
  nlohmann::json generate_specific_resource(const ResolvedPath & resolved) const;

  /// Validate entity exists and parent-child relationships hold.
  bool validate_entity_hierarchy(const ResolvedPath & resolved) const;

  /// Build base spec with standard info/server/security blocks.
  nlohmann::json build_base_spec() const;

  /// Build the server URL from node parameters.
  std::string build_server_url() const;

  /// Map path resolver entity type to SovdEntityType for cache lookups.
  static ros2_medkit_gateway::SovdEntityType entity_type_from_keyword(const std::string & keyword);

  /// Get resource collection enum from path keyword string.
  static std::optional<ros2_medkit_gateway::ResourceCollection>
  resource_collection_from_keyword(const std::string & keyword);

  /// Build resource collection paths for a specific entity based on its capabilities.
  void add_resource_collection_paths(nlohmann::json & paths, const std::string & entity_path,
                                     const std::string & entity_id,
                                     ros2_medkit_gateway::SovdEntityType entity_type) const;

  handlers::HandlerContext & ctx_;
  GatewayNode & node_;
  PluginManager * plugin_mgr_;
  SchemaBuilder schema_builder_;
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
