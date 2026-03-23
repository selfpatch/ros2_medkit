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

#include <string>
#include <vector>

namespace ros2_medkit_gateway {
namespace openapi {

/// Classification of a resolved gateway path.
enum class PathCategory {
  kRoot,                ///< The API root path "/"
  kEntityCollection,    ///< e.g. "/areas", "/apps", "/areas/{id}/components"
  kSpecificEntity,      ///< e.g. "/apps/my_app", "/areas/powertrain"
  kResourceCollection,  ///< e.g. "/apps/my_app/data", "/apps/my_app/faults"
  kSpecificResource,    ///< e.g. "/apps/my_app/data/temperature"
  kUnresolved,          ///< Path does not match any known pattern
  kError                ///< Path contains a reserved or invalid segment
};

/// Represents a parent entity in a nested path (e.g. area "powertrain" in
/// /areas/powertrain/components/engine).
struct ParentEntity {
  std::string entity_type;
  std::string entity_id;
};

/// Result of resolving a gateway path. Contains the classification and
/// all extracted identifiers needed by downstream generators.
struct ResolvedPath {
  PathCategory category{PathCategory::kUnresolved};
  std::string entity_type;                 ///< Entity type keyword (e.g. "apps", "components")
  std::string entity_id;                   ///< Entity identifier (e.g. "my_app")
  std::string resource_collection;         ///< Resource collection keyword (e.g. "data", "faults")
  std::string resource_id;                 ///< Specific resource identifier (e.g. "temperature")
  std::vector<ParentEntity> parent_chain;  ///< Parent entities for nested paths
  std::string error;                       ///< Error description when category == kError
};

/// Pure path parser for gateway API paths. Classifies any valid gateway path
/// into one of 5 categories and extracts entity IDs and resource types.
/// No ROS 2 dependency - operates purely on path string structure.
class PathResolver {
 public:
  /// Resolve a gateway path (without /api/v1 prefix) into its components.
  /// @param path The path to resolve, e.g. "/apps/my_app/data/temperature"
  /// @return ResolvedPath with classification and extracted identifiers
  static ResolvedPath resolve(const std::string & path);

 private:
  /// Check if a segment is a known entity type keyword.
  static bool is_entity_type_keyword(const std::string & segment);

  /// Check if a segment is a known resource collection keyword.
  static bool is_resource_collection_keyword(const std::string & segment);

  /// Check if a segment is reserved and must not be used as an entity ID.
  static bool is_reserved_segment(const std::string & segment);
};

}  // namespace openapi
}  // namespace ros2_medkit_gateway
