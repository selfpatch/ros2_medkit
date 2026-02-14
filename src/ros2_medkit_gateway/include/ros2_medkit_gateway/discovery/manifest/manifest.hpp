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

#include "ros2_medkit_gateway/discovery/models/app.hpp"
#include "ros2_medkit_gateway/discovery/models/area.hpp"
#include "ros2_medkit_gateway/discovery/models/component.hpp"
#include "ros2_medkit_gateway/discovery/models/function.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

using json = nlohmann::json;

/**
 * @brief Configuration for discovery behavior when manifest is present
 */
struct ManifestConfig {
  /**
   * @brief Policy for ROS nodes not declared in manifest
   */
  enum class UnmanifestedNodePolicy {
    IGNORE,            ///< Don't expose unmanifested nodes
    WARN,              ///< Log warning, include as orphan
    ERROR,             ///< Fail startup
    INCLUDE_AS_ORPHAN  ///< Include with source="orphan"
  };

  UnmanifestedNodePolicy unmanifested_nodes{UnmanifestedNodePolicy::WARN};
  bool inherit_runtime_resources{true};  ///< Copy topics/services from runtime
  bool allow_manifest_override{true};    ///< Manifest can override runtime props

  /// Parse policy from string
  static UnmanifestedNodePolicy parse_policy(const std::string & str);
  /// Convert policy to string
  static std::string policy_to_string(UnmanifestedNodePolicy policy);
};

/**
 * @brief Manifest metadata
 */
struct ManifestMetadata {
  std::string name;
  std::string description;
  std::string version;
  std::string created_at;
};

/**
 * @brief Full manifest structure
 *
 * Represents a parsed manifest YAML file containing entity definitions
 * and discovery configuration.
 */
struct Manifest {
  std::string manifest_version;  ///< Must be "1.0"
  ManifestMetadata metadata;
  ManifestConfig config;

  std::vector<Area> areas;
  std::vector<Component> components;
  std::vector<App> apps;
  std::vector<Function> functions;

  /// Custom capabilities overrides per entity
  std::unordered_map<std::string, json> capabilities;

  /// Check if manifest has been loaded
  bool is_loaded() const {
    return !manifest_version.empty();
  }
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway

