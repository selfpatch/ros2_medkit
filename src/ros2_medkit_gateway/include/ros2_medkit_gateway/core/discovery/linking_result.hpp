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

#include "ros2_medkit_gateway/core/discovery/manifest/manifest.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"

#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Result of runtime linking operation
 *
 * Contains all the information about how manifest Apps were linked
 * to runtime ROS 2 nodes, including orphan detection.
 */
struct LinkingResult {
  /// Apps with successfully bound ROS nodes (and those that failed)
  std::vector<App> linked_apps;

  /// App IDs that have no matching ROS node (offline or external)
  std::vector<std::string> unlinked_app_ids;

  /// ROS node FQNs that have no matching manifest App (orphans)
  std::vector<std::string> orphan_nodes;

  /// Mapping from App ID to bound node FQN
  std::unordered_map<std::string, std::string> app_to_node;

  /// Mapping from node FQN to App ID (reverse lookup)
  std::unordered_map<std::string, std::string> node_to_app;

  /// Number of times two apps competed for the same node
  size_t binding_conflicts{0};

  /// Number of wildcard bindings that matched >1 node
  size_t wildcard_multi_match{0};

  /// Human-readable diagnostic warnings
  std::vector<std::string> warnings;

  /// Check if linking produced any errors based on policy
  bool has_errors(ManifestConfig::UnmanifestedNodePolicy policy) const {
    return policy == ManifestConfig::UnmanifestedNodePolicy::ERROR && !orphan_nodes.empty();
  }

  /// Get statistics summary
  std::string summary() const {
    std::string s = std::to_string(app_to_node.size()) + " linked, " + std::to_string(unlinked_app_ids.size()) +
                    " unlinked, " + std::to_string(orphan_nodes.size()) + " orphan nodes";
    if (binding_conflicts > 0) {
      s += ", " + std::to_string(binding_conflicts) + " binding conflict(s)";
    }
    return s;
  }
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
