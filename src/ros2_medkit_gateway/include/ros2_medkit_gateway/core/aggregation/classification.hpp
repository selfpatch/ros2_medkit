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
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/component.hpp"

namespace ros2_medkit_gateway {

/// Deployment-level anomaly: >1 peer announced the same leaf Component ID.
/// Emitted as operator-visible warning (RCLCPP_WARN + optional /health field);
/// runtime falls back to last-writer-wins for the routing table entry.
struct LeafCollisionWarning {
  std::string entity_id;
  std::vector<std::string> peer_names;
};

/// Result of classifying a merged Component set into hierarchical parents
/// (served locally) and leaves (routed to an owning peer).
///
/// ``malformed_parent_warnings`` surfaces diagnostic strings for invalid
/// ``parent_component_id`` references detected during classification
/// (self-parent, non-existent parent, or a parent cycle). Such references are
/// ignored for hierarchical-parent detection, so the affected Components fall
/// back to leaf routing. Caller is expected to log each string so operators
/// can fix the underlying manifest or peer configuration.
struct ClassifiedRouting {
  std::unordered_map<std::string, std::string> routing_table;
  std::vector<LeafCollisionWarning> leaf_warnings;
  std::vector<std::string> malformed_parent_warnings;
};

/// Per-peer set of Component IDs that peer contributed to the routing table.
/// Used to detect multi-peer leaf collisions that would otherwise be hidden
/// by the last-writer-wins merge of per-peer routing tables.
struct PeerClaim {
  std::string peer_name;
  std::unordered_set<std::string> claimed_entity_ids;
};

/// Classify merged Components into hierarchical parents and leaves.
///
/// A Component is a hierarchical parent iff some other Component in the
/// merged set references it via parent_component_id. Hierarchical parents
/// are removed from the routing table (served locally from the merged
/// cache, mirroring how Areas/Functions are handled). Leaves stay in the
/// routing table and forward on request.
///
/// Multi-peer collisions on a leaf Component ID are reported as warnings;
/// routing falls back to last-writer-wins (the last peer in @p peer_claims
/// whose claim set contains the ID).
ClassifiedRouting classify_component_routing(const std::vector<Component> & merged_components,
                                             const std::vector<PeerClaim> & peer_claims);

}  // namespace ros2_medkit_gateway
