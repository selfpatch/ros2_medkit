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

#include "ros2_medkit_gateway/aggregation/classification.hpp"

namespace ros2_medkit_gateway {

ClassifiedRouting classify_component_routing(const std::vector<Component> & merged_components,
                                             const std::vector<PeerClaim> & peer_claims) {
  ClassifiedRouting result;

  // Identify hierarchical parents: any Component referenced as parent_component_id.
  std::unordered_set<std::string> hierarchical_parents;
  for (const auto & comp : merged_components) {
    if (!comp.parent_component_id.empty()) {
      hierarchical_parents.insert(comp.parent_component_id);
    }
  }

  // Rebuild routing table applying last-writer-wins, skipping hierarchical
  // parents. Track which peers claimed each ID so we can emit warnings on
  // leaf IDs contested by more than one peer.
  std::unordered_map<std::string, std::vector<std::string>> claims_by_id;
  for (const auto & pc : peer_claims) {
    for (const auto & id : pc.claimed_entity_ids) {
      claims_by_id[id].push_back(pc.peer_name);
      if (hierarchical_parents.count(id) == 0u) {
        result.routing_table[id] = pc.peer_name;
      }
    }
  }

  // Warnings: emit ONLY for leaves (never for hierarchical parents, whose
  // multi-peer presence is expected by design).
  for (const auto & [id, peers] : claims_by_id) {
    if (peers.size() >= 2u && hierarchical_parents.count(id) == 0u) {
      LeafCollisionWarning w;
      w.entity_id = id;
      w.peer_names = peers;
      result.leaf_warnings.push_back(std::move(w));
    }
  }

  return result;
}

}  // namespace ros2_medkit_gateway
