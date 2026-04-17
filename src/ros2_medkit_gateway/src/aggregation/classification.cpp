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

#include <algorithm>
#include <functional>

namespace ros2_medkit_gateway {

namespace {

/// Resolve valid parent edges from merged Components. Self-references,
/// references to unknown IDs, and cycle participants are excluded and a
/// warning string is emitted for each case. The returned map contains only
/// edges safe to use for hierarchical-parent detection.
std::unordered_map<std::string, std::string> resolve_valid_parents(const std::vector<Component> & merged_components,
                                                                   std::vector<std::string> & warnings_out) {
  std::unordered_set<std::string> known_ids;
  known_ids.reserve(merged_components.size());
  for (const auto & comp : merged_components) {
    known_ids.insert(comp.id);
  }

  std::unordered_map<std::string, std::string> valid_parent;
  for (const auto & comp : merged_components) {
    const auto & p = comp.parent_component_id;
    if (p.empty()) {
      continue;
    }
    if (p == comp.id) {
      warnings_out.push_back("Component '" + comp.id +
                             "' declares itself as parent_component_id; ignoring to preserve leaf routing");
      continue;
    }
    if (known_ids.count(p) == 0u) {
      warnings_out.push_back("Component '" + comp.id + "' references non-existent parent_component_id '" + p +
                             "'; ignoring (missing from merged entity set)");
      continue;
    }
    valid_parent[comp.id] = p;
  }

  // Cycle detection via 3-color DFS on the parent chain. Any Component whose
  // parent chain closes back on itself (A->B->A or longer) is removed from
  // valid_parent so it stays a routed leaf. One warning per cycle, listing
  // member IDs alphabetically for stable operator output.
  enum Color { White, Gray, Black };
  std::unordered_map<std::string, Color> color;
  std::unordered_set<std::string> cycle_members;

  std::function<void(const std::string &, std::vector<std::string> &)> visit = [&](const std::string & node,
                                                                                   std::vector<std::string> & stack) {
    color[node] = Gray;
    stack.push_back(node);
    auto edge = valid_parent.find(node);
    if (edge != valid_parent.end()) {
      const auto & next = edge->second;
      auto c = color.find(next);
      if (c != color.end() && c->second == Gray) {
        auto cycle_start = std::find(stack.begin(), stack.end(), next);
        for (auto it = cycle_start; it != stack.end(); ++it) {
          cycle_members.insert(*it);
        }
      } else if (c == color.end()) {
        visit(next, stack);
      }
    }
    stack.pop_back();
    color[node] = Black;
  };

  for (const auto & [id, _] : valid_parent) {
    if (color.count(id) == 0u) {
      std::vector<std::string> stack;
      visit(id, stack);
    }
  }

  if (!cycle_members.empty()) {
    std::vector<std::string> sorted_members(cycle_members.begin(), cycle_members.end());
    std::sort(sorted_members.begin(), sorted_members.end());
    std::string joined;
    for (size_t i = 0; i < sorted_members.size(); ++i) {
      if (i > 0u) {
        joined += ", ";
      }
      joined += sorted_members[i];
    }
    warnings_out.push_back("Parent cycle detected among Components [" + joined +
                           "]; cycle members kept as routed leaves");
    for (const auto & id : cycle_members) {
      valid_parent.erase(id);
    }
  }

  return valid_parent;
}

}  // namespace

ClassifiedRouting classify_component_routing(const std::vector<Component> & merged_components,
                                             const std::vector<PeerClaim> & peer_claims) {
  ClassifiedRouting result;

  // Validate parent edges first. Self-references, unknown parents, and cycle
  // members are dropped before hierarchical-parent detection so malformed
  // manifests cannot silently mask leaves.
  auto valid_parent = resolve_valid_parents(merged_components, result.malformed_parent_warnings);

  std::unordered_set<std::string> hierarchical_parents;
  for (const auto & [_, p] : valid_parent) {
    hierarchical_parents.insert(p);
  }

  // Rebuild routing table applying last-writer-wins, skipping hierarchical
  // parents. Track which peers claimed each ID so we can emit warnings on
  // leaf IDs contested by more than one peer. Iteration here is intentionally
  // driven by the input vector order (peer_claims), but the per-peer set is
  // unordered - sort each peer's IDs before consuming to keep downstream
  // output deterministic across runs.
  std::unordered_map<std::string, std::vector<std::string>> claims_by_id;
  for (const auto & pc : peer_claims) {
    std::vector<std::string> sorted_ids(pc.claimed_entity_ids.begin(), pc.claimed_entity_ids.end());
    std::sort(sorted_ids.begin(), sorted_ids.end());
    for (const auto & id : sorted_ids) {
      claims_by_id[id].push_back(pc.peer_name);
      if (hierarchical_parents.count(id) == 0u) {
        result.routing_table[id] = pc.peer_name;
      }
    }
  }

  // Warnings: emit ONLY for leaves (never for hierarchical parents, whose
  // multi-peer presence is expected by design). Produce results in a stable
  // order so operator logs and /health.warnings snapshot cleanly: collect
  // colliding leaf IDs, sort them, sort each peer_names list alphabetically.
  std::vector<std::string> colliding_leaf_ids;
  colliding_leaf_ids.reserve(claims_by_id.size());
  for (const auto & [id, peers] : claims_by_id) {
    if (peers.size() >= 2u && hierarchical_parents.count(id) == 0u) {
      colliding_leaf_ids.push_back(id);
    }
  }
  std::sort(colliding_leaf_ids.begin(), colliding_leaf_ids.end());
  for (const auto & id : colliding_leaf_ids) {
    LeafCollisionWarning w;
    w.entity_id = id;
    w.peer_names = claims_by_id.at(id);
    std::sort(w.peer_names.begin(), w.peer_names.end());
    result.leaf_warnings.push_back(std::move(w));
  }

  return result;
}

}  // namespace ros2_medkit_gateway
