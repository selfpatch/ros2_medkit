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

#include "ros2_medkit_gateway/core/aggregation/entity_merger.hpp"

#include <unordered_set>
#include <utility>

#include "ros2_medkit_gateway/core/discovery/identity_merge.hpp"

namespace ros2_medkit_gateway {

EntityMerger::EntityMerger(const std::string & peer_name) : peer_name_(peer_name) {
}

EntityMerger::EntityMerger(const std::string & peer_name, LocalIds local_ids)
  : peer_name_(peer_name), local_ids_(std::move(local_ids)) {
}

EntityMerger::LocalIds EntityMerger::collect_ids(const std::vector<Area> & areas,
                                                 const std::vector<Component> & components,
                                                 const std::vector<App> & apps,
                                                 const std::vector<Function> & functions) {
  LocalIds ids;
  for (const auto & area : areas) {
    ids.areas.insert(area.id);
  }
  for (const auto & comp : components) {
    ids.components.insert(comp.id);
  }
  for (const auto & app : apps) {
    ids.apps.insert(app.id);
  }
  for (const auto & func : functions) {
    ids.functions.insert(func.id);
  }
  return ids;
}

std::string EntityMerger::prefix_id(const std::string & id) const {
  return peer_name_ + SEPARATOR + id;
}

std::string EntityMerger::peer_source() const {
  return "peer:" + peer_name_;
}

bool EntityMerger::held_by_other_type(const std::string & id, Kind kind) const {
  return (kind != Kind::kArea && local_ids_.areas.count(id) > 0u) ||
         (kind != Kind::kComponent && local_ids_.components.count(id) > 0u) ||
         (kind != Kind::kApp && local_ids_.apps.count(id) > 0u) ||
         (kind != Kind::kFunction && local_ids_.functions.count(id) > 0u);
}

bool EntityMerger::held_by_any_type(const std::string & id) const {
  return local_ids_.areas.count(id) > 0u || local_ids_.components.count(id) > 0u || local_ids_.apps.count(id) > 0u ||
         local_ids_.functions.count(id) > 0u;
}

namespace {
void append_contributor_unique(std::vector<std::string> & contributors, const std::string & entry) {
  for (const auto & existing : contributors) {
    if (existing == entry) {
      return;
    }
  }
  contributors.push_back(entry);
}

/// Rewrite `id` to the ID it was renamed to, if it was.
void apply_rename(std::string & id, const std::unordered_map<std::string, std::string> & renames) {
  auto it = renames.find(id);
  if (it != renames.end()) {
    id = it->second;
  }
}

void apply_renames(std::vector<std::string> & ids, const std::unordered_map<std::string, std::string> & renames) {
  for (auto & id : ids) {
    apply_rename(id, renames);
  }
}
}  // namespace

const std::unordered_map<std::string, std::string> & EntityMerger::get_routing_table() const {
  return routing_table_;
}

const EntityMerger::Renames & EntityMerger::get_renames() const {
  return renames_;
}

const std::vector<std::string> & EntityMerger::get_dropped() const {
  return dropped_;
}

std::vector<Area> EntityMerger::merge_areas(const std::vector<Area> & local, const std::vector<Area> & remote) {
  // Start with copies of all local areas
  std::vector<Area> result = local;

  // Build index of local area IDs for fast lookup
  std::unordered_map<std::string, size_t> local_index;
  for (size_t i = 0; i < result.size(); ++i) {
    local_index[result[i].id] = i;
  }

  std::vector<size_t> added_index;
  for (const auto & remote_area : remote) {
    auto it = local_index.find(remote_area.id);
    if (it != local_index.end()) {
      // Collision: merge by combining tags (no duplication)
      auto & merged = result[it->second];

      // Merge tags without duplicates
      std::unordered_set<std::string> tag_set(merged.tags.begin(), merged.tags.end());
      for (const auto & tag : remote_area.tags) {
        if (tag_set.insert(tag).second) {
          merged.tags.push_back(tag);
        }
      }

      // If local has no description but remote does, take remote's
      if (merged.description.empty() && !remote_area.description.empty()) {
        merged.description = remote_area.description;
      }

      append_contributor_unique(merged.contributors, peer_source());

      // Merged areas do NOT go into routing table - they are combined local+remote
      continue;
    }

    // No collision: add remote area with source tagged
    Area added = remote_area;
    added.source = peer_source();
    append_contributor_unique(added.contributors, peer_source());

    if (held_by_other_type(remote_area.id, Kind::kArea)) {
      // Another type holds the ID locally; the routing entry below would take its requests.
      added.id = prefix_id(remote_area.id);
      if (local_index.count(added.id) > 0u || held_by_any_type(added.id)) {
        dropped_.push_back("Area '" + remote_area.id + "'");
        continue;
      }
      added.name = peer_name_ + SEPARATOR + remote_area.name;
      renames_.areas[remote_area.id] = added.id;
    }

    // Remote-only areas get a routing entry
    routing_table_[added.id] = peer_name_;
    result.push_back(added);
    added_index.push_back(result.size() - 1);
  }

  // A subarea names its parent by the ID the peer gave it.
  for (auto i : added_index) {
    apply_rename(result[i].parent_area_id, renames_.areas);
  }

  return result;
}

std::vector<Function> EntityMerger::merge_functions(const std::vector<Function> & local,
                                                    const std::vector<Function> & remote) {
  // Start with copies of all local functions
  std::vector<Function> result = local;

  // Build index of local function IDs for fast lookup
  std::unordered_map<std::string, size_t> local_index;
  for (size_t i = 0; i < result.size(); ++i) {
    local_index[result[i].id] = i;
  }

  std::vector<size_t> added_index;
  for (auto remote_func : remote) {
    // A host names the peer's own App or Component; a prefixed one is served under the prefixed ID. An App is
    // looked up first because that is what a Function host usually is.
    for (auto & host : remote_func.hosts) {
      auto app = renames_.apps.find(host);
      if (app != renames_.apps.end()) {
        host = app->second;
      } else {
        apply_rename(host, renames_.components);
      }
    }

    auto it = local_index.find(remote_func.id);
    if (it != local_index.end()) {
      // Collision: merge by combining hosts lists
      auto & merged = result[it->second];

      std::unordered_set<std::string> host_set(merged.hosts.begin(), merged.hosts.end());
      for (const auto & host : remote_func.hosts) {
        if (host_set.insert(host).second) {
          merged.hosts.push_back(host);
        }
      }

      // Merge tags without duplicates
      std::unordered_set<std::string> tag_set(merged.tags.begin(), merged.tags.end());
      for (const auto & tag : remote_func.tags) {
        if (tag_set.insert(tag).second) {
          merged.tags.push_back(tag);
        }
      }

      append_contributor_unique(merged.contributors, peer_source());

      // Merged functions do NOT go into routing table
      continue;
    }

    // No collision: add remote function with source tagged
    Function added = remote_func;
    added.source = peer_source();
    append_contributor_unique(added.contributors, peer_source());

    if (held_by_other_type(remote_func.id, Kind::kFunction)) {
      // Another type holds the ID locally; the routing entry below would take its requests.
      added.id = prefix_id(remote_func.id);
      if (local_index.count(added.id) > 0u || held_by_any_type(added.id)) {
        dropped_.push_back("Function '" + remote_func.id + "'");
        continue;
      }
      added.name = peer_name_ + SEPARATOR + remote_func.name;
      renames_.functions[remote_func.id] = added.id;
    }

    // Remote-only functions get a routing entry
    routing_table_[added.id] = peer_name_;
    result.push_back(added);
    added_index.push_back(result.size() - 1);
  }

  for (auto i : added_index) {
    apply_renames(result[i].depends_on, renames_.functions);
  }

  return result;
}

std::vector<Component> EntityMerger::merge_components(const std::vector<Component> & local,
                                                      const std::vector<Component> & remote) {
  // Start with copies of all local components
  std::vector<Component> result = local;

  // Build index of local component IDs for fast lookup
  std::unordered_map<std::string, size_t> local_index;
  for (size_t i = 0; i < result.size(); ++i) {
    local_index[result[i].id] = i;
  }

  std::vector<size_t> added_index;
  for (auto remote_comp : remote) {
    apply_rename(remote_comp.area, renames_.areas);

    // A merged Component is routed to the peer by its ID, and so would be a local entity of another type holding
    // that ID. Such a remote Component is prefixed like any other cross-type collision instead of merged.
    const bool held_elsewhere = held_by_other_type(remote_comp.id, Kind::kComponent);
    auto it = local_index.find(remote_comp.id);
    if (it != local_index.end() && !held_elsewhere) {
      // Collision: merge by combining tags and subcomponent relationships
      auto & merged = result[it->second];

      // Merge tags without duplicates
      std::unordered_set<std::string> tag_set(merged.tags.begin(), merged.tags.end());
      for (const auto & tag : remote_comp.tags) {
        if (tag_set.insert(tag).second) {
          merged.tags.push_back(tag);
        }
      }

      // If local has no description but remote does, take remote's
      if (merged.description.empty() && !remote_comp.description.empty()) {
        merged.description = remote_comp.description;
      }

      // Merge the remote asset-identity nameplate into the local one. The peer is
      // tagged as the lowest authority (peer:<name> is not in the default precedence),
      // so this is a gap-fill: empty local fields are populated from the remote, but a
      // value the local side already knows is never overridden. Stamp local provenance
      // first so existing local fields are protected during the merge.
      static const discovery::IdentityMergeConfig kIdentityConfig;
      discovery::stamp_identity_provenance(merged.identity, merged.source);
      discovery::merge_identity(merged.identity, remote_comp.identity, peer_source(), kIdentityConfig);

      // Note the deliberate divergence between the two attributions below: the
      // peer is the LEAST authoritative source for *identity* (gap-fill only,
      // above) yet the authoritative owner of *runtime state* (routing, below).
      // This is intentional and conservative: a remote peer must not clobber
      // local nameplate knowledge, but a Component ID refers to one physical ECU
      // and when the same ID is present locally and remotely the peer owns its
      // live state (data, logs, hosts, operations, faults). Route all requests
      // for the merged Component to the peer; the primary only aggregates its
      // presence in discovery listings.
      routing_table_[merged.id] = peer_name_;
      append_contributor_unique(merged.contributors, peer_source());
      continue;
    }

    // No collision: add remote component with source tagged
    Component added = remote_comp;
    added.source = peer_source();
    append_contributor_unique(added.contributors, peer_source());

    if (held_elsewhere) {
      added.id = prefix_id(remote_comp.id);
      if (local_index.count(added.id) > 0u || held_by_any_type(added.id)) {
        dropped_.push_back("Component '" + remote_comp.id + "'");
        continue;
      }
      added.name = peer_name_ + SEPARATOR + remote_comp.name;
      renames_.components[remote_comp.id] = added.id;
    }

    // Remote-only components get a routing entry
    routing_table_[added.id] = peer_name_;
    result.push_back(added);
    added_index.push_back(result.size() - 1);
  }

  for (auto i : added_index) {
    apply_rename(result[i].parent_component_id, renames_.components);
    apply_renames(result[i].depends_on, renames_.components);
  }

  return result;
}

std::vector<App> EntityMerger::merge_apps(const std::vector<App> & local, const std::vector<App> & remote) {
  // Start with copies of all local apps
  std::vector<App> result = local;

  // Build set of local app IDs
  std::unordered_set<std::string> local_ids;
  for (const auto & app : local) {
    local_ids.insert(app.id);
  }

  std::vector<size_t> added_index;
  for (const auto & remote_app : remote) {
    App added = remote_app;
    added.source = peer_source();
    // Preserve the original component_id from the remote peer. The remote component
    // is either merged with a local component (same ID) or added as a new entry -
    // in both cases the component_id remains valid, unless the Component was
    // prefixed, in which case the App follows it. Store the peer origin in
    // x-medkit metadata via the source field for traceability.
    apply_rename(added.component_id, renames_.components);

    if (local_ids.count(remote_app.id) > 0u || held_by_other_type(remote_app.id, Kind::kApp)) {
      // Collision: prefix the remote entity ID, preserve original for clients
      added.original_id = remote_app.id;
      added.id = prefix_id(remote_app.id);
      if (local_ids.count(added.id) > 0u || held_by_any_type(added.id)) {
        dropped_.push_back("App '" + remote_app.id + "'");
        continue;
      }
      added.name = peer_name_ + SEPARATOR + remote_app.name;
      renames_.apps[remote_app.id] = added.id;
    }

    append_contributor_unique(added.contributors, peer_source());
    routing_table_[added.id] = peer_name_;
    result.push_back(added);
    added_index.push_back(result.size() - 1);
  }

  for (auto i : added_index) {
    apply_renames(result[i].depends_on, renames_.apps);
  }

  return result;
}

}  // namespace ros2_medkit_gateway
