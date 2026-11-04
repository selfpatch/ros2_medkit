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

#include <algorithm>
#include <unordered_set>
#include <utility>

#include "ros2_medkit_gateway/core/discovery/identity_merge.hpp"

namespace ros2_medkit_gateway {

EntityMerger::EntityMerger(const std::string & peer_name) : peer_name_(peer_name) {
}

EntityMerger::EntityMerger(const std::string & peer_name, LocalIds local_ids, std::optional<LocalIds> peer_ids)
  : peer_name_(peer_name)
  , local_ids_(std::move(local_ids))
  , peer_ids_(peer_ids ? std::move(*peer_ids) : LocalIds{})
  , checks_references_(peer_ids.has_value()) {
}

const std::unordered_set<std::string> * EntityMerger::declared(const std::unordered_set<std::string> & ids) const {
  return checks_references_ ? &ids : nullptr;
}

EntityMerger::LocalIds EntityMerger::collect_ids(const std::vector<Area> & areas,
                                                 const std::vector<Component> & components,
                                                 const std::vector<App> & apps,
                                                 const std::vector<Function> & functions) {
  LocalIds ids;
  auto collect = [&ids](const auto & entities, std::unordered_set<std::string> & out) {
    for (const auto & entity : entities) {
      out.insert(entity.id);
      if (!entity.original_id.empty()) {
        ids.generated.insert(entity.id);
      }
    }
  };
  collect(areas, ids.areas);
  collect(components, ids.components);
  collect(apps, ids.apps);
  collect(functions, ids.functions);
  return ids;
}

std::string EntityMerger::prefix_id(const std::string & id) const {
  return peer_name_ + SEPARATOR + id;
}

std::string EntityMerger::prefix_name(const std::string & name) const {
  std::string served = peer_name_ + SEPARATOR + name;
  if (served.size() > kMaxServedNameBytes) {
    // Back off to the start of the character the cut would split: a byte
    // 10xxxxxx continues one, and a name ending inside a character is not
    // UTF-8, which the JSON writer refuses.
    size_t cut = kMaxServedNameBytes;
    while (cut > 0 && (static_cast<unsigned char>(served[cut]) & 0xC0u) == 0x80u) {
      --cut;
    }
    served.resize(cut);
  }
  return served;
}

std::string EntityMerger::peer_source() const {
  return "peer:" + peer_name_;
}

bool EntityMerger::held_by_other_type(const std::string & id, Kind kind) const {
  auto held = [&id, kind](const LocalIds & ids) {
    return (kind != Kind::kArea && ids.areas.count(id) > 0u) ||
           (kind != Kind::kComponent && ids.components.count(id) > 0u) ||
           (kind != Kind::kApp && ids.apps.count(id) > 0u) || (kind != Kind::kFunction && ids.functions.count(id) > 0u);
  };
  return held(local_ids_) || held(added_);
}

bool EntityMerger::held_by_any_type(const std::string & id) const {
  return local_ids_.areas.count(id) > 0u || local_ids_.components.count(id) > 0u || local_ids_.apps.count(id) > 0u ||
         local_ids_.functions.count(id) > 0u;
}

bool EntityMerger::can_generate(const std::string & id, Kind kind, const std::string & original_id) const {
  if (held_by_any_type(id)) {
    return false;
  }
  if (peer_ids_.areas.count(id) > 0u || peer_ids_.components.count(id) > 0u || peer_ids_.apps.count(id) > 0u ||
      peer_ids_.functions.count(id) > 0u) {
    return false;
  }
  auto it = generated_.find(id);
  return it == generated_.end() || (it->second.kind == kind && it->second.original_id == original_id);
}

void EntityMerger::record_generated(const std::string & id, Kind kind, const std::string & original_id) {
  generated_[id] = GeneratedFor{kind, original_id};
}

void EntityMerger::drop(Kind kind, const std::string & original_id, const std::string & taken_id) {
  const char * type_name = "Area";
  std::unordered_set<std::string> * drops = &drops_.areas;
  switch (kind) {
    case Kind::kArea:
      break;
    case Kind::kComponent:
      type_name = "Component";
      drops = &drops_.components;
      break;
    case Kind::kApp:
      type_name = "App";
      drops = &drops_.apps;
      break;
    case Kind::kFunction:
      type_name = "Function";
      drops = &drops_.functions;
      break;
  }
  drops->insert(original_id);
  dropped_.push_back(std::string(type_name) + " '" + original_id + "' (its prefixed ID '" + taken_id + "' is taken)");
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

/// True when the reference `id` names nothing the peer may name here: an
/// entity the merge dropped, whose ID names a local entity here, or, when the
/// peer's IDs of that type are known (`declared`), one the peer never declared.
bool unresolvable(const std::string & id, const std::unordered_set<std::string> & drops,
                  const std::unordered_set<std::string> * declared) {
  return drops.count(id) > 0u || (declared != nullptr && declared->count(id) == 0u);
}

/// A reference the peer made by its own ID, as served here: the prefixed ID of
/// a renamed entity, and nothing for one it may not name (unresolvable).
void resolve_reference(std::string & id, const std::unordered_map<std::string, std::string> & renames,
                       const std::unordered_set<std::string> & drops,
                       const std::unordered_set<std::string> * declared) {
  if (id.empty()) {
    return;
  }
  if (unresolvable(id, drops, declared)) {
    id.clear();
    return;
  }
  apply_rename(id, renames);
}

/// resolve_reference over a list; the entries it may not name are removed.
void resolve_references(std::vector<std::string> & ids, const std::unordered_map<std::string, std::string> & renames,
                        const std::unordered_set<std::string> & drops,
                        const std::unordered_set<std::string> * declared) {
  ids.erase(std::remove_if(ids.begin(), ids.end(),
                           [&drops, declared](const std::string & id) {
                             return unresolvable(id, drops, declared);
                           }),
            ids.end());
  for (auto & id : ids) {
    apply_rename(id, renames);
  }
}
}  // namespace

void EntityMerger::resolve_hosts(std::vector<std::string> & hosts) const {
  std::vector<std::string> resolved;
  resolved.reserve(hosts.size());
  for (const auto & host : hosts) {
    // The peer's App when it declares one by that ID - what a host usually is -
    // and its Component otherwise. Without the peer's declaration, an App this
    // merger renamed or dropped by that ID counts as declared.
    const bool app = peer_ids_.apps.count(host) > 0u || renames_.apps.count(host) > 0u || drops_.apps.count(host) > 0u;
    const auto & renames = app ? renames_.apps : renames_.components;
    const auto & drops = app ? drops_.apps : drops_.components;
    if (drops.count(host) > 0u) {
      continue;
    }
    // Neither: with the peer's IDs known, the host names nothing of the peer's,
    // and here it would name whatever holds the ID.
    if (checks_references_ && !app && peer_ids_.components.count(host) == 0u && renames_.components.count(host) == 0u) {
      continue;
    }
    auto it = renames.find(host);
    resolved.push_back(it != renames.end() ? it->second : host);
  }
  hosts = std::move(resolved);
}

const std::unordered_map<std::string, std::string> & EntityMerger::get_routing_table() const {
  return routing_table_;
}

const EntityMerger::Renames & EntityMerger::get_renames() const {
  return renames_;
}

const std::vector<std::string> & EntityMerger::get_dropped() const {
  return dropped_;
}

const EntityMerger::LocalIds & EntityMerger::get_drops() const {
  return drops_;
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
    // An ID a merge generated names the entity it was generated for, not one
    // this peer declares: a peer Area under it is another Area, prefixed below.
    if (it != local_index.end() && local_ids_.generated.count(remote_area.id) == 0u) {
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

    if (it != local_index.end() || held_by_other_type(remote_area.id, Kind::kArea)) {
      // Another entity holds the ID here; serving this one under it would put both under one ID.
      added.id = prefix_id(remote_area.id);
      if (local_index.count(added.id) > 0u || !can_generate(added.id, Kind::kArea, remote_area.id)) {
        drop(Kind::kArea, remote_area.id, added.id);
        continue;
      }
      added.name = prefix_name(remote_area.name);
      added.original_id = remote_area.id;
      renames_.areas[remote_area.id] = added.id;
      record_generated(added.id, Kind::kArea, remote_area.id);
    }

    // Remote-only areas get a routing entry
    routing_table_[added.id] = peer_name_;
    added_.areas.insert(added.id);
    result.push_back(added);
    added_index.push_back(result.size() - 1);
  }

  // A subarea names its parent by the ID the peer gave it.
  for (auto i : added_index) {
    resolve_reference(result[i].parent_area_id, renames_.areas, drops_.areas, declared(peer_ids_.areas));
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

  // Entries holding dependencies read from the peer.
  std::vector<size_t> renamed_refs_index;
  for (auto remote_func : remote) {
    // A host names the peer's own App or Component; a prefixed one is served under the prefixed ID, a dropped one
    // is not a host here.
    resolve_hosts(remote_func.hosts);

    auto it = local_index.find(remote_func.id);
    // An ID a merge generated is never a merge target; see merge_areas.
    if (it != local_index.end() && local_ids_.generated.count(remote_func.id) == 0u) {
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

      // A local declaration that names no dependencies takes the peer's. One
      // that names some keeps them.
      if (merged.depends_on.empty() && !remote_func.depends_on.empty()) {
        merged.depends_on = remote_func.depends_on;
        renamed_refs_index.push_back(it->second);
      }

      append_contributor_unique(merged.contributors, peer_source());

      // Merged functions do NOT go into routing table
      continue;
    }

    // No collision: add remote function with source tagged
    Function added = remote_func;
    added.source = peer_source();
    append_contributor_unique(added.contributors, peer_source());

    if (it != local_index.end() || held_by_other_type(remote_func.id, Kind::kFunction)) {
      // Another entity holds the ID here; serving this one under it would put both under one ID.
      added.id = prefix_id(remote_func.id);
      if (local_index.count(added.id) > 0u || !can_generate(added.id, Kind::kFunction, remote_func.id)) {
        drop(Kind::kFunction, remote_func.id, added.id);
        continue;
      }
      added.name = prefix_name(remote_func.name);
      added.original_id = remote_func.id;
      renames_.functions[remote_func.id] = added.id;
      record_generated(added.id, Kind::kFunction, remote_func.id);
    }

    // Remote-only functions get a routing entry
    routing_table_[added.id] = peer_name_;
    added_.functions.insert(added.id);
    result.push_back(added);
    renamed_refs_index.push_back(result.size() - 1);
  }

  // Dependencies the peer named, by the IDs the Functions are served under.
  for (auto i : renamed_refs_index) {
    resolve_references(result[i].depends_on, renames_.functions, drops_.functions, declared(peer_ids_.functions));
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
    resolve_reference(remote_comp.area, renames_.areas, drops_.areas, declared(peer_ids_.areas));

    // A merged Component is routed to the peer by its ID, and so would be a local entity of another type holding
    // that ID. Such a remote Component is prefixed like any other cross-type collision instead of merged. An ID a
    // merge generated is never a merge target; see merge_areas.
    const bool held_elsewhere = held_by_other_type(remote_comp.id, Kind::kComponent);
    auto it = local_index.find(remote_comp.id);
    if (it != local_index.end() && !held_elsewhere && local_ids_.generated.count(remote_comp.id) == 0u) {
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

      // The same for the Area: a local declaration that names none takes the
      // peer's, so the Area listings place the Component where its detail does.
      // An Area the local side declared is never overridden.
      if (merged.area.empty() && !remote_comp.area.empty()) {
        merged.area = remote_comp.area;
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

    if (it != local_index.end() || held_elsewhere) {
      added.id = prefix_id(remote_comp.id);
      if (local_index.count(added.id) > 0u || !can_generate(added.id, Kind::kComponent, remote_comp.id)) {
        drop(Kind::kComponent, remote_comp.id, added.id);
        continue;
      }
      added.name = prefix_name(remote_comp.name);
      added.original_id = remote_comp.id;
      renames_.components[remote_comp.id] = added.id;
      record_generated(added.id, Kind::kComponent, remote_comp.id);
    }

    // Remote-only components get a routing entry
    routing_table_[added.id] = peer_name_;
    added_.components.insert(added.id);
    result.push_back(added);
    added_index.push_back(result.size() - 1);
  }

  for (auto i : added_index) {
    resolve_reference(result[i].parent_component_id, renames_.components, drops_.components,
                      declared(peer_ids_.components));
    resolve_references(result[i].depends_on, renames_.components, drops_.components, declared(peer_ids_.components));
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
    resolve_reference(added.component_id, renames_.components, drops_.components, declared(peer_ids_.components));

    if (local_ids.count(remote_app.id) > 0u || held_by_other_type(remote_app.id, Kind::kApp)) {
      // Collision: prefix the remote entity ID, preserve original for clients
      added.original_id = remote_app.id;
      added.id = prefix_id(remote_app.id);
      if (local_ids.count(added.id) > 0u || !can_generate(added.id, Kind::kApp, remote_app.id)) {
        drop(Kind::kApp, remote_app.id, added.id);
        continue;
      }
      added.name = prefix_name(remote_app.name);
      renames_.apps[remote_app.id] = added.id;
      record_generated(added.id, Kind::kApp, remote_app.id);
    }

    append_contributor_unique(added.contributors, peer_source());
    routing_table_[added.id] = peer_name_;
    added_.apps.insert(added.id);
    result.push_back(added);
    added_index.push_back(result.size() - 1);
  }

  for (auto i : added_index) {
    resolve_references(result[i].depends_on, renames_.apps, drops_.apps, declared(peer_ids_.apps));
  }

  return result;
}

}  // namespace ros2_medkit_gateway
