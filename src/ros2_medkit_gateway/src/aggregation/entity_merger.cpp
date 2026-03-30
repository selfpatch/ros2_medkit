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

#include "ros2_medkit_gateway/aggregation/entity_merger.hpp"

#include <unordered_set>

namespace ros2_medkit_gateway {

EntityMerger::EntityMerger(const std::string & peer_name) : peer_name_(peer_name) {
}

std::string EntityMerger::prefix_id(const std::string & id) const {
  return peer_name_ + SEPARATOR + id;
}

std::string EntityMerger::peer_source() const {
  return "peer:" + peer_name_;
}

const std::unordered_map<std::string, std::string> & EntityMerger::get_routing_table() const {
  return routing_table_;
}

std::vector<Area> EntityMerger::merge_areas(const std::vector<Area> & local, const std::vector<Area> & remote) {
  // Start with copies of all local areas
  std::vector<Area> result = local;

  // Build index of local area IDs for fast lookup
  std::unordered_map<std::string, size_t> local_index;
  for (size_t i = 0; i < result.size(); ++i) {
    local_index[result[i].id] = i;
  }

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

      // Merged areas do NOT go into routing table - they are combined local+remote
    } else {
      // No collision: add remote area with source tagged
      Area added = remote_area;
      added.source = peer_source();
      result.push_back(added);

      // Remote-only areas get a routing entry
      routing_table_[added.id] = peer_name_;
    }
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

  for (const auto & remote_func : remote) {
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

      // Merged functions do NOT go into routing table
    } else {
      // No collision: add remote function with source tagged
      Function added = remote_func;
      added.source = peer_source();
      result.push_back(added);

      // Remote-only functions get a routing entry
      routing_table_[added.id] = peer_name_;
    }
  }

  return result;
}

std::vector<Component> EntityMerger::merge_components(const std::vector<Component> & local,
                                                      const std::vector<Component> & remote) {
  // Start with copies of all local components
  std::vector<Component> result = local;

  // Build set of local component IDs
  std::unordered_set<std::string> local_ids;
  for (const auto & comp : local) {
    local_ids.insert(comp.id);
  }

  for (const auto & remote_comp : remote) {
    Component added = remote_comp;
    added.source = peer_source();

    if (local_ids.count(remote_comp.id) > 0) {
      // Collision: prefix the remote entity ID
      added.id = prefix_id(remote_comp.id);
      added.name = peer_name_ + SEPARATOR + remote_comp.name;
    }

    routing_table_[added.id] = peer_name_;
    result.push_back(added);
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

  for (const auto & remote_app : remote) {
    App added = remote_app;
    added.source = peer_source();

    if (local_ids.count(remote_app.id) > 0) {
      // Collision: prefix the remote entity ID
      added.id = prefix_id(remote_app.id);
      added.name = peer_name_ + SEPARATOR + remote_app.name;
    }

    routing_table_[added.id] = peer_name_;
    result.push_back(added);
  }

  return result;
}

}  // namespace ros2_medkit_gateway
