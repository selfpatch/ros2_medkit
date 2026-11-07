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

#include "ros2_medkit_gateway/core/aggregation/peer_id_index.hpp"

#include <string_view>

#include "ros2_medkit_gateway/core/aggregation/peer_ids.hpp"
#include "ros2_medkit_gateway/core/discovery/models/common.hpp"

namespace ros2_medkit_gateway {

namespace {

const PeerIdIndex::Ids & empty_ids() {
  static const PeerIdIndex::Ids kEmpty;
  return kEmpty;
}

/// Record every entity of `entities` the merge renamed, under the peer that
/// contributed it.
template <class Entity, class Member>
void add_renames(PeerIdIndex & index, const std::vector<Entity> & entities, Member member) {
  static constexpr std::string_view kPeerPrefix = "peer:";
  for (const auto & entity : entities) {
    if (entity.original_id.empty() || entity.source.size() <= kPeerPrefix.size() ||
        entity.source.compare(0, kPeerPrefix.size(), kPeerPrefix) != 0) {
      continue;
    }
    auto & ids = index.peers[entity.source.substr(kPeerPrefix.size())].*member;
    ids.renamed[entity.original_id] = PeerIdIndex::Renamed{entity.id, entity.name.empty() ? entity.id : entity.name};
  }
}

/// Record every ID a peer contributes to an entity of `entities` under, as that
/// peer knows it (peer_targets_of).
template <class Entity, class Member>
void add_admitted(PeerIdIndex & index, const std::vector<Entity> & entities, Member member) {
  for (const auto & entity : entities) {
    for (auto & target : peer_targets_of(entity)) {
      (index.peers[target.peer].*member).admitted.insert(std::move(target.id));
    }
  }
}

/// Record the provenance of every entity of `entities` a peer contributes, by
/// the ID it is served under.
template <class Entity>
void add_provenance(std::unordered_map<std::string, PeerIdIndex::Provenance> & provenance,
                    const std::vector<Entity> & entities) {
  for (const auto & entity : entities) {
    if (!peer_targets_of(entity).empty()) {
      provenance[entity.id] = PeerIdIndex::Provenance{entity.source, sorted_contributors(entity.contributors)};
    }
  }
}

const PeerIdIndex::Provenance *
find_provenance(const std::unordered_map<std::string, PeerIdIndex::Provenance> & of_type,
                const std::string & served_id) {
  auto it = of_type.find(served_id);
  return it != of_type.end() ? &it->second : nullptr;
}

}  // namespace

const PeerIdIndex::Ids & PeerIdIndex::Peer::of(SovdEntityType type) const {
  switch (type) {
    case SovdEntityType::AREA:
      return areas;
    case SovdEntityType::COMPONENT:
      return components;
    case SovdEntityType::APP:
      return apps;
    case SovdEntityType::FUNCTION:
      return functions;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      break;
  }
  return empty_ids();
}

const PeerIdIndex::Peer & PeerIdIndex::peer(const std::string & name) const {
  static const Peer kEmpty;
  auto it = peers.find(name);
  return it != peers.end() ? it->second : kEmpty;
}

const PeerIdIndex::Provenance * PeerIdIndex::provenance_of(SovdEntityType type, const std::string & served_id) const {
  switch (type) {
    case SovdEntityType::AREA:
      return find_provenance(provenance.areas, served_id);
    case SovdEntityType::COMPONENT:
      return find_provenance(provenance.components, served_id);
    case SovdEntityType::APP:
      return find_provenance(provenance.apps, served_id);
    case SovdEntityType::FUNCTION:
      return find_provenance(provenance.functions, served_id);
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      break;
  }
  return nullptr;
}

PeerIdIndex make_peer_id_index(const std::vector<Area> & areas, const std::vector<Component> & components,
                               const std::vector<App> & apps, const std::vector<Function> & functions,
                               const std::unordered_map<std::string, PeerDrops> & drops) {
  PeerIdIndex index;
  add_renames(index, areas, &PeerIdIndex::Peer::areas);
  add_renames(index, components, &PeerIdIndex::Peer::components);
  add_renames(index, apps, &PeerIdIndex::Peer::apps);
  add_renames(index, functions, &PeerIdIndex::Peer::functions);
  add_admitted(index, areas, &PeerIdIndex::Peer::areas);
  add_admitted(index, components, &PeerIdIndex::Peer::components);
  add_admitted(index, apps, &PeerIdIndex::Peer::apps);
  add_admitted(index, functions, &PeerIdIndex::Peer::functions);
  add_provenance(index.provenance.areas, areas);
  add_provenance(index.provenance.components, components);
  add_provenance(index.provenance.apps, apps);
  add_provenance(index.provenance.functions, functions);
  for (const auto & [peer, dropped] : drops) {
    auto & entry = index.peers[peer];
    entry.areas.dropped.insert(dropped.areas.begin(), dropped.areas.end());
    entry.components.dropped.insert(dropped.components.begin(), dropped.components.end());
    entry.apps.dropped.insert(dropped.apps.begin(), dropped.apps.end());
    entry.functions.dropped.insert(dropped.functions.begin(), dropped.functions.end());
  }
  return index;
}

}  // namespace ros2_medkit_gateway
