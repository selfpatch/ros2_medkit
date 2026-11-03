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

PeerIdIndex make_peer_id_index(const std::vector<Area> & areas, const std::vector<Component> & components,
                               const std::vector<App> & apps, const std::vector<Function> & functions,
                               const std::unordered_map<std::string, PeerDrops> & drops) {
  PeerIdIndex index;
  add_renames(index, areas, &PeerIdIndex::Peer::areas);
  add_renames(index, components, &PeerIdIndex::Peer::components);
  add_renames(index, apps, &PeerIdIndex::Peer::apps);
  add_renames(index, functions, &PeerIdIndex::Peer::functions);
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
