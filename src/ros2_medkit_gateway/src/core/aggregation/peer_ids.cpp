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

#include "ros2_medkit_gateway/core/aggregation/peer_ids.hpp"

#include "ros2_medkit_gateway/core/aggregation/entity_merger.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

namespace ros2_medkit_gateway {

namespace {

/// True when `entity` is the one the merge renamed from `peer_id` for `peer`.
template <class Entity>
bool renamed_from(const Entity & entity, const std::string & peer, const std::string & peer_id) {
  return entity.original_id == peer_id && entity.source == "peer:" + peer;
}

}  // namespace

std::string served_entity_id(const ThreadSafeEntityCache & cache, SovdEntityType type, const std::string & peer,
                             const std::string & peer_id) {
  if (peer.empty() || peer_id.empty()) {
    return peer_id;
  }
  // The merge generates exactly this ID for a renamed entity, or drops the
  // entity when the ID is taken, so there is one candidate to check.
  std::string candidate = peer + EntityMerger::SEPARATOR + peer_id;
  bool renamed = false;
  switch (type) {
    case SovdEntityType::AREA:
      if (auto area = cache.get_area(candidate)) {
        renamed = renamed_from(*area, peer, peer_id);
      }
      break;
    case SovdEntityType::COMPONENT:
      if (auto comp = cache.get_component(candidate)) {
        renamed = renamed_from(*comp, peer, peer_id);
      }
      break;
    case SovdEntityType::APP:
      if (auto app = cache.get_app(candidate)) {
        renamed = renamed_from(*app, peer, peer_id);
      }
      break;
    case SovdEntityType::FUNCTION:
      if (auto func = cache.get_function(candidate)) {
        renamed = renamed_from(*func, peer, peer_id);
      }
      break;
    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
      break;
  }
  return renamed ? candidate : peer_id;
}

std::string served_member_id(const ThreadSafeEntityCache & cache, const std::string & peer,
                             const std::string & member_id) {
  auto as_app = served_entity_id(cache, SovdEntityType::APP, peer, member_id);
  if (as_app != member_id) {
    return as_app;
  }
  return served_entity_id(cache, SovdEntityType::COMPONENT, peer, member_id);
}

}  // namespace ros2_medkit_gateway
