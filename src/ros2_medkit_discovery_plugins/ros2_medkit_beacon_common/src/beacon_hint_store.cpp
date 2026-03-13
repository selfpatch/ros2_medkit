// Copyright 2026 selfpatch GmbH
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

#include "ros2_medkit_beacon_common/beacon_hint_store.hpp"

#include <algorithm>

namespace ros2_medkit_beacon {

BeaconHintStore::BeaconHintStore() : BeaconHintStore(Config{}) {
}

BeaconHintStore::BeaconHintStore(Config config) : config_(config) {
  // Auto-correct invalid expiry: expiry must be strictly greater than TTL.
  if (config_.beacon_expiry_sec <= config_.beacon_ttl_sec) {
    config_.beacon_expiry_sec = 10.0 * config_.beacon_ttl_sec;
  }
}

bool BeaconHintStore::update(const BeaconHint & hint) {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  auto it = hints_.find(hint.entity_id);
  if (it != hints_.end()) {
    // Refresh existing hint: update last_seen and overwrite non-empty fields.
    StoredHint & stored = it->second;
    stored.last_seen = std::chrono::steady_clock::now();

    // Also update received_at if the incoming hint carries a more recent timestamp.
    if (hint.received_at != std::chrono::steady_clock::time_point{}) {
      stored.hint.received_at = hint.received_at;
    }

    // Overwrite non-empty string fields.
    if (!hint.stable_id.empty()) {
      stored.hint.stable_id = hint.stable_id;
    }
    if (!hint.display_name.empty()) {
      stored.hint.display_name = hint.display_name;
    }
    if (!hint.component_id.empty()) {
      stored.hint.component_id = hint.component_id;
    }
    if (!hint.transport_type.empty()) {
      stored.hint.transport_type = hint.transport_type;
    }
    if (!hint.negotiated_format.empty()) {
      stored.hint.negotiated_format = hint.negotiated_format;
    }
    if (!hint.process_name.empty()) {
      stored.hint.process_name = hint.process_name;
    }
    if (!hint.hostname.empty()) {
      stored.hint.hostname = hint.hostname;
    }
    if (hint.process_id != 0) {
      stored.hint.process_id = hint.process_id;
    }

    // Overwrite non-empty vector fields.
    if (!hint.function_ids.empty()) {
      stored.hint.function_ids = hint.function_ids;
    }
    if (!hint.depends_on.empty()) {
      stored.hint.depends_on = hint.depends_on;
    }

    // Merge metadata (new keys take precedence).
    for (const auto & kv : hint.metadata) {
      stored.hint.metadata[kv.first] = kv.second;
    }

    // Recompute status.
    stored.status = compute_status(stored);
    return true;
  }

  // New entity - check capacity.
  if (hints_.size() >= config_.max_hints) {
    return false;
  }

  // Insert new entry.
  StoredHint stored;
  stored.hint = hint;
  stored.last_seen = std::chrono::steady_clock::now();

  // If received_at was not set by caller, use now as a sensible default.
  if (stored.hint.received_at == std::chrono::steady_clock::time_point{}) {
    stored.hint.received_at = stored.last_seen;
  }

  stored.status = compute_status(stored);
  hints_.emplace(hint.entity_id, std::move(stored));
  return true;
}

std::vector<BeaconHintStore::StoredHint> BeaconHintStore::evict_and_snapshot() {
  std::unique_lock<std::shared_mutex> lock(mutex_);

  std::vector<StoredHint> result;
  result.reserve(hints_.size());

  auto it = hints_.begin();
  while (it != hints_.end()) {
    StoredHint & stored = it->second;
    stored.status = compute_status(stored);

    if (stored.status == HintStatus::EXPIRED) {
      it = hints_.erase(it);
    } else {
      result.push_back(stored);
      ++it;
    }
  }

  return result;
}

std::optional<BeaconHintStore::StoredHint> BeaconHintStore::get(const std::string & entity_id) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);

  auto it = hints_.find(entity_id);
  if (it == hints_.end()) {
    return std::nullopt;
  }

  StoredHint copy = it->second;
  // Compute status at read time (do not mutate stored entry under shared lock).
  copy.status = compute_status(copy);
  return copy;
}

size_t BeaconHintStore::size() const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return hints_.size();
}

BeaconHintStore::HintStatus BeaconHintStore::compute_status(const StoredHint & stored) const {
  auto now = std::chrono::steady_clock::now();
  auto age = std::chrono::duration<double>(now - stored.last_seen).count();

  if (age >= config_.beacon_expiry_sec) {
    return HintStatus::EXPIRED;
  }
  if (age >= config_.beacon_ttl_sec) {
    return HintStatus::STALE;
  }
  return HintStatus::ACTIVE;
}

}  // namespace ros2_medkit_beacon
