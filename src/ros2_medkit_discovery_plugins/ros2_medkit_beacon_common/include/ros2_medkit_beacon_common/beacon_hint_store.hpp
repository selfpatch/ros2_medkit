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

#pragma once

#include <chrono>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_beacon_common/beacon_types.hpp"

namespace ros2_medkit_beacon {

class BeaconHintStore {
 public:
  struct Config {
    double beacon_ttl_sec{10.0};
    double beacon_expiry_sec{300.0};
    bool check_process_alive{true};
    size_t max_hints{10000};
  };

  enum class HintStatus { ACTIVE, STALE, EXPIRED };

  struct StoredHint {
    BeaconHint hint;
    HintStatus status;
    std::chrono::steady_clock::time_point last_seen;
  };

  BeaconHintStore();
  explicit BeaconHintStore(Config config);

  /// Insert or refresh a hint. Returns false if capacity full for new entity_ids.
  bool update(const BeaconHint & hint);

  /// Atomic: evict expired hints, then return consistent snapshot of remaining hints.
  std::vector<StoredHint> evict_and_snapshot();

  /// Get a single hint by entity_id (thread-safe read).
  std::optional<StoredHint> get(const std::string & entity_id) const;

  /// Current number of stored hints.
  size_t size() const;

 private:
  HintStatus compute_status(const StoredHint & stored) const;

  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, StoredHint> hints_;
  Config config_;
};

}  // namespace ros2_medkit_beacon
