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

#pragma once

#include <algorithm>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// One stored entity freeze-frame, keyed by (fault_code, owner, entity_id).
/// `frame` holds what has no column of its own, so a reload reproduces the frame exactly.
struct StoredEntityFreezeFrame {
  std::string fault_code;
  /// Owner of the record the frame belongs to: the reporting source of a per_source
  /// record, "*" for a shared one.
  std::string owner;
  std::string entity_id;
  nlohmann::json frame;  ///< {"values": {...}, "connected"?: bool, "source_timestamp"?: any}
  int64_t captured_at_ns{0};
  std::string source;          ///< capture path that read the values
  std::string capture_origin;  ///< "startup" for a catch-up frame, empty for a confirm edge
  /// The fault manager's occurrence of the record at capture. 0 and 0 mean "not known".
  uint32_t occurrence_count{0};
  int64_t first_occurred_ns{0};
};

/// Persistence of the entity freeze-frames, so a frame outlives the gateway process.
/// Writes replace all rows of one record, as the capture replaces the record's frames.
/// Implementations must be thread-safe and must not throw: they report a failure through
/// the returned `tl::expected`.
class EntityFreezeFrameStore {
 public:
  EntityFreezeFrameStore() = default;
  EntityFreezeFrameStore(const EntityFreezeFrameStore &) = delete;
  EntityFreezeFrameStore & operator=(const EntityFreezeFrameStore &) = delete;
  EntityFreezeFrameStore(EntityFreezeFrameStore &&) = delete;
  EntityFreezeFrameStore & operator=(EntityFreezeFrameStore &&) = delete;
  virtual ~EntityFreezeFrameStore() = default;

  /// Replaces every row of the record (@p fault_code, @p owner) with @p frames. An empty
  /// vector leaves no rows. The rows of the code's other records stay.
  [[nodiscard]] virtual tl::expected<void, std::string>
  replace_frames(const std::string & fault_code, const std::string & owner,
                 const std::vector<StoredEntityFreezeFrame> & frames) = 0;

  /// Deletes every row of the record (@p fault_code, @p owner). A record with no rows is
  /// not an error.
  [[nodiscard]] virtual tl::expected<void, std::string> erase_frames(const std::string & fault_code,
                                                                     const std::string & owner) = 0;

  /// Every row, oldest capture first, then by fault code, owner and entity id.
  [[nodiscard]] virtual tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() = 0;
};

/// The store contract in memory: rows end with the object. A host that wants no persistence passes no store.
class InMemoryEntityFreezeFrameStore : public EntityFreezeFrameStore {
 public:
  tl::expected<void, std::string> replace_frames(const std::string & fault_code, const std::string & owner,
                                                 const std::vector<StoredEntityFreezeFrame> & frames) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames.empty()) {
      rows_.erase({fault_code, owner});
      return {};
    }
    rows_[{fault_code, owner}] = frames;
    return {};
  }

  tl::expected<void, std::string> erase_frames(const std::string & fault_code, const std::string & owner) override {
    std::lock_guard<std::mutex> lock(mutex_);
    rows_.erase({fault_code, owner});
    return {};
  }

  tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<StoredEntityFreezeFrame> all;
    for (const auto & entry : rows_) {
      all.insert(all.end(), entry.second.begin(), entry.second.end());
    }
    // The order of SqliteEntityFreezeFrameStore::load_all.
    std::stable_sort(all.begin(), all.end(), [](const StoredEntityFreezeFrame & a, const StoredEntityFreezeFrame & b) {
      if (a.captured_at_ns != b.captured_at_ns) {
        return a.captured_at_ns < b.captured_at_ns;
      }
      if (a.fault_code != b.fault_code) {
        return a.fault_code < b.fault_code;
      }
      if (a.owner != b.owner) {
        return a.owner < b.owner;
      }
      return a.entity_id < b.entity_id;
    });
    return all;
  }

 private:
  std::mutex mutex_;
  /// Keyed by record: (fault_code, owner).
  std::map<std::pair<std::string, std::string>, std::vector<StoredEntityFreezeFrame>> rows_;
};

}  // namespace ros2_medkit_gateway
