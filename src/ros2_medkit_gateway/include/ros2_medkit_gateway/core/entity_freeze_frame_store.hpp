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
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// One persisted entity freeze-frame: the row shape of the store, keyed by
/// (fault_code, entity_id).
///
/// `frame` holds everything that is not already a column of its own - the
/// compact {resource_id: value} dict under "values", plus the payload
/// provenance the capture recorded ("connected", "source_timestamp") when the
/// plugin reported it. Keeping those in the blob rather than in columns is
/// what lets a reloaded frame be served byte for byte as it was captured.
struct StoredEntityFreezeFrame {
  std::string fault_code;
  std::string entity_id;
  nlohmann::json frame;  ///< {"values": {...}, "connected"?: bool, "source_timestamp"?: any}
  int64_t captured_at_ns{0};
  std::string source;          ///< capture path that read the values
  std::string capture_origin;  ///< "startup" for a catch-up frame, empty on a confirm edge
};

/// Persistence for the gateway's entity freeze-frames.
///
/// The gateway's frames are process memory, so a restart re-derives them from
/// whatever the plant reads *now* - the values at fault time are gone and the
/// re-read is stamped with the restart. This store is what makes the captured
/// frame outlive the process.
///
/// Writes are per fault code and wholesale: a re-confirm replaces every row
/// for that code, mirroring the in-memory map, whose entry for a code is
/// likewise replaced as a unit. Implementations must be thread-safe.
class EntityFreezeFrameStore {
 public:
  virtual ~EntityFreezeFrameStore() = default;

  /// Replace every row for @p fault_code with @p frames (one row per entity).
  /// An empty vector leaves no rows for the code.
  virtual tl::expected<void, std::string> replace_frames(const std::string & fault_code,
                                                         const std::vector<StoredEntityFreezeFrame> & frames) = 0;

  /// Drop every row for @p fault_code. Removing a code that has no rows is
  /// not an error: the caller evicts by code and does not track what is on disk.
  virtual tl::expected<void, std::string> erase_frames(const std::string & fault_code) = 0;

  /// Every row, oldest capture first. The order is what lets a caller honour a
  /// retained-frame bound by keeping the newest codes.
  virtual tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() = 0;
};

/// In-memory backend: the store contract without a file, for tests and for
/// callers that want the interface without persistence.
class InMemoryEntityFreezeFrameStore : public EntityFreezeFrameStore {
 public:
  tl::expected<void, std::string> replace_frames(const std::string & fault_code,
                                                 const std::vector<StoredEntityFreezeFrame> & frames) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frames.empty()) {
      rows_.erase(fault_code);
      return {};
    }
    rows_[fault_code] = frames;
    return {};
  }

  tl::expected<void, std::string> erase_frames(const std::string & fault_code) override {
    std::lock_guard<std::mutex> lock(mutex_);
    rows_.erase(fault_code);
    return {};
  }

  tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<StoredEntityFreezeFrame> all;
    for (const auto & entry : rows_) {
      all.insert(all.end(), entry.second.begin(), entry.second.end());
    }
    // Same total order the SQLite backend serves, so a caller's bound-keeping
    // behaves identically on both.
    std::stable_sort(all.begin(), all.end(), [](const StoredEntityFreezeFrame & a, const StoredEntityFreezeFrame & b) {
      if (a.captured_at_ns != b.captured_at_ns) {
        return a.captured_at_ns < b.captured_at_ns;
      }
      if (a.fault_code != b.fault_code) {
        return a.fault_code < b.fault_code;
      }
      return a.entity_id < b.entity_id;
    });
    return all;
  }

 private:
  mutable std::mutex mutex_;
  std::map<std::string, std::vector<StoredEntityFreezeFrame>> rows_;
};

}  // namespace ros2_medkit_gateway
