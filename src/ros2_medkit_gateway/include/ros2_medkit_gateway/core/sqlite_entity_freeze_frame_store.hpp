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

#include <sqlite3.h>

#include <mutex>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/entity_freeze_frame_store.hpp"

namespace ros2_medkit_gateway {

/// SQLite-backed entity freeze-frame persistence.
///
/// Thread-safe via internal mutex. The table is created on first open. Use
/// ":memory:" for an ephemeral database.
class SqliteEntityFreezeFrameStore : public EntityFreezeFrameStore {
 public:
  /// Open (or create) the database at `db_path`.
  /// @throws std::runtime_error on SQLite open/init failure.
  explicit SqliteEntityFreezeFrameStore(const std::string & db_path);

  ~SqliteEntityFreezeFrameStore() override;

  // Non-copyable, non-movable (owns SQLite connection)
  SqliteEntityFreezeFrameStore(const SqliteEntityFreezeFrameStore &) = delete;
  SqliteEntityFreezeFrameStore & operator=(const SqliteEntityFreezeFrameStore &) = delete;
  SqliteEntityFreezeFrameStore(SqliteEntityFreezeFrameStore &&) = delete;
  SqliteEntityFreezeFrameStore & operator=(SqliteEntityFreezeFrameStore &&) = delete;

  tl::expected<void, std::string> replace_frames(const std::string & fault_code,
                                                 const std::vector<StoredEntityFreezeFrame> & frames) override;
  tl::expected<void, std::string> erase_frames(const std::string & fault_code) override;
  tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() override;

 private:
  /// Create the table if it does not exist.
  void initialize_schema();

  /// Delete every row for a code. Caller holds mutex_.
  tl::expected<void, std::string> delete_code_locked(const std::string & fault_code);

  std::string db_path_;
  sqlite3 * db_{nullptr};
  mutable std::mutex mutex_;
};

}  // namespace ros2_medkit_gateway
