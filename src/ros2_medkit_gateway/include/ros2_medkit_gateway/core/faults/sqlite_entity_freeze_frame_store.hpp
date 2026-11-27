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

#include "ros2_medkit_gateway/core/faults/entity_freeze_frame_store.hpp"
#include "ros2_medkit_gateway/core/logging.hpp"

namespace ros2_medkit_gateway {

/// SQLite file of entity freeze-frames, table `entity_freeze_frames`.
/// Thread-safe. The table is created on open. ":memory:" opens an ephemeral database.
/// `load_all` returns a row whose `frame` column is not JSON with a null `frame`.
/// A table of the earlier schema, keyed by (fault_code, entity_id) with no owner, is
/// rebuilt on open: every row becomes a row of the shared record ("*") of its code.
class SqliteEntityFreezeFrameStore : public EntityFreezeFrameStore {
 public:
  /// Opens or creates the database at @p db_path.
  /// @throws std::runtime_error when the file cannot be opened or the table cannot be created.
  explicit SqliteEntityFreezeFrameStore(const std::string & db_path);

  ~SqliteEntityFreezeFrameStore() override;
  SqliteEntityFreezeFrameStore(const SqliteEntityFreezeFrameStore &) = delete;
  SqliteEntityFreezeFrameStore & operator=(const SqliteEntityFreezeFrameStore &) = delete;
  SqliteEntityFreezeFrameStore(SqliteEntityFreezeFrameStore &&) = delete;
  SqliteEntityFreezeFrameStore & operator=(SqliteEntityFreezeFrameStore &&) = delete;

  tl::expected<void, std::string> replace_frames(const std::string & fault_code, const std::string & owner,
                                                 const std::vector<StoredEntityFreezeFrame> & frames) override;
  tl::expected<void, std::string> erase_frames(const std::string & fault_code, const std::string & owner) override;
  tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() override;

 private:
  void initialize_schema();
  /// Rebuilds a table that has no owner column. The caller holds no transaction.
  void migrate_earlier_schema();

  /// Deletes every row of a record. The caller holds mutex_.
  tl::expected<void, std::string> delete_record_locked(const std::string & fault_code, const std::string & owner);

  sqlite3 * db_{nullptr};
  const log::Logger logger_{"entity_freeze_frame"};
  std::mutex mutex_;
};

}  // namespace ros2_medkit_gateway
