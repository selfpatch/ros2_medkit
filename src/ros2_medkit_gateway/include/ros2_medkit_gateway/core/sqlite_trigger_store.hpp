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

#include "ros2_medkit_gateway/core/trigger_store.hpp"

namespace ros2_medkit_gateway {

/// SQLite-backed trigger persistence.
///
/// Thread-safe via internal mutex.  Tables are created on first open.
/// Use ":memory:" for an ephemeral in-memory database (useful in tests).
class SqliteTriggerStore : public TriggerStore {
 public:
  /// Open (or create) the database at `db_path`.
  /// @throws std::runtime_error on SQLite open/init failure.
  explicit SqliteTriggerStore(const std::string & db_path);

  ~SqliteTriggerStore() override;

  // Non-copyable, non-movable (owns SQLite connection)
  SqliteTriggerStore(const SqliteTriggerStore &) = delete;
  SqliteTriggerStore & operator=(const SqliteTriggerStore &) = delete;
  SqliteTriggerStore(SqliteTriggerStore &&) = delete;
  SqliteTriggerStore & operator=(SqliteTriggerStore &&) = delete;

  tl::expected<void, std::string> save(const TriggerInfo & trigger) override;
  tl::expected<void, std::string> update(const std::string & id, const nlohmann::json & fields) override;
  tl::expected<void, std::string> remove(const std::string & id) override;
  tl::expected<std::vector<TriggerInfo>, std::string> load_all() override;
  tl::expected<void, std::string> save_state(const std::string & trigger_id,
                                             const nlohmann::json & previous_value) override;
  tl::expected<std::optional<nlohmann::json>, std::string> load_state(const std::string & trigger_id) override;

 private:
  /// Create tables if they do not exist.
  void initialize_schema();

  /// Format a system_clock time_point as ISO 8601 text.
  static std::string to_iso8601(const std::chrono::system_clock::time_point & tp);

  /// Parse an ISO 8601 text value back to a time_point.
  static std::chrono::system_clock::time_point from_iso8601(const std::string & text);

  std::string db_path_;
  sqlite3 * db_{nullptr};
  mutable std::mutex mutex_;
};

}  // namespace ros2_medkit_gateway
