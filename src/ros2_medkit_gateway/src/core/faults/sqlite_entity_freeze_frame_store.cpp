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

#include "ros2_medkit_gateway/core/faults/sqlite_entity_freeze_frame_store.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace ros2_medkit_gateway {

namespace {

/// RAII prepared statement with the 64-bit binds a nanosecond timestamp needs.
class SqliteStatement {
 public:
  SqliteStatement(sqlite3 * db, const char * sql) : db_(db) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to prepare statement: ") + sqlite3_errmsg(db));
    }
  }

  ~SqliteStatement() {
    if (stmt_) {
      sqlite3_finalize(stmt_);
    }
  }

  SqliteStatement(const SqliteStatement &) = delete;
  SqliteStatement & operator=(const SqliteStatement &) = delete;
  SqliteStatement(SqliteStatement &&) = delete;
  SqliteStatement & operator=(SqliteStatement &&) = delete;

  void bind_text(int index, const std::string & value) {
    if (value.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("bind_text: value exceeds SQLite int length limit");
    }
    if (sqlite3_bind_text(stmt_, index, value.c_str(), static_cast<int>(value.size()), SQLITE_TRANSIENT) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind text: ") + sqlite3_errmsg(db_));
    }
  }

  void bind_int64(int index, int64_t value) {
    if (sqlite3_bind_int64(stmt_, index, value) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind int64: ") + sqlite3_errmsg(db_));
    }
  }

  int step() {
    return sqlite3_step(stmt_);
  }

  std::string column_text(int index) {
    const auto * text = reinterpret_cast<const char *>(sqlite3_column_text(stmt_, index));
    return text ? std::string(text) : std::string();
  }

  int64_t column_int64(int index) {
    return sqlite3_column_int64(stmt_, index);
  }

 private:
  sqlite3 * db_;
  sqlite3_stmt * stmt_{nullptr};
};

std::string take_error(char * err_msg) {
  std::string error = err_msg ? err_msg : "Unknown error";
  sqlite3_free(err_msg);
  return error;
}

}  // namespace

SqliteEntityFreezeFrameStore::SqliteEntityFreezeFrameStore(const std::string & db_path) {
  const int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;
  if (sqlite3_open_v2(db_path.c_str(), &db_, flags, nullptr) != SQLITE_OK) {
    const std::string error = db_ ? sqlite3_errmsg(db_) : "Unknown error";
    sqlite3_close(db_);
    db_ = nullptr;
    throw std::runtime_error("Failed to open entity freeze-frame database '" + db_path + "': " + error);
  }
  try {
    char * err_msg = nullptr;
    if (sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &err_msg) != SQLITE_OK) {
      throw std::runtime_error("Failed to enable WAL mode: " + take_error(err_msg));
    }
    sqlite3_busy_timeout(db_, 5000);
    initialize_schema();
  } catch (...) {
    // The destructor does not run for a constructor that throws.
    sqlite3_close(db_);
    db_ = nullptr;
    throw;
  }
}

SqliteEntityFreezeFrameStore::~SqliteEntityFreezeFrameStore() {
  if (db_) {
    sqlite3_close(db_);
  }
}

namespace {

// One row per entity of a record. A record is a fault code and its owner: the reporting
// source of a per-source record, "*" for a shared one.
constexpr const char * kCreateTable = R"(
    CREATE TABLE IF NOT EXISTS entity_freeze_frames (
      fault_code     TEXT NOT NULL,
      owner          TEXT NOT NULL,
      entity_id      TEXT NOT NULL,
      frame          TEXT NOT NULL,
      captured_at_ns INTEGER NOT NULL,
      source         TEXT NOT NULL DEFAULT '',
      capture_origin TEXT NOT NULL DEFAULT '',
      occurrence_count  INTEGER NOT NULL DEFAULT 0,
      first_occurred_ns INTEGER NOT NULL DEFAULT 0,
      PRIMARY KEY (fault_code, owner, entity_id)
    );
  )";

}  // namespace

void SqliteEntityFreezeFrameStore::initialize_schema() {
  migrate_earlier_schema();
  char * err_msg = nullptr;
  if (sqlite3_exec(db_, kCreateTable, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    throw std::runtime_error("Failed to create entity_freeze_frames table: " + take_error(err_msg));
  }
}

void SqliteEntityFreezeFrameStore::migrate_earlier_schema() {
  bool table = false;
  bool owner = false;
  {
    SqliteStatement columns(db_, "PRAGMA table_info(entity_freeze_frames)");
    while (columns.step() == SQLITE_ROW) {
      table = true;
      owner = owner || columns.column_text(1) == "owner";
    }
  }
  if (!table || owner) {
    return;
  }
  // Every record of the earlier schema was shared, so "*" names each row's record.
  char * err_msg = nullptr;
  if (sqlite3_exec(db_, "BEGIN IMMEDIATE", nullptr, nullptr, &err_msg) != SQLITE_OK) {
    throw std::runtime_error("Failed to migrate entity_freeze_frames: BEGIN failed: " + take_error(err_msg));
  }
  const auto run = [this](const char * sql) {
    char * error = nullptr;
    if (sqlite3_exec(db_, sql, nullptr, nullptr, &error) != SQLITE_OK) {
      const std::string message = take_error(error);
      sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
      throw std::runtime_error("Failed to migrate entity_freeze_frames: " + message);
    }
  };
  run("ALTER TABLE entity_freeze_frames RENAME TO entity_freeze_frames_earlier");
  run(kCreateTable);
  run("INSERT INTO entity_freeze_frames (fault_code, owner, entity_id, frame, captured_at_ns, source, "
      "capture_origin, occurrence_count, first_occurred_ns) "
      "SELECT fault_code, '*', entity_id, frame, captured_at_ns, source, capture_origin, occurrence_count, "
      "first_occurred_ns FROM entity_freeze_frames_earlier");
  const int moved = sqlite3_changes(db_);
  run("DROP TABLE entity_freeze_frames_earlier");
  run("COMMIT");
  MEDKIT_LOG_INFO(logger_,
                  "Entity freeze-frame store: moved %d row(s) of the earlier schema to the shared record ('*') of "
                  "their code",
                  moved);
}

tl::expected<void, std::string> SqliteEntityFreezeFrameStore::delete_record_locked(const std::string & fault_code,
                                                                                   const std::string & owner) {
  SqliteStatement del(db_, "DELETE FROM entity_freeze_frames WHERE fault_code = ? AND owner = ?");
  del.bind_text(1, fault_code);
  del.bind_text(2, owner);
  if (del.step() != SQLITE_DONE) {
    return tl::make_unexpected(std::string("Failed to delete entity freeze-frames: ") + sqlite3_errmsg(db_));
  }
  return {};
}

tl::expected<void, std::string>
SqliteEntityFreezeFrameStore::replace_frames(const std::string & fault_code, const std::string & owner,
                                             const std::vector<StoredEntityFreezeFrame> & frames) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto rollback = [this] {
    sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
  };
  // Delete then insert in one transaction: an entity the new capture no longer
  // reports loses its row, and no reader sees the record half written.
  char * err_msg = nullptr;
  if (sqlite3_exec(db_, "BEGIN IMMEDIATE", nullptr, nullptr, &err_msg) != SQLITE_OK) {
    return tl::make_unexpected("replace_frames: BEGIN failed: " + take_error(err_msg));
  }
  try {
    auto deleted = delete_record_locked(fault_code, owner);
    if (!deleted) {
      rollback();
      return deleted;
    }
    for (const auto & frame : frames) {
      SqliteStatement stmt(db_,
                           "INSERT OR REPLACE INTO entity_freeze_frames "
                           "(fault_code, owner, entity_id, frame, captured_at_ns, source, capture_origin, "
                           "occurrence_count, first_occurred_ns) "
                           "VALUES (?,?,?,?,?,?,?,?,?)");
      stmt.bind_text(1, fault_code);
      stmt.bind_text(2, owner);
      stmt.bind_text(3, frame.entity_id);
      stmt.bind_text(4, frame.frame.dump());
      stmt.bind_int64(5, frame.captured_at_ns);
      stmt.bind_text(6, frame.source);
      stmt.bind_text(7, frame.capture_origin);
      stmt.bind_int64(8, frame.occurrence_count);
      stmt.bind_int64(9, frame.first_occurred_ns);
      if (stmt.step() != SQLITE_DONE) {
        const std::string error = sqlite3_errmsg(db_);
        rollback();
        return tl::make_unexpected("Failed to save entity freeze-frame: " + error);
      }
    }
    if (sqlite3_exec(db_, "COMMIT", nullptr, nullptr, &err_msg) != SQLITE_OK) {
      const std::string error = take_error(err_msg);
      rollback();
      return tl::make_unexpected("replace_frames: COMMIT failed: " + error);
    }
    return {};
  } catch (const std::exception & e) {
    rollback();
    return tl::make_unexpected(std::string("replace_frames: ") + e.what());
  }
}

tl::expected<void, std::string> SqliteEntityFreezeFrameStore::erase_frames(const std::string & fault_code,
                                                                           const std::string & owner) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    return delete_record_locked(fault_code, owner);
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("erase_frames: ") + e.what());
  }
}

tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> SqliteEntityFreezeFrameStore::load_all() {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    SqliteStatement stmt(db_,
                         "SELECT fault_code, owner, entity_id, frame, captured_at_ns, source, capture_origin, "
                         "occurrence_count, first_occurred_ns "
                         "FROM entity_freeze_frames "
                         "ORDER BY captured_at_ns ASC, fault_code ASC, owner ASC, entity_id ASC");
    std::vector<StoredEntityFreezeFrame> result;
    int rc = SQLITE_ROW;
    while ((rc = stmt.step()) == SQLITE_ROW) {
      auto parsed = nlohmann::json::parse(stmt.column_text(3), nullptr, false);
      StoredEntityFreezeFrame row;
      row.fault_code = stmt.column_text(0);
      row.owner = stmt.column_text(1);
      row.entity_id = stmt.column_text(2);
      // Not JSON: a null frame, so the caller skips and counts it like any unreadable row.
      row.frame = parsed.is_discarded() ? nlohmann::json() : std::move(parsed);
      row.captured_at_ns = stmt.column_int64(4);
      row.source = stmt.column_text(5);
      row.capture_origin = stmt.column_text(6);
      // Out of range reads as "not known".
      const int64_t count = stmt.column_int64(7);
      row.occurrence_count =
          count > 0 && count <= std::numeric_limits<uint32_t>::max() ? static_cast<uint32_t>(count) : 0;
      row.first_occurred_ns = stmt.column_int64(8);
      result.push_back(std::move(row));
    }
    if (rc != SQLITE_DONE) {
      return tl::make_unexpected(std::string("load_all: ") + sqlite3_errmsg(db_));
    }
    return result;
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("load_all: ") + e.what());
  }
}

}  // namespace ros2_medkit_gateway
