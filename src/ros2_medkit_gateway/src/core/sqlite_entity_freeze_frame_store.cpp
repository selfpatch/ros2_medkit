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

#include "ros2_medkit_gateway/core/sqlite_entity_freeze_frame_store.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace ros2_medkit_gateway {

namespace {

/// RAII wrapper for SQLite prepared statements (mirrors SqliteTriggerStore's,
/// plus the 64-bit binds a nanosecond timestamp needs).
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
    const auto size = value.size();
    if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("bind_text: value exceeds SQLite int length limit");
    }
    if (sqlite3_bind_text(stmt_, index, value.c_str(), static_cast<int>(size), SQLITE_TRANSIENT) != SQLITE_OK) {
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

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

SqliteEntityFreezeFrameStore::SqliteEntityFreezeFrameStore(const std::string & db_path) : db_path_(db_path) {
  int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;
  if (sqlite3_open_v2(db_path.c_str(), &db_, flags, nullptr) != SQLITE_OK) {
    std::string error = db_ ? sqlite3_errmsg(db_) : "Unknown error";
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
    throw std::runtime_error("Failed to open entity freeze-frame database '" + db_path + "': " + error);
  }

  char * err_msg = nullptr;
  if (sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    sqlite3_close(db_);
    db_ = nullptr;
    throw std::runtime_error("Failed to enable WAL mode: " + error);
  }

  sqlite3_busy_timeout(db_, 5000);
  initialize_schema();
}

SqliteEntityFreezeFrameStore::~SqliteEntityFreezeFrameStore() {
  if (db_) {
    sqlite3_close(db_);
  }
}

// ---------------------------------------------------------------------------
// Schema
// ---------------------------------------------------------------------------

void SqliteEntityFreezeFrameStore::initialize_schema() {
  // One row per (fault_code, entity_id): a fault reported by two entities
  // freezes both, and a re-confirm replaces the code's rows as a unit.
  const char * create_table = R"(
    CREATE TABLE IF NOT EXISTS entity_freeze_frames (
      fault_code     TEXT NOT NULL,
      entity_id      TEXT NOT NULL,
      frame          TEXT NOT NULL,
      captured_at_ns INTEGER NOT NULL,
      source         TEXT NOT NULL DEFAULT '',
      capture_origin TEXT NOT NULL DEFAULT '',
      PRIMARY KEY (fault_code, entity_id)
    );
  )";

  char * err_msg = nullptr;
  if (sqlite3_exec(db_, create_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create entity_freeze_frames table: " + error);
  }
}

// ---------------------------------------------------------------------------
// Writes
// ---------------------------------------------------------------------------

tl::expected<void, std::string> SqliteEntityFreezeFrameStore::delete_code_locked(const std::string & fault_code) {
  SqliteStatement del(db_, "DELETE FROM entity_freeze_frames WHERE fault_code = ?");
  del.bind_text(1, fault_code);
  if (del.step() != SQLITE_DONE) {
    return tl::make_unexpected(std::string("Failed to delete entity freeze-frames: ") + sqlite3_errmsg(db_));
  }
  return {};
}

tl::expected<void, std::string>
SqliteEntityFreezeFrameStore::replace_frames(const std::string & fault_code,
                                             const std::vector<StoredEntityFreezeFrame> & frames) {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    // Delete-then-insert in one transaction: a re-confirm that no longer
    // reports an entity must not leave that entity's stale row behind, and a
    // reader must never see the code half-written.
    char * err_msg = nullptr;
    if (sqlite3_exec(db_, "BEGIN IMMEDIATE", nullptr, nullptr, &err_msg) != SQLITE_OK) {
      std::string error = err_msg ? err_msg : "Unknown error";
      sqlite3_free(err_msg);
      return tl::make_unexpected("replace_frames: BEGIN failed: " + error);
    }

    const auto rollback = [this] {
      sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    };

    auto deleted = delete_code_locked(fault_code);
    if (!deleted) {
      rollback();
      return deleted;
    }

    for (const auto & frame : frames) {
      SqliteStatement stmt(db_,
                           "INSERT OR REPLACE INTO entity_freeze_frames "
                           "(fault_code, entity_id, frame, captured_at_ns, source, capture_origin) "
                           "VALUES (?,?,?,?,?,?)");
      stmt.bind_text(1, fault_code);
      stmt.bind_text(2, frame.entity_id);
      stmt.bind_text(3, frame.frame.dump());
      stmt.bind_int64(4, frame.captured_at_ns);
      stmt.bind_text(5, frame.source);
      stmt.bind_text(6, frame.capture_origin);
      if (stmt.step() != SQLITE_DONE) {
        std::string error = sqlite3_errmsg(db_);
        rollback();
        return tl::make_unexpected("Failed to save entity freeze-frame: " + error);
      }
    }

    if (sqlite3_exec(db_, "COMMIT", nullptr, nullptr, &err_msg) != SQLITE_OK) {
      std::string error = err_msg ? err_msg : "Unknown error";
      sqlite3_free(err_msg);
      rollback();
      return tl::make_unexpected("replace_frames: COMMIT failed: " + error);
    }
    return {};
  } catch (const std::exception & e) {
    sqlite3_exec(db_, "ROLLBACK", nullptr, nullptr, nullptr);
    return tl::make_unexpected(std::string("replace_frames: ") + e.what());
  }
}

tl::expected<void, std::string> SqliteEntityFreezeFrameStore::erase_frames(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);
  try {
    return delete_code_locked(fault_code);
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("erase_frames: ") + e.what());
  }
}

// ---------------------------------------------------------------------------
// load_all
// ---------------------------------------------------------------------------

tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> SqliteEntityFreezeFrameStore::load_all() {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    SqliteStatement stmt(db_,
                         "SELECT fault_code, entity_id, frame, captured_at_ns, source, capture_origin "
                         "FROM entity_freeze_frames "
                         "ORDER BY captured_at_ns ASC, fault_code ASC, entity_id ASC");

    std::vector<StoredEntityFreezeFrame> result;
    while (stmt.step() == SQLITE_ROW) {
      StoredEntityFreezeFrame row;
      row.fault_code = stmt.column_text(0);
      row.entity_id = stmt.column_text(1);
      auto parsed = nlohmann::json::parse(stmt.column_text(2), nullptr, false);
      if (parsed.is_discarded()) {
        // One unreadable row must not cost the operator every other frame:
        // skip it, the caller's catch-up re-reads that fault if it is still
        // standing.
        continue;
      }
      row.frame = std::move(parsed);
      row.captured_at_ns = stmt.column_int64(3);
      row.source = stmt.column_text(4);
      row.capture_origin = stmt.column_text(5);
      result.push_back(std::move(row));
    }
    return result;
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("load_all: ") + e.what());
  }
}

}  // namespace ros2_medkit_gateway
