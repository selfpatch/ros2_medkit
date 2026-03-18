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

#include "ros2_medkit_gateway/sqlite_trigger_store.hpp"

#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

namespace ros2_medkit_gateway {

namespace {

/// RAII wrapper for SQLite prepared statements (mirrors fault_manager pattern).
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

  sqlite3_stmt * get() const {
    return stmt_;
  }

  void bind_text(int index, const std::string & value) {
    const auto size = value.size();
    if (size > static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      throw std::runtime_error("bind_text: value exceeds SQLite int length limit");
    }
    if (sqlite3_bind_text(stmt_, index, value.c_str(), static_cast<int>(size), SQLITE_TRANSIENT) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind text: ") + sqlite3_errmsg(db_));
    }
  }

  void bind_int(int index, int value) {
    if (sqlite3_bind_int(stmt_, index, value) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind int: ") + sqlite3_errmsg(db_));
    }
  }

  void bind_null(int index) {
    if (sqlite3_bind_null(stmt_, index) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind null: ") + sqlite3_errmsg(db_));
    }
  }

  int step() {
    return sqlite3_step(stmt_);
  }

  std::string column_text(int index) {
    const auto * text = reinterpret_cast<const char *>(sqlite3_column_text(stmt_, index));
    return text ? std::string(text) : std::string();
  }

  int column_int(int index) {
    return sqlite3_column_int(stmt_, index);
  }

  bool column_is_null(int index) {
    return sqlite3_column_type(stmt_, index) == SQLITE_NULL;
  }

 private:
  sqlite3 * db_;
  sqlite3_stmt * stmt_{nullptr};
};

/// Map TriggerStatus enum to persisted text.
std::string status_to_text(TriggerStatus s) {
  switch (s) {
    case TriggerStatus::ACTIVE:
      return "ACTIVE";
    case TriggerStatus::TERMINATED:
      return "TERMINATED";
  }
  return "ACTIVE";
}

/// Map persisted text to TriggerStatus enum.
TriggerStatus text_to_status(const std::string & s) {
  if (s == "TERMINATED") {
    return TriggerStatus::TERMINATED;
  }
  return TriggerStatus::ACTIVE;
}

}  // namespace

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------

SqliteTriggerStore::SqliteTriggerStore(const std::string & db_path) : db_path_(db_path) {
  int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;
  if (sqlite3_open_v2(db_path.c_str(), &db_, flags, nullptr) != SQLITE_OK) {
    std::string error = db_ ? sqlite3_errmsg(db_) : "Unknown error";
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
    throw std::runtime_error("Failed to open trigger database '" + db_path + "': " + error);
  }

  // WAL mode for better concurrent read/write performance
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

SqliteTriggerStore::~SqliteTriggerStore() {
  if (db_) {
    sqlite3_close(db_);
  }
}

// ---------------------------------------------------------------------------
// Schema
// ---------------------------------------------------------------------------

void SqliteTriggerStore::initialize_schema() {
  const char * create_triggers_table = R"(
    CREATE TABLE IF NOT EXISTS triggers (
      id              TEXT PRIMARY KEY,
      entity_id       TEXT NOT NULL,
      entity_type     TEXT NOT NULL,
      resource_uri    TEXT NOT NULL,
      collection      TEXT NOT NULL,
      resource_path   TEXT NOT NULL,
      path            TEXT NOT NULL,
      condition_type  TEXT NOT NULL,
      condition_params TEXT NOT NULL,
      protocol        TEXT NOT NULL,
      multishot       INTEGER NOT NULL,
      persistent      INTEGER NOT NULL,
      lifetime_sec    INTEGER,
      log_settings    TEXT,
      status          TEXT NOT NULL,
      created_at      TEXT NOT NULL,
      expires_at      TEXT
    );
  )";

  char * err_msg = nullptr;
  if (sqlite3_exec(db_, create_triggers_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create triggers table: " + error);
  }

  const char * create_state_table = R"(
    CREATE TABLE IF NOT EXISTS trigger_state (
      trigger_id      TEXT PRIMARY KEY,
      previous_value  TEXT NOT NULL
    );
  )";

  if (sqlite3_exec(db_, create_state_table, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create trigger_state table: " + error);
  }
}

// ---------------------------------------------------------------------------
// ISO 8601 helpers
// ---------------------------------------------------------------------------

std::string SqliteTriggerStore::to_iso8601(const std::chrono::system_clock::time_point & tp) {
  auto time_t_val = std::chrono::system_clock::to_time_t(tp);
  std::tm tm_val{};
  gmtime_r(&time_t_val, &tm_val);

  std::ostringstream oss;
  oss << std::put_time(&tm_val, "%Y-%m-%dT%H:%M:%S") << "Z";
  return oss.str();
}

std::chrono::system_clock::time_point SqliteTriggerStore::from_iso8601(const std::string & text) {
  std::tm tm_val{};
  std::istringstream iss(text);
  iss >> std::get_time(&tm_val, "%Y-%m-%dT%H:%M:%S");
  if (iss.fail()) {
    return std::chrono::system_clock::time_point{};
  }
  auto time_t_val = timegm(&tm_val);
  return std::chrono::system_clock::from_time_t(time_t_val);
}

// ---------------------------------------------------------------------------
// save  (INSERT OR REPLACE)
// ---------------------------------------------------------------------------

tl::expected<void, std::string> SqliteTriggerStore::save(const TriggerInfo & trigger) {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    SqliteStatement stmt(db_,
                         "INSERT OR REPLACE INTO triggers "
                         "(id, entity_id, entity_type, resource_uri, collection, resource_path, "
                         "path, condition_type, condition_params, protocol, multishot, persistent, "
                         "lifetime_sec, log_settings, status, created_at, expires_at) "
                         "VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)");

    stmt.bind_text(1, trigger.id);
    stmt.bind_text(2, trigger.entity_id);
    stmt.bind_text(3, trigger.entity_type);
    stmt.bind_text(4, trigger.resource_uri);
    stmt.bind_text(5, trigger.collection);
    stmt.bind_text(6, trigger.resource_path);
    stmt.bind_text(7, trigger.path);
    stmt.bind_text(8, trigger.condition_type);
    stmt.bind_text(9, trigger.condition_params.dump());
    stmt.bind_text(10, trigger.protocol);
    stmt.bind_int(11, trigger.multishot ? 1 : 0);
    stmt.bind_int(12, trigger.persistent ? 1 : 0);

    if (trigger.lifetime_sec.has_value()) {
      stmt.bind_int(13, trigger.lifetime_sec.value());
    } else {
      stmt.bind_null(13);
    }

    if (trigger.log_settings.has_value()) {
      stmt.bind_text(14, trigger.log_settings.value().dump());
    } else {
      stmt.bind_null(14);
    }

    stmt.bind_text(15, status_to_text(trigger.status));
    stmt.bind_text(16, to_iso8601(trigger.created_at));

    if (trigger.expires_at.has_value()) {
      stmt.bind_text(17, to_iso8601(trigger.expires_at.value()));
    } else {
      stmt.bind_null(17);
    }

    if (stmt.step() != SQLITE_DONE) {
      return tl::make_unexpected(std::string("Failed to save trigger: ") + sqlite3_errmsg(db_));
    }

    return {};
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("save: ") + e.what());
  }
}

// ---------------------------------------------------------------------------
// update  (partial field update)
// ---------------------------------------------------------------------------

tl::expected<void, std::string> SqliteTriggerStore::update(const std::string & id, const nlohmann::json & fields) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!fields.is_object() || fields.empty()) {
    return tl::make_unexpected("update: fields must be a non-empty JSON object");
  }

  // Allowlist of mutable columns to prevent arbitrary column modification
  static const std::unordered_set<std::string> allowed_columns = {
      "status", "lifetime_sec", "expires_at", "condition_params", "condition_type", "protocol", "multishot"};

  try {
    // Build SET clause dynamically from supplied keys
    std::string sql = "UPDATE triggers SET ";
    std::vector<std::pair<std::string, std::string>> bindings;  // column, value

    bool first = true;
    for (auto it = fields.begin(); it != fields.end(); ++it) {
      if (allowed_columns.find(it.key()) == allowed_columns.end()) {
        return tl::make_unexpected("update: column '" + it.key() + "' is not updatable");
      }
      if (!first) {
        sql += ", ";
      }
      sql += it.key() + " = ?";
      first = false;

      // Convert JSON value to text for binding
      if (it.value().is_string()) {
        bindings.emplace_back(it.key(), it.value().get<std::string>());
      } else {
        bindings.emplace_back(it.key(), it.value().dump());
      }
    }
    sql += " WHERE id = ?";

    SqliteStatement stmt(db_, sql.c_str());

    int idx = 1;
    for (const auto & [col, val] : bindings) {
      stmt.bind_text(idx++, val);
    }
    stmt.bind_text(idx, id);

    if (stmt.step() != SQLITE_DONE) {
      return tl::make_unexpected(std::string("Failed to update trigger: ") + sqlite3_errmsg(db_));
    }

    if (sqlite3_changes(db_) == 0) {
      return tl::make_unexpected("update: trigger '" + id + "' not found");
    }

    return {};
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("update: ") + e.what());
  }
}

// ---------------------------------------------------------------------------
// remove
// ---------------------------------------------------------------------------

tl::expected<void, std::string> SqliteTriggerStore::remove(const std::string & id) {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    // Remove associated evaluator state first
    {
      SqliteStatement del_state(db_, "DELETE FROM trigger_state WHERE trigger_id = ?");
      del_state.bind_text(1, id);
      if (del_state.step() != SQLITE_DONE) {
        return tl::make_unexpected(std::string("Failed to delete trigger state: ") + sqlite3_errmsg(db_));
      }
    }

    SqliteStatement stmt(db_, "DELETE FROM triggers WHERE id = ?");
    stmt.bind_text(1, id);

    if (stmt.step() != SQLITE_DONE) {
      return tl::make_unexpected(std::string("Failed to remove trigger: ") + sqlite3_errmsg(db_));
    }

    if (sqlite3_changes(db_) == 0) {
      return tl::make_unexpected("remove: trigger '" + id + "' not found");
    }

    return {};
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("remove: ") + e.what());
  }
}

// ---------------------------------------------------------------------------
// load_all
// ---------------------------------------------------------------------------

tl::expected<std::vector<TriggerInfo>, std::string> SqliteTriggerStore::load_all() {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    SqliteStatement stmt(db_,
                         "SELECT id, entity_id, entity_type, resource_uri, collection, "
                         "resource_path, path, condition_type, condition_params, protocol, "
                         "multishot, persistent, lifetime_sec, log_settings, status, "
                         "created_at, expires_at FROM triggers");

    std::vector<TriggerInfo> result;
    while (stmt.step() == SQLITE_ROW) {
      TriggerInfo t;
      t.id = stmt.column_text(0);
      t.entity_id = stmt.column_text(1);
      t.entity_type = stmt.column_text(2);
      t.resource_uri = stmt.column_text(3);
      t.collection = stmt.column_text(4);
      t.resource_path = stmt.column_text(5);
      t.path = stmt.column_text(6);
      t.condition_type = stmt.column_text(7);
      auto cond = nlohmann::json::parse(stmt.column_text(8), nullptr, false);
      if (cond.is_discarded()) {
        return tl::make_unexpected(std::string("load_all: corrupt condition_params for trigger '") + t.id + "'");
      }
      t.condition_params = std::move(cond);
      t.protocol = stmt.column_text(9);
      t.multishot = stmt.column_int(10) != 0;
      t.persistent = stmt.column_int(11) != 0;

      if (!stmt.column_is_null(12)) {
        t.lifetime_sec = stmt.column_int(12);
      }

      if (!stmt.column_is_null(13)) {
        auto parsed = nlohmann::json::parse(stmt.column_text(13), nullptr, false);
        if (!parsed.is_discarded()) {
          t.log_settings = parsed;
        }
      }

      t.status = text_to_status(stmt.column_text(14));
      t.created_at = from_iso8601(stmt.column_text(15));

      if (!stmt.column_is_null(16)) {
        t.expires_at = from_iso8601(stmt.column_text(16));
      }

      result.push_back(std::move(t));
    }

    return result;
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("load_all: ") + e.what());
  }
}

// ---------------------------------------------------------------------------
// save_state / load_state
// ---------------------------------------------------------------------------

tl::expected<void, std::string> SqliteTriggerStore::save_state(const std::string & trigger_id,
                                                               const nlohmann::json & previous_value) {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    SqliteStatement stmt(db_,
                         "INSERT OR REPLACE INTO trigger_state (trigger_id, previous_value) "
                         "VALUES (?, ?)");
    stmt.bind_text(1, trigger_id);
    stmt.bind_text(2, previous_value.dump());

    if (stmt.step() != SQLITE_DONE) {
      return tl::make_unexpected(std::string("Failed to save trigger state: ") + sqlite3_errmsg(db_));
    }

    return {};
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("save_state: ") + e.what());
  }
}

tl::expected<std::optional<nlohmann::json>, std::string>
SqliteTriggerStore::load_state(const std::string & trigger_id) {
  std::lock_guard<std::mutex> lock(mutex_);

  try {
    SqliteStatement stmt(db_, "SELECT previous_value FROM trigger_state WHERE trigger_id = ?");
    stmt.bind_text(1, trigger_id);

    if (stmt.step() != SQLITE_ROW) {
      return std::optional<nlohmann::json>{std::nullopt};
    }

    auto parsed = nlohmann::json::parse(stmt.column_text(0), nullptr, false);
    if (parsed.is_discarded()) {
      return tl::make_unexpected("load_state: corrupt JSON for trigger '" + trigger_id + "'");
    }

    return std::optional<nlohmann::json>{std::move(parsed)};
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("load_state: ") + e.what());
  }
}

}  // namespace ros2_medkit_gateway
