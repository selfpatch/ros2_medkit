// Copyright 2025 mfaferek93
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

#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"

#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include "rcutils/logging_macros.h"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// RAII wrapper for SQLite statements
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
      throw std::runtime_error("Failed to bind text: value size exceeds SQLite int length limit");
    }
    const auto length = static_cast<int>(size);
    if (sqlite3_bind_text(stmt_, index, value.c_str(), length, SQLITE_TRANSIENT) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind text: ") + sqlite3_errmsg(db_));
    }
  }

  void bind_int(int index, int value) {
    if (sqlite3_bind_int(stmt_, index, value) != SQLITE_OK) {
      throw std::runtime_error(std::string("Failed to bind int: ") + sqlite3_errmsg(db_));
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

  void reset() {
    sqlite3_reset(stmt_);
    sqlite3_clear_bindings(stmt_);
  }

  std::string column_text(int index) {
    const auto * text = reinterpret_cast<const char *>(sqlite3_column_text(stmt_, index));
    return text ? std::string(text) : std::string();
  }

  int column_int(int index) {
    return sqlite3_column_int(stmt_, index);
  }

  int64_t column_int64(int index) {
    return sqlite3_column_int64(stmt_, index);
  }

 private:
  sqlite3 * db_;
  sqlite3_stmt * stmt_{nullptr};
};

}  // namespace

SqliteFaultStorage::SqliteFaultStorage(const std::string & db_path) : db_path_(db_path) {
  int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE | SQLITE_OPEN_FULLMUTEX;
  if (sqlite3_open_v2(db_path.c_str(), &db_, flags, nullptr) != SQLITE_OK) {
    std::string error = db_ ? sqlite3_errmsg(db_) : "Unknown error";
    if (db_) {
      sqlite3_close(db_);
      db_ = nullptr;
    }
    throw std::runtime_error("Failed to open database '" + db_path + "': " + error);
  }

  // Enable WAL mode for better concurrent performance
  char * err_msg = nullptr;
  if (sqlite3_exec(db_, "PRAGMA journal_mode=WAL;", nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    sqlite3_close(db_);
    db_ = nullptr;
    throw std::runtime_error("Failed to enable WAL mode: " + error);
  }

  // Set busy timeout to handle concurrent access
  sqlite3_busy_timeout(db_, 5000);

  initialize_schema();
}

SqliteFaultStorage::~SqliteFaultStorage() {
  if (db_) {
    sqlite3_close(db_);
  }
}

void SqliteFaultStorage::set_debounce_config(const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

DebounceConfig SqliteFaultStorage::get_debounce_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void SqliteFaultStorage::initialize_schema() {
  const char * create_table_sql = R"(
    CREATE TABLE IF NOT EXISTS faults (
      fault_code TEXT PRIMARY KEY,
      severity INTEGER NOT NULL,
      description TEXT NOT NULL,
      first_occurred_ns INTEGER NOT NULL,
      last_occurred_ns INTEGER NOT NULL,
      occurrence_count INTEGER NOT NULL,
      status TEXT NOT NULL,
      reporting_sources TEXT NOT NULL,
      debounce_counter INTEGER NOT NULL DEFAULT 0,
      last_failed_ns INTEGER NOT NULL DEFAULT 0,
      last_passed_ns INTEGER NOT NULL DEFAULT 0
    );
  )";

  char * err_msg = nullptr;
  if (sqlite3_exec(db_, create_table_sql, nullptr, nullptr, &err_msg) != SQLITE_OK) {
    std::string error = err_msg ? err_msg : "Unknown error";
    sqlite3_free(err_msg);
    throw std::runtime_error("Failed to create schema: " + error);
  }
}

std::vector<std::string> SqliteFaultStorage::parse_json_array(const std::string & json_str) {
  std::vector<std::string> result;

  // Simple JSON array parser for ["a", "b", "c"] format
  if (json_str.size() < 2 || json_str.front() != '[' || json_str.back() != ']') {
    if (!json_str.empty()) {
      RCUTILS_LOG_WARN_NAMED("sqlite_fault_storage", "Malformed JSON array in database: '%s'", json_str.c_str());
    }
    return result;
  }

  std::string content = json_str.substr(1, json_str.size() - 2);
  if (content.empty()) {
    return result;
  }

  size_t pos = 0;
  while (pos < content.size()) {
    // Skip whitespace
    while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos]))) {
      ++pos;
    }
    if (pos >= content.size()) {
      break;
    }

    // Expect opening quote
    if (content[pos] != '"') {
      break;
    }
    ++pos;

    // Find closing quote (handle escape sequences)
    std::string value;
    while (pos < content.size() && content[pos] != '"') {
      if (content[pos] == '\\' && pos + 1 < content.size()) {
        ++pos;
        char escaped = content[pos];
        switch (escaped) {
          case '"':
            value.push_back('"');
            break;
          case '\\':
            value.push_back('\\');
            break;
          case '/':
            value.push_back('/');
            break;
          case 'b':
            value.push_back('\b');
            break;
          case 'f':
            value.push_back('\f');
            break;
          case 'n':
            value.push_back('\n');
            break;
          case 'r':
            value.push_back('\r');
            break;
          case 't':
            value.push_back('\t');
            break;
          default:
            // Unknown escape sequence: preserve character as-is
            value.push_back(escaped);
            break;
        }
        ++pos;
        continue;
      }
      value.push_back(content[pos]);
      ++pos;
    }

    if (pos < content.size()) {
      ++pos;  // Skip closing quote
    }

    result.push_back(value);

    // Skip whitespace and comma
    while (pos < content.size() && (std::isspace(static_cast<unsigned char>(content[pos])) || content[pos] == ',')) {
      ++pos;
    }
  }

  return result;
}

std::string SqliteFaultStorage::serialize_json_array(const std::vector<std::string> & vec) {
  std::ostringstream oss;
  oss << '[';
  for (size_t i = 0; i < vec.size(); ++i) {
    if (i > 0) {
      oss << ',';
    }
    oss << '"';
    // Escape special characters per JSON specification
    for (char c : vec[i]) {
      switch (c) {
        case '"':
          oss << "\\\"";
          break;
        case '\\':
          oss << "\\\\";
          break;
        case '\b':
          oss << "\\b";
          break;
        case '\f':
          oss << "\\f";
          break;
        case '\n':
          oss << "\\n";
          break;
        case '\r':
          oss << "\\r";
          break;
        case '\t':
          oss << "\\t";
          break;
        default:
          oss << c;
          break;
      }
    }
    oss << '"';
  }
  oss << ']';
  return oss.str();
}

bool SqliteFaultStorage::report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                            const std::string & description, const std::string & source_id,
                                            const rclcpp::Time & timestamp) {
  std::lock_guard<std::mutex> lock(mutex_);

  int64_t timestamp_ns = timestamp.nanoseconds();
  const bool is_failed = (event_type == EventType::EVENT_FAILED);

  // Check if fault exists
  SqliteStatement check_stmt(db_,
                             "SELECT severity, occurrence_count, reporting_sources, status, debounce_counter FROM "
                             "faults WHERE fault_code = ?");
  check_stmt.bind_text(1, fault_code);

  if (check_stmt.step() == SQLITE_ROW) {
    // Fault exists - update it
    int existing_severity = check_stmt.column_int(0);
    int64_t existing_count = check_stmt.column_int64(1);
    std::string sources_json = check_stmt.column_text(2);
    std::string current_status = check_stmt.column_text(3);
    int32_t debounce_counter = static_cast<int32_t>(check_stmt.column_int(4));

    // Don't update CLEARED faults
    if (current_status == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
      return false;
    }

    if (is_failed) {
      // FAILED event
      // Parse existing sources and add new one
      std::vector<std::string> sources = parse_json_array(sources_json);
      std::set<std::string> sources_set(sources.begin(), sources.end());
      sources_set.insert(source_id);
      sources.assign(sources_set.begin(), sources_set.end());

      // Escalate severity if new severity is higher
      int new_severity = std::max(existing_severity, static_cast<int>(severity));

      // Increment count with saturation
      int64_t new_count = existing_count;
      if (new_count < std::numeric_limits<uint32_t>::max()) {
        ++new_count;
      }

      // Decrement debounce counter with saturation
      if (debounce_counter > std::numeric_limits<int32_t>::min()) {
        --debounce_counter;
      }

      // Check for immediate confirmation of CRITICAL
      std::string new_status = current_status;
      if (config_.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
      } else if (debounce_counter <= config_.confirmation_threshold) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
      } else if (debounce_counter < 0) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED;
      } else if (debounce_counter > 0) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED;
      }
      // Note: debounce_counter == 0 keeps current status

      // Update with new values
      SqliteStatement update_stmt(
          db_,
          description.empty()
              ? "UPDATE faults SET severity = ?, last_occurred_ns = ?, last_failed_ns = ?, occurrence_count = ?, "
                "reporting_sources = ?, status = ?, debounce_counter = ? WHERE fault_code = ?"
              : "UPDATE faults SET severity = ?, description = ?, last_occurred_ns = ?, last_failed_ns = ?, "
                "occurrence_count = ?, reporting_sources = ?, status = ?, debounce_counter = ? WHERE fault_code = ?");

      if (description.empty()) {
        update_stmt.bind_int(1, new_severity);
        update_stmt.bind_int64(2, timestamp_ns);
        update_stmt.bind_int64(3, timestamp_ns);
        update_stmt.bind_int64(4, new_count);
        update_stmt.bind_text(5, serialize_json_array(sources));
        update_stmt.bind_text(6, new_status);
        update_stmt.bind_int(7, debounce_counter);
        update_stmt.bind_text(8, fault_code);
      } else {
        update_stmt.bind_int(1, new_severity);
        update_stmt.bind_text(2, description);
        update_stmt.bind_int64(3, timestamp_ns);
        update_stmt.bind_int64(4, timestamp_ns);
        update_stmt.bind_int64(5, new_count);
        update_stmt.bind_text(6, serialize_json_array(sources));
        update_stmt.bind_text(7, new_status);
        update_stmt.bind_int(8, debounce_counter);
        update_stmt.bind_text(9, fault_code);
      }

      if (update_stmt.step() != SQLITE_DONE) {
        throw std::runtime_error(std::string("Failed to update fault: ") + sqlite3_errmsg(db_));
      }
    } else {
      // PASSED event - increment debounce counter with saturation
      if (debounce_counter < std::numeric_limits<int32_t>::max()) {
        ++debounce_counter;
      }

      std::string new_status = current_status;
      if (config_.healing_enabled && debounce_counter >= config_.healing_threshold) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_HEALED;
      } else if (debounce_counter > 0) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED;
      } else if (debounce_counter < 0) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED;
      }

      SqliteStatement update_stmt(
          db_,
          "UPDATE faults SET last_occurred_ns = ?, last_passed_ns = ?, status = ?, debounce_counter = ? WHERE "
          "fault_code = ?");
      update_stmt.bind_int64(1, timestamp_ns);
      update_stmt.bind_int64(2, timestamp_ns);
      update_stmt.bind_text(3, new_status);
      update_stmt.bind_int(4, debounce_counter);
      update_stmt.bind_text(5, fault_code);

      if (update_stmt.step() != SQLITE_DONE) {
        throw std::runtime_error(std::string("Failed to update fault: ") + sqlite3_errmsg(db_));
      }
    }

    return false;  // Existing fault updated
  }

  // New fault - only create for FAILED events
  if (!is_failed) {
    return false;  // PASSED event for non-existent fault is ignored
  }

  // Determine initial status based on debounce logic
  std::string initial_status;
  constexpr int32_t initial_counter = -1;  // First FAILED event sets counter to -1
  // CRITICAL severity bypasses debounce and confirms immediately
  if (config_.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
    initial_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  } else if (initial_counter <= config_.confirmation_threshold) {
    // Counter already meets threshold (e.g., threshold >= -1)
    initial_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  } else {
    initial_status = ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED;
  }

  // New fault - insert with debounce_counter = -1
  SqliteStatement insert_stmt(db_,
                              "INSERT INTO faults (fault_code, severity, description, first_occurred_ns, "
                              "last_occurred_ns, occurrence_count, status, reporting_sources, "
                              "debounce_counter, last_failed_ns, last_passed_ns) "
                              "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)");

  insert_stmt.bind_text(1, fault_code);
  insert_stmt.bind_int(2, static_cast<int>(severity));
  insert_stmt.bind_text(3, description);
  insert_stmt.bind_int64(4, timestamp_ns);
  insert_stmt.bind_int64(5, timestamp_ns);
  insert_stmt.bind_int(6, 1);  // occurrence_count = 1
  insert_stmt.bind_text(7, initial_status);
  insert_stmt.bind_text(8, serialize_json_array({source_id}));
  insert_stmt.bind_int(9, -1);               // debounce_counter = -1 for first FAILED
  insert_stmt.bind_int64(10, timestamp_ns);  // last_failed_ns
  insert_stmt.bind_int64(11, 0);             // last_passed_ns (never passed)

  if (insert_stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to insert fault: ") + sqlite3_errmsg(db_));
  }

  return true;  // New fault created
}

std::vector<ros2_medkit_msgs::msg::Fault>
SqliteFaultStorage::get_faults(bool filter_by_severity, uint8_t severity,
                               const std::vector<std::string> & statuses) const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Determine which statuses to include
  std::set<std::string> status_filter;
  if (statuses.empty()) {
    status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  } else {
    for (const auto & s : statuses) {
      if (s == ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED || s == ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED || s == ros2_medkit_msgs::msg::Fault::STATUS_HEALED ||
          s == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
        status_filter.insert(s);
      }
    }
    if (status_filter.empty()) {
      status_filter.insert(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
    }
  }

  // Build query
  std::string sql =
      "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
      "occurrence_count, status, reporting_sources FROM faults WHERE status IN (";
  for (size_t i = 0; i < status_filter.size(); ++i) {
    if (i > 0) {
      sql += ", ";
    }
    sql += "?";
  }
  sql += ")";

  if (filter_by_severity) {
    sql += " AND severity = ?";
  }

  SqliteStatement stmt(db_, sql.c_str());

  int param_index = 1;
  for (const auto & s : status_filter) {
    stmt.bind_text(param_index++, s);
  }
  if (filter_by_severity) {
    stmt.bind_int(param_index, static_cast<int>(severity));
  }

  std::vector<ros2_medkit_msgs::msg::Fault> result;
  while (stmt.step() == SQLITE_ROW) {
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = stmt.column_text(0);
    fault.severity = static_cast<uint8_t>(stmt.column_int(1));
    fault.description = stmt.column_text(2);

    int64_t first_ns = stmt.column_int64(3);
    int64_t last_ns = stmt.column_int64(4);
    fault.first_occurred = rclcpp::Time(first_ns, RCL_SYSTEM_TIME);
    fault.last_occurred = rclcpp::Time(last_ns, RCL_SYSTEM_TIME);

    fault.occurrence_count = static_cast<uint32_t>(stmt.column_int64(5));
    fault.status = stmt.column_text(6);
    fault.reporting_sources = parse_json_array(stmt.column_text(7));

    result.push_back(fault);
  }

  return result;
}

std::optional<ros2_medkit_msgs::msg::Fault> SqliteFaultStorage::get_fault(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_,
                       "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
                       "occurrence_count, status, reporting_sources FROM faults WHERE fault_code = ?");
  stmt.bind_text(1, fault_code);

  if (stmt.step() != SQLITE_ROW) {
    return std::nullopt;
  }

  ros2_medkit_msgs::msg::Fault fault;
  fault.fault_code = stmt.column_text(0);
  fault.severity = static_cast<uint8_t>(stmt.column_int(1));
  fault.description = stmt.column_text(2);

  int64_t first_ns = stmt.column_int64(3);
  int64_t last_ns = stmt.column_int64(4);
  fault.first_occurred = rclcpp::Time(first_ns, RCL_SYSTEM_TIME);
  fault.last_occurred = rclcpp::Time(last_ns, RCL_SYSTEM_TIME);

  fault.occurrence_count = static_cast<uint32_t>(stmt.column_int64(5));
  fault.status = stmt.column_text(6);
  fault.reporting_sources = parse_json_array(stmt.column_text(7));

  return fault;
}

bool SqliteFaultStorage::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_, "UPDATE faults SET status = ? WHERE fault_code = ?");
  stmt.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_CLEARED);
  stmt.bind_text(2, fault_code);

  if (stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to clear fault: ") + sqlite3_errmsg(db_));
  }

  return sqlite3_changes(db_) > 0;
}

size_t SqliteFaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_, "SELECT COUNT(*) FROM faults");

  if (stmt.step() != SQLITE_ROW) {
    return 0;
  }

  return static_cast<size_t>(stmt.column_int64(0));
}

bool SqliteFaultStorage::contains(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  SqliteStatement stmt(db_, "SELECT 1 FROM faults WHERE fault_code = ? LIMIT 1");
  stmt.bind_text(1, fault_code);

  return stmt.step() == SQLITE_ROW;
}

size_t SqliteFaultStorage::check_time_based_confirmation(const rclcpp::Time & current_time) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (config_.auto_confirm_after_sec <= 0.0) {
    return 0;  // Time-based confirmation disabled
  }

  int64_t current_ns = current_time.nanoseconds();
  int64_t threshold_ns = static_cast<int64_t>(config_.auto_confirm_after_sec * 1e9);
  int64_t cutoff_ns = current_ns - threshold_ns;

  SqliteStatement update_stmt(
      db_, "UPDATE faults SET status = ? WHERE status = ? AND last_failed_ns <= ? AND last_failed_ns > 0");
  update_stmt.bind_text(1, ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  update_stmt.bind_text(2, ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
  update_stmt.bind_int64(3, cutoff_ns);

  if (update_stmt.step() != SQLITE_DONE) {
    throw std::runtime_error(std::string("Failed to confirm faults: ") + sqlite3_errmsg(db_));
  }

  return static_cast<size_t>(sqlite3_changes(db_));
}

}  // namespace ros2_medkit_fault_manager
