// Copyright 2026 gstavrinos
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

#include "ros2_medkit_fault_manager/postgres_fault_storage.hpp"

#include <libpq-fe.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "rcutils/logging_macros.h"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_fault_manager {

namespace {

/// Seconds per connection attempt when neither database_url nor PGCONNECT_TIMEOUT sets it. 2 is the
/// libpq minimum. Two attempts then fit in the gateway's 5 s service timeout.
constexpr const char * kDefaultConnectTimeout = "2";

/// Milliseconds sent data may stay unacknowledged before libpq drops an open connection. Bounds a
/// request to a server that went off the network.
constexpr const char * kDefaultTcpUserTimeout = "5000";

/// Idle seconds before TCP keepalive probes start, so a lost server is found while a reply is due.
constexpr const char * kDefaultKeepalivesIdle = "5";

/// Time after a failed connection round before the next one. Requests in between fail at once.
constexpr std::chrono::seconds kReconnectBackoff{5};

/// Parsed connection options by keyword, or nullopt when libpq rejects the string.
std::optional<std::map<std::string, std::string>> parse_conn_info(const std::string & conn_info) {
  char * err = nullptr;
  PQconninfoOption * options = PQconninfoParse(conn_info.c_str(), &err);
  if (err != nullptr) {
    PQfreemem(err);
  }
  if (options == nullptr) {
    return std::nullopt;
  }
  std::map<std::string, std::string> values;
  for (const PQconninfoOption * o = options; o->keyword != nullptr; ++o) {
    if (o->val != nullptr) {
      values[o->keyword] = o->val;
    }
  }
  PQconninfoFree(options);
  return values;
}

/// libpq key='value' form of @p values, with quotes and backslashes escaped.
std::string to_keyword_string(const std::map<std::string, std::string> & values) {
  std::string out;
  for (const auto & [key, value] : values) {
    std::string escaped;
    for (const char c : value) {
      if (c == '\\' || c == '\'') {
        escaped += '\\';
      }
      escaped += c;
    }
    if (!out.empty()) {
      out += ' ';
    }
    out += key;
    out += "='";
    out += escaped;
    out += '\'';
  }
  return out;
}

/// Named libpq service from @p options or PGSERVICE, empty when none.
std::string service_name(const std::map<std::string, std::string> & options) {
  const auto it = options.find("service");
  if (it != options.end()) {
    return it->second;
  }
  const char * env = std::getenv("PGSERVICE");
  return env == nullptr ? "" : env;
}

}  // namespace

PgFaultStorage::PgFaultStorage(const std::string & conn_info) : PgFaultStorage(conn_info, 1, 500) {
}

PgFaultStorage::PgFaultStorage(const std::string & conn_info, const int max_retries,
                               const unsigned reconnection_delay_ms)
  : conn_info_(conn_info), max_retries_(max_retries), reconnection_delay_(reconnection_delay_ms) {
  // Fixed text: the libpq message quotes parts of the string, and those can be the password.
  auto options = parse_conn_info(conn_info_);
  if (!options) {
    throw std::invalid_argument("database_url is not a valid PostgreSQL connection string");
  }
  // A service file sets its own values; an option given here would override them.
  if (service_name(*options).empty()) {
    if (options->count("connect_timeout") == 0 && std::getenv("PGCONNECT_TIMEOUT") == nullptr) {
      (*options)["connect_timeout"] = kDefaultConnectTimeout;
    }
    options->emplace("tcp_user_timeout", kDefaultTcpUserTimeout);
    options->emplace("keepalives_idle", kDefaultKeepalivesIdle);
  }
  if (const auto it = options->find("password"); it != options->end()) {
    password_ = it->second;
  } else if (const char * env = std::getenv("PGPASSWORD")) {
    password_ = env;
  }
  connect_string_ = to_keyword_string(*options);
  // Connect and create the schema. A server that cannot be reached is not fatal: calls fail until a
  // later connection round succeeds. An SQL error while creating the schema is a wrong
  // configuration and propagates.
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    run_in_transaction("initialize_schema", [](pqxx::work &) {});
  } catch (const IgnorableConnectionException & e) {
    RCUTILS_LOG_WARN_NAMED("postgres_fault_storage", "Starting without fault storage, PostgreSQL is unreachable: %s",
                           e.what());
  }
}

std::string PgFaultStorage::target() const {
  // Keywords from libpq's defaults (environment included), then from database_url. Never the password.
  const auto parsed = parse_conn_info(conn_info_).value_or(std::map<std::string, std::string>{});
  const std::string service = service_name(parsed);
  std::map<std::string, std::string> values;
  // The service file is not read here, so its values are not shown; the defaults would be wrong.
  if (PQconninfoOption * defaults = service.empty() ? PQconndefaults() : nullptr) {
    for (const PQconninfoOption * o = defaults; o->keyword != nullptr; ++o) {
      if (o->val != nullptr) {
        values[o->keyword] = o->val;
      }
    }
    PQconninfoFree(defaults);
  }
  // An explicit empty value overrides the default, as it does in libpq.
  for (const auto & [key, value] : parsed) {
    values[key] = value;
  }
  std::string out = service.empty() ? "" : "service=" + service;
  for (const char * key : {"host", "port", "dbname", "user"}) {
    const auto it = values.find(key);
    if (!service.empty() && it == values.end()) {
      continue;
    }
    out += std::string(out.empty() ? "" : " ") + key + "=" + (it == values.end() ? "" : it->second);
  }
  return out;
}

std::string PgFaultStorage::redact(std::string text) const {
  if (password_.empty()) {
    return text;
  }
  for (size_t pos = text.find(password_); pos != std::string::npos; pos = text.find(password_, pos + 3)) {
    text.replace(pos, password_.size(), "***");
  }
  return text;
}

void PgFaultStorage::fail_round(const std::string & error) const {
  db_conn_.reset();
  connect_failed_ = true;
  last_connect_error_ = error;
  next_connect_attempt_ = std::chrono::steady_clock::now() + kReconnectBackoff;
}

bool PgFaultStorage::connected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return db_conn_ && db_conn_->is_open();
}

PgFaultStorage::~PgFaultStorage() {
  if (!db_conn_ || !db_conn_->is_open()) {
    return;
  }
  try {
    db_conn_->close();
  } catch (...) {
  }
}

template <typename... Args>
pqxx::result PgFaultStorage::execute(pqxx::work & tx, const std::string & query, Args &&... args) const {
  if constexpr (sizeof...(Args) > 0) {
    return tx.exec(query, pqxx::params(std::forward<Args>(args)...));
  } else {
    return tx.exec(query);
  }
}

void PgFaultStorage::ensure_connection() const {
  if (db_conn_ && db_conn_->is_open()) {
    return;
  }
  if (std::chrono::steady_clock::now() < next_connect_attempt_) {
    throw IgnorableConnectionException("Failed to connect to PostgreSQL: " + last_connect_error_);
  }
  // After a failed round the backoff is the retry, so one attempt keeps the caller's wait short.
  const int retries = connect_failed_ ? 0 : max_retries_;
  std::string last_error = "connection is not open";
  for (int attempt = 0; attempt <= retries; ++attempt) {
    if (attempt > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(reconnection_delay_));
    }
    try {
      auto new_connection = std::make_unique<pqxx::connection>(connect_string_);
      if (new_connection->is_open()) {
        db_conn_ = std::move(new_connection);
        schema_ready_ = false;  // the server behind a new connection can be a fresh one
        connect_failed_ = false;
        return;
      }
      last_error = "connection reported itself closed immediately after connecting";
    } catch (const std::exception & e) {
      // The server writes this text, and a server can put the password in it.
      last_error = redact(e.what());
    }
    RCUTILS_LOG_WARN_NAMED("postgres_fault_storage", "Reconnection attempt %d/%d failed: %s", attempt + 1, retries + 1,
                           last_error.c_str());
  }
  fail_round(last_error);
  throw IgnorableConnectionException("Failed to connect to PostgreSQL: " + last_error);
}

template <typename Fn>
auto PgFaultStorage::run_in_transaction(const char * what,
                                        Fn && fn) const -> decltype(fn(std::declval<pqxx::work &>())) {
  using Result = decltype(fn(std::declval<pqxx::work &>()));

  for (int attempt = 0;; ++attempt) {
    ensure_connection();

    try {
      pqxx::work tx(*db_conn_);  // inside the try: BEGIN is a network operation too
      if (!schema_ready_) {
        create_schema(tx);
      }
      if constexpr (std::is_void_v<Result>) {
        fn(tx);
        tx.commit();
        schema_ready_ = true;
        return;
      } else {
        Result result = fn(tx);
        tx.commit();
        schema_ready_ = true;
        return result;
      }
    } catch (const pqxx::broken_connection & e) {
      const std::string error = redact(e.what());
      if (attempt >= max_retries_) {
        fail_round(error);
        throw FaultStorage::IgnorableConnectionException(std::string(what) +
                                                         " failed to connect to PostgreSQL: " + error);
      }
      // No sleep here: ensure_connection spaces its own attempts.
      RCUTILS_LOG_WARN_NAMED("postgres_fault_storage", "%s lost the connection (%s); retrying transaction %d/%d", what,
                             error.c_str(), attempt + 1, max_retries_);
    } catch (const std::exception & e) {
      // There are other types of error that "hide" connection issues
      // Before throwing a runtime_error, check if the connection is open
      const std::string error = redact(e.what());
      if (!db_conn_ || !db_conn_->is_open()) {
        fail_round(error);
        throw FaultStorage::IgnorableConnectionException(std::string(what) +
                                                         " failed to connect to PostgreSQL: " + error);
      }
      throw std::runtime_error(std::string(what) + " PostgreSQL error: " + error);
    }
  }
}

void PgFaultStorage::set_debounce_config(const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);
  config_ = config;
}

DebounceConfig PgFaultStorage::get_debounce_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return config_;
}

void PgFaultStorage::create_schema(pqxx::work & tx) const {
  execute(tx, R"(
      CREATE TABLE IF NOT EXISTS faults (
        fault_code TEXT PRIMARY KEY,
        severity INTEGER NOT NULL,
        description TEXT NOT NULL,
        first_occurred_ns BIGINT NOT NULL,
        last_occurred_ns BIGINT NOT NULL,
        occurrence_count BIGINT NOT NULL,
        status TEXT NOT NULL,
        reporting_sources TEXT NOT NULL,
        debounce_counter INTEGER NOT NULL DEFAULT 0,
        last_failed_ns BIGINT NOT NULL DEFAULT 0,
        last_passed_ns BIGINT NOT NULL DEFAULT 0,
        confirmed_at_ns BIGINT NOT NULL DEFAULT 0
      );
    )");

  // Create snapshots table for storing topic data captured when faults are confirmed
  execute(tx, R"(
      CREATE TABLE IF NOT EXISTS snapshots (
        id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY,
        fault_code TEXT NOT NULL,
        topic TEXT NOT NULL,
        message_type TEXT NOT NULL,
        data TEXT NOT NULL,
        captured_at_ns BIGINT NOT NULL,
        capture_id BIGINT NOT NULL DEFAULT 0
      );
      CREATE INDEX IF NOT EXISTS idx_snapshots_fault_code ON snapshots(fault_code);
      CREATE INDEX IF NOT EXISTS idx_snapshots_fault_topic ON snapshots(fault_code, topic);)");

  // Create freeze_frames table: one compact JSON dict of captured topic values per fault
  // code. Unlike snapshots, freeze frames are keyed by fault_code and are NOT removed on
  // clear_fault, so the confirmed-state record is retained after acknowledgement.
  execute(tx, R"(
      CREATE TABLE IF NOT EXISTS freeze_frames (
        fault_code TEXT PRIMARY KEY,
        data TEXT NOT NULL,
        captured_at_ns BIGINT NOT NULL
      );
    )");

  // Create near_misses table: append-only series of FAILED reports that moved the debounce
  // counter without confirming the fault. One row per qualifying report, never updated in
  // place, and NOT removed on clear_fault - acknowledging a fault cycle must not erase how
  // often that code approached confirmation. Bounded per fault code by the caller-supplied
  // limit, evicting the oldest rows first.
  execute(tx, R"(
      CREATE TABLE IF NOT EXISTS near_misses (
        id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY,
        fault_code TEXT NOT NULL,
        occurred_at_ns BIGINT NOT NULL,
        debounce_counter INTEGER NOT NULL,
        confirmation_threshold INTEGER NOT NULL,
        severity INTEGER NOT NULL,
        source_id TEXT NOT NULL,
        resulting_status TEXT NOT NULL DEFAULT ''
      );
      CREATE INDEX IF NOT EXISTS idx_near_misses_fault_code ON near_misses(fault_code, id);
    )");

  // Create rosbag_files table. One row = one LINK (a fault claiming a recording):
  // several faults of a burst link to one bag, and one fault links to several bags
  // over time. Bytes belong to file_path, not to the row.
  execute(tx, R"(
      CREATE TABLE IF NOT EXISTS rosbag_files (
        id BIGINT PRIMARY KEY GENERATED ALWAYS AS IDENTITY,
        fault_code TEXT NOT NULL,
        recording_id TEXT NOT NULL DEFAULT '',
        file_path TEXT NOT NULL,
        format TEXT NOT NULL,
        duration_sec DOUBLE PRECISION NOT NULL,
        size_bytes BIGINT NOT NULL,
        created_at_ns BIGINT NOT NULL
      );
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_fault_code ON rosbag_files(fault_code);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_created_at ON rosbag_files(created_at_ns);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_fault_created ON rosbag_files(fault_code, created_at_ns, id);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_recording ON rosbag_files(recording_id);
      CREATE INDEX IF NOT EXISTS idx_rosbag_files_path ON rosbag_files(file_path);
      CREATE UNIQUE INDEX IF NOT EXISTS idx_rosbag_files_fault_path ON rosbag_files(fault_code, file_path);
    )");

  // CREATE TABLE IF NOT EXISTS keeps a table of another layout. Check the columns so that startup fails.
  execute(tx, R"(
      SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, occurrence_count, status,
             reporting_sources, debounce_counter, last_failed_ns, last_passed_ns, confirmed_at_ns
        FROM faults LIMIT 0;
      SELECT id, fault_code, topic, message_type, data, captured_at_ns, capture_id FROM snapshots LIMIT 0;
      SELECT fault_code, data, captured_at_ns FROM freeze_frames LIMIT 0;
      SELECT id, fault_code, occurred_at_ns, debounce_counter, confirmation_threshold, severity, source_id,
             resulting_status
        FROM near_misses LIMIT 0;
      SELECT id, fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns
        FROM rosbag_files LIMIT 0;
    )");
}

std::vector<std::string> PgFaultStorage::parse_json_array(const std::string & json_str) {
  std::vector<std::string> result;

  // Simple JSON array parser for ["a", "b", "c"] format
  if (json_str.size() < 2 || json_str.front() != '[' || json_str.back() != ']') {
    if (!json_str.empty()) {
      RCUTILS_LOG_WARN_NAMED("postgres_fault_storage", "Malformed JSON array in database: '%s'", json_str.c_str());
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

std::string PgFaultStorage::serialize_json_array(const std::vector<std::string> & vec) {
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

bool PgFaultStorage::report_fault_event(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                        const std::string & description, const std::string & source_id,
                                        const rclcpp::Time & timestamp, const DebounceConfig & config) {
  std::lock_guard<std::mutex> lock(mutex_);

  // Only a FAILED report can write two rows (fault row and near-miss row) and
  // both have to land together: written separately, a failure on the second would
  // leave the debounce counter already advanced, so the caller's retry would
  // advance it a second time and the near miss it retried for would still be
  // missing from the series. run_in_transaction commits both rows together.
  return run_in_transaction("report_fault_event", [&](pqxx::work & tx) {
    return report_fault_event_locked(fault_code, event_type, severity, description, source_id, timestamp, config, tx);
  });
}

bool PgFaultStorage::report_fault_event_locked(const std::string & fault_code, uint8_t event_type, uint8_t severity,
                                               const std::string & description, const std::string & source_id,
                                               const rclcpp::Time & timestamp, const DebounceConfig & config,
                                               pqxx::work & tx) {
  int64_t timestamp_ns = timestamp.nanoseconds();
  const bool is_failed = (event_type == EventType::EVENT_FAILED);

  // No row lock needed: mutex_ serializes every call on this connection, and one database belongs to exactly one
  // fault manager.
  // Check if fault exists
  auto res = execute(tx,
                     "SELECT severity, occurrence_count, reporting_sources, status, debounce_counter, confirmed_at_ns, "
                     "first_occurred_ns FROM faults WHERE fault_code = $1",
                     fault_code);

  if (!res.empty()) {
    // Fault exists - update it
    int existing_severity = res[0]["severity"].as<int>();
    int64_t existing_count = res[0]["occurrence_count"].as<int64_t>();
    std::string sources_json = res[0]["reporting_sources"].as<std::string>();
    std::string current_status = res[0]["status"].as<std::string>();
    int32_t debounce_counter = res[0]["debounce_counter"].as<int32_t>();
    int64_t confirmed_at_ns = res[0]["confirmed_at_ns"].as<int64_t>();
    int64_t first_occurred_ns = res[0]["first_occurred_ns"].as<int64_t>();

    // Bring a runaway counter persisted by an older build (the bug this fixes) back into range on
    // first touch; this also keeps the +1/-1 below overflow-safe. The counter is local to this call.
    debounce_counter = clamp_debounce_counter(debounce_counter, config);

    // CLEARED faults can be reactivated by FAILED events
    bool is_reactivation = false;
    if (current_status == ros2_medkit_msgs::msg::Fault::STATUS_CLEARED) {
      if (!is_failed) {
        // PASSED events for CLEARED faults are ignored
        return false;
      }
      // FAILED event reactivates - reset debounce counter to 0 so FAILED branch
      // decrements it to -1, then reuse the existing FAILED logic below. Also
      // reset first_occurred: this is a new outage cycle, not a continuation
      // of the one that just cleared.
      debounce_counter = 0;
      first_occurred_ns = timestamp_ns;
      is_reactivation = true;
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

      // Increment count with saturation - only on a genuine new occurrence (reactivation
      // after CLEARED). A still-active fault being re-reported (level-triggered poller,
      // or debounce building toward confirmation) is the same continuous occurrence.
      int64_t new_count = existing_count;
      if (is_reactivation && new_count < std::numeric_limits<uint32_t>::max()) {
        ++new_count;
      }

      // Decrement towards confirmation, clamped to the thresholds.
      debounce_counter = clamp_debounce_counter(debounce_counter - 1, config);

      // CRITICAL bypasses debounce; otherwise the shared state machine decides (with hysteresis).
      std::string new_status;
      if (config.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
        new_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
      } else {
        new_status = compute_debounce_status(debounce_counter, current_status, config);
      }

      // Record the confirmation instant on the transition into CONFIRMED (also
      // on a reactivation that re-confirms); an already-confirmed fault keeps
      // its original timestamp.
      if (new_status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED &&
          current_status != ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED) {
        confirmed_at_ns = timestamp_ns;
      }

      if (description.empty()) {
        execute(tx,
                "UPDATE faults SET severity = $1, last_occurred_ns = $2, last_failed_ns = $3, occurrence_count = $4, "
                "reporting_sources = $5, status = $6, debounce_counter = $7, confirmed_at_ns = $8, "
                "first_occurred_ns = $9 WHERE fault_code = $10",
                new_severity, timestamp_ns, timestamp_ns, new_count, serialize_json_array(sources), new_status,
                debounce_counter, confirmed_at_ns, first_occurred_ns, fault_code);
      } else {
        execute(tx,
                "UPDATE faults SET severity = $1, description = $2, last_occurred_ns = $3, last_failed_ns = $4, "
                "occurrence_count = $5, reporting_sources = $6, status = $7, debounce_counter = $8, "
                "confirmed_at_ns = $9, first_occurred_ns = $10 WHERE fault_code = $11",
                new_severity, description, timestamp_ns, timestamp_ns, new_count, serialize_json_array(sources),
                new_status, debounce_counter, confirmed_at_ns, first_occurred_ns, fault_code);
      }

      if (is_near_miss(true, new_status)) {
        record_near_miss_locked(fault_code, timestamp_ns, debounce_counter, config, severity, source_id, new_status,
                                tx);
      }
    } else {
      // PASSED event - increment towards healing, clamped to the thresholds.
      debounce_counter = clamp_debounce_counter(debounce_counter + 1, config);

      std::string new_status = compute_debounce_status(debounce_counter, current_status, config);

      // last_occurred_ns is deliberately NOT touched: a PASSED event is the fault
      // ENDING, not occurring. Bumping it made a long-stale CONFIRMED fault look
      // freshly active. The PASSED instant is recorded in last_passed_ns.
      execute(tx, "UPDATE faults SET last_passed_ns = $1, status = $2, debounce_counter = $3 WHERE fault_code = $4",
              timestamp_ns, new_status, debounce_counter, fault_code);
    }

    return is_reactivation;  // Reactivation treated as new occurrence for event publishing
  }

  // New fault - only create for FAILED events
  if (!is_failed) {
    return false;  // PASSED event for non-existent fault is ignored
  }

  // Determine initial status based on debounce logic (shared with the in-memory backend).
  std::string initial_status;
  constexpr int32_t initial_counter = -1;  // First FAILED event sets counter to -1
  if (config.critical_immediate_confirm && severity == ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL) {
    initial_status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  } else {
    initial_status = compute_debounce_status(initial_counter, "", config);
  }

  const bool confirmed_now = initial_status == ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
  execute(
      tx,
      "INSERT INTO faults (fault_code, severity, description, first_occurred_ns, last_occurred_ns, occurrence_count, "
      "status, reporting_sources, debounce_counter, last_failed_ns, last_passed_ns, confirmed_at_ns) "
      "VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12)",
      fault_code, static_cast<int>(severity), description, timestamp_ns, timestamp_ns, 1 /* occurrence_count = 1 */,
      initial_status, serialize_json_array({source_id}), initial_counter, timestamp_ns, 0,
      confirmed_now ? timestamp_ns : 0);

  if (is_near_miss(true, initial_status)) {
    record_near_miss_locked(fault_code, timestamp_ns, initial_counter, config, severity, source_id, initial_status, tx);
  }

  return true;  // New fault created
}

std::vector<ros2_medkit_msgs::msg::Fault> PgFaultStorage::list_faults(bool filter_by_severity, uint8_t severity,
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
  const std::vector<std::string> status_filter_vector(status_filter.begin(), status_filter.end());

  // Build query
  std::string sql =
      "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, occurrence_count, status, "
      "reporting_sources, last_passed_ns FROM faults WHERE status = ANY($1) ";
  if (filter_by_severity) {
    sql += " AND severity = $2";
  }

  return run_in_transaction("list_faults", [&](pqxx::work & tx) {
    auto res = filter_by_severity ? execute(tx, sql, status_filter_vector, static_cast<int>(severity))
                                  : execute(tx, sql, status_filter_vector);

    std::vector<ros2_medkit_msgs::msg::Fault> result;
    for (const auto & r : res) {
      ros2_medkit_msgs::msg::Fault fault;
      fault.fault_code = r["fault_code"].as<std::string>();
      fault.severity = static_cast<uint8_t>(r["severity"].as<int>());
      fault.description = r["description"].as<std::string>();
      fault.first_occurred = rclcpp::Time(r["first_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.last_occurred = rclcpp::Time(r["last_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.occurrence_count = static_cast<uint32_t>(r["occurrence_count"].as<int64_t>());
      fault.status = r["status"].as<std::string>();
      fault.reporting_sources = parse_json_array(r["reporting_sources"].as<std::string>());
      fault.last_passed = rclcpp::Time(r["last_passed_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      result.push_back(fault);
    }
    return result;
  });
}

std::optional<ros2_medkit_msgs::msg::Fault> PgFaultStorage::get_fault(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_fault", [&](pqxx::work & tx) -> std::optional<ros2_medkit_msgs::msg::Fault> {
    auto res = execute(tx,
                       "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
                       "occurrence_count, status, reporting_sources, last_passed_ns FROM faults WHERE fault_code = $1",
                       fault_code);
    if (res.empty()) {
      return std::nullopt;
    }
    auto r = res[0];
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = r["fault_code"].as<std::string>();
    fault.severity = static_cast<uint8_t>(r["severity"].as<int>());
    fault.description = r["description"].as<std::string>();
    fault.first_occurred = rclcpp::Time(r["first_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
    fault.last_occurred = rclcpp::Time(r["last_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
    fault.occurrence_count = static_cast<uint32_t>(r["occurrence_count"].as<int64_t>());
    fault.status = r["status"].as<std::string>();
    fault.reporting_sources = parse_json_array(r["reporting_sources"].as<std::string>());
    fault.last_passed = rclcpp::Time(r["last_passed_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
    return fault;
  });
}

bool PgFaultStorage::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("clear_fault", [&](pqxx::work & tx) {
    // The near_misses rows for this code are deliberately left alone. Clearing acknowledges one
    // fault cycle; the record of how often the code approached confirmation spans cycles and
    // cannot be reconstructed once deleted.

    // Acknowledging a fault drops its value snapshots, unless a history was asked
    // for: with recordings retained past a clear, deleting the readings that go with
    // them leaves a fault holding bags whose matching values are gone.
    if (!retain_snapshots_on_clear_) {
      execute(tx, "DELETE FROM snapshots WHERE fault_code = $1", fault_code);
    }

    auto res = execute(tx, "UPDATE faults SET status = $1 WHERE fault_code = $2",
                       ros2_medkit_msgs::msg::Fault::STATUS_CLEARED, fault_code);
    return res.affected_rows() > 0;
  });
}

std::vector<std::string> PgFaultStorage::reclassify_healed_as_cleared() {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("reclassify_healed_as_cleared", [&](pqxx::work & tx) {
    // Collect the codes that will flip first so the caller can audit each one. The
    // SELECT predicate mirrors the UPDATE exactly, and both run in the same
    // transaction, so the returned list matches the rows actually reclassified below.
    auto res =
        execute(tx, "SELECT fault_code FROM faults WHERE status = $1", ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    std::vector<std::string> reclassified;
    for (const auto & r : res) {
      reclassified.push_back(r["fault_code"].as<std::string>());
    }
    if (reclassified.empty()) {
      return reclassified;
    }
    // Drop snapshots for the affected faults so a reclassified row matches CLEARED semantics.
    // clear_fault is not the only place that takes a fault's readings, so retain_snapshots_on_clear_
    // has to reach here too: otherwise the setting holds until the next restart and then the
    // reclassification deletes exactly what it was set to keep.
    if (!retain_snapshots_on_clear_) {
      execute(tx, "DELETE FROM snapshots WHERE fault_code IN (SELECT fault_code FROM faults WHERE status = $1)",
              ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    }
    execute(tx, "UPDATE faults SET status = $1 WHERE status = $2", ros2_medkit_msgs::msg::Fault::STATUS_CLEARED,
            ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    return reclassified;
  });
}

size_t PgFaultStorage::size() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("size", [&](pqxx::work & tx) {
    auto res = execute(tx, "SELECT COUNT(*) AS sz FROM faults");
    return res.empty() ? size_t{0} : static_cast<size_t>(res[0]["sz"].as<int64_t>());
  });
}

bool PgFaultStorage::contains(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("contains", [&](pqxx::work & tx) {
    auto res = execute(tx, "SELECT 1 FROM faults WHERE fault_code = $1 LIMIT 1", fault_code);
    return !res.empty();
  });
}

std::vector<std::string> PgFaultStorage::check_time_based_confirmation(const rclcpp::Time & current_time) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (config_.auto_confirm_after_sec <= 0.0) {
    return {};  // Time-based confirmation disabled
  }

  const int64_t current_ns = current_time.nanoseconds();
  const int64_t cutoff_ns = current_ns - static_cast<int64_t>(config_.auto_confirm_after_sec * 1e9);

  return run_in_transaction("check_time_based_confirmation", [&](pqxx::work & tx) {
    // Collect the codes that will flip first so the caller can audit each one. The
    // SELECT predicate mirrors the UPDATE exactly, and both run under the same lock,
    // so the returned list matches the rows actually confirmed below.
    auto res =
        execute(tx, "SELECT fault_code FROM faults WHERE status = $1 AND last_failed_ns <= $2 AND last_failed_ns > 0 ",
                ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED, cutoff_ns);

    std::vector<std::string> confirmed;
    for (const auto & r : res) {
      confirmed.push_back(r["fault_code"].as<std::string>());
    }

    if (!confirmed.empty()) {
      execute(tx,
              "UPDATE faults SET status = $1, confirmed_at_ns = $2 WHERE status = $3 AND last_failed_ns <= $4 "
              "AND last_failed_ns > 0",
              ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED, current_ns,
              ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED, cutoff_ns);
    }
    return confirmed;
  });
}

void PgFaultStorage::set_max_snapshots_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_snapshots_per_fault_ = max_count;
}

void PgFaultStorage::set_retain_snapshots_on_clear(bool retain) {
  std::lock_guard<std::mutex> lock(mutex_);
  retain_snapshots_on_clear_ = retain;
}

bool PgFaultStorage::retains_snapshots_on_clear() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return retain_snapshots_on_clear_;
}

void PgFaultStorage::store_snapshot(const SnapshotData & snapshot) {
  store_snapshots({snapshot});
}

void PgFaultStorage::store_snapshots(const std::vector<SnapshotData> & snapshots) {
  if (snapshots.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  const std::string & fault_code = snapshots.front().fault_code;

  // One transaction for the whole capture: a capture is all-or-nothing, and the
  // old row-at-a-time path could leave a confirmation's values half stored.
  run_in_transaction("store_snapshots", [&](pqxx::work & tx) {
    for (const auto & snapshot : snapshots) {
      execute(
          tx,
          "INSERT INTO snapshots (fault_code, topic, message_type, data, captured_at_ns, capture_id) VALUES ($1, $2, "
          "$3, $4, $5, $6)",
          snapshot.fault_code, snapshot.topic, snapshot.message_type, snapshot.data, snapshot.captured_at_ns,
          snapshot.capture_id);
    }
    if (max_snapshots_per_fault_ > 0) {
      // Trim whole capture sets, oldest first, until the fault fits. The old rule
      // counted rows and rejected the NEW row once full, so a capture straddling
      // the cap was stored in part - some topics present, the rest silently gone,
      // indistinguishable from "that topic was not publishing". Keep-newest also
      // stops this cap from opposing the rosbag one.
      //
      // The newest capture is never trimmed: if it alone exceeds the cap, the cap
      // is smaller than this fault's topic count and tearing it would be the very
      // thing being fixed.
      auto res =
          execute(tx, "SELECT MAX(capture_id) AS max_capture_id FROM snapshots WHERE fault_code = $1", fault_code);
      int64_t newest_capture = 0;
      if (!res.empty() && !res[0]["max_capture_id"].is_null()) {
        newest_capture = res[0]["max_capture_id"].as<int64_t>();
      }
      while (true) {
        auto count_res = execute(tx, "SELECT COUNT(*) AS sz FROM snapshots WHERE fault_code = $1", fault_code);
        if (count_res.empty() || count_res[0]["sz"].as<size_t>() <= max_snapshots_per_fault_) {
          break;
        }
        auto trim_res = execute(tx,
                                "DELETE FROM snapshots WHERE fault_code = $1 AND capture_id = "
                                "(SELECT MIN(capture_id) FROM snapshots WHERE fault_code = $1) "
                                "AND capture_id <> $2",
                                fault_code, newest_capture);
        if (trim_res.affected_rows() == 0) {
          break;
        }
      }
    }
  });
}

std::vector<SnapshotData> PgFaultStorage::get_snapshots(const std::string & fault_code,
                                                        const std::string & topic_filter) const {
  std::lock_guard<std::mutex> lock(mutex_);

  std::string sql =
      "SELECT fault_code, topic, message_type, data, captured_at_ns, capture_id FROM snapshots WHERE fault_code = $1";
  if (!topic_filter.empty()) {
    sql += " AND topic = $2";
  }
  // capture_id before the timestamp: the rows of one capture are written seconds
  // apart under load and their timestamps interleave with a neighbouring capture's,
  // so ordering by time alone splits a set the reader then cannot regroup.
  sql += " ORDER BY capture_id DESC, captured_at_ns DESC";

  return run_in_transaction("get_snapshots", [&](pqxx::work & tx) {
    auto res = topic_filter.empty() ? execute(tx, sql, fault_code) : execute(tx, sql, fault_code, topic_filter);

    std::vector<SnapshotData> result;
    for (const auto & r : res) {
      SnapshotData snapshot;
      snapshot.fault_code = r["fault_code"].as<std::string>();
      snapshot.topic = r["topic"].as<std::string>();
      snapshot.message_type = r["message_type"].as<std::string>();
      snapshot.data = r["data"].as<std::string>();
      snapshot.captured_at_ns = r["captured_at_ns"].as<int64_t>();
      snapshot.capture_id = r["capture_id"].as<int64_t>();
      result.push_back(std::move(snapshot));
    }
    return result;
  });
}

int64_t PgFaultStorage::get_max_capture_id() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_max_capture_id", [&](pqxx::work & tx) {
    // Global, not per fault: the counter that mints these is global, and seeding it
    // below any id already on disk is what lets a restart evict the capture it just
    // wrote. NULL on an empty table reads back as 0.
    auto res = execute(tx, "SELECT COALESCE(MAX(capture_id), 0) AS max_capture FROM snapshots");
    return res.empty() ? int64_t{0} : res[0]["max_capture"].as<int64_t>();
  });
}

void PgFaultStorage::store_freeze_frame(const FreezeFrameData & frame) {
  std::lock_guard<std::mutex> lock(mutex_);

  run_in_transaction("store_freeze_frame", [&](pqxx::work & tx) {
    // Keyed by fault_code (PRIMARY KEY): a re-confirm replaces the previous frame.
    execute(tx,
            "INSERT INTO freeze_frames (fault_code, data, captured_at_ns) VALUES ($1, $2, $3) ON CONFLICT(fault_code) "
            "DO "
            "UPDATE SET data = $2, captured_at_ns = $3 ",
            frame.fault_code, frame.data, frame.captured_at_ns);
  });
}

std::optional<FreezeFrameData> PgFaultStorage::get_freeze_frame(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_freeze_frame", [&](pqxx::work & tx) -> std::optional<FreezeFrameData> {
    auto res =
        execute(tx, "SELECT fault_code, data, captured_at_ns FROM freeze_frames WHERE fault_code = $1", fault_code);
    if (res.empty()) {
      return std::nullopt;
    }
    FreezeFrameData frame;
    frame.fault_code = res[0]["fault_code"].as<std::string>();
    frame.data = res[0]["data"].as<std::string>();
    frame.captured_at_ns = res[0]["captured_at_ns"].as<int64_t>();
    return frame;
  });
}

size_t PgFaultStorage::set_max_near_misses_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_near_misses_per_fault_ = max_count;

  // 0 and any bound past what PostgreSQL can hold both mean "keep everything". Binding SIZE_MAX
  // straight into an int64 makes it -1, and every row then compares as beyond the bound, so the
  // idiomatic spelling of "no limit" would empty the table.
  if (max_count == 0 || max_count > static_cast<size_t>(std::numeric_limits<int64_t>::max())) {
    return 0;  // Unlimited
  }

  // Apply the bound to what is already in the database. Without this, a database that grew under
  // a larger bound (or none) stays over the new bound until each fault code happens to record
  // another near miss - and a code that never does keeps its rows for good.
  return run_in_transaction("set_max_near_misses_per_fault", [&](pqxx::work & tx) {
    auto res = execute(tx,
                       "DELETE FROM near_misses WHERE id IN ("
                       "SELECT id FROM (SELECT id, ROW_NUMBER() OVER "
                       "(PARTITION BY fault_code ORDER BY id DESC) AS rn FROM near_misses) AS ranked "
                       "WHERE rn > $1)",
                       static_cast<int64_t>(max_count));
    // Returned rather than logged: the storage layer has no logger, and a bound applied by mistake
    // deletes history that cannot be recovered, so the caller has to be able to report it.
    return static_cast<size_t>(res.affected_rows());
  });
}

void PgFaultStorage::record_near_miss_locked(const std::string & fault_code, int64_t occurred_at_ns,
                                             int32_t debounce_counter, const DebounceConfig & config, uint8_t severity,
                                             const std::string & source_id, const std::string & resulting_status,
                                             pqxx::work & tx) {
  execute(tx,
          "INSERT INTO near_misses (fault_code, occurred_at_ns, debounce_counter, confirmation_threshold, severity, "
          "source_id, resulting_status) VALUES ($1, $2, $3, $4, $5, $6, $7)",
          fault_code, occurred_at_ns, debounce_counter, config.confirmation_threshold, static_cast<int>(severity),
          source_id, resulting_status);

  if (max_near_misses_per_fault_ == 0 ||
      max_near_misses_per_fault_ > static_cast<size_t>(std::numeric_limits<int64_t>::max())) {
    return;  // Unlimited
  }

  // Evict oldest-first, keeping the newest max_near_misses_per_fault_ rows - the same direction as
  // the snapshot and rosbag caps. A series frozen at boot answers nothing about whether the rate
  // of near misses is changing.
  //
  // "Oldest" means earliest ARRIVAL (id), not earliest occurred_at_ns. Reporters carry their own
  // clocks, so a report can arrive with a timestamp behind one already stored; ordering eviction
  // by timestamp would then drop the row that was just appended and make the two backends, which
  // append in arrival order, disagree on the same input.
  execute(tx,
          "DELETE FROM near_misses WHERE fault_code = $1 AND id NOT IN "
          "(SELECT id FROM near_misses WHERE fault_code = $1 ORDER BY id DESC LIMIT $2)",
          fault_code, static_cast<int64_t>(max_near_misses_per_fault_));
}

std::vector<NearMissRecord> PgFaultStorage::get_near_misses(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_near_misses", [&](pqxx::work & tx) {
    auto res = execute(tx,
                       "SELECT fault_code, occurred_at_ns, debounce_counter, confirmation_threshold, "
                       "severity, source_id, resulting_status FROM near_misses WHERE fault_code = $1 "
                       "ORDER BY id ASC",
                       fault_code);

    std::vector<NearMissRecord> result;
    for (const auto & r : res) {
      NearMissRecord record;
      record.fault_code = r["fault_code"].as<std::string>();
      record.occurred_at_ns = r["occurred_at_ns"].as<int64_t>();
      record.debounce_counter = r["debounce_counter"].as<int32_t>();
      record.confirmation_threshold = r["confirmation_threshold"].as<int32_t>();
      record.severity = static_cast<uint8_t>(r["severity"].as<int>());
      record.source_id = r["source_id"].as<std::string>();
      record.resulting_status = r["resulting_status"].as<std::string>();
      result.push_back(std::move(record));
    }
    return result;
  });
}

void PgFaultStorage::set_max_rosbags_per_fault(size_t max_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  max_rosbags_per_fault_ = max_count;
}

void PgFaultStorage::store_rosbag_file(const RosbagFileInfo & info) {
  // Routed through the batch path deliberately: with a per-fault cap a single
  // store is insert + trim, i.e. several statements that must share one
  // transaction and one post-commit unlink pass.
  store_rosbag_files({info});
}

void PgFaultStorage::store_rosbag_files(const std::vector<RosbagFileInfo> & infos) {
  if (infos.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // One transaction for the whole burst: a crash mid-store must not leave some
  // faults of the shared recording without their lookup row. Evicted bags are
  // unlinked only after COMMIT - a ROLLBACK (PostgreSQL transaction abort) resurrects the rows, which must keep
  // pointing at bags that still exist.
  const std::vector<std::string> evicted = run_in_transaction("store_rosbag_files", [&](pqxx::work & tx) {
    std::vector<std::string> paths;
    for (const auto & info : infos) {
      auto evicted_paths = store_rosbag_file_locked(info, tx);
      paths.insert(paths.end(), std::make_move_iterator(evicted_paths.begin()),
                   std::make_move_iterator(evicted_paths.end()));
    }
    return paths;
  });

  // Referencing is re-checked on the committed state, not on the state each
  // eviction saw: two faults of one burst can link the same bag, and a row-by-row
  // check inside the loop would find it still held by a sibling that a later
  // iteration then evicts, leaking the directory.
  const std::set<std::string> unique_paths(evicted.begin(), evicted.end());
  for (const auto & path : unique_paths) {
    try {
      if (path_referenced(path)) {
        continue;
      }
    } catch (const std::exception & e) {
      continue;  // The rows are already committed, so keep the directory.
    }

    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
}

std::vector<std::string> PgFaultStorage::store_rosbag_file_locked(const RosbagFileInfo & info, pqxx::work & tx) {
  RosbagFileInfo row = info;
  if (row.recording_id.empty()) {
    row.recording_id = rosbag_recording_id(row.file_path);
  }

  // Upserts on idx_rosbag_files_fault_path, i.e. on the (fault, recording) link.
  // Re-storing the SAME link refreshes it; a link to a DIFFERENT recording is a new
  // row now, which is the feature. Nothing is unlinked here - byte lifetime is the
  // cap's business below, and the caller's, after the commit.
  //
  // ON CONFLICT DO UPDATE keeps the row's identity, so a refresh does not move the
  // link to the end of the id order. Every read below breaks created_at_ns ties by
  // id, and the in-memory backend keeps its sequence number across a refresh, so a
  // delete-and-reinsert would put the two backends in a different order for a
  // re-stored row inside a tie group.
  execute(tx,
          "INSERT INTO rosbag_files "
          "(fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns) "
          "VALUES ($1, $2, $3, $4, $5, $6, $7) "
          "ON CONFLICT (fault_code, file_path) DO UPDATE SET "
          "recording_id = EXCLUDED.recording_id, format = EXCLUDED.format, "
          "duration_sec = EXCLUDED.duration_sec, size_bytes = EXCLUDED.size_bytes, "
          "created_at_ns = EXCLUDED.created_at_ns",
          row.fault_code, row.recording_id, row.file_path, row.format, row.duration_sec,
          static_cast<int64_t>(row.size_bytes), row.created_at_ns);

  if (max_rosbags_per_fault_ == 0) {
    return {};  // unlimited per fault; only the global byte quota bounds this
  }

  auto res =
      execute(tx,
              "DELETE FROM rosbag_files WHERE fault_code = $1 AND id NOT IN "
              "(SELECT id FROM rosbag_files WHERE fault_code = $1 ORDER BY created_at_ns DESC, id DESC LIMIT $2) "
              "RETURNING file_path",
              row.fault_code, static_cast<int64_t>(max_rosbags_per_fault_));

  std::set<std::string> evicted;
  for (const auto & r : res) {
    evicted.insert(r["file_path"].as<std::string>());
  }
  return {evicted.begin(), evicted.end()};
}
namespace {

/// Shared projection so every rosbag read decodes the same column order.
RosbagFileInfo read_rosbag_row(const pqxx::row & r) {
  RosbagFileInfo info;
  info.fault_code = r["fault_code"].as<std::string>();
  info.recording_id = r["recording_id"].as<std::string>();
  info.file_path = r["file_path"].as<std::string>();
  info.format = r["format"].as<std::string>();
  info.duration_sec = r["duration_sec"].as<double>();
  info.size_bytes = static_cast<size_t>(r["size_bytes"].as<int64_t>());
  info.created_at_ns = r["created_at_ns"].as<int64_t>();
  return info;
}

constexpr const char * kRosbagColumns =
    "fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns";

}  // namespace

std::vector<RosbagFileInfo> PgFaultStorage::get_rosbag_files(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_rosbag_files", [&](pqxx::work & tx) {
    auto res = execute(tx,
                       std::string("SELECT ") + kRosbagColumns +
                           " FROM rosbag_files WHERE fault_code = $1 ORDER BY created_at_ns DESC, id DESC",
                       fault_code);

    std::vector<RosbagFileInfo> result;
    for (const auto & r : res) {
      result.push_back(read_rosbag_row(r));
    }
    return result;
  });
}

std::vector<RosbagFileInfo> PgFaultStorage::get_rosbag_files_by_recording(const std::string & recording_id) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_rosbag_files_by_recording", [&](pqxx::work & tx) {
    auto res = execute(tx,
                       std::string("SELECT ") + kRosbagColumns +
                           " FROM rosbag_files WHERE recording_id = $1 ORDER BY fault_code ASC",
                       recording_id);

    std::vector<RosbagFileInfo> result;
    for (const auto & r : res) {
      result.push_back(read_rosbag_row(r));
    }
    return result;
  });
}

size_t PgFaultStorage::delete_rosbag_recording(const std::string & recording_id) {
  // An empty id is not a recording that happens to be unnamed, it is a row whose
  // backfill did not finish. Matching on it would take every such row of every
  // fault with one DELETE, and evict_bags_over_quota calls this with whatever the
  // row held.
  if (recording_id.empty()) {
    return 0;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  auto outcome =
      run_in_transaction("delete_rosbag_recording", [&](pqxx::work & tx) -> std::pair<std::set<std::string>, size_t> {
        // DELETE ... RETURNING reads and deletes the rows in one statement.
        auto res = execute(tx, "DELETE FROM rosbag_files WHERE recording_id = $1 RETURNING file_path", recording_id);
        std::set<std::string> paths;
        for (const auto & r : res) {
          paths.insert(r["file_path"].as<std::string>());
        }
        return {paths, static_cast<size_t>(res.affected_rows())};
      });

  for (const auto & path : outcome.first) {
    try {
      if (path_referenced(path)) {
        continue;  // another recording writes into the same directory - leave it
      }
    } catch (const std::exception & e) {
      continue;  // The rows are already committed, so keep the directory.
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
  return outcome.second;
}

std::optional<RosbagFileInfo> PgFaultStorage::get_rosbag_file(const std::string & fault_code) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_rosbag_file", [&](pqxx::work & tx) -> std::optional<RosbagFileInfo> {
    // ORDER BY is load-bearing now that a fault can hold several recordings. Without
    // it PostgreSQL may return any matching row, so the fault detail and the download
    // would serve an arbitrary recording - non-deterministically, which no test
    // catches reliably. id breaks the tie because a burst stamps one created_at_ns
    // across all its rows.
    auto res = execute(tx,
                       "SELECT fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns "
                       "FROM rosbag_files WHERE fault_code = $1 ORDER BY created_at_ns DESC, id DESC LIMIT 1",
                       fault_code);
    if (res.empty()) {
      return std::nullopt;
    }
    return read_rosbag_row(res[0]);
  });
}

bool PgFaultStorage::delete_rosbag_file(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(mutex_);

  auto outcome =
      run_in_transaction("delete_rosbag_file", [&](pqxx::work & tx) -> std::pair<std::set<std::string>, bool> {
        // Row first, file after. Deleting the bag before its row would, on a failed
        // DELETE, leave a surviving row pointing at a directory that is gone -
        // unreadable for good, and still charged against the storage quota, which sums
        // rows. This way the worst case is an orphaned directory instead.
        auto res = execute(tx, "DELETE FROM rosbag_files WHERE fault_code = $1 RETURNING file_path", fault_code);
        // Every path, not the first one: a fault holds as many recordings as its cap
        // allows, and reading one row would unlink one bag and leak the rest - rows gone,
        // directories left behind, uncounted by a quota that sums rows.
        std::set<std::string> paths;
        for (const auto & r : res) {
          paths.insert(r["file_path"].as<std::string>());
        }
        return {paths, res.affected_rows() > 0};
      });

  for (const auto & path : outcome.first) {
    try {
      if (path_referenced(path)) {
        continue;
      }
    } catch (const std::exception & e) {
      continue;  // The rows are already committed, so keep the directory.
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    // Ignore errors - file may already be deleted
  }

  return outcome.second;
}

size_t PgFaultStorage::delete_rosbag_files(const std::vector<std::string> & fault_codes) {
  if (fault_codes.empty()) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  // Rows go first, in one transaction, and the files only after the commit: a
  // crash mid-delete leaves at worst an orphaned directory, never a row whose
  // bag is already gone.
  auto outcome =
      run_in_transaction("delete_rosbag_files", [&](pqxx::work & tx) -> std::pair<std::set<std::string>, size_t> {
        std::set<std::string> paths;
        size_t deleted = 0;
        for (const auto & code : fault_codes) {
          // RETURNING gives every path this code held, not just the first: one fault
          // code can name several recordings now, and the sweep must be able to
          // reclaim every one of their bags.
          auto res = execute(tx, "DELETE FROM rosbag_files WHERE fault_code = $1 RETURNING file_path", code);
          for (const auto & r : res) {
            paths.insert(r["file_path"].as<std::string>());
          }
          if (res.affected_rows() > 0) {
            ++deleted;
          }
        }
        return {paths, deleted};
      });

  for (const auto & path : outcome.first) {
    try {
      if (path_referenced(path)) {
        continue;
      }
    } catch (const std::exception & e) {
      continue;  // The rows are already committed, so keep the directory.
    }
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
    // Ignore errors - file may already be deleted
  }
  return outcome.second;
}

bool PgFaultStorage::path_referenced(const std::string & file_path) const {
  return run_in_transaction("path_referenced", [&](pqxx::work & tx) {
    auto res = execute(tx, "SELECT COUNT(*) AS n FROM rosbag_files WHERE file_path = $1", file_path);
    return !res.empty() && res[0]["n"].as<int64_t>() > 0;
  });
}

size_t PgFaultStorage::get_total_rosbag_storage_bytes() const {
  std::lock_guard<std::mutex> lock(mutex_);

  // Sum per bag, not per fault: one recording can back a burst of correlated
  // faults, and double-counting it would evict bags that still fit the quota.
  return run_in_transaction("get_total_rosbag_storage_bytes", [&](pqxx::work & tx) {
    auto res =
        execute(tx,
                "SELECT COALESCE(SUM(size_bytes),0) AS sz FROM (SELECT MAX(size_bytes) AS size_bytes FROM rosbag_files "
                "GROUP BY file_path) AS per_bag");
    return res.empty() ? size_t{0} : static_cast<size_t>(res[0]["sz"].as<int64_t>());
  });
}

std::vector<RosbagFileInfo> PgFaultStorage::get_all_rosbag_files() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_all_rosbag_files", [&](pqxx::work & tx) {
    auto res = execute(tx,
                       "SELECT fault_code, recording_id, file_path, format, duration_sec, size_bytes, created_at_ns "
                       "FROM rosbag_files ORDER BY created_at_ns ASC, id ASC");

    std::vector<RosbagFileInfo> result;
    for (const auto & r : res) {
      RosbagFileInfo info;
      info.fault_code = r["fault_code"].as<std::string>();
      info.recording_id = r["recording_id"].as<std::string>();
      info.file_path = r["file_path"].as<std::string>();
      info.format = r["format"].as<std::string>();
      info.duration_sec = r["duration_sec"].as<double>();
      info.size_bytes = static_cast<size_t>(r["size_bytes"].as<int64_t>());
      info.created_at_ns = r["created_at_ns"].as<int64_t>();
      result.push_back(info);
    }
    return result;
  });
}

std::vector<RosbagFileInfo> PgFaultStorage::list_rosbags_for_entity(const std::string & entity_fqn) const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("list_rosbags_for_entity", [&](pqxx::work & tx) {
    auto res = execute(
        tx,
        "SELECT r.fault_code, r.recording_id, r.file_path, r.format, r.duration_sec, r.size_bytes, r.created_at_ns "
        "FROM rosbag_files r "
        "JOIN faults f ON r.fault_code = f.fault_code "
        "JOIN jsonb_array_elements_text(f.reporting_sources::jsonb) AS j(value) ON j.value = $1 "
        "ORDER BY r.created_at_ns DESC, r.id DESC",
        entity_fqn);

    std::vector<RosbagFileInfo> result;
    for (const auto & r : res) {
      RosbagFileInfo info;
      info.fault_code = r["fault_code"].as<std::string>();
      info.recording_id = r["recording_id"].as<std::string>();
      info.file_path = r["file_path"].as<std::string>();
      info.format = r["format"].as<std::string>();
      info.duration_sec = r["duration_sec"].as<double>();
      info.size_bytes = static_cast<size_t>(r["size_bytes"].as<int64_t>());
      info.created_at_ns = r["created_at_ns"].as<int64_t>();
      result.push_back(info);
    }
    return result;
  });
}

std::vector<ros2_medkit_msgs::msg::Fault> PgFaultStorage::get_all_faults() const {
  std::lock_guard<std::mutex> lock(mutex_);

  return run_in_transaction("get_all_faults", [&](pqxx::work & tx) {
    auto res = execute(tx,
                       "SELECT fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
                       "occurrence_count, status, reporting_sources, last_passed_ns FROM faults");

    std::vector<ros2_medkit_msgs::msg::Fault> result;
    for (const auto & r : res) {
      ros2_medkit_msgs::msg::Fault fault;
      fault.fault_code = r["fault_code"].as<std::string>();
      fault.severity = static_cast<uint8_t>(r["severity"].as<int>());
      fault.description = r["description"].as<std::string>();
      fault.first_occurred = rclcpp::Time(r["first_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.last_occurred = rclcpp::Time(r["last_occurred_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      fault.occurrence_count = static_cast<uint32_t>(r["occurrence_count"].as<int64_t>());
      fault.status = r["status"].as<std::string>();
      fault.reporting_sources = parse_json_array(r["reporting_sources"].as<std::string>());
      fault.last_passed = rclcpp::Time(r["last_passed_ns"].as<int64_t>(), RCL_SYSTEM_TIME);
      result.push_back(fault);
    }
    return result;
  });
}

}  // namespace ros2_medkit_fault_manager
