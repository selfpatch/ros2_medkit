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

#include <optional>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/log_types.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Provider interface for log storage backends
 *
 * Typed provider interface implemented by plugins alongside GatewayPlugin
 * via multiple inheritance. When a LogProvider plugin is loaded, LogManager
 * delegates all query/config operations to it and stops using the local
 * /rosout ring buffer for serving results.
 *
 * Multiple plugins implementing LogProvider can be loaded; only the first
 * registered LogProvider is used for queries (same as UpdateProvider).
 * ALL LogProvider plugins receive on_log_entry() calls (observer pattern) -
 * unless the primary provider returns true from manages_ingestion(), in which
 * case the /rosout subscription is never created and on_log_entry() is never called.
 *
 * Two modes of operation:
 * - **Observer mode** (default, manages_ingestion() == false): LogManager subscribes
 *   to /rosout and feeds entries to all observers via on_log_entry(). The primary
 *   provider replaces the query/config backend; the /rosout ring buffer is still
 *   populated unless on_log_entry() returns true.
 * - **Full ingestion** (manages_ingestion() == true): The primary provider owns the
 *   entire log pipeline - choosing its own source (journald, CloudWatch, custom
 *   topics), format, and storage. LogManager skips the /rosout subscription and
 *   ring buffer entirely.
 *
 * @par Thread safety
 * All methods may be called from multiple threads concurrently.
 * Implementations must provide their own synchronization.
 *
 * @par Example use cases
 * - OpenTelemetry exporter: implement on_log_entry() to forward to OTLP endpoint
 * - Database backend: implement get_logs() to query a SQLite/InfluxDB/etc. store
 * - Log aggregator: combine /rosout with external log sources
 * - Full ingestion: subscribe to journald/CloudWatch/custom topics directly
 *
 * @see GatewayPlugin for the base class all plugins must also implement
 * @see LogManager for the subsystem that uses this
 */
class LogProvider {
 public:
  virtual ~LogProvider() = default;

  /**
   * @brief Whether this provider fully manages log ingestion
   *
   * When true, LogManager skips the /rosout subscription and ring buffer entirely.
   * The plugin is responsible for sourcing log entries from whatever transport it
   * chooses (journald, CloudWatch, custom ROS 2 topics, etc.).
   *
   * When false (the default), LogManager subscribes to /rosout and populates the
   * ring buffer as usual; all LogProvider observers receive on_log_entry() calls.
   *
   * Only the primary LogProvider's return value is checked (at LogManager construction).
   *
   * @return true if the plugin owns the entire log pipeline; false for observer mode
   */
  virtual bool manages_ingestion() const {
    return false;
  }

  /**
   * @brief Query log entries for a set of node names
   *
   * @param node_fqns   Node FQNs WITHOUT leading slash (e.g. "powertrain/engine/temp_sensor")
   * @param prefix_match If true, treat each entry as a namespace prefix (for Component queries).
   *                     If false, match exactly (for App queries).
   * @param min_severity Optional additional severity filter ("debug","info","warning","error","fatal").
   *                     Empty string = no additional filter beyond entity config.
   * @param context_filter Optional substring filter applied to the log entry's node/logger name.
   * @param entity_id   Entity ID for config lookup (determines base severity_filter and max_entries).
   * @return LogEntry objects sorted by id ascending, capped at config.max_entries.
   *         LogManager handles JSON serialization via entry_to_json().
   */
  virtual std::vector<LogEntry> get_logs(const std::vector<std::string> & node_fqns, bool prefix_match,
                                         const std::string & min_severity, const std::string & context_filter,
                                         const std::string & entity_id) = 0;

  /**
   * @brief Get the current log configuration for an entity
   * @return Entity config (default values if entity was never configured)
   */
  virtual LogConfig get_config(const std::string & entity_id) const = 0;

  /**
   * @brief Update log configuration for an entity
   * @return Empty string on success, error message on validation failure
   */
  virtual std::string update_config(const std::string & entity_id, const std::optional<std::string> & severity_filter,
                                    const std::optional<size_t> & max_entries) = 0;

  /**
   * @brief Called for each /rosout log entry before it is stored in the default ring buffer
   *
   * All LogProvider plugins receive this call (observer pattern), regardless of
   * whether they are the primary query provider.
   *
   * @note This method is never called when the primary provider's manages_ingestion()
   * returns true, because the /rosout subscription is not created in that case.
   *
   * @param entry The log entry (name field is WITHOUT leading slash)
   * @return true to suppress default ring buffer storage; false to allow it (default)
   *
   * OR-aggregation: if ANY observer returns true, ring buffer storage is suppressed.
   * All observers are always called regardless of earlier return values.
   *
   * Example: An OpenTelemetry plugin forwards to OTLP and returns false (keeps ring buffer).
   *          A database plugin stores in SQLite, returns true (replaces ring buffer for that entry).
   */
  virtual bool on_log_entry(const LogEntry & entry) {
    (void)entry;
    return false;
  }
};

}  // namespace ros2_medkit_gateway
