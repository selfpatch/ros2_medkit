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

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tl/expected.hpp>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/log_types.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider_registry.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/transports/log_source.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief Manages log collection via the default ring-buffer backend.
 *
 * Pure C++ application service. Middleware-side ingestion is performed by an
 * injected LogSource adapter (typically Ros2LogSource) that subscribes to
 * /rosout and converts each message into a neutral LogEntry. The manager body
 * keeps its ring-buffer storage, per-entity config map, monotonic id counter,
 * the plugin observer pattern, and the manages_ingestion short-circuit. None
 * of those depend on ROS types.
 *
 * Plugin integration (two modes):
 * - **Observer mode** (default): If a LogProvider plugin is registered, get_logs()
 *   and get_config() delegate to it. on_log_entry() is called on ALL LogProvider
 *   observers for every LogEntry produced by the source. Observers returning true
 *   suppress ring-buffer storage.
 * - **Full ingestion** (manages_ingestion() == true): The primary LogProvider owns
 *   the entire log pipeline. LogManager never calls source->start(), so no
 *   subscription is created. All queries and config operations delegate to the plugin.
 *
 * FQN normalization:
 * - entity.fqn from the entity cache has a leading '/' (e.g. "/powertrain/engine/temp_sensor")
 * - Source-emitted entries DO NOT have a leading '/' (rcl_node_get_logger_name convention)
 * - Callers pass raw FQNs from entity.fqn; LogManager strips leading '/' internally.
 */
class LogManager {
 public:
  /// Default maximum number of entries retained per node in the ring buffer
  static constexpr size_t kDefaultBufferSize = 200;

  /// Severity used by the LogManager's log_sink callback. Numeric values
  /// match rcl/rcutils log levels (30=WARN, 40=ERROR) so an adapter can
  /// forward to RCLCPP_* macros without translation.
  static constexpr int kLogLevelWarn = 30;
  static constexpr int kLogLevelError = 40;

  /// Sink for internal diagnostics (plugin provider exceptions, etc.).
  /// Keeps LogManager middleware-neutral while preserving observability
  /// when the gateway adapter forwards to /rosout. May be null in tests.
  using LogSink = std::function<void(int level, std::string_view message)>;

  /// Maps a logger name to the entity id its "logs" change carries.
  using LoggerOwnerFn = std::function<std::string(const std::string & logger_name)>;

  /**
   * @brief Construct LogManager
   *
   * If the primary LogProvider's manages_ingestion() returns true, source->start()
   * is never called, so no subscription is created. Otherwise the source is
   * started with on_log_entry() as the entry-point callback.
   *
   * @param source             LogSource adapter (typically Ros2LogSource).
   *                           Manager takes shared ownership.
   * @param provider_registry  LogProviderRegistry port for LogProvider lookup
   *                           (typically the PluginManager). May be nullptr -
   *                           equivalent to "no plugins registered".
   * @param max_buffer_size    Ring buffer size per node (override for unit testing)
   * @param log_sink           Callback for internal diagnostics. The gateway
   *                           passes an adapter that forwards to RCLCPP_WARN/ERROR;
   *                           unit tests may capture calls or pass nullptr.
   */
  explicit LogManager(std::shared_ptr<LogSource> source, LogProviderRegistry * provider_registry = nullptr,
                      size_t max_buffer_size = kDefaultBufferSize, LogSink log_sink = nullptr);

  ~LogManager();

  LogManager(const LogManager &) = delete;
  LogManager & operator=(const LogManager &) = delete;
  LogManager(LogManager &&) = delete;
  LogManager & operator=(LogManager &&) = delete;

  /**
   * @brief Query log entries for a set of node FQNs
   *
   * If a LogProvider plugin is registered, delegates to it.
   * Otherwise uses the local ring buffer.
   *
   * @param node_fqns    Node FQNs from entity.fqn (WITH leading '/' - normalized internally)
   * @param prefix_match If true, match all buffered nodes whose name starts with the given prefix
   *                     (used for Component queries). If false, exact match (App queries).
   * @param min_severity Additional severity filter from query parameter. Empty = no override.
   * @param context_filter Substring filter applied to log entry's name (logger name). Empty = no filter.
   * @param entity_id    Entity ID for config lookup. Empty = use defaults.
   * @param marker_ids   Optional out: receives the ids of the returned entries that add_log_entry()
   *                     wrote. A LogProvider plugin's entries are never markers.
   * @return JSON array of LogEntry objects sorted by id ascending, capped at entity config max_entries.
   */
  tl::expected<json, std::string> get_logs(const std::vector<std::string> & node_fqns, bool prefix_match,
                                           const std::string & min_severity, const std::string & context_filter,
                                           const std::string & entity_id,
                                           std::unordered_set<std::string> * marker_ids = nullptr);

  /// Get current log configuration for entity (returns defaults if unconfigured)
  tl::expected<LogConfig, std::string> get_config(const std::string & entity_id) const;

  /**
   * @brief Update log configuration for an entity
   * @return Empty string on success, error message on validation failure
   */
  std::string update_config(const std::string & entity_id, const std::optional<std::string> & severity_filter,
                            const std::optional<size_t> & max_entries);

  /**
   * @brief Add the marker entry of a trigger's log_settings
   *
   * Creates a LogEntry and pushes it to the internal ring buffer using the
   * same path as on_log_entry(). If a ResourceChangeNotifier is set, emits the
   * same "logs" CREATED change a source entry does, to every subscription but
   * `skipped_subscription`. get_logs() reports the entry as a marker.
   *
   * @param entity_id  Entity to associate the log with (used as logger name)
   * @param severity   SOVD severity string (debug, info, warning, error, fatal)
   * @param message    Human-readable log message
   * @param metadata   Additional JSON metadata stored in the message (appended)
   * @param skipped_subscription Notifier subscription the change skips; 0 = none
   */
  void add_log_entry(const std::string & entity_id, const std::string & severity, const std::string & message,
                     const nlohmann::json & metadata, NotifierSubscriptionId skipped_subscription = 0);

  /// Set the ResourceChangeNotifier for emitting log change events.
  /// Called by GatewayNode after both LogManager and the notifier are available.
  /// Each stored entry emits one "logs" CREATED change: entity_id from the
  /// logger owner function, the entry as value, its logger name in
  /// value.context.node. An entry the buffer cap drops emits none.
  void set_notifier(ResourceChangeNotifier * notifier);

  /// Set the function that gives a "logs" change its entity_id from the logger
  /// name. Without one the entity_id is "". Call before set_notifier().
  void set_logger_owner_fn(LoggerOwnerFn fn);

  /// Numeric level of the entry in a "logs" change value. The value names the
  /// severity only, so for an entry logged at a level between the named ones
  /// the level is looked up by entry id. The lookup keeps as many such entries
  /// as the notifier can hold waiting for dispatch
  /// (ResourceChangeNotifier::max_in_flight), counting changes the queue drops;
  /// an older one falls back to the severity name. 0 without a severity.
  uint8_t entry_level(const nlohmann::json & entry) const;

  // ---- Static utilities (no middleware required - safe in unit tests) ----

  /// Convert ROS 2 uint8 log level -> SOVD severity string ("debug" for unknown levels)
  static std::string level_to_severity(uint8_t level);

  /// Convert SOVD severity string -> ROS 2 uint8 log level (0 for invalid/empty)
  static uint8_t severity_to_level(const std::string & severity);

  /// Check if a severity string is valid (one of: debug, info, warning, error, fatal)
  static bool is_valid_severity(const std::string & severity);

  /// Format a LogEntry as SOVD JSON (id, timestamp, severity, message, context)
  static json entry_to_json(const LogEntry & entry);

  /// Strip leading '/' from a node FQN for ring-buffer key normalization
  static std::string normalize_fqn(const std::string & fqn);

  /// True when get_logs() with `node_fqn` covers entries of `logger_name`: the
  /// FQN in slash or dot form, and with `prefix_match` any logger below it.
  static bool logger_matches(const std::string & logger_name, const std::string & node_fqn, bool prefix_match);

  // ---- Test injection (for unit tests - do not use in production code) ----

  /**
   * @brief Inject a log entry directly into the ring buffer (bypasses the source)
   *
   * Used by unit tests to populate the buffer without a live source feed.
   * In production the buffer is populated exclusively by source-emitted entries.
   */
  void inject_entry_for_testing(LogEntry entry);

  /**
   * @brief Total number of log entries dropped because the per-node-buffer cap
   *        (max_buffer_size_ * 10) was exceeded.
   *
   * Each "new node beyond the cap" entry counts as one drop. Per-node
   * ring-buffer pops (oldest entries replaced inside an existing buffer) are
   * NOT counted here - this counter tracks only entries fully rejected from
   * being buffered. Used by tests and (eventually) a /server-info or
   * /diagnostics endpoint to surface buffer pressure.
   */
  std::size_t dropped_entries_count() const noexcept;

 private:
  /// Entry point invoked for every LogEntry produced by the source. Performs
  /// observer notification, ring-buffer storage, and trigger-side notifier
  /// dispatch.
  void on_log_entry(const LogEntry & entry);

  /// Emit the "logs" CREATED change for a stored entry, if a notifier is set.
  void notify_entry(const LogEntry & entry, NotifierSubscriptionId skipped_subscription = 0);

  /// Drop the oldest entry of `buf`. Caller holds buffers_mutex_.
  void pop_oldest_locked(std::deque<LogEntry> & buf);

  /// Keep the level of an entry whose level has no severity name.
  void remember_unnamed_level(const LogEntry & entry);

  /// Get the effective LogProvider: plugin if registered, else nullptr (use local buffer)
  LogProvider * effective_provider() const;

  /// Forward an internal diagnostic to the injected log_sink (if any).
  /// No-op when the sink is null, keeping unit tests free of /rosout coupling.
  void emit(int level, std::string_view msg) const {
    if (log_sink_) {
      log_sink_(level, msg);
    }
  }

  std::shared_ptr<LogSource> source_;
  LogProviderRegistry * provider_registry_;
  ResourceChangeNotifier * notifier_ = nullptr;
  LoggerOwnerFn logger_owner_fn_;
  size_t max_buffer_size_;
  LogSink log_sink_;

  // Ring buffers keyed by normalized node name (no leading '/')
  // e.g. "powertrain/engine/temp_sensor" -> deque<LogEntry>
  std::unordered_map<std::string, std::deque<LogEntry>> buffers_;
  // Set of node names for which we've already emitted a "buffer cap exceeded"
  // WARN. Subsequent drops for the same node only bump dropped_total_, so we
  // never spam the log sink. Guarded by buffers_mutex_.
  std::unordered_set<std::string> dropped_nodes_;
  // Total number of entries that were rejected because adding a new node
  // would exceed the buffer cap. Guarded by buffers_mutex_.
  std::size_t dropped_total_{0};
  // Ids of buffered entries that add_log_entry() wrote. Guarded by buffers_mutex_.
  std::unordered_set<int64_t> marker_ids_;
  mutable std::mutex buffers_mutex_;

  // Per-entity configuration keyed by entity_id
  std::unordered_map<std::string, LogConfig> configs_;
  mutable std::mutex configs_mutex_;

  // Monotonically increasing entry ID (never resets; starts at 1)
  std::atomic<int64_t> next_id_{1};

  // Exact levels of notified entries with no severity name, oldest first.
  std::unordered_map<int64_t, uint8_t> unnamed_levels_;
  std::deque<int64_t> unnamed_level_order_;
  mutable std::mutex unnamed_levels_mutex_;
};

}  // namespace ros2_medkit_gateway
