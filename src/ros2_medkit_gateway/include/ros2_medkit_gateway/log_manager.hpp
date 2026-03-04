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
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/log_types.hpp"
#include "ros2_medkit_gateway/providers/log_provider.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

class PluginManager;  // forward declaration — full include in .cpp

/**
 * @brief Manages /rosout log collection via the default ring-buffer backend.
 *
 * Subscribes to /rosout, normalizes logger names (strips leading '/'),
 * and stores entries per node-name in fixed-size deque ring buffers.
 *
 * Plugin integration (two modes):
 * - **Observer mode** (default): If a LogProvider plugin is registered, get_logs()
 *   and get_config() delegate to it. on_log_entry() is called on ALL LogProvider
 *   observers for every /rosout message. Observers returning true suppress ring-buffer
 *   storage.
 * - **Full ingestion** (manages_ingestion() == true): The primary LogProvider owns
 *   the entire log pipeline. LogManager skips the /rosout subscription and ring buffer
 *   entirely. All queries and config operations delegate to the plugin.
 *
 * FQN normalization:
 * - entity.fqn from the entity cache has a leading '/' (e.g. "/powertrain/engine/temp_sensor")
 * - /rosout msg.name does NOT have a leading '/' (rcl_node_get_logger_name convention)
 * - Callers pass raw FQNs from entity.fqn; LogManager strips leading '/' internally.
 */
class LogManager {
 public:
  /// Default maximum number of entries retained per node in the ring buffer
  static constexpr size_t kDefaultBufferSize = 200;

  /**
   * @brief Construct LogManager
   *
   * If the primary LogProvider's manages_ingestion() returns true, the /rosout
   * subscription and ring buffer are skipped entirely. Otherwise subscribes to
   * /rosout as usual.
   *
   * @param node             ROS 2 node (used for subscription and logging)
   * @param plugin_mgr       PluginManager for LogProvider lookup (may be nullptr)
   * @param max_buffer_size  Ring buffer size per node (override for unit testing)
   */
  explicit LogManager(rclcpp::Node * node, PluginManager * plugin_mgr = nullptr,
                      size_t max_buffer_size = kDefaultBufferSize);

  /**
   * @brief Query log entries for a set of node FQNs
   *
   * If a LogProvider plugin is registered, delegates to it.
   * Otherwise uses the local ring buffer.
   *
   * @param node_fqns    Node FQNs from entity.fqn (WITH leading '/' — normalized internally)
   * @param prefix_match If true, match all buffered nodes whose name starts with the given prefix
   *                     (used for Component queries). If false, exact match (App queries).
   * @param min_severity Additional severity filter from query parameter. Empty = no override.
   * @param context_filter Substring filter applied to log entry's name (logger name). Empty = no filter.
   * @param entity_id    Entity ID for config lookup. Empty = use defaults.
   * @return JSON array of LogEntry objects sorted by id ascending, capped at entity config max_entries.
   */
  json get_logs(const std::vector<std::string> & node_fqns, bool prefix_match, const std::string & min_severity,
                const std::string & context_filter, const std::string & entity_id);

  /// Get current log configuration for entity (returns defaults if unconfigured)
  LogConfig get_config(const std::string & entity_id) const;

  /**
   * @brief Update log configuration for an entity
   * @return Empty string on success, error message on validation failure
   */
  std::string update_config(const std::string & entity_id, const std::optional<std::string> & severity_filter,
                            const std::optional<size_t> & max_entries);

  // ---- Static utilities (no ROS 2 required — safe in unit tests) ----

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

  // ---- Test injection (for unit tests — do not use in production code) ----

  /**
   * @brief Inject a log entry directly into the ring buffer (bypasses /rosout subscription)
   *
   * Used by unit tests to populate the buffer without a live ROS 2 graph.
   * In production the buffer is populated exclusively by the /rosout callback.
   */
  void inject_entry_for_testing(LogEntry entry);

 private:
  void on_rosout(const rcl_interfaces::msg::Log::ConstSharedPtr & msg);

  /// Get the effective LogProvider: plugin if registered, else nullptr (use local buffer)
  LogProvider * effective_provider() const;

  rclcpp::Node * node_;
  PluginManager * plugin_mgr_;
  size_t max_buffer_size_;

  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;

  // Ring buffers keyed by normalized node name (no leading '/')
  // e.g. "powertrain/engine/temp_sensor" -> deque<LogEntry>
  std::unordered_map<std::string, std::deque<LogEntry>> buffers_;
  mutable std::mutex buffers_mutex_;

  // Per-entity configuration keyed by entity_id
  std::unordered_map<std::string, LogConfig> configs_;
  mutable std::mutex configs_mutex_;

  // Monotonically increasing entry ID (never resets; starts at 1)
  std::atomic<int64_t> next_id_{1};
};

}  // namespace ros2_medkit_gateway
