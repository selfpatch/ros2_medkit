// Copyright 2026 mfaferek93
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
#include <map>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {

/// Configuration for rosbag recording (optional time-window capture)
struct RosbagConfig {
  /// Whether rosbag recording is enabled (opt-in)
  bool enabled{false};

  /// Duration in seconds to buffer before fault confirmation
  double duration_sec{5.0};

  /// Duration in seconds to continue recording after fault confirmation
  double duration_after_sec{1.0};

  /// Topic selection mode: "config" (reuse JSON config), "all", or comma-separated list
  std::string topics{"config"};

  /// Additional topics to include (added to resolved list)
  std::vector<std::string> include_topics;

  /// Topics to exclude from recording
  std::vector<std::string> exclude_topics;

  /// If true, start ring buffer only when fault enters PREFAILED state
  /// If false (default), ring buffer runs continuously from startup
  bool lazy_start{false};

  /// Storage format: "sqlite3" or "mcap"
  std::string format{"sqlite3"};

  /// Path to store bag files (empty = system temp directory)
  std::string storage_path;

  /// Maximum size of a single bag file in MB
  size_t max_bag_size_mb{50};

  /// Maximum total storage for all bag files in MB
  size_t max_total_storage_mb{500};

  /// If true, delete bag file when fault is cleared
  bool auto_cleanup{true};
};

/// Configuration for snapshot capture
struct SnapshotConfig {
  /// Whether snapshot capture is enabled
  bool enabled{true};

  /// Whether to use background subscriptions (always have latest data)
  /// If false, creates on-demand subscriptions when fault is confirmed
  bool background_capture{false};

  /// Timeout in seconds for on-demand topic sampling
  double timeout_sec{1.0};

  /// Maximum message size in bytes (messages larger than this are skipped)
  size_t max_message_size{65536};

  /// Topics to capture for specific fault codes (exact match)
  /// Key: fault_code, Value: list of topics
  std::map<std::string, std::vector<std::string>> fault_specific;

  /// Topics to capture for fault code patterns (regex)
  /// Key: regex pattern, Value: list of topics
  /// First matching pattern wins
  std::map<std::string, std::vector<std::string>> patterns;

  /// Default topics to capture if no specific or pattern match
  std::vector<std::string> default_topics;

  /// Rosbag recording configuration (optional)
  RosbagConfig rosbag;
};

/// Cached message from background subscription
struct CachedMessage {
  std::string topic;
  std::string message_type;
  std::string data;  ///< JSON-encoded message
  int64_t timestamp_ns{0};
};

/// Captures topic snapshots when faults are confirmed
///
/// Supports two modes:
/// - On-demand: Creates temporary subscriptions when fault is confirmed, waits for data
/// - Background: Maintains subscriptions to configured topics, caches latest messages
class SnapshotCapture {
 public:
  /// Create snapshot capture
  /// @param node ROS 2 node for creating subscriptions
  /// @param storage Fault storage for persisting snapshots
  /// @param config Snapshot configuration
  SnapshotCapture(rclcpp::Node * node, FaultStorage * storage, const SnapshotConfig & config);

  ~SnapshotCapture();

  // Non-copyable, non-movable
  SnapshotCapture(const SnapshotCapture &) = delete;
  SnapshotCapture & operator=(const SnapshotCapture &) = delete;
  SnapshotCapture(SnapshotCapture &&) = delete;
  SnapshotCapture & operator=(SnapshotCapture &&) = delete;

  /// Capture snapshots for a fault that was just confirmed
  /// @param fault_code The fault code that was confirmed
  void capture(const std::string & fault_code);

  /// Get current configuration
  const SnapshotConfig & config() const {
    return config_;
  }

  /// Check if snapshot capture is enabled
  bool is_enabled() const {
    return config_.enabled;
  }

 private:
  /// Resolve which topics to capture for a given fault code
  /// Priority: fault_specific > patterns > default_topics
  std::vector<std::string> resolve_topics(const std::string & fault_code) const;

  /// Capture a single topic on-demand (creates temporary subscription)
  /// @return true if capture was successful
  bool capture_topic_on_demand(const std::string & fault_code, const std::string & topic);

  /// Capture a topic from background cache
  /// @return true if data was available in cache
  bool capture_topic_from_cache(const std::string & fault_code, const std::string & topic);

  /// Initialize background subscriptions for all configured topics
  void init_background_subscriptions();

  /// Collect all unique topics from configuration
  std::vector<std::string> collect_all_configured_topics() const;

  /// Get message type for a topic
  std::string get_topic_type(const std::string & topic) const;

  rclcpp::Node * node_;
  FaultStorage * storage_;
  SnapshotConfig config_;

  /// Compiled regex patterns (cached for performance)
  std::vector<std::pair<std::regex, std::vector<std::string>>> compiled_patterns_;

  /// Background subscription cache (topic -> latest message)
  mutable std::mutex cache_mutex_;
  std::map<std::string, CachedMessage> message_cache_;

  /// Background subscriptions (kept alive for continuous caching)
  std::vector<rclcpp::GenericSubscription::SharedPtr> background_subscriptions_;
};

}  // namespace ros2_medkit_fault_manager
