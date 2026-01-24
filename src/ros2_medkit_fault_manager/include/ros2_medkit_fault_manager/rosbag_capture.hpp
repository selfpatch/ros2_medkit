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
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialized_message.hpp>

#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/snapshot_capture.hpp"

namespace ros2_medkit_fault_manager {

/// A buffered message stored in the ring buffer
struct BufferedMessage {
  std::string topic;
  std::string message_type;
  std::shared_ptr<rclcpp::SerializedMessage> serialized_data;
  int64_t timestamp_ns{0};
};

/// Manages rosbag recording with ring buffer for time-window capture
///
/// This class implements a "black box" style recording where messages are
/// continuously buffered in memory. When a fault is confirmed, the buffer
/// is flushed to a bag file along with continued recording for a short
/// period after the fault.
///
/// Lifecycle:
/// - start() begins buffering messages (or lazy_start waits for PREFAILED)
/// - on_fault_confirmed() flushes buffer to bag file
/// - on_fault_cleared() deletes bag file if auto_cleanup enabled
class RosbagCapture {
 public:
  /// Create rosbag capture
  /// @param node ROS 2 node for creating subscriptions and timers
  /// @param storage Fault storage for persisting bag file metadata
  /// @param config Rosbag configuration
  /// @param snapshot_config Snapshot configuration (for topic resolution when topics="config")
  RosbagCapture(rclcpp::Node * node, FaultStorage * storage, const RosbagConfig & config,
                const SnapshotConfig & snapshot_config);

  ~RosbagCapture();

  // Non-copyable, non-movable
  RosbagCapture(const RosbagCapture &) = delete;
  RosbagCapture & operator=(const RosbagCapture &) = delete;
  RosbagCapture(RosbagCapture &&) = delete;
  RosbagCapture & operator=(RosbagCapture &&) = delete;

  /// Start the ring buffer recording
  /// If lazy_start is false, this is called automatically on construction
  void start();

  /// Stop recording and clean up subscriptions
  void stop();

  /// Check if ring buffer is currently running
  bool is_running() const;

  /// Called when a fault enters PREFAILED state (for lazy_start mode)
  /// @param fault_code The fault code that entered PREFAILED
  void on_fault_prefailed(const std::string & fault_code);

  /// Called when a fault is confirmed - flushes buffer to bag file
  /// @param fault_code The fault code that was confirmed
  void on_fault_confirmed(const std::string & fault_code);

  /// Called when a fault is cleared - deletes bag file if auto_cleanup
  /// @param fault_code The fault code that was cleared
  void on_fault_cleared(const std::string & fault_code);

  /// Get current configuration
  const RosbagConfig & config() const {
    return config_;
  }

  /// Check if rosbag capture is enabled
  bool is_enabled() const {
    return config_.enabled;
  }

 private:
  /// Initialize subscriptions for configured topics
  void init_subscriptions();

  /// Message callback for all subscribed topics
  void message_callback(const std::string & topic, const std::string & msg_type,
                        const std::shared_ptr<const rclcpp::SerializedMessage> & msg);

  /// Prune old messages from buffer based on duration_sec
  void prune_buffer();

  /// Resolve which topics to record based on config
  std::vector<std::string> resolve_topics() const;

  /// Get message type for a topic
  std::string get_topic_type(const std::string & topic) const;

  /// Flush ring buffer to a bag file
  /// @param fault_code The fault code to associate with the bag
  /// @return Path to the created bag file, or empty string on failure
  std::string flush_to_bag(const std::string & fault_code);

  /// Generate bag file path for a fault
  std::string generate_bag_path(const std::string & fault_code) const;

  /// Calculate total size of a bag directory
  size_t calculate_bag_size(const std::string & bag_path) const;

  /// Enforce storage limits by deleting oldest bags
  void enforce_storage_limits();

  /// Timer callback for post-fault recording
  void post_fault_timer_callback();

  /// Try to subscribe to a single topic
  /// @param topic The topic to subscribe to
  /// @return True if subscription was created, false if type couldn't be determined
  bool try_subscribe_topic(const std::string & topic);

  /// Timer callback for retrying topic discovery
  void discovery_retry_callback();

  /// Validate storage format and check if plugin is available
  /// @throws std::runtime_error if format is invalid or plugin unavailable
  void validate_storage_format() const;

  rclcpp::Node * node_;
  FaultStorage * storage_;
  RosbagConfig config_;
  SnapshotConfig snapshot_config_;

  /// Ring buffer for messages
  mutable std::mutex buffer_mutex_;
  std::deque<BufferedMessage> message_buffer_;

  /// Subscriptions (kept alive for continuous recording)
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  /// Running state
  std::atomic<bool> running_{false};

  /// Post-fault recording state
  std::string current_fault_code_;
  std::string current_bag_path_;
  rclcpp::TimerBase::SharedPtr post_fault_timer_;
  std::atomic<bool> recording_post_fault_{false};

  /// Topic types cache
  mutable std::mutex topic_types_mutex_;
  std::map<std::string, std::string> topic_types_;

  /// Discovery retry state
  rclcpp::TimerBase::SharedPtr discovery_retry_timer_;
  std::vector<std::string> pending_topics_;
  int discovery_retry_count_{0};
};

}  // namespace ros2_medkit_fault_manager
