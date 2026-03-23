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

#include "ros2_medkit_fault_manager/snapshot_capture.hpp"

#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <set>
#include <thread>

#include "ros2_medkit_fault_manager/time_utils.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"

namespace ros2_medkit_fault_manager {

SnapshotCapture::SnapshotCapture(rclcpp::Node * node, FaultStorage * storage, const SnapshotConfig & config)
  : node_(node), storage_(storage), config_(config) {
  if (!node_) {
    throw std::invalid_argument("SnapshotCapture requires a valid node pointer");
  }
  if (!storage_) {
    throw std::invalid_argument("SnapshotCapture requires a valid storage pointer");
  }

  // Compile regex patterns for performance
  size_t failed_patterns = 0;
  for (const auto & [pattern, topics] : config_.patterns) {
    try {
      compiled_patterns_.emplace_back(std::regex(pattern), topics);
    } catch (const std::regex_error & e) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid regex pattern '%s' in snapshot config will be IGNORED: %s",
                   pattern.c_str(), e.what());
      ++failed_patterns;
    }
  }

  // Initialize background subscriptions if enabled
  if (config_.enabled && config_.background_capture) {
    init_background_subscriptions();
  }

  if (failed_patterns > 0) {
    RCLCPP_WARN(node_->get_logger(), "SnapshotCapture initialized with %zu/%zu patterns failed to compile",
                failed_patterns, config_.patterns.size());
  }

  RCLCPP_INFO(node_->get_logger(),
              "SnapshotCapture initialized (enabled=%s, background=%s, timeout=%.1fs, patterns=%zu)",
              config_.enabled ? "true" : "false", config_.background_capture ? "true" : "false", config_.timeout_sec,
              compiled_patterns_.size());
}

SnapshotCapture::~SnapshotCapture() {
  // Subscriptions are automatically cleaned up by shared_ptr
  background_subscriptions_.clear();
}

void SnapshotCapture::capture(const std::string & fault_code) {
  if (!config_.enabled) {
    RCLCPP_DEBUG(node_->get_logger(), "Snapshot capture disabled, skipping for fault '%s'", fault_code.c_str());
    return;
  }

  auto topics = resolve_topics(fault_code);
  if (topics.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "No topics configured for fault '%s'", fault_code.c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Capturing snapshots for fault '%s' (%zu topics)", fault_code.c_str(),
              topics.size());

  size_t captured_count = 0;
  for (const auto & topic : topics) {
    bool success = false;
    if (config_.background_capture) {
      success = capture_topic_from_cache(fault_code, topic);
    } else {
      success = capture_topic_on_demand(fault_code, topic);
    }

    if (success) {
      ++captured_count;
    }
  }

  RCLCPP_INFO(node_->get_logger(), "Captured %zu/%zu snapshots for fault '%s'", captured_count, topics.size(),
              fault_code.c_str());
}

std::vector<std::string> SnapshotCapture::resolve_topics(const std::string & fault_code) const {
  // Priority 1: Exact match in fault_specific
  auto it = config_.fault_specific.find(fault_code);
  if (it != config_.fault_specific.end()) {
    RCLCPP_DEBUG(node_->get_logger(), "Using fault_specific topics for '%s'", fault_code.c_str());
    return it->second;
  }

  // Priority 2: First matching regex pattern
  for (const auto & [pattern_regex, topics] : compiled_patterns_) {
    if (std::regex_match(fault_code, pattern_regex)) {
      RCLCPP_DEBUG(node_->get_logger(), "Using pattern-matched topics for '%s'", fault_code.c_str());
      return topics;
    }
  }

  // Priority 3: Default topics
  if (!config_.default_topics.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "Using default_topics for '%s'", fault_code.c_str());
    return config_.default_topics;
  }

  return {};
}

bool SnapshotCapture::capture_topic_on_demand(const std::string & fault_code, const std::string & topic) {
  // Get topic type
  std::string msg_type = get_topic_type(topic);
  if (msg_type.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Topic '%s' not found, skipping snapshot", topic.c_str());
    return false;
  }

  // Check if topic has publishers
  if (node_->count_publishers(topic) == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "Topic '%s' has no publishers, skipping snapshot", topic.c_str());
    return false;
  }

  // Create one-shot subscription
  std::atomic<bool> received{false};
  rclcpp::SerializedMessage captured_msg;
  std::mutex msg_mutex;

  rclcpp::QoS qos = rclcpp::SensorDataQoS();  // Best effort for sensor data

  // Try to match publisher QoS
  auto pub_info = node_->get_publishers_info_by_topic(topic);
  if (!pub_info.empty()) {
    if (pub_info[0].qos_profile().reliability() == rclcpp::ReliabilityPolicy::Reliable) {
      qos = rclcpp::QoS(10);  // Reliable
    }
  }

  // Create a local callback group for this capture operation (ensures clean executor lifecycle).
  // Pass false to prevent automatic association with the node's main executor —
  // we manually add this group to a local SingleThreadedExecutor below.
  auto local_callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  rclcpp::GenericSubscription::SharedPtr subscription;
  try {
    // NOLINTNEXTLINE(performance-unnecessary-value-param)
    auto callback = [&received, &captured_msg, &msg_mutex](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      bool expected = false;
      if (received.compare_exchange_strong(expected, true)) {
        std::lock_guard<std::mutex> lock(msg_mutex);
        captured_msg = *msg;
      }
    };

    // Use local callback group to avoid reentrancy with service callbacks
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = local_callback_group;

    subscription = node_->create_generic_subscription(topic, msg_type, qos, callback, sub_options);
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to create subscription for '%s': %s", topic.c_str(), e.what());
    return false;
  }

  // Use a local executor with the local callback group (both destroyed together on exit)
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_callback_group(local_callback_group, node_->get_node_base_interface());

  // Wait for message with timeout
  const auto timeout = std::chrono::duration<double>(config_.timeout_sec);
  const auto start_time = std::chrono::steady_clock::now();

  while (!received.load()) {
    auto elapsed = std::chrono::steady_clock::now() - start_time;
    if (elapsed >= timeout) {
      RCLCPP_DEBUG(node_->get_logger(), "Timeout waiting for message on '%s'", topic.c_str());
      // Subscription and callback group destroyed on return, executor cleanup is automatic
      return false;
    }

    executor.spin_some(std::chrono::milliseconds(10));
  }

  // Deserialize to JSON
  try {
    std::lock_guard<std::mutex> lock(msg_mutex);

    // Check message size
    if (captured_msg.size() > config_.max_message_size) {
      RCLCPP_WARN(node_->get_logger(), "Message from '%s' too large (%zu > %zu), skipping", topic.c_str(),
                  captured_msg.size(), config_.max_message_size);
      return false;
    }

    ros2_medkit_serialization::JsonSerializer serializer;
    auto json_data = serializer.deserialize(msg_type, captured_msg);

    // Store snapshot (use wall clock time, not sim time, for proper timestamps)
    SnapshotData snapshot;
    snapshot.fault_code = fault_code;
    snapshot.topic = topic;
    snapshot.message_type = msg_type;
    snapshot.data = json_data.dump();
    snapshot.captured_at_ns = get_wall_clock_ns();

    storage_->store_snapshot(snapshot);

    RCLCPP_DEBUG(node_->get_logger(), "Captured snapshot from '%s' for fault '%s'", topic.c_str(), fault_code.c_str());
    return true;

  } catch (const ros2_medkit_serialization::TypeNotFoundError & e) {
    RCLCPP_WARN(node_->get_logger(), "Unknown type '%s' for topic '%s': %s", msg_type.c_str(), topic.c_str(), e.what());
  } catch (const ros2_medkit_serialization::SerializationError & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to deserialize message from '%s': %s", topic.c_str(), e.what());
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Failed to process message from '%s': %s", topic.c_str(), e.what());
  }

  return false;
}

bool SnapshotCapture::capture_topic_from_cache(const std::string & fault_code, const std::string & topic) {
  std::lock_guard<std::mutex> lock(cache_mutex_);

  auto it = message_cache_.find(topic);
  if (it == message_cache_.end() || it->second.data.empty()) {
    RCLCPP_DEBUG(node_->get_logger(), "No cached data for topic '%s'", topic.c_str());
    return false;
  }

  const auto & cached = it->second;

  // Store snapshot from cache
  SnapshotData snapshot;
  snapshot.fault_code = fault_code;
  snapshot.topic = topic;
  snapshot.message_type = cached.message_type;
  snapshot.data = cached.data;
  snapshot.captured_at_ns = cached.timestamp_ns;

  storage_->store_snapshot(snapshot);

  RCLCPP_DEBUG(node_->get_logger(), "Captured snapshot from cache for '%s' (fault '%s')", topic.c_str(),
               fault_code.c_str());
  return true;
}

void SnapshotCapture::init_background_subscriptions() {
  auto topics = collect_all_configured_topics();
  if (topics.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No topics configured for background capture");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Initializing %zu background subscriptions for snapshot capture", topics.size());

  for (const auto & topic : topics) {
    std::string msg_type = get_topic_type(topic);
    if (msg_type.empty()) {
      RCLCPP_WARN(node_->get_logger(), "Cannot determine type for topic '%s', skipping background subscription",
                  topic.c_str());
      continue;
    }

    try {
      rclcpp::QoS qos = rclcpp::SensorDataQoS();

      // NOLINTNEXTLINE(performance-unnecessary-value-param)
      auto callback = [this, topic, msg_type](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
        try {
          // Check message size
          if (msg->size() > config_.max_message_size) {
            return;  // Skip oversized messages silently
          }

          ros2_medkit_serialization::JsonSerializer ser;
          auto json_data = ser.deserialize(msg_type, *msg);

          std::lock_guard<std::mutex> lock(cache_mutex_);
          auto & cached = message_cache_[topic];
          cached.topic = topic;
          cached.message_type = msg_type;
          cached.data = json_data.dump();
          // Use wall clock time, not sim time, for proper timestamps
          cached.timestamp_ns = get_wall_clock_ns();

        } catch (const std::exception & e) {
          RCLCPP_DEBUG(node_->get_logger(), "Failed to cache message from '%s': %s", topic.c_str(), e.what());
        }
      };

      auto subscription = node_->create_generic_subscription(topic, msg_type, qos, callback);
      background_subscriptions_.push_back(subscription);

      RCLCPP_DEBUG(node_->get_logger(), "Created background subscription for '%s'", topic.c_str());

    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Failed to create background subscription for '%s': %s", topic.c_str(),
                  e.what());
    }
  }
}

std::vector<std::string> SnapshotCapture::collect_all_configured_topics() const {
  std::set<std::string> unique_topics;

  // Collect from fault_specific
  for (const auto & [fault_code, topics] : config_.fault_specific) {
    unique_topics.insert(topics.begin(), topics.end());
  }

  // Collect from patterns
  for (const auto & [pattern, topics] : config_.patterns) {
    unique_topics.insert(topics.begin(), topics.end());
  }

  // Collect from default_topics
  unique_topics.insert(config_.default_topics.begin(), config_.default_topics.end());

  return {unique_topics.begin(), unique_topics.end()};
}

std::string SnapshotCapture::get_topic_type(const std::string & topic) const {
  auto topic_names_and_types = node_->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topic);
  if (it != topic_names_and_types.end() && !it->second.empty()) {
    return it->second[0];
  }
  return "";
}

}  // namespace ros2_medkit_fault_manager
