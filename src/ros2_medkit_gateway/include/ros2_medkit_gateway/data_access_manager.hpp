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

#pragma once

#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_gateway/native_topic_sampler.hpp"
#include "ros2_medkit_gateway/type_introspection.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

class DataAccessManager {
 public:
  explicit DataAccessManager(rclcpp::Node * node);

  /**
   * @brief Publish data to a specific topic
   * @param topic_path Full topic path (e.g., /chassis/brakes/command)
   * @param msg_type ROS 2 message type (e.g., std_msgs/msg/Float32)
   * @param data JSON data to publish
   * @param timeout_sec Timeout for the publish operation
   * @return JSON with publish status
   */
  json publish_to_topic(const std::string & topic_path, const std::string & msg_type, const json & data,
                        double timeout_sec = 5.0);

  /**
   * @brief Get topic sample with fallback to metadata on timeout
   *
   * If the topic is publishing, returns actual data with type info.
   * If the topic times out, returns metadata (type, schema, pub/sub counts) instead of error.
   *
   * @param topic_name Full topic path (e.g., "/powertrain/engine/temperature")
   * @param timeout_sec Timeout for data retrieval. Use -1.0 to use the topic_sample_timeout_sec parameter (default)
   * @return JSON object with one of two structures:
   *   - status="data": {topic, timestamp, data, status, type, type_info, publisher_count, subscriber_count}
   *   - status="metadata_only": {topic, timestamp, status, type, type_info, publisher_count, subscriber_count}
   * @throws TopicNotAvailableException if topic doesn't exist or metadata cannot be retrieved
   */
  json get_topic_sample_with_fallback(const std::string & topic_name, double timeout_sec = -1.0);

  /**
   * @brief Get the type introspection instance
   */
  TypeIntrospection * get_type_introspection() const {
    return type_introspection_.get();
  }

  /**
   * @brief Get the native topic sampler instance
   *
   * Used by DiscoveryManager to build component-topic mappings.
   */
  NativeTopicSampler * get_native_sampler() const {
    return native_sampler_.get();
  }

  /**
   * @brief Get single topic sample using native rclcpp APIs
   *
   * Fast path for single topic sampling with publisher count check.
   *
   * @param topic_name Full topic path
   * @param timeout_sec Timeout for sampling (only used if topic has publishers)
   * @return JSON with topic data or metadata
   */
  json get_topic_sample_native(const std::string & topic_name, double timeout_sec = 1.0);

  /**
   * @brief Get the configured topic sample timeout
   * @return Timeout in seconds for topic sampling
   */
  double get_topic_sample_timeout() const {
    return topic_sample_timeout_sec_;
  }

 private:
  /**
   * @brief Convert TopicSampleResult to JSON with type info enrichment
   */
  json sample_result_to_json(const TopicSampleResult & sample);

  /**
   * @brief Get or create a cached GenericPublisher for a topic
   * @param topic_path Full topic path
   * @param msg_type ROS 2 message type
   * @return Shared pointer to GenericPublisher
   */
  rclcpp::GenericPublisher::SharedPtr get_or_create_publisher(const std::string & topic_path,
                                                              const std::string & msg_type);

  rclcpp::Node * node_;

  /// JSON serializer for native message serialization
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;

  /// Cached publishers (topic+type -> publisher)
  std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;

  /// Mutex for thread-safe publisher cache access
  mutable std::shared_mutex publishers_mutex_;

  std::unique_ptr<TypeIntrospection> type_introspection_;
  std::unique_ptr<NativeTopicSampler> native_sampler_;
  int max_parallel_samples_;
  double topic_sample_timeout_sec_;

  /**
   * @brief Get default timeout for topic sampling (from parameter)
   */
  double get_default_topic_timeout() const {
    return topic_sample_timeout_sec_;
  }
};

}  // namespace ros2_medkit_gateway
