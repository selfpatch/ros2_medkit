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

#include <chrono>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief Result of a topic discovery operation
 */
struct TopicInfo {
  std::string name;            ///< Full topic path (e.g., "/powertrain/engine/temperature")
  std::string type;            ///< Message type (e.g., "sensor_msgs/msg/Temperature")
  size_t publisher_count{0};   ///< Number of publishers on this topic
  size_t subscriber_count{0};  ///< Number of subscribers on this topic
};

/**
 * @brief Result of a topic sample operation
 */
struct TopicSampleResult {
  std::string topic_name;
  std::string message_type;
  std::optional<json> data;  ///< Message data as JSON (nullopt if unavailable)
  bool has_data{false};      ///< Whether actual data was received
  size_t publisher_count{0};
  size_t subscriber_count{0};
  int64_t timestamp_ns{0};  ///< Sample timestamp in nanoseconds since epoch
};

/**
 * @brief Native rclcpp-based topic sampler that avoids CLI overhead
 *
 * This class provides fast topic discovery and sampling using native rclcpp APIs
 * instead of shelling out to ros2 CLI commands. Key benefits:
 * - No process spawn overhead (~100ms per CLI call saved)
 * - Requires GNU timeout command only for topics with active publishers
 * - Faster timeout detection for idle topics (return instantly)
 * - Publisher count check before attempting to sample (skip idle topics immediately)
 *
 * Usage:
 * @code
 * NativeTopicSampler sampler(node);
 * auto topics = sampler.discover_topics("/powertrain/engine");
 * auto sample = sampler.sample_topic("/powertrain/engine/temperature", 1.0);
 * @endcode
 */
class NativeTopicSampler {
 public:
  /**
   * @brief Construct a new NativeTopicSampler
   * @param node Pointer to the owning node (must outlive sampler)
   */
  explicit NativeTopicSampler(rclcpp::Node * node);

  ~NativeTopicSampler() = default;

  // Disable copy/move (holds raw pointer to node)
  NativeTopicSampler(const NativeTopicSampler &) = delete;
  NativeTopicSampler & operator=(const NativeTopicSampler &) = delete;
  NativeTopicSampler(NativeTopicSampler &&) = delete;
  NativeTopicSampler & operator=(NativeTopicSampler &&) = delete;

  /**
   * @brief Discover all topics in the ROS 2 graph
   * @return Vector of TopicInfo for all available topics
   */
  std::vector<TopicInfo> discover_all_topics();

  /**
   * @brief Discover topics under a specific namespace
   *
   * @param namespace_prefix Namespace to filter by (e.g., "/powertrain/engine")
   * @return Vector of TopicInfo for matching topics
   */
  std::vector<TopicInfo> discover_topics(const std::string & namespace_prefix);

  /**
   * @brief Get metadata for a specific topic (type, pub/sub counts)
   *
   * This is very fast as it only queries the graph, no message waiting.
   *
   * @param topic_name Full topic path
   * @return TopicInfo with metadata, or nullopt if topic doesn't exist
   */
  std::optional<TopicInfo> get_topic_info(const std::string & topic_name);

  /**
   * @brief Check if a topic has active publishers
   *
   * Fast check to determine if sampling would be useful.
   *
   * @param topic_name Full topic path
   * @return true if at least one publisher exists
   */
  bool has_publishers(const std::string & topic_name);

  /**
   * @brief Sample a topic to get one message
   *
   * Creates a temporary subscription, waits for a message, and returns.
   * If no message arrives within timeout, returns result with has_data=false.
   *
   * **Important**: This method handles topics without publishers gracefully
   * by checking publisher count first and returning immediately with metadata
   * if no publishers are available.
   *
   * @param topic_name Full topic path
   * @param timeout_sec Maximum time to wait for a message
   * @return TopicSampleResult with data (if received) or metadata only
   */
  TopicSampleResult sample_topic(const std::string & topic_name, double timeout_sec = 1.0);

  /**
   * @brief Sample multiple topics in parallel with metadata fallback
   *
   * Efficiently samples multiple topics, returning metadata immediately
   * for topics without publishers and only waiting for active topics.
   *
   * @param topic_names List of topic paths to sample
   * @param timeout_sec Timeout per topic
   * @param max_parallel Maximum concurrent sampling operations
   * @return Vector of TopicSampleResult for each topic
   */
  std::vector<TopicSampleResult> sample_topics_parallel(const std::vector<std::string> & topic_names,
                                                        double timeout_sec = 1.0, int max_parallel = 10);

 private:
  /**
   * @brief Parse YAML-formatted message string to JSON
   */
  json parse_message_yaml(const std::string & yaml_str);

  /**
   * @brief Get the message type for a topic from the graph
   */
  std::string get_topic_type(const std::string & topic_name);

  rclcpp::Node * node_;
};

}  // namespace ros2_medkit_gateway
