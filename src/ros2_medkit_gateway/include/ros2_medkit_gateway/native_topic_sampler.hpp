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
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/models.hpp"

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
  int64_t timestamp_ns{0};                 ///< Sample timestamp in nanoseconds since epoch
  std::vector<TopicEndpoint> publishers;   ///< List of publisher endpoints with QoS
  std::vector<TopicEndpoint> subscribers;  ///< List of subscriber endpoints with QoS
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

  /**
   * @brief Get all publishers for a specific topic
   *
   * Uses native rclcpp API to discover which nodes publish to a topic.
   *
   * @param topic_name Full topic path (e.g., "/cmd_vel")
   * @return Vector of TopicEndpoint describing each publisher
   */
  std::vector<TopicEndpoint> get_topic_publishers(const std::string & topic_name);

  /**
   * @brief Get all subscribers for a specific topic
   *
   * Uses native rclcpp API to discover which nodes subscribe to a topic.
   *
   * @param topic_name Full topic path (e.g., "/cmd_vel")
   * @return Vector of TopicEndpoint describing each subscriber
   */
  std::vector<TopicEndpoint> get_topic_subscribers(const std::string & topic_name);

  /**
   * @brief Get full connection info for a topic (publishers + subscribers)
   *
   * @param topic_name Full topic path
   * @return TopicConnection with all endpoint information
   */
  TopicConnection get_topic_connection(const std::string & topic_name);

  /**
   * @brief Build a map of component FQN -> topics it publishes/subscribes
   *
   * Scans all topics in the graph and builds a mapping from each node (component)
   * to the topics it publishes and subscribes to. This is the core function for
   * the new topic-to-component association approach.
   *
   * @return Map from component FQN (e.g., "/navigation/controller_server") to ComponentTopics
   */
  std::map<std::string, ComponentTopics> build_component_topic_map();

  /**
   * @brief Get topics for a specific component
   *
   * @param component_fqn Fully qualified node name (e.g., "/navigation/controller_server")
   * @return ComponentTopics with publishes/subscribes lists
   */
  ComponentTopics get_component_topics(const std::string & component_fqn);

  /**
   * @brief Result of topic-based discovery containing namespaces and their topics
   *
   * This struct aggregates all topic-based discovery results to avoid
   * multiple ROS 2 graph queries (N+1 query problem).
   */
  struct TopicDiscoveryResult {
    std::set<std::string> namespaces;                     ///< Unique namespace prefixes
    std::map<std::string, ComponentTopics> topics_by_ns;  ///< Topics grouped by namespace
  };

  /**
   * @brief Discover namespaces and their topics in a single graph query
   *
   * This method performs a single call to get_topic_names_and_types() and
   * processes all topics to extract namespaces and group topics by namespace.
   * This avoids the N+1 query problem of calling discover_topic_namespaces()
   * followed by get_topics_for_namespace() for each namespace.
   *
   * Example: Topics ["/carter1/odom", "/carter1/cmd_vel", "/carter2/imu"]
   * Returns: {
   *   namespaces: {"carter1", "carter2"},
   *   topics_by_ns: {
   *     "/carter1": {publishes: ["/carter1/odom", "/carter1/cmd_vel"]},
   *     "/carter2": {publishes: ["/carter2/imu"]}
   *   }
   * }
   *
   * @return TopicDiscoveryResult with namespaces and topics grouped by namespace
   */
  TopicDiscoveryResult discover_topics_by_namespace();

  /**
   * @brief Discover unique namespace prefixes from all topics
   *
   * Extracts the first segment of each topic path to identify namespaces.
   * Used for topic-based component discovery when nodes are not available
   * (e.g., Isaac Sim publishing topics without creating ROS 2 nodes).
   *
   * Example: Topics ["/carter1/odom", "/carter2/cmd_vel", "/tf"]
   * Returns: {"carter1", "carter2"} (root topics like /tf are excluded)
   *
   * @note Consider using discover_topics_by_namespace() instead to avoid N+1 queries
   *
   * @return Set of unique namespace prefixes (without leading slash)
   */
  std::set<std::string> discover_topic_namespaces();

  /**
   * @brief Get all topics under a specific namespace prefix
   *
   * Returns ComponentTopics containing all topics that start with the given
   * namespace prefix. For topic-based discovery, all matched topics are
   * placed in the 'publishes' list since direction cannot be determined
   * without node information.
   *
   * @note Consider using discover_topics_by_namespace() instead to avoid N+1 queries
   *
   * @param ns_prefix Namespace prefix including leading slash (e.g., "/carter1")
   * @return ComponentTopics with matching topics in publishes list
   */
  ComponentTopics get_topics_for_namespace(const std::string & ns_prefix);

  /**
   * @brief Check if a topic is a ROS 2 system/infrastructure topic
   *
   * System topics are filtered out during topic-based discovery to avoid
   * creating spurious components. Filtered topics include:
   * - /parameter_events
   * - /rosout
   * - /clock
   *
   * Note: /tf and /tf_static are NOT filtered (useful for diagnostics).
   *
   * @param topic_name Full topic path
   * @return true if this is a system topic that should be filtered
   */
  static bool is_system_topic(const std::string & topic_name);

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
