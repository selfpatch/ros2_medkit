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

#include <chrono>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/data/data_types.hpp"
#include "ros2_medkit_gateway/discovery/models/common.hpp"
#include "ros2_medkit_gateway/models/error_info.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Transport-neutral interface for reading and discovering topic data.
 *
 * This is the infrastructure abstraction consumed by DataAccessManager,
 * DiscoveryManager and RuntimeDiscovery. The ROS 2 default implementation
 * (`Ros2TopicDataProvider`) services requests via a long-lived subscription
 * pool running under `Ros2SubscriptionExecutor`. Alternate transports (Zenoh,
 * XRCE-DDS, mocks for tests) plug in by providing another implementation of
 * this interface.
 *
 * @par Thread safety
 * All methods may be called concurrently from multiple HTTP handler threads.
 * Implementations must provide internal synchronization.
 *
 * @par Relationship to plugin-facing DataProvider
 * The unrelated `ros2_medkit_gateway::DataProvider` in `providers/data_provider.hpp`
 * is the plugin-authoring interface for per-entity SOVD data resource
 * delegation (`list_data` / `read_data` / `write_data`). It operates at a
 * different architectural layer. Despite the similar name, the two have no
 * inheritance or composition relationship.
 */
class TopicDataProvider {
 public:
  virtual ~TopicDataProvider() = default;

  TopicDataProvider() = default;
  TopicDataProvider(const TopicDataProvider &) = delete;
  TopicDataProvider & operator=(const TopicDataProvider &) = delete;
  TopicDataProvider(TopicDataProvider &&) = delete;
  TopicDataProvider & operator=(TopicDataProvider &&) = delete;

  // ---- Sampling ----

  /**
   * @brief Sample a single topic, returning either fresh data or metadata-only.
   *
   * If the topic has no publishers, returns metadata-only immediately
   * (`has_data == false`). Otherwise, waits up to `timeout` for a message.
   */
  [[nodiscard]] virtual tl::expected<TopicSampleResult, ErrorInfo> sample(const std::string & topic,
                                                                          std::chrono::milliseconds timeout) = 0;

  /**
   * @brief Sample multiple topics concurrently, returning one result per input topic.
   *
   * Order of the returned vector matches the input order. Idle topics return
   * immediately with metadata only; active topics wait up to `timeout`.
   */
  [[nodiscard]] virtual tl::expected<std::vector<TopicSampleResult>, ErrorInfo>
  sample_parallel(const std::vector<std::string> & topics, std::chrono::milliseconds timeout) = 0;

  // ---- Discovery / graph queries (synchronous, metadata only) ----

  [[nodiscard]] virtual std::optional<TopicInfo> get_topic_info(const std::string & topic) = 0;
  [[nodiscard]] virtual bool has_publishers(const std::string & topic) = 0;
  [[nodiscard]] virtual std::vector<TopicInfo> discover(const std::string & namespace_prefix) = 0;
  [[nodiscard]] virtual std::vector<TopicInfo> discover_all() = 0;

  [[nodiscard]] virtual std::map<std::string, ComponentTopics> build_component_topic_map() = 0;
  [[nodiscard]] virtual ComponentTopics get_component_topics(const std::string & component_fqn) = 0;

  [[nodiscard]] virtual TopicDiscoveryResult discover_topics_by_namespace() = 0;
  [[nodiscard]] virtual std::set<std::string> discover_topic_namespaces() = 0;
  [[nodiscard]] virtual ComponentTopics get_topics_for_namespace(const std::string & ns_prefix) = 0;

  [[nodiscard]] virtual std::vector<TopicEndpoint> get_topic_publishers(const std::string & topic) = 0;
  [[nodiscard]] virtual std::vector<TopicEndpoint> get_topic_subscribers(const std::string & topic) = 0;
  [[nodiscard]] virtual TopicConnection get_topic_connection(const std::string & topic) = 0;
};

}  // namespace ros2_medkit_gateway
