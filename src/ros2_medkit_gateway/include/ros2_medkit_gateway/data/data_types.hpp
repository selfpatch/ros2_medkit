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

#include <cstddef>
#include <cstdint>
#include <map>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/discovery/models/common.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Result of a topic discovery operation.
 *
 * Transport-neutral. Returned by TopicDataProvider::discover and friends.
 */
struct TopicInfo {
  std::string name;                 ///< Full topic path (e.g. "/powertrain/engine/temperature")
  std::string type;                 ///< Message type (e.g. "sensor_msgs/msg/Temperature")
  std::size_t publisher_count{0};   ///< Number of publishers on this topic
  std::size_t subscriber_count{0};  ///< Number of subscribers on this topic
};

/**
 * @brief Result of a single-topic sample operation.
 *
 * `has_data` is true if a message was actually received within the timeout;
 * otherwise the result is metadata-only. Metadata fields (topic_name,
 * message_type, publisher_count, subscriber_count, publishers, subscribers)
 * are populated on a best-effort basis even when no data arrived.
 *
 * `error_code` is populated when the per-topic sample failed with a recoverable
 * provider error (cold-wait cap exhausted, subscribe failed, etc) inside a
 * batch call (`sample_parallel`). Single-topic `sample()` surfaces these via
 * `tl::expected<..., ErrorInfo>` instead. When `error_code` is set, `has_data`
 * is always false and metadata may be partial.
 */
struct TopicSampleResult {
  std::string topic_name;
  std::string message_type;
  std::optional<nlohmann::json> data;  ///< Message data as JSON (nullopt when unavailable)
  bool has_data{false};                ///< Whether actual data was received
  std::size_t publisher_count{0};
  std::size_t subscriber_count{0};
  std::int64_t timestamp_ns{0};              ///< Sample timestamp in ns since epoch
  std::vector<TopicEndpoint> publishers;     ///< Publisher endpoints with QoS
  std::vector<TopicEndpoint> subscribers;    ///< Subscriber endpoints with QoS
  std::optional<std::string> error_code;     ///< Per-topic SOVD error code (set on partial-batch failure)
  std::optional<std::string> error_message;  ///< Per-topic error message paired with error_code
  int error_http_status{0};                  ///< Per-topic HTTP status (0 when error_code is empty)
};

/**
 * @brief Aggregated result of namespace-based topic discovery.
 *
 * Returned by `discover_topics_by_namespace()` in a single graph query to
 * avoid the N+1 pattern of looking up each namespace independently.
 */
struct TopicDiscoveryResult {
  std::set<std::string> namespaces;                     ///< Unique namespace prefixes (without leading slash)
  std::map<std::string, ComponentTopics> topics_by_ns;  ///< Topics grouped by namespace (with leading slash)
};

}  // namespace ros2_medkit_gateway
