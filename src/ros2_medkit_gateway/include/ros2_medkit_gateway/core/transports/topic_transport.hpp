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
#include <cstdint>
#include <nlohmann/json.hpp>
#include <string>
#include <utility>

namespace ros2_medkit_gateway {

class TypeIntrospection;  // forward decl - defined in core/type_introspection.hpp.

using json = nlohmann::json;

/// Neutral result for a topic-sample call routed through the transport port.
///
/// The existing `TopicSampleResult` struct in
/// `core/data/data_types.hpp` belongs to the topic-data-provider plugin
/// surface and has a different shape (per-topic batch errors, endpoint QoS,
/// etc.). The transport-level result here is intentionally narrower: it
/// captures only what the manager facade needs to forward to handlers. A
/// later phase reconciles the two if it makes sense to do so.
struct TopicSample {
  bool success;
  std::string status;  ///< "data" | "metadata_only" | "error"
  std::string topic;
  std::string type;
  json data = json::object();
  json type_info = json::object();
  uint64_t publisher_count = 0;
  uint64_t subscriber_count = 0;
  std::string error_message;
};

/// Port: topic publish + sample. Concrete adapter
/// `Ros2TopicTransport` (src/ros2/transports/) implements it via
/// rclcpp GenericPublisher / NativeTopicSampler.
class TopicTransport {
 public:
  TopicTransport() = default;
  TopicTransport(const TopicTransport &) = delete;
  TopicTransport & operator=(const TopicTransport &) = delete;
  TopicTransport(TopicTransport &&) = delete;
  TopicTransport & operator=(TopicTransport &&) = delete;
  virtual ~TopicTransport() = default;

  /// Publish JSON-encoded data to a topic. Implementations cache the underlying
  /// publisher keyed by (topic, type).
  virtual json publish(const std::string & topic_path, const std::string & msg_type, const json & data,
                       std::chrono::duration<double> timeout) = 0;

  /// Sample a topic with timeout. Implementations fall back to metadata if no
  /// publisher is active. `timeout < 0` means "use the implementation default".
  virtual TopicSample sample(const std::string & topic_name, std::chrono::duration<double> timeout) = 0;

  /// Publisher and subscriber count for a topic. Implementations may return
  /// stale data if the underlying graph snapshot is cached.
  virtual std::pair<uint64_t, uint64_t> count_publishers_subscribers(const std::string & topic_name) const = 0;

  /// Type-introspection helper used by handlers to enrich SOVD payloads with
  /// schema and default-value templates. The TypeIntrospection backend is
  /// rclcpp-coupled in the current implementation; the transport adapter owns
  /// it so the manager body remains middleware-neutral. May return nullptr in
  /// transports that do not support introspection (test mocks, alternative
  /// middlewares).
  virtual TypeIntrospection * get_type_introspection() const = 0;
};

}  // namespace ros2_medkit_gateway
