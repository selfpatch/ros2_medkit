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

#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/transports/topic_transport.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

class TopicDataProvider;  // forward decl, see core/data/topic_data_provider.hpp
class TypeIntrospection;  // forward decl, defined in core/type_introspection.hpp

/**
 * @brief Application service for topic publish + sample.
 *
 * Pure C++; ROS-side I/O is performed by the injected TopicTransport adapter
 * (typically Ros2TopicTransport). Type introspection and the per-entity
 * TopicDataProvider pointer remain on the manager because they are consumed
 * directly by handlers and the discovery manager.
 */
class DataAccessManager {
 public:
  /**
   * @param transport Concrete TopicTransport adapter. Manager takes shared
   *                  ownership.
   * @param topic_sample_timeout_sec Default sample timeout in seconds. Used
   *                                 when callers pass a negative timeout.
   */
  explicit DataAccessManager(std::shared_ptr<TopicTransport> transport, double topic_sample_timeout_sec = 1.0);

  ~DataAccessManager() = default;

  DataAccessManager(const DataAccessManager &) = delete;
  DataAccessManager & operator=(const DataAccessManager &) = delete;
  DataAccessManager(DataAccessManager &&) = delete;
  DataAccessManager & operator=(DataAccessManager &&) = delete;

  /**
   * @brief Publish data to a specific topic.
   *
   * @param topic_path Full topic path (e.g., /chassis/brakes/command).
   * @param msg_type ROS 2 message type (e.g., std_msgs/msg/Float32).
   * @param data JSON data to publish.
   * @param timeout_sec Timeout for the publish operation.
   * @return JSON with publish status.
   */
  json publish_to_topic(const std::string & topic_path, const std::string & msg_type, const json & data,
                        double timeout_sec = 5.0);

  /**
   * @brief Get topic sample with fallback to metadata on timeout.
   *
   * If the topic is publishing, returns actual data with type info. If the
   * topic times out, returns metadata (type, schema, pub/sub counts) instead
   * of an error.
   *
   * @param topic_name Full topic path.
   * @param timeout_sec Timeout for data retrieval. Use -1.0 to use the
   *                    configured default.
   */
  json get_topic_sample_with_fallback(const std::string & topic_name, double timeout_sec = -1.0);

  /**
   * @brief Get single topic sample using the native fast path.
   *
   * Fast path for single-topic sampling. When no publishers are present,
   * returns metadata-only without calling into the transport.
   *
   * @param topic_name Full topic path.
   * @param timeout_sec Timeout for sampling (only used if topic has publishers).
   */
  json get_topic_sample_native(const std::string & topic_name, double timeout_sec = 1.0);

  /**
   * @brief Get the type introspection instance (used by handlers and discovery).
   *
   * Forwarded to the transport adapter, which owns the rclcpp-coupled
   * implementation. May return nullptr for transports that do not support
   * introspection.
   */
  TypeIntrospection * get_type_introspection() const {
    return transport_ ? transport_->get_type_introspection() : nullptr;
  }

  /**
   * @brief Attach a TopicDataProvider for sampling.
   *
   * The provider owns the pool-backed subscription path. Non-owning pointer;
   * caller retains ownership. Safe to call once at wiring time.
   */
  void set_topic_data_provider(TopicDataProvider * provider) {
    topic_data_provider_ = provider;
  }

  TopicDataProvider * get_topic_data_provider() const {
    return topic_data_provider_;
  }

  /**
   * @brief Get the configured topic sample timeout in seconds.
   */
  double get_topic_sample_timeout() const {
    return topic_sample_timeout_sec_;
  }

 private:
  std::shared_ptr<TopicTransport> transport_;
  TopicDataProvider * topic_data_provider_{nullptr};  ///< Non-owning; set at wiring time.
  double topic_sample_timeout_sec_;
};

}  // namespace ros2_medkit_gateway
