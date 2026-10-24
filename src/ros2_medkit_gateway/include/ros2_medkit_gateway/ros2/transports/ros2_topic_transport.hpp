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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <utility>

#include "ros2_medkit_gateway/core/transports/topic_transport.hpp"
#include "ros2_medkit_gateway/core/types/type_schema_provider.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {

class TopicDataProvider;

namespace ros2 {

/**
 * @brief rclcpp adapter implementing the TopicTransport port.
 *
 * Owns the publisher cache and the JsonSerializer instance. Message type
 * metadata for samples comes from the TypeSchemaProvider it holds a
 * reference to.
 * The sample path delegates to an attached TopicDataProvider; the adapter
 * itself does not maintain a sampling executor.
 */
class Ros2TopicTransport : public TopicTransport {
 public:
  /**
   * @param node Non-owning ROS node used for graph queries and generic
   *             publisher creation.
   * @param default_sample_timeout_sec Used when callers pass a non-positive
   *                                   timeout into sample().
   * @param schemas Type schemas for the metadata a sample carries. Must
   *                outlive the transport.
   */
  Ros2TopicTransport(rclcpp::Node * node, double default_sample_timeout_sec, TypeSchemaProvider & schemas);
  ~Ros2TopicTransport() override = default;

  Ros2TopicTransport(const Ros2TopicTransport &) = delete;
  Ros2TopicTransport & operator=(const Ros2TopicTransport &) = delete;
  Ros2TopicTransport(Ros2TopicTransport &&) = delete;
  Ros2TopicTransport & operator=(Ros2TopicTransport &&) = delete;

  /**
   * @brief Attach the per-entity TopicDataProvider used for the sample path.
   *
   * Non-owning. The wiring layer drops the provider before destroying it by
   * passing nullptr. Concurrent set during steady-state traffic is not
   * supported; the wiring contract expects this to be called from gateway
   * lifecycle code only.
   */
  void set_data_provider(TopicDataProvider * provider);

  json publish(const std::string & topic_path, const std::string & msg_type, const json & data,
               std::chrono::duration<double> timeout) override;

  TopicSample sample(const std::string & topic_name, std::chrono::duration<double> timeout) override;

  std::pair<uint64_t, uint64_t> count_publishers_subscribers(const std::string & topic_name) const override;

 private:
  rclcpp::GenericPublisher::SharedPtr get_or_create_publisher(const std::string & topic_path,
                                                              const std::string & msg_type);

  rclcpp::Node * node_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;
  TypeSchemaProvider & schemas_;

  mutable std::shared_mutex publishers_mutex_;
  std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr> publishers_;

  TopicDataProvider * data_provider_{nullptr};
  double default_sample_timeout_sec_;
};

}  // namespace ros2
}  // namespace ros2_medkit_gateway
