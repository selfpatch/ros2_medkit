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

#include "ros2_medkit_gateway/ros2/transports/ros2_topic_transport.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <rclcpp/generic_publisher.hpp>
#include <stdexcept>
#include <utility>

#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/exceptions.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"

namespace ros2_medkit_gateway::ros2 {

Ros2TopicTransport::Ros2TopicTransport(rclcpp::Node * node, double default_sample_timeout_sec)
  : node_(node)
  , serializer_(std::make_shared<ros2_medkit_serialization::JsonSerializer>())
  , type_introspection_(std::make_unique<TypeIntrospection>(""))
  , default_sample_timeout_sec_(default_sample_timeout_sec) {
  if (default_sample_timeout_sec_ < 0.1 || default_sample_timeout_sec_ > 30.0) {
    RCLCPP_WARN(node_->get_logger(),
                "topic_sample_timeout_sec (%.2f) out of valid range (0.1-30.0), using default: 1.0",
                default_sample_timeout_sec_);
    default_sample_timeout_sec_ = 1.0;
  }

  RCLCPP_INFO(node_->get_logger(),
              "Ros2TopicTransport initialised (native_publishing=enabled, "
              "topic_sample_timeout=%.2fs).",
              default_sample_timeout_sec_);
}

void Ros2TopicTransport::set_data_provider(TopicDataProvider * provider) {
  data_provider_ = provider;
}

rclcpp::GenericPublisher::SharedPtr Ros2TopicTransport::get_or_create_publisher(const std::string & topic_path,
                                                                                const std::string & msg_type) {
  const std::string key = topic_path + "|" + msg_type;

  // Try read lock first (fast path).
  {
    std::shared_lock<std::shared_mutex> lock(publishers_mutex_);
    auto it = publishers_.find(key);
    if (it != publishers_.end()) {
      return it->second;
    }
  }

  // Need to create - take exclusive lock.
  std::unique_lock<std::shared_mutex> lock(publishers_mutex_);

  // Double-check (another thread might have created it).
  auto it = publishers_.find(key);
  if (it != publishers_.end()) {
    return it->second;
  }

  // Create new publisher with default QoS (reliable, keep last 10).
  auto publisher = node_->create_generic_publisher(topic_path, msg_type, rclcpp::QoS(10));
  publishers_[key] = publisher;

  RCLCPP_DEBUG(node_->get_logger(), "Created generic publisher for %s (%s)", topic_path.c_str(), msg_type.c_str());

  return publisher;
}

json Ros2TopicTransport::publish(const std::string & topic_path, const std::string & msg_type, const json & data,
                                 std::chrono::duration<double> /* timeout */) {
  try {
    auto publisher = get_or_create_publisher(topic_path, msg_type);
    rclcpp::SerializedMessage serialized_msg = serializer_->serialize(msg_type, data);
    publisher->publish(serialized_msg);

    RCLCPP_INFO(node_->get_logger(), "Published to topic '%s' with type '%s' (native)", topic_path.c_str(),
                msg_type.c_str());

    return json{{"topic", topic_path},
                {"type", msg_type},
                {"status", "published"},
                {"timestamp", std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count()}};

  } catch (const ros2_medkit_serialization::TypeNotFoundError & e) {
    RCLCPP_ERROR(node_->get_logger(), "Unknown message type '%s': %s", msg_type.c_str(), e.what());
    throw std::runtime_error("Unknown message type '" + msg_type + "': " + e.what());

  } catch (const ros2_medkit_serialization::SerializationError & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to serialize message for '%s': %s", topic_path.c_str(), e.what());
    throw std::runtime_error("Failed to serialize message for '" + topic_path + "': " + e.what());

  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to publish to topic '%s': %s", topic_path.c_str(), e.what());
    throw std::runtime_error("Failed to publish to topic '" + topic_path + "': " + e.what());
  }
}

TopicSample Ros2TopicTransport::sample(const std::string & topic_name, std::chrono::duration<double> timeout) {
  TopicSample out;
  out.topic = topic_name;

  if (!data_provider_) {
    RCLCPP_ERROR(node_->get_logger(), "TopicDataProvider not configured - sampling '%s' is unavailable",
                 topic_name.c_str());
    throw TopicNotAvailableException(topic_name);
  }

  const double timeout_sec = (timeout.count() > 0.0) ? timeout.count() : default_sample_timeout_sec_;
  const auto timeout_ms = std::chrono::milliseconds{static_cast<std::int64_t>(std::max(timeout_sec, 0.0) * 1000.0)};

  auto r = data_provider_->sample(topic_name, timeout_ms);
  if (!r) {
    const auto & err = r.error();
    RCLCPP_WARN(node_->get_logger(), "TopicDataProvider::sample('%s') failed: %s [%d]", topic_name.c_str(),
                err.message.c_str(), err.http_status);
    if (err.http_status == 404) {
      throw TopicNotAvailableException(topic_name);
    }
    throw ProviderErrorException(err);
  }

  const TopicSampleResult & res = *r;

  if (res.message_type.empty() && !res.has_data) {
    throw TopicNotAvailableException(topic_name);
  }

  out.success = true;
  out.publisher_count = static_cast<uint64_t>(res.publisher_count);
  out.subscriber_count = static_cast<uint64_t>(res.subscriber_count);
  out.type = res.message_type;

  if (res.has_data && res.data) {
    out.status = "data";
    out.data = *res.data;
  } else {
    out.status = "metadata_only";
  }

  if (!res.message_type.empty()) {
    try {
      TopicTypeInfo info = type_introspection_->get_type_info(res.message_type);
      out.type_info = json{{"schema", info.schema}, {"default_value", info.default_value}};
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(node_->get_logger(), "Could not get type info for '%s': %s", res.message_type.c_str(), e.what());
    }
  }

  return out;
}

std::pair<uint64_t, uint64_t> Ros2TopicTransport::count_publishers_subscribers(const std::string & topic_name) const {
  try {
    return {node_->count_publishers(topic_name), node_->count_subscribers(topic_name)};
  } catch (const std::exception & e) {
    RCLCPP_DEBUG(node_->get_logger(), "count_publishers/subscribers failed for '%s': %s", topic_name.c_str(), e.what());
    return {0, 0};
  }
}

}  // namespace ros2_medkit_gateway::ros2
