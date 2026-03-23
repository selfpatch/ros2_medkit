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

#include "ros2_medkit_gateway/data_access_manager.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <rclcpp/generic_publisher.hpp>
#include <sstream>

#include "ros2_medkit_gateway/exceptions.hpp"
#include "ros2_medkit_serialization/serialization_error.hpp"

namespace ros2_medkit_gateway {

DataAccessManager::DataAccessManager(rclcpp::Node * node)
  : node_(node)
  , serializer_(std::make_shared<ros2_medkit_serialization::JsonSerializer>())
  , type_introspection_(
        std::make_unique<TypeIntrospection>(ament_index_cpp::get_package_share_directory("ros2_medkit_gateway") + "/scr"
                                                                                                                  "ipt"
                                                                                                                  "s"))
  , native_sampler_(std::make_unique<NativeTopicSampler>(node))
  , max_parallel_samples_(static_cast<int>(node->declare_parameter<int64_t>("max_parallel_topic_samples", 10)))
  , topic_sample_timeout_sec_(node->declare_parameter<double>("topic_sample_timeout_sec", 1.0)) {
  // Validate max_parallel_samples_ against allowed range [1, 50]
  if (max_parallel_samples_ < 1 || max_parallel_samples_ > 50) {
    RCLCPP_WARN(node_->get_logger(), "max_parallel_topic_samples (%d) out of valid range (1-50), using default: 10",
                max_parallel_samples_);
    max_parallel_samples_ = 10;
  }

  // Validate topic_sample_timeout_sec_ against allowed range [0.1, 30.0]
  if (topic_sample_timeout_sec_ < 0.1 || topic_sample_timeout_sec_ > 30.0) {
    RCLCPP_WARN(node_->get_logger(),
                "topic_sample_timeout_sec (%.2f) out of valid range (0.1-30.0), using default: 1.0",
                topic_sample_timeout_sec_);
    topic_sample_timeout_sec_ = 1.0;
  }

  RCLCPP_INFO(node_->get_logger(),
              "DataAccessManager initialized (native_sampling=enabled, native_publishing=enabled, "
              "max_parallel_samples=%d, topic_sample_timeout=%.2fs)",
              max_parallel_samples_, topic_sample_timeout_sec_);
}

rclcpp::GenericPublisher::SharedPtr DataAccessManager::get_or_create_publisher(const std::string & topic_path,
                                                                               const std::string & msg_type) {
  const std::string key = topic_path + "|" + msg_type;

  // Try read lock first (fast path)
  {
    std::shared_lock<std::shared_mutex> lock(publishers_mutex_);
    auto it = publishers_.find(key);
    if (it != publishers_.end()) {
      return it->second;
    }
  }

  // Need to create - take exclusive lock
  std::unique_lock<std::shared_mutex> lock(publishers_mutex_);

  // Double-check (another thread might have created it)
  auto it = publishers_.find(key);
  if (it != publishers_.end()) {
    return it->second;
  }

  // Create new publisher with default QoS (reliable, keep last 10)
  auto publisher = node_->create_generic_publisher(topic_path, msg_type, rclcpp::QoS(10));
  publishers_[key] = publisher;

  RCLCPP_DEBUG(node_->get_logger(), "Created generic publisher for %s (%s)", topic_path.c_str(), msg_type.c_str());

  return publisher;
}

json DataAccessManager::publish_to_topic(const std::string & topic_path, const std::string & msg_type,
                                         const json & data, double /* timeout_sec */) {
  try {
    // Step 1: Get or create cached publisher
    auto publisher = get_or_create_publisher(topic_path, msg_type);

    // Step 2: Serialize JSON to CDR format
    rclcpp::SerializedMessage serialized_msg = serializer_->serialize(msg_type, data);

    // Step 3: Publish serialized message
    publisher->publish(serialized_msg);

    RCLCPP_INFO(node_->get_logger(), "Published to topic '%s' with type '%s' (native)", topic_path.c_str(),
                msg_type.c_str());

    json result = {{"topic", topic_path},
                   {"type", msg_type},
                   {"status", "published"},
                   {"timestamp", std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count()}};

    return result;

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

json DataAccessManager::get_topic_sample_with_fallback(const std::string & topic_name, double timeout_sec) {
  // Use configured parameter if timeout_sec is negative (default)
  double effective_timeout = (timeout_sec < 0) ? topic_sample_timeout_sec_ : timeout_sec;
  // Always use native sampling - much faster for idle topics
  return get_topic_sample_native(topic_name, effective_timeout);
}

json DataAccessManager::sample_result_to_json(const TopicSampleResult & sample) {
  json result;
  result["topic"] = sample.topic_name;
  result["timestamp"] = sample.timestamp_ns;
  result["publisher_count"] = sample.publisher_count;
  result["subscriber_count"] = sample.subscriber_count;

  if (sample.has_data && sample.data) {
    result["status"] = "data";
    result["data"] = *sample.data;
  } else {
    result["status"] = "metadata_only";
  }

  // Add endpoint information with QoS
  json publishers_json = json::array();
  for (const auto & pub : sample.publishers) {
    publishers_json.push_back(pub.to_json());
  }
  result["publishers"] = publishers_json;

  json subscribers_json = json::array();
  for (const auto & sub : sample.subscribers) {
    subscribers_json.push_back(sub.to_json());
  }
  result["subscribers"] = subscribers_json;

  // Enrich with message type
  if (!sample.message_type.empty()) {
    result["type"] = sample.message_type;

    // Try to add schema/default value info
    try {
      TopicTypeInfo type_info = type_introspection_->get_type_info(sample.message_type);
      result["type_info"] = {{"schema", type_info.schema}, {"default_value", type_info.default_value}};
    } catch (const std::exception & e) {
      RCLCPP_DEBUG(node_->get_logger(), "Could not get type info for '%s': %s", sample.message_type.c_str(), e.what());
    }
  }

  return result;
}

json DataAccessManager::get_topic_sample_native(const std::string & topic_name, double timeout_sec) {
  RCLCPP_DEBUG(node_->get_logger(), "get_topic_sample_native: topic='%s', timeout=%.2f", topic_name.c_str(),
               timeout_sec);
  auto sample = native_sampler_->sample_topic(topic_name, timeout_sec);
  RCLCPP_DEBUG(node_->get_logger(), "get_topic_sample_native: sample returned, has_data=%d, type='%s'", sample.has_data,
               sample.message_type.c_str());

  if (!sample.message_type.empty() || sample.has_data) {
    return sample_result_to_json(sample);
  }

  // Topic not found at all
  RCLCPP_DEBUG(node_->get_logger(), "get_topic_sample_native: topic not available '%s'", topic_name.c_str());
  throw TopicNotAvailableException(topic_name);
}

}  // namespace ros2_medkit_gateway
