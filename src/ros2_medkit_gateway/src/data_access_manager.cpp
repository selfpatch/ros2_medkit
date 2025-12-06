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

#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <cmath>
#include <future>
#include <sstream>

#include "ros2_medkit_gateway/exceptions.hpp"

namespace ros2_medkit_gateway {

DataAccessManager::DataAccessManager(rclcpp::Node * node)
  : node_(node)
  , cli_wrapper_(std::make_unique<ROS2CLIWrapper>())
  , output_parser_(std::make_unique<OutputParser>())
  , type_introspection_(
        std::make_unique<TypeIntrospection>(ament_index_cpp::get_package_share_directory("ros2_medkit_gateway") + "/scr"
                                                                                                                  "ipt"
                                                                                                                  "s"))
  , native_sampler_(std::make_unique<NativeTopicSampler>(node))
  , max_parallel_samples_(node->declare_parameter<int>("max_parallel_topic_samples", 10)) {
  // Validate max_parallel_samples_ against allowed range [1, 50]
  if (max_parallel_samples_ < 1 || max_parallel_samples_ > 50) {
    RCLCPP_WARN(node_->get_logger(), "max_parallel_topic_samples (%d) out of valid range (1-50), using default: 10",
                max_parallel_samples_);
    max_parallel_samples_ = 10;
  }

  // CLI is still needed for publishing (ros2 topic pub)
  if (!cli_wrapper_->is_command_available("ros2")) {
    RCLCPP_WARN(node_->get_logger(), "ROS 2 CLI not found, publishing will not be available");
  }

  RCLCPP_INFO(node_->get_logger(), "DataAccessManager initialized (native_sampling=enabled, max_parallel_samples=%d)",
              max_parallel_samples_);
}

json DataAccessManager::publish_to_topic(const std::string & topic_path, const std::string & msg_type,
                                         const json & data, double timeout_sec) {
  try {
    // Convert JSON data to string for ros2 topic pub
    // Note: data.dump() produces JSON format, but ROS 2 CLI accepts JSON
    // as valid YAML (JSON is a subset of YAML 1.2)
    std::string yaml_data = data.dump();

    // TODO(mfaferek93) #32: Check timeout command availability
    // GNU coreutils 'timeout' may not be available on all systems (BSD, containers)
    // Should check in constructor or provide fallback mechanism
    std::ostringstream cmd;
    cmd << "timeout " << static_cast<int>(std::ceil(timeout_sec)) << "s " << "ros2 topic pub --once -w 0 "
        << ROS2CLIWrapper::escape_shell_arg(topic_path) << " " << ROS2CLIWrapper::escape_shell_arg(msg_type) << " "
        << ROS2CLIWrapper::escape_shell_arg(yaml_data);

    RCLCPP_INFO(node_->get_logger(), "Executing: %s", cmd.str().c_str());

    std::string output = cli_wrapper_->exec(cmd.str());

    RCLCPP_INFO(node_->get_logger(), "Published to topic '%s' with type '%s'", topic_path.c_str(), msg_type.c_str());

    json result = {{"topic", topic_path},
                   {"type", msg_type},
                   {"status", "published"},
                   {"timestamp", std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count()}};

    return result;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to publish to topic '%s': %s", topic_path.c_str(), e.what());
    throw std::runtime_error("Failed to publish to topic '" + topic_path + "': " + e.what());
  }
}

json DataAccessManager::get_topic_sample_with_fallback(const std::string & topic_name, double timeout_sec) {
  // Always use native sampling - much faster for idle topics
  return get_topic_sample_native(topic_name, timeout_sec);
}

json DataAccessManager::get_component_data_with_fallback(const std::string & component_namespace, double timeout_sec) {
  // Always use native sampling - much faster for idle topics
  return get_component_data_native(component_namespace, timeout_sec);
}

std::vector<std::string> DataAccessManager::find_component_topics_native(const std::string & component_namespace) {
  auto topics = native_sampler_->discover_topics(component_namespace);
  std::vector<std::string> topic_names;
  topic_names.reserve(topics.size());
  for (const auto & topic : topics) {
    topic_names.push_back(topic.name);
  }
  RCLCPP_INFO(node_->get_logger(), "Found %zu topics under namespace '%s' (native)", topic_names.size(),
              component_namespace.c_str());
  return topic_names;
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
  auto sample = native_sampler_->sample_topic(topic_name, timeout_sec);

  if (!sample.message_type.empty() || sample.has_data) {
    return sample_result_to_json(sample);
  }

  // Topic not found at all
  throw TopicNotAvailableException(topic_name);
}

json DataAccessManager::get_component_data_native(const std::string & component_namespace, double timeout_sec) {
  json result = json::array();

  // Use native discovery
  auto topics = find_component_topics_native(component_namespace);

  if (topics.empty()) {
    RCLCPP_WARN(node_->get_logger(), "No topics found under namespace '%s'", component_namespace.c_str());
    return result;
  }

  // Use native parallel sampling with publisher count optimization
  auto samples = native_sampler_->sample_topics_parallel(topics, timeout_sec, max_parallel_samples_);

  for (const auto & sample : samples) {
    try {
      result.push_back(sample_result_to_json(sample));
    } catch (const std::exception & e) {
      RCLCPP_WARN(node_->get_logger(), "Failed to convert sample for '%s': %s", sample.topic_name.c_str(), e.what());
    }
  }

  return result;
}

}  // namespace ros2_medkit_gateway
