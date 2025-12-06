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

#include "ros2_medkit_gateway/native_topic_sampler.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sstream>
#include <thread>

#include "ros2_medkit_gateway/output_parser.hpp"
#include "ros2_medkit_gateway/ros2_cli_wrapper.hpp"

namespace ros2_medkit_gateway {

NativeTopicSampler::NativeTopicSampler(rclcpp::Node * node) : node_(node) {
  if (!node_) {
    throw std::invalid_argument("NativeTopicSampler requires a valid node pointer");
  }
  RCLCPP_INFO(node_->get_logger(), "NativeTopicSampler initialized");
}

std::vector<TopicInfo> NativeTopicSampler::discover_all_topics() {
  std::vector<TopicInfo> result;

  // Use native rclcpp API to get all topics with their types
  auto topic_names_and_types = node_->get_topic_names_and_types();

  for (const auto & [topic_name, types] : topic_names_and_types) {
    TopicInfo info;
    info.name = topic_name;

    // A topic can have multiple types (rare but possible), take the first
    if (!types.empty()) {
      info.type = types[0];
    }

    // Get publisher/subscriber counts using native APIs
    info.publisher_count = node_->count_publishers(topic_name);
    info.subscriber_count = node_->count_subscribers(topic_name);

    result.push_back(info);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Discovered %zu topics via native API", result.size());
  return result;
}

std::vector<TopicInfo> NativeTopicSampler::discover_topics(const std::string & namespace_prefix) {
  auto all_topics = discover_all_topics();
  std::vector<TopicInfo> filtered;

  // Filter topics that match the namespace prefix (followed by '/' or exact match)
  std::copy_if(
      all_topics.begin(), all_topics.end(), std::back_inserter(filtered), [&namespace_prefix](const TopicInfo & topic) {
        if (topic.name.find(namespace_prefix) != 0) {
          return false;
        }
        return topic.name.length() == namespace_prefix.length() || topic.name[namespace_prefix.length()] == '/';
      });

  RCLCPP_DEBUG(node_->get_logger(), "Found %zu topics under namespace '%s'", filtered.size(), namespace_prefix.c_str());
  return filtered;
}

std::optional<TopicInfo> NativeTopicSampler::get_topic_info(const std::string & topic_name) {
  auto topic_names_and_types = node_->get_topic_names_and_types();

  auto it = topic_names_and_types.find(topic_name);
  if (it == topic_names_and_types.end()) {
    return std::nullopt;
  }

  TopicInfo info;
  info.name = topic_name;
  if (!it->second.empty()) {
    info.type = it->second[0];
  }
  info.publisher_count = node_->count_publishers(topic_name);
  info.subscriber_count = node_->count_subscribers(topic_name);

  return info;
}

bool NativeTopicSampler::has_publishers(const std::string & topic_name) {
  return node_->count_publishers(topic_name) > 0;
}

std::string NativeTopicSampler::get_topic_type(const std::string & topic_name) {
  auto topic_names_and_types = node_->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topic_name);
  if (it != topic_names_and_types.end() && !it->second.empty()) {
    return it->second[0];
  }
  return "";
}

json NativeTopicSampler::parse_message_yaml(const std::string & yaml_str) {
  OutputParser parser;
  return parser.parse_yaml(yaml_str);
}

TopicSampleResult NativeTopicSampler::sample_topic(const std::string & topic_name, double timeout_sec) {
  TopicSampleResult result;
  result.topic_name = topic_name;
  result.timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  // Get topic info (type, pub/sub counts) - this is always fast
  auto info = get_topic_info(topic_name);
  if (!info) {
    RCLCPP_WARN(node_->get_logger(), "Topic '%s' not found in graph", topic_name.c_str());
    result.has_data = false;
    return result;
  }

  result.message_type = info->type;
  result.publisher_count = info->publisher_count;
  result.subscriber_count = info->subscriber_count;

  // Fast path: if no publishers, return metadata immediately
  // This is the key UX improvement - no waiting for idle topics!
  if (info->publisher_count == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "Topic '%s' has no publishers, returning metadata only", topic_name.c_str());
    result.has_data = false;
    return result;
  }

  // Create a generic subscription to receive one message
  // We use serialized messages + ros2 topic echo approach via CLI for now,
  // as GenericSubscription requires type support which needs runtime loading.
  // TODO(mfaferek93): Implement true generic subscription with rosidl_typesupport_introspection_cpp

  // For now, use a hybrid approach:
  // 1. Fast metadata via native API (already done above)
  // 2. Use ros2 topic echo with very short timeout only for active topics

  // We fall back to CLI sampling but with a much shorter timeout since we know
  // there are publishers. This still provides significant UX improvement:
  // - Topics without publishers return immediately (no CLI call)
  // - Topics with publishers use CLI but we've eliminated ~50-90% of CLI calls

  try {
    // Use ros2 topic echo with the standard approach but shorter timeout
    // Note: timeout_sec is rounded up to integer seconds (minimum 1s) for GNU timeout command
    std::ostringstream cmd;
    int timeout_int = std::max(1, static_cast<int>(std::ceil(timeout_sec)));
    // Use escape_shell_arg to prevent command injection
    cmd << "timeout " << timeout_int << "s ros2 topic echo " << ROS2CLIWrapper::escape_shell_arg(topic_name)
        << " --once --no-arr 2>/dev/null";

    // Execute command with RAII pipe management to prevent resource leaks
    auto pipe_closer = [](FILE * f) {
      if (f) {
        pclose(f);
      }
    };
    std::unique_ptr<FILE, decltype(pipe_closer)> pipe(popen(cmd.str().c_str(), "r"), pipe_closer);

    if (!pipe) {
      RCLCPP_WARN(node_->get_logger(), "Failed to execute CLI command for topic '%s'", topic_name.c_str());
      result.has_data = false;
      return result;
    }

    // Use larger buffer for better performance with large messages
    std::array<char, 4096> buffer;
    std::string output;
    output.reserve(4096);  // Pre-allocate for typical message sizes

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
      output += buffer.data();
    }

    // Get exit code - release pipe to get exit code before RAII cleanup
    int exit_code = pclose(pipe.release());

    // Check for success (exit code 0 and non-empty output without warnings)
    if (exit_code == 0 && !output.empty() && output.find("WARNING") == std::string::npos) {
      result.data = parse_message_yaml(output);
      // Check if result.data has a value AND is not null
      if (result.data && !result.data->is_null()) {
        result.has_data = true;
        RCLCPP_DEBUG(node_->get_logger(), "Sampled data from topic '%s'", topic_name.c_str());
      }
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "No data received from topic '%s' (timeout or error)", topic_name.c_str());
      result.has_data = false;
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(node_->get_logger(), "Exception sampling topic '%s': %s", topic_name.c_str(), e.what());
    result.has_data = false;
  }

  return result;
}

std::vector<TopicSampleResult> NativeTopicSampler::sample_topics_parallel(const std::vector<std::string> & topic_names,
                                                                          double timeout_sec, int max_parallel) {
  std::vector<TopicSampleResult> results;
  results.reserve(topic_names.size());

  // Query graph once for all topics (optimization suggested by mfaferek93)
  auto all_topic_info = node_->get_topic_names_and_types();

  // Separate topics into those with publishers (need sampling) and without (immediate return)
  std::vector<std::string> active_topics;
  std::vector<TopicSampleResult> immediate_results;

  for (const auto & topic_name : topic_names) {
    auto it = all_topic_info.find(topic_name);
    if (it == all_topic_info.end()) {
      // Topic not found
      TopicSampleResult res;
      res.topic_name = topic_name;
      res.has_data = false;
      res.timestamp_ns =
          std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
              .count();
      immediate_results.push_back(res);
    } else {
      // Get type from cached info
      std::string msg_type = it->second.empty() ? "" : it->second[0];
      size_t pub_count = node_->count_publishers(topic_name);
      size_t sub_count = node_->count_subscribers(topic_name);

      if (pub_count == 0) {
        // No publishers - return metadata immediately
        TopicSampleResult res;
        res.topic_name = topic_name;
        res.message_type = msg_type;
        res.publisher_count = pub_count;
        res.subscriber_count = sub_count;
        res.has_data = false;
        res.timestamp_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
                .count();
        immediate_results.push_back(res);
      } else {
        // Has publishers - need to sample
        active_topics.push_back(topic_name);
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "Parallel sampling: %zu topics total, %zu with publishers (sampling), %zu without (immediate)",
              topic_names.size(), active_topics.size(), immediate_results.size());

  // Process active topics in parallel batches
  for (size_t i = 0; i < active_topics.size(); i += static_cast<size_t>(max_parallel)) {
    size_t batch_size = std::min(static_cast<size_t>(max_parallel), active_topics.size() - i);

    std::vector<std::future<TopicSampleResult>> futures;
    futures.reserve(batch_size);

    for (size_t j = 0; j < batch_size; ++j) {
      const auto & topic = active_topics[i + j];
      futures.push_back(std::async(std::launch::async, [this, topic, timeout_sec]() -> TopicSampleResult {
        return sample_topic(topic, timeout_sec);
      }));
    }

    for (auto & future : futures) {
      try {
        results.push_back(future.get());
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_->get_logger(), "Exception in parallel sampling: %s", e.what());
      }
    }
  }

  // Combine immediate results with sampled results
  results.insert(results.end(), immediate_results.begin(), immediate_results.end());

  return results;
}

}  // namespace ros2_medkit_gateway
