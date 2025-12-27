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
#include <iomanip>
#include <mutex>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sstream>
#include <thread>

#include "ros2_medkit_gateway/output_parser.hpp"
#include "ros2_medkit_gateway/ros2_cli_wrapper.hpp"

namespace ros2_medkit_gateway {

namespace {

/// Convert rclcpp ReliabilityPolicy to string
/// Note: BestAvailable policy requires ROS 2 Humble or newer
std::string reliability_to_string(rclcpp::ReliabilityPolicy policy) {
  switch (policy) {
    case rclcpp::ReliabilityPolicy::Reliable:
      return "reliable";
    case rclcpp::ReliabilityPolicy::BestEffort:
      return "best_effort";
    case rclcpp::ReliabilityPolicy::SystemDefault:
      return "system_default";
    case rclcpp::ReliabilityPolicy::BestAvailable:
      return "best_available";
    default:
      return "unknown";
  }
}

/// Convert rclcpp DurabilityPolicy to string
/// Note: BestAvailable policy requires ROS 2 Humble or newer
std::string durability_to_string(rclcpp::DurabilityPolicy policy) {
  switch (policy) {
    case rclcpp::DurabilityPolicy::Volatile:
      return "volatile";
    case rclcpp::DurabilityPolicy::TransientLocal:
      return "transient_local";
    case rclcpp::DurabilityPolicy::SystemDefault:
      return "system_default";
    case rclcpp::DurabilityPolicy::BestAvailable:
      return "best_available";
    default:
      return "unknown";
  }
}

/// Convert rclcpp HistoryPolicy to string
std::string history_to_string(rclcpp::HistoryPolicy policy) {
  switch (policy) {
    case rclcpp::HistoryPolicy::KeepLast:
      return "keep_last";
    case rclcpp::HistoryPolicy::KeepAll:
      return "keep_all";
    case rclcpp::HistoryPolicy::SystemDefault:
      return "system_default";
    default:
      return "unknown";
  }
}

/// Convert rclcpp LivelinessPolicy to string
/// Note: BestAvailable policy requires ROS 2 Humble or newer
std::string liveliness_to_string(rclcpp::LivelinessPolicy policy) {
  switch (policy) {
    case rclcpp::LivelinessPolicy::Automatic:
      return "automatic";
    case rclcpp::LivelinessPolicy::ManualByTopic:
      return "manual_by_topic";
    case rclcpp::LivelinessPolicy::SystemDefault:
      return "system_default";
    case rclcpp::LivelinessPolicy::BestAvailable:
      return "best_available";
    default:
      return "unknown";
  }
}

/// Convert rclcpp::QoS to our QosProfile struct
QosProfile qos_to_profile(const rclcpp::QoS & qos) {
  QosProfile profile;
  const auto & rmw_qos = qos.get_rmw_qos_profile();

  profile.reliability = reliability_to_string(qos.reliability());
  profile.durability = durability_to_string(qos.durability());
  profile.history = history_to_string(qos.history());
  profile.depth = rmw_qos.depth;
  profile.liveliness = liveliness_to_string(qos.liveliness());

  return profile;
}

}  // namespace

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
  RCLCPP_DEBUG(node_->get_logger(), "sample_topic: START topic='%s', timeout=%.2f", topic_name.c_str(), timeout_sec);
  TopicSampleResult result;
  result.topic_name = topic_name;
  result.timestamp_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

  // Get topic info (type, pub/sub counts) - this is always fast
  auto info = get_topic_info(topic_name);
  if (!info) {
    RCLCPP_DEBUG(node_->get_logger(), "sample_topic: Topic '%s' not found in graph", topic_name.c_str());
    result.has_data = false;
    return result;
  }

  RCLCPP_DEBUG(node_->get_logger(), "sample_topic: topic='%s' type='%s' pubs=%zu subs=%zu", topic_name.c_str(),
               info->type.c_str(), info->publisher_count, info->subscriber_count);

  result.message_type = info->type;
  result.publisher_count = info->publisher_count;
  result.subscriber_count = info->subscriber_count;

  // Get detailed endpoint info with QoS
  result.publishers = get_topic_publishers(topic_name);
  result.subscribers = get_topic_subscribers(topic_name);

  // Fast path: if no publishers, return metadata immediately
  // This is the key UX improvement - no waiting for idle topics!
  if (info->publisher_count == 0) {
    RCLCPP_DEBUG(node_->get_logger(), "sample_topic: Topic '%s' has no publishers, returning metadata only",
                 topic_name.c_str());
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
    // Use ros2 topic echo --once with native --timeout flag
    // timeout_sec is passed from caller (default 1s for single topic, configurable)
    constexpr double kDefaultTimeoutSec = 1.0;
    if (timeout_sec <= 0) {
      RCLCPP_WARN(node_->get_logger(),
                  "sample_topic: Invalid timeout_sec (%.3f) for topic '%s'. Using default (%.1fs).", timeout_sec,
                  topic_name.c_str(), kDefaultTimeoutSec);
      timeout_sec = kDefaultTimeoutSec;
    }
    int timeout_int = static_cast<int>(std::ceil(timeout_sec));
    std::ostringstream cmd;
    cmd << "ros2 topic echo " << ROS2CLIWrapper::escape_shell_arg(topic_name) << " --once --no-arr --timeout "
        << timeout_int << " 2>/dev/null";

    RCLCPP_DEBUG(node_->get_logger(), "sample_topic: executing CLI: %s", cmd.str().c_str());

    // Execute command with RAII pipe management to prevent resource leaks
    auto pipe_closer = [](FILE * f) {
      if (f) {
        pclose(f);
      }
    };
    std::unique_ptr<FILE, decltype(pipe_closer)> pipe(popen(cmd.str().c_str(), "r"), pipe_closer);

    if (!pipe) {
      RCLCPP_WARN(node_->get_logger(), "sample_topic: Failed to execute CLI command for topic '%s'",
                  topic_name.c_str());
      result.has_data = false;
      return result;
    }

    RCLCPP_DEBUG(node_->get_logger(), "sample_topic: CLI started, reading output...");

    // Use larger buffer for better performance with large messages
    std::array<char, 4096> buffer;
    std::string output;
    output.reserve(4096);  // Pre-allocate for typical message sizes

    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
      output += buffer.data();
    }

    // Get exit code - release pipe to get exit code before RAII cleanup
    int exit_code = pclose(pipe.release());

    RCLCPP_DEBUG(node_->get_logger(), "sample_topic: CLI finished, exit_code=%d, output_len=%zu", exit_code,
                 output.length());

    // Check for success (exit code 0 and non-empty output without warnings)
    if (exit_code == 0 && !output.empty() && output.find("WARNING") == std::string::npos) {
      result.data = parse_message_yaml(output);
      // Check if result.data has a value AND is not null
      if (result.data && !result.data->is_null()) {
        result.has_data = true;
        RCLCPP_DEBUG(node_->get_logger(), "sample_topic: Sampled data from topic '%s'", topic_name.c_str());
      }
    } else {
      RCLCPP_DEBUG(node_->get_logger(), "sample_topic: No data received from topic '%s' (exit_code=%d)",
                   topic_name.c_str(), exit_code);
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

std::vector<TopicEndpoint> NativeTopicSampler::get_topic_publishers(const std::string & topic_name) {
  std::vector<TopicEndpoint> endpoints;

  auto publishers_info = node_->get_publishers_info_by_topic(topic_name);
  for (const auto & pub_info : publishers_info) {
    TopicEndpoint endpoint;
    endpoint.node_name = pub_info.node_name();
    endpoint.node_namespace = pub_info.node_namespace();
    endpoint.topic_type = pub_info.topic_type();
    endpoint.qos = qos_to_profile(pub_info.qos_profile());
    endpoints.push_back(endpoint);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Topic '%s' has %zu publishers", topic_name.c_str(), endpoints.size());
  return endpoints;
}

std::vector<TopicEndpoint> NativeTopicSampler::get_topic_subscribers(const std::string & topic_name) {
  std::vector<TopicEndpoint> endpoints;

  auto subscribers_info = node_->get_subscriptions_info_by_topic(topic_name);
  for (const auto & sub_info : subscribers_info) {
    TopicEndpoint endpoint;
    endpoint.node_name = sub_info.node_name();
    endpoint.node_namespace = sub_info.node_namespace();
    endpoint.topic_type = sub_info.topic_type();
    endpoint.qos = qos_to_profile(sub_info.qos_profile());
    endpoints.push_back(endpoint);
  }

  RCLCPP_DEBUG(node_->get_logger(), "Topic '%s' has %zu subscribers", topic_name.c_str(), endpoints.size());
  return endpoints;
}

TopicConnection NativeTopicSampler::get_topic_connection(const std::string & topic_name) {
  TopicConnection conn;
  conn.topic_name = topic_name;
  conn.topic_type = get_topic_type(topic_name);
  conn.publishers = get_topic_publishers(topic_name);
  conn.subscribers = get_topic_subscribers(topic_name);
  return conn;
}

std::map<std::string, ComponentTopics> NativeTopicSampler::build_component_topic_map() {
  std::map<std::string, ComponentTopics> component_map;

  // Get all topics
  auto all_topics = node_->get_topic_names_and_types();

  for (const auto & [topic_name, types] : all_topics) {
    // Get publishers info (lightweight - just node names, no QoS)
    auto publishers_info = node_->get_publishers_info_by_topic(topic_name);
    for (const auto & pub_info : publishers_info) {
      std::string ns = pub_info.node_namespace();
      std::string name = pub_info.node_name();
      std::string component_fqn;
      if (ns == "/" || ns.empty()) {
        component_fqn = "/" + name;
      } else {
        component_fqn = ns;
        component_fqn += "/";
        component_fqn += name;
      }
      component_map[component_fqn].publishes.push_back(topic_name);
    }

    // Get subscribers info (lightweight - just node names, no QoS)
    auto subscribers_info = node_->get_subscriptions_info_by_topic(topic_name);
    for (const auto & sub_info : subscribers_info) {
      std::string ns = sub_info.node_namespace();
      std::string name = sub_info.node_name();
      std::string component_fqn;
      if (ns == "/" || ns.empty()) {
        component_fqn = "/" + name;
      } else {
        component_fqn = ns;
        component_fqn += "/";
        component_fqn += name;
      }
      component_map[component_fqn].subscribes.push_back(topic_name);
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "Built topic map for %zu components", component_map.size());
  return component_map;
}

ComponentTopics NativeTopicSampler::get_component_topics(const std::string & component_fqn) {
  // Build full map and extract for this component
  // TODO(optimization): Cache the map and invalidate on graph changes
  auto full_map = build_component_topic_map();

  auto it = full_map.find(component_fqn);
  if (it != full_map.end()) {
    return it->second;
  }

  // Component not found or has no topics
  return ComponentTopics{};
}

bool NativeTopicSampler::is_system_topic(const std::string & topic_name) {
  // System topics to filter out during topic-based discovery
  // Note: /tf and /tf_static are NOT filtered (useful for diagnostics)
  static const std::vector<std::string> system_topics = {"/parameter_events", "/rosout", "/clock"};

  return std::find(system_topics.begin(), system_topics.end(), topic_name) != system_topics.end();
}

NativeTopicSampler::TopicDiscoveryResult NativeTopicSampler::discover_topics_by_namespace() {
  TopicDiscoveryResult result;

  // Single graph query - avoids N+1 problem
  auto all_topics = node_->get_topic_names_and_types();

  for (const auto & [topic_name, types] : all_topics) {
    // Skip system topics
    if (is_system_topic(topic_name)) {
      continue;
    }

    // Extract first segment from topic path
    // "/carter1/odom" -> namespace "carter1", ns_prefix "/carter1"
    if (topic_name.length() > 1 && topic_name[0] == '/') {
      size_t second_slash = topic_name.find('/', 1);
      if (second_slash != std::string::npos) {
        std::string ns = topic_name.substr(1, second_slash - 1);
        if (!ns.empty()) {
          // Add namespace to set
          result.namespaces.insert(ns);

          // Add topic to the namespace's topic list
          std::string ns_prefix = "/" + ns;
          result.topics_by_ns[ns_prefix].publishes.push_back(topic_name);
        }
      }
      // else: root topic like "/tf", skip (no namespace)
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "Discovered %zu topic namespaces with topics in single query",
               result.namespaces.size());
  return result;
}

std::set<std::string> NativeTopicSampler::discover_topic_namespaces() {
  // Delegate to optimized method (for backward compatibility)
  return discover_topics_by_namespace().namespaces;
}

ComponentTopics NativeTopicSampler::get_topics_for_namespace(const std::string & ns_prefix) {
  ComponentTopics topics;

  auto all_topics = node_->get_topic_names_and_types();

  for (const auto & [topic_name, types] : all_topics) {
    // Skip system topics
    if (is_system_topic(topic_name)) {
      continue;
    }

    // Check if topic starts with namespace prefix followed by '/'
    // ns_prefix is like "/carter1", topic is like "/carter1/odom"
    if (topic_name.length() > ns_prefix.length() && topic_name.find(ns_prefix) == 0 &&
        topic_name[ns_prefix.length()] == '/') {
      // For topic-based discovery, we put all topics in 'publishes'
      // since we can't determine direction without node info
      topics.publishes.push_back(topic_name);
    }
  }

  RCLCPP_DEBUG(node_->get_logger(), "Found %zu topics for namespace '%s'", topics.publishes.size(), ns_prefix.c_str());
  return topics;
}

}  // namespace ros2_medkit_gateway
