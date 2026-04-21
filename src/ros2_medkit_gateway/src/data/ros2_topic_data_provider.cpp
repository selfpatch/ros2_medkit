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

#include "ros2_medkit_gateway/data/ros2_topic_data_provider.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/version.h>

namespace ros2_medkit_gateway {

namespace {

std::string reliability_to_string(rclcpp::ReliabilityPolicy policy) {
  switch (policy) {
    case rclcpp::ReliabilityPolicy::Reliable:
      return "reliable";
    case rclcpp::ReliabilityPolicy::BestEffort:
      return "best_effort";
    case rclcpp::ReliabilityPolicy::SystemDefault:
      return "system_default";
#if RCLCPP_VERSION_MAJOR >= 21
    case rclcpp::ReliabilityPolicy::BestAvailable:
      return "best_available";
#endif
    case rclcpp::ReliabilityPolicy::Unknown:
    default:
      return "unknown";
  }
}

std::string durability_to_string(rclcpp::DurabilityPolicy policy) {
  switch (policy) {
    case rclcpp::DurabilityPolicy::Volatile:
      return "volatile";
    case rclcpp::DurabilityPolicy::TransientLocal:
      return "transient_local";
    case rclcpp::DurabilityPolicy::SystemDefault:
      return "system_default";
#if RCLCPP_VERSION_MAJOR >= 21
    case rclcpp::DurabilityPolicy::BestAvailable:
      return "best_available";
#endif
    case rclcpp::DurabilityPolicy::Unknown:
    default:
      return "unknown";
  }
}

std::string history_to_string(rclcpp::HistoryPolicy policy) {
  switch (policy) {
    case rclcpp::HistoryPolicy::KeepLast:
      return "keep_last";
    case rclcpp::HistoryPolicy::KeepAll:
      return "keep_all";
    case rclcpp::HistoryPolicy::SystemDefault:
      return "system_default";
    case rclcpp::HistoryPolicy::Unknown:
    default:
      return "unknown";
  }
}

std::string liveliness_to_string(rclcpp::LivelinessPolicy policy) {
  switch (policy) {
    case rclcpp::LivelinessPolicy::Automatic:
      return "automatic";
    case rclcpp::LivelinessPolicy::ManualByTopic:
      return "manual_by_topic";
    case rclcpp::LivelinessPolicy::SystemDefault:
      return "system_default";
#if RCLCPP_VERSION_MAJOR >= 21
    case rclcpp::LivelinessPolicy::BestAvailable:
      return "best_available";
#endif
    case rclcpp::LivelinessPolicy::Unknown:
    default:
      return "unknown";
  }
}

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

std::int64_t now_ns_epoch() noexcept {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

}  // namespace

Ros2TopicDataProvider::Ros2TopicDataProvider(std::shared_ptr<ros2_common::Ros2SubscriptionExecutor> exec,
                                             std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer,
                                             Config cfg)
  : cfg_{cfg}, exec_{std::move(exec)}, serializer_{std::move(serializer)} {
  if (!exec_) {
    throw std::invalid_argument{"Ros2TopicDataProvider requires a valid Ros2SubscriptionExecutor"};
  }
  if (!exec_->node()) {
    throw std::invalid_argument{"Ros2SubscriptionExecutor has no subscription node"};
  }
  // Serializer is optional for discovery-only use; sampling path will check.
}

Ros2TopicDataProvider::~Ros2TopicDataProvider() {
  shutdown_.store(true, std::memory_order_release);
}

// ---- Sampling (metadata-only placeholder) -----------------------------------

tl::expected<TopicSampleResult, ErrorInfo> Ros2TopicDataProvider::sample(const std::string & topic,
                                                                         std::chrono::milliseconds /*timeout*/) {
  if (shutdown_.load(std::memory_order_acquire)) {
    return tl::unexpected(ErrorInfo{"ERR_GATEWAY_SHUTDOWN", "gateway shutting down", 503, nlohmann::json::object()});
  }
  // Pool-based sampling lands in a follow-up commit. For now, return
  // metadata-only so callers can be wired up without behavioral regression.
  return build_metadata_only_sample(topic);
}

tl::expected<std::vector<TopicSampleResult>, ErrorInfo>
Ros2TopicDataProvider::sample_parallel(const std::vector<std::string> & topics, std::chrono::milliseconds /*timeout*/) {
  if (shutdown_.load(std::memory_order_acquire)) {
    return tl::unexpected(ErrorInfo{"ERR_GATEWAY_SHUTDOWN", "gateway shutting down", 503, nlohmann::json::object()});
  }
  std::vector<TopicSampleResult> results;
  results.reserve(topics.size());
  for (const auto & t : topics) {
    results.push_back(build_metadata_only_sample(t));
  }
  return results;
}

TopicSampleResult Ros2TopicDataProvider::build_metadata_only_sample(const std::string & topic) {
  TopicSampleResult r;
  r.topic_name = topic;
  r.timestamp_ns = now_ns_epoch();
  r.has_data = false;
  if (auto info = get_topic_info(topic)) {
    r.message_type = info->type;
    r.publisher_count = info->publisher_count;
    r.subscriber_count = info->subscriber_count;
    r.publishers = get_topic_publishers(topic);
    r.subscribers = get_topic_subscribers(topic);
  }
  return r;
}

// ---- Discovery --------------------------------------------------------------

std::optional<TopicInfo> Ros2TopicDataProvider::get_topic_info(const std::string & topic) {
  auto topic_names_and_types = exec_->node()->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topic);
  if (it == topic_names_and_types.end()) {
    return std::nullopt;
  }
  TopicInfo info;
  info.name = topic;
  if (!it->second.empty()) {
    info.type = it->second[0];
  }
  info.publisher_count = exec_->node()->count_publishers(topic);
  info.subscriber_count = exec_->node()->count_subscribers(topic);
  return info;
}

bool Ros2TopicDataProvider::has_publishers(const std::string & topic) {
  return exec_->node()->count_publishers(topic) > 0;
}

std::vector<TopicInfo> Ros2TopicDataProvider::discover_all() {
  std::vector<TopicInfo> result;
  auto topic_names_and_types = exec_->node()->get_topic_names_and_types();
  result.reserve(topic_names_and_types.size());
  for (const auto & [topic_name, types] : topic_names_and_types) {
    TopicInfo info;
    info.name = topic_name;
    if (!types.empty()) {
      info.type = types[0];
    }
    info.publisher_count = exec_->node()->count_publishers(topic_name);
    info.subscriber_count = exec_->node()->count_subscribers(topic_name);
    result.push_back(info);
  }
  return result;
}

std::vector<TopicInfo> Ros2TopicDataProvider::discover(const std::string & namespace_prefix) {
  auto all = discover_all();
  std::vector<TopicInfo> filtered;
  std::copy_if(all.begin(), all.end(), std::back_inserter(filtered), [&namespace_prefix](const TopicInfo & topic) {
    if (topic.name.find(namespace_prefix) != 0) {
      return false;
    }
    return topic.name.length() == namespace_prefix.length() || topic.name[namespace_prefix.length()] == '/';
  });
  return filtered;
}

std::vector<TopicEndpoint> Ros2TopicDataProvider::get_topic_publishers(const std::string & topic) {
  std::vector<TopicEndpoint> endpoints;
  auto publishers_info = exec_->node()->get_publishers_info_by_topic(topic);
  endpoints.reserve(publishers_info.size());
  for (const auto & pub_info : publishers_info) {
    TopicEndpoint endpoint;
    endpoint.node_name = pub_info.node_name();
    endpoint.node_namespace = pub_info.node_namespace();
    endpoint.topic_type = pub_info.topic_type();
    endpoint.qos = qos_to_profile(pub_info.qos_profile());
    endpoints.push_back(endpoint);
  }
  return endpoints;
}

std::vector<TopicEndpoint> Ros2TopicDataProvider::get_topic_subscribers(const std::string & topic) {
  std::vector<TopicEndpoint> endpoints;
  auto subscribers_info = exec_->node()->get_subscriptions_info_by_topic(topic);
  endpoints.reserve(subscribers_info.size());
  for (const auto & sub_info : subscribers_info) {
    TopicEndpoint endpoint;
    endpoint.node_name = sub_info.node_name();
    endpoint.node_namespace = sub_info.node_namespace();
    endpoint.topic_type = sub_info.topic_type();
    endpoint.qos = qos_to_profile(sub_info.qos_profile());
    endpoints.push_back(endpoint);
  }
  return endpoints;
}

TopicConnection Ros2TopicDataProvider::get_topic_connection(const std::string & topic) {
  TopicConnection conn;
  conn.topic_name = topic;
  auto topic_names_and_types = exec_->node()->get_topic_names_and_types();
  auto it = topic_names_and_types.find(topic);
  if (it != topic_names_and_types.end() && !it->second.empty()) {
    conn.topic_type = it->second[0];
  }
  conn.publishers = get_topic_publishers(topic);
  conn.subscribers = get_topic_subscribers(topic);
  return conn;
}

std::map<std::string, ComponentTopics> Ros2TopicDataProvider::build_component_topic_map() {
  std::map<std::string, ComponentTopics> component_map;
  auto all_topics = exec_->node()->get_topic_names_and_types();
  for (const auto & [topic_name, types] : all_topics) {
    (void)types;
    const auto build_fqn = [](const std::string & ns, const std::string & name) {
      std::string fqn;
      if (ns == "/" || ns.empty()) {
        fqn.reserve(1 + name.size());
        fqn += '/';
        fqn += name;
      } else {
        fqn.reserve(ns.size() + 1 + name.size());
        fqn += ns;
        fqn += '/';
        fqn += name;
      }
      return fqn;
    };
    for (const auto & pub_info : exec_->node()->get_publishers_info_by_topic(topic_name)) {
      component_map[build_fqn(pub_info.node_namespace(), pub_info.node_name())].publishes.push_back(topic_name);
    }
    for (const auto & sub_info : exec_->node()->get_subscriptions_info_by_topic(topic_name)) {
      component_map[build_fqn(sub_info.node_namespace(), sub_info.node_name())].subscribes.push_back(topic_name);
    }
  }
  return component_map;
}

ComponentTopics Ros2TopicDataProvider::get_component_topics(const std::string & component_fqn) {
  auto map = build_component_topic_map();
  auto it = map.find(component_fqn);
  if (it != map.end()) {
    return it->second;
  }
  return ComponentTopics{};
}

bool Ros2TopicDataProvider::is_system_topic(const std::string & topic_name) {
  static const std::vector<std::string> system_topics = {"/parameter_events", "/rosout", "/clock"};
  return std::find(system_topics.begin(), system_topics.end(), topic_name) != system_topics.end();
}

TopicDiscoveryResult Ros2TopicDataProvider::discover_topics_by_namespace() {
  TopicDiscoveryResult result;
  auto all_topics = exec_->node()->get_topic_names_and_types();
  for (const auto & [topic_name, types] : all_topics) {
    (void)types;
    if (is_system_topic(topic_name)) {
      continue;
    }
    if (topic_name.length() > 1 && topic_name[0] == '/') {
      const std::size_t second_slash = topic_name.find('/', 1);
      if (second_slash != std::string::npos) {
        const std::string ns = topic_name.substr(1, second_slash - 1);
        if (!ns.empty()) {
          result.namespaces.insert(ns);
          std::string ns_prefix;
          ns_prefix.reserve(1 + ns.size());
          ns_prefix += '/';
          ns_prefix += ns;
          result.topics_by_ns[ns_prefix].publishes.push_back(topic_name);
        }
      }
    }
  }
  return result;
}

std::set<std::string> Ros2TopicDataProvider::discover_topic_namespaces() {
  return discover_topics_by_namespace().namespaces;
}

ComponentTopics Ros2TopicDataProvider::get_topics_for_namespace(const std::string & ns_prefix) {
  ComponentTopics topics;
  auto all_topics = exec_->node()->get_topic_names_and_types();
  for (const auto & [topic_name, types] : all_topics) {
    (void)types;
    if (is_system_topic(topic_name)) {
      continue;
    }
    if (topic_name.length() > ns_prefix.length() && topic_name.find(ns_prefix) == 0 &&
        topic_name[ns_prefix.length()] == '/') {
      topics.publishes.push_back(topic_name);
    }
  }
  return topics;
}

// ---- Observability ----------------------------------------------------------

Ros2TopicDataProvider::PoolStats Ros2TopicDataProvider::stats() const {
  PoolStats s{};
  s.pool_size = 0;  // pool not yet implemented
  s.pool_cap = cfg_.max_pool_size;
  s.pool_hits = pool_hits_.load();
  s.pool_misses = pool_misses_.load();
  s.metadata_cache_size = 0;
  s.metadata_cache_cap = cfg_.metadata_cache_cap;
  s.evictions_total = evictions_total_.load();
  s.type_change_events = type_change_events_.load();
  s.graph_events_received = graph_events_received_.load();
  s.concurrent_cold_waits = concurrent_cold_waits_.load();
  s.cold_wait_cap = cfg_.cold_wait_cap;
  return s;
}

}  // namespace ros2_medkit_gateway
