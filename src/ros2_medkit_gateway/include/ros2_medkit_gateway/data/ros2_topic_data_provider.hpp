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

#include <atomic>
#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "ros2_medkit_gateway/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief ROS 2 default implementation of TopicDataProvider.
 *
 * Provides topic discovery (via direct graph queries on the subscription node)
 * and topic sampling (via subscriptions managed by Ros2SubscriptionExecutor).
 *
 * @par Current scope (this revision)
 * - All TopicDataProvider discovery methods are live (graph queries).
 * - `sample()` and `sample_parallel()` currently return metadata-only results
 *   with `has_data == false`. The subscription pool implementation lands in a
 *   follow-up commit. This lets the interface be wired up to the callers
 *   (DataAccessManager et al.) before the pool is ready, without regressing
 *   the legacy NativeTopicSampler path that still handles production sampling.
 *
 * @par Thread safety
 * All methods may be called concurrently. Discovery methods call back into
 * rclcpp graph APIs which are documented thread-safe for reads. Future pool
 * state will be protected by an internal mutex.
 */
class Ros2TopicDataProvider final : public TopicDataProvider {
 public:
  struct Config {
    std::size_t max_pool_size;
    std::chrono::minutes idle_safety_net;
    std::size_t metadata_cache_cap;
    std::size_t cold_wait_cap;

    // Explicit ctor: GCC nested-default-member-init limitation (same as
    // Ros2SubscriptionExecutor::Config).
    Config() : max_pool_size{256}, idle_safety_net{15}, metadata_cache_cap{512}, cold_wait_cap{4} {
    }
  };

  struct PoolStats {
    std::size_t pool_size;
    std::size_t pool_cap;
    std::size_t pool_hits;
    std::size_t pool_misses;
    std::size_t metadata_cache_size;
    std::size_t metadata_cache_cap;
    std::size_t evictions_total;
    std::size_t type_change_events;
    std::size_t graph_events_received;
    std::size_t concurrent_cold_waits;
    std::size_t cold_wait_cap;
  };

  Ros2TopicDataProvider(std::shared_ptr<ros2_common::Ros2SubscriptionExecutor> exec,
                        std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer, Config cfg = Config());
  ~Ros2TopicDataProvider() override;

  // ---- TopicDataProvider overrides ----

  tl::expected<TopicSampleResult, ErrorInfo> sample(const std::string & topic,
                                                    std::chrono::milliseconds timeout) override;

  tl::expected<std::vector<TopicSampleResult>, ErrorInfo> sample_parallel(const std::vector<std::string> & topics,
                                                                          std::chrono::milliseconds timeout) override;

  std::optional<TopicInfo> get_topic_info(const std::string & topic) override;
  bool has_publishers(const std::string & topic) override;
  std::vector<TopicInfo> discover(const std::string & namespace_prefix) override;
  std::vector<TopicInfo> discover_all() override;

  std::map<std::string, ComponentTopics> build_component_topic_map() override;
  ComponentTopics get_component_topics(const std::string & component_fqn) override;

  TopicDiscoveryResult discover_topics_by_namespace() override;
  std::set<std::string> discover_topic_namespaces() override;
  ComponentTopics get_topics_for_namespace(const std::string & ns_prefix) override;

  std::vector<TopicEndpoint> get_topic_publishers(const std::string & topic) override;
  std::vector<TopicEndpoint> get_topic_subscribers(const std::string & topic) override;
  TopicConnection get_topic_connection(const std::string & topic) override;

  // ---- Observability ----

  [[nodiscard]] PoolStats stats() const;

  /// Identify whether a topic is a ROS 2 system/infrastructure topic (filter target).
  [[nodiscard]] static bool is_system_topic(const std::string & topic_name);

 private:
  TopicSampleResult build_metadata_only_sample(const std::string & topic);

  Config cfg_;
  std::shared_ptr<ros2_common::Ros2SubscriptionExecutor> exec_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;

  std::atomic<bool> shutdown_{false};

  // Counters surfaced through stats() (pool itself added in follow-up commit).
  mutable std::mutex cache_mtx_;
  std::unordered_map<std::string, TopicInfo> component_topic_map_cache_;  ///< Populated by build_component_topic_map

  std::atomic<std::size_t> pool_hits_{0};
  std::atomic<std::size_t> pool_misses_{0};
  std::atomic<std::size_t> evictions_total_{0};
  std::atomic<std::size_t> type_change_events_{0};
  std::atomic<std::size_t> graph_events_received_{0};
  std::atomic<std::size_t> concurrent_cold_waits_{0};
};

}  // namespace ros2_medkit_gateway
