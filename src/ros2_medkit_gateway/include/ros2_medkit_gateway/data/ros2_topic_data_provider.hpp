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
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <rclcpp/serialized_message.hpp>

#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_slot.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief ROS 2 default implementation of TopicDataProvider.
 *
 * Provides topic discovery (via direct graph queries on the subscription node)
 * and topic sampling via a pool of long-lived subscriptions managed by
 * Ros2SubscriptionExecutor. Each pool entry keeps the always-newest message,
 * so concurrent samplers on the same topic share the latest without creating
 * additional subscriptions.
 *
 * @par Race fix (issue #375)
 * Subscription creation / destruction is serialized by the executor. The pool
 * amortizes cost across samples so hot topics reuse the same subscription.
 *
 * @par Thread safety
 * All methods may be called concurrently. Pool state is protected by
 * `pool_mtx_`; per-entry buffers by `PoolEntry::buf_mtx`.
 *
 * @par Eviction
 * - Graph change: registered callback walks the pool and drops entries whose
 *   topic has disappeared from the graph.
 * - Pool cap: on miss, if pool is at `max_pool_size`, evicts the
 *   least-recently-sampled entry and installs the new one (LRU).
 * - Shutdown: destructor signals every per-entry CV, resets all slots.
 */
class Ros2TopicDataProvider final : public TopicDataProvider {
 public:
  struct Config {
    std::size_t max_pool_size;
    std::chrono::milliseconds idle_safety_net;
    std::chrono::milliseconds idle_sweep_tick;
    std::size_t cold_wait_cap;
    /// Upper bound on concurrent sampler threads launched by sample_parallel().
    /// Prevents the provider from becoming a self-DoS vector under large topic
    /// lists: excess topics are processed in follow-up chunks instead of
    /// spawning one std::async per topic.
    std::size_t max_parallel_samples;

    // Explicit ctor: GCC nested-default-member-init limitation (same as
    // Ros2SubscriptionExecutor::Config).
    Config()
      : max_pool_size{256}
      , idle_safety_net{std::chrono::minutes{15}}
      , idle_sweep_tick{std::chrono::minutes{1}}
      , cold_wait_cap{4}
      , max_parallel_samples{8} {
    }
  };

  struct PoolStats {
    std::size_t pool_size;
    std::size_t pool_cap;
    std::size_t pool_hits;
    std::size_t pool_misses;
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
  [[nodiscard]] nlohmann::json x_medkit_stats() const override;

  /**
   * @brief Walk the pool and evict entries idle longer than cfg.idle_safety_net.
   *
   * Normally driven by an internal wall timer on the subscription node. Exposed
   * publicly so tests can trigger a deterministic sweep without relying on
   * timer cadence.
   */
  void sweep_idle_entries();

  /// Identify whether a topic is a ROS 2 system/infrastructure topic (filter target).
  [[nodiscard]] static bool is_system_topic(const std::string & topic_name);

 private:
  struct PoolEntry {
    std::string topic;
    std::string cached_type;
    std::unique_ptr<ros2_common::Ros2SubscriptionSlot> slot;

    mutable std::mutex buf_mtx;
    std::condition_variable buf_cv;
    std::optional<rclcpp::SerializedMessage> latest;
    std::int64_t latest_ns{0};
    std::chrono::steady_clock::time_point last_sample_time;
    std::atomic<bool> shutdown{false};
  };

  rclcpp::QoS qos_for(const std::string & topic) const;
  void on_graph_change();

  Config cfg_;
  std::shared_ptr<ros2_common::Ros2SubscriptionExecutor> exec_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;

  std::atomic<bool> shutdown_{false};

  // Shared alive flag for graph callback: registered with the executor so the
  // callback can check `*alive_` before dereferencing `this`. The executor's
  // fire_graph_callbacks() snapshots callbacks and posts them to the worker
  // queue; a callback posted before ~Ros2TopicDataProvider ran must no-op
  // once this object is gone. Shared ownership keeps the flag readable even
  // after the provider is destroyed.
  std::shared_ptr<std::atomic<bool>> alive_ = std::make_shared<std::atomic<bool>>(true);

  // Guards sweep_idle_entries execution. Worker thread runs the timer
  // callback via sub_executor_.spin_some(); timer->cancel() does not wait
  // for an in-flight callback, so the destructor grabs this mutex AFTER
  // cancel to synchronise with any sweep that slipped past the cancel.
  mutable std::mutex sweep_mutex_;

  mutable std::mutex pool_mtx_;
  std::unordered_map<std::string, std::shared_ptr<PoolEntry>> pool_;
  /// MRU at front, LRU at back. Only touched under pool_mtx_.
  std::list<std::string> lru_order_;
  std::unordered_map<std::string, std::list<std::string>::iterator> lru_pos_;

  std::size_t graph_listener_token_{ros2_common::Ros2SubscriptionExecutor::kMaxGraphListeners};
  rclcpp::TimerBase::SharedPtr idle_sweep_timer_;

  std::atomic<std::size_t> pool_hits_{0};
  std::atomic<std::size_t> pool_misses_{0};
  std::atomic<std::size_t> evictions_total_{0};
  std::atomic<std::size_t> type_change_events_{0};
  std::atomic<std::size_t> graph_events_received_{0};
  std::atomic<std::size_t> concurrent_cold_waits_{0};
};

}  // namespace ros2_medkit_gateway
