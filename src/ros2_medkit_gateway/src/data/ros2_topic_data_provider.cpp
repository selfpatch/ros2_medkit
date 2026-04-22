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
#include <future>
#include <stdexcept>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/version.h>

#include "ros2_medkit_serialization/serialization_error.hpp"

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

  graph_listener_token_ = exec_->on_graph_change([this] {
    on_graph_change();
  });

  if (cfg_.idle_sweep_tick.count() > 0 && cfg_.idle_safety_net.count() > 0) {
    idle_sweep_timer_ = exec_->node()->create_wall_timer(cfg_.idle_sweep_tick, [this] {
      sweep_idle_entries();
    });
  }
}

Ros2TopicDataProvider::~Ros2TopicDataProvider() {
  shutdown_.store(true, std::memory_order_release);
  if (idle_sweep_timer_) {
    idle_sweep_timer_->cancel();
    idle_sweep_timer_.reset();
  }
  if (exec_ && graph_listener_token_ < ros2_common::Ros2SubscriptionExecutor::kMaxGraphListeners) {
    exec_->remove_graph_change(graph_listener_token_);
  }
  // Drain pool: wake every waiter, then drop each entry (slot dtor posts or releases).
  std::unordered_map<std::string, std::shared_ptr<PoolEntry>> taken;
  {
    std::lock_guard<std::mutex> lk(pool_mtx_);
    taken.swap(pool_);
    lru_order_.clear();
    lru_pos_.clear();
  }
  for (auto & [topic, entry] : taken) {
    (void)topic;
    entry->shutdown.store(true, std::memory_order_release);
    {
      std::lock_guard<std::mutex> bl(entry->buf_mtx);
      entry->buf_cv.notify_all();
    }
    entry->slot.reset();
  }
}

// ---- Sampling ---------------------------------------------------------------

rclcpp::QoS Ros2TopicDataProvider::qos_for(const std::string & topic) const {
  // Derive a subscriber QoS that is compatible with the topic's publishers.
  // - If any publisher is Reliable, use Reliable (best-effort would still
  //   connect but is narrower on delivery semantics).
  // - If any publisher is TransientLocal, use TransientLocal so we receive
  //   the latched last-message on subscribe (typical for "status" topics).
  // - History: always keep_last depth 1. The pool only keeps the newest
  //   serialized message; a deeper queue would just allocate and discard.
  //
  // Graph query is thread-safe (read-only) and runs on the caller thread
  // rather than the worker since it is purely informational.
  bool any_reliable = false;
  bool any_transient_local = false;
  auto pubs = exec_->node()->get_publishers_info_by_topic(topic);
  for (const auto & pub : pubs) {
    const auto & q = pub.qos_profile();
    if (q.reliability() == rclcpp::ReliabilityPolicy::Reliable) {
      any_reliable = true;
    }
    if (q.durability() == rclcpp::DurabilityPolicy::TransientLocal) {
      any_transient_local = true;
    }
  }
  rclcpp::QoS qos(1);  // keep_last depth 1 - we only need the newest
  if (any_reliable) {
    qos.reliable();
  } else {
    qos.best_effort();
  }
  if (any_transient_local) {
    qos.transient_local();
  }
  return qos;
}

tl::expected<TopicSampleResult, ErrorInfo> Ros2TopicDataProvider::sample(const std::string & topic,
                                                                         std::chrono::milliseconds timeout) {
  if (shutdown_.load(std::memory_order_acquire)) {
    return tl::unexpected(ErrorInfo{"ERR_GATEWAY_SHUTDOWN", "gateway shutting down", 503, nlohmann::json::object()});
  }

  auto info = get_topic_info(topic);
  TopicSampleResult result;
  result.topic_name = topic;
  result.timestamp_ns = now_ns_epoch();
  if (!info) {
    result.has_data = false;
    return result;
  }
  result.message_type = info->type;
  result.publisher_count = info->publisher_count;
  result.subscriber_count = info->subscriber_count;
  result.publishers = get_topic_publishers(topic);
  result.subscribers = get_topic_subscribers(topic);

  if (info->publisher_count == 0) {
    result.has_data = false;
    return result;
  }

  // Phase 1: look up existing pool entry; on hit, promote to MRU.
  std::shared_ptr<PoolEntry> entry;
  {
    std::lock_guard<std::mutex> lk(pool_mtx_);
    auto it = pool_.find(topic);
    if (it != pool_.end()) {
      entry = it->second;
      auto pos_it = lru_pos_.find(topic);
      if (pos_it != lru_pos_.end()) {
        lru_order_.splice(lru_order_.begin(), lru_order_, pos_it->second);
      }
    }
  }

  if (entry) {
    pool_hits_.fetch_add(1, std::memory_order_relaxed);
  } else {
    // Phase 2: build entry + callback, create slot outside pool_mtx_.
    auto new_entry = std::make_shared<PoolEntry>();
    new_entry->topic = topic;
    new_entry->cached_type = info->type;
    new_entry->last_sample_time = std::chrono::steady_clock::now();

    std::weak_ptr<PoolEntry> entry_weak = new_entry;
    auto cb = [entry_weak](std::shared_ptr<const rclcpp::SerializedMessage> msg) {
      auto e = entry_weak.lock();
      if (!e || e->shutdown.load(std::memory_order_acquire)) {
        return;
      }
      std::lock_guard<std::mutex> bl(e->buf_mtx);
      e->latest = *msg;
      e->latest_ns = now_ns_epoch();
      e->buf_cv.notify_all();
    };

    auto slot_or_err = ros2_common::Ros2SubscriptionSlot::create_generic(*exec_, topic, info->type, qos_for(topic), cb);
    if (!slot_or_err) {
      return tl::unexpected(ErrorInfo{"ERR_SUBSCRIBE_FAILED", slot_or_err.error(), 500, nlohmann::json::object()});
    }
    new_entry->slot = std::move(*slot_or_err);

    // Phase 3: insert. Drop ours if another thread raced us; otherwise evict
    // the LRU entry when the pool is already at cap.
    std::shared_ptr<PoolEntry> evicted;
    {
      std::lock_guard<std::mutex> lk(pool_mtx_);
      auto it = pool_.find(topic);
      if (it != pool_.end()) {
        entry = it->second;
        auto pos_it = lru_pos_.find(topic);
        if (pos_it != lru_pos_.end()) {
          lru_order_.splice(lru_order_.begin(), lru_order_, pos_it->second);
        }
      } else {
        if (pool_.size() >= cfg_.max_pool_size && !lru_order_.empty()) {
          const std::string lru_topic = lru_order_.back();
          lru_order_.pop_back();
          auto lru_it = pool_.find(lru_topic);
          if (lru_it != pool_.end()) {
            evicted = lru_it->second;
            pool_.erase(lru_it);
          }
          lru_pos_.erase(lru_topic);
          evictions_total_.fetch_add(1, std::memory_order_relaxed);
        }
        pool_[topic] = new_entry;
        lru_order_.push_front(topic);
        lru_pos_[topic] = lru_order_.begin();
        entry = new_entry;
      }
    }
    pool_misses_.fetch_add(1, std::memory_order_relaxed);
    if (evicted) {
      // Drop the evicted entry outside the lock; its slot destructor posts to
      // the worker (or takes the shutdown fast path).
      evicted->shutdown.store(true, std::memory_order_release);
      std::lock_guard<std::mutex> bl(evicted->buf_mtx);
      evicted->buf_cv.notify_all();
    }
  }

  // Phase 5: wait for latest message on the per-entry CV.
  std::unique_lock<std::mutex> bl(entry->buf_mtx);
  entry->last_sample_time = std::chrono::steady_clock::now();
  const bool cold = !entry->latest.has_value();
  if (cold) {
    // Cold-wait cap (httplib liveness): if too many HTTP handler threads are
    // already blocked on cold topics, degrade the remaining callers to
    // metadata-only so discovery / health endpoints keep responding.
    if (cfg_.cold_wait_cap > 0 && concurrent_cold_waits_.load(std::memory_order_acquire) >= cfg_.cold_wait_cap) {
      result.has_data = false;
      return result;
    }
    concurrent_cold_waits_.fetch_add(1, std::memory_order_acq_rel);
    entry->buf_cv.wait_for(bl, timeout, [&] {
      return entry->latest.has_value() || entry->shutdown.load(std::memory_order_acquire);
    });
    concurrent_cold_waits_.fetch_sub(1, std::memory_order_acq_rel);
  }
  if (!entry->latest.has_value() || entry->shutdown.load(std::memory_order_acquire)) {
    result.has_data = false;
    return result;
  }

  rclcpp::SerializedMessage serialized_copy = *entry->latest;
  const std::int64_t msg_ns = entry->latest_ns;
  const std::string type_name = entry->cached_type.empty() ? info->type : entry->cached_type;
  bl.unlock();

  if (!serializer_) {
    result.has_data = false;
    return result;
  }
  try {
    result.data = serializer_->deserialize(type_name, serialized_copy);
    result.has_data = true;
    if (msg_ns != 0) {
      result.timestamp_ns = msg_ns;
    }
  } catch (const ros2_medkit_serialization::TypeNotFoundError & e) {
    RCLCPP_WARN(exec_->node()->get_logger(), "Unknown type '%s' for topic '%s': %s", type_name.c_str(), topic.c_str(),
                e.what());
    result.has_data = false;
  } catch (const ros2_medkit_serialization::SerializationError & e) {
    RCLCPP_WARN(exec_->node()->get_logger(), "Deserialize failed on '%s': %s", topic.c_str(), e.what());
    result.has_data = false;
  } catch (const std::exception & e) {
    RCLCPP_WARN(exec_->node()->get_logger(), "Exception processing '%s': %s", topic.c_str(), e.what());
    result.has_data = false;
  }
  return result;
}

tl::expected<std::vector<TopicSampleResult>, ErrorInfo>
Ros2TopicDataProvider::sample_parallel(const std::vector<std::string> & topics, std::chrono::milliseconds timeout) {
  if (shutdown_.load(std::memory_order_acquire)) {
    return tl::unexpected(ErrorInfo{"ERR_GATEWAY_SHUTDOWN", "gateway shutting down", 503, nlohmann::json::object()});
  }

  std::vector<std::future<TopicSampleResult>> futures;
  futures.reserve(topics.size());
  for (const auto & t : topics) {
    futures.push_back(std::async(std::launch::async, [this, t, timeout] {
      auto r = sample(t, timeout);
      if (r) {
        return *r;
      }
      TopicSampleResult fallback;
      fallback.topic_name = t;
      fallback.has_data = false;
      fallback.timestamp_ns = now_ns_epoch();
      return fallback;
    }));
  }
  std::vector<TopicSampleResult> results;
  results.reserve(topics.size());
  for (auto & f : futures) {
    results.push_back(f.get());
  }
  return results;
}

void Ros2TopicDataProvider::on_graph_change() {
  graph_events_received_.fetch_add(1, std::memory_order_relaxed);
  if (shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  // Query current graph once, then evict entries no longer present.
  // Swallow "rcl context invalid" throws that race rclcpp::shutdown during
  // SIGINT - we either evict later or get picked up by idle sweep.
  std::map<std::string, std::vector<std::string>> current;
  try {
    current = exec_->node()->get_topic_names_and_types();
  } catch (const std::runtime_error &) {
    return;
  }

  std::vector<std::shared_ptr<PoolEntry>> to_drop;
  {
    std::lock_guard<std::mutex> lk(pool_mtx_);
    for (auto it = pool_.begin(); it != pool_.end();) {
      const std::string topic = it->first;
      auto graph_it = current.find(topic);
      const bool gone = (graph_it == current.end());
      const bool type_changed = (!gone && !graph_it->second.empty() && graph_it->second[0] != it->second->cached_type);
      if (gone || type_changed) {
        if (type_changed) {
          type_change_events_.fetch_add(1, std::memory_order_relaxed);
        }
        to_drop.push_back(it->second);
        it = pool_.erase(it);
        auto pos_it = lru_pos_.find(topic);
        if (pos_it != lru_pos_.end()) {
          lru_order_.erase(pos_it->second);
          lru_pos_.erase(pos_it);
        }
        evictions_total_.fetch_add(1, std::memory_order_relaxed);
      } else {
        ++it;
      }
    }
  }
  // Release slots outside pool_mtx_ so the slot destructor's run_sync does not
  // deadlock against any sample() thread waiting on pool_mtx_.
  for (auto & entry : to_drop) {
    entry->shutdown.store(true, std::memory_order_release);
    std::lock_guard<std::mutex> bl(entry->buf_mtx);
    entry->buf_cv.notify_all();
  }
  // shared_ptrs go out of scope here, triggering slot destruction.
}

void Ros2TopicDataProvider::sweep_idle_entries() {
  if (shutdown_.load(std::memory_order_acquire)) {
    return;
  }
  if (cfg_.idle_safety_net.count() <= 0) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const auto threshold = cfg_.idle_safety_net;

  std::vector<std::shared_ptr<PoolEntry>> to_drop;
  {
    std::lock_guard<std::mutex> lk(pool_mtx_);
    for (auto it = pool_.begin(); it != pool_.end();) {
      const std::string topic = it->first;
      std::chrono::steady_clock::time_point last_sample;
      {
        std::lock_guard<std::mutex> bl(it->second->buf_mtx);
        last_sample = it->second->last_sample_time;
      }
      if (now - last_sample >= threshold) {
        to_drop.push_back(it->second);
        it = pool_.erase(it);
        auto pos_it = lru_pos_.find(topic);
        if (pos_it != lru_pos_.end()) {
          lru_order_.erase(pos_it->second);
          lru_pos_.erase(pos_it);
        }
        evictions_total_.fetch_add(1, std::memory_order_relaxed);
      } else {
        ++it;
      }
    }
  }
  for (auto & entry : to_drop) {
    entry->shutdown.store(true, std::memory_order_release);
    std::lock_guard<std::mutex> bl(entry->buf_mtx);
    entry->buf_cv.notify_all();
  }
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

nlohmann::json Ros2TopicDataProvider::x_medkit_pool_snapshot() const {
  auto out = nlohmann::json::array();
  const auto now = std::chrono::steady_clock::now();
  std::lock_guard<std::mutex> lk(pool_mtx_);
  std::size_t lru_rank = 0;
  for (const auto & topic : lru_order_) {
    auto it = pool_.find(topic);
    if (it == pool_.end()) {
      continue;
    }
    const auto & entry = *it->second;
    std::chrono::steady_clock::time_point last_sample;
    bool has_latest = false;
    std::int64_t latest_ns = 0;
    {
      std::lock_guard<std::mutex> bl(entry.buf_mtx);
      last_sample = entry.last_sample_time;
      has_latest = entry.latest.has_value();
      latest_ns = entry.latest_ns;
    }
    const auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_sample).count();
    out.push_back({
        {"topic", topic},
        {"message_type", entry.cached_type},
        {"lru_rank", lru_rank},
        {"has_latest", has_latest},
        {"latest_ns", latest_ns},
        {"last_sample_age_ms", age_ms},
    });
    ++lru_rank;
  }
  return out;
}

nlohmann::json Ros2TopicDataProvider::x_medkit_stats() const {
  const auto p = stats();
  nlohmann::json out;
  out["x-medkit-data-provider"] = {
      {"pool_size", p.pool_size},
      {"pool_cap", p.pool_cap},
      {"pool_hits", p.pool_hits},
      {"pool_misses", p.pool_misses},
      {"metadata_cache_size", p.metadata_cache_size},
      {"metadata_cache_cap", p.metadata_cache_cap},
      {"evictions_total", p.evictions_total},
      {"type_change_events", p.type_change_events},
      {"graph_events_received", p.graph_events_received},
      {"concurrent_cold_waits", p.concurrent_cold_waits},
      {"cold_wait_cap", p.cold_wait_cap},
  };
  if (exec_) {
    const auto e = exec_->stats();
    out["x-medkit-subscription-executor"] = {
        {"worker_alive", e.worker_alive},
        {"degraded", e.degraded},
        {"queue_depth", e.queue_depth},
        {"queue_max_depth_observed", e.queue_max_depth_observed},
        {"queue_dropped", e.queue_dropped},
        {"tasks_completed", e.tasks_completed},
        {"tasks_failed", e.tasks_failed},
        {"last_task_latency_us", e.last_task_latency_us},
        {"max_task_latency_us", e.max_task_latency_us},
        {"current_task_age_ms", e.current_task_age_ms},
        {"watchdog_trips", e.watchdog_trips},
        {"graph_events_received", e.graph_events_received},
    };
  }
  return out;
}

Ros2TopicDataProvider::PoolStats Ros2TopicDataProvider::stats() const {
  PoolStats s{};
  {
    std::lock_guard<std::mutex> lk(pool_mtx_);
    s.pool_size = pool_.size();
  }
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
