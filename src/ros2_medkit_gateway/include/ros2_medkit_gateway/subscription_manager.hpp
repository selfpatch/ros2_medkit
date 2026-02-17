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
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Cyclic subscription delivery interval (SOVD SubscriptionInterval)
enum class CyclicInterval { FAST, NORMAL, SLOW };

/// Parse interval string ("fast", "normal", "slow") — throws std::invalid_argument
CyclicInterval parse_interval(const std::string & str);

/// Convert interval enum to string
std::string interval_to_string(CyclicInterval interval);

/// Convert interval enum to millisecond duration
std::chrono::milliseconds interval_to_duration(CyclicInterval interval);

/// Data model for a single cyclic subscription
struct CyclicSubscriptionInfo {
  std::string id;
  std::string entity_id;
  std::string entity_type;  // "apps" or "components"
  std::string resource_uri;
  std::string topic_name;
  std::string protocol;  // Always "sse" for now
  CyclicInterval interval{CyclicInterval::NORMAL};
  int duration_sec{0};
  std::chrono::steady_clock::time_point created_at;
  std::chrono::steady_clock::time_point expires_at;
};

/// Per-subscription synchronization state for SSE stream coordination.
/// The SSE stream thread waits on the CV; update/remove operations notify it.
struct SubscriptionState {
  CyclicSubscriptionInfo info;
  std::mutex mtx;
  std::condition_variable cv;
  std::atomic<bool> active{true};
};

/**
 * @brief Thread-safe in-memory store for cyclic subscriptions.
 *
 * Pure C++ class (no ROS 2 dependency). Manages subscription lifecycle:
 * CRUD operations, capacity enforcement, expiry cleanup, and stream
 * synchronization via per-subscription condition variables.
 */
class SubscriptionManager {
 public:
  explicit SubscriptionManager(size_t max_subscriptions = 100);
  ~SubscriptionManager();

  // Disable copy/move
  SubscriptionManager(const SubscriptionManager &) = delete;
  SubscriptionManager & operator=(const SubscriptionManager &) = delete;
  SubscriptionManager(SubscriptionManager &&) = delete;
  SubscriptionManager & operator=(SubscriptionManager &&) = delete;

  /// Create a new subscription. Returns subscription info or error string.
  tl::expected<CyclicSubscriptionInfo, std::string>
  create(const std::string & entity_id, const std::string & entity_type, const std::string & resource_uri,
         const std::string & topic_name, const std::string & protocol, CyclicInterval interval, int duration_sec);

  /// Get a subscription by ID
  std::optional<CyclicSubscriptionInfo> get(const std::string & sub_id) const;

  /// List all subscriptions for an entity
  std::vector<CyclicSubscriptionInfo> list(const std::string & entity_id) const;

  /// Update interval and/or duration. Returns updated info or error.
  tl::expected<CyclicSubscriptionInfo, std::string>
  update(const std::string & sub_id, std::optional<CyclicInterval> new_interval, std::optional<int> new_duration);

  /// Remove a subscription. Returns true if found and removed.
  bool remove(const std::string & sub_id);

  /// Remove all expired subscriptions. Returns number removed.
  size_t cleanup_expired();

  /// Number of active subscriptions
  size_t active_count() const;

  /// Maximum allowed subscriptions
  size_t max_subscriptions() const;

  /// Signal shutdown — wakes all waiting streams
  void shutdown();

  /**
   * @brief Wait for an update or removal on the given subscription.
   *
   * Used by SSE stream threads to sleep between samples. Returns true if
   * woken by notify/remove/shutdown, false on timeout (normal interval wait).
   */
  bool wait_for_update(const std::string & sub_id, std::chrono::milliseconds timeout);

  /// Check if a subscription is active (exists and not expired)
  bool is_active(const std::string & sub_id) const;

  /// Notify a subscription's waiting stream (e.g., after update or for shutdown)
  void notify(const std::string & sub_id);

 private:
  std::string generate_id();

  mutable std::mutex map_mutex_;
  std::unordered_map<std::string, std::shared_ptr<SubscriptionState>> subscriptions_;
  size_t max_subscriptions_;
  std::atomic<uint64_t> next_id_{1};
  std::atomic<bool> shutdown_flag_{false};
};

}  // namespace ros2_medkit_gateway
