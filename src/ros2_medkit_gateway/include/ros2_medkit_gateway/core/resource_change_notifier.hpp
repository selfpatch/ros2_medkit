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
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include <nlohmann/json.hpp>

namespace ros2_medkit_gateway {

/// Type of resource change that occurred.
enum class ChangeType { CREATED, UPDATED, DELETED };

/// Describes a single resource change event.
struct ResourceChange {
  std::string collection;     ///< "faults", "updates", "data", "x-custom"
  std::string entity_id;      ///< "temp_sensor"
  std::string resource_path;  ///< "fault_003" or "temperature"
  nlohmann::json value;       ///< The changed item (not full collection)
  ChangeType change_type;
  std::chrono::system_clock::time_point timestamp;
};

/// Filter for selecting which resource changes a subscriber cares about.
struct NotifierFilter {
  std::string collection;     ///< Optional - empty = all collections (catch-all)
  std::string entity_id;      ///< Optional - empty = all entities
  std::string resource_path;  ///< Optional - empty = all resources
};

using NotifierCallback = std::function<void(const ResourceChange &)>;
using NotifierSubscriptionId = uint64_t;

/// Central async notification hub for resource changes.
///
/// Producers (managers, plugins) call notify() to report resource changes.
/// Observers (TriggerManager) register callbacks with filters.
/// notify() is non-blocking - it pushes to an internal queue processed by a
/// dedicated worker thread.
class ResourceChangeNotifier {
 public:
  using ErrorLoggerFn = std::function<void(const std::string &)>;

  explicit ResourceChangeNotifier(ErrorLoggerFn error_logger = {});
  ~ResourceChangeNotifier();

  // Non-copyable, non-movable
  ResourceChangeNotifier(const ResourceChangeNotifier &) = delete;
  ResourceChangeNotifier & operator=(const ResourceChangeNotifier &) = delete;
  ResourceChangeNotifier(ResourceChangeNotifier &&) = delete;
  ResourceChangeNotifier & operator=(ResourceChangeNotifier &&) = delete;

  /// Register a callback to be invoked when a matching resource change occurs.
  /// @param filter Determines which changes trigger the callback
  /// @param callback Function called with the ResourceChange details
  /// @return A unique subscription ID for later unsubscription
  NotifierSubscriptionId subscribe(NotifierFilter filter, NotifierCallback callback);

  /// Remove a subscription by ID. No-op if ID not found.
  void unsubscribe(NotifierSubscriptionId id);

  /// Report a resource change. Non-blocking - pushes to async work queue.
  /// After shutdown(), this is a no-op.
  void notify(const std::string & collection, const std::string & entity_id, const std::string & resource_path,
              const nlohmann::json & value, ChangeType change_type = ChangeType::UPDATED);

  /// Shut down the worker thread. After this, notify() is a no-op.
  /// Safe to call multiple times.
  void shutdown();

  /// Default maximum number of pending notifications in the queue.
  static constexpr size_t kDefaultMaxQueueSize = 10000;

  /// Set the maximum queue size. When exceeded, the oldest entries are dropped.
  /// Must be called before concurrent use (typically from GatewayNode init).
  void set_max_queue_size(size_t max_size);

 private:
  struct SubscriptionEntry {
    NotifierFilter filter;
    NotifierCallback callback;
  };

  /// Check if a change matches a filter.
  static bool matches_filter(const NotifierFilter & filter, const ResourceChange & change);

  /// Worker thread loop - processes queued changes.
  void worker_loop();

  // Subscription storage (separate mutex to avoid contention with queue)
  std::mutex sub_mutex_;
  std::unordered_map<NotifierSubscriptionId, SubscriptionEntry> subscriptions_;
  std::atomic<uint64_t> next_id_{1};

  // Async work queue
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<ResourceChange> queue_;
  size_t max_queue_size_{kDefaultMaxQueueSize};
  size_t overflow_drop_count_{0};  ///< Accumulated drops since last overflow log

  // Optional error logger - must be declared before worker_thread_ so it is
  // initialized before the worker thread starts (C++ initializes members in
  // declaration order, not initializer-list order).
  ErrorLoggerFn error_logger_;

  // Worker thread lifecycle
  std::atomic<bool> shutdown_flag_{false};
  std::atomic<int> active_notify_count_{0};
  std::mutex drain_mutex_;
  std::condition_variable drain_cv_;
  std::thread worker_thread_;
};

}  // namespace ros2_medkit_gateway
