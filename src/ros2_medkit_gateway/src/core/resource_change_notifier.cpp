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

#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"

#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

ResourceChangeNotifier::ResourceChangeNotifier(ErrorLoggerFn error_logger)
  : error_logger_(std::move(error_logger)), worker_thread_(&ResourceChangeNotifier::worker_loop, this) {
}

ResourceChangeNotifier::~ResourceChangeNotifier() {
  shutdown();
}

NotifierSubscriptionId ResourceChangeNotifier::subscribe(NotifierFilter filter, NotifierCallback callback) {
  std::lock_guard<std::mutex> lock(sub_mutex_);
  auto id = next_id_.fetch_add(1);
  subscriptions_.emplace(id, SubscriptionEntry{std::move(filter), std::move(callback)});
  return id;
}

void ResourceChangeNotifier::unsubscribe(NotifierSubscriptionId id) {
  std::lock_guard<std::mutex> lock(sub_mutex_);
  subscriptions_.erase(id);
}

void ResourceChangeNotifier::notify(const std::string & collection, const std::string & entity_id,
                                    const std::string & resource_path, const nlohmann::json & value,
                                    ChangeType change_type) {
  // Prevent destruction while we're inside this function. The counter is
  // incremented BEFORE checking the shutdown flag (both seq_cst) so that
  // shutdown() is guaranteed to see the increment and wait for us to finish.
  active_notify_count_.fetch_add(1);
  struct NotifyGuard {
    ResourceChangeNotifier & self;
    explicit NotifyGuard(ResourceChangeNotifier & s) : self(s) {
    }
    ~NotifyGuard() {
      if (self.active_notify_count_.fetch_sub(1) == 1) {
        std::lock_guard<std::mutex> lk(self.drain_mutex_);
        self.drain_cv_.notify_one();
      }
    }
    NotifyGuard(const NotifyGuard &) = delete;
    NotifyGuard & operator=(const NotifyGuard &) = delete;
    NotifyGuard(NotifyGuard &&) = delete;
    NotifyGuard & operator=(NotifyGuard &&) = delete;
  } guard{*this};

  if (shutdown_flag_.load()) {
    return;
  }

  ResourceChange change;
  change.collection = collection;
  change.entity_id = entity_id;
  change.resource_path = resource_path;
  change.value = value;
  change.change_type = change_type;
  change.timestamp = std::chrono::system_clock::now();

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    queue_.push_back(std::move(change));

    // Enforce bounded queue: drop oldest entries when limit is exceeded.
    // Log only once per overflow batch to avoid log flooding.
    if (queue_.size() > max_queue_size_) {
      size_t drop_count = 0;
      while (queue_.size() > max_queue_size_) {
        queue_.pop_front();
        ++drop_count;
      }
      overflow_drop_count_ += drop_count;
      if (error_logger_) {
        error_logger_("ResourceChangeNotifier: queue overflow - dropped " + std::to_string(drop_count) +
                      " notification(s) (total dropped: " + std::to_string(overflow_drop_count_) + ")");
      }
    }
  }
  queue_cv_.notify_one();
}

void ResourceChangeNotifier::shutdown() {
  bool expected = false;
  if (!shutdown_flag_.compare_exchange_strong(expected, true)) {
    return;  // Already shut down
  }

  // Wait for in-flight notify() calls to finish. Uses a CV (not a spin loop)
  // so TSan can reason about the synchronization correctly.
  {
    std::unique_lock<std::mutex> lock(drain_mutex_);
    drain_cv_.wait(lock, [this]() {
      return active_notify_count_.load() == 0;
    });
  }

  // Synchronize with worker_loop()'s predicate check. Without this, the flag
  // store above can land between the worker's predicate evaluation and its
  // wait() call, losing the subsequent notify_one(). Acquiring queue_mutex_
  // here guarantees the worker is either still outside the critical section
  // (will observe the new flag) or already enqueued on queue_cv_ (notify will
  // wake it).
  { std::lock_guard<std::mutex> sync(queue_mutex_); }
  queue_cv_.notify_one();
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

bool ResourceChangeNotifier::matches_filter(const NotifierFilter & filter, const ResourceChange & change) {
  // Empty collection = match all; non-empty = exact match
  if (!filter.collection.empty() && filter.collection != change.collection) {
    return false;
  }
  // Empty entity_id = match all; non-empty = exact match
  if (!filter.entity_id.empty() && filter.entity_id != change.entity_id) {
    return false;
  }
  // Empty resource_path = match all; non-empty = exact match
  if (!filter.resource_path.empty() && filter.resource_path != change.resource_path) {
    return false;
  }
  return true;
}

void ResourceChangeNotifier::worker_loop() {
  while (true) {
    std::vector<ResourceChange> batch;

    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this]() {
        return !queue_.empty() || shutdown_flag_.load();
      });

      if (shutdown_flag_.load() && queue_.empty()) {
        return;
      }

      // Drain the queue
      batch.assign(std::make_move_iterator(queue_.begin()), std::make_move_iterator(queue_.end()));
      queue_.clear();
    }

    // Snapshot subscriptions to avoid holding sub_mutex_ while calling callbacks
    std::vector<SubscriptionEntry> snapshot;
    {
      std::lock_guard<std::mutex> lock(sub_mutex_);
      snapshot.reserve(subscriptions_.size());
      for (const auto & [id, entry] : subscriptions_) {
        snapshot.push_back(entry);
      }
    }

    for (const auto & change : batch) {
      for (const auto & entry : snapshot) {
        if (!matches_filter(entry.filter, change)) {
          continue;
        }
        try {
          entry.callback(change);
        } catch (const std::exception & e) {
          if (error_logger_) {
            error_logger_(std::string("Exception in subscriber callback: ") + e.what());
          }
        } catch (...) {
          if (error_logger_) {
            error_logger_("Unknown exception in subscriber callback");
          }
        }
      }
    }
  }
}

void ResourceChangeNotifier::set_max_queue_size(size_t max_size) {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  max_queue_size_ = max_size;
}

}  // namespace ros2_medkit_gateway
