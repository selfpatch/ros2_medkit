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

#include "ros2_medkit_gateway/resource_change_notifier.hpp"

#include <iostream>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

ResourceChangeNotifier::ResourceChangeNotifier() : worker_thread_(&ResourceChangeNotifier::worker_loop, this) {
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
  if (shutdown_flag_.load(std::memory_order_relaxed)) {
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
  }
  queue_cv_.notify_one();
}

void ResourceChangeNotifier::shutdown() {
  bool expected = false;
  if (!shutdown_flag_.compare_exchange_strong(expected, true)) {
    return;  // Already shut down
  }

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
        return !queue_.empty() || shutdown_flag_.load(std::memory_order_relaxed);
      });

      if (shutdown_flag_.load(std::memory_order_relaxed) && queue_.empty()) {
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
          std::cerr << "[ResourceChangeNotifier] Exception in subscriber callback: " << e.what() << '\n';
        } catch (...) {
          std::cerr << "[ResourceChangeNotifier] Unknown exception in subscriber callback" << '\n';
        }
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
