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

#include "ros2_medkit_gateway/subscription_manager.hpp"

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace ros2_medkit_gateway {

CyclicInterval parse_interval(const std::string & str) {
  if (str == "fast") {
    return CyclicInterval::FAST;
  }
  if (str == "normal") {
    return CyclicInterval::NORMAL;
  }
  if (str == "slow") {
    return CyclicInterval::SLOW;
  }
  throw std::invalid_argument("Invalid interval: " + str);
}

std::string interval_to_string(CyclicInterval interval) {
  switch (interval) {
    case CyclicInterval::FAST:
      return "fast";
    case CyclicInterval::NORMAL:
      return "normal";
    case CyclicInterval::SLOW:
      return "slow";
  }
  return "normal";
}

std::chrono::milliseconds interval_to_duration(CyclicInterval interval) {
  switch (interval) {
    case CyclicInterval::FAST:
      return std::chrono::milliseconds(50);
    case CyclicInterval::NORMAL:
      return std::chrono::milliseconds(200);
    case CyclicInterval::SLOW:
      return std::chrono::milliseconds(500);
  }
  return std::chrono::milliseconds(200);
}

SubscriptionManager::SubscriptionManager(size_t max_subscriptions) : max_subscriptions_(max_subscriptions) {
}

SubscriptionManager::~SubscriptionManager() {
  shutdown();
}

std::string SubscriptionManager::generate_id() {
  uint64_t id = next_id_.fetch_add(1);
  std::ostringstream oss;
  oss << "sub_" << std::setw(3) << std::setfill('0') << id;
  return oss.str();
}

tl::expected<CyclicSubscriptionInfo, std::string>
SubscriptionManager::create(const std::string & entity_id, const std::string & entity_type,
                            const std::string & resource_uri, const std::string & topic_name,
                            const std::string & protocol, CyclicInterval interval, int duration_sec) {
  std::lock_guard<std::mutex> lock(map_mutex_);

  if (subscriptions_.size() >= max_subscriptions_) {
    return tl::make_unexpected("Maximum subscription capacity (" + std::to_string(max_subscriptions_) + ") reached");
  }

  auto now = std::chrono::steady_clock::now();
  auto state = std::make_shared<SubscriptionState>();
  state->info.id = generate_id();
  state->info.entity_id = entity_id;
  state->info.entity_type = entity_type;
  state->info.resource_uri = resource_uri;
  state->info.topic_name = topic_name;
  state->info.protocol = protocol;
  state->info.interval = interval;
  state->info.duration_sec = duration_sec;
  state->info.created_at = now;
  state->info.expires_at = now + std::chrono::seconds(duration_sec);

  auto info_copy = state->info;
  subscriptions_[state->info.id] = std::move(state);
  return info_copy;
}

std::optional<CyclicSubscriptionInfo> SubscriptionManager::get(const std::string & sub_id) const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  auto it = subscriptions_.find(sub_id);
  if (it == subscriptions_.end()) {
    return std::nullopt;
  }
  return it->second->info;
}

std::vector<CyclicSubscriptionInfo> SubscriptionManager::list(const std::string & entity_id) const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  std::vector<CyclicSubscriptionInfo> result;
  for (const auto & [id, state] : subscriptions_) {
    if (state->info.entity_id == entity_id) {
      result.push_back(state->info);
    }
  }
  return result;
}

tl::expected<CyclicSubscriptionInfo, std::string>
SubscriptionManager::update(const std::string & sub_id, std::optional<CyclicInterval> new_interval,
                            std::optional<int> new_duration) {
  std::shared_ptr<SubscriptionState> state;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = subscriptions_.find(sub_id);
    if (it == subscriptions_.end()) {
      return tl::make_unexpected("Subscription not found: " + sub_id);
    }
    state = it->second;
  }

  // Update fields under the per-subscription lock
  std::lock_guard<std::mutex> sub_lock(state->mtx);
  if (new_interval) {
    state->info.interval = *new_interval;
  }
  if (new_duration) {
    state->info.duration_sec = *new_duration;
    state->info.expires_at = std::chrono::steady_clock::now() + std::chrono::seconds(*new_duration);
  }

  auto info_copy = state->info;

  // Wake up any waiting SSE stream so it picks up the new interval
  state->cv.notify_all();

  return info_copy;
}

bool SubscriptionManager::remove(const std::string & sub_id) {
  std::shared_ptr<SubscriptionState> state;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = subscriptions_.find(sub_id);
    if (it == subscriptions_.end()) {
      return false;
    }
    state = it->second;
    subscriptions_.erase(it);
  }

  // Mark inactive and wake up any waiting SSE stream
  state->active.store(false);
  state->cv.notify_all();
  return true;
}

size_t SubscriptionManager::cleanup_expired() {
  auto now = std::chrono::steady_clock::now();
  std::vector<std::shared_ptr<SubscriptionState>> expired_states;

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    for (auto it = subscriptions_.begin(); it != subscriptions_.end();) {
      if (it->second->info.expires_at <= now) {
        expired_states.push_back(it->second);
        it = subscriptions_.erase(it);
      } else {
        ++it;
      }
    }
  }

  // Notify expired streams outside the map lock
  for (auto & state : expired_states) {
    state->active.store(false);
    state->cv.notify_all();
  }

  return expired_states.size();
}

size_t SubscriptionManager::active_count() const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return subscriptions_.size();
}

size_t SubscriptionManager::max_subscriptions() const {
  return max_subscriptions_;
}

void SubscriptionManager::shutdown() {
  shutdown_flag_.store(true);

  std::lock_guard<std::mutex> lock(map_mutex_);
  for (auto & [id, state] : subscriptions_) {
    state->active.store(false);
    state->cv.notify_all();
  }
}

bool SubscriptionManager::wait_for_update(const std::string & sub_id, std::chrono::milliseconds timeout) {
  std::shared_ptr<SubscriptionState> state;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = subscriptions_.find(sub_id);
    if (it == subscriptions_.end()) {
      return true;  // Already removed — signal caller to stop
    }
    state = it->second;
  }

  std::unique_lock<std::mutex> sub_lock(state->mtx);
  bool woken = state->cv.wait_for(sub_lock, timeout, [&]() {
    return state->notified || !state->active.load() || shutdown_flag_.load();
  });
  state->notified = false;

  return woken;
}

bool SubscriptionManager::is_active(const std::string & sub_id) const {
  std::lock_guard<std::mutex> lock(map_mutex_);
  auto it = subscriptions_.find(sub_id);
  if (it == subscriptions_.end()) {
    return false;
  }
  // Check both the active flag and expiry
  return it->second->active.load() && it->second->info.expires_at > std::chrono::steady_clock::now();
}

void SubscriptionManager::notify(const std::string & sub_id) {
  std::shared_ptr<SubscriptionState> state;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto it = subscriptions_.find(sub_id);
    if (it == subscriptions_.end()) {
      return;
    }
    state = it->second;
  }
  {
    std::lock_guard<std::mutex> sub_lock(state->mtx);
    state->notified = true;
  }
  state->cv.notify_all();
}

}  // namespace ros2_medkit_gateway
