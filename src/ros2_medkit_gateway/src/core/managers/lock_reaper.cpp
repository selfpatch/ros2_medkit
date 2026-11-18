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

#include "ros2_medkit_gateway/core/managers/lock_reaper.hpp"

#include <algorithm>
#include <exception>
#include <utility>

namespace ros2_medkit_gateway {

LockReaper::LockReaper(LockManager & locks, SubscriptionManager * subscriptions, int interval_sec, log::Logger logger)
  : locks_(locks)
  , subscriptions_(subscriptions)
  , interval_(std::chrono::seconds(std::max(1, interval_sec)))
  , logger_(std::move(logger)) {
  thread_ = std::thread([this] {
    run();
  });
}

LockReaper::~LockReaper() {
  stop();
}

void LockReaper::stop() {
  {
    std::lock_guard<std::mutex> guard(mutex_);
    stopping_ = true;
  }
  wake_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void LockReaper::run() {
  std::unique_lock<std::mutex> guard(mutex_);
  while (!stopping_) {
    if (wake_.wait_for(guard, interval_, [this] {
          return stopping_;
        })) {
      return;
    }
    guard.unlock();
    try {
      reap_once();
    } catch (const std::exception & e) {
      MEDKIT_LOG_ERROR(logger_, "Lock cleanup failed: %s", e.what());
    } catch (...) {
      MEDKIT_LOG_ERROR(logger_, "Lock cleanup failed with an unknown exception");
    }
    guard.lock();
  }
}

void LockReaper::reap_once() {
  for (const auto & info : locks_.cleanup_expired()) {
    MEDKIT_LOG_INFO(logger_, "Lock %s expired on entity %s", info.lock_id.c_str(), info.entity_id.c_str());
    const bool covers_subscriptions = info.scopes.empty() || std::find(info.scopes.begin(), info.scopes.end(),
                                                                       "cyclic-subscriptions") != info.scopes.end();
    if (!covers_subscriptions || subscriptions_ == nullptr) {
      continue;
    }
    for (const auto & sub : subscriptions_->list(info.entity_id)) {
      if (subscriptions_->remove(sub.id)) {
        MEDKIT_LOG_INFO(logger_, "Removed subscription %s on lock expiry", sub.id.c_str());
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
