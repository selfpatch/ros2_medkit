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

#include <condition_variable>
#include <mutex>
#include <thread>

#include "ros2_medkit_gateway/core/logging.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Removes expired locks on a thread of its own.
 *
 * Every `interval_sec` the thread calls LockManager::cleanup_expired(). For each expired lock whose scope
 * includes `cyclic-subscriptions` (or has no scope), it removes the cyclic subscriptions of the locked entity.
 * An interval below one second is raised to one second.
 *
 * The destructor stops and joins the thread. The lock manager and the subscription manager must outlive the reaper.
 */
class LockReaper {
 public:
  /// @p subscriptions may be null: then only the locks are removed.
  LockReaper(LockManager & locks, SubscriptionManager * subscriptions, int interval_sec, log::Logger logger);
  ~LockReaper();

  LockReaper(const LockReaper &) = delete;
  LockReaper & operator=(const LockReaper &) = delete;
  LockReaper(LockReaper &&) = delete;
  LockReaper & operator=(LockReaper &&) = delete;

  /// Stops and joins the thread. Safe to call more than once.
  void stop();

 private:
  void run();
  void reap_once();

  LockManager & locks_;
  SubscriptionManager * subscriptions_;
  std::chrono::seconds interval_;
  log::Logger logger_;
  std::mutex mutex_;
  std::condition_variable wake_;
  bool stopping_{false};
  std::thread thread_;
};

}  // namespace ros2_medkit_gateway
