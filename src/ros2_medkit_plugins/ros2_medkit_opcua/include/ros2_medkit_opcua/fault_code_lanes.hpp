// Copyright 2026 mfaferek93
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

#include <chrono>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

/// One fault service call in flight per fault code.
///
/// ReportFault and ClearFault are separate services, so a report and the clear
/// that follows it a moment later (an alarm that came and went inside one
/// publishing interval) can reach fault_manager in either order. A clear that
/// overtakes its report leaves the fault raised for good. Calls for one code
/// are therefore sent one after another, each when the previous one was
/// answered; calls for different codes do not wait for each other.
///
/// A call never answered (fault_manager restarted under it) holds its code
/// for at most ``stale_after``: the next ``submit`` for that code, or
/// ``expire``, then lets the next call out.
class FaultCodeLanes {
 public:
  using Clock = std::chrono::steady_clock;
  /// Sends one call and invokes ``done`` (from any thread) when it is answered.
  using Send = std::function<void(std::function<void()> done)>;

  explicit FaultCodeLanes(Clock::duration stale_after = std::chrono::seconds(5)) : stale_after_(stale_after) {
  }

  void submit(const std::string & fault_code, Send send, Clock::time_point now = Clock::now()) {
    unsigned generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      Lane & lane = lanes_[fault_code];
      if (lane.busy && now - lane.since < stale_after_) {
        lane.queue.push_back(std::move(send));
        return;
      }
      if (lane.busy) {  // the call in flight went unanswered: let the queue move
        lane.queue.push_back(std::move(send));
        send = std::move(lane.queue.front());
        lane.queue.pop_front();
      }
      lane.busy = true;
      lane.since = now;
      generation = ++lane.generation;
    }
    start(fault_code, send, generation);
  }

  /// Give up every call unanswered for ``stale_after`` and send what waits
  /// behind it. Call it on a timer: without it a queued call would wait for
  /// the next ``submit`` of its code, which may never come.
  void expire(Clock::time_point now = Clock::now()) {
    std::vector<std::tuple<std::string, Send, unsigned>> next;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (auto & [fault_code, lane] : lanes_) {
        if (!lane.busy || now - lane.since < stale_after_) {
          continue;
        }
        if (lane.queue.empty()) {
          lane.busy = false;
          continue;
        }
        next.emplace_back(fault_code, std::move(lane.queue.front()), ++lane.generation);
        lane.queue.pop_front();
        lane.since = now;
      }
    }
    for (const auto & [fault_code, send, generation] : next) {
      start(fault_code, send, generation);
    }
  }

  /// Codes with a call in flight (tests and diagnostics).
  size_t busy_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t n = 0;
    for (const auto & entry : lanes_) {
      n += entry.second.busy ? 1 : 0;
    }
    return n;
  }

 private:
  struct Lane {
    bool busy{false};
    Clock::time_point since{};
    unsigned generation{0};  // which call ``done`` belongs to
    std::deque<Send> queue;
  };

  void start(const std::string & fault_code, const Send & send, unsigned generation) {
    send([this, fault_code, generation]() {
      finish(fault_code, generation);
    });
  }

  void finish(const std::string & fault_code, unsigned generation) {
    Send next;
    unsigned next_generation = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = lanes_.find(fault_code);
      if (it == lanes_.end() || !it->second.busy || it->second.generation != generation) {
        return;  // a late answer to a call already given up as stale
      }
      Lane & lane = it->second;
      if (lane.queue.empty()) {
        lane.busy = false;
        return;
      }
      next = std::move(lane.queue.front());
      lane.queue.pop_front();
      lane.since = Clock::now();
      next_generation = ++lane.generation;
    }
    start(fault_code, next, next_generation);
  }

  Clock::duration stale_after_;
  mutable std::mutex mutex_;
  std::unordered_map<std::string, Lane> lanes_;
};

}  // namespace ros2_medkit_gateway
