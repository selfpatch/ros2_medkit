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

#include "ros2_medkit_gateway/ros2/transports/ros2_log_source.hpp"

#include <utility>

#include "ros2_medkit_gateway/core/log_types.hpp"

namespace ros2_medkit_gateway::ros2 {

Ros2LogSource::Ros2LogSource(rclcpp::Node * node) : node_(node) {
}

Ros2LogSource::~Ros2LogSource() {
  // Idempotent stop() also runs from destructors; calling it explicitly here
  // gives a deterministic teardown sequence regardless of how the shared_ptr
  // last reference drops.
  stop();
}

void Ros2LogSource::start(EntryCallback callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  callback_ = std::move(callback);

  if (rosout_sub_) {
    // Subscription already created - reusing it is enough; the lambda below
    // reads callback_ under the same mutex.
    return;
  }

  rosout_sub_ = node_->create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", rclcpp::QoS(100), [this](const rcl_interfaces::msg::Log::ConstSharedPtr & msg) {
        if (shutdown_requested_.load(std::memory_order_acquire)) {
          return;
        }

        EntryCallback cb;
        {
          std::lock_guard<std::mutex> cb_lock(mutex_);
          cb = callback_;
        }
        if (!cb) {
          return;
        }

        LogEntry entry;
        // The manager assigns the monotonic id; the source leaves it unset.
        entry.id = 0;
        entry.stamp_sec = msg->stamp.sec;
        entry.stamp_nanosec = msg->stamp.nanosec;
        entry.level = msg->level;
        entry.name = msg->name;  // already without leading slash per rcl convention
        entry.msg = msg->msg;
        entry.function = msg->function;
        entry.file = msg->file;
        entry.line = msg->line;

        cb(entry);
      });
}

void Ros2LogSource::stop() {
  // Mark shutdown so any in-flight callback short-circuits before touching
  // members that are about to destruct.
  shutdown_requested_.store(true, std::memory_order_release);

  std::lock_guard<std::mutex> lock(mutex_);
  rosout_sub_.reset();
  callback_ = nullptr;
}

}  // namespace ros2_medkit_gateway::ros2
