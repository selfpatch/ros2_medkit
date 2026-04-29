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
#include <mutex>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/transports/log_source.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing LogSource by subscribing to /rosout.
 *
 * Owns the rclcpp::Subscription<rcl_interfaces::msg::Log> that previously
 * lived inside LogManager. Performs the rcl_interfaces::msg::Log -> LogEntry
 * conversion (level, name, message, function/file/line, timestamp) and
 * delivers each entry to the registered callback. The manager body then lives
 * in the ROS-free build layer.
 *
 * Threading: the underlying rclcpp callback is invoked on the gateway's
 * executor thread. The shutdown guard pattern ensures that any in-flight
 * callback short-circuits before the subscription is released, so the
 * registered EntryCallback is guaranteed not to fire after stop() returns.
 */
class Ros2LogSource : public LogSource {
 public:
  /**
   * @param node Non-owning ROS node used for subscription creation.
   */
  explicit Ros2LogSource(rclcpp::Node * node);

  ~Ros2LogSource() override;

  Ros2LogSource(const Ros2LogSource &) = delete;
  Ros2LogSource & operator=(const Ros2LogSource &) = delete;
  Ros2LogSource(Ros2LogSource &&) = delete;
  Ros2LogSource & operator=(Ros2LogSource &&) = delete;

  /// Subscribe to /rosout and route each message through to `callback`.
  /// Idempotent: a second start() replaces the previous callback and reuses
  /// the existing subscription.
  void start(EntryCallback callback) override;

  /// Stop delivering entries. Idempotent. After stop() returns the registered
  /// callback is guaranteed not to fire again, even if rclcpp has not yet
  /// drained queued messages.
  void stop() override;

 private:
  rclcpp::Node * node_;

  /// Set to true on stop() (or destruction) before the subscription is
  /// released, so any in-flight callback short-circuits before touching
  /// members that are about to destruct.
  std::atomic<bool> shutdown_requested_{false};

  /// Guards callback_ + rosout_sub_ across start()/stop() races.
  std::mutex mutex_;
  EntryCallback callback_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr rosout_sub_;
};

}  // namespace ros2_medkit_gateway::ros2
