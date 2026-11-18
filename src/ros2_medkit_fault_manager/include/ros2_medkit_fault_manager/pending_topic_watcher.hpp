// Copyright 2026 Wiktor Jelen
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
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace ros2_medkit_fault_manager {

/// Waits for topics the graph has no type for yet and hands each one over,
/// once, as soon as it has one.
///
/// A capture is configured with topic names, but a topic exists only while
/// something publishes it, and the fault manager is often up before the nodes
/// it captures. A subscription needs the type, so a topic configured ahead of
/// its publisher has to be looked for again later rather than dropped.
///
/// Every callback runs on the thread that spins the node. The node-ops mutex is
/// held only while the watcher creates or destroys its own timer, never during
/// a hand-over, so the hand-over may take it to create its subscription.
class PendingTopicWatcher {
 public:
  using OnTypeKnown = std::function<void(const std::string & topic, const std::string & type)>;

  /// @param node Node whose graph is read and whose wall timer drives the checks
  /// @param node_ops_mutex Mutex that serialises creating and destroying entities on @p node
  /// @param topics Topics to wait for; none may be handed over yet
  /// @param period Time between two reads of the graph
  /// @param on_type_known Called once per topic, with the first type the graph reports for it
  PendingTopicWatcher(rclcpp::Node * node, std::mutex & node_ops_mutex, std::vector<std::string> topics,
                      std::chrono::milliseconds period, OnTypeKnown on_type_known);

  /// Must not run while the node is being spun, as for any entity the timer's
  /// owner destroys: a check already under way would outlive the watcher.
  ~PendingTopicWatcher();

  PendingTopicWatcher(const PendingTopicWatcher &) = delete;
  PendingTopicWatcher & operator=(const PendingTopicWatcher &) = delete;
  PendingTopicWatcher(PendingTopicWatcher &&) = delete;
  PendingTopicWatcher & operator=(PendingTopicWatcher &&) = delete;

 private:
  void check();

  rclcpp::Node * node_;
  std::mutex & node_ops_mutex_;
  std::vector<std::string> pending_;
  OnTypeKnown on_type_known_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace ros2_medkit_fault_manager
