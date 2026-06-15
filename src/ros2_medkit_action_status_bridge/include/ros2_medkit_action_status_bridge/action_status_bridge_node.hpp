// Copyright 2026 mfaferek93, bburda
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

#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

namespace ros2_medkit_action_status_bridge {

/// Bridge node that turns terminal ROS 2 action goal states into FaultManager
/// faults. Generic across every action: it observes the per-action
/// `/<action>/_action/status` topic (`action_msgs/msg/GoalStatusArray`) that
/// every action server publishes, so no per-project code is needed.
///
/// This catches the authoritative "the goal failed" verdict that neither the
/// /diagnostics bridge nor the /rosout log bridge can see - e.g. a Nav2
/// NavigateToPose aborting or a MoveIt MoveGroup goal aborting. The *reason*
/// (action-specific error code in the result) is a separate enrichment concern;
/// this bridge delivers the generic terminal status.
///
/// Status mapping (action_msgs/msg/GoalStatus):
///   - ABORTED (6)  -> fault (severity configurable, default ERROR)
///   - CANCELED (5) -> fault only if canceled_is_fault (usually intentional)
///   - SUCCEEDED (4)-> PASSED (heals the per-action ABORTED code) if enabled
class ActionStatusBridgeNode : public rclcpp::Node {
 public:
  explicit ActionStatusBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Derive the action name from a `/<action>/_action/status` topic name.
  /// Returns empty when the topic is not an action status topic.
  static std::string action_name_from_status_topic(const std::string & topic);

  /// Build the ABORTED fault code for an action name.
  /// Format: <PREFIX>_<ACTION>_ABORTED, charset/length per medkit rules.
  std::string aborted_fault_code(const std::string & action_name) const;

  /// Lowercase hex of a 16-byte goal UUID, for dedup keys and short display.
  static std::string uuid_to_hex(const std::array<uint8_t, 16> & uuid);

 private:
  void rescan_actions();
  void status_callback(const std::string & action_name,
                       const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg);

  ros2_medkit_fault_reporter::FaultReporter * reporter_for(const std::string & action_name);

  /// Returns true if this (goal, status) pair was not handled before, marking
  /// it handled. Bounded to avoid unbounded growth.
  bool mark_handled(const std::string & goal_status_key);

  bool action_is_eligible(const std::string & action_name) const;

  void load_parameters();

  static std::string to_upper_snake(const std::string & in, size_t max_len);

  rclcpp::TimerBase::SharedPtr rescan_timer_;
  std::map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> subs_;

  std::map<std::string, std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter>> reporters_;
  std::mutex reporters_mutex_;

  // Bounded dedup of handled (goal_id:status) keys.
  std::unordered_set<std::string> handled_;
  std::deque<std::string> handled_order_;
  std::mutex handled_mutex_;
  size_t handled_capacity_;

  // Configuration
  uint8_t aborted_severity_;
  bool canceled_is_fault_;
  bool heal_on_succeeded_;
  double rescan_period_sec_;
  std::string code_prefix_;
  std::vector<std::string> exclude_actions_;
  std::vector<std::string> include_only_actions_;
};

}  // namespace ros2_medkit_action_status_bridge
