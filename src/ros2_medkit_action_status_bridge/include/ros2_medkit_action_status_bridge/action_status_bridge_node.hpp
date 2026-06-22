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

#include <array>
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
///   - SUCCEEDED (4)-> PASSED (heals the per-action fault code) if enabled
///
/// Fault state is per-ACTION, not per-goal: every message is scanned for the net
/// state of the whole GoalStatusArray and a fault is raised/healed only on the
/// action-level transition. See `derive_state`.
class ActionStatusBridgeNode : public rclcpp::Node {
 public:
  explicit ActionStatusBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// Net fault state of an action, derived from a whole GoalStatusArray.
  ///   - kUnknown: no terminal goal in the array yet (no transition)
  ///   - kHealthy: array has terminal goals and none of them are failing
  ///   - kFailed:  at least one goal is failing (ABORTED, or CANCELED when
  ///               canceled_is_fault)
  enum class ActionState { kUnknown, kHealthy, kFailed };

  /// Derive the action name from a `/<action>/_action/status` topic name.
  /// Returns empty when the topic is not an action status topic.
  static std::string action_name_from_status_topic(const std::string & topic);

  /// Build a node FQN from a publisher endpoint's node name + namespace, or ""
  /// if unresolved. During DDS discovery rcl reports the placeholders
  /// "_NODE_NAME_UNKNOWN_" / "_NODE_NAMESPACE_UNKNOWN_" before the real name
  /// propagates; those (and an empty name) are treated as unresolved so a fault
  /// is never attributed to the placeholder.
  static std::string server_fqn_from_endpoint(const std::string & node_name, const std::string & node_namespace);

  /// Build the fault code for an action name and terminal status.
  /// Format: <PREFIX>_<ACTION>_<ABORTED|CANCELED>, charset/length per medkit
  /// rules. `canceled` selects the CANCELED suffix.
  std::string fault_code_for(const std::string & action_name, bool canceled) const;

  /// Lowercase hex of a 16-byte goal UUID, for dedup keys and short display.
  static std::string uuid_to_hex(const std::array<uint8_t, 16> & uuid);

  /// Scan a whole GoalStatusArray and return the action-level net state.
  /// `canceled_is_fault` decides whether CANCELED counts as failing. Order of
  /// the goals in the array does not affect the result (any failing goal wins).
  static ActionState derive_state(const action_msgs::msg::GoalStatusArray & msg, bool canceled_is_fault);

  /// Update per-action state from a message and act on the transition only.
  /// Returns the state that was reported on (kFailed on raise, kHealthy on
  /// heal) or kUnknown when nothing was reported. Side-effect free w.r.t.
  /// reporting when `reporter` is null (test seam): the transition decision and
  /// stored per-action state still update, so tests assert on the return value.
  ActionState apply_message(const std::string & action_name, const action_msgs::msg::GoalStatusArray & msg,
                            ros2_medkit_fault_reporter::FaultReporter * reporter);

 private:
  void rescan_actions();
  void status_callback(const std::string & action_name, const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg);

  ros2_medkit_fault_reporter::FaultReporter * reporter_for(const std::string & action_name);

  /// Resolve the action server's node FQN from its status-topic publisher, for
  /// use as the fault source_id so faults associate with the gateway's SOVD
  /// entity. Returns "" if no publisher with a discovery-resolved node name is
  /// visible yet (the caller falls back and re-resolves on a later message).
  std::string server_fqn_for_action(const std::string & action_name);

  /// Returns true if this (goal, status) pair was not logged before, marking it
  /// logged. Bounded to avoid unbounded growth. Suppresses duplicate LOG lines
  /// only; never gates the action-level state transition.
  bool mark_logged(const std::string & goal_status_key);

  bool action_is_eligible(const std::string & action_name) const;

  void load_parameters();

  /// Drop subscriptions, reporters and per-action state for actions whose status
  /// topic has vanished from `present_topics`.
  void prune_vanished(const std::map<std::string, std::string> & present_topics);

  static std::string to_upper_snake(const std::string & in, size_t max_len);

  rclcpp::TimerBase::SharedPtr rescan_timer_;
  std::map<std::string, rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr> subs_;

  std::map<std::string, std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter>> reporters_;
  // Actions whose reporter is attributed to the real (discovery-resolved) server
  // node FQN. Until an action is in this set its reporter uses a fallback source
  // and is re-resolved on each status message. Guarded by reporters_mutex_.
  std::unordered_set<std::string> resolved_actions_;
  std::mutex reporters_mutex_;

  // Last reported action-level state, keyed by action name. Drives transitions.
  std::map<std::string, ActionState> last_reported_state_;
  std::mutex state_mutex_;

  // Bounded dedup of logged (goal_id:status) keys (LOG suppression only).
  std::unordered_set<std::string> logged_;
  std::deque<std::string> logged_order_;
  std::mutex logged_mutex_;
  size_t logged_capacity_;

  // Configuration
  uint8_t aborted_severity_;
  bool canceled_is_fault_;
  bool heal_on_succeeded_;
  double rescan_period_sec_;
  std::string code_prefix_;
  std::vector<std::string> exclude_actions_;
  std::vector<std::string> include_only_actions_;

  friend class ActionStatusBridgeTestAccess;
};

/// Test-only accessor for the bridge's private maps and prune path. Lets unit
/// tests exercise rescan add+prune without a live action graph.
class ActionStatusBridgeTestAccess {
 public:
  explicit ActionStatusBridgeTestAccess(ActionStatusBridgeNode * node) : node_(node) {
  }

  /// Seed a watched action (subscription placeholder + per-action state) as if
  /// rescan had discovered its `/<action>/_action/status` topic.
  void add_watched(const std::string & action_name);

  /// Run the prune pass against a set of still-present action names.
  void prune_to(const std::vector<std::string> & present_action_names);

  bool is_watched(const std::string & action_name) const;
  bool has_state(const std::string & action_name) const;

 private:
  ActionStatusBridgeNode * node_;
};

}  // namespace ros2_medkit_action_status_bridge
