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

#include "ros2_medkit_action_status_bridge/action_status_bridge_node.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>

#include "action_msgs/msg/goal_status.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_action_status_bridge {

namespace {

constexpr char kStatusSuffix[] = "/_action/status";
constexpr char kStatusType[] = "action_msgs/msg/GoalStatusArray";

bool contains_substr(const std::vector<std::string> & list, const std::string & value) {
  for (const auto & item : list) {
    if (!item.empty() && value.find(item) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

ActionStatusBridgeNode::ActionStatusBridgeNode(const rclcpp::NodeOptions & options)
    : Node("action_status_bridge", options) {
  load_parameters();

  rescan_actions();
  rescan_timer_ = create_wall_timer(
      std::chrono::duration<double>(rescan_period_sec_), [this]() { rescan_actions(); });

  RCLCPP_INFO(get_logger(), "ActionStatusBridge started (prefix=%s, aborted_severity=%u, rescan=%.1fs)",
              code_prefix_.c_str(), static_cast<unsigned>(aborted_severity_), rescan_period_sec_);
}

void ActionStatusBridgeNode::load_parameters() {
  aborted_severity_ = static_cast<uint8_t>(
      declare_parameter<int>("aborted_severity", ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR));
  canceled_is_fault_ = declare_parameter<bool>("canceled_is_fault", false);
  heal_on_succeeded_ = declare_parameter<bool>("heal_on_succeeded", true);
  rescan_period_sec_ = declare_parameter<double>("rescan_period_sec", 2.0);
  code_prefix_ = declare_parameter<std::string>("code_prefix", "ACTION");
  exclude_actions_ = declare_parameter<std::vector<std::string>>("exclude_actions", std::vector<std::string>{});
  include_only_actions_ =
      declare_parameter<std::vector<std::string>>("include_only_actions", std::vector<std::string>{});
  handled_capacity_ = static_cast<size_t>(declare_parameter<int>("dedup_capacity", 4096));

  // Action terminal states are discrete, authoritative events, not flapping
  // conditions, and per-goal dedup above already prevents repeats. Pre-declare
  // the FaultReporter LocalFilter as disabled so ABORTED reports immediately
  // and, crucially, PASSED (healing on a later SUCCEEDED) is not throttled by
  // the default occurrence threshold. The reporter ctor's has_parameter guard
  // honours this value. Override via fault_reporter.local_filtering.enabled.
  if (!has_parameter("fault_reporter.local_filtering.enabled")) {
    declare_parameter("fault_reporter.local_filtering.enabled", false);
  }
}

void ActionStatusBridgeNode::rescan_actions() {
  const auto topics = get_topic_names_and_types();
  // Action status QoS is reliable + transient_local + keep_last; match it so we
  // receive the latched terminal status of a goal.
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().transient_local();

  for (const auto & [topic, types] : topics) {
    const std::string action_name = action_name_from_status_topic(topic);
    if (action_name.empty()) {
      continue;
    }
    if (std::find(types.begin(), types.end(), kStatusType) == types.end()) {
      continue;
    }
    if (!action_is_eligible(action_name)) {
      continue;
    }
    if (subs_.find(topic) != subs_.end()) {
      continue;  // already subscribed
    }
    auto sub = create_subscription<action_msgs::msg::GoalStatusArray>(
        topic, qos,
        [this, action_name](const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
          status_callback(action_name, msg);
        });
    subs_.emplace(topic, sub);
    RCLCPP_INFO(get_logger(), "Watching action '%s' (topic %s)", action_name.c_str(), topic.c_str());
  }
}

void ActionStatusBridgeNode::status_callback(const std::string & action_name,
                                             const action_msgs::msg::GoalStatusArray::ConstSharedPtr & msg) {
  using GoalStatus = action_msgs::msg::GoalStatus;

  for (const auto & gs : msg->status_list) {
    std::array<uint8_t, 16> uuid{};
    std::copy(gs.goal_info.goal_id.uuid.begin(), gs.goal_info.goal_id.uuid.end(), uuid.begin());
    const std::string uuid_hex = uuid_to_hex(uuid);
    const std::string key = uuid_hex + ":" + std::to_string(static_cast<int>(gs.status));

    if (gs.status == GoalStatus::STATUS_ABORTED ||
        (gs.status == GoalStatus::STATUS_CANCELED && canceled_is_fault_)) {
      if (!mark_handled(key)) {
        continue;  // already reported this terminal state for this goal
      }
      auto * reporter = reporter_for(action_name);
      if (reporter == nullptr) {
        continue;
      }
      const char * what = (gs.status == GoalStatus::STATUS_ABORTED) ? "aborted" : "canceled";
      const std::string desc = std::string("Goal ") + uuid_hex.substr(0, 8) + " " + what + " on action " +
                               action_name;
      reporter->report(aborted_fault_code(action_name), aborted_severity_, desc);
      RCLCPP_INFO(get_logger(), "Action %s goal %s %s -> fault %s", action_name.c_str(),
                  uuid_hex.substr(0, 8).c_str(), what, aborted_fault_code(action_name).c_str());
    } else if (gs.status == GoalStatus::STATUS_SUCCEEDED && heal_on_succeeded_) {
      if (!mark_handled(key)) {
        continue;
      }
      auto * reporter = reporter_for(action_name);
      if (reporter != nullptr) {
        reporter->report_passed(aborted_fault_code(action_name));
      }
    }
  }
}

ros2_medkit_fault_reporter::FaultReporter * ActionStatusBridgeNode::reporter_for(const std::string & action_name) {
  std::lock_guard<std::mutex> lock(reporters_mutex_);
  auto it = reporters_.find(action_name);
  if (it != reporters_.end()) {
    return it->second.get();
  }
  // Attribute the fault to the action (best stable identifier from the status
  // topic alone). Multiple reporters on one node are safe (has_parameter guard).
  auto reporter =
      std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(this->shared_from_this(), action_name);
  auto * raw = reporter.get();
  reporters_.emplace(action_name, std::move(reporter));
  return raw;
}

bool ActionStatusBridgeNode::mark_handled(const std::string & goal_status_key) {
  std::lock_guard<std::mutex> lock(handled_mutex_);
  if (handled_.find(goal_status_key) != handled_.end()) {
    return false;
  }
  handled_.insert(goal_status_key);
  handled_order_.push_back(goal_status_key);
  while (handled_order_.size() > handled_capacity_) {
    handled_.erase(handled_order_.front());
    handled_order_.pop_front();
  }
  return true;
}

bool ActionStatusBridgeNode::action_is_eligible(const std::string & action_name) const {
  if (!include_only_actions_.empty() && !contains_substr(include_only_actions_, action_name)) {
    return false;
  }
  if (contains_substr(exclude_actions_, action_name)) {
    return false;
  }
  return true;
}

std::string ActionStatusBridgeNode::action_name_from_status_topic(const std::string & topic) {
  const std::string suffix = kStatusSuffix;
  if (topic.size() <= suffix.size()) {
    return "";
  }
  if (topic.compare(topic.size() - suffix.size(), suffix.size(), suffix) != 0) {
    return "";
  }
  return topic.substr(0, topic.size() - suffix.size());
}

std::string ActionStatusBridgeNode::uuid_to_hex(const std::array<uint8_t, 16> & uuid) {
  std::string out;
  out.reserve(32);
  char buf[3];
  for (uint8_t b : uuid) {
    std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned>(b));
    out += buf;
  }
  return out;
}

std::string ActionStatusBridgeNode::to_upper_snake(const std::string & in, size_t max_len) {
  std::string result;
  result.reserve(in.size());
  bool last_sep = true;
  for (char c : in) {
    const auto uc = static_cast<unsigned char>(c);
    if (std::isalnum(uc)) {
      result += static_cast<char>(std::toupper(uc));
      last_sep = false;
    } else if (!last_sep) {
      result += '_';
      last_sep = true;
    }
  }
  while (!result.empty() && result.back() == '_') {
    result.pop_back();
  }
  if (result.size() > max_len) {
    result.resize(max_len);
    while (!result.empty() && result.back() == '_') {
      result.pop_back();
    }
  }
  return result;
}

std::string ActionStatusBridgeNode::aborted_fault_code(const std::string & action_name) const {
  const std::string action_part = to_upper_snake(action_name, 48);
  std::string code = code_prefix_;
  if (!action_part.empty()) {
    code += '_';
    code += action_part;
  }
  code += "_ABORTED";
  if (code.size() > 64) {
    code.resize(64);
  }
  return code;
}

}  // namespace ros2_medkit_action_status_bridge
