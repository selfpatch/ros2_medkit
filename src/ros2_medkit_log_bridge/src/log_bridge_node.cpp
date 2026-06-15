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

#include "ros2_medkit_log_bridge/log_bridge_node.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <functional>

#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_log_bridge {

namespace {

// rcl_interfaces/msg/Log severity levels.
constexpr uint8_t kLevelDebug = 10;
constexpr uint8_t kLevelInfo = 20;
constexpr uint8_t kLevelWarn = 30;
constexpr uint8_t kLevelError = 40;
constexpr uint8_t kLevelFatal = 50;

bool contains_substr(const std::vector<std::string> & list, const std::string & value) {
  for (const auto & item : list) {
    if (!item.empty() && value.find(item) != std::string::npos) {
      return true;
    }
  }
  return false;
}

}  // namespace

LogBridgeNode::LogBridgeNode(const rclcpp::NodeOptions & options) : Node("log_bridge", options) {
  load_parameters();
  own_node_name_ = get_fully_qualified_name();

  // /rosout is published reliable+volatile; keep a deep queue so log bursts at
  // fault time are not dropped before the bridge processes them.
  log_sub_ = create_subscription<rcl_interfaces::msg::Log>(
      rosout_topic_, rclcpp::QoS(rclcpp::KeepLast(1000)),
      [this](const rcl_interfaces::msg::Log::ConstSharedPtr & msg) { log_callback(msg); });

  RCLCPP_INFO(get_logger(), "LogBridge started (topic=%s, severity_floor=%u, prefix=%s)", rosout_topic_.c_str(),
              static_cast<unsigned>(severity_floor_), code_prefix_.c_str());
}

void LogBridgeNode::load_parameters() {
  rosout_topic_ = declare_parameter<std::string>("rosout_topic", "/rosout");
  // Default floor is WARN: WARN lands as PREFAILED (with a low FaultManager
  // confirmation_threshold), ERROR/FATAL confirm. Raise to 40 (ERROR) on
  // chatty / constrained targets to cut volume.
  severity_floor_ = static_cast<uint8_t>(declare_parameter<int>("severity_floor", kLevelWarn));
  code_prefix_ = declare_parameter<std::string>("code_prefix", "LOG");
  exclude_nodes_ = declare_parameter<std::vector<std::string>>("exclude_nodes", std::vector<std::string>{});
  include_only_nodes_ =
      declare_parameter<std::vector<std::string>>("include_only_nodes", std::vector<std::string>{});
}

void LogBridgeNode::log_callback(const rcl_interfaces::msg::Log::ConstSharedPtr & msg) {
  // Never promote our own logs (avoids any self-referential loop).
  if (msg->name == own_node_name_ || msg->name == "log_bridge") {
    return;
  }
  if (!node_is_eligible(msg->name)) {
    return;
  }

  uint8_t severity = 0;
  if (!map_level_to_severity(msg->level, severity_floor_, &severity)) {
    return;  // below floor / INFO / DEBUG -> dropped
  }

  const std::string fault_code = generate_fault_code(msg->name, msg->msg);
  // Attribute to the node FQN so the fault associates with the gateway's
  // runtime-discovered entity (and its snapshots/rosbag become reachable).
  auto * reporter = reporter_for(node_source_id(msg->name));
  if (reporter == nullptr) {
    return;
  }
  reporter->report(fault_code, severity, msg->msg);
  RCLCPP_DEBUG(get_logger(), "Log %s [lvl=%u] '%s' -> fault %s (severity=%u)", msg->name.c_str(),
               static_cast<unsigned>(msg->level), msg->msg.c_str(), fault_code.c_str(),
               static_cast<unsigned>(severity));
}

std::string LogBridgeNode::node_source_id(const std::string & log_name) {
  // The logger name is the node name, optionally with a sub-logger suffix after
  // '.' (node names themselves cannot contain '.'), so the first dotted segment
  // is the node. Prefix '/' to form the FQN the gateway's entity discovery uses.
  std::string node = log_name.substr(0, log_name.find('.'));
  if (node.empty()) {
    node = log_name;
  }
  if (node.empty() || node.front() != '/') {
    node = "/" + node;
  }
  return node;
}

bool LogBridgeNode::node_is_eligible(const std::string & node_name) const {
  if (!include_only_nodes_.empty() && !contains_substr(include_only_nodes_, node_name)) {
    return false;
  }
  if (contains_substr(exclude_nodes_, node_name)) {
    return false;
  }
  return true;
}

ros2_medkit_fault_reporter::FaultReporter * LogBridgeNode::reporter_for(const std::string & node_name) {
  std::lock_guard<std::mutex> lock(reporters_mutex_);
  auto it = reporters_.find(node_name);
  if (it != reporters_.end()) {
    return it->second.get();
  }
  // source_id is the ORIGINATING node so faults are attributed correctly.
  // Multiple FaultReporters on one node are safe: the ctor guards parameter
  // declaration with has_parameter().
  auto reporter =
      std::make_unique<ros2_medkit_fault_reporter::FaultReporter>(this->shared_from_this(), node_name);
  auto * raw = reporter.get();
  reporters_.emplace(node_name, std::move(reporter));
  return raw;
}

bool LogBridgeNode::map_level_to_severity(uint8_t log_level, uint8_t severity_floor, uint8_t * severity_out) {
  using Fault = ros2_medkit_msgs::msg::Fault;
  if (log_level < severity_floor || log_level <= kLevelInfo) {
    return false;
  }
  switch (log_level) {
    case kLevelWarn:
      *severity_out = Fault::SEVERITY_WARN;
      return true;
    case kLevelError:
      *severity_out = Fault::SEVERITY_ERROR;
      return true;
    case kLevelFatal:
      *severity_out = Fault::SEVERITY_CRITICAL;
      return true;
    default:
      // Unknown level >= floor and above INFO: treat as ERROR rather than drop.
      *severity_out = Fault::SEVERITY_ERROR;
      return true;
  }
}

std::string LogBridgeNode::normalize_message(const std::string & message) {
  // Collapse anything that varies per-occurrence (numbers, hex, paths) so the
  // same logical message hashes to the same code. Keep only lowercase letters
  // and single spaces between word runs.
  std::string out;
  out.reserve(message.size());
  bool last_space = true;
  for (char c : message) {
    const auto uc = static_cast<unsigned char>(c);
    if (std::isalpha(uc)) {
      out += static_cast<char>(std::tolower(uc));
      last_space = false;
    } else {
      // digits, punctuation, slashes, whitespace -> a single separator space
      if (!last_space) {
        out += ' ';
        last_space = true;
      }
    }
  }
  // trim trailing space
  if (!out.empty() && out.back() == ' ') {
    out.pop_back();
  }
  return out;
}

std::string LogBridgeNode::to_upper_snake(const std::string & in, size_t max_len) {
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

std::string LogBridgeNode::generate_fault_code(const std::string & node_name, const std::string & message) const {
  // Node basename (drop namespace), upper-snake, length-capped.
  std::string base = node_name;
  const auto slash = base.find_last_of('/');
  if (slash != std::string::npos) {
    base = base.substr(slash + 1);
  }
  const std::string node_part = to_upper_snake(base, 32);

  // Stable 8-hex-char hash of the normalized message template.
  const std::string templ = normalize_message(message);
  const std::size_t h = std::hash<std::string>{}(templ);
  char hashbuf[9];
  std::snprintf(hashbuf, sizeof(hashbuf), "%08X", static_cast<unsigned>(h & 0xFFFFFFFFu));

  std::string code = code_prefix_;
  if (!node_part.empty()) {
    code += '_';
    code += node_part;
  }
  code += '_';
  code += hashbuf;

  if (code.size() > 64) {
    code.resize(64);
  }
  return code;
}

}  // namespace ros2_medkit_log_bridge
