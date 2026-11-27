// Copyright 2026 ahmedsleem109
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
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_reporter/fault_reporter.hpp"
#include "ros2_medkit_policy_bridge/core/fault_sync.hpp"
#include "ros2_medkit_policy_bridge/core/policy_monitor.hpp"
#include "ros2_medkit_policy_msgs/msg/policy_introspection.hpp"

namespace ros2_medkit_policy_bridge {

/// Bridge node that watches a learned policy through what it publishes about
/// its own inferences (ros2_medkit_policy_msgs/PolicyIntrospection) and turns
/// signs of trouble into FaultManager faults:
///
///   - <PREFIX>_CHUNK_DISAGREEMENT: consecutive predictions stop agreeing with each
///     other (chunk spread above spread_threshold for raise_after readings).
///   - <PREFIX>_SLOW_INFERENCE: inference took longer than max_inference_ms for
///     raise_after readings.
///
/// The signal logic lives in core::PolicyMonitor, and what was sent to the
/// FaultManager in core::FaultSync; this node moves messages in and reports out.
///
/// Threads: the node is meant for a single-threaded executor (main.cpp). The
/// subscription and the sync timer both run on it, so the members are not
/// locked. Adding the node to a multi-threaded executor needs a lock around
/// monitor_, sync_ and reporter_.
class PolicyBridgeNode : public rclcpp::Node {
 public:
  explicit PolicyBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Drops the subscription and timer before the rest of the node goes: their
  // callbacks capture `this`.
  ~PolicyBridgeNode() override;
  PolicyBridgeNode(const PolicyBridgeNode &) = delete;
  PolicyBridgeNode & operator=(const PolicyBridgeNode &) = delete;
  PolicyBridgeNode(PolicyBridgeNode &&) = delete;
  PolicyBridgeNode & operator=(PolicyBridgeNode &&) = delete;

  /// Copy a message into the ROS-free sample type.
  static core::PolicySample to_sample(const ros2_medkit_policy_msgs::msg::PolicyIntrospection & msg);

  /// Upper-snake fault code: <prefix>_<suffix>, limited to the medkit charset.
  static std::string fault_code(const std::string & prefix, const std::string & suffix);

  /// Settings in effect, after validation.
  const core::MonitorConfig & config() const {
    return config_;
  }
  uint8_t chunk_disagreement_severity() const {
    return chunk_disagreement_severity_;
  }
  uint8_t slow_inference_severity() const {
    return slow_inference_severity_;
  }
  double sync_period_sec() const {
    return sync_period_sec_;
  }
  const std::string & chunk_disagreement_code() const {
    return chunk_disagreement_code_;
  }
  const std::string & slow_inference_code() const {
    return slow_inference_code_;
  }

  /// Reports not yet handed to the FaultManager (for tests).
  size_t due_reports() const {
    return sync_.due().size();
  }

  /// How long the bridge waits for the publisher's name before it reports
  /// under the topic name instead.
  static constexpr std::chrono::seconds kSourceResolveGrace{5};
  /// The FaultManager's report service, and its event topic: each running
  /// FaultManager is one publisher there.
  static constexpr const char * kReportService = "/fault_manager/report_fault";
  static constexpr const char * kFaultManagerEvents = "/fault_manager/events";
  /// A raise that has waited this long for the FaultManager is logged once.
  static constexpr std::chrono::seconds kFaultManagerMissingWarn{10};
  /// Messages ignored for a reason already logged are counted, and the count
  /// is logged at most this often.
  static constexpr std::chrono::seconds kRejectionSummaryPeriod{60};

 private:
  void load_parameters();
  void on_introspection(const ros2_medkit_policy_msgs::msg::PolicyIntrospection::ConstSharedPtr & msg);
  void sync();
  bool new_fault_manager_seen();
  ros2_medkit_fault_reporter::FaultReporter * reporter();

  core::MonitorConfig config_;
  std::unique_ptr<core::PolicyMonitor> monitor_;
  core::FaultSync sync_{{}};
  std::string topic_param_;
  std::string source_id_;
  std::string chunk_disagreement_code_;
  std::string slow_inference_code_;
  uint8_t chunk_disagreement_severity_{1};
  uint8_t slow_inference_severity_{1};
  double sync_period_sec_{0.1};

  rclcpp::Subscription<ros2_medkit_policy_msgs::msg::PolicyIntrospection>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr sync_timer_;
  std::unique_ptr<ros2_medkit_fault_reporter::FaultReporter> reporter_;
  std::optional<std::chrono::steady_clock::time_point> source_wait_since_;
  bool fault_manager_ready_{false};
  std::set<std::array<uint8_t, RMW_GID_STORAGE_SIZE>> fault_manager_instances_;
  bool graph_error_logged_{false};
  std::optional<std::chrono::steady_clock::time_point> raise_waits_since_;
  bool missing_logged_{false};

  // Ignored messages, per kind of reason: the first is logged with its
  // reason, the rest are counted and summed up once per period.
  struct RejectionLog {
    std::string last_reason;
    uint64_t unlogged{0};
    std::chrono::steady_clock::time_point last_logged;
  };
  void log_rejection(const core::MonitorResult & result, const ros2_medkit_policy_msgs::msg::PolicyIntrospection & msg);
  std::map<core::Rejection, RejectionLog> rejections_;
};

}  // namespace ros2_medkit_policy_bridge
