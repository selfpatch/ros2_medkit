// Copyright 2025 mfaferek93
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

#include "ros2_medkit_fault_reporter/fault_reporter.hpp"

#include "rclcpp/version.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace ros2_medkit_fault_reporter {

FaultReporter::FaultReporter(const rclcpp::node_interfaces::NodeBaseInterface::SharedPtr & node_base,
                             const rclcpp::node_interfaces::NodeGraphInterface::SharedPtr & node_graph,
                             const rclcpp::node_interfaces::NodeServicesInterface::SharedPtr & node_services,
                             const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
                             const rclcpp::Logger & logger, const std::string & source_id,
                             const std::string & service_name)
  : source_id_(source_id)
  , logger_(logger)
  , core_(source_id, load_parameters(node_params, logger),
          [this](const ros2_medkit::fault_detection::FaultReport & report) {
            send_report(report);
          }) {
  // Validate source_id
  if (source_id_.empty()) {
    RCLCPP_WARN(logger_, "FaultReporter created with empty source_id, fault origins will be difficult to trace");
  }

  // Create service client. It outlives the node handle used to construct it, but still relies on the
  // ROS context and the node interfaces staying valid - reporting after node teardown / rclcpp shutdown
  // is not safe.
#if RCLCPP_VERSION_GTE(29, 0, 0)  // Kilted and later
  client_ = rclcpp::create_client<ros2_medkit_msgs::srv::ReportFault>(node_base, node_graph, node_services,
                                                                      service_name, rclcpp::ServicesQoS());
#else  // Humble, Jazzy
  client_ = rclcpp::create_client<ros2_medkit_msgs::srv::ReportFault>(
      node_base, node_graph, node_services, service_name, rmw_qos_profile_services_default, nullptr);
#endif

  RCLCPP_DEBUG(logger_, "FaultReporter initialized for source: %s", source_id_.c_str());
}

FaultReporter::FaultReporter(const rclcpp::Node::SharedPtr & node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node->get_node_base_interface(), node->get_node_graph_interface(),
                  node->get_node_services_interface(), node->get_node_parameters_interface(), node->get_logger(),
                  source_id, service_name) {
}

FaultReporter::FaultReporter(rclcpp::Node & node, const std::string & source_id, const std::string & service_name)
  : FaultReporter(node.get_node_base_interface(), node.get_node_graph_interface(), node.get_node_services_interface(),
                  node.get_node_parameters_interface(), node.get_logger(), source_id, service_name) {
}

FaultReporter::FaultReporter(rclcpp_lifecycle::LifecycleNode & node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node.get_node_base_interface(), node.get_node_graph_interface(), node.get_node_services_interface(),
                  node.get_node_parameters_interface(), node.get_logger(), source_id, service_name) {
}

FaultReporter::FaultReporter(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & source_id,
                             const std::string & service_name)
  : FaultReporter(node->get_node_base_interface(), node->get_node_graph_interface(),
                  node->get_node_services_interface(), node->get_node_parameters_interface(), node->get_logger(),
                  source_id, service_name) {
}

FilterConfig
FaultReporter::load_parameters(const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & node_params,
                               const rclcpp::Logger & logger) {
  const FilterConfig defaults;

  // Declare parameters with defaults if not already declared
  if (!node_params->has_parameter("fault_reporter.local_filtering.enabled")) {
    node_params->declare_parameter("fault_reporter.local_filtering.enabled", rclcpp::ParameterValue(defaults.enabled));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.default_threshold")) {
    node_params->declare_parameter("fault_reporter.local_filtering.default_threshold",
                                   rclcpp::ParameterValue(defaults.default_threshold));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.default_window_sec")) {
    node_params->declare_parameter("fault_reporter.local_filtering.default_window_sec",
                                   rclcpp::ParameterValue(defaults.default_window_sec));
  }
  if (!node_params->has_parameter("fault_reporter.local_filtering.bypass_severity")) {
    node_params->declare_parameter("fault_reporter.local_filtering.bypass_severity",
                                   rclcpp::ParameterValue(static_cast<int>(defaults.bypass_severity)));
  }

  // Parameters are int64: core::validate_filter_params checks the range before narrowing.
  std::vector<std::string> warnings;
  const FilterConfig config = core::validate_filter_params(
      node_params->get_parameter("fault_reporter.local_filtering.enabled").as_bool(),
      node_params->get_parameter("fault_reporter.local_filtering.default_threshold").as_int(),
      node_params->get_parameter("fault_reporter.local_filtering.default_window_sec").as_double(),
      node_params->get_parameter("fault_reporter.local_filtering.bypass_severity").as_int(), &warnings);
  for (const auto & warning : warnings) {
    RCLCPP_WARN(logger, "%s", warning.c_str());
  }

  RCLCPP_DEBUG(logger, "Filter config: enabled=%d, threshold=%d, window=%.1fs, bypass_severity=%d", config.enabled,
               config.default_threshold, config.default_window_sec, config.bypass_severity);
  return config;
}

void FaultReporter::report(const std::string & fault_code, uint8_t severity, const std::string & description) {
  // Validate fault_code
  if (fault_code.empty()) {
    RCLCPP_WARN(logger_, "Attempted to report fault with empty fault_code, ignoring");
    return;
  }

  if (!core_.report(fault_code, severity, description)) {
    RCLCPP_DEBUG(logger_, "Fault '%s' filtered (threshold not met)", fault_code.c_str());
  }
}

void FaultReporter::report_passed(const std::string & fault_code) {
  if (fault_code.empty()) {
    RCLCPP_WARN(logger_, "Attempted to report PASSED with empty fault_code, ignoring");
    return;
  }

  if (!core_.report_passed(fault_code)) {
    RCLCPP_DEBUG(logger_, "PASSED for '%s' filtered (threshold not met)", fault_code.c_str());
  }
}

bool FaultReporter::is_service_ready() const {
  return client_->service_is_ready();
}

void FaultReporter::send_report(const ros2_medkit::fault_detection::FaultReport & report) {
  using Request = ros2_medkit_msgs::srv::ReportFault::Request;
  static_assert(static_cast<uint8_t>(ros2_medkit::fault_detection::ReportKind::kFailed) == Request::EVENT_FAILED,
                "ReportKind::kFailed must match ReportFault EVENT_FAILED");
  static_assert(static_cast<uint8_t>(ros2_medkit::fault_detection::ReportKind::kPassed) == Request::EVENT_PASSED,
                "ReportKind::kPassed must match ReportFault EVENT_PASSED");

  const bool failed = report.kind == ros2_medkit::fault_detection::ReportKind::kFailed;
  if (!client_->service_is_ready()) {
    // Use WARN level for high-severity faults that would bypass filtering
    if (failed && report.severity >= core_.filter().config().bypass_severity) {
      RCLCPP_WARN(logger_, "FaultManager service not available, dropping high-severity fault '%s'",
                  report.fault_code.c_str());
    } else {
      RCLCPP_DEBUG(logger_, "FaultManager service not available, skipping report for '%s'", report.fault_code.c_str());
    }
    return;
  }

  auto request = std::make_shared<Request>();
  request->fault_code = report.fault_code;
  request->event_type = static_cast<uint8_t>(report.kind);
  request->severity = report.severity;
  request->description = report.description;
  request->source_id = source_id_;

  // Fire and forget - don't block on response
  client_->async_send_request(request);

  if (failed) {
    RCLCPP_DEBUG(logger_, "Reported FAILED: %s (severity=%d)", report.fault_code.c_str(), report.severity);
  } else {
    RCLCPP_DEBUG(logger_, "Reported PASSED: %s", report.fault_code.c_str());
  }
}

}  // namespace ros2_medkit_fault_reporter
