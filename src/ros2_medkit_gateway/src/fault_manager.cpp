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

#include "ros2_medkit_gateway/fault_manager.hpp"

#include <algorithm>
#include <chrono>

#include <builtin_interfaces/msg/time.hpp>

using namespace std::chrono_literals;

namespace ros2_medkit_gateway {

FaultManager::FaultManager(rclcpp::Node * node) : node_(node) {
  // Create service clients for fault_manager services
  report_fault_client_ = node_->create_client<ros2_medkit_msgs::srv::ReportFault>("/fault_manager/report_fault");
  get_faults_client_ = node_->create_client<ros2_medkit_msgs::srv::GetFaults>("/fault_manager/get_faults");
  clear_fault_client_ = node_->create_client<ros2_medkit_msgs::srv::ClearFault>("/fault_manager/clear_fault");

  // Get configurable timeout
  service_timeout_sec_ = node_->declare_parameter("fault_service_timeout_sec", 5.0);

  RCLCPP_INFO(node_->get_logger(), "FaultManager initialized");
}

bool FaultManager::wait_for_services(std::chrono::duration<double> timeout) {
  return report_fault_client_->wait_for_service(timeout) && get_faults_client_->wait_for_service(timeout) &&
         clear_fault_client_->wait_for_service(timeout);
}

bool FaultManager::is_available() const {
  return report_fault_client_->service_is_ready() && get_faults_client_->service_is_ready() &&
         clear_fault_client_->service_is_ready();
}

/// Convert a ROS 2 Fault message to JSON for REST API responses.
/// Timestamps are converted from builtin_interfaces::msg::Time (sec + nanosec) to seconds as double.
/// A human-readable severity_label is added based on the severity level.
json FaultManager::fault_to_json(const ros2_medkit_msgs::msg::Fault & fault) {
  // Convert ROS 2 Time to seconds as double
  auto to_seconds = [](const builtin_interfaces::msg::Time & t) {
    return t.sec + static_cast<double>(t.nanosec) / 1e9;
  };

  json j;
  j["fault_code"] = fault.fault_code;
  j["severity"] = fault.severity;
  j["description"] = fault.description;
  j["first_occurred"] = to_seconds(fault.first_occurred);
  j["last_occurred"] = to_seconds(fault.last_occurred);
  j["occurrence_count"] = fault.occurrence_count;
  j["status"] = fault.status;
  j["reporting_sources"] = fault.reporting_sources;

  // Add severity label for readability
  switch (fault.severity) {
    case ros2_medkit_msgs::msg::Fault::SEVERITY_INFO:
      j["severity_label"] = "INFO";
      break;
    case ros2_medkit_msgs::msg::Fault::SEVERITY_WARN:
      j["severity_label"] = "WARN";
      break;
    case ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR:
      j["severity_label"] = "ERROR";
      break;
    case ros2_medkit_msgs::msg::Fault::SEVERITY_CRITICAL:
      j["severity_label"] = "CRITICAL";
      break;
    default:
      j["severity_label"] = "UNKNOWN";
      break;
  }

  return j;
}

FaultResult FaultManager::report_fault(const std::string & fault_code, uint8_t severity,
                                       const std::string & description, const std::string & source_id) {
  std::lock_guard<std::mutex> lock(service_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!report_fault_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "ReportFault service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
  request->fault_code = fault_code;
  request->severity = severity;
  request->description = description;
  request->source_id = source_id;

  auto future = report_fault_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "ReportFault service call timed out";
    return result;
  }

  auto response = future.get();
  result.success = response->success;
  result.data = {{"success", response->success}, {"message", response->message}};
  if (!response->success) {
    result.error_message = response->message;
  }

  return result;
}

FaultResult FaultManager::get_faults(const std::string & source_id, bool include_pending, bool include_confirmed,
                                     bool include_cleared) {
  std::lock_guard<std::mutex> lock(service_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!get_faults_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "GetFaults service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetFaults::Request>();
  request->filter_by_severity = false;
  request->severity = 0;

  // Build status filter
  if (include_pending) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_PENDING);
  }
  if (include_confirmed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  }
  if (include_cleared) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CLEARED);
  }

  auto future = get_faults_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "GetFaults service call timed out";
    return result;
  }

  auto response = future.get();

  // Filter by source_id if provided (uses prefix matching)
  json faults_array = json::array();
  for (const auto & fault : response->faults) {
    // If source_id filter is provided, check if any reporting source starts with the filter
    // This allows querying by namespace (e.g., "/perception/lidar" matches "/perception/lidar/lidar_sensor")
    if (!source_id.empty()) {
      auto & sources = fault.reporting_sources;
      bool matches = false;
      for (const auto & src : sources) {
        // Match if source starts with the filter (prefix match for namespace hierarchy)
        if (src.rfind(source_id, 0) == 0) {
          matches = true;
          break;
        }
      }
      if (!matches) {
        continue;  // Skip faults not reported by this component/namespace
      }
    }
    faults_array.push_back(fault_to_json(fault));
  }

  result.success = true;
  result.data = {{"faults", faults_array}, {"count", faults_array.size()}};

  return result;
}

FaultResult FaultManager::get_fault(const std::string & fault_code, const std::string & source_id) {
  // Get all faults and filter by fault_code
  auto all_faults = get_faults(source_id, true, true, true);

  if (!all_faults.success) {
    return all_faults;
  }

  FaultResult result;

  // Find the specific fault
  for (const auto & fault : all_faults.data["faults"]) {
    if (fault["fault_code"] == fault_code) {
      result.success = true;
      result.data = fault;
      return result;
    }
  }

  result.success = false;
  result.error_message = "Fault not found: " + fault_code;
  return result;
}

FaultResult FaultManager::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(service_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!clear_fault_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "ClearFault service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ClearFault::Request>();
  request->fault_code = fault_code;

  auto future = clear_fault_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "ClearFault service call timed out";
    return result;
  }

  auto response = future.get();
  result.success = response->success;
  result.data = {{"success", response->success}, {"message", response->message}};
  if (!response->success) {
    result.error_message = response->message;
  }

  return result;
}

}  // namespace ros2_medkit_gateway
