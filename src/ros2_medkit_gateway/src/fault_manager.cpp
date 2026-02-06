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
  get_fault_client_ = node_->create_client<ros2_medkit_msgs::srv::GetFault>("/fault_manager/get_fault");
  list_faults_client_ = node_->create_client<ros2_medkit_msgs::srv::ListFaults>("/fault_manager/list_faults");
  clear_fault_client_ = node_->create_client<ros2_medkit_msgs::srv::ClearFault>("/fault_manager/clear_fault");
  get_snapshots_client_ = node_->create_client<ros2_medkit_msgs::srv::GetSnapshots>("/fault_manager/get_snapshots");
  get_rosbag_client_ = node_->create_client<ros2_medkit_msgs::srv::GetRosbag>("/fault_manager/get_rosbag");
  list_rosbags_client_ = node_->create_client<ros2_medkit_msgs::srv::ListRosbags>("/fault_manager/list_rosbags");

  // Get configurable timeout
  service_timeout_sec_ = node_->declare_parameter("fault_service_timeout_sec", 5.0);

  RCLCPP_INFO(node_->get_logger(), "FaultManager initialized");
}

bool FaultManager::wait_for_services(std::chrono::duration<double> timeout) {
  return report_fault_client_->wait_for_service(timeout) && get_fault_client_->wait_for_service(timeout) &&
         list_faults_client_->wait_for_service(timeout) && clear_fault_client_->wait_for_service(timeout);
}

bool FaultManager::is_available() const {
  return report_fault_client_->service_is_ready() && get_fault_client_->service_is_ready() &&
         list_faults_client_->service_is_ready() && clear_fault_client_->service_is_ready();
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
  std::lock_guard<std::mutex> lock(report_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!report_fault_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "ReportFault service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
  request->fault_code = fault_code;
  request->event_type = ros2_medkit_msgs::srv::ReportFault::Request::EVENT_FAILED;
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
  result.success = response->accepted;
  result.data = {{"accepted", response->accepted}};
  if (!response->accepted) {
    result.error_message = "Fault report rejected";
  }

  return result;
}

FaultResult FaultManager::list_faults(const std::string & source_id, bool include_prefailed, bool include_confirmed,
                                      bool include_cleared, bool include_muted, bool include_clusters) {
  std::lock_guard<std::mutex> lock(list_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!list_faults_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "ListFaults service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListFaults::Request>();
  request->filter_by_severity = false;
  request->severity = 0;

  // Build status filter
  if (include_prefailed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
  }
  if (include_confirmed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  }
  if (include_cleared) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CLEARED);
  }

  // Correlation options
  request->include_muted = include_muted;
  request->include_clusters = include_clusters;

  auto future = list_faults_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "ListFaults service call timed out";
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

  // Add correlation data (always include counts, details only if requested)
  result.data["muted_count"] = response->muted_count;
  result.data["cluster_count"] = response->cluster_count;

  if (include_muted && !response->muted_faults.empty()) {
    json muted_array = json::array();
    for (const auto & muted : response->muted_faults) {
      muted_array.push_back({{"fault_code", muted.fault_code},
                             {"root_cause_code", muted.root_cause_code},
                             {"rule_id", muted.rule_id},
                             {"delay_ms", muted.delay_ms}});
    }
    result.data["muted_faults"] = muted_array;
  }

  if (include_clusters && !response->clusters.empty()) {
    auto to_seconds = [](const builtin_interfaces::msg::Time & t) {
      return t.sec + static_cast<double>(t.nanosec) / 1e9;
    };

    json clusters_array = json::array();
    for (const auto & cluster : response->clusters) {
      clusters_array.push_back({{"cluster_id", cluster.cluster_id},
                                {"rule_id", cluster.rule_id},
                                {"rule_name", cluster.rule_name},
                                {"label", cluster.label},
                                {"representative_code", cluster.representative_code},
                                {"representative_severity", cluster.representative_severity},
                                {"fault_codes", cluster.fault_codes},
                                {"count", cluster.count},
                                {"first_at", to_seconds(cluster.first_at)},
                                {"last_at", to_seconds(cluster.last_at)}});
    }
    result.data["clusters"] = clusters_array;
  }

  return result;
}

FaultWithEnvResult FaultManager::get_fault_with_env(const std::string & fault_code, const std::string & source_id) {
  std::lock_guard<std::mutex> lock(get_mutex_);
  FaultWithEnvResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!get_fault_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "GetFault service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetFault::Request>();
  request->fault_code = fault_code;

  auto future = get_fault_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "GetFault service call timed out";
    return result;
  }

  auto response = future.get();
  result.success = response->success;

  if (response->success) {
    result.fault = response->fault;
    result.environment_data = response->environment_data;

    // Verify source_id if provided
    if (!source_id.empty()) {
      bool matches = false;
      for (const auto & src : result.fault.reporting_sources) {
        if (src.rfind(source_id, 0) == 0) {
          matches = true;
          break;
        }
      }
      if (!matches) {
        result.success = false;
        result.error_message = "Fault not found for source: " + source_id;
        result.fault = ros2_medkit_msgs::msg::Fault();
        result.environment_data = ros2_medkit_msgs::msg::EnvironmentData();
      }
    }
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

FaultResult FaultManager::get_fault(const std::string & fault_code, const std::string & source_id) {
  // Use get_fault_with_env and convert to JSON
  auto env_result = get_fault_with_env(fault_code, source_id);

  FaultResult result;
  result.success = env_result.success;
  result.error_message = env_result.error_message;

  if (env_result.success) {
    result.data = fault_to_json(env_result.fault);
  }

  return result;
}

FaultResult FaultManager::clear_fault(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(clear_mutex_);
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

  // Include auto-cleared symptom codes if correlation is enabled
  if (!response->auto_cleared_codes.empty()) {
    result.data["auto_cleared_codes"] = response->auto_cleared_codes;
  }

  return result;
}

FaultResult FaultManager::get_snapshots(const std::string & fault_code, const std::string & topic) {
  std::lock_guard<std::mutex> lock(snapshots_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!get_snapshots_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "GetSnapshots service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetSnapshots::Request>();
  request->fault_code = fault_code;
  request->topic = topic;

  auto future = get_snapshots_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "GetSnapshots service call timed out";
    return result;
  }

  auto response = future.get();
  result.success = response->success;

  if (response->success) {
    // Parse the JSON data from the service response
    try {
      result.data = json::parse(response->data);
    } catch (const json::exception & e) {
      result.data = {{"raw_data", response->data}};
    }
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

FaultResult FaultManager::get_rosbag(const std::string & fault_code) {
  std::lock_guard<std::mutex> lock(rosbag_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!get_rosbag_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "GetRosbag service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::GetRosbag::Request>();
  request->fault_code = fault_code;

  auto future = get_rosbag_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "GetRosbag service call timed out";
    return result;
  }

  auto response = future.get();
  result.success = response->success;

  if (response->success) {
    result.data = {{"file_path", response->file_path},
                   {"format", response->format},
                   {"duration_sec", response->duration_sec},
                   {"size_bytes", response->size_bytes}};
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

FaultResult FaultManager::list_rosbags(const std::string & entity_fqn) {
  std::lock_guard<std::mutex> lock(list_rosbags_mutex_);
  FaultResult result;

  auto timeout = std::chrono::duration<double>(service_timeout_sec_);
  if (!list_rosbags_client_->wait_for_service(timeout)) {
    result.success = false;
    result.error_message = "ListRosbags service not available";
    return result;
  }

  auto request = std::make_shared<ros2_medkit_msgs::srv::ListRosbags::Request>();
  request->entity_fqn = entity_fqn;

  auto future = list_rosbags_client_->async_send_request(request);

  if (future.wait_for(timeout) != std::future_status::ready) {
    result.success = false;
    result.error_message = "ListRosbags service call timed out";
    return result;
  }

  auto response = future.get();
  result.success = response->success;

  if (response->success) {
    json rosbags = json::array();
    for (size_t i = 0; i < response->fault_codes.size(); ++i) {
      rosbags.push_back({{"fault_code", response->fault_codes[i]},
                         {"file_path", response->file_paths[i]},
                         {"format", response->formats[i]},
                         {"duration_sec", response->durations_sec[i]},
                         {"size_bytes", response->sizes_bytes[i]}});
    }
    result.data = {{"rosbags", rosbags}};
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

}  // namespace ros2_medkit_gateway
