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

#include "ros2_medkit_gateway/ros2/transports/ros2_fault_service_transport.hpp"

#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <future>

#include "ros2_medkit_gateway/fault_manager_paths.hpp"
#include "ros2_medkit_gateway/ros2/conversions/fault_msg_conversions.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_gateway::ros2 {

Ros2FaultServiceTransport::Ros2FaultServiceTransport(rclcpp::Node * node) : node_(node) {
  // Pick up configurable timeout. GatewayNode declares this parameter up front,
  // but unit tests may construct the transport with a plain rclcpp::Node.
  if (!node_->get_parameter("fault_manager.service_timeout_sec", service_timeout_sec_)) {
    service_timeout_sec_ = 5.0;
  }
  fault_manager_base_path_ = build_fault_manager_base_path(node_);

  report_fault_client_ =
      node_->create_client<ros2_medkit_msgs::srv::ReportFault>(fault_manager_base_path_ + "/report_fault");
  get_fault_client_ = node_->create_client<ros2_medkit_msgs::srv::GetFault>(fault_manager_base_path_ + "/get_fault");
  list_faults_client_ =
      node_->create_client<ros2_medkit_msgs::srv::ListFaults>(fault_manager_base_path_ + "/list_faults");
  clear_fault_client_ =
      node_->create_client<ros2_medkit_msgs::srv::ClearFault>(fault_manager_base_path_ + "/clear_fault");
  get_snapshots_client_ =
      node_->create_client<ros2_medkit_msgs::srv::GetSnapshots>(fault_manager_base_path_ + "/get_snapshots");
  get_rosbag_client_ = node_->create_client<ros2_medkit_msgs::srv::GetRosbag>(fault_manager_base_path_ + "/get_rosbag");
  list_rosbags_client_ =
      node_->create_client<ros2_medkit_msgs::srv::ListRosbags>(fault_manager_base_path_ + "/list_rosbags");

  RCLCPP_INFO(node_->get_logger(), "Ros2FaultServiceTransport initialized (base_path=%s, timeout=%.1fs)",
              fault_manager_base_path_.c_str(), service_timeout_sec_);
}

Ros2FaultServiceTransport::~Ros2FaultServiceTransport() {
  // Reset clients before implicit member destruction so any in-flight
  // future-callback paths drop their references in a defined order.
  report_fault_client_.reset();
  get_fault_client_.reset();
  list_faults_client_.reset();
  clear_fault_client_.reset();
  get_snapshots_client_.reset();
  get_rosbag_client_.reset();
  list_rosbags_client_.reset();
}

bool Ros2FaultServiceTransport::wait_for_services(std::chrono::duration<double> timeout) {
  return report_fault_client_->wait_for_service(timeout) && get_fault_client_->wait_for_service(timeout) &&
         list_faults_client_->wait_for_service(timeout) && clear_fault_client_->wait_for_service(timeout);
}

bool Ros2FaultServiceTransport::is_available() const {
  return report_fault_client_->service_is_ready() && get_fault_client_->service_is_ready() &&
         list_faults_client_->service_is_ready() && clear_fault_client_->service_is_ready();
}

FaultResult Ros2FaultServiceTransport::report_fault(const std::string & fault_code, uint8_t severity,
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

FaultResult Ros2FaultServiceTransport::list_faults(const std::string & source_id, bool include_prefailed,
                                                   bool include_confirmed, bool include_cleared, bool include_healed,
                                                   bool include_muted, bool include_clusters) {
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

  if (include_prefailed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_PREFAILED);
  }
  if (include_confirmed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED);
  }
  if (include_cleared) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_CLEARED);
  }
  if (include_healed) {
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_HEALED);
    request->statuses.push_back(ros2_medkit_msgs::msg::Fault::STATUS_PREPASSED);
  }

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
    if (!source_id.empty()) {
      const auto & sources = fault.reporting_sources;
      bool matches = false;
      for (const auto & src : sources) {
        if (src.rfind(source_id, 0) == 0) {
          matches = true;
          break;
        }
      }
      if (!matches) {
        continue;
      }
    }
    faults_array.push_back(conversions::fault_to_json(fault));
  }

  result.success = true;
  result.data = {{"faults", faults_array}, {"count", faults_array.size()}};
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

FaultWithEnvJsonResult Ros2FaultServiceTransport::get_fault_with_env(const std::string & fault_code,
                                                                     const std::string & source_id) {
  std::lock_guard<std::mutex> lock(get_mutex_);
  FaultWithEnvJsonResult result;

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
  if (!response->success) {
    result.success = false;
    result.error_message = response->error_message;
    return result;
  }

  // Verify source_id if provided (prefix match against any reporting source).
  if (!source_id.empty()) {
    bool matches = false;
    for (const auto & src : response->fault.reporting_sources) {
      if (src.rfind(source_id, 0) == 0) {
        matches = true;
        break;
      }
    }
    if (!matches) {
      result.success = false;
      result.error_message = "Fault not found for source: " + source_id;
      return result;
    }
  }

  result.success = true;
  result.data = {{"fault", conversions::fault_to_json(response->fault)},
                 {"environment_data", conversions::environment_data_to_json(response->environment_data)}};
  return result;
}

FaultResult Ros2FaultServiceTransport::get_fault(const std::string & fault_code, const std::string & source_id) {
  // Use get_fault_with_env and pull only the fault portion of the body.
  auto env_result = get_fault_with_env(fault_code, source_id);

  FaultResult result;
  result.success = env_result.success;
  result.error_message = env_result.error_message;

  if (env_result.success) {
    result.data = env_result.data["fault"];
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::clear_fault(const std::string & fault_code) {
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

  if (!response->auto_cleared_codes.empty()) {
    result.data["auto_cleared_codes"] = response->auto_cleared_codes;
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::get_snapshots(const std::string & fault_code, const std::string & topic) {
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
    try {
      result.data = json::parse(response->data);
    } catch (const json::exception & /*e*/) {
      result.data = {{"raw_data", response->data}};
    }
  } else {
    result.error_message = response->error_message;
  }

  return result;
}

FaultResult Ros2FaultServiceTransport::get_rosbag(const std::string & fault_code) {
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

FaultResult Ros2FaultServiceTransport::list_rosbags(const std::string & entity_fqn) {
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

}  // namespace ros2_medkit_gateway::ros2
