// Copyright 2025 bburda
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

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_msgs/msg/environment_data.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for fault management REST API endpoints.
 *
 * Provides handlers for:
 * - GET /faults - List all faults
 * - GET /components/{component_id}/faults - List faults for a component
 * - GET /components/{component_id}/faults/{fault_code} - Get specific fault
 * - DELETE /components/{component_id}/faults/{fault_code} - Clear fault
 *
 * Note: Snapshot data is inline in fault responses (environment_data).
 * Rosbag downloads use the bulk-data endpoint pattern.
 */
class FaultHandlers {
 public:
  /**
   * @brief Construct fault handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit FaultHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /faults - list all faults.
   */
  void handle_list_all_faults(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/faults - list faults for component.
   */
  void handle_list_faults(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/faults/{fault_code} - get specific fault.
   */
  void handle_get_fault(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /components/{component_id}/faults/{fault_code} - clear fault.
   */
  void handle_clear_fault(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /components/{component_id}/faults - clear all faults for entity.
   */
  void handle_clear_all_faults(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Build SOVD-compliant fault response with environment data.
   *
   * Creates a response containing:
   * - "item" with fault details and SOVD status object
   * - "environment_data" with extended_data_records and snapshots
   * - "x-medkit" extensions with occurrence_count, severity_label, etc.
   *
   * @param fault The fault message from fault_manager
   * @param env_data Environment data with snapshots
   * @param entity_path Entity path for bulk_data_uri generation (e.g., "/apps/motor")
   * @return SOVD-compliant JSON response
   */
  static nlohmann::json build_sovd_fault_response(const ros2_medkit_msgs::msg::Fault & fault,
                                                  const ros2_medkit_msgs::msg::EnvironmentData & env_data,
                                                  const std::string & entity_path);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

