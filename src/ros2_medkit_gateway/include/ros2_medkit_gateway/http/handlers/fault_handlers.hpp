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

#ifndef ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__FAULT_HANDLERS_HPP_
#define ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__FAULT_HANDLERS_HPP_

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for fault management REST API endpoints.
 *
 * Provides handlers for:
 * - GET /faults - List all faults
 * - GET /faults/{fault_code}/snapshots - Get snapshots for a fault
 * - GET /components/{component_id}/faults - List faults for a component
 * - GET /components/{component_id}/faults/{fault_code} - Get specific fault
 * - GET /components/{component_id}/faults/{fault_code}/snapshots - Get snapshots for a fault
 * - DELETE /components/{component_id}/faults/{fault_code} - Clear fault
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
   * @brief Handle GET /faults/{fault_code}/snapshots - get snapshots for a fault (system-wide).
   */
  void handle_get_snapshots(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/faults/{fault_code}/snapshots - get snapshots for a fault.
   */
  void handle_get_component_snapshots(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /faults/{fault_code}/snapshots/bag - download rosbag file for a fault.
   */
  void handle_get_rosbag(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__FAULT_HANDLERS_HPP_
