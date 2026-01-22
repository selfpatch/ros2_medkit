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

#ifndef ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__OPERATION_HANDLERS_HPP_
#define ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__OPERATION_HANDLERS_HPP_

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for operation-related REST API endpoints (services and actions).
 *
 * Provides handlers for:
 * - GET /components/{component_id}/operations - List all operations
 * - POST /components/{component_id}/operations/{operation_name} - Execute operation
 * - GET /components/{component_id}/operations/{operation_name}/status - Get action status
 * - GET /components/{component_id}/operations/{operation_name}/result - Get action result
 * - DELETE /components/{component_id}/operations/{operation_name} - Cancel action
 */
class OperationHandlers {
 public:
  /**
   * @brief Construct operation handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit OperationHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /components/{component_id}/operations - list all operations.
   */
  void handle_list_operations(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /{entity}/operations/{op-id} - get operation details.
   */
  void handle_get_operation(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle POST /components/{component_id}/operations/{operation_name} - execute.
   * @deprecated Use handle_create_execution for SOVD-compliant execution via /executions path.
   */
  void handle_component_operation(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle POST /{entity}/operations/{op-id}/executions - SOVD-compliant execution start.
   */
  void handle_create_execution(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /{entity}/operations/{op-id}/executions - list executions.
   */
  void handle_list_executions(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /{entity}/operations/{op-id}/executions/{exec-id} - execution status.
   */
  void handle_get_execution(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /{entity}/operations/{op-id}/executions/{exec-id} - cancel execution.
   */
  void handle_cancel_execution(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/operations/{operation_name}/status.
   */
  void handle_action_status(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/operations/{operation_name}/result.
   */
  void handle_action_result(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /components/{component_id}/operations/{operation_name}.
   */
  void handle_action_cancel(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__OPERATION_HANDLERS_HPP_
