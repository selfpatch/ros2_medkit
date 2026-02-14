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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for operation-related REST API endpoints (services and actions).
 *
 * Handlers:
 * - GET /{entity}/operations - List all operations (7.14.3)
 * - GET /{entity}/operations/{op-id} - Get operation details (7.14.4)
 * - GET /{entity}/operations/{op-id}/executions - List executions (7.14.5)
 * - POST /{entity}/operations/{op-id}/executions - Start execution (7.14.6)
 * - GET /{entity}/operations/{op-id}/executions/{exec-id} - Get execution status (7.14.7)
 * - PUT /{entity}/operations/{op-id}/executions/{exec-id} - Update execution (7.14.9)
 * - DELETE /{entity}/operations/{op-id}/executions/{exec-id} - Terminate execution (7.14.8)
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
   * @brief Handle POST /{entity}/operations/{op-id}/executions - execution start.
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
   * @brief Handle PUT /{entity}/operations/{op-id}/executions/{exec-id} - update execution.
   *
   * Executes the given capability on the provided operation execution.
   * Supported capabilities for ROS 2 actions: stop (maps to cancel).
   * Unsupported: execute (re-execute), freeze, reset (I/O control specific).
   */
  void handle_update_execution(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

