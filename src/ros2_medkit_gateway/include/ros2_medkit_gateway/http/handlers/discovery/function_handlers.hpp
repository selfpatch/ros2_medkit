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

#ifndef ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__FUNCTION_HANDLERS_HPP_
#define ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__FUNCTION_HANDLERS_HPP_

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for function-related REST API endpoints.
 *
 * Functions are high-level capability groupings (e.g., "navigation",
 * "localization") that may be hosted by multiple apps.
 *
 * Provides handlers for:
 * - GET /functions - List all functions
 * - GET /functions/{function-id} - Get function capabilities
 * - GET /functions/{function-id}/hosts - Get apps that host this function
 * - GET /functions/{function-id}/data - Get aggregated function data
 * - GET /functions/{function-id}/operations - List function operations
 *
 * @verifies REQ_DISCOVERY_003 Functions discovery
 */
class FunctionHandlers {
 public:
  /**
   * @brief Construct function handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit FunctionHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  // =========================================================================
  // Collection endpoints
  // =========================================================================

  /**
   * @brief Handle GET /functions - list all functions.
   *
   * Returns all functions discovered from manifest.
   * In runtime-only mode, returns empty list.
   */
  void handle_list_functions(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /functions/{function-id} - get function capabilities.
   *
   * Returns function details with capabilities (hosts, data, operations)
   * and HATEOAS links.
   */
  void handle_get_function(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // Function hosts
  // =========================================================================

  /**
   * @brief Handle GET /functions/{function-id}/hosts - get hosts for function.
   *
   * Returns apps that provide this function.
   */
  void handle_function_hosts(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // Function data & operations (aggregated from host apps)
  // =========================================================================

  /**
   * @brief Handle GET /functions/{function-id}/data - get aggregated data.
   *
   * Returns topics aggregated from all host apps.
   */
  void handle_get_function_data(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /functions/{function-id}/operations - list operations.
   *
   * Returns services and actions aggregated from all host apps.
   */
  void handle_list_function_operations(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__FUNCTION_HANDLERS_HPP_
