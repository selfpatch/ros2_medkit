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
 * @brief Handlers for configuration (parameter) REST API endpoints.
 *
 * Provides handlers for:
 * - GET /components/{component_id}/configurations - List all parameters
 * - GET /components/{component_id}/configurations/{param_name} - Get parameter
 * - PUT /components/{component_id}/configurations/{param_name} - Set parameter
 * - DELETE /components/{component_id}/configurations/{param_name} - Reset parameter
 * - DELETE /components/{component_id}/configurations - Reset all parameters
 */
class ConfigHandlers {
 public:
  /**
   * @brief Construct configuration handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit ConfigHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /components/{component_id}/configurations - list all parameters.
   */
  void handle_list_configurations(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component_id}/configurations/{param_name}.
   */
  void handle_get_configuration(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle PUT /components/{component_id}/configurations/{param_name}.
   */
  void handle_set_configuration(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /components/{component_id}/configurations/{param_name}.
   */
  void handle_delete_configuration(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle DELETE /components/{component_id}/configurations - reset all.
   */
  void handle_delete_all_configurations(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

