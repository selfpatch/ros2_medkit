// Copyright 2025 selfpatch
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

#ifndef ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__APP_HANDLERS_HPP_
#define ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__APP_HANDLERS_HPP_

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for app-related REST API endpoints.
 *
 * Provides handlers for:
 * - GET /apps - List all apps
 * - GET /apps/{app-id} - Get app capabilities
 * - GET /apps/{app-id}/data - Get app data
 * - GET /apps/{app-id}/operations - Get app operations
 * - GET /apps/{app-id}/configurations - Get app configurations
 * - GET /components/{id}/related-apps - List apps on component
 *
 * @note TODO: Implement in TASK_007
 */
class AppHandlers {
 public:
  /**
   * @brief Construct app handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit AppHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /apps - list all apps.
   *
   * @note TODO: Implement in TASK_007
   */
  void handle_list_apps(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /apps/{app-id} - get app capabilities.
   *
   * @note TODO: Implement in TASK_007
   */
  void handle_get_app(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{id}/related-apps - list apps on component.
   *
   * @note TODO: Implement in TASK_009
   */
  void handle_related_apps(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__APP_HANDLERS_HPP_
