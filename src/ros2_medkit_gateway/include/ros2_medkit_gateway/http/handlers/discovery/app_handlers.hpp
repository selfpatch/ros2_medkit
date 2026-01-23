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

#ifndef ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__APP_HANDLERS_HPP_
#define ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__APP_HANDLERS_HPP_

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for app-related REST API endpoints.
 *
 * Apps represent software applications (typically 1:1 with ROS nodes)
 * that provide functionality within the system.
 *
 * Provides handlers for:
 * - GET /apps - List all apps
 * - GET /apps/{app-id} - Get app capabilities
 * - GET /apps/{app-id}/data - Get app topic data
 * - GET /apps/{app-id}/data/{data-id} - Get specific data item
 * - GET /apps/{app-id}/operations - List app operations
 * - GET /apps/{app-id}/configurations - Get app configurations
 * - GET /components/{id}/related-apps - List apps on component
 *
 * @verifies REQ_DISCOVERY_002 Apps discovery
 */
class AppHandlers {
 public:
  /**
   * @brief Construct app handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit AppHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  // =========================================================================
  // Collection endpoints
  // =========================================================================

  /**
   * @brief Handle GET /apps - list all apps.
   *
   * Returns all apps discovered from manifest or runtime.
   * In runtime-only mode, returns empty list.
   */
  void handle_list_apps(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /apps/{app-id} - get app capabilities.
   *
   * Returns app details with capabilities (data, operations, configurations)
   * and HATEOAS links.
   */
  void handle_get_app(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // App data endpoints
  // =========================================================================

  /**
   * @brief Handle GET /apps/{app-id}/data - list app topics.
   *
   * Returns all topics associated with the app.
   */
  void handle_get_app_data(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /apps/{app-id}/data/{data-id} - get specific data item.
   *
   * Returns metadata for a specific topic.
   */
  void handle_get_app_data_item(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // App operations
  // =========================================================================

  /**
   * @brief Handle GET /apps/{app-id}/operations - list app operations.
   *
   * Returns all services and actions associated with the app.
   */
  void handle_list_app_operations(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // App configurations
  // =========================================================================

  /**
   * @brief Handle GET /apps/{app-id}/configurations - list app parameters.
   *
   * Returns parameters if app is linked to a runtime node.
   */
  void handle_list_app_configurations(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // Relationship endpoints
  // =========================================================================

  /**
   * @brief Handle GET /components/{id}/related-apps - list apps on component.
   */
  void handle_related_apps(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /apps/{app-id}/depends-on - list app dependencies.
   *
   * Returns list of other apps that this app depends on.
   *
   * @verifies REQ_INTEROP_009
   */
  void handle_get_depends_on(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__DISCOVERY__APP_HANDLERS_HPP_
