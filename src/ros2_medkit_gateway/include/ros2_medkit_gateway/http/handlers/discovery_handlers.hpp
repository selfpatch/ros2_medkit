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

#pragma once

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Unified handlers for entity discovery REST API endpoints.
 *
 * Consolidated handlers for discovering and navigating SOVD entities:
 * - Areas: ROS 2 namespace groupings
 * - Components: Hardware or logical groupings
 * - Apps: Software applications (ROS nodes)
 * - Functions: Capability abstractions
 *
 * @verifies REQ_DISCOVERY_001 Areas discovery
 * @verifies REQ_DISCOVERY_002 Apps discovery
 * @verifies REQ_DISCOVERY_003 Components discovery
 * @verifies REQ_DISCOVERY_004 Functions discovery
 */
class DiscoveryHandlers {
 public:
  /**
   * @brief Construct discovery handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit DiscoveryHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  // =========================================================================
  // Area endpoints
  // =========================================================================

  /**
   * @brief Handle GET /areas - list all areas.
   */
  void handle_list_areas(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /areas/{area-id} - get area capabilities.
   */
  void handle_get_area(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /areas/{area-id}/components - list components in area.
   */
  void handle_area_components(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /areas/{area-id}/subareas - list subareas.
   */
  void handle_get_subareas(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /areas/{area-id}/contains - SOVD 7.6.2.4 relationship.
   * @verifies REQ_INTEROP_006
   */
  void handle_get_contains(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // Component endpoints
  // =========================================================================

  /**
   * @brief Handle GET /components - list all components.
   */
  void handle_list_components(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{component-id} - get component capabilities.
   */
  void handle_get_component(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{id}/subcomponents - list subcomponents.
   */
  void handle_get_subcomponents(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{id}/hosts - list hosted apps.
   * @verifies REQ_INTEROP_007
   */
  void handle_get_hosts(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /components/{id}/depends-on - list dependencies.
   * @verifies REQ_INTEROP_008
   */
  void handle_component_depends_on(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // App endpoints
  // =========================================================================

  /**
   * @brief Handle GET /apps - list all apps.
   */
  void handle_list_apps(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /apps/{app-id} - get app capabilities.
   */
  void handle_get_app(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /apps/{app-id}/depends-on - list app dependencies.
   * @verifies REQ_INTEROP_009
   */
  void handle_app_depends_on(const httplib::Request & req, httplib::Response & res);

  // =========================================================================
  // Function endpoints
  // =========================================================================

  /**
   * @brief Handle GET /functions - list all functions.
   */
  void handle_list_functions(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /functions/{function-id} - get function capabilities.
   */
  void handle_get_function(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /functions/{function-id}/hosts - list host apps.
   */
  void handle_function_hosts(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
