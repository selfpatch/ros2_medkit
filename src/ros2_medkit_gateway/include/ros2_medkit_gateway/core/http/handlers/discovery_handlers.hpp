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

#include "ros2_medkit_gateway/dto/entities.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

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
 * All 18 routes follow the PR-403 typed RouteRegistry convention:
 *
 *   http::Result<dto::TResponse> get_X(const http::TypedRequest & req);
 *
 * The framework owns the cpp-httplib response object - handlers never touch
 * it. Errors are returned as `tl::unexpected(ErrorInfo)` and the framework
 * renders them via the SOVD GenericError schema. When the typed entity
 * validator returns Forwarded (the request was proxied to a remote peer in
 * an aggregation setup), the handler propagates that outcome by returning
 * `tl::unexpected(HandlerContext::forwarded_sentinel_error())` so the typed
 * wrapper performs no further wire writes.
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

  /// GET /areas - list all top-level areas.
  http::Result<dto::Collection<dto::AreaListItem>> get_areas(const http::TypedRequest & req);

  /// GET /areas/{area-id} - area detail with capabilities.
  http::Result<dto::AreaDetail> get_area(const http::TypedRequest & req);

  /// GET /areas/{area-id}/components - list components in area.
  http::Result<dto::Collection<dto::ComponentListItem>> get_area_components(const http::TypedRequest & req);

  /// GET /areas/{area-id}/subareas - list subareas.
  http::Result<dto::Collection<dto::AreaListItem>> get_subareas(const http::TypedRequest & req);

  /**
   * @brief GET /areas/{area-id}/contains - SOVD 7.6.2.4 relationship.
   * @verifies REQ_INTEROP_006
   */
  http::Result<dto::Collection<dto::ComponentListItem>> get_area_contains(const http::TypedRequest & req);

  // =========================================================================
  // Component endpoints
  // =========================================================================

  /// GET /components - list all top-level components.
  http::Result<dto::Collection<dto::ComponentListItem>> get_components(const http::TypedRequest & req);

  /// GET /components/{component-id} - component detail with capabilities.
  http::Result<dto::ComponentDetail> get_component(const http::TypedRequest & req);

  /// GET /components/{id}/subcomponents - list subcomponents.
  http::Result<dto::Collection<dto::ComponentListItem>> get_subcomponents(const http::TypedRequest & req);

  /**
   * @brief GET /components/{id}/hosts - list hosted apps.
   * @verifies REQ_INTEROP_007
   */
  http::Result<dto::Collection<dto::AppListItem>> get_component_hosts(const http::TypedRequest & req);

  /**
   * @brief GET /components/{id}/depends-on - list dependencies.
   * @verifies REQ_INTEROP_008
   */
  http::Result<dto::Collection<dto::ComponentListItem>> get_component_depends_on(const http::TypedRequest & req);

  // =========================================================================
  // App endpoints
  // =========================================================================

  /// GET /apps - list all apps.
  http::Result<dto::Collection<dto::AppListItem>> get_apps(const http::TypedRequest & req);

  /// GET /apps/{app-id} - app detail with capabilities.
  http::Result<dto::AppDetail> get_app(const http::TypedRequest & req);

  /**
   * @brief GET /apps/{app-id}/depends-on - list app dependencies.
   * @verifies REQ_INTEROP_009
   */
  http::Result<dto::Collection<dto::AppListItem>> get_app_depends_on(const http::TypedRequest & req);

  /// GET /apps/{app-id}/is-located-on - parent component (single-item collection).
  http::Result<dto::Collection<dto::ComponentListItem>> get_app_is_located_on(const http::TypedRequest & req);

  /**
   * @brief GET /apps/{app-id}/belongs-to - parent area (single-item collection).
   * @verifies REQ_INTEROP_106
   */
  http::Result<dto::Collection<dto::AreaListItem>> get_app_belongs_to(const http::TypedRequest & req);

  // =========================================================================
  // Function endpoints
  // =========================================================================

  /// GET /functions - list all functions.
  http::Result<dto::Collection<dto::FunctionListItem>> get_functions(const http::TypedRequest & req);

  /// GET /functions/{function-id} - function detail with capabilities.
  http::Result<dto::FunctionDetail> get_function(const http::TypedRequest & req);

  /// GET /functions/{function-id}/hosts - list host apps.
  http::Result<dto::Collection<dto::AppListItem>> get_function_hosts(const http::TypedRequest & req);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
