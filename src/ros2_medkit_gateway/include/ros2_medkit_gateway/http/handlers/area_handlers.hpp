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

#ifndef ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__AREA_HANDLERS_HPP_
#define ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__AREA_HANDLERS_HPP_

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/**
 * @brief Handlers for area-related REST API endpoints.
 *
 * Provides handlers for:
 * - GET /areas - List all areas
 * - GET /areas/{area_id}/components - List components in an area
 */
class AreaHandlers {
 public:
  /**
   * @brief Construct area handlers with shared context.
   * @param ctx The shared handler context
   */
  explicit AreaHandlers(HandlerContext & ctx) : ctx_(ctx) {
  }

  /**
   * @brief Handle GET /areas - list all discovery areas.
   */
  void handle_list_areas(const httplib::Request & req, httplib::Response & res);

  /**
   * @brief Handle GET /areas/{area_id}/components - list components in area.
   */
  void handle_area_components(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway

#endif  // ROS2_MEDKIT_GATEWAY__HTTP__HANDLERS__AREA_HANDLERS_HPP_
