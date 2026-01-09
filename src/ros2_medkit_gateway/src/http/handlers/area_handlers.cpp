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

#include "ros2_medkit_gateway/http/handlers/area_handlers.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void AreaHandlers::handle_list_areas(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    const auto cache = ctx_.node()->get_entity_cache();

    json areas_json = json::array();
    for (const auto & area : cache.areas) {
      areas_json.push_back(area.to_json());
    }

    HandlerContext::send_json(res, areas_json);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_areas: %s", e.what());
  }
}

void AreaHandlers::handle_area_components(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract area_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    // Validate area_id
    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid area ID",
                                 {{"details", validation_result.error()}, {"area_id", area_id}});
      return;
    }

    const auto cache = ctx_.node()->get_entity_cache();

    // Check if area exists
    bool area_exists = false;
    for (const auto & area : cache.areas) {
      if (area.id == area_id) {
        area_exists = true;
        break;
      }
    }

    if (!area_exists) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, "Area not found", {{"area_id", area_id}});
      return;
    }

    // Filter components by area
    json components_json = json::array();
    for (const auto & component : cache.components) {
      if (component.area == area_id) {
        components_json.push_back(component.to_json());
      }
    }

    HandlerContext::send_json(res, components_json);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_area_components: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
