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

#include "ros2_medkit_gateway/http/handlers/discovery/area_handlers.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/capability_builder.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void AreaHandlers::handle_list_areas(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    const auto cache = ctx_.node()->get_entity_cache();

    json items = json::array();
    for (const auto & area : cache.areas) {
      items.push_back(area.to_json());
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_areas: %s", e.what());
  }
}

void AreaHandlers::handle_get_area(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid area ID",
                                 {{"details", validation_result.error()}, {"area_id", area_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto area_opt = discovery->get_area(area_id);

    if (!area_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Area not found",
                                 {{"area_id", area_id}});
      return;
    }

    const auto & area = *area_opt;

    json response;
    response["id"] = area.id;
    response["name"] = area.name;
    response["type"] = area.type;

    if (!area.description.empty()) {
      response["description"] = area.description;
    }

    // Build capabilities for areas
    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::SUBAREAS, Cap::RELATED_COMPONENTS};
    response["capabilities"] = CapabilityBuilder::build_capabilities("areas", area.id, caps);

    // Build HATEOAS links
    LinksBuilder links;
    links.self("/api/v1/areas/" + area.id).collection("/api/v1/areas");
    if (!area.parent_area_id.empty()) {
      links.parent("/api/v1/areas/" + area.parent_area_id);
    }
    response["_links"] = links.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_area: %s", e.what());
  }
}

void AreaHandlers::handle_area_components(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract area_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    // Validate area_id
    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid area ID",
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
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Area not found",
                                 {{"area_id", area_id}});
      return;
    }

    // Filter components by area
    json items = json::array();
    for (const auto & component : cache.components) {
      if (component.area == area_id) {
        items.push_back(component.to_json());
      }
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_area_components: %s", e.what());
  }
}

void AreaHandlers::handle_get_subareas(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid area ID",
                                 {{"details", validation_result.error()}, {"area_id", area_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto area_opt = discovery->get_area(area_id);

    if (!area_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Area not found",
                                 {{"area_id", area_id}});
      return;
    }

    // Get subareas
    auto subareas = discovery->get_subareas(area_id);

    json items = json::array();
    for (const auto & subarea : subareas) {
      json item;
      item["id"] = subarea.id;
      item["name"] = subarea.name;
      item["href"] = "/api/v1/areas/" + subarea.id;
      items.push_back(item);
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    // HATEOAS links
    json links;
    links["self"] = "/api/v1/areas/" + area_id + "/subareas";
    links["parent"] = "/api/v1/areas/" + area_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_subareas: %s", e.what());
  }
}

void AreaHandlers::handle_get_related_components(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string area_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(area_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid area ID",
                                 {{"details", validation_result.error()}, {"area_id", area_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto area_opt = discovery->get_area(area_id);

    if (!area_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Area not found",
                                 {{"area_id", area_id}});
      return;
    }

    // Get components for this area
    auto components = discovery->get_components_for_area(area_id);

    json items = json::array();
    for (const auto & comp : components) {
      json item;
      item["id"] = comp.id;
      item["name"] = comp.name;
      item["href"] = "/api/v1/components/" + comp.id;
      items.push_back(item);
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    // HATEOAS links
    json links;
    links["self"] = "/api/v1/areas/" + area_id + "/related-components";
    links["area"] = "/api/v1/areas/" + area_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_related_components: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
