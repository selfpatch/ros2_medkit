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
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void AreaHandlers::handle_list_areas(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    const auto& cache = ctx_.node()->get_thread_safe_cache();
    const auto areas = cache.get_areas();

    // Build items array with EntityReference format
    json items = json::array();
    for (const auto & area : areas) {
      json area_item;
      // Required fields for EntityReference
      area_item["id"] = area.id;
      area_item["name"] = area.name.empty() ? area.id : area.name;
      area_item["href"] = "/api/v1/areas/" + area.id;

      // Optional fields
      if (!area.description.empty()) {
        area_item["description"] = area.description;
      }
      if (!area.tags.empty()) {
        area_item["tags"] = area.tags;
      }

      // x-medkit extension for ROS2-specific data
      XMedkit ext;
      ext.ros2_namespace(area.namespace_path);
      if (!area.parent_area_id.empty()) {
        ext.add("parent_area_id", area.parent_area_id);
      }
      area_item["x-medkit"] = ext.build();

      items.push_back(area_item);
    }

    json response;
    response["items"] = items;

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

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

    // Build response
    json response;
    response["id"] = area.id;
    response["name"] = area.name.empty() ? area.id : area.name;

    if (!area.description.empty()) {
      response["description"] = area.description;
    }
    if (!area.tags.empty()) {
      response["tags"] = area.tags;
    }

    // Capability URIs as flat fields at top level
    std::string base_uri = "/api/v1/areas/" + area.id;
    response["subareas"] = base_uri + "/subareas";
    response["components"] = base_uri + "/components";
    response["contains"] = base_uri + "/contains";  // SOVD 7.6.2.4

    // Build capabilities for areas
    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::SUBAREAS, Cap::CONTAINS};
    response["capabilities"] = CapabilityBuilder::build_capabilities("areas", area.id, caps);

    // Build HATEOAS links
    LinksBuilder links;
    links.self("/api/v1/areas/" + area.id).collection("/api/v1/areas");
    if (!area.parent_area_id.empty()) {
      links.parent("/api/v1/areas/" + area.parent_area_id);
    }
    response["_links"] = links.build();

    // x-medkit extension for ROS2-specific data
    XMedkit ext;
    ext.ros2_namespace(area.namespace_path);
    if (!area.parent_area_id.empty()) {
      ext.add("parent_area_id", area.parent_area_id);
    }
    response["x-medkit"] = ext.build();

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

    const auto& cache = ctx_.node()->get_thread_safe_cache();

    // Check if area exists (O(1) lookup)
    if (!cache.has_area(area_id)) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Area not found",
                                 {{"area_id", area_id}});
      return;
    }

    // Filter components by area
    const auto components = cache.get_components();
    json items = json::array();
    for (const auto & component : components) {
      if (component.area == area_id) {
        json comp_item;
        // SOVD required fields for EntityReference
        comp_item["id"] = component.id;
        comp_item["name"] = component.name.empty() ? component.id : component.name;
        comp_item["href"] = "/api/v1/components/" + component.id;

        // Optional SOVD fields
        if (!component.description.empty()) {
          comp_item["description"] = component.description;
        }

        // x-medkit extension for ROS2-specific data
        XMedkit ext;
        ext.source(component.source);
        if (!component.namespace_path.empty()) {
          ext.ros2_namespace(component.namespace_path);
        }
        comp_item["x-medkit"] = ext.build();

        items.push_back(comp_item);
      }
    }

    json response;
    response["items"] = items;

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

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
      item["name"] = subarea.name.empty() ? subarea.id : subarea.name;
      item["href"] = "/api/v1/areas/" + subarea.id;

      // x-medkit extension for ROS2-specific data
      XMedkit ext;
      ext.ros2_namespace(subarea.namespace_path);
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

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

void AreaHandlers::handle_get_contains(const httplib::Request & req, httplib::Response & res) {
  // @verifies REQ_INTEROP_006
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

    // Get components for this area (SOVD 7.6.2.4 - non-deprecated relationship)
    auto components = discovery->get_components_for_area(area_id);

    json items = json::array();
    for (const auto & comp : components) {
      json item;
      item["id"] = comp.id;
      item["name"] = comp.name.empty() ? comp.id : comp.name;
      item["href"] = "/api/v1/components/" + comp.id;

      // x-medkit extension for ROS2-specific data
      XMedkit ext;
      ext.source(comp.source);
      if (!comp.namespace_path.empty()) {
        ext.ros2_namespace(comp.namespace_path);
      }
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    // HATEOAS links
    json links;
    links["self"] = "/api/v1/areas/" + area_id + "/contains";
    links["area"] = "/api/v1/areas/" + area_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_contains: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
