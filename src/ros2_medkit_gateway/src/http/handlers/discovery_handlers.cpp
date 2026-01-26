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

#include "ros2_medkit_gateway/http/handlers/discovery_handlers.hpp"

#include <map>
#include <set>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/capability_builder.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

// =============================================================================
// Area handlers
// =============================================================================

void DiscoveryHandlers::handle_list_areas(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    const auto areas = cache.get_areas();

    json items = json::array();
    for (const auto & area : areas) {
      json area_item;
      area_item["id"] = area.id;
      area_item["name"] = area.name.empty() ? area.id : area.name;
      area_item["href"] = "/api/v1/areas/" + area.id;

      if (!area.description.empty()) {
        area_item["description"] = area.description;
      }
      if (!area.tags.empty()) {
        area_item["tags"] = area.tags;
      }

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

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_areas: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_area(const httplib::Request & req, httplib::Response & res) {
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
    response["name"] = area.name.empty() ? area.id : area.name;

    if (!area.description.empty()) {
      response["description"] = area.description;
    }
    if (!area.tags.empty()) {
      response["tags"] = area.tags;
    }

    std::string base_uri = "/api/v1/areas/" + area.id;
    response["subareas"] = base_uri + "/subareas";
    response["components"] = base_uri + "/components";
    response["contains"] = base_uri + "/contains";
    response["data"] = base_uri + "/data";
    response["operations"] = base_uri + "/operations";
    response["configurations"] = base_uri + "/configurations";
    response["faults"] = base_uri + "/faults";

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::SUBAREAS, Cap::CONTAINS, Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS, Cap::FAULTS};
    response["capabilities"] = CapabilityBuilder::build_capabilities("areas", area.id, caps);

    LinksBuilder links;
    links.self("/api/v1/areas/" + area.id).collection("/api/v1/areas");
    if (!area.parent_area_id.empty()) {
      links.parent("/api/v1/areas/" + area.parent_area_id);
    }
    response["_links"] = links.build();

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

void DiscoveryHandlers::handle_area_components(const httplib::Request & req, httplib::Response & res) {
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

    const auto & cache = ctx_.node()->get_thread_safe_cache();

    if (!cache.has_area(area_id)) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Area not found",
                                 {{"area_id", area_id}});
      return;
    }

    const auto components = cache.get_components();
    json items = json::array();
    for (const auto & component : components) {
      if (component.area == area_id) {
        json comp_item;
        comp_item["id"] = component.id;
        comp_item["name"] = component.name.empty() ? component.id : component.name;
        comp_item["href"] = "/api/v1/components/" + component.id;

        if (!component.description.empty()) {
          comp_item["description"] = component.description;
        }

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

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_area_components: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_subareas(const httplib::Request & req, httplib::Response & res) {
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

    auto subareas = discovery->get_subareas(area_id);

    json items = json::array();
    for (const auto & subarea : subareas) {
      json item;
      item["id"] = subarea.id;
      item["name"] = subarea.name.empty() ? subarea.id : subarea.name;
      item["href"] = "/api/v1/areas/" + subarea.id;

      XMedkit ext;
      ext.ros2_namespace(subarea.namespace_path);
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

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

void DiscoveryHandlers::handle_get_contains(const httplib::Request & req, httplib::Response & res) {
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

    auto components = discovery->get_components_for_area(area_id);

    json items = json::array();
    for (const auto & comp : components) {
      json item;
      item["id"] = comp.id;
      item["name"] = comp.name.empty() ? comp.id : comp.name;
      item["href"] = "/api/v1/components/" + comp.id;

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

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

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

// =============================================================================
// Component handlers
// =============================================================================

void DiscoveryHandlers::handle_list_components(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    const auto components = cache.get_components();

    json items = json::array();
    for (const auto & component : components) {
      json item;
      item["id"] = component.id;
      item["name"] = component.name.empty() ? component.id : component.name;
      item["href"] = "/api/v1/components/" + component.id;

      if (!component.description.empty()) {
        item["description"] = component.description;
      }
      if (!component.tags.empty()) {
        item["tags"] = component.tags;
      }

      XMedkit ext;
      ext.source(component.source);
      if (!component.fqn.empty()) {
        ext.ros2_node(component.fqn);
      }
      if (!component.namespace_path.empty()) {
        ext.ros2_namespace(component.namespace_path);
      }
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_components: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_component(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto comp_opt = discovery->get_component(component_id);

    if (!comp_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    const auto & comp = *comp_opt;

    json response;
    response["id"] = comp.id;
    response["name"] = comp.name.empty() ? comp.id : comp.name;

    if (!comp.description.empty()) {
      response["description"] = comp.description;
    }
    if (!comp.tags.empty()) {
      response["tags"] = comp.tags;
    }

    std::string base = "/api/v1/components/" + comp.id;
    response["data"] = base + "/data";
    response["operations"] = base + "/operations";
    response["configurations"] = base + "/configurations";
    response["faults"] = base + "/faults";
    response["subcomponents"] = base + "/subcomponents";
    response["hosts"] = base + "/hosts";

    if (!comp.depends_on.empty()) {
      response["depends-on"] = base + "/depends-on";
    }

    if (!comp.area.empty()) {
      response["belongs-to"] = "/api/v1/areas/" + comp.area;
    }

    LinksBuilder links;
    links.self(base).collection("/api/v1/components");
    if (!comp.area.empty()) {
      links.add("area", "/api/v1/areas/" + comp.area);
    }
    if (!comp.parent_component_id.empty()) {
      links.parent("/api/v1/components/" + comp.parent_component_id);
    }
    response["_links"] = links.build();

    XMedkit ext;
    ext.source(comp.source);
    if (!comp.fqn.empty()) {
      ext.ros2_node(comp.fqn);
    }
    if (!comp.namespace_path.empty()) {
      ext.ros2_namespace(comp.namespace_path);
    }
    if (!comp.type.empty()) {
      ext.add("type", comp.type);
    }

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS, Cap::FAULTS, Cap::SUBCOMPONENTS, Cap::HOSTS};
    if (!comp.depends_on.empty()) {
      caps.push_back(Cap::DEPENDS_ON);
    }
    ext.add("capabilities", CapabilityBuilder::build_capabilities("components", comp.id, caps));
    response["x-medkit"] = ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_component: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_subcomponents(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto comp_opt = discovery->get_component(component_id);

    if (!comp_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    auto subcomponents = discovery->get_subcomponents(component_id);

    json items = json::array();
    for (const auto & sub : subcomponents) {
      json item;
      item["id"] = sub.id;
      item["name"] = sub.name.empty() ? sub.id : sub.name;
      item["href"] = "/api/v1/components/" + sub.id;

      XMedkit ext;
      ext.source(sub.source);
      if (!sub.namespace_path.empty()) {
        ext.ros2_namespace(sub.namespace_path);
      }
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/subcomponents";
    links["parent"] = "/api/v1/components/" + component_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_subcomponents: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_hosts(const httplib::Request & req, httplib::Response & res) {
  // @verifies REQ_INTEROP_007
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto comp_opt = discovery->get_component(component_id);

    if (!comp_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    auto apps = discovery->get_apps_for_component(component_id);

    json items = json::array();
    for (const auto & app : apps) {
      json item;
      item["id"] = app.id;
      item["name"] = app.name.empty() ? app.id : app.name;
      item["href"] = "/api/v1/apps/" + app.id;

      XMedkit ext;
      ext.is_online(app.is_online).source(app.source);
      if (app.bound_fqn) {
        ext.ros2_node(*app.bound_fqn);
      }
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/hosts";
    links["component"] = "/api/v1/components/" + component_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_hosts: %s", e.what());
  }
}

void DiscoveryHandlers::handle_component_depends_on(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto comp_opt = discovery->get_component(component_id);

    if (!comp_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    const auto & comp = *comp_opt;

    json items = json::array();
    for (const auto & dep_id : comp.depends_on) {
      json item;
      item["id"] = dep_id;
      item["href"] = "/api/v1/components/" + dep_id;

      auto dep_opt = discovery->get_component(dep_id);
      if (dep_opt) {
        item["name"] = dep_opt->name.empty() ? dep_id : dep_opt->name;

        XMedkit ext;
        ext.source(dep_opt->source);
        item["x-medkit"] = ext.build();
      } else {
        item["name"] = dep_id;
        XMedkit ext;
        ext.add("missing", true);
        item["x-medkit"] = ext.build();
        RCLCPP_WARN(HandlerContext::logger(), "Component '%s' declares dependency on unknown component '%s'",
                    component_id.c_str(), dep_id.c_str());
      }

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    json links;
    links["self"] = "/api/v1/components/" + component_id + "/depends-on";
    links["component"] = "/api/v1/components/" + component_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_component_depends_on: %s", e.what());
  }
}

// =============================================================================
// App handlers
// =============================================================================

void DiscoveryHandlers::handle_list_apps(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    auto discovery = ctx_.node()->get_discovery_manager();
    auto apps = discovery->discover_apps();

    json items = json::array();
    for (const auto & app : apps) {
      json app_item;
      app_item["id"] = app.id;
      app_item["name"] = app.name.empty() ? app.id : app.name;
      app_item["href"] = "/api/v1/apps/" + app.id;

      if (!app.description.empty()) {
        app_item["description"] = app.description;
      }
      if (!app.tags.empty()) {
        app_item["tags"] = app.tags;
      }

      XMedkit ext;
      ext.source(app.source).is_online(app.is_online);
      if (!app.component_id.empty()) {
        ext.component_id(app.component_id);
      }
      if (app.bound_fqn) {
        ext.ros2_node(*app.bound_fqn);
      }
      app_item["x-medkit"] = ext.build();

      items.push_back(app_item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_apps: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_app(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(app_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid app ID",
                                 {{"details", validation_result.error()}, {"app_id", app_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto app_opt = discovery->get_app(app_id);

    if (!app_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "App not found",
                                 {{"app_id", app_id}});
      return;
    }

    const auto & app = *app_opt;

    json response;
    response["id"] = app.id;
    response["name"] = app.name;

    if (!app.description.empty()) {
      response["description"] = app.description;
    }
    if (!app.translation_id.empty()) {
      response["translation_id"] = app.translation_id;
    }
    if (!app.tags.empty()) {
      response["tags"] = app.tags;
    }

    std::string base_uri = "/api/v1/apps/" + app.id;
    response["data"] = base_uri + "/data";
    response["operations"] = base_uri + "/operations";
    response["configurations"] = base_uri + "/configurations";
    response["faults"] = base_uri + "/faults";

    if (!app.component_id.empty()) {
      response["is-located-on"] = "/api/v1/components/" + app.component_id;
    }

    if (!app.depends_on.empty()) {
      response["depends-on"] = base_uri + "/depends-on";
    }

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS, Cap::FAULTS};
    response["capabilities"] = CapabilityBuilder::build_capabilities("apps", app.id, caps);

    LinksBuilder links;
    links.self("/api/v1/apps/" + app.id).collection("/api/v1/apps");
    if (!app.component_id.empty()) {
      links.add("is-located-on", "/api/v1/components/" + app.component_id);
    }
    response["_links"] = links.build();

    if (!app.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : app.depends_on) {
        depends_links.push_back("/api/v1/apps/" + dep_id);
      }
      response["_links"]["depends-on"] = depends_links;
    }

    XMedkit ext;
    ext.source(app.source).is_online(app.is_online);
    if (app.bound_fqn) {
      ext.ros2_node(*app.bound_fqn);
    }
    if (!app.component_id.empty()) {
      ext.component_id(app.component_id);
    }
    response["x-medkit"] = ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_app: %s", e.what());
  }
}

void DiscoveryHandlers::handle_app_depends_on(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(app_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid app ID",
                                 {{"details", validation_result.error()}, {"app_id", app_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto app_opt = discovery->get_app(app_id);

    if (!app_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "App not found",
                                 {{"app_id", app_id}});
      return;
    }

    const auto & app = *app_opt;

    json items = json::array();
    for (const auto & dep_id : app.depends_on) {
      json item;
      item["id"] = dep_id;
      item["href"] = "/api/v1/apps/" + dep_id;

      auto dep_opt = discovery->get_app(dep_id);
      if (dep_opt) {
        item["name"] = dep_opt->name.empty() ? dep_id : dep_opt->name;

        XMedkit ext;
        ext.source(dep_opt->source).is_online(dep_opt->is_online);
        item["x-medkit"] = ext.build();
      } else {
        item["name"] = dep_id;
        XMedkit ext;
        ext.add("missing", true);
        item["x-medkit"] = ext.build();
        RCLCPP_WARN(HandlerContext::logger(), "App '%s' declares dependency on unknown app '%s'", app_id.c_str(),
                    dep_id.c_str());
      }

      items.push_back(item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/depends-on";
    links["app"] = "/api/v1/apps/" + app_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_app_depends_on: %s", e.what());
  }
}

// =============================================================================
// Function handlers
// =============================================================================

void DiscoveryHandlers::handle_list_functions(const httplib::Request & req, httplib::Response & res) {
  (void)req;

  try {
    auto discovery = ctx_.node()->get_discovery_manager();
    auto functions = discovery->discover_functions();

    json items = json::array();
    for (const auto & func : functions) {
      json func_item;
      func_item["id"] = func.id;
      func_item["name"] = func.name.empty() ? func.id : func.name;
      func_item["href"] = "/api/v1/functions/" + func.id;

      if (!func.description.empty()) {
        func_item["description"] = func.description;
      }
      if (!func.tags.empty()) {
        func_item["tags"] = func.tags;
      }

      XMedkit ext;
      ext.source(func.source);
      func_item["x-medkit"] = ext.build();

      items.push_back(func_item);
    }

    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.add("total_count", functions.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_functions: %s", e.what());
  }
}

void DiscoveryHandlers::handle_get_function(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(function_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid function ID",
                                 {{"details", validation_result.error()}, {"function_id", function_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto func_opt = discovery->get_function(function_id);

    if (!func_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Function not found",
                                 {{"function_id", function_id}});
      return;
    }

    const auto & func = *func_opt;

    json response;
    response["id"] = func.id;
    response["name"] = func.name.empty() ? func.id : func.name;

    if (!func.description.empty()) {
      response["description"] = func.description;
    }
    if (!func.translation_id.empty()) {
      response["translation_id"] = func.translation_id;
    }
    if (!func.tags.empty()) {
      response["tags"] = func.tags;
    }

    std::string base_uri = "/api/v1/functions/" + func.id;
    response["hosts"] = base_uri + "/hosts";
    response["data"] = base_uri + "/data";
    response["operations"] = base_uri + "/operations";
    response["configurations"] = base_uri + "/configurations";
    response["faults"] = base_uri + "/faults";

    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::HOSTS, Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS, Cap::FAULTS};
    response["capabilities"] = CapabilityBuilder::build_capabilities("functions", func.id, caps);

    LinksBuilder links;
    links.self("/api/v1/functions/" + func.id).collection("/api/v1/functions");
    response["_links"] = links.build();

    if (!func.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : func.depends_on) {
        depends_links.push_back("/api/v1/functions/" + dep_id);
      }
      response["_links"]["depends-on"] = depends_links;
    }

    XMedkit ext;
    ext.source(func.source);
    response["x-medkit"] = ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_function: %s", e.what());
  }
}

void DiscoveryHandlers::handle_function_hosts(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(function_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid function ID",
                                 {{"details", validation_result.error()}, {"function_id", function_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto func_opt = discovery->get_function(function_id);

    if (!func_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Function not found",
                                 {{"function_id", function_id}});
      return;
    }

    auto host_ids = discovery->get_hosts_for_function(function_id);

    json items = json::array();
    for (const auto & app_id : host_ids) {
      auto app_opt = discovery->get_app(app_id);
      if (app_opt) {
        json item;
        item["id"] = app_opt->id;
        item["name"] = app_opt->name;
        item["href"] = "/api/v1/apps/" + app_opt->id;
        if (app_opt->is_online) {
          item["is_online"] = true;
        }
        items.push_back(item);
      }
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_function_hosts: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
