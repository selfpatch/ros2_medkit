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

#include "ros2_medkit_gateway/http/handlers/discovery/app_handlers.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/capability_builder.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void AppHandlers::handle_list_apps(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    auto discovery = ctx_.node()->get_discovery_manager();
    auto apps = discovery->discover_apps();

    // Build SOVD-compliant items array with EntityReference format
    json items = json::array();
    for (const auto & app : apps) {
      json app_item;
      // SOVD required fields for EntityReference
      app_item["id"] = app.id;
      app_item["name"] = app.name.empty() ? app.id : app.name;
      app_item["href"] = "/api/v1/apps/" + app.id;

      // Optional SOVD fields
      if (!app.description.empty()) {
        app_item["description"] = app.description;
      }
      if (!app.tags.empty()) {
        app_item["tags"] = app.tags;
      }

      // x-medkit extension for ROS2-specific data
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

    // x-medkit for response-level metadata
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

void AppHandlers::handle_get_app(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract app_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];

    // Validate app_id
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

    // Build response with SOVD-compliant structure
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

    // SOVD capability URIs as flat fields at top level
    std::string base_uri = "/api/v1/apps/" + app.id;
    response["data"] = base_uri + "/data";
    response["operations"] = base_uri + "/operations";
    response["configurations"] = base_uri + "/configurations";
    // Apps don't have faults/subcomponents in SOVD model

    // Build capabilities using CapabilityBuilder (for capability introspection)
    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::DATA, Cap::OPERATIONS, Cap::CONFIGURATIONS};
    response["capabilities"] = CapabilityBuilder::build_capabilities("apps", app.id, caps);

    // Build HATEOAS links using LinksBuilder
    LinksBuilder links;
    links.self("/api/v1/apps/" + app.id).collection("/api/v1/apps");
    if (!app.component_id.empty()) {
      links.add("is-located-on", "/api/v1/components/" + app.component_id);
    }
    response["_links"] = links.build();

    // Add depends-on as array if present (special case - multiple links)
    if (!app.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : app.depends_on) {
        depends_links.push_back("/api/v1/apps/" + dep_id);
      }
      response["_links"]["depends-on"] = depends_links;
    }

    // x-medkit extension for ROS2-specific data
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

void AppHandlers::handle_get_app_data(const httplib::Request & req, httplib::Response & res) {
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

    // Build SOVD-compliant data items from app's topics
    json items = json::array();

    // Publishers - SOVD category "currentData"
    for (const auto & topic_name : app.topics.publishes) {
      json item;
      // SOVD required fields
      item["id"] = normalize_topic_to_id(topic_name);
      item["name"] = topic_name;  // Use topic name as display name
      item["category"] = "currentData";

      // x-medkit extension for ROS2-specific data
      XMedkit ext;
      ext.ros2_topic(topic_name).add_ros2("direction", "publish");
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    // Subscribers - SOVD category "currentData"
    for (const auto & topic_name : app.topics.subscribes) {
      json item;
      // SOVD required fields
      item["id"] = normalize_topic_to_id(topic_name);
      item["name"] = topic_name;
      item["category"] = "currentData";

      // x-medkit extension for ROS2-specific data
      XMedkit ext;
      ext.ros2_topic(topic_name).add_ros2("direction", "subscribe");
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    // Build response with x-medkit for total_count
    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.entity_id(app_id).add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_app_data: %s", e.what());
  }
}

void AppHandlers::handle_get_app_data_item(const httplib::Request & req, httplib::Response & res) {
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string app_id = req.matches[1];
    std::string data_id = req.matches[2];

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

    // Get data access manager for topic sampling
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto native_sampler = data_access_mgr->get_native_sampler();

    // Helper lambda to build SOVD ReadValue response
    auto build_data_response = [&](const std::string & topic_name, const std::string & direction) {
      json response;
      // SOVD required fields
      response["id"] = normalize_topic_to_id(topic_name);

      // Sample topic value via native sampler
      auto sample = native_sampler->sample_topic(topic_name, data_access_mgr->get_topic_sample_timeout());

      // SOVD "data" field contains the actual value
      if (sample.has_data && sample.data) {
        response["data"] = *sample.data;
      } else {
        response["data"] = json::object();  // Empty object if no data available
      }

      // Build x-medkit extension with ROS2-specific data
      XMedkit ext;
      ext.ros2_topic(topic_name).add_ros2("direction", direction);
      if (!sample.message_type.empty()) {
        ext.ros2_type(sample.message_type);
      }
      ext.add("timestamp", sample.timestamp_ns);
      ext.add("publisher_count", sample.publisher_count);
      ext.add("subscriber_count", sample.subscriber_count);
      ext.add("status", sample.has_data ? "data" : "metadata_only");
      response["x-medkit"] = ext.build();

      return response;
    };

    // Try matching by normalized ID or original topic name
    // Search in publishers
    for (const auto & topic_name : app.topics.publishes) {
      if (normalize_topic_to_id(topic_name) == data_id || topic_name == data_id) {
        HandlerContext::send_json(res, build_data_response(topic_name, "publish"));
        return;
      }
    }

    // Search in subscribers
    for (const auto & topic_name : app.topics.subscribes) {
      if (normalize_topic_to_id(topic_name) == data_id || topic_name == data_id) {
        HandlerContext::send_json(res, build_data_response(topic_name, "subscribe"));
        return;
      }
    }

    HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_RESOURCE_NOT_FOUND, "Data item not found",
                               {{"app_id", app_id}, {"data_id", data_id}});
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_app_data_item: %s", e.what());
  }
}

void AppHandlers::handle_list_app_operations(const httplib::Request & req, httplib::Response & res) {
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

    // Add services
    for (const auto & svc : app.services) {
      json item;
      item["id"] = svc.name;
      item["name"] = svc.name;
      item["type"] = "service";
      item["service_type"] = svc.type;
      items.push_back(item);
    }

    // Add actions
    for (const auto & act : app.actions) {
      json item;
      item["id"] = act.name;
      item["name"] = act.name;
      item["type"] = "action";
      item["action_type"] = act.type;
      items.push_back(item);
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_app_operations: %s", e.what());
  }
}

void AppHandlers::handle_list_app_configurations(const httplib::Request & req, httplib::Response & res) {
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
    json items = json::array();

    // If app is linked to a runtime node, fetch parameters from it
    if (app.bound_fqn) {
      auto config_mgr = ctx_.node()->get_configuration_manager();
      auto result = config_mgr->list_parameters(*app.bound_fqn);

      if (result.success && result.data.is_array()) {
        for (const auto & param : result.data) {
          json item;
          item["id"] = param["name"];
          item["name"] = param["name"];
          item["value"] = param["value"];
          item["type"] = param["type"];
          items.push_back(item);
        }
      }
      response["bound_node"] = *app.bound_fqn;
    }

    response["items"] = items;
    response["total_count"] = items.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_app_configurations: %s", e.what());
  }
}

void AppHandlers::handle_related_apps(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract component_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string component_id = req.matches[1];

    // Validate component_id
    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto apps = discovery->get_apps_for_component(component_id);

    json items = json::array();
    for (const auto & app : apps) {
      json app_item;
      app_item["id"] = app.id;
      app_item["name"] = app.name;
      if (!app.description.empty()) {
        app_item["description"] = app.description;
      }
      if (app.is_online) {
        app_item["is_online"] = true;
      }
      app_item["_links"] = {{"self", {{"href", "/api/v1/apps/" + app.id}}}};
      items.push_back(app_item);
    }

    json response;
    response["items"] = items;
    response["total_count"] = apps.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_related_apps: %s", e.what());
  }
}

void AppHandlers::handle_get_depends_on(const httplib::Request & req, httplib::Response & res) {
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

    // Build list of dependency references
    json items = json::array();
    for (const auto & dep_id : app.depends_on) {
      json item;
      item["id"] = dep_id;
      item["href"] = "/api/v1/apps/" + dep_id;

      // Try to get the dependency app for additional info
      auto dep_opt = discovery->get_app(dep_id);
      if (dep_opt) {
        item["name"] = dep_opt->name.empty() ? dep_id : dep_opt->name;

        // x-medkit extension for ROS2-specific data
        XMedkit ext;
        ext.source(dep_opt->source).is_online(dep_opt->is_online);
        item["x-medkit"] = ext.build();
      } else {
        // Dependency app could not be resolved; mark as missing in x-medkit
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

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    // HATEOAS links
    json links;
    links["self"] = "/api/v1/apps/" + app_id + "/depends-on";
    links["app"] = "/api/v1/apps/" + app_id;
    response["_links"] = links;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_depends_on: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
