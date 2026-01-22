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

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void AppHandlers::handle_list_apps(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    auto discovery = ctx_.node()->get_discovery_manager();
    auto apps = discovery->discover_apps();

    json items = json::array();
    for (const auto & app : apps) {
      json app_item;
      app_item["id"] = app.id;
      app_item["name"] = app.name;
      if (!app.description.empty()) {
        app_item["description"] = app.description;
      }
      if (!app.tags.empty()) {
        app_item["tags"] = app.tags;
      }
      if (app.is_online) {
        app_item["is_online"] = true;
      }
      if (!app.component_id.empty()) {
        app_item["component_id"] = app.component_id;
      }
      items.push_back(app_item);
    }

    json response;
    response["items"] = items;
    response["total_count"] = apps.size();

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

    // Build response with capabilities (SOVD entity/{id} pattern)
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
    if (app.is_online) {
      response["is_online"] = true;
    }
    if (app.bound_fqn) {
      response["bound_fqn"] = *app.bound_fqn;
    }
    response["source"] = app.source;

    // Build capabilities using CapabilityBuilder
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

    // Build data items from app's topics
    json items = json::array();

    // Publishers
    for (const auto & topic_name : app.topics.publishes) {
      json item;
      item["id"] = topic_name;
      item["name"] = topic_name;
      item["direction"] = "publish";
      item["href"] = "/api/v1/apps/" + app.id + "/data/" + topic_name;
      items.push_back(item);
    }

    // Subscribers
    for (const auto & topic_name : app.topics.subscribes) {
      json item;
      item["id"] = topic_name;
      item["name"] = topic_name;
      item["direction"] = "subscribe";
      item["href"] = "/api/v1/apps/" + app.id + "/data/" + topic_name;
      items.push_back(item);
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

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

    // Search in publishers
    for (const auto & topic_name : app.topics.publishes) {
      if (topic_name == data_id) {
        json response;
        response["id"] = topic_name;
        response["name"] = topic_name;
        response["direction"] = "publish";

        // Sample topic value via native sampler
        auto sample = native_sampler->sample_topic(topic_name, data_access_mgr->get_topic_sample_timeout());
        response["timestamp"] = sample.timestamp_ns;
        response["publisher_count"] = sample.publisher_count;
        response["subscriber_count"] = sample.subscriber_count;

        if (sample.has_data && sample.data) {
          response["status"] = "data";
          response["data"] = *sample.data;
        } else {
          response["status"] = "metadata_only";
        }

        if (!sample.message_type.empty()) {
          response["type"] = sample.message_type;
        }

        HandlerContext::send_json(res, response);
        return;
      }
    }

    // Search in subscribers
    for (const auto & topic_name : app.topics.subscribes) {
      if (topic_name == data_id) {
        json response;
        response["id"] = topic_name;
        response["name"] = topic_name;
        response["direction"] = "subscribe";

        // Sample topic value via native sampler
        auto sample = native_sampler->sample_topic(topic_name, data_access_mgr->get_topic_sample_timeout());
        response["timestamp"] = sample.timestamp_ns;
        response["publisher_count"] = sample.publisher_count;
        response["subscriber_count"] = sample.subscriber_count;

        if (sample.has_data && sample.data) {
          response["status"] = "data";
          response["data"] = *sample.data;
        } else {
          response["status"] = "metadata_only";
        }

        if (!sample.message_type.empty()) {
          response["type"] = sample.message_type;
        }

        HandlerContext::send_json(res, response);
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

}  // namespace handlers
}  // namespace ros2_medkit_gateway
