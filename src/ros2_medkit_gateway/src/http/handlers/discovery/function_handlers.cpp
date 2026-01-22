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

#include "ros2_medkit_gateway/http/handlers/discovery/function_handlers.hpp"

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/capability_builder.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void FunctionHandlers::handle_list_functions(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    auto discovery = ctx_.node()->get_discovery_manager();
    auto functions = discovery->discover_functions();

    json items = json::array();
    for (const auto & func : functions) {
      json func_item;
      func_item["id"] = func.id;
      func_item["name"] = func.name;
      if (!func.description.empty()) {
        func_item["description"] = func.description;
      }
      if (!func.tags.empty()) {
        func_item["tags"] = func.tags;
      }
      items.push_back(func_item);
    }

    json response;
    response["items"] = items;
    response["total_count"] = functions.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_functions: %s", e.what());
  }
}

void FunctionHandlers::handle_get_function(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract function_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    // Validate function_id
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

    // Build response with capabilities (SOVD entity/{id} pattern)
    json response;
    response["id"] = func.id;
    response["name"] = func.name;

    if (!func.description.empty()) {
      response["description"] = func.description;
    }
    if (!func.translation_id.empty()) {
      response["translation_id"] = func.translation_id;
    }
    if (!func.tags.empty()) {
      response["tags"] = func.tags;
    }
    response["source"] = func.source;

    // Build capabilities using CapabilityBuilder
    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::HOSTS, Cap::DATA, Cap::OPERATIONS};
    response["capabilities"] = CapabilityBuilder::build_capabilities("functions", func.id, caps);

    // Build HATEOAS links using LinksBuilder
    LinksBuilder links;
    links.self("/api/v1/functions/" + func.id).collection("/api/v1/functions");
    response["_links"] = links.build();

    // Add depends-on as array if present (special case - multiple links)
    if (!func.depends_on.empty()) {
      json depends_links = json::array();
      for (const auto & dep_id : func.depends_on) {
        depends_links.push_back("/api/v1/functions/" + dep_id);
      }
      response["_links"]["depends-on"] = depends_links;
    }

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_function: %s", e.what());
  }
}

void FunctionHandlers::handle_function_hosts(const httplib::Request & req, httplib::Response & res) {
  try {
    // Extract function_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    std::string function_id = req.matches[1];

    // Validate function_id
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

    // Get host app IDs
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

void FunctionHandlers::handle_get_function_data(const httplib::Request & req, httplib::Response & res) {
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

    // Aggregate data from all host apps
    auto host_ids = discovery->get_hosts_for_function(function_id);

    json items = json::array();
    for (const auto & app_id : host_ids) {
      auto app_opt = discovery->get_app(app_id);
      if (app_opt) {
        // Publishers
        for (const auto & topic_name : app_opt->topics.publishes) {
          json item;
          item["id"] = topic_name;
          item["name"] = topic_name;
          item["direction"] = "publish";
          item["source_app"] = app_id;
          std::string href = "/api/v1/apps/";
          href.append(app_id).append("/data/").append(topic_name);
          item["href"] = href;
          items.push_back(item);
        }
        // Subscribers
        for (const auto & topic_name : app_opt->topics.subscribes) {
          json item;
          item["id"] = topic_name;
          item["name"] = topic_name;
          item["direction"] = "subscribe";
          item["source_app"] = app_id;
          std::string href = "/api/v1/apps/";
          href.append(app_id).append("/data/").append(topic_name);
          item["href"] = href;
          items.push_back(item);
        }
      }
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_function_data: %s", e.what());
  }
}

void FunctionHandlers::handle_list_function_operations(const httplib::Request & req, httplib::Response & res) {
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

    // Aggregate operations from all host apps
    auto host_ids = discovery->get_hosts_for_function(function_id);

    json items = json::array();
    for (const auto & app_id : host_ids) {
      auto app_opt = discovery->get_app(app_id);
      if (app_opt) {
        // Services
        for (const auto & svc : app_opt->services) {
          json item;
          item["id"] = svc.name;
          item["name"] = svc.name;
          item["type"] = "service";
          item["service_type"] = svc.type;
          item["source_app"] = app_id;
          items.push_back(item);
        }
        // Actions
        for (const auto & act : app_opt->actions) {
          json item;
          item["id"] = act.name;
          item["name"] = act.name;
          item["type"] = "action";
          item["action_type"] = act.type;
          item["source_app"] = app_id;
          items.push_back(item);
        }
      }
    }

    json response;
    response["items"] = items;
    response["total_count"] = items.size();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error",
                               {{"details", e.what()}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_function_operations: %s", e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
