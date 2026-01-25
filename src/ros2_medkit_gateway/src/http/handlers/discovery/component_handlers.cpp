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

#include "ros2_medkit_gateway/http/handlers/discovery/component_handlers.hpp"

#include <algorithm>
#include <set>

#include "ros2_medkit_gateway/exceptions.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/capability_builder.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void ComponentHandlers::handle_list_components(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    const auto& cache = ctx_.node()->get_thread_safe_cache();
    const auto components = cache.get_components();

    // Build items array with EntityReference format
    json items = json::array();
    for (const auto & component : components) {
      json item;
      // Required fields for EntityReference
      item["id"] = component.id;
      item["name"] = component.name.empty() ? component.id : component.name;
      item["href"] = "/api/v1/components/" + component.id;

      // Optional fields
      if (!component.description.empty()) {
        item["description"] = component.description;
      }
      if (!component.tags.empty()) {
        item["tags"] = component.tags;
      }

      // x-medkit extension for ROS2-specific data
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

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_components: %s", e.what());
  }
}

void ComponentHandlers::handle_get_component(const httplib::Request & req, httplib::Response & res) {
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
    // Required fields
    response["id"] = comp.id;
    response["name"] = comp.name.empty() ? comp.id : comp.name;

    // Optional fields
    if (!comp.description.empty()) {
      response["description"] = comp.description;
    }
    if (!comp.tags.empty()) {
      response["tags"] = comp.tags;
    }

    // Capability URIs (flat at top level)
    std::string base = "/api/v1/components/" + comp.id;
    response["data"] = base + "/data";
    response["operations"] = base + "/operations";
    response["configurations"] = base + "/configurations";
    response["faults"] = base + "/faults";
    response["subcomponents"] = base + "/subcomponents";
    response["hosts"] = base + "/hosts";  // SOVD 7.6.2.4

    // Add depends-on only when component has dependencies
    if (!comp.depends_on.empty()) {
      response["depends-on"] = base + "/depends-on";
    }

    // Add belongs-to field referencing the parent Area (SOVD 7.6.3)
    if (!comp.area.empty()) {
      response["belongs-to"] = "/api/v1/areas/" + comp.area;
    }

    // Build HATEOAS links
    LinksBuilder links;
    links.self(base).collection("/api/v1/components");
    if (!comp.area.empty()) {
      links.add("area", "/api/v1/areas/" + comp.area);
    }
    if (!comp.parent_component_id.empty()) {
      links.parent("/api/v1/components/" + comp.parent_component_id);
    }
    response["_links"] = links.build();

    // x-medkit extension for ROS2-specific data
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

    // Add detailed capabilities object to x-medkit
    using Cap = CapabilityBuilder::Capability;
    std::vector<Cap> caps = {Cap::DATA,   Cap::OPERATIONS,    Cap::CONFIGURATIONS,
                             Cap::FAULTS, Cap::SUBCOMPONENTS, Cap::HOSTS};
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

void ComponentHandlers::handle_component_data(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  try {
    // Extract component_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    component_id = req.matches[1];

    // Validate component_id
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

    const auto & component = *comp_opt;

    // Collect all topics - from component directly OR aggregated from related Apps
    // Track publisher vs subscriber for direction info
    std::set<std::string> publish_topics;
    std::set<std::string> subscribe_topics;

    // First, check component's own topics
    for (const auto & topic : component.topics.publishes) {
      publish_topics.insert(topic);
    }
    for (const auto & topic : component.topics.subscribes) {
      subscribe_topics.insert(topic);
    }

    // If component has no direct topics (synthetic component), aggregate from Apps
    if (publish_topics.empty() && subscribe_topics.empty()) {
      auto apps = discovery->get_apps_for_component(component_id);
      for (const auto & app : apps) {
        for (const auto & topic : app.topics.publishes) {
          publish_topics.insert(topic);
        }
        for (const auto & topic : app.topics.subscribes) {
          subscribe_topics.insert(topic);
        }
      }
    }

    // Build items array with ValueMetadata format
    json items = json::array();

    // Add publisher topics
    for (const auto & topic_name : publish_topics) {
      json item;
      // SOVD required fields
      item["id"] = normalize_topic_to_id(topic_name);
      item["name"] = topic_name;
      item["category"] = "currentData";

      // x-medkit extension for ROS2-specific data
      XMedkit ext;
      ext.ros2_topic(topic_name).add_ros2("direction", "publish");
      item["x-medkit"] = ext.build();

      items.push_back(item);
    }

    // Add subscriber topics (avoid duplicates)
    for (const auto & topic_name : subscribe_topics) {
      // Skip if already added as publisher
      if (publish_topics.count(topic_name) > 0) {
        continue;
      }

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
    resp_ext.entity_id(component_id).add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to retrieve component data",
                               {{"details", e.what()}, {"component_id", component_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_component_data for component '%s': %s",
                 component_id.c_str(), e.what());
  }
}

void ComponentHandlers::handle_component_topic_data(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string topic_name;
  try {
    // Extract component_id and topic_name from URL path
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    component_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    // e.g., "powertrain%2Fengine%2Ftemperature" -> "powertrain/engine/temperature"
    topic_name = req.matches[2];

    // Validate component_id
    auto component_validation = ctx_.validate_entity_id(component_id);
    if (!component_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", component_validation.error()}, {"component_id", component_id}});
      return;
    }

    // Skip topic_name validation - it may contain slashes after URL decoding
    // The actual validation happens when we try to find the topic in the ROS graph

    auto discovery = ctx_.node()->get_discovery_manager();
    auto comp_opt = discovery->get_component(component_id);

    if (!comp_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    // cpp-httplib has already decoded %2F to / in topic_name
    // Determine the full ROS topic path
    std::string full_topic_path;
    if (topic_name.empty() || topic_name[0] == '/') {
      full_topic_path = topic_name;
    } else {
      full_topic_path = "/" + topic_name;
    }

    // Also support normalized data IDs (sensor_temperature -> /sensor/temperature search)
    // Try both the normalized ID and the raw topic name

    // Get topic data from DataAccessManager
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto native_sampler = data_access_mgr->get_native_sampler();
    auto sample = native_sampler->sample_topic(full_topic_path, data_access_mgr->get_topic_sample_timeout());

    // Build SOVD ReadValue response
    json response;
    // SOVD required fields
    response["id"] = normalize_topic_to_id(full_topic_path);

    // SOVD "data" field contains the actual value
    if (sample.has_data && sample.data) {
      response["data"] = *sample.data;
    } else {
      response["data"] = json::object();  // Empty object if no data available
    }

    // Build x-medkit extension with ROS2-specific data
    XMedkit ext;
    ext.ros2_topic(full_topic_path).entity_id(component_id);
    if (!sample.message_type.empty()) {
      ext.ros2_type(sample.message_type);
    }
    ext.add("timestamp", sample.timestamp_ns);
    ext.add("publisher_count", sample.publisher_count);
    ext.add("subscriber_count", sample.subscriber_count);
    ext.add("status", sample.has_data ? "data" : "metadata_only");
    response["x-medkit"] = ext.build();

    HandlerContext::send_json(res, response);
  } catch (const TopicNotAvailableException & e) {
    // Topic doesn't exist or metadata retrieval failed
    HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE, "Topic not found",
                               {{"component_id", component_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Topic not available for component '%s', topic '%s': %s",
                 component_id.c_str(), topic_name.c_str(), e.what());
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to retrieve topic data",
                               {{"details", e.what()}, {"component_id", component_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_component_topic_data for component '%s', topic '%s': %s",
                 component_id.c_str(), topic_name.c_str(), e.what());
  }
}

void ComponentHandlers::handle_component_topic_publish(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  std::string topic_name;
  try {
    // Extract component_id and topic_name from URL path
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    component_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    // e.g., "chassis%2Fbrakes%2Fcommand" -> "chassis/brakes/command"
    topic_name = req.matches[2];

    // Validate component_id
    auto component_validation = ctx_.validate_entity_id(component_id);
    if (!component_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid component ID",
                                 {{"details", component_validation.error()}, {"component_id", component_id}});
      return;
    }

    // Skip topic_name validation - it may contain slashes after URL decoding
    // The actual validation happens when we try to publish to the topic

    // Parse request body
    json body;
    try {
      body = json::parse(req.body);
    } catch (const json::parse_error & e) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid JSON in request body",
                                 {{"details", e.what()}});
      return;
    }

    // Validate required fields: type and data
    if (!body.contains("type") || !body["type"].is_string()) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER,
                                 "Missing or invalid 'type' field",
                                 {{"details", "Request body must contain 'type' string field"}});
      return;
    }

    if (!body.contains("data")) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Missing 'data' field",
                                 {{"details", "Request body must contain 'data' field"}});
      return;
    }

    std::string msg_type = body["type"].get<std::string>();
    json data = body["data"];

    // Validate message type format (e.g., std_msgs/msg/Float32)
    // Expected format: package/msg/Type (exactly 2 slashes)
    size_t slash_count = std::count(msg_type.begin(), msg_type.end(), '/');
    size_t msg_pos = msg_type.find("/msg/");
    bool valid_format = (slash_count == 2) && (msg_pos != std::string::npos) &&
                        (msg_pos > 0) &&                    // package before /msg/
                        (msg_pos + 5 < msg_type.length());  // Type after /msg/

    if (!valid_format) {
      HandlerContext::send_error(
          res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid message type format",
          {{"details", "Message type should be in format: package/msg/Type"}, {"type", msg_type}});
      return;
    }

    auto discovery = ctx_.node()->get_discovery_manager();
    auto comp_opt = discovery->get_component(component_id);

    if (!comp_opt) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    // cpp-httplib has already decoded %2F to / in topic_name
    // Now just add leading slash to make it a full ROS topic path
    // e.g., "chassis/brakes/command" -> "/chassis/brakes/command"
    std::string full_topic_path = "/" + topic_name;

    // Publish data using DataAccessManager
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    json result = data_access_mgr->publish_to_topic(full_topic_path, msg_type, data);

    // Build response with x-medkit extension
    json response;
    // Required fields
    response["id"] = normalize_topic_to_id(full_topic_path);
    response["data"] = data;  // Echo back the written data

    // Build x-medkit extension with ROS2-specific data
    XMedkit ext;
    ext.ros2_topic(full_topic_path).ros2_type(msg_type).entity_id(component_id);
    if (result.contains("status")) {
      ext.add("status", result["status"]);
    }
    if (result.contains("publisher_created")) {
      ext.add("publisher_created", result["publisher_created"]);
    }
    response["x-medkit"] = ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to publish to topic",
                               {{"details", e.what()}, {"component_id", component_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_component_topic_publish for component '%s', topic '%s': %s",
                 component_id.c_str(), topic_name.c_str(), e.what());
  }
}

void ComponentHandlers::handle_get_subcomponents(const httplib::Request & req, httplib::Response & res) {
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

    // Get subcomponents
    auto subcomponents = discovery->get_subcomponents(component_id);

    json items = json::array();
    for (const auto & sub : subcomponents) {
      json item;
      item["id"] = sub.id;
      item["name"] = sub.name.empty() ? sub.id : sub.name;
      item["href"] = "/api/v1/components/" + sub.id;

      // x-medkit extension for ROS2-specific data
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

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    // HATEOAS links
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

void ComponentHandlers::handle_get_hosts(const httplib::Request & req, httplib::Response & res) {
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

    // Get apps hosted on this component (SOVD 7.6.2.4 - non-deprecated relationship)
    auto apps = discovery->get_apps_for_component(component_id);

    json items = json::array();
    for (const auto & app : apps) {
      json item;
      item["id"] = app.id;
      item["name"] = app.name.empty() ? app.id : app.name;
      item["href"] = "/api/v1/apps/" + app.id;

      // x-medkit extension for ROS2-specific data
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

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    // HATEOAS links
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

void ComponentHandlers::handle_get_depends_on(const httplib::Request & req, httplib::Response & res) {
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

    // Build list of dependency references
    json items = json::array();
    for (const auto & dep_id : comp.depends_on) {
      json item;
      item["id"] = dep_id;
      item["href"] = "/api/v1/components/" + dep_id;

      // Try to get the dependency component for additional info
      auto dep_opt = discovery->get_component(dep_id);
      if (dep_opt) {
        item["name"] = dep_opt->name.empty() ? dep_id : dep_opt->name;

        // x-medkit extension for ROS2-specific data
        XMedkit ext;
        ext.source(dep_opt->source);
        item["x-medkit"] = ext.build();
      } else {
        // Dependency component could not be resolved; mark as missing in x-medkit
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

    // x-medkit for response-level metadata
    XMedkit resp_ext;
    resp_ext.add("total_count", items.size());
    response["x-medkit"] = resp_ext.build();

    // HATEOAS links
    json links;
    links["self"] = "/api/v1/components/" + component_id + "/depends-on";
    links["component"] = "/api/v1/components/" + component_id;
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
