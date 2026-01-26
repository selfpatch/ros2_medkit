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

#include "ros2_medkit_gateway/http/handlers/data_handlers.hpp"

#include <algorithm>
#include <map>

#include "ros2_medkit_gateway/exceptions.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/http/x_medkit.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void DataHandlers::handle_list_data(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    auto validation_result = ctx_.validate_entity_id(entity_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", validation_result.error()}, {"entity_id", entity_id}});
      return;
    }

    // First, verify that the entity actually exists in the cache
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto entity_ref = cache.find_entity(entity_id);
    if (!entity_ref) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Use unified cache method to get aggregated data
    auto aggregated = cache.get_entity_data(entity_id);

    // Get data access manager for type introspection
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto native_sampler = data_access_mgr->get_native_sampler();
    auto type_introspection = data_access_mgr->get_type_introspection();

    // Build topic name -> type map from all discovered topics
    std::map<std::string, std::string> topic_type_map;
    auto all_topics = native_sampler->discover_all_topics();
    for (const auto & topic_info : all_topics) {
      topic_type_map[topic_info.name] = topic_info.type;
    }

    // Build items array with ValueMetadata format
    json items = json::array();

    for (const auto & topic : aggregated.topics) {
      json item;
      // SOVD required fields
      item["id"] = normalize_topic_to_id(topic.name);
      item["name"] = topic.name;
      item["category"] = "currentData";

      // x-medkit extension for ROS2-specific data
      XMedkit ext;
      ext.ros2_topic(topic.name).add_ros2("direction", topic.direction);

      // Add type info if available
      auto type_it = topic_type_map.find(topic.name);
      if (type_it != topic_type_map.end() && !type_it->second.empty()) {
        ext.ros2_type(type_it->second);
        try {
          auto type_info = type_introspection->get_type_info(type_it->second);
          ext.type_info(type_info.schema);
        } catch (const std::exception & e) {
          RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for topic '%s': %s", topic.name.c_str(),
                       e.what());
        }
      }

      item["x-medkit"] = ext.build();
      items.push_back(item);
    }

    // Build response with x-medkit for total_count
    json response;
    response["items"] = items;

    XMedkit resp_ext;
    resp_ext.entity_id(entity_id).add("total_count", items.size());
    if (aggregated.is_aggregated) {
      resp_ext.add("aggregated_from", aggregated.source_ids);
      resp_ext.add("aggregation_level", aggregated.aggregation_level);
    }
    response["x-medkit"] = resp_ext.build();

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to retrieve entity data",
                               {{"details", e.what()}, {"entity_id", entity_id}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_data for entity '%s': %s", entity_id.c_str(),
                 e.what());
  }
}

void DataHandlers::handle_get_data_item(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string topic_name;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    topic_name = req.matches[2];

    auto validation_result = ctx_.validate_entity_id(entity_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", validation_result.error()}, {"entity_id", entity_id}});
      return;
    }

    // Verify entity exists
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto entity_ref = cache.find_entity(entity_id);
    if (!entity_ref) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Determine the full ROS topic path
    std::string full_topic_path;
    if (topic_name.empty() || topic_name[0] == '/') {
      full_topic_path = topic_name;
    } else {
      full_topic_path = "/" + topic_name;
    }

    // Get topic data from DataAccessManager
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto native_sampler = data_access_mgr->get_native_sampler();
    auto sample = native_sampler->sample_topic(full_topic_path, data_access_mgr->get_topic_sample_timeout());

    // Build SOVD ReadValue response
    json response;
    response["id"] = normalize_topic_to_id(full_topic_path);

    // SOVD "data" field contains the actual value
    if (sample.has_data && sample.data) {
      response["data"] = *sample.data;
    } else {
      response["data"] = json::object();
    }

    // Build x-medkit extension with ROS2-specific data
    XMedkit ext;
    ext.ros2_topic(full_topic_path).entity_id(entity_id);
    if (!sample.message_type.empty()) {
      ext.ros2_type(sample.message_type);

      // Add type_info schema for the message type
      auto type_introspection = data_access_mgr->get_type_introspection();
      try {
        auto type_info = type_introspection->get_type_info(sample.message_type);
        ext.type_info(type_info.schema);
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for topic '%s': %s", full_topic_path.c_str(),
                     e.what());
      }
    }
    ext.add("timestamp", sample.timestamp_ns);
    ext.add("publisher_count", sample.publisher_count);
    ext.add("subscriber_count", sample.subscriber_count);
    ext.add("status", sample.has_data ? "data" : "metadata_only");
    response["x-medkit"] = ext.build();

    HandlerContext::send_json(res, response);
  } catch (const TopicNotAvailableException & e) {
    HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE, "Topic not found",
                               {{"entity_id", entity_id}, {"topic_name", topic_name}});
    RCLCPP_DEBUG(HandlerContext::logger(), "Topic not available for entity '%s', topic '%s': %s", entity_id.c_str(),
                 topic_name.c_str(), e.what());
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, ERR_INTERNAL_ERROR,
                               "Failed to retrieve topic data",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_get_data_item for entity '%s', topic '%s': %s",
                 entity_id.c_str(), topic_name.c_str(), e.what());
  }
}

void DataHandlers::handle_put_data_item(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  std::string topic_name;
  try {
    if (req.matches.size() < 3) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    topic_name = req.matches[2];

    auto validation_result = ctx_.validate_entity_id(entity_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid entity ID",
                                 {{"details", validation_result.error()}, {"entity_id", entity_id}});
      return;
    }

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
    size_t slash_count = std::count(msg_type.begin(), msg_type.end(), '/');
    size_t msg_pos = msg_type.find("/msg/");
    bool valid_format =
        (slash_count == 2) && (msg_pos != std::string::npos) && (msg_pos > 0) && (msg_pos + 5 < msg_type.length());

    if (!valid_format) {
      HandlerContext::send_error(
          res, StatusCode::BadRequest_400, ERR_INVALID_PARAMETER, "Invalid message type format",
          {{"details", "Message type should be in format: package/msg/Type"}, {"type", msg_type}});
      return;
    }

    // Verify entity exists
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto entity_ref = cache.find_entity(entity_id);
    if (!entity_ref) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, ERR_ENTITY_NOT_FOUND, "Entity not found",
                                 {{"entity_id", entity_id}});
      return;
    }

    // Build full topic path (mirror GET logic: only prefix '/' when needed)
    std::string full_topic_path = topic_name;
    if (!full_topic_path.empty() && full_topic_path.front() != '/') {
      full_topic_path = "/" + full_topic_path;
    }

    // Publish data using DataAccessManager
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    json result = data_access_mgr->publish_to_topic(full_topic_path, msg_type, data);

    // Build response with x-medkit extension
    json response;
    response["id"] = normalize_topic_to_id(full_topic_path);
    response["data"] = data;  // Echo back the written data

    XMedkit ext;
    ext.ros2_topic(full_topic_path).ros2_type(msg_type).entity_id(entity_id);
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
                               {{"details", e.what()}, {"entity_id", entity_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_put_data_item for entity '%s', topic '%s': %s",
                 entity_id.c_str(), topic_name.c_str(), e.what());
  }
}

void DataHandlers::handle_data_categories(const httplib::Request & req, httplib::Response & res) {
  (void)req;
  HandlerContext::send_error(res, StatusCode::NotImplemented_501, ERR_NOT_IMPLEMENTED,
                             "Data categories are not implemented for ROS 2", {{"feature", "data-categories"}});
}

void DataHandlers::handle_data_groups(const httplib::Request & req, httplib::Response & res) {
  (void)req;
  HandlerContext::send_error(res, StatusCode::NotImplemented_501, ERR_NOT_IMPLEMENTED,
                             "Data groups are not implemented for ROS 2", {{"feature", "data-groups"}});
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
