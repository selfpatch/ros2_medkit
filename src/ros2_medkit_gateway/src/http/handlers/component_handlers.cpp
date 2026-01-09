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

#include "ros2_medkit_gateway/http/handlers/component_handlers.hpp"

#include <algorithm>
#include <set>

#include "ros2_medkit_gateway/exceptions.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using httplib::StatusCode;

namespace ros2_medkit_gateway {
namespace handlers {

void ComponentHandlers::handle_list_components(const httplib::Request & req, httplib::Response & res) {
  (void)req;  // Unused parameter

  try {
    const auto cache = ctx_.node()->get_entity_cache();

    json components_json = json::array();
    for (const auto & component : cache.components) {
      components_json.push_back(component.to_json());
    }

    HandlerContext::send_json(res, components_json);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Internal server error");
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_list_components: %s", e.what());
  }
}

void ComponentHandlers::handle_component_data(const httplib::Request & req, httplib::Response & res) {
  std::string component_id;
  try {
    // Extract component_id from URL path
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    component_id = req.matches[1];

    // Validate component_id
    auto validation_result = ctx_.validate_entity_id(component_id);
    if (!validation_result) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid component ID",
                                 {{"details", validation_result.error()}, {"component_id", component_id}});
      return;
    }

    const auto cache = ctx_.node()->get_entity_cache();

    // Find component in cache and get its topics from the topic map
    ComponentTopics component_topics;
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        component_topics = component.topics;
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    // Combine publishes and subscribes into unique set of topics
    std::set<std::string> all_topics;
    for (const auto & topic : component_topics.publishes) {
      all_topics.insert(topic);
    }
    for (const auto & topic : component_topics.subscribes) {
      all_topics.insert(topic);
    }

    // Sample all topics for this component
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    json component_data = json::array();

    if (!all_topics.empty()) {
      // Convert set to vector for parallel sampling
      std::vector<std::string> topics_vec(all_topics.begin(), all_topics.end());

      // Use native sampler for parallel sampling with fallback to metadata
      auto native_sampler = data_access_mgr->get_native_sampler();
      auto samples =
          native_sampler->sample_topics_parallel(topics_vec, data_access_mgr->get_topic_sample_timeout(), 10);

      for (const auto & sample : samples) {
        json topic_json;
        topic_json["topic"] = sample.topic_name;
        topic_json["timestamp"] = sample.timestamp_ns;
        topic_json["publisher_count"] = sample.publisher_count;
        topic_json["subscriber_count"] = sample.subscriber_count;

        if (sample.has_data && sample.data) {
          topic_json["status"] = "data";
          topic_json["data"] = *sample.data;
        } else {
          topic_json["status"] = "metadata_only";
        }

        // Add endpoint information with QoS
        json publishers_json = json::array();
        for (const auto & pub : sample.publishers) {
          publishers_json.push_back(pub.to_json());
        }
        topic_json["publishers"] = publishers_json;

        json subscribers_json = json::array();
        for (const auto & sub : sample.subscribers) {
          subscribers_json.push_back(sub.to_json());
        }
        topic_json["subscribers"] = subscribers_json;

        // Enrich with message type and schema
        if (!sample.message_type.empty()) {
          topic_json["type"] = sample.message_type;

          try {
            auto type_introspection = data_access_mgr->get_type_introspection();
            auto type_info = type_introspection->get_type_info(sample.message_type);
            topic_json["type_info"] = {{"schema", type_info.schema}, {"default_value", type_info.default_value}};
          } catch (const std::exception & e) {
            RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for '%s': %s", sample.message_type.c_str(),
                         e.what());
          }
        }

        component_data.push_back(topic_json);
      }
    }

    HandlerContext::send_json(res, component_data);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to retrieve component data",
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
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    component_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    // e.g., "powertrain%2Fengine%2Ftemperature" -> "powertrain/engine/temperature"
    topic_name = req.matches[2];

    // Validate component_id
    auto component_validation = ctx_.validate_entity_id(component_id);
    if (!component_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid component ID",
                                 {{"details", component_validation.error()}, {"component_id", component_id}});
      return;
    }

    // Skip topic_name validation - it may contain slashes after URL decoding
    // The actual validation happens when we try to find the topic in the ROS graph

    const auto cache = ctx_.node()->get_entity_cache();

    // Find component in cache - only needed to verify it exists
    // We use the full topic path from the URL, not the component namespace
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, "Component not found",
                                 {{"component_id", component_id}});
      return;
    }

    // cpp-httplib has already decoded %2F to / in topic_name
    // Now just add leading slash to make it a full ROS topic path
    // e.g., "powertrain/engine/temperature" -> "/powertrain/engine/temperature"
    std::string full_topic_path = "/" + topic_name;

    // Get topic data from DataAccessManager (with fallback to metadata if data unavailable)
    // Uses topic_sample_timeout_sec parameter (default: 1.0s)
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    json topic_data = data_access_mgr->get_topic_sample_with_fallback(full_topic_path);

    HandlerContext::send_json(res, topic_data);
  } catch (const TopicNotAvailableException & e) {
    // Topic doesn't exist or metadata retrieval failed
    HandlerContext::send_error(res, StatusCode::NotFound_404, "Topic not found",
                               {{"component_id", component_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Topic not available for component '%s', topic '%s': %s",
                 component_id.c_str(), topic_name.c_str(), e.what());
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to retrieve topic data",
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
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid request");
      return;
    }

    component_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    // e.g., "chassis%2Fbrakes%2Fcommand" -> "chassis/brakes/command"
    topic_name = req.matches[2];

    // Validate component_id
    auto component_validation = ctx_.validate_entity_id(component_id);
    if (!component_validation) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid component ID",
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
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Invalid JSON in request body",
                                 {{"details", e.what()}});
      return;
    }

    // Validate required fields: type and data
    if (!body.contains("type") || !body["type"].is_string()) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Missing or invalid 'type' field",
                                 {{"details", "Request body must contain 'type' string field"}});
      return;
    }

    if (!body.contains("data")) {
      HandlerContext::send_error(res, StatusCode::BadRequest_400, "Missing 'data' field",
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
          res, StatusCode::BadRequest_400, "Invalid message type format",
          {{"details", "Message type should be in format: package/msg/Type"}, {"type", msg_type}});
      return;
    }

    const auto cache = ctx_.node()->get_entity_cache();

    // Find component in cache - only needed to verify it exists
    bool component_found = false;

    for (const auto & component : cache.components) {
      if (component.id == component_id) {
        component_found = true;
        break;
      }
    }

    if (!component_found) {
      HandlerContext::send_error(res, StatusCode::NotFound_404, "Component not found",
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

    // Add component info to result
    result["component_id"] = component_id;
    result["topic_name"] = topic_name;

    HandlerContext::send_json(res, result);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, StatusCode::InternalServerError_500, "Failed to publish to topic",
                               {{"details", e.what()}, {"component_id", component_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_component_topic_publish for component '%s', topic '%s': %s",
                 component_id.c_str(), topic_name.c_str(), e.what());
  }
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
