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

#include "ros2_medkit_gateway/core/http/handlers/data_handlers.hpp"

#include <algorithm>

#include "ros2_medkit_gateway/core/data/topic_data_provider.hpp"
#include "ros2_medkit_gateway/core/exceptions.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/fan_out_helpers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/dto/data.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_serialization/type_introspection.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

void DataHandlers::handle_list_data(const httplib::Request & req, httplib::Response & res) {
  std::string entity_id;
  try {
    if (req.matches.size() < 2) {
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }
    auto entity_info = *entity_opt;

    // Delegate to plugin DataProvider if entity is plugin-owned
    if (entity_info.is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * data_prov = pmgr ? pmgr->get_data_provider_for_entity(entity_id) : nullptr;
      if (data_prov) {
        try {
          auto result = data_prov->list_data(entity_id);
          if (result) {
            HandlerContext::send_json(res, *result);
          } else {
            HandlerContext::send_plugin_error(res, result.error().http_status, result.error().message,
                                              {{"entity_id", entity_id}});
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw exception", {{"entity_id", entity_id}});
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw unknown exception", {{"entity_id", entity_id}});
        }
        return;
      }
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                                 "No data provider for plugin entity '" + entity_id + "'");
      return;
    }

    // Use unified cache method to get aggregated data
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto aggregated = cache.get_entity_data(entity_id);

    // Get data access manager for type introspection
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    auto type_introspection = data_access_mgr->get_type_introspection();

    // Build items array with ValueMetadata format (typed DTO)
    dto::Collection<dto::DataItem> collection;

    for (const auto & topic : aggregated.topics) {
      dto::DataItem di;
      // SOVD required fields - use topic.name directly as ID (clients URL-encode for GET/PUT)
      di.id = topic.name;
      di.name = topic.name;
      di.category = "currentData";

      // x-medkit extension for ROS2-specific data (typed DTO)
      dto::XMedkitDataItem item_xm;
      dto::XMedkitRos2 ros2_meta;
      ros2_meta.topic = topic.name;
      ros2_meta.direction = topic.direction;

      // Add type info if available (use cached topic types for O(1) lookup)
      std::string topic_type = cache.get_topic_type(topic.name);
      if (!topic_type.empty()) {
        ros2_meta.type = topic_type;
        try {
          auto type_info = type_introspection->get_type_info(topic_type);
          json type_info_obj;
          type_info_obj["schema"] = type_info.schema;
          type_info_obj["default_value"] = type_info.default_value;
          item_xm.type_info = type_info_obj;
        } catch (const std::exception & e) {
          RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for topic '%s': %s", topic.name.c_str(),
                       e.what());
        }
      }
      item_xm.ros2 = ros2_meta;
      di.x_medkit = item_xm;
      collection.items.push_back(di);
    }

    // Build response JSON from collection, then attach typed collection x-medkit
    json response = dto::JsonWriter<dto::Collection<dto::DataItem>>::write(collection);

    // Fan-out peer merge: use JSON-overload to avoid the legacy XMedkit builder
    json ext_json = json::object();
    merge_peer_items(ctx_.aggregation_manager(), req, response, ext_json);

    dto::DataListXMedkit resp_xm;
    resp_xm.entity_id = entity_id;
    if (aggregated.is_aggregated) {
      resp_xm.aggregated = true;
      resp_xm.aggregation_sources = aggregated.source_ids;
      resp_xm.aggregation_level = aggregated.aggregation_level;
    }
    resp_xm.total_count = response["items"].size();
    // Merge any partial/failed_peers injected by merge_peer_items into the typed xm
    json resp_xm_json = dto::JsonWriter<dto::DataListXMedkit>::write(resp_xm);
    for (const auto & [k, v] : ext_json.items()) {
      resp_xm_json[k] = v;
    }
    response["x-medkit"] = resp_xm_json;

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to retrieve entity data",
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
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    // cpp-httplib automatically decodes percent-encoded characters in URL path
    topic_name = req.matches[2];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }

    // Delegate to plugin DataProvider if entity is plugin-owned
    if (entity_opt->is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * data_prov = pmgr ? pmgr->get_data_provider_for_entity(entity_id) : nullptr;
      if (data_prov) {
        try {
          auto result = data_prov->read_data(entity_id, topic_name);
          if (result) {
            HandlerContext::send_json(res, *result);
          } else {
            HandlerContext::send_plugin_error(res, result.error().http_status, result.error().message,
                                              {{"entity_id", entity_id}});
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw exception", {{"entity_id", entity_id}});
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw unknown exception", {{"entity_id", entity_id}});
        }
        return;
      }
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                                 "No data provider for plugin entity '" + entity_id + "'");
      return;
    }

    // Determine the full ROS topic path
    std::string full_topic_path;
    if (topic_name.empty() || topic_name[0] == '/') {
      full_topic_path = topic_name;
    } else {
      full_topic_path = "/" + topic_name;
    }

    // Sampling goes through the pool-backed TopicDataProvider (issue #375
    // race fix). The provider is configured in main() before serving traffic.
    auto data_access_mgr = ctx_.node()->get_data_access_manager();
    TopicSampleResult sample;
    const auto timeout_sec = data_access_mgr->get_topic_sample_timeout();
    auto * provider = data_access_mgr->get_topic_data_provider();
    if (!provider) {
      HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Topic sampling is not configured");
      return;
    }
    const auto timeout_ms = std::chrono::milliseconds{static_cast<std::int64_t>(std::max(timeout_sec, 0.0) * 1000.0)};
    auto r = provider->sample(full_topic_path, timeout_ms);
    if (!r) {
      // Propagate the provider's ErrorInfo verbatim: http_status, code (SOVD
      // constant or x-medkit-*), message, and structured params. Collapsing
      // all errors to metadata-only (previous behaviour) would mask 503s
      // (gateway shutdown, cold-wait cap) and 500s (subscribe failed) as
      // successful metadata-only responses, which breaks retry-on-5xx clients.
      const auto & err = r.error();
      nlohmann::json params = err.params;
      params["entity_id"] = entity_id;
      params["topic_name"] = topic_name;
      const std::string code = err.code.empty() ? std::string{ERR_INTERNAL_ERROR} : err.code;
      const int status = (err.http_status >= 400 && err.http_status < 600) ? err.http_status : 500;
      HandlerContext::send_error(res, status, code, err.message, params);
      return;
    }
    sample = *r;

    // Build SOVD ReadValue response (id must match what list returns for round-trip)
    json response;
    response["id"] = full_topic_path;

    // SOVD "data" field contains the actual value
    if (sample.has_data && sample.data) {
      response["data"] = *sample.data;
    } else {
      response["data"] = json::object();
    }

    // Build typed x-medkit extension with ROS2-specific data
    dto::XMedkitDataItem xm;
    dto::XMedkitRos2 ros2_meta;
    ros2_meta.topic = full_topic_path;
    xm.entity_id = entity_id;
    if (!sample.message_type.empty()) {
      ros2_meta.type = sample.message_type;

      // Add type_info schema for the message type
      auto type_introspection = data_access_mgr->get_type_introspection();
      try {
        auto type_info = type_introspection->get_type_info(sample.message_type);
        json type_info_obj;
        type_info_obj["schema"] = type_info.schema;
        type_info_obj["default_value"] = type_info.default_value;
        xm.type_info = type_info_obj;
      } catch (const std::exception & e) {
        RCLCPP_DEBUG(HandlerContext::logger(), "Could not get type info for topic '%s': %s", full_topic_path.c_str(),
                     e.what());
      }
    }
    xm.ros2 = ros2_meta;
    xm.timestamp = sample.timestamp_ns;
    xm.publisher_count = static_cast<std::int64_t>(sample.publisher_count);
    xm.subscriber_count = static_cast<std::int64_t>(sample.subscriber_count);
    xm.status = json(sample.has_data ? "data" : "metadata_only");
    response["x-medkit"] = dto::JsonWriter<dto::XMedkitDataItem>::write(xm);

    HandlerContext::send_json(res, response);
  } catch (const TopicNotAvailableException & e) {
    HandlerContext::send_error(res, 404, ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE, "Topic not found",
                               {{"entity_id", entity_id}, {"topic_name", topic_name}});
    RCLCPP_DEBUG(HandlerContext::logger(), "Topic not available for entity '%s', topic '%s': %s", entity_id.c_str(),
                 topic_name.c_str(), e.what());
  } catch (const ProviderErrorException & e) {
    // Provider returned a non-404 ErrorInfo via DAM (shutdown, subscribe
    // failed, pool saturation). Preserve the original http_status / code
    // so 5xx surfaces to retry-on-5xx clients instead of looking like 404.
    const auto & info = e.info();
    nlohmann::json params = info.params;
    params["entity_id"] = entity_id;
    params["topic_name"] = topic_name;
    const std::string code = info.code.empty() ? std::string{ERR_INTERNAL_ERROR} : info.code;
    const int status = (info.http_status >= 400 && info.http_status < 600) ? info.http_status : 500;
    HandlerContext::send_error(res, status, code, info.message, params);
    RCLCPP_WARN(HandlerContext::logger(), "Provider error for entity '%s', topic '%s': %s [%d]", entity_id.c_str(),
                topic_name.c_str(), info.message.c_str(), info.http_status);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to retrieve topic data",
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
      HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid request");
      return;
    }

    entity_id = req.matches[1];
    topic_name = req.matches[2];

    // Validate entity ID and type for this route
    auto entity_opt = ctx_.validate_entity_for_route(req, res, entity_id);
    if (!entity_opt) {
      return;  // Response already sent (error or forwarded to peer)
    }

    // Check lock access for data (before plugin delegation - locks apply to all entities)
    if (ctx_.validate_lock_access(req, res, *entity_opt, "data")) {
      return;
    }

    // Delegate to plugin DataProvider if entity is plugin-owned
    if (entity_opt->is_plugin) {
      auto * pmgr = ctx_.node()->get_plugin_manager();
      auto * data_prov = pmgr ? pmgr->get_data_provider_for_entity(entity_id) : nullptr;
      if (data_prov) {
        json value;
        if (!req.body.empty()) {
          value = json::parse(req.body, nullptr, false);
          if (value.is_discarded()) {
            HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON body");
            return;
          }
        }
        try {
          auto result = data_prov->write_data(entity_id, topic_name, value);
          if (result) {
            HandlerContext::send_json(res, *result);
          } else {
            HandlerContext::send_plugin_error(res, result.error().http_status, result.error().message,
                                              {{"entity_id", entity_id}});
          }
        } catch (const std::exception & e) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw for entity '%s': %s", entity_id.c_str(),
                       e.what());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw exception", {{"entity_id", entity_id}});
        } catch (...) {
          RCLCPP_ERROR(HandlerContext::logger(), "Plugin DataProvider threw unknown exception for entity '%s'",
                       entity_id.c_str());
          HandlerContext::send_plugin_error(res, 500, "Plugin threw unknown exception", {{"entity_id", entity_id}});
        }
        return;
      }
      HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                                 "No data provider for plugin entity '" + entity_id + "'");
      return;
    }

    // Parse and validate request body via DTO (checks type:string + data:json required)
    auto body_opt = ctx_.parse_body<dto::DataWriteRequest>(req, res);
    if (!body_opt) {
      return;  // 400 already sent by parse_body
    }
    const std::string & msg_type = body_opt->type;
    const json & data = body_opt->data;

    // Validate message type format (e.g., std_msgs/msg/Float32)
    auto slash_count = static_cast<size_t>(std::count(msg_type.begin(), msg_type.end(), '/'));
    size_t msg_pos = msg_type.find("/msg/");
    bool valid_format =
        (slash_count == 2) && (msg_pos != std::string::npos) && (msg_pos > 0) && (msg_pos + 5 < msg_type.length());

    if (!valid_format) {
      HandlerContext::send_error(
          res, 400, ERR_INVALID_PARAMETER, "Invalid message type format",
          {{"details", "Message type should be in format: package/msg/Type"}, {"type", msg_type}});
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

    // Build response with typed x-medkit extension (id must match what list returns for round-trip)
    json response;
    response["id"] = full_topic_path;
    response["data"] = data;  // Echo back the written data

    dto::XMedkitDataItem xm;
    dto::XMedkitRos2 ros2_meta;
    ros2_meta.topic = full_topic_path;
    ros2_meta.type = msg_type;
    xm.ros2 = ros2_meta;
    xm.entity_id = entity_id;
    if (result.contains("status")) {
      xm.status = result["status"];
    }
    if (result.contains("publisher_created") && result["publisher_created"].is_boolean()) {
      xm.publisher_created = result["publisher_created"].get<bool>();
    }
    response["x-medkit"] = dto::JsonWriter<dto::XMedkitDataItem>::write(xm);

    HandlerContext::send_json(res, response);
  } catch (const std::exception & e) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to publish to topic",
                               {{"details", e.what()}, {"entity_id", entity_id}, {"topic_name", topic_name}});
    RCLCPP_ERROR(HandlerContext::logger(), "Error in handle_put_data_item for entity '%s', topic '%s': %s",
                 entity_id.c_str(), topic_name.c_str(), e.what());
  }
}

void DataHandlers::handle_data_categories(const httplib::Request & req, httplib::Response & res) {
  (void)req;
  HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Data categories are not implemented for ROS 2",
                             {{"feature", "data-categories"}});
}

void DataHandlers::handle_data_groups(const httplib::Request & req, httplib::Response & res) {
  (void)req;
  HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Data groups are not implemented for ROS 2",
                             {{"feature", "data-groups"}});
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
