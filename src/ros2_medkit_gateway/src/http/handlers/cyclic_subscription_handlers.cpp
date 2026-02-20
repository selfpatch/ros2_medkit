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

#include "ros2_medkit_gateway/http/handlers/cyclic_subscription_handlers.hpp"

#include <chrono>
#include <cstring>
#include <regex>
#include <sstream>

#include "ros2_medkit_gateway/data_access_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/native_topic_sampler.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

CyclicSubscriptionHandlers::CyclicSubscriptionHandlers(HandlerContext & ctx, SubscriptionManager & sub_mgr,
                                                       std::shared_ptr<SSEClientTracker> client_tracker)
  : ctx_(ctx), sub_mgr_(sub_mgr), client_tracker_(std::move(client_tracker)) {
}

// ---------------------------------------------------------------------------
// POST — create subscription
// ---------------------------------------------------------------------------
void CyclicSubscriptionHandlers::handle_create(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  // Parse JSON body
  json body;
  try {
    body = json::parse(req.body);
  } catch (const json::exception &) {
    HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON request body");
    return;
  }

  // Validate required fields
  if (!body.contains("resource") || !body["resource"].is_string()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing or invalid 'resource'",
                               {{"parameter", "resource"}});
    return;
  }

  if (!body.contains("interval") || !body["interval"].is_string()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing or invalid 'interval'",
                               {{"parameter", "interval"}});
    return;
  }

  if (!body.contains("duration") || !body["duration"].is_number_integer()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing or invalid 'duration'",
                               {{"parameter", "duration"}});
    return;
  }

  // Validate protocol (optional, defaults to "sse")
  std::string protocol = "sse";
  if (body.contains("protocol")) {
    protocol = body["protocol"].get<std::string>();
    if (protocol != "sse") {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                                 "Unsupported protocol. Only 'sse' is supported.",
                                 {{"parameter", "protocol"}, {"value", protocol}});
      return;
    }
  }

  // Parse interval
  CyclicInterval interval;
  try {
    interval = parse_interval(body["interval"].get<std::string>());
  } catch (const std::invalid_argument &) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                               "Invalid interval. Must be 'fast', 'normal', or 'slow'.",
                               {{"parameter", "interval"}, {"value", body["interval"]}});
    return;
  }

  // Validate duration
  int duration = body["duration"].get<int>();
  if (duration <= 0) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                               "Duration must be a positive integer (seconds).",
                               {{"parameter", "duration"}, {"value", duration}});
    return;
  }

  // Parse resource URI to extract topic name
  std::string resource = body["resource"].get<std::string>();
  auto topic_result = parse_resource_uri(resource);
  if (!topic_result) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                               "Invalid resource URI: " + topic_result.error(),
                               {{"parameter", "resource"}, {"value", resource}});
    return;
  }

  std::string entity_type = extract_entity_type(req);

  // Validate resource URI references the same entity as the route
  std::string expected_prefix = "/api/v1/" + entity_type + "/" + entity_id + "/data/";
  if (resource.find(expected_prefix) != 0) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                               "Resource URI must reference the same entity as the route",
                               {{"parameter", "resource"}, {"value", resource}});
    return;
  }

  // Create subscription
  auto result = sub_mgr_.create(entity_id, entity_type, resource, *topic_result, protocol, interval, duration);
  if (!result) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, result.error());
    return;
  }

  std::string event_source = build_event_source(*result);
  auto response_json = subscription_to_json(*result, event_source);

  res.status = 201;
  HandlerContext::send_json(res, response_json);
}

// ---------------------------------------------------------------------------
// GET — list subscriptions
// ---------------------------------------------------------------------------
void CyclicSubscriptionHandlers::handle_list(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto subs = sub_mgr_.list(entity_id);
  json items = json::array();
  for (const auto & sub : subs) {
    items.push_back(subscription_to_json(sub, build_event_source(sub)));
  }

  json response;
  response["items"] = items;
  HandlerContext::send_json(res, response);
}

// ---------------------------------------------------------------------------
// GET — get single subscription
// ---------------------------------------------------------------------------
void CyclicSubscriptionHandlers::handle_get(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto sub_id = req.matches[2].str();
  auto sub = sub_mgr_.get(sub_id);
  if (!sub || sub->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  HandlerContext::send_json(res, subscription_to_json(*sub, build_event_source(*sub)));
}

// ---------------------------------------------------------------------------
// PUT — update subscription
// ---------------------------------------------------------------------------
void CyclicSubscriptionHandlers::handle_update(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto sub_id = req.matches[2].str();

  // Parse JSON body
  json body;
  try {
    body = json::parse(req.body);
  } catch (const json::exception &) {
    HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON request body");
    return;
  }

  // Parse optional interval
  std::optional<CyclicInterval> new_interval;
  if (body.contains("interval") && body["interval"].is_string()) {
    try {
      new_interval = parse_interval(body["interval"].get<std::string>());
    } catch (const std::invalid_argument &) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                                 "Invalid interval. Must be 'fast', 'normal', or 'slow'.",
                                 {{"parameter", "interval"}, {"value", body["interval"]}});
      return;
    }
  }

  // Parse optional duration
  std::optional<int> new_duration;
  if (body.contains("duration") && body["duration"].is_number_integer()) {
    new_duration = body["duration"].get<int>();
    if (*new_duration <= 0) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                                 "Duration must be a positive integer (seconds).",
                                 {{"parameter", "duration"}, {"value", *new_duration}});
      return;
    }
  }

  // Verify subscription exists and belongs to this entity before updating
  auto existing = sub_mgr_.get(sub_id);
  if (!existing || existing->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  auto result = sub_mgr_.update(sub_id, new_interval, new_duration);
  if (!result) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  HandlerContext::send_json(res, subscription_to_json(*result, build_event_source(*result)));
}

// ---------------------------------------------------------------------------
// DELETE — remove subscription
// ---------------------------------------------------------------------------
void CyclicSubscriptionHandlers::handle_delete(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto sub_id = req.matches[2].str();

  // Verify subscription exists and belongs to this entity before deleting
  auto existing = sub_mgr_.get(sub_id);
  if (!existing || existing->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  if (!sub_mgr_.remove(sub_id)) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  res.status = 204;
}

// ---------------------------------------------------------------------------
// GET /events — SSE stream
// ---------------------------------------------------------------------------
void CyclicSubscriptionHandlers::handle_events(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto sub_id = req.matches[2].str();
  auto sub = sub_mgr_.get(sub_id);
  if (!sub) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  // Verify subscription belongs to this entity
  if (sub->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  // Reject SSE connections for expired or inactive subscriptions
  if (!sub_mgr_.is_active(sub_id)) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND,
                               "Subscription expired or inactive", {{"subscription_id", sub_id}});
    return;
  }

  // Check combined SSE client limit (shared with fault streams)
  if (!client_tracker_->try_connect()) {
    RCLCPP_WARN(HandlerContext::logger(),
                "SSE client limit reached (%zu), rejecting cyclic subscription stream from %s",
                client_tracker_->max_clients(), req.remote_addr.c_str());
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE,
                               "Maximum number of SSE clients reached. Please try again later.");
    return;
  }

  RCLCPP_INFO(HandlerContext::logger(), "SSE cyclic subscription client connected from %s (%zu/%zu)",
              req.remote_addr.c_str(), client_tracker_->connected_clients(), client_tracker_->max_clients());

  // Set SSE headers (Content-Type set by set_chunked_content_provider below)
  res.set_header("Cache-Control", "no-cache");
  res.set_header("Connection", "keep-alive");
  res.set_header("X-Accel-Buffering", "no");

  auto captured_sub_id = sub->id;
  auto captured_topic = sub->topic_name;

  res.set_chunked_content_provider(
      "text/event-stream",
      [this, captured_sub_id, captured_topic](size_t /*offset*/, httplib::DataSink & sink) {
        auto keepalive_timeout = std::chrono::seconds(kKeepaliveIntervalSec);
        auto last_write = std::chrono::steady_clock::now();

        while (true) {
          // Check if subscription is still active
          if (!sub_mgr_.is_active(captured_sub_id)) {
            return false;  // Subscription expired or removed
          }

          // Send keepalive if no data was written recently (e.g. after a slow sample_topic)
          auto since_last_write = std::chrono::steady_clock::now() - last_write;
          if (since_last_write >= keepalive_timeout) {
            const char * keepalive = ":keepalive\n\n";
            if (!sink.write(keepalive, std::strlen(keepalive))) {
              return false;  // Client disconnected
            }
            last_write = std::chrono::steady_clock::now();
          }

          // Get current interval from subscription (may have been updated)
          auto current_sub = sub_mgr_.get(captured_sub_id);
          if (!current_sub) {
            return false;  // Removed
          }

          auto sample_interval = interval_to_duration(current_sub->interval);

          // Sample the topic
          auto * data_access_mgr = ctx_.node()->get_data_access_manager();
          auto * native_sampler = data_access_mgr->get_native_sampler();
          auto sample = native_sampler->sample_topic(captured_topic, data_access_mgr->get_topic_sample_timeout());

          // Build EventEnvelope
          json envelope;

          // UTC timestamp in ISO 8601 format
          auto now = std::chrono::system_clock::now();
          auto time_t = std::chrono::system_clock::to_time_t(now);
          auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
          std::ostringstream ts;
          struct tm tm_buf;
          gmtime_r(&time_t, &tm_buf);
          ts << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%S");
          ts << "." << std::setw(3) << std::setfill('0') << ms << "Z";
          envelope["timestamp"] = ts.str();

          if (sample.has_data && sample.data.has_value()) {
            json payload;
            payload["id"] = captured_topic;
            payload["data"] = *sample.data;
            envelope["payload"] = payload;
          } else {
            // No data available — send error event
            json error;
            error["error_code"] = ERR_X_MEDKIT_ROS2_TOPIC_UNAVAILABLE;
            error["message"] = "Topic data not available: " + captured_topic;
            envelope["error"] = error;
          }

          // Format SSE data frame
          std::string sse_msg = "data: " + envelope.dump() + "\n\n";
          if (!sink.write(sse_msg.data(), sse_msg.size())) {
            return false;  // Client disconnected
          }
          last_write = std::chrono::steady_clock::now();

          // Wait for interval or notification (update/delete/shutdown)
          sub_mgr_.wait_for_update(captured_sub_id, sample_interval);
        }

        return true;
      },
      [this, captured_sub_id](bool /*success*/) {
        client_tracker_->disconnect();
        RCLCPP_DEBUG(rclcpp::get_logger("rest_server"), "SSE cyclic subscription stream disconnected: %s",
                     captured_sub_id.c_str());
      });
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
json CyclicSubscriptionHandlers::subscription_to_json(const CyclicSubscriptionInfo & info,
                                                      const std::string & event_source) {
  json j;
  j["id"] = info.id;
  j["observed_resource"] = info.resource_uri;
  j["event_source"] = event_source;
  j["protocol"] = info.protocol;
  j["interval"] = interval_to_string(info.interval);
  return j;
}

std::string CyclicSubscriptionHandlers::build_event_source(const CyclicSubscriptionInfo & info) {
  return std::string(API_BASE_PATH) + "/" + info.entity_type + "/" + info.entity_id + "/cyclic-subscriptions/" +
         info.id + "/events";
}

std::string CyclicSubscriptionHandlers::extract_entity_type(const httplib::Request & req) {
  // Path is like /api/v1/apps/{id}/cyclic-subscriptions or /api/v1/components/{id}/cyclic-subscriptions
  if (req.path.find("/apps/") != std::string::npos) {
    return "apps";
  }
  if (req.path.find("/components/") != std::string::npos) {
    return "components";
  }
  return "apps";  // Default fallback
}

tl::expected<std::string, std::string> CyclicSubscriptionHandlers::parse_resource_uri(const std::string & resource) {
  // Expected format: /api/v1/{entity_type}/{entity_id}/data/{topic_path}
  // We need to extract the topic path (everything after /data/)
  static const std::regex resource_regex(R"(^/api/v1/(?:apps|components)/[^/]+/data(/.*))");
  std::smatch match;
  if (!std::regex_match(resource, match, resource_regex)) {
    return tl::make_unexpected("Resource URI must match /api/v1/{apps|components}/{id}/data/{topic}");
  }
  return match[1].str();
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
