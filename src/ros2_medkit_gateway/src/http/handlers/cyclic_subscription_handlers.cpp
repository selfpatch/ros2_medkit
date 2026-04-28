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

#include <regex>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/models/entity_types.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

CyclicSubscriptionHandlers::CyclicSubscriptionHandlers(HandlerContext & ctx, SubscriptionManager & sub_mgr,
                                                       ResourceSamplerRegistry & sampler_registry,
                                                       TransportRegistry & transport_registry, int max_duration_sec)
  : ctx_(ctx)
  , sub_mgr_(sub_mgr)
  , sampler_registry_(sampler_registry)
  , transport_registry_(transport_registry)
  , max_duration_sec_(max_duration_sec) {
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
  }

  // Check transport is registered
  auto * transport = transport_registry_.get_transport(protocol);
  if (!transport) {
    HandlerContext::send_error(res, 400, ERR_X_MEDKIT_UNSUPPORTED_PROTOCOL, "Protocol '" + protocol + "' not available",
                               {{"parameter", "protocol"}, {"value", protocol}});
    return;
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
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Duration must be a positive integer (seconds).",
                               {{"parameter", "duration"}, {"value", duration}});
    return;
  }
  if (duration > max_duration_sec_) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                               "Duration must not exceed " + std::to_string(max_duration_sec_) + " seconds.",
                               {{"parameter", "duration"}, {"max_value", max_duration_sec_}});
    return;
  }

  // Parse resource URI to extract collection and resource path
  std::string resource = body["resource"].get<std::string>();
  auto parsed = parse_resource_uri(resource);
  if (!parsed) {
    HandlerContext::send_error(res, 400, ERR_X_MEDKIT_INVALID_RESOURCE_URI, "Invalid resource URI: " + parsed.error(),
                               {{"parameter", "resource"}, {"value", resource}});
    return;
  }

  std::string entity_type = extract_entity_type(req);

  // Server-level resources (e.g. updates) skip entity-mismatch and collection checks
  bool is_server_level = parsed->entity_type.empty();

  if (!is_server_level) {
    // Validate resource URI references the same entity as the route
    if (parsed->entity_type != entity_type || parsed->entity_id != entity_id) {
      HandlerContext::send_error(res, 400, ERR_X_MEDKIT_ENTITY_MISMATCH,
                                 "Resource URI must reference the same entity as the route",
                                 {{"parameter", "resource"}, {"value", resource}});
      return;
    }

    // Validate collection support
    auto known_collection = parse_resource_collection(parsed->collection);
    if (known_collection.has_value()) {
      // Known SOVD collection - check entity supports it
      if (!entity->supports_collection(*known_collection)) {
        HandlerContext::send_error(res, 400, ERR_X_MEDKIT_COLLECTION_NOT_SUPPORTED,
                                   "Collection '" + parsed->collection + "' not supported for " + entity_type,
                                   {{"collection", parsed->collection}, {"entity_type", entity_type}});
        return;
      }
    } else if (parsed->collection.size() < 2 || parsed->collection.substr(0, 2) != "x-") {
      // Not a known collection and not a vendor extension
      HandlerContext::send_error(res, 400, ERR_X_MEDKIT_INVALID_RESOURCE_URI,
                                 "Unknown collection '" + parsed->collection +
                                     "'. Use a known SOVD collection or 'x-' vendor extension.",
                                 {{"collection", parsed->collection}});
      return;
    }

    // Data collection requires a resource path (topic name)
    if (parsed->collection == "data" && parsed->resource_path.empty()) {
      HandlerContext::send_error(res, 400, ERR_X_MEDKIT_INVALID_RESOURCE_URI,
                                 "Data collection requires a resource path (e.g. /api/v1/apps/{id}/data/{topic})",
                                 {{"parameter", "resource"}, {"value", resource}});
      return;
    }
  }

  // Check sampler is registered
  if (!sampler_registry_.has_sampler(parsed->collection)) {
    HandlerContext::send_error(res, 400, ERR_X_MEDKIT_COLLECTION_NOT_AVAILABLE,
                               "No data provider for collection '" + parsed->collection + "'",
                               {{"collection", parsed->collection}});
    return;
  }

  // Create subscription
  auto result = sub_mgr_.create(entity_id, entity_type, resource, parsed->collection, parsed->resource_path, protocol,
                                interval, duration);
  if (!result) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, result.error());
    return;
  }

  // Start transport delivery
  auto sampler = sampler_registry_.get_sampler(parsed->collection);
  auto event_source_result = transport->start(*result, *sampler, ctx_.node());
  if (!event_source_result) {
    sub_mgr_.remove(result->id);
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, event_source_result.error());
    return;
  }

  auto response_json = subscription_to_json(*result, *event_source_result);

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
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Duration must be a positive integer (seconds).",
                                 {{"parameter", "duration"}, {"value", *new_duration}});
      return;
    }
    if (*new_duration > max_duration_sec_) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                                 "Duration must not exceed " + std::to_string(max_duration_sec_) + " seconds.",
                                 {{"parameter", "duration"}, {"max_value", max_duration_sec_}});
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
// GET /events — SSE stream (delegates to transport provider)
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

  if (sub->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription not found",
                               {{"subscription_id", sub_id}});
    return;
  }

  if (!sub_mgr_.is_active(sub_id)) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription expired or inactive",
                               {{"subscription_id", sub_id}});
    return;
  }

  auto * transport = transport_registry_.get_transport(sub->protocol);
  if (!transport) {
    HandlerContext::send_error(res, 400, ERR_X_MEDKIT_UNSUPPORTED_PROTOCOL,
                               "Transport for protocol '" + sub->protocol + "' not available",
                               {{"protocol", sub->protocol}});
    return;
  }

  if (!transport->handle_client_connect(sub_id, req, res)) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Subscription stream not found",
                               {{"subscription_id", sub_id}});
  }
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
  auto type = extract_entity_type_from_path(req.path);
  switch (type) {
    case SovdEntityType::APP:
      return "apps";
    case SovdEntityType::COMPONENT:
      return "components";
    case SovdEntityType::FUNCTION:
      return "functions";
    case SovdEntityType::SERVER:
    case SovdEntityType::AREA:
    case SovdEntityType::UNKNOWN:
    default:
      RCLCPP_WARN(HandlerContext::logger(), "Unexpected entity type in cyclic subscription path: %s", req.path.c_str());
      return "apps";
  }
}

tl::expected<ParsedResourceUri, std::string>
CyclicSubscriptionHandlers::parse_resource_uri(const std::string & resource) {
  // Try entity-scoped format first: /api/v1/{entity_type}/{entity_id}/{collection}[/{resource_path}]
  static const std::regex entity_regex(R"(^/api/v1/(apps|components|functions)/([^/]+)/([^/]+)(/.*)?$)");
  std::smatch match;
  if (std::regex_match(resource, match, entity_regex)) {
    ParsedResourceUri parsed;
    parsed.entity_type = match[1].str();
    parsed.entity_id = match[2].str();
    parsed.collection = match[3].str();
    parsed.resource_path = match[4].matched ? match[4].str() : "";

    // Security: reject '..' as a path segment (not as substring of e.g. '/..foo')
    if (!parsed.resource_path.empty()) {
      std::string path = parsed.resource_path;
      size_t pos = 0;
      while (pos < path.size()) {
        size_t next = path.find('/', pos + 1);
        std::string segment = (next == std::string::npos) ? path.substr(pos) : path.substr(pos, next - pos);
        if (segment == "/.." || segment == "..") {
          return tl::make_unexpected("Resource path must not contain '..' as a path segment");
        }
        pos = (next == std::string::npos) ? path.size() : next;
      }
    }

    return parsed;
  }

  // Try server-level update status: /api/v1/updates/{update-package-id}/status
  static const std::regex update_status_regex(R"(^/api/v1/updates/([^/]+)/status$)");
  if (std::regex_match(resource, match, update_status_regex)) {
    ParsedResourceUri parsed;
    parsed.entity_type = "";  // server-level, no entity
    parsed.entity_id = "";
    parsed.collection = "updates";
    parsed.resource_path = match[1].str();  // update package ID
    return parsed;
  }

  return tl::make_unexpected(
      "Resource URI must match /api/v1/{apps|components|functions}/{id}/{collection}[/{path}] "
      "or /api/v1/updates/{id}/status");
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
