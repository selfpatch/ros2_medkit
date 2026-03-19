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

#include "ros2_medkit_gateway/http/handlers/trigger_handlers.hpp"

#include <chrono>
#include <regex>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/models/entity_types.hpp"

using json = nlohmann::json;

namespace ros2_medkit_gateway {
namespace handlers {

TriggerHandlers::TriggerHandlers(HandlerContext & ctx, TriggerManager & trigger_mgr,
                                 std::shared_ptr<SSEClientTracker> client_tracker)
  : ctx_(ctx), trigger_mgr_(trigger_mgr), client_tracker_(std::move(client_tracker)) {
}

// ---------------------------------------------------------------------------
// POST - create trigger
// ---------------------------------------------------------------------------
void TriggerHandlers::handle_create(const httplib::Request & req, httplib::Response & res) {
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

  if (!body.contains("trigger_condition") || !body["trigger_condition"].is_object()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing or invalid 'trigger_condition'",
                               {{"parameter", "trigger_condition"}});
    return;
  }

  auto trigger_condition = body["trigger_condition"];
  if (!trigger_condition.contains("condition_type") || !trigger_condition["condition_type"].is_string()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER,
                               "Missing or invalid 'condition_type' in trigger_condition",
                               {{"parameter", "trigger_condition.condition_type"}});
    return;
  }

  std::string condition_type = trigger_condition["condition_type"].get<std::string>();

  // Extract condition_params (everything in trigger_condition except condition_type)
  json condition_params = trigger_condition;
  condition_params.erase("condition_type");

  // Parse resource URI
  std::string resource = body["resource"].get<std::string>();
  auto parsed = parse_resource_uri(resource);
  if (!parsed) {
    HandlerContext::send_error(res, 400, ERR_X_MEDKIT_INVALID_RESOURCE_URI, "Invalid resource URI: " + parsed.error(),
                               {{"parameter", "resource"}, {"value", resource}});
    return;
  }

  // Validate resource URI references the same entity as the route
  std::string entity_type = extract_entity_type(req);
  if (parsed->entity_type != entity_type || parsed->entity_id != entity_id) {
    HandlerContext::send_error(res, 400, ERR_X_MEDKIT_ENTITY_MISMATCH,
                               "Resource URI must reference the same entity as the route",
                               {{"parameter", "resource"}, {"value", resource}});
    return;
  }

  // Validate collection
  static const std::unordered_set<std::string> known_collections = {"data",    "faults", "operations", "configurations",
                                                                    "updates", "logs",   "bulk-data"};
  if (known_collections.find(parsed->collection) == known_collections.end() &&
      parsed->collection.substr(0, 2) != "x-") {
    HandlerContext::send_error(
        res, 400, ERR_INVALID_PARAMETER,
        "Unknown collection. Supported: data, faults, operations, configurations, updates, logs, bulk-data, or x-* "
        "vendor extensions",
        {{"parameter", "resource"}, {"collection", parsed->collection}});
    return;
  }

  // Parse optional fields
  std::string path;
  if (body.contains("path") && body["path"].is_string()) {
    path = body["path"].get<std::string>();
  }

  // Validate JSON Pointer path
  if (!path.empty()) {
    if (path.size() > 1024) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Path too long (max 1024)", {{"parameter", "path"}});
      return;
    }
    try {
      (void)nlohmann::json::json_pointer(path);
    } catch (const nlohmann::json::exception &) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid JSON Pointer in 'path'",
                                 {{"parameter", "path"}, {"value", path}});
      return;
    }
  }

  std::string protocol = "sse";
  if (body.contains("protocol") && body["protocol"].is_string()) {
    protocol = body["protocol"].get<std::string>();
  }

  // Validate protocol
  if (protocol != "sse") {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Unsupported protocol. Supported: 'sse'",
                               {{"parameter", "protocol"}, {"value", protocol}});
    return;
  }

  bool multishot = false;
  if (body.contains("multishot") && body["multishot"].is_boolean()) {
    multishot = body["multishot"].get<bool>();
  }

  bool persistent = false;
  if (body.contains("persistent") && body["persistent"].is_boolean()) {
    persistent = body["persistent"].get<bool>();
  }

  std::optional<int> lifetime_sec;
  if (body.contains("lifetime") && body["lifetime"].is_number_integer()) {
    lifetime_sec = body["lifetime"].get<int>();
    if (*lifetime_sec <= 0) {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Lifetime must be a positive integer (seconds)",
                                 {{"parameter", "lifetime"}, {"value", *lifetime_sec}});
      return;
    }
  }

  std::optional<json> log_settings;
  if (body.contains("log_settings") && body["log_settings"].is_object()) {
    log_settings = body["log_settings"];
  }

  // For data triggers, resolve the resource_path URI segment to a full ROS 2 topic name.
  // The resource_path from parse_resource_uri is e.g. "/temperature" (URI segment), but the
  // ROS 2 topic name is e.g. "/sensor/temperature". We look up the entity's topics from the
  // cache and find the one whose name ends with the resource_path.
  std::string resolved_topic_name;
  if (parsed->collection == "data" && !parsed->resource_path.empty()) {
    const auto & cache = ctx_.node()->get_thread_safe_cache();
    auto aggregated = cache.get_entity_data(entity_id);
    for (const auto & topic : aggregated.topics) {
      // Match: topic.name ends with the resource_path (e.g., topic "/sensor/temperature"
      // ends with "/temperature"). Also accept exact match.
      if (topic.name == parsed->resource_path ||
          (topic.name.size() > parsed->resource_path.size() &&
           topic.name.compare(topic.name.size() - parsed->resource_path.size(), parsed->resource_path.size(),
                              parsed->resource_path) == 0)) {
        resolved_topic_name = topic.name;
        break;
      }
    }
    if (resolved_topic_name.empty()) {
      RCLCPP_WARN(HandlerContext::logger(),
                  "Could not resolve resource_path '%s' to a ROS 2 topic for entity '%s'. "
                  "Topic subscription will be attempted when topic becomes available.",
                  parsed->resource_path.c_str(), entity_id.c_str());
    }
  }

  // Build create request
  TriggerCreateRequest create_req;
  create_req.entity_id = entity_id;
  create_req.entity_type = entity_type;
  create_req.resource_uri = resource;
  create_req.collection = parsed->collection;
  create_req.resource_path = parsed->resource_path;
  create_req.resolved_topic_name = resolved_topic_name;
  create_req.path = path;
  create_req.condition_type = condition_type;
  create_req.condition_params = condition_params;
  create_req.protocol = protocol;
  create_req.multishot = multishot;
  create_req.persistent = persistent;
  create_req.lifetime_sec = lifetime_sec;
  create_req.log_settings = log_settings;

  auto result = trigger_mgr_.create(create_req);
  if (!result) {
    // Distinguish between validation errors (400) and capacity errors (503).
    // NOTE: String matching on TriggerManager error messages - coupled to the exact
    // error text in trigger_manager.cpp::create(). If those strings change, update here.
    const auto & err = result.error();
    if (err.find("Maximum trigger capacity") != std::string::npos) {
      HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, err);
    } else {
      HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, err, {{"parameter", "trigger_condition"}});
    }
    return;
  }

  auto event_source = build_event_source(*result);
  auto response_json = trigger_to_json(*result, event_source);

  res.status = 201;
  HandlerContext::send_json(res, response_json);
}

// ---------------------------------------------------------------------------
// GET - list triggers
// ---------------------------------------------------------------------------
void TriggerHandlers::handle_list(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto triggers = trigger_mgr_.list(entity_id);
  json items = json::array();
  for (const auto & trig : triggers) {
    items.push_back(trigger_to_json(trig, build_event_source(trig)));
  }

  json response;
  response["items"] = items;
  HandlerContext::send_json(res, response);
}

// ---------------------------------------------------------------------------
// GET - get single trigger
// ---------------------------------------------------------------------------
void TriggerHandlers::handle_get(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto trigger_id = req.matches[2].str();
  auto trig = trigger_mgr_.get(trigger_id);
  if (!trig || trig->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", {{"trigger_id", trigger_id}});
    return;
  }

  HandlerContext::send_json(res, trigger_to_json(*trig, build_event_source(*trig)));
}

// ---------------------------------------------------------------------------
// PUT - update trigger
// ---------------------------------------------------------------------------
void TriggerHandlers::handle_update(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto trigger_id = req.matches[2].str();

  // Parse JSON body
  json body;
  try {
    body = json::parse(req.body);
  } catch (const json::exception &) {
    HandlerContext::send_error(res, 400, ERR_INVALID_REQUEST, "Invalid JSON request body");
    return;
  }

  if (!body.contains("lifetime") || !body["lifetime"].is_number_integer()) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Missing or invalid 'lifetime'",
                               {{"parameter", "lifetime"}});
    return;
  }

  int new_lifetime = body["lifetime"].get<int>();

  if (new_lifetime <= 0) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, "Lifetime must be a positive integer",
                               {{"parameter", "lifetime"}, {"value", new_lifetime}});
    return;
  }

  // Verify trigger exists and belongs to this entity before updating
  auto existing = trigger_mgr_.get(trigger_id);
  if (!existing || existing->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", {{"trigger_id", trigger_id}});
    return;
  }

  auto result = trigger_mgr_.update(trigger_id, new_lifetime);
  if (!result) {
    HandlerContext::send_error(res, 400, ERR_INVALID_PARAMETER, result.error(),
                               {{"parameter", "lifetime"}, {"value", new_lifetime}});
    return;
  }

  HandlerContext::send_json(res, trigger_to_json(*result, build_event_source(*result)));
}

// ---------------------------------------------------------------------------
// DELETE - remove trigger
// ---------------------------------------------------------------------------
void TriggerHandlers::handle_delete(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto trigger_id = req.matches[2].str();

  // Verify trigger exists and belongs to this entity before deleting
  auto existing = trigger_mgr_.get(trigger_id);
  if (!existing || existing->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", {{"trigger_id", trigger_id}});
    return;
  }

  if (!trigger_mgr_.remove(trigger_id)) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", {{"trigger_id", trigger_id}});
    return;
  }

  res.status = 204;
}

// ---------------------------------------------------------------------------
// GET /events - SSE stream
// ---------------------------------------------------------------------------
void TriggerHandlers::handle_events(const httplib::Request & req, httplib::Response & res) {
  auto entity_id = req.matches[1].str();
  auto entity = ctx_.validate_entity_for_route(req, res, entity_id);
  if (!entity) {
    return;
  }

  auto trigger_id = req.matches[2].str();
  auto trig = trigger_mgr_.get(trigger_id);
  if (!trig) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", {{"trigger_id", trigger_id}});
    return;
  }

  if (trig->entity_id != entity_id) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Trigger not found", {{"trigger_id", trigger_id}});
    return;
  }

  if (!trigger_mgr_.is_active(trigger_id)) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "Trigger expired or inactive",
                               {{"trigger_id", trigger_id}});
    return;
  }

  if (!client_tracker_->try_connect()) {
    HandlerContext::send_error(res, 503, ERR_SERVICE_UNAVAILABLE, "Maximum SSE client limit reached");
    return;
  }

  // SSE headers for proxy compatibility (matches sse_fault_handler pattern)
  res.set_header("Cache-Control", "no-cache");
  res.set_header("X-Accel-Buffering", "no");

  auto tracker = client_tracker_;
  // NOTE: trigger_mgr_ is captured by reference. TriggerHandlers must outlive all SSE
  // connections. This is guaranteed because GatewayNode destroys the REST server
  // (closing all connections) before destroying managers. See shutdown lifecycle in design spec.
  auto & mgr = trigger_mgr_;
  auto tid = trigger_id;

  res.set_chunked_content_provider(
      "text/event-stream",
      [&mgr, tid, tracker](size_t /*offset*/, httplib::DataSink & sink) -> bool {
        while (mgr.is_active(tid)) {
          // Wait for event or timeout (15 seconds for keepalive)
          bool woken = mgr.wait_for_event(tid, std::chrono::milliseconds(15000));

          if (!mgr.is_active(tid)) {
            break;
          }

          if (!woken) {
            // Timeout - send keepalive
            if (!sink.write(":keepalive\n\n", 12)) {
              break;
            }
            continue;
          }

          // Drain all pending events per wakeup (C2 fix: bounded queue can
          // accumulate multiple events between wakeups for multishot triggers)
          bool write_ok = true;
          while (auto event = mgr.consume_pending_event(tid)) {
            std::string frame = "data: " + event->dump() + "\n\n";
            if (!sink.write(frame.c_str(), frame.size())) {
              write_ok = false;
              break;
            }
          }
          if (!write_ok) {
            break;
          }
        }

        sink.done();
        return true;
      },
      [tracker](bool /*success*/) {
        tracker->disconnect();
      });
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
json TriggerHandlers::trigger_to_json(const TriggerInfo & info, const std::string & event_source) {
  json j;
  j["id"] = info.id;
  j["status"] = (info.status == TriggerStatus::ACTIVE) ? "active" : "terminated";
  j["observed_resource"] = info.resource_uri;
  j["event_source"] = event_source;
  j["protocol"] = info.protocol;

  json condition;
  condition["condition_type"] = info.condition_type;
  // Merge condition_params into the condition object
  if (info.condition_params.is_object()) {
    for (auto & [key, val] : info.condition_params.items()) {
      condition[key] = val;
    }
  }
  j["trigger_condition"] = condition;

  j["multishot"] = info.multishot;
  j["persistent"] = info.persistent;

  if (info.lifetime_sec.has_value()) {
    j["lifetime"] = info.lifetime_sec.value();
  }

  if (!info.path.empty()) {
    j["path"] = info.path;
  }

  return j;
}

std::string TriggerHandlers::build_event_source(const TriggerInfo & info) {
  return std::string(API_BASE_PATH) + "/" + info.entity_type + "/" + info.entity_id + "/triggers/" + info.id +
         "/events";
}

std::string TriggerHandlers::extract_entity_type(const httplib::Request & req) {
  auto type = extract_entity_type_from_path(req.path);
  switch (type) {
    case SovdEntityType::APP:
      return "apps";
    case SovdEntityType::COMPONENT:
      return "components";
    case SovdEntityType::AREA:
      return "areas";
    case SovdEntityType::FUNCTION:
      return "functions";
    default:
      RCLCPP_WARN(HandlerContext::logger(), "Unexpected entity type in trigger path: %s", req.path.c_str());
      return "apps";
  }
}

tl::expected<TriggerParsedResourceUri, std::string> TriggerHandlers::parse_resource_uri(const std::string & resource) {
  // Entity-scoped format: /api/v1/{entity_type}/{entity_id}/{collection}[/{resource_path}]
  // Includes areas in addition to apps/components/functions
  static const std::regex entity_regex(R"(^/api/v1/(areas|apps|components|functions)/([^/]+)/([^/]+)(/.*)?$)");
  std::smatch match;
  if (std::regex_match(resource, match, entity_regex)) {
    TriggerParsedResourceUri parsed;
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
          return tl::make_unexpected(std::string("Resource path must not contain '..' as a path segment"));
        }
        pos = (next == std::string::npos) ? path.size() : next;
      }
    }

    return parsed;
  }

  return tl::make_unexpected(
      std::string("Resource URI must match /api/v1/{areas|apps|components|functions}/{id}/{collection}[/{path}]"));
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
