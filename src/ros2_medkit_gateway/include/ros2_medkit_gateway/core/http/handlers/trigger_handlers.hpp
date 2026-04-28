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

#pragma once

#include <httplib.h>

#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

/// Result of parsing a SOVD resource URI for triggers (includes areas)
struct TriggerParsedResourceUri {
  std::string entity_type;  // "areas", "apps", "components", or "functions"
  std::string entity_id;
  std::string collection;     // "data", "faults", "x-medkit-metrics", etc.
  std::string resource_path;  // path within collection (may be empty)
};

/**
 * @brief HTTP handlers for trigger CRUD and SSE streaming.
 *
 * Implements SOVD trigger endpoints:
 * - POST /{entity}/triggers - create trigger
 * - GET /{entity}/triggers - list triggers
 * - GET /{entity}/triggers/{id} - get trigger
 * - PUT /{entity}/triggers/{id} - update trigger
 * - DELETE /{entity}/triggers/{id} - delete trigger
 * - GET /{entity}/triggers/{id}/events - SSE stream
 */
class TriggerHandlers {
 public:
  TriggerHandlers(HandlerContext & ctx, TriggerManager & trigger_mgr, std::shared_ptr<SSEClientTracker> client_tracker);

  /// POST /{entity}/triggers - create trigger
  void handle_create(const httplib::Request & req, httplib::Response & res);

  /// GET /{entity}/triggers - list all triggers for entity
  void handle_list(const httplib::Request & req, httplib::Response & res);

  /// GET /{entity}/triggers/{id} - get single trigger
  void handle_get(const httplib::Request & req, httplib::Response & res);

  /// PUT /{entity}/triggers/{id} - update trigger
  void handle_update(const httplib::Request & req, httplib::Response & res);

  /// DELETE /{entity}/triggers/{id} - delete trigger
  void handle_delete(const httplib::Request & req, httplib::Response & res);

  /// GET /{entity}/triggers/{id}/events - SSE event stream
  void handle_events(const httplib::Request & req, httplib::Response & res);

  /// Convert trigger info to JSON response
  static nlohmann::json trigger_to_json(const TriggerInfo & info, const std::string & event_source);

  /// Parse resource URI for triggers (includes areas in addition to apps/components/functions).
  static tl::expected<TriggerParsedResourceUri, std::string> parse_resource_uri(const std::string & resource);

 private:
  /// Build event_source URI from trigger info
  static std::string build_event_source(const TriggerInfo & info);

  /// Extract entity type string from request path
  static std::string extract_entity_type(const httplib::Request & req);

  HandlerContext & ctx_;
  TriggerManager & trigger_mgr_;
  std::shared_ptr<SSEClientTracker> client_tracker_;
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
