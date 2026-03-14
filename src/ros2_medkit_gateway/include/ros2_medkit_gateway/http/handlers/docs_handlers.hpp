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

#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

namespace ros2_medkit_gateway {

class GatewayNode;
class PluginManager;

namespace openapi {
class CapabilityGenerator;
}  // namespace openapi

namespace handlers {

/**
 * @brief OpenAPI capability description endpoint handlers
 *
 * Handles:
 * - GET /docs - Root capability description (full OpenAPI spec)
 * - GET /{path}/docs - Context-scoped capability description
 */
class DocsHandlers {
 public:
  DocsHandlers(HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr);
  ~DocsHandlers();

  /// GET /docs - Root capability description
  void handle_docs_root(const httplib::Request & req, httplib::Response & res);

  /// GET /{path}/docs - Context-scoped capability description
  void handle_docs_any_path(const httplib::Request & req, httplib::Response & res);

 private:
  HandlerContext & ctx_;
  std::unique_ptr<openapi::CapabilityGenerator> generator_;
  bool docs_enabled_{true};
};

}  // namespace handlers
}  // namespace ros2_medkit_gateway
