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

#include "ros2_medkit_gateway/http/handlers/docs_handlers.hpp"

#include "../../openapi/capability_generator.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"

namespace ros2_medkit_gateway {
namespace handlers {

DocsHandlers::DocsHandlers(HandlerContext & ctx, GatewayNode & node, PluginManager * plugin_mgr)
  : ctx_(ctx), generator_(std::make_unique<openapi::CapabilityGenerator>(ctx, node, plugin_mgr)), docs_enabled_(true) {
  // Read docs.enabled parameter if it has been declared
  try {
    docs_enabled_ = node.get_parameter("docs.enabled").as_bool();
  } catch (...) {
    docs_enabled_ = true;  // Default to enabled
  }
}

DocsHandlers::~DocsHandlers() = default;

void DocsHandlers::handle_docs_root(const httplib::Request & /*req*/, httplib::Response & res) {
  if (!docs_enabled_) {
    HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Capability description is disabled");
    return;
  }

  auto spec = generator_->generate("/");
  if (!spec) {
    HandlerContext::send_error(res, 500, ERR_INTERNAL_ERROR, "Failed to generate capability description");
    return;
  }
  HandlerContext::send_json(res, *spec);
}

void DocsHandlers::handle_docs_any_path(const httplib::Request & req, httplib::Response & res) {
  if (!docs_enabled_) {
    HandlerContext::send_error(res, 501, ERR_NOT_IMPLEMENTED, "Capability description is disabled");
    return;
  }

  auto base_path = req.matches[1].str();
  auto spec = generator_->generate(base_path);
  if (!spec) {
    HandlerContext::send_error(res, 404, ERR_RESOURCE_NOT_FOUND, "No capability description for path: " + base_path);
    return;
  }
  HandlerContext::send_json(res, *spec);
}

}  // namespace handlers
}  // namespace ros2_medkit_gateway
