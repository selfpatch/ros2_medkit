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

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/core/providers/update_provider.hpp"

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

using namespace ros2_medkit_gateway;

/**
 * @brief Test gateway plugin implementing all provider interfaces.
 *
 * Used by test_plugin_loader to verify dlopen/dlsym-based loading,
 * API version checking, and extern "C" provider query functions.
 *
 * Also demonstrates vendor extension endpoints via register_capability()
 * and entity-scoped routes. The plugin registers an "x-medkit-diagnostics"
 * capability on all Components and serves diagnostic data per entity.
 */
class TestGatewayPlugin : public GatewayPlugin, public UpdateProvider, public IntrospectionProvider {
 public:
  // --- GatewayPlugin ---
  std::string name() const override {
    return "test_plugin";
  }

  void configure(const nlohmann::json & /*config*/) override {
  }

  void set_context(PluginContext & context) override {
    ctx_ = &context;
    // Register vendor extension capability for all Components
    ctx_->register_capability(SovdEntityType::COMPONENT, "x-medkit-diagnostics");
  }

  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", "x-test/ping",
         [](const PluginRequest &, PluginResponse & res) {
           res.send_json({{"response", "pong"}});
         }},
        {"GET", R"(components/([^/]+)/x-medkit-diagnostics)",
         [this](const PluginRequest & req, PluginResponse & res) {
           auto entity_id = req.path_param(1);
           auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
           if (!entity) {
             return;
           }
           nlohmann::json data = {{"entity_id", entity->id},
                                  {"plugin", "test_plugin"},
                                  {"cpu_usage", 42.5},
                                  {"memory_mb", 128},
                                  {"uptime_seconds", 3600}};
           res.send_json(data);
         }},
    };
  }

 private:
  PluginContext * ctx_{nullptr};

  // --- UpdateProvider ---
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter &) override {
    return std::vector<std::string>{};
  }

  tl::expected<nlohmann::json, UpdateBackendErrorInfo> get_update(const std::string & id) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found: " + id});
  }

  tl::expected<void, UpdateBackendErrorInfo> register_update(const nlohmann::json &) override {
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string &) override {
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string &, UpdateProgressReporter &) override {
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string &, UpdateProgressReporter &) override {
    return {};
  }

  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string &) override {
    return false;
  }

  // --- IntrospectionProvider ---
  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }
};

// --- Plugin exports (with visibility macros) ---

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new TestGatewayPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT UpdateProvider * get_update_provider(GatewayPlugin * plugin) {
  return static_cast<TestGatewayPlugin *>(plugin);
}

extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin) {
  return static_cast<TestGatewayPlugin *>(plugin);
}
