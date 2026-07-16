// Copyright 2026 mfaferek93
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

#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

using namespace ros2_medkit_gateway;

/**
 * @brief Demo plugin mirroring the commercial PLC bridge shape (.so for tests).
 *
 * Deliberately does NOT export get_data_provider: like the S7comm/Modbus/ADS
 * bridges it exports only create_plugin + get_introspection_provider +
 * get_fault_provider and serves its live values exclusively through a
 * registered `apps/<id>/x-plc-data` route (poller-snapshot style response
 * with `connected` + `items[]`). Used by test_entity_freeze_frame.test.py to
 * prove the gateway's in-process route-dispatch fallback captures a
 * zero-config freeze-frame for such plugins.
 */
class TestRouteDataPlugin : public GatewayPlugin, public IntrospectionProvider, public FaultProvider {
 public:
  static constexpr const char * kAppId = "test_route_plc_app";

  std::string name() const override {
    return "test_route_data";
  }

  void configure(const nlohmann::json & /*config*/) override {
  }

  void set_context(PluginContext & context) override {
    ctx_ = &context;
  }

  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", R"(apps/([^/]+)/x-plc-data)",
         [this](const PluginRequest & req, PluginResponse & res) {
           handle_plc_data(req, res);
         }},
    };
  }

  // --- IntrospectionProvider ---
  IntrospectionResult introspect(const IntrospectionInput & /*input*/) override {
    IntrospectionResult result;

    Area area;
    area.id = "test_route_plc_area";
    area.name = "Test Route PLC Cell";
    area.namespace_path = "/test_route_plc_area";
    area.source = "plugin";
    result.new_entities.areas.push_back(std::move(area));

    Component comp;
    comp.id = "test_route_plc";
    comp.name = "Test Route PLC Runtime";
    comp.namespace_path = "/test_route_plc_area";
    comp.fqn = "/test_route_plc_area/test_route_plc";
    comp.area = "test_route_plc_area";
    comp.source = "plugin";
    result.new_entities.components.push_back(std::move(comp));

    App app;
    app.id = kAppId;
    app.name = "Test Route PLC Process";
    app.component_id = "test_route_plc";
    app.external = true;  // faults report under the bare app id, not a ROS FQN
    app.is_online = true;
    app.source = "plugin";
    result.new_entities.apps.push_back(std::move(app));

    return result;
  }

  // --- FaultProvider (same projection as test_data_provider_plugin: the list
  // mirrors the fault_manager's records; get_fault returns not-found so the
  // handler serves the enriched record with environment_data) ---
  tl::expected<dto::FaultListResult, FaultProviderErrorInfo> list_faults(const std::string & entity_id) override {
    nlohmann::json items = nlohmann::json::array();
    if (ctx_ != nullptr) {
      auto faults = ctx_->list_entity_faults(entity_id);
      if (faults.is_array()) {
        for (const auto & f : faults) {
          items.push_back({{"code", f.value("fault_code", "")},
                           {"severity", f.value("severity", 0)},
                           {"status", f.value("status", "")}});
        }
      }
    }
    return dto::FaultListResult{nlohmann::json{{"items", std::move(items)}}};
  }

  tl::expected<dto::FaultDetailResult, FaultProviderErrorInfo> get_fault(const std::string & /*entity_id*/,
                                                                         const std::string & fault_code) override {
    return tl::make_unexpected(
        FaultProviderErrorInfo{FaultProviderError::FaultNotFound, "Fault not found: " + fault_code, 404});
  }

  tl::expected<dto::FaultClearResult, FaultProviderErrorInfo> clear_fault(const std::string & /*entity_id*/,
                                                                          const std::string & fault_code) override {
    return dto::FaultClearResult{nlohmann::json{{"code", fault_code}, {"cleared", true}}};
  }

 private:
  void handle_plc_data(const PluginRequest & req, PluginResponse & res) {
    auto entity_id = req.path_param(1);
    if (ctx_ != nullptr) {
      // Same gate as the real PLC bridges: entity must exist and match the
      // route's entity type (exercises the synthesized request's path()).
      auto entity = ctx_->validate_entity_for_route(req, res, entity_id);
      if (!entity) {
        return;
      }
    }
    if (entity_id != kAppId) {
      res.send_error(404, ERR_RESOURCE_NOT_FOUND, "No PLC data mapped for entity: " + entity_id);
      return;
    }
    // Poller-snapshot style response, canned values (S7/ADS bridge shape).
    res.send_json({{"connected", true},
                   {"entity_id", entity_id},
                   {"items", nlohmann::json::array({{{"name", "level"}, {"value", 87.5}, {"unit", "mm"}},
                                                    {{"name", "alarm"}, {"value", true}}})},
                   {"timestamp", 1234567890}});
  }

  PluginContext * ctx_{nullptr};
};

// --- Plugin exports (intentionally NO get_data_provider) ---

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new TestRouteDataPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin) {
  return static_cast<TestRouteDataPlugin *>(plugin);
}

extern "C" GATEWAY_PLUGIN_EXPORT FaultProvider * get_fault_provider(GatewayPlugin * plugin) {
  return static_cast<TestRouteDataPlugin *>(plugin);
}
