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

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/core/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"

using namespace ros2_medkit_gateway;

/**
 * @brief Demo plugin mimicking a PLC protocol bridge (.so for integration tests).
 *
 * Introspects one external app entity ("test_plc_app") whose current values are
 * served by DataProvider::list_data with per-item "value" fields, like the
 * OPC-UA plugin. Used by test_entity_freeze_frame.test.py to prove a fault
 * reported under the app's bare id carries a zero-config freeze-frame of the
 * entity's own values.
 */
class TestDataProviderPlugin : public GatewayPlugin,
                               public IntrospectionProvider,
                               public DataProvider,
                               public FaultProvider {
 public:
  static constexpr const char * kAppId = "test_plc_app";

  std::string name() const override {
    return "test_data_provider";
  }

  void configure(const nlohmann::json & /*config*/) override {
  }

  void set_context(PluginContext & context) override {
    ctx_ = &context;
  }

  // --- IntrospectionProvider ---
  IntrospectionResult introspect(const IntrospectionInput & /*input*/) override {
    IntrospectionResult result;

    Area area;
    area.id = "test_plc_area";
    area.name = "Test PLC Cell";
    area.namespace_path = "/test_plc_area";
    area.source = "plugin";
    result.new_entities.areas.push_back(std::move(area));

    Component comp;
    comp.id = "test_plc";
    comp.name = "Test PLC Runtime";
    comp.namespace_path = "/test_plc_area";
    comp.fqn = "/test_plc_area/test_plc";
    comp.area = "test_plc_area";
    comp.source = "plugin";
    result.new_entities.components.push_back(std::move(comp));

    App app;
    app.id = kAppId;
    app.name = "Test PLC Process";
    app.component_id = "test_plc";
    app.external = true;  // faults report under the bare app id, not a ROS FQN
    app.is_online = true;
    app.source = "plugin";
    result.new_entities.apps.push_back(std::move(app));

    return result;
  }

  // --- DataProvider ---
  tl::expected<dto::DataListResult, DataProviderErrorInfo> list_data(const std::string & entity_id) override {
    if (entity_id != kAppId) {
      return tl::make_unexpected(
          DataProviderErrorInfo{DataProviderError::EntityNotFound, "No data for entity: " + entity_id, 404});
    }
    nlohmann::json items = nlohmann::json::array();
    items.push_back({{"id", "temperature"},
                     {"name", "Temperature"},
                     {"category", "currentData"},
                     {"value", 42.5},
                     {"unit", "degC"}});
    items.push_back(
        {{"id", "pressure"}, {"name", "Pressure"}, {"category", "currentData"}, {"value", 3.2}, {"unit", "bar"}});
    return dto::DataListResult{nlohmann::json{{"items", std::move(items)}}};
  }

  tl::expected<dto::DataValue, DataProviderErrorInfo> read_data(const std::string & entity_id,
                                                                const std::string & resource_name) override {
    auto list = list_data(entity_id);
    if (!list) {
      return tl::make_unexpected(list.error());
    }
    for (const auto & item : list->content["items"]) {
      if (item.value("id", "") == resource_name) {
        return dto::DataValue{nlohmann::json{{"id", resource_name}, {"value", item["value"]}}};
      }
    }
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::ResourceNotFound, "No such resource: " + resource_name, 404});
  }

  tl::expected<dto::DataWriteResult, DataProviderErrorInfo> write_data(const std::string & /*entity_id*/,
                                                                       const std::string & resource_name,
                                                                       const nlohmann::json & /*value*/) override {
    return tl::make_unexpected(
        DataProviderErrorInfo{DataProviderError::ReadOnly, "Resource is read-only: " + resource_name, 405});
  }

  // --- FaultProvider (the fault_manager owns this entity's faults; the list
  // projects them via PluginContext like the OPC-UA plugin, and get_fault
  // returns not-found so the handler serves the fault_manager's enriched
  // record with environment_data) ---
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
  PluginContext * ctx_{nullptr};
};

// --- Plugin exports ---

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new TestDataProviderPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT IntrospectionProvider * get_introspection_provider(GatewayPlugin * plugin) {
  return static_cast<TestDataProviderPlugin *>(plugin);
}

extern "C" GATEWAY_PLUGIN_EXPORT DataProvider * get_data_provider(GatewayPlugin * plugin) {
  return static_cast<TestDataProviderPlugin *>(plugin);
}

extern "C" GATEWAY_PLUGIN_EXPORT FaultProvider * get_fault_provider(GatewayPlugin * plugin) {
  return static_cast<TestDataProviderPlugin *>(plugin);
}
