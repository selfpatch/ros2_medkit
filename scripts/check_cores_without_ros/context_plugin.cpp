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

// A plugin with no ROS dependency. The check loads it as a shared object through the core PluginManager.

#include <atomic>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

using namespace ros2_medkit_gateway;

namespace {

class ContextPlugin : public GatewayPlugin, public DataProvider {
 public:
  std::string name() const override {
    return "check_context_plugin";
  }

  void configure(const nlohmann::json & /*config*/) override {
  }

  void set_context(PluginContext & context) override {
    ctx_ = &context;
    // The cast is valid on every host: a host without a ROS node gives a null node().
    ros_ctx_ = as_ros_plugin_context(context);
    ctx_->register_capability(SovdEntityType::APP, "x-check-context");
  }

  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", "x-check/context",
         [this](const PluginRequest &, PluginResponse & res) {
           res.send_json(describe_context());
         }},
        {"GET", "x-check/set",
         [this](const PluginRequest & req, PluginResponse & res) {
           value_ = std::stoi(req.query_param("value"));
           res.send_json({{"value", value_.load()}});
         }},
        {"GET", R"(apps/([^/]+)/x-check-context)",
         [this](const PluginRequest & req, PluginResponse & res) {
           auto entity = ctx_->validate_entity_for_route(req, res, req.path_param(1));
           if (!entity) {
             return;
           }
           res.send_json({{"entity_id", entity->id}, {"capabilities", ctx_->get_entity_capabilities(entity->id)}});
         }},
        // The route the core reads the values of a plugin-served entity from.
        {"GET", R"(apps/([^/]+)/x-plc-data)",
         [this](const PluginRequest &, PluginResponse & res) {
           res.send_json({{"connected", true}, {"items", items()}});
         }},
    };
  }

  tl::expected<dto::DataListResult, DataProviderErrorInfo> list_data(const std::string & /*entity_id*/) override {
    return dto::DataListResult{{{"items", items()}}};
  }
  tl::expected<dto::DataValue, DataProviderErrorInfo> read_data(const std::string & /*entity_id*/,
                                                                const std::string & resource_name) override {
    return dto::DataValue{{{"source", "plugin"}, {"name", resource_name}, {"value", value_.load()}}};
  }
  tl::expected<dto::DataWriteResult, DataProviderErrorInfo> write_data(const std::string & /*entity_id*/,
                                                                       const std::string & /*resource_name*/,
                                                                       const nlohmann::json & /*value*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read only", 405});
  }

 private:
  nlohmann::json items() const {
    return nlohmann::json::array({{{"id", "speed"}, {"name", "speed"}, {"value", value_.load()}}});
  }

  // What the plugin can reach through the context, as the plugin sees it.
  nlohmann::json describe_context() {
    nlohmann::json out;
    out["node_is_null"] = ros_ctx_ != nullptr && ros_ctx_->node() == nullptr;
    const auto entity = ctx_->get_entity("plugin_app");
    out["entity_found"] = entity.has_value() && entity->type == SovdEntityType::APP;
    out["snapshot_apps"] = ctx_->get_entity_snapshot().apps.size();
    out["type_capabilities"] = ctx_->get_type_capabilities(SovdEntityType::APP);
    out["notifier"] = ctx_->get_resource_change_notifier() != nullptr;
    out["conditions"] = ctx_->get_condition_registry() != nullptr;
    const auto lock = ctx_->acquire_lock("plugin_app", "plugin_client", {}, 60);
    out["lock_acquired"] = lock.has_value();
    out["lock_denies_other_client"] = !ctx_->check_lock("plugin_app", "other_client", "data").allowed;
    out["lock_released"] = ctx_->release_lock("plugin_app", "plugin_client").has_value();
    out["faults"] = ctx_->list_all_faults();
    ctx_->notify_entities_changed(EntityChangeScope::full_refresh());
    return out;
  }

  PluginContext * ctx_{nullptr};
  RosPluginContext * ros_ctx_{nullptr};
  std::atomic<int> value_{10};
};

}  // namespace

extern "C" GATEWAY_PLUGIN_EXPORT int plugin_api_version() {
  return PLUGIN_API_VERSION;
}

extern "C" GATEWAY_PLUGIN_EXPORT GatewayPlugin * create_plugin() {
  return new ContextPlugin();
}

extern "C" GATEWAY_PLUGIN_EXPORT DataProvider * get_data_provider(GatewayPlugin * plugin) {
  return static_cast<ContextPlugin *>(plugin);
}
