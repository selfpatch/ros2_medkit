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

#include "ros2_medkit_gateway/plugins/plugin_context.hpp"

#include "ros2_medkit_gateway/fault_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/error_codes.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_gateway/http/http_utils.hpp"
#include "ros2_medkit_gateway/resource_sampler.hpp"

#include <mutex>
#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway {

// ---- Static utility methods (delegate to HandlerContext) ----

void PluginContext::send_error(httplib::Response & res, int status, const std::string & error_code,
                               const std::string & message, const nlohmann::json & parameters) {
  handlers::HandlerContext::send_error(res, status, error_code, message, parameters);
}

void PluginContext::send_json(httplib::Response & res, const nlohmann::json & data) {
  handlers::HandlerContext::send_json(res, data);
}

// ---- Concrete implementation ----

class GatewayPluginContext : public PluginContext {
 public:
  GatewayPluginContext(GatewayNode * node, FaultManager * fault_manager, ResourceSamplerRegistry * sampler_registry)
    : node_(node), fault_manager_(fault_manager), sampler_registry_(sampler_registry) {
  }

  rclcpp::Node * node() const override {
    return node_;
  }

  std::optional<PluginEntityInfo> get_entity(const std::string & id) const override {
    const auto & cache = node_->get_thread_safe_cache();

    if (auto comp = cache.get_component(id)) {
      return PluginEntityInfo{SovdEntityType::COMPONENT, id, comp->namespace_path, comp->fqn};
    }
    if (auto app = cache.get_app(id)) {
      return PluginEntityInfo{SovdEntityType::APP, id, {}, app->effective_fqn()};
    }
    if (auto area = cache.get_area(id)) {
      return PluginEntityInfo{SovdEntityType::AREA, id, area->namespace_path, {}};
    }
    if (cache.get_function(id)) {
      return PluginEntityInfo{SovdEntityType::FUNCTION, id, {}, {}};
    }
    return std::nullopt;
  }

  std::vector<PluginEntityInfo> get_child_apps(const std::string & component_id) const override {
    std::vector<PluginEntityInfo> result;
    const auto & cache = node_->get_thread_safe_cache();
    auto app_ids = cache.get_apps_for_component(component_id);
    for (const auto & app_id : app_ids) {
      if (auto app = cache.get_app(app_id)) {
        result.push_back(PluginEntityInfo{SovdEntityType::APP, app_id,
                                          "",  // namespace_path not needed for Apps
                                          app->effective_fqn()});
      }
    }
    return result;
  }

  nlohmann::json list_entity_faults(const std::string & entity_id) const override {
    if (!fault_manager_ || !fault_manager_->is_available()) {
      return nlohmann::json::array();
    }

    // Determine source_id for fault filtering based on entity type
    auto entity = get_entity(entity_id);
    if (!entity) {
      return nlohmann::json::array();
    }

    std::string source_id;
    if (entity->type == SovdEntityType::COMPONENT) {
      source_id = entity->namespace_path;
    } else if (entity->type == SovdEntityType::APP) {
      source_id = entity->fqn;
    }

    auto result = fault_manager_->list_faults(source_id);
    if (result.success && result.data.is_array()) {
      return result.data;
    }
    return nlohmann::json::array();
  }

  std::optional<PluginEntityInfo> validate_entity_for_route(const httplib::Request & req, httplib::Response & res,
                                                            const std::string & entity_id) const override {
    // Validate entity ID format
    if (entity_id.empty() || entity_id.size() > 256) {
      send_error(res, 400, ERR_INVALID_PARAMETER, "Invalid entity ID");
      return std::nullopt;
    }

    // Determine expected type from route path (segment-boundary-aware matching)
    auto expected_type = extract_entity_type_from_path(req.path);

    auto entity = get_entity(entity_id);
    if (!entity) {
      send_error(res, 404, ERR_ENTITY_NOT_FOUND, to_string(expected_type) + " not found: " + entity_id);
      return std::nullopt;
    }

    // Check type matches route
    if (expected_type != SovdEntityType::UNKNOWN && entity->type != expected_type) {
      send_error(res, 400, ERR_INVALID_PARAMETER,
                 "Entity '" + entity_id + "' is a " + to_string(entity->type) + ", not a " + to_string(expected_type));
      return std::nullopt;
    }

    return entity;
  }

  void register_capability(SovdEntityType entity_type, const std::string & capability_name) override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto & caps = type_capabilities_[entity_type];
    // Avoid duplicates
    for (const auto & c : caps) {
      if (c == capability_name) {
        return;
      }
    }
    caps.push_back(capability_name);
  }

  void register_entity_capability(const std::string & entity_id, const std::string & capability_name) override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto & caps = entity_capabilities_[entity_id];
    for (const auto & c : caps) {
      if (c == capability_name) {
        return;
      }
    }
    caps.push_back(capability_name);
  }

  std::vector<std::string> get_type_capabilities(SovdEntityType entity_type) const override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto it = type_capabilities_.find(entity_type);
    if (it != type_capabilities_.end()) {
      return it->second;
    }
    return {};
  }

  std::vector<std::string> get_entity_capabilities(const std::string & entity_id) const override {
    std::lock_guard<std::mutex> lock(capabilities_mutex_);
    auto it = entity_capabilities_.find(entity_id);
    if (it != entity_capabilities_.end()) {
      return it->second;
    }
    return {};
  }

  IntrospectionInput get_entity_snapshot() const override {
    IntrospectionInput input;
    const auto & cache = node_->get_thread_safe_cache();
    input.areas = cache.get_areas();
    input.components = cache.get_components();
    input.apps = cache.get_apps();
    input.functions = cache.get_functions();
    return input;
  }

  nlohmann::json list_all_faults() const override {
    if (!fault_manager_ || !fault_manager_->is_available()) {
      return nlohmann::json::object();
    }
    auto result = fault_manager_->list_faults("");
    if (result.success) {
      return result.data;
    }
    return nlohmann::json::object();
  }

  void register_sampler(
      const std::string & collection,
      std::function<tl::expected<nlohmann::json, std::string>(const std::string &, const std::string &)> fn) override {
    if (sampler_registry_) {
      sampler_registry_->register_sampler(collection, std::move(fn));
    }
  }

 private:
  GatewayNode * node_;
  FaultManager * fault_manager_;
  ResourceSamplerRegistry * sampler_registry_;

  mutable std::mutex capabilities_mutex_;
  std::unordered_map<SovdEntityType, std::vector<std::string>> type_capabilities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_capabilities_;
};

std::unique_ptr<PluginContext> make_gateway_plugin_context(GatewayNode * node, FaultManager * fault_manager,
                                                           ResourceSamplerRegistry * sampler_registry) {
  return std::make_unique<GatewayPluginContext>(node, fault_manager, sampler_registry);
}

}  // namespace ros2_medkit_gateway
