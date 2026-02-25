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
  GatewayPluginContext(GatewayNode * node, FaultManager * fault_manager) : node_(node), fault_manager_(fault_manager) {
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
      return PluginEntityInfo{SovdEntityType::APP, id, {}, app->bound_fqn.value_or("")};
    }
    if (auto area = cache.get_area(id)) {
      return PluginEntityInfo{SovdEntityType::AREA, id, area->namespace_path, {}};
    }
    if (cache.get_function(id)) {
      return PluginEntityInfo{SovdEntityType::FUNCTION, id, {}, {}};
    }
    return std::nullopt;
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

    // Determine expected type from route path
    auto expected_type = SovdEntityType::UNKNOWN;
    auto path = req.path;
    if (path.find("/components/") != std::string::npos) {
      expected_type = SovdEntityType::COMPONENT;
    } else if (path.find("/apps/") != std::string::npos) {
      expected_type = SovdEntityType::APP;
    } else if (path.find("/areas/") != std::string::npos) {
      expected_type = SovdEntityType::AREA;
    } else if (path.find("/functions/") != std::string::npos) {
      expected_type = SovdEntityType::FUNCTION;
    }

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

 private:
  GatewayNode * node_;
  FaultManager * fault_manager_;

  mutable std::mutex capabilities_mutex_;
  std::unordered_map<SovdEntityType, std::vector<std::string>> type_capabilities_;
  std::unordered_map<std::string, std::vector<std::string>> entity_capabilities_;
};

std::unique_ptr<PluginContext> make_gateway_plugin_context(GatewayNode * node, FaultManager * fault_manager) {
  return std::make_unique<GatewayPluginContext>(node, fault_manager);
}

}  // namespace ros2_medkit_gateway
