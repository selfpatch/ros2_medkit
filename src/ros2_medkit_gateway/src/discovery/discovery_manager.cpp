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

#include "ros2_medkit_gateway/discovery/discovery_manager.hpp"

#include "ros2_medkit_gateway/discovery/hybrid_discovery.hpp"
#include "ros2_medkit_gateway/discovery/layers/manifest_layer.hpp"
#include "ros2_medkit_gateway/discovery/layers/plugin_layer.hpp"
#include "ros2_medkit_gateway/discovery/layers/runtime_layer.hpp"
#include "ros2_medkit_gateway/discovery/manifest/runtime_linker.hpp"
#include "ros2_medkit_gateway/discovery/merge_pipeline.hpp"

namespace ros2_medkit_gateway {

DiscoveryManager::DiscoveryManager(rclcpp::Node * node)
  : node_(node), runtime_strategy_(std::make_unique<discovery::RuntimeDiscoveryStrategy>(node)) {
  // Default to runtime strategy
  active_strategy_ = runtime_strategy_.get();
}

bool DiscoveryManager::initialize(const DiscoveryConfig & config) {
  config_ = config;

  RCLCPP_INFO(node_->get_logger(), "Initializing discovery with mode: %s",
              discovery_mode_to_string(config.mode).c_str());

  // Create manifest manager if needed
  if (config.mode == DiscoveryMode::MANIFEST_ONLY || config.mode == DiscoveryMode::HYBRID) {
    manifest_manager_ = std::make_unique<discovery::ManifestManager>(node_);

    if (config.manifest_path.empty()) {
      if (config.mode == DiscoveryMode::HYBRID) {
        RCLCPP_WARN(node_->get_logger(),
                    "No manifest_path set for hybrid mode. Falling back to runtime_only. "
                    "This fallback is deprecated and will be removed in a future release.");
        config_.mode = DiscoveryMode::RUNTIME_ONLY;
        create_strategy();
        return true;
      }
      RCLCPP_ERROR(node_->get_logger(), "Manifest path required for %s mode. Set discovery.manifest_path.",
                   discovery_mode_to_string(config.mode).c_str());
      return false;
    }

    if (!manifest_manager_->load_manifest(config.manifest_path, config.manifest_strict_validation)) {
      if (config.mode == DiscoveryMode::HYBRID) {
        RCLCPP_WARN(node_->get_logger(),
                    "Manifest load failed in hybrid mode. Falling back to runtime_only. "
                    "This fallback is deprecated and will be removed in a future release.");
        config_.mode = DiscoveryMode::RUNTIME_ONLY;
        manifest_manager_.reset();
        create_strategy();
        return true;
      }
      RCLCPP_ERROR(node_->get_logger(), "Manifest load failed in %s mode. Cannot proceed.",
                   discovery_mode_to_string(config.mode).c_str());
      return false;
    }
  }

  create_strategy();
  return true;
}

template <typename LayerT>
void DiscoveryManager::apply_layer_policy_overrides(const std::string & layer_name, LayerT & layer) {
  auto it = config_.merge_pipeline.layer_policies.find(layer_name);
  if (it == config_.merge_pipeline.layer_policies.end()) {
    return;
  }
  for (const auto & [fg_str, policy_str] : it->second) {
    auto fg = discovery::field_group_from_string(fg_str);
    auto policy = discovery::merge_policy_from_string(policy_str);
    if (fg && policy) {
      layer.set_policy(*fg, *policy);
      RCLCPP_INFO(node_->get_logger(), "Layer '%s': override %s = %s", layer_name.c_str(), fg_str.c_str(),
                  policy_str.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "Layer '%s': ignoring invalid override %s = %s", layer_name.c_str(),
                  fg_str.c_str(), policy_str.c_str());
    }
  }
}

void DiscoveryManager::create_strategy() {
  // Configure runtime strategy with runtime options
  discovery::RuntimeDiscoveryStrategy::RuntimeConfig runtime_config;
  runtime_config.create_synthetic_areas = config_.runtime.create_synthetic_areas;
  runtime_config.create_synthetic_components = config_.runtime.create_synthetic_components;
  runtime_config.grouping = config_.runtime.grouping;
  runtime_config.synthetic_component_name_pattern = config_.runtime.synthetic_component_name_pattern;
  runtime_config.topic_only_policy = config_.runtime.topic_only_policy;
  runtime_config.min_topics_for_component = config_.runtime.min_topics_for_component;
  runtime_strategy_->set_config(runtime_config);

  switch (config_.mode) {
    case DiscoveryMode::MANIFEST_ONLY:
      // In MANIFEST_ONLY mode, entity structure comes from manifest (via manifest_manager_),
      // NOT from the active_strategy_. The discover_*() methods check the mode and return
      // manifest data directly. We still point active_strategy_ to runtime_strategy_ because:
      // 1. It's used for service/action introspection when explicitly requested
      // 2. It avoids creating a separate ManifestOnlyStrategy class
      // 3. It provides a fallback if manifest is unavailable
      active_strategy_ = runtime_strategy_.get();
      RCLCPP_INFO(node_->get_logger(), "Discovery mode: manifest_only");
      break;

    case DiscoveryMode::HYBRID: {
      discovery::MergePipeline pipeline(node_->get_logger());

      if (config_.manifest_enabled) {
        auto manifest_layer = std::make_unique<discovery::ManifestLayer>(manifest_manager_.get());
        apply_layer_policy_overrides("manifest", *manifest_layer);
        pipeline.add_layer(std::move(manifest_layer));
      } else {
        RCLCPP_INFO(node_->get_logger(), "Manifest layer disabled in hybrid mode");
      }

      if (config_.runtime_enabled) {
        auto runtime_layer = std::make_unique<discovery::RuntimeLayer>(runtime_strategy_.get());
        runtime_layer->set_gap_fill_config(config_.merge_pipeline.gap_fill);
        apply_layer_policy_overrides("runtime", *runtime_layer);
        pipeline.add_layer(std::move(runtime_layer));
      } else {
        RCLCPP_INFO(node_->get_logger(), "Runtime layer disabled in hybrid mode");
      }

      // Warn if no layers at all (plugins may still be loaded later)
      if (!config_.manifest_enabled && !config_.runtime_enabled) {
        RCLCPP_WARN(node_->get_logger(),
                    "Both manifest and runtime layers disabled in hybrid mode. "
                    "Entity discovery relies entirely on plugins.");
      }

      // RuntimeLinker only makes sense when runtime is enabled
      if (config_.runtime_enabled) {
        auto manifest_config = manifest_manager_ ? manifest_manager_->get_config() : discovery::ManifestConfig{};
        pipeline.set_linker(std::make_unique<discovery::RuntimeLinker>(node_), manifest_config);
      }

      hybrid_strategy_ = std::make_unique<discovery::HybridDiscoveryStrategy>(node_, std::move(pipeline));
      active_strategy_ = hybrid_strategy_.get();
      RCLCPP_INFO(node_->get_logger(), "Discovery mode: hybrid (manifest=%s, runtime=%s)",
                  config_.manifest_enabled ? "on" : "off", config_.runtime_enabled ? "on" : "off");
      break;
    }

    default:
      active_strategy_ = runtime_strategy_.get();
      RCLCPP_INFO(node_->get_logger(), "Discovery mode: runtime_only (synthetic_components=%s)",
                  config_.runtime.create_synthetic_components ? "true" : "false");
      break;
  }
}

std::vector<Area> DiscoveryManager::discover_areas() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_areas();
  }
  return active_strategy_->discover_areas();
}

std::vector<Component> DiscoveryManager::discover_components() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_components();
  }
  return active_strategy_->discover_components();
}

std::vector<App> DiscoveryManager::discover_apps() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_apps();
  }
  return active_strategy_->discover_apps();
}

std::vector<Function> DiscoveryManager::discover_functions() {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_functions();
  }
  return active_strategy_->discover_functions();
}

std::optional<Area> DiscoveryManager::get_area(const std::string & id) {
  // In MANIFEST_ONLY mode, use direct manifest lookup (O(1))
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_area(id);
  }
  // For HYBRID and RUNTIME modes, scan the strategy's output (cached for HYBRID)
  auto areas = discover_areas();
  for (const auto & a : areas) {
    if (a.id == id) {
      return a;
    }
  }
  return std::nullopt;
}

std::optional<Component> DiscoveryManager::get_component(const std::string & id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_component(id);
  }
  auto components = discover_components();
  for (const auto & c : components) {
    if (c.id == id) {
      return c;
    }
  }
  return std::nullopt;
}

std::optional<App> DiscoveryManager::get_app(const std::string & id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_app(id);
  }
  auto apps = discover_apps();
  for (const auto & app : apps) {
    if (app.id == id) {
      return app;
    }
  }
  return std::nullopt;
}

std::optional<Function> DiscoveryManager::get_function(const std::string & id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_function(id);
  }
  auto functions = discover_functions();
  for (const auto & f : functions) {
    if (f.id == id) {
      return f;
    }
  }
  return std::nullopt;
}

std::vector<Area> DiscoveryManager::get_subareas(const std::string & area_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_subareas(area_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: no subareas
  std::vector<Area> result;
  for (const auto & a : discover_areas()) {
    if (a.parent_area_id == area_id) {
      result.push_back(a);
    }
  }
  return result;
}

std::vector<Component> DiscoveryManager::get_subcomponents(const std::string & component_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_subcomponents(component_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: no subcomponents
  std::vector<Component> result;
  for (const auto & c : discover_components()) {
    if (c.parent_component_id == component_id) {
      result.push_back(c);
    }
  }
  return result;
}

std::vector<Component> DiscoveryManager::get_components_for_area(const std::string & area_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_components_for_area(area_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: filter by area
  std::vector<Component> result;
  for (const auto & c : discover_components()) {
    if (c.area == area_id) {
      result.push_back(c);
    }
  }
  return result;
}

std::vector<App> DiscoveryManager::get_apps_for_component(const std::string & component_id) {
  if (config_.mode == DiscoveryMode::MANIFEST_ONLY && manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_apps_for_component(component_id);
  }
  // HYBRID: filter from pipeline-merged output; RUNTIME: filter by component
  std::vector<App> result;
  for (const auto & app : discover_apps()) {
    if (app.component_id == component_id) {
      result.push_back(app);
    }
  }
  return result;
}

std::vector<std::string> DiscoveryManager::get_hosts_for_function(const std::string & function_id) {
  if (manifest_manager_ && manifest_manager_->is_manifest_active()) {
    return manifest_manager_->get_hosts_for_function(function_id);
  }
  return {};
}

std::vector<Component> DiscoveryManager::discover_topic_components() {
  return runtime_strategy_->discover_topic_components();
}

std::vector<ServiceInfo> DiscoveryManager::discover_services() {
  return runtime_strategy_->discover_services();
}

std::vector<ActionInfo> DiscoveryManager::discover_actions() {
  return runtime_strategy_->discover_actions();
}

std::optional<ServiceInfo> DiscoveryManager::find_service(const std::string & component_ns,
                                                          const std::string & operation_name) const {
  return runtime_strategy_->find_service(component_ns, operation_name);
}

std::optional<ActionInfo> DiscoveryManager::find_action(const std::string & component_ns,
                                                        const std::string & operation_name) const {
  return runtime_strategy_->find_action(component_ns, operation_name);
}

void DiscoveryManager::set_topic_sampler(NativeTopicSampler * sampler) {
  runtime_strategy_->set_topic_sampler(sampler);
}

void DiscoveryManager::set_type_introspection(TypeIntrospection * introspection) {
  runtime_strategy_->set_type_introspection(introspection);
}

void DiscoveryManager::refresh_topic_map() {
  runtime_strategy_->refresh_topic_map();
  if (hybrid_strategy_) {
    hybrid_strategy_->refresh();
  }
}

bool DiscoveryManager::is_topic_map_ready() const {
  return runtime_strategy_->is_topic_map_ready();
}

void DiscoveryManager::add_plugin_layer(const std::string & plugin_name, IntrospectionProvider * provider) {
  if (!hybrid_strategy_) {
    RCLCPP_WARN(node_->get_logger(), "Cannot add plugin layer '%s': not in hybrid mode", plugin_name.c_str());
    return;
  }
  hybrid_strategy_->add_layer(std::make_unique<discovery::PluginLayer>(plugin_name, provider));
  RCLCPP_INFO(node_->get_logger(), "Added plugin layer '%s' to merge pipeline", plugin_name.c_str());
}

void DiscoveryManager::refresh_pipeline() {
  if (hybrid_strategy_) {
    hybrid_strategy_->refresh();
  }
}

discovery::ManifestManager * DiscoveryManager::get_manifest_manager() {
  return manifest_manager_.get();
}

std::string DiscoveryManager::get_strategy_name() const {
  if (active_strategy_) {
    return active_strategy_->get_name();
  }
  return "unknown";
}

std::optional<discovery::MergeReport> DiscoveryManager::get_merge_report() const {
  if (hybrid_strategy_) {
    return hybrid_strategy_->get_merge_report();
  }
  return std::nullopt;
}

std::optional<discovery::LinkingResult> DiscoveryManager::get_linking_result() const {
  if (hybrid_strategy_) {
    return hybrid_strategy_->get_linking_result();
  }
  return std::nullopt;
}

}  // namespace ros2_medkit_gateway
