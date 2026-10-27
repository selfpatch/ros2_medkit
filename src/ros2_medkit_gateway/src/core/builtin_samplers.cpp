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

#include "ros2_medkit_gateway/core/builtin_samplers.hpp"

#include <string>
#include <utility>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/data/data_item_reader.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/gateway_services.hpp"
#include "ros2_medkit_gateway/core/logs/log_scope.hpp"
#include "ros2_medkit_gateway/core/managers/configuration_manager.hpp"
#include "ros2_medkit_gateway/core/managers/data_access_manager.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/managers/update_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/update_types.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"

namespace ros2_medkit_gateway {

namespace {

using Sampled = tl::expected<nlohmann::json, std::string>;

Sampled sample_data(GatewayServices & services, const std::string & entity_id, const std::string & resource_path) {
  // A plugin-owned entity is read through its provider, as GET /{entity}/data/{item} does.
  auto * plugins = services.plugin_manager();
  if (plugins != nullptr && plugins->get_entity_owner(entity_id)) {
    const std::string item_name =
        (!resource_path.empty() && resource_path.front() == '/') ? resource_path.substr(1) : resource_path;
    auto item = read_plugin_data_item(plugins, entity_id, item_name);
    if (!item) {
      return tl::make_unexpected(item.error().message);
    }
    return std::move(item->content);
  }
  auto sample = sample_data_topic(*services.data_access_manager(), to_full_topic_path(resource_path));
  if (!sample) {
    return tl::make_unexpected(std::string{"Topic sample failed: "} + sample.error().message);
  }
  if (sample->has_data && sample->data.has_value()) {
    nlohmann::json payload;
    payload["id"] = resource_path;
    payload["data"] = *sample->data;
    return payload;
  }
  return tl::make_unexpected("Topic data not available: " + resource_path);
}

Sampled sample_faults(GatewayServices & services, const std::string & entity_id) {
  auto * fault_mgr = services.fault_manager();
  if (!fault_mgr) {
    return tl::make_unexpected(std::string("FaultManager not available"));
  }
  const auto & cache = services.entity_cache();
  auto entity_ref = cache.find_entity(entity_id);
  if (!entity_ref) {
    return tl::make_unexpected("Entity not found: " + entity_id);
  }
  // Same scoping as GET /{entity}/faults: shared source resolution (external app -> bare id,
  // external component owns its own id) and the shared boundary-aware filter.
  auto source_fqns = faults::resolve_entity_source_fqns(cache, entity_ref->type, entity_id);
  auto result = fault_mgr->list_faults("");
  if (!result.success) {
    return tl::make_unexpected(result.error_message);
  }
  auto filtered = faults::filter_faults_by_sources(result.data["faults"], source_fqns);
  result.data["count"] = filtered.size();
  result.data["faults"] = std::move(filtered);
  return result.data;
}

Sampled sample_configurations(GatewayServices & services, const std::string & entity_id) {
  auto * config_mgr = services.configuration_manager();
  if (!config_mgr) {
    return tl::make_unexpected(std::string("ConfigurationManager not available"));
  }
  auto entity_configs = services.entity_cache().get_entity_configurations(entity_id);
  nlohmann::json items = nlohmann::json::array();
  for (const auto & node : entity_configs.nodes) {
    auto result = config_mgr->list_parameters(node.node_fqn);
    if (result.success && result.data.is_array()) {
      for (auto & param : result.data) {
        items.push_back(std::move(param));
      }
    }
  }
  nlohmann::json payload;
  payload["items"] = std::move(items);
  return payload;
}

Sampled sample_logs(GatewayServices & services, const std::string & entity_id) {
  auto * log_mgr = services.log_manager();
  if (!log_mgr) {
    return tl::make_unexpected(std::string("LogManager not available"));
  }
  const auto & cache = services.entity_cache();
  auto entity_ref = cache.find_entity(entity_id);
  if (!entity_ref) {
    return tl::make_unexpected("Entity not found: " + entity_id);
  }
  // Same scoping as GET /{entity}/logs via the shared helper (external app -> bare id, external
  // component owns its own id, plus the namespace-prefix merge for COMPONENT / AREA).
  std::string entity_fqn;
  if (entity_ref->type == SovdEntityType::COMPONENT) {
    if (auto comp = cache.get_component(entity_id)) {
      entity_fqn = comp->fqn;
    }
  } else if (entity_ref->type == SovdEntityType::AREA) {
    if (auto area = cache.get_area(entity_id)) {
      entity_fqn = area->namespace_path;
    }
  } else if (entity_ref->type == SovdEntityType::APP) {
    if (auto app = cache.get_app(entity_id)) {
      entity_fqn = app->effective_fqn();
    }
  }
  auto result = logs::query_entity_logs(*log_mgr, cache, entity_ref->type, entity_id, entity_fqn, "", "");
  if (!result.has_value()) {
    return tl::make_unexpected(result.error());
  }
  nlohmann::json payload;
  payload["items"] = std::move(*result);
  return payload;
}

Sampled sample_updates(GatewayServices & services, const std::string & resource_path) {
  auto * mgr = services.update_manager();
  if (!mgr || !mgr->has_backend()) {
    return tl::make_unexpected(std::string("Update backend not available"));
  }
  auto result = mgr->get_status(resource_path);
  if (!result) {
    return tl::make_unexpected(result.error().message);
  }
  return update_status_to_json(*result);
}

}  // namespace

void register_builtin_samplers(GatewayServices & services) {
  auto & registry = *services.sampler_registry();
  registry.register_sampler(
      "data",
      [&services](const std::string & entity_id, const std::string & resource_path) {
        return sample_data(services, entity_id, resource_path);
      },
      /*is_builtin=*/true, /*honours_resource_path=*/true);
  registry.register_sampler(
      "faults",
      [&services](const std::string & entity_id, const std::string & /*resource_path*/) {
        return sample_faults(services, entity_id);
      },
      /*is_builtin=*/true);
  registry.register_sampler(
      "configurations",
      [&services](const std::string & entity_id, const std::string & /*resource_path*/) {
        return sample_configurations(services, entity_id);
      },
      /*is_builtin=*/true);
  registry.register_sampler(
      "logs",
      [&services](const std::string & entity_id, const std::string & /*resource_path*/) {
        return sample_logs(services, entity_id);
      },
      /*is_builtin=*/true);
  if (services.update_manager() != nullptr) {
    registry.register_sampler(
        "updates",
        [&services](const std::string & /*entity_id*/, const std::string & resource_path) {
          return sample_updates(services, resource_path);
        },
        /*is_builtin=*/true, /*honours_resource_path=*/true);
  }
}

}  // namespace ros2_medkit_gateway
