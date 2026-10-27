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

#include "ros2_medkit_gateway/core/trigger_host.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/data/polling_data_item_source.hpp"
#include "ros2_medkit_gateway/core/sqlite_trigger_store.hpp"

namespace ros2_medkit_gateway {

namespace {
constexpr int kDefaultMaxTriggers = 1000;
constexpr int64_t kMinPollIntervalMs = 100;
constexpr int64_t kMaxPollIntervalMs = 60000;

std::vector<std::string> apps_under(const ThreadSafeEntityCache & cache, const std::string & entity_id,
                                    const std::string & entity_type) {
  if (entity_type == "components") {
    return cache.get_apps_for_component(entity_id);
  }
  if (entity_type == "areas") {
    std::vector<std::string> all_apps;
    for (const auto & component_id : cache.get_components_for_area(entity_id)) {
      auto apps = cache.get_apps_for_component(component_id);
      all_apps.insert(all_apps.end(), apps.begin(), apps.end());
    }
    return all_apps;
  }
  if (entity_type == "functions") {
    return cache.get_apps_for_function(entity_id);
  }
  return {};
}

std::string topic_of(const ThreadSafeEntityCache & cache, const std::string & entity_id,
                     const std::string & resource_path) {
  for (const auto & topic : cache.get_entity_data(entity_id).topics) {
    if (topic.name == resource_path ||
        (topic.name.size() > resource_path.size() &&
         topic.name.compare(topic.name.size() - resource_path.size(), resource_path.size(), resource_path) == 0)) {
      return topic.name;
    }
  }
  return "";
}
}  // namespace

void register_builtin_conditions(ConditionRegistry & conditions) {
  conditions.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  conditions.register_condition("OnChangeTo", std::make_shared<OnChangeToEvaluator>());
  conditions.register_condition("EnterRange", std::make_shared<EnterRangeEvaluator>());
  conditions.register_condition("LeaveRange", std::make_shared<LeaveRangeEvaluator>());
}

std::string resolve_trigger_entity(const ThreadSafeEntityCache & cache, const std::string & node_fqn) {
  auto entity_id = cache.resolve_node_to_app(node_fqn);
  if (entity_id.empty() && !node_fqn.empty() && node_fqn[0] == '/') {
    entity_id = cache.resolve_node_to_app(node_fqn.substr(1));
  }
  return entity_id;
}

TriggerHost make_trigger_host(const TriggerHostConfig & config, ResourceChangeNotifier & notifier,
                              ConditionRegistry & conditions, const TriggerHostWiring & wiring) {
  TriggerHost host;
  if (!config.enabled) {
    return host;
  }
  if (wiring.cache == nullptr) {
    throw std::invalid_argument("make_trigger_host needs an entity cache");
  }
  const ThreadSafeEntityCache * cache = wiring.cache;
  const auto warn = [notice = wiring.warn](const std::string & message) {
    if (notice) {
      notice(message);
    }
  };

  host.store = std::make_unique<SqliteTriggerStore>(config.storage_path.empty() ? ":memory:" : config.storage_path);

  // The limit is compared with a size_t in TriggerManager, so a negative would become a huge bound and the
  // documented 503 at the limit would never come. Checked as int64 before it is narrowed.
  TriggerConfig trigger_config;
  if (config.max_triggers < 1 || config.max_triggers > std::numeric_limits<int>::max()) {
    warn("triggers.max_triggers " + std::to_string(config.max_triggers) + " out of range. Using default " +
         std::to_string(kDefaultMaxTriggers) + ".");
    trigger_config.max_triggers = kDefaultMaxTriggers;
  } else {
    trigger_config.max_triggers = static_cast<int>(config.max_triggers);
  }
  trigger_config.on_restart_behavior = config.on_restart_behavior;
  host.max_triggers = trigger_config.max_triggers;

  host.data_poll_interval_ms = std::clamp(config.data_poll_interval_ms, kMinPollIntervalMs, kMaxPollIntervalMs);
  if (host.data_poll_interval_ms != config.data_poll_interval_ms) {
    warn("triggers.data_poll_interval_ms " + std::to_string(config.data_poll_interval_ms) + " clamped to " +
         std::to_string(host.data_poll_interval_ms));
  }
  if (wiring.polled_plugins != nullptr) {
    host.data_source =
        make_plugin_data_item_source(wiring.polled_plugins, std::chrono::milliseconds(host.data_poll_interval_ms));
  }

  host.manager = std::make_unique<TriggerManager>(notifier, conditions, *host.store, trigger_config,
                                                  wiring.topic_transport, host.data_source);
  host.manager->set_entity_children_fn([cache](const std::string & entity_id, const std::string & entity_type) {
    return apps_under(*cache, entity_id, entity_type);
  });
  host.manager->set_entity_exists_fn([cache](const std::string & entity_id, const std::string & /*entity_type*/) {
    return cache->find_entity(entity_id).has_value();
  });
  host.manager->set_resolve_topic_fn([cache](const std::string & entity_id, const std::string & resource_path) {
    return topic_of(*cache, entity_id, resource_path);
  });
  host.manager->set_warn_log_fn(warn);
  host.manager->set_unresolved_timeout(wiring.unresolved_timeout);
  if (wiring.log_manager != nullptr) {
    host.manager->set_log_manager(wiring.log_manager);
  }
  host.restored = host.manager->load_persistent_triggers();

  if (wiring.fault_events != nullptr) {
    host.fault_feed =
        std::make_unique<TriggerFaultFeed>(*wiring.fault_events, notifier, [cache](const std::string & node_fqn) {
          return resolve_trigger_entity(*cache, node_fqn);
        });
  }
  return host;
}

}  // namespace ros2_medkit_gateway
