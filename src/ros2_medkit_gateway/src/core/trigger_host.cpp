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
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/data/polling_data_item_source.hpp"
#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/logs/log_scope.hpp"
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

/// Entity type of a trigger, which stores it as its route segment ("apps", ...).
SovdEntityType entity_type_from_collection_segment(const std::string & segment) {
  if (segment == "apps") {
    return SovdEntityType::APP;
  }
  if (segment == "components") {
    return SovdEntityType::COMPONENT;
  }
  if (segment == "areas") {
    return SovdEntityType::AREA;
  }
  if (segment == "functions") {
    return SovdEntityType::FUNCTION;
  }
  return SovdEntityType::UNKNOWN;
}

}  // namespace

void register_builtin_conditions(ConditionRegistry & conditions) {
  conditions.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  conditions.register_condition("OnChangeTo", std::make_shared<OnChangeToEvaluator>());
  conditions.register_condition("EnterRange", std::make_shared<EnterRangeEvaluator>());
  conditions.register_condition("LeaveRange", std::make_shared<LeaveRangeEvaluator>());
}

TriggerFaultFeed::FaultOwnerFn make_fault_owner_resolver(const ThreadSafeEntityCache & cache) {
  auto owners = std::make_shared<faults::AppSourceIndex>();
  return [&cache, owners](const std::string & owner) {
    return owners->find_owner(cache, owner);
  };
}

std::string resolve_trigger_topic(const ThreadSafeEntityCache & cache, const std::string & entity_id,
                                  const std::string & resource_path) {
  // "//x" is the separator slash followed by the item "/x", whose topic path is "/x".
  const std::string path = resource_path.rfind("//", 0) == 0 ? resource_path.substr(1) : resource_path;
  for (const auto & topic : cache.get_entity_data(entity_id).topics) {
    if (topic.name == path || (topic.name.size() > path.size() &&
                               topic.name.compare(topic.name.size() - path.size(), path.size(), path) == 0)) {
      return topic.name;
    }
  }
  return "";
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
  // Orphan sweep: an entity of the trigger's type, not any entity with its id.
  host.manager->set_entity_exists_fn([cache](const std::string & entity_id, const std::string & entity_type) {
    return cache->has_entity(entity_type_from_collection_segment(entity_type), entity_id);
  });
  host.manager->set_resolve_topic_fn([cache](const std::string & entity_id, const std::string & resource_path) {
    return resolve_trigger_topic(*cache, entity_id, resource_path);
  });
  host.manager->set_warn_log_fn(warn);
  host.manager->set_unresolved_timeout(wiring.unresolved_timeout);
  LogManager * log_manager = wiring.log_manager;
  if (log_manager != nullptr) {
    host.manager->set_log_manager(log_manager);
  }
  // A fault event matches an entity when its scope holds the record's owner, the rule of
  // GET /{entity}/faults?status=all. Clear and heal events included.
  host.manager->set_collection_scope_fn("faults", [cache](const std::string & entity_type,
                                                          const std::string & entity_id, const nlohmann::json & fault) {
    return faults::fault_in_source_scope(
        fault, faults::resolve_entity_source_fqns(*cache, entity_type_from_collection_segment(entity_type), entity_id));
  });
  // A log trigger checks each entry by the rule GET /{entity}/logs lists by: the entity's log scope and
  // severity_filter, read at dispatch, against the logger name and level the entry carries.
  host.manager->set_collection_scope_fn("logs", [cache, log_manager](const std::string & entity_type,
                                                                     const std::string & entity_id,
                                                                     const nlohmann::json & entry) {
    return log_manager != nullptr &&
           logs::entity_lists_log_change(*log_manager, *cache, entity_type_from_collection_segment(entity_type),
                                         entity_id, entry);
  });
  // A restored log trigger may wait for its entity's discovery while the entity logs. Once the entity is in the cache
  // the trigger replays what GET /{entity}/logs lists, less trigger markers, so those entries reach it too. Set before
  // the restore below.
  host.manager->set_restore_replay_fn(
      "logs",
      [cache, log_manager](const std::string & entity_type,
                           const std::string & entity_id) -> std::optional<std::vector<nlohmann::json>> {
        if (log_manager == nullptr) {
          return std::vector<nlohmann::json>{};
        }
        return logs::restore_replay_entries(*log_manager, *cache, entity_type_from_collection_segment(entity_type),
                                            entity_id);
      });
  host.restored = host.manager->load_persistent_triggers();

  if (wiring.fault_events != nullptr) {
    host.fault_feed =
        std::make_unique<TriggerFaultFeed>(*wiring.fault_events, notifier, make_fault_owner_resolver(*cache));
  }
  return host;
}

}  // namespace ros2_medkit_gateway
