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

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/faults/trigger_fault_feed.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/transports/data_item_source.hpp"
#include "ros2_medkit_gateway/core/trigger_store.hpp"

namespace ros2_medkit_gateway {

class LogManager;
class PluginManager;

/// Settings of the trigger subsystem. The names and defaults are those of the `triggers.*` parameters.
struct TriggerHostConfig {
  bool enabled{true};                        ///< `triggers.enabled`
  int64_t max_triggers{1000};                ///< `triggers.max_triggers`; outside [1, INT_MAX] the default is used
  std::string on_restart_behavior{"reset"};  ///< `triggers.on_restart_behavior`
  std::string storage_path;                  ///< `triggers.storage.path`; empty keeps triggers in memory
  int64_t data_poll_interval_ms{1000};       ///< `triggers.data_poll_interval_ms`; clamped to [100, 60000]
};

/// What the host supplies around the trigger manager.
struct TriggerHostWiring {
  /// Entity hierarchy, existence and data-item lookup. Required, and must outlive the host.
  const ThreadSafeEntityCache * cache{nullptr};
  /// Subscribes data triggers to topics. Null for a host without topics.
  std::shared_ptr<TopicSubscriptionTransport> topic_transport;
  /// Non-null: a data trigger with no topic reads the item through this plugin manager, once per
  /// `data_poll_interval_ms`. Null: no polling. Must outlive the host.
  PluginManager * polled_plugins{nullptr};
  /// Non-null: fault confirmations from this source fire fault triggers. Must outlive the host.
  FaultEventSource * fault_events{nullptr};
  /// Receives the trigger log settings, when set. Must outlive the host.
  LogManager * log_manager{nullptr};
  /// Operator-facing notices (clamped settings, a trigger that gave up resolving its topic).
  std::function<void(const std::string &)> warn;
  /// How long a data trigger with an unknown topic keeps resolving it.
  std::chrono::seconds unresolved_timeout{60};
};

/// The trigger manager and the objects it works with. Empty when the configuration is disabled.
struct TriggerHost {
  // Declaration order is destruction order in reverse: the feed and the manager go before what they use.
  std::unique_ptr<TriggerStore> store;
  std::shared_ptr<DataItemSource> data_source;
  std::unique_ptr<TriggerManager> manager;
  std::unique_ptr<TriggerFaultFeed> fault_feed;
  size_t restored{0};                ///< persistent triggers put back from the store
  int max_triggers{0};               ///< the limit in use
  int64_t data_poll_interval_ms{0};  ///< the polling interval in use
};

/// Registers OnChange, OnChangeTo, EnterRange and LeaveRange.
void register_builtin_conditions(ConditionRegistry & conditions);

/// The entity a reporting source (a node FQN) belongs to, or an empty string. A leading slash is optional.
std::string resolve_trigger_entity(const ThreadSafeEntityCache & cache, const std::string & node_fqn);

/// Builds the store and the manager, wires them to `cache`, restores persistent triggers and, when the wiring has a
/// fault event source, starts the fault feed. Throws `std::invalid_argument` when `wiring.cache` is null.
TriggerHost make_trigger_host(const TriggerHostConfig & config, ResourceChangeNotifier & notifier,
                              ConditionRegistry & conditions, const TriggerHostWiring & wiring);

}  // namespace ros2_medkit_gateway
