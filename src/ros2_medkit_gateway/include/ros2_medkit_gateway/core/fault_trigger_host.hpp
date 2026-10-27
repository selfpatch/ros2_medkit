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

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/core/fault_trigger_engine.hpp"

namespace ros2_medkit_gateway {

class FaultManager;
class PluginManager;
class ThreadSafeEntityCache;

/// Settings of the fault-trigger (threshold-rule) engine. Names and defaults are those of the `fault_triggers.*`
/// parameters.
struct FaultTriggerHostConfig {
  bool enabled{true};              ///< `fault_triggers.enabled`
  int64_t poll_interval_ms{1000};  ///< `fault_triggers.poll_interval_ms`; raised to 50 ms, capped at INT_MAX
  std::string storage_path;        ///< `fault_triggers.storage.path`; empty keeps the rules in memory
};

/// What the host supplies around the engine. Every pointer must outlive the host.
struct FaultTriggerHostWiring {
  /// Entity existence for rule creation. Required.
  const ThreadSafeEntityCache * cache{nullptr};
  /// The rules read the values of their data points through the plugins' data routes. Required.
  PluginManager * plugins{nullptr};
  /// A rule that crosses its threshold reports its fault here, and clears it here. Null: nothing is reported.
  FaultManager * faults{nullptr};
  /// Notices: the engine's log lines and its evaluation errors.
  std::function<void(const std::string &)> log;
  /// Operator-facing notices (a clamped setting, an evaluation error).
  std::function<void(const std::string &)> warn;
};

/// The fault-trigger engine and its evaluation loop.
///
/// The loop runs on a thread of its own: reading a value and reporting a fault must not run on the host's executor.
/// The destructor joins it, so destroy the host before the plugins and the fault manager go.
class FaultTriggerHost {
 public:
  /// Starts the loop. Use `make_fault_trigger_host()`.
  FaultTriggerHost(const FaultTriggerHostConfig & config, int poll_interval_ms, const FaultTriggerHostWiring & wiring);
  ~FaultTriggerHost();
  FaultTriggerHost(const FaultTriggerHost &) = delete;
  FaultTriggerHost & operator=(const FaultTriggerHost &) = delete;
  FaultTriggerHost(FaultTriggerHost &&) = delete;
  FaultTriggerHost & operator=(FaultTriggerHost &&) = delete;

  FaultTriggerEngine & engine() {
    return *engine_;
  }

  /// The interval in use.
  int poll_interval_ms() const {
    return poll_interval_ms_;
  }

 private:
  std::function<void(const std::string &)> warn_;
  int poll_interval_ms_;
  std::unique_ptr<FaultTriggerEngine> engine_;
  std::atomic<bool> stop_{false};
  std::thread thread_;
};

/// Builds the engine and starts its loop. Null when the configuration is disabled or `wiring.plugins` has no plugin
/// loaded: there is nothing to read the values from. Throws `std::invalid_argument` when `wiring.cache` or
/// `wiring.plugins` is null.
std::unique_ptr<FaultTriggerHost> make_fault_trigger_host(const FaultTriggerHostConfig & config,
                                                          const FaultTriggerHostWiring & wiring);

}  // namespace ros2_medkit_gateway
