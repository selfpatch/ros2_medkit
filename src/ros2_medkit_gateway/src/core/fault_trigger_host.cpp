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

#include "ros2_medkit_gateway/core/fault_trigger_host.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/faults/freeze_frame_source.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"

namespace ros2_medkit_gateway {

namespace {

constexpr int64_t kMinPollIntervalMs = 50;

// Coerce a discovered data point's JSON value to a double for threshold
// comparison. Handles the numeric, boolean and numeric-string shapes the PLC
// bridges emit; returns nullopt for anything non-numeric.
std::optional<double> value_to_double(const nlohmann::json & v) {
  if (v.is_number()) {
    return v.get<double>();
  }
  if (v.is_boolean()) {
    return v.get<bool>() ? 1.0 : 0.0;
  }
  if (v.is_string()) {
    try {
      size_t pos = 0;
      const double d = std::stod(v.get<std::string>(), &pos);
      if (pos > 0) {
        return d;
      }
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return std::nullopt;
}

uint8_t severity_level(const std::string & severity) {
  if (severity == "INFO") {
    return 0;
  }
  if (severity == "ERROR") {
    return 2;
  }
  if (severity == "CRITICAL") {
    return 3;
  }
  return 1;  // WARNING
}

}  // namespace

FaultTriggerHost::FaultTriggerHost(const FaultTriggerHostConfig & config, int poll_interval_ms,
                                   const FaultTriggerHostWiring & wiring)
  : warn_(wiring.warn), poll_interval_ms_(poll_interval_ms) {
  PluginManager * plugins = wiring.plugins;
  FaultManager * faults = wiring.faults;
  const ThreadSafeEntityCache * cache = wiring.cache;

  // Same in-process x-plc-data route the freeze-frame capture uses, so a rule sees exactly the value the /data
  // endpoint would serve.
  auto fetcher = [plugins](const std::string & app_id, const std::string & data_name) -> std::optional<double> {
    const auto content = plugins->fetch_entity_data_via_route(app_id);
    // A down link serves frozen last-known values. Returning nullopt holds the rule state instead of firing on a stale
    // number for the whole outage.
    if (!content || !FreezeFrameSource::content_has_live_data(*content) ||
        FreezeFrameSource::content_reports_disconnected(*content)) {
      return std::nullopt;
    }
    const auto values = FreezeFrameSource::values_from_list_content(*content);
    if (!values.is_object() || !values.contains(data_name)) {
      return std::nullopt;
    }
    return value_to_double(values[data_name]);
  };

  auto report = [faults](const std::string & app_id, const std::string & fault_code, const std::string & severity,
                         const std::string & description) {
    if (faults) {
      faults->report_fault(fault_code, severity_level(severity), description, app_id);
    }
  };

  // A rule's auto-clear is local to the rule: skip the correlation cascade, so clearing one rule's fault never wipes
  // correlated faults owned by anything else. fault_code is unique across rules, so the app scope is in the code.
  auto clear = [faults](const std::string & /*app_id*/, const std::string & fault_code) {
    if (faults) {
      faults->clear_fault(fault_code, /*skip_correlation_auto_clear=*/true);
    }
  };

  // Same route as the fetcher: "exists" means exactly "the fetcher could ever read it".
  auto data_point_names = [plugins](const std::string & app_id) -> std::optional<std::vector<std::string>> {
    const auto content = plugins->fetch_entity_data_via_route(app_id);
    if (!content || !FreezeFrameSource::content_has_live_data(*content)) {
      return std::nullopt;
    }
    const auto values = FreezeFrameSource::values_from_list_content(*content);
    if (!values.is_object()) {
      return std::nullopt;
    }
    std::vector<std::string> names;
    names.reserve(values.size());
    for (auto it = values.begin(); it != values.end(); ++it) {
      names.push_back(it.key());
    }
    return names;
  };

  // Distinct from the data route, which returns nullopt for both an unknown app and a transient outage.
  auto entity_exists = [cache](const std::string & app_id) -> bool {
    return cache->find_entity(app_id).has_value();
  };

  engine_ =
      std::make_unique<FaultTriggerEngine>(config.storage_path, std::move(fetcher), std::move(report), std::move(clear),
                                           wiring.log, std::move(data_point_names), std::move(entity_exists));

  const auto interval = std::chrono::milliseconds(poll_interval_ms_);
  thread_ = std::thread([this, interval]() {
    while (!stop_.load(std::memory_order_relaxed)) {
      try {
        engine_->evaluate_once();
      } catch (const std::exception & e) {
        if (warn_) {
          warn_(std::string("fault-trigger evaluation error: ") + e.what());
        }
      }
      // Short slices, so shutdown does not wait a whole interval.
      auto slept = std::chrono::milliseconds(0);
      while (slept < interval && !stop_.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
        slept += std::chrono::milliseconds(25);
      }
    }
  });
}

FaultTriggerHost::~FaultTriggerHost() {
  stop_.store(true, std::memory_order_relaxed);
  if (thread_.joinable()) {
    thread_.join();
  }
}

std::unique_ptr<FaultTriggerHost> make_fault_trigger_host(const FaultTriggerHostConfig & config,
                                                          const FaultTriggerHostWiring & wiring) {
  if (!wiring.cache || !wiring.plugins) {
    throw std::invalid_argument("make_fault_trigger_host needs an entity cache and a plugin manager");
  }
  if (!config.enabled || !wiring.plugins->has_plugins()) {
    return nullptr;
  }
  // Floor so the loop can never busy-spin. The value is int64 and is checked before it is narrowed, or one above
  // INT_MAX would wrap past the floor.
  const int64_t used = std::clamp<int64_t>(config.poll_interval_ms, kMinPollIntervalMs,
                                           static_cast<int64_t>(std::numeric_limits<int>::max()));
  if (used != config.poll_interval_ms && wiring.warn) {
    wiring.warn("fault_triggers.poll_interval_ms " + std::to_string(config.poll_interval_ms) + " clamped to " +
                std::to_string(used));
  }
  auto host = std::make_unique<FaultTriggerHost>(config, static_cast<int>(used), wiring);
  if (wiring.log) {
    wiring.log("fault-trigger engine active (poll " + std::to_string(used) + " ms, store '" +
               (config.storage_path.empty() ? std::string("in-memory") : config.storage_path) + "')");
  }
  return host;
}

}  // namespace ros2_medkit_gateway
