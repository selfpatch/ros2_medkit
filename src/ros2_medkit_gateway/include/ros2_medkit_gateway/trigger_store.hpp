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
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Status of a trigger
enum class TriggerStatus { ACTIVE, TERMINATED };

/// Full trigger descriptor persisted in the store
struct TriggerInfo {
  std::string id;                              ///< Server-generated ID (e.g. "trig_001")
  std::string entity_id;                       ///< Target entity (e.g. "temp_sensor")
  std::string entity_type;                     ///< "apps" | "components" | "areas" | "functions"
  std::string resource_uri;                    ///< Full URI (e.g. "/api/v1/apps/temp_sensor/data/temperature")
  std::string collection;                      ///< Parsed collection segment (e.g. "data")
  std::string resource_path;                   ///< Parsed resource name (e.g. "temperature")
  std::string path;                            ///< JSON Pointer within the resource (e.g. "/data")
  std::string condition_type;                  ///< "OnChange", "LeaveRange", "x-custom", etc.
  nlohmann::json condition_params;             ///< e.g. {"lower_bound":20, "upper_bound":30}
  std::string protocol;                        ///< Delivery protocol (e.g. "sse")
  bool multishot{true};                        ///< false = fire once then terminate
  bool persistent{false};                      ///< Survives gateway restart
  std::optional<int> lifetime_sec;             ///< Optional TTL in seconds
  std::optional<nlohmann::json> log_settings;  ///< Optional logging configuration
  TriggerStatus status{TriggerStatus::ACTIVE};
  std::chrono::system_clock::time_point created_at;
  std::optional<std::chrono::system_clock::time_point> expires_at;
};

/// Abstract trigger persistence backend.
///
/// Implementations must be thread-safe. The default implementation is
/// SqliteTriggerStore; gateway plugins may provide alternatives.
class TriggerStore {
 public:
  virtual ~TriggerStore() = default;

  /// Persist a new (or replace an existing) trigger.
  virtual tl::expected<void, std::string> save(const TriggerInfo & trigger) = 0;

  /// Partially update a trigger by ID.  `fields` is a JSON object whose keys
  /// map to TriggerInfo field names (e.g. {"status":"TERMINATED"}).
  virtual tl::expected<void, std::string> update(const std::string & id, const nlohmann::json & fields) = 0;

  /// Remove a trigger by ID.
  virtual tl::expected<void, std::string> remove(const std::string & id) = 0;

  /// Load every trigger in the store.
  virtual tl::expected<std::vector<TriggerInfo>, std::string> load_all() = 0;

  /// Persist opaque evaluator state (e.g. previous value) for a trigger.
  virtual tl::expected<void, std::string> save_state(const std::string & trigger_id,
                                                     const nlohmann::json & previous_value) = 0;

  /// Load evaluator state for a trigger.  Returns nullopt when no state exists.
  virtual tl::expected<std::optional<nlohmann::json>, std::string> load_state(const std::string & trigger_id) = 0;
};

}  // namespace ros2_medkit_gateway
