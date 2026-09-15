// Copyright 2026 mfaferek93
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

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_fault_manager/fault_storage.hpp"

namespace ros2_medkit_fault_manager {

/// Per-entity debounce overrides. Unset fields inherit from the global DebounceConfig.
struct EntityDebounceOverride {
  std::string prefix;  ///< Entity path prefix (e.g. "/sensors/lidar")
  std::optional<int32_t> confirmation_threshold;
  std::optional<bool> healing_enabled;
  std::optional<int32_t> healing_threshold;
};

/// Per-fault_code debounce overrides. Unset fields inherit from the layer below:
/// the entity override if one matched, otherwise the global DebounceConfig.
struct FaultCodeDebounceOverride {
  std::string fault_code;  ///< Exact fault code (e.g. "MOTOR_OVERHEAT")
  std::optional<int32_t> confirmation_threshold;
  std::optional<bool> healing_enabled;
  std::optional<int32_t> healing_threshold;
};

/// Resolves per-entity debounce thresholds using longest-prefix matching.
///
/// Given a source_id (entity FQN like "/powertrain/motor_left") and the global
/// DebounceConfig, returns a merged config where entity-specific overrides take
/// precedence over global defaults.
class EntityThresholdResolver {
 public:
  EntityThresholdResolver() = default;

  /// Construct with a list of entity overrides. Entries are sorted internally
  /// by prefix length (longest first) for efficient matching.
  explicit EntityThresholdResolver(std::vector<EntityDebounceOverride> entries);

  /// Resolve effective DebounceConfig for a given source_id.
  /// Finds the longest prefix match among entries and merges with global_default.
  /// If no match, returns global_default unchanged.
  DebounceConfig resolve(const std::string & source_id, const DebounceConfig & global_default) const;

  /// Number of configured entity entries.
  size_t size() const;

  /// Load entity threshold overrides from a YAML file.
  /// Returns an empty vector on parse error (logs warning via rcutils).
  /// YAML format: map of entity prefix -> {confirmation_threshold, healing_enabled, healing_threshold}
  static std::vector<EntityDebounceOverride> load_from_yaml(const std::string & path);

 private:
  /// Entries sorted by prefix length descending (longest first).
  std::vector<EntityDebounceOverride> entries_;
};

/// Resolves per-fault_code debounce thresholds by exact match on the fault code.
///
/// The debounce counter lives on the fault code while an entity override is
/// selected by the reporting source, so two entities reporting one code debounce
/// it under two policies. A fault-code override is the layer that removes that:
/// it resolves the same whoever reports, and is applied on top of whatever the
/// entity layer produced.
class FaultCodeThresholdResolver {
 public:
  FaultCodeThresholdResolver() = default;

  /// Construct with a list of fault-code overrides. A code repeated in the list
  /// keeps its first entry; YAML loading cannot produce one, a caller can.
  explicit FaultCodeThresholdResolver(std::vector<FaultCodeDebounceOverride> entries);

  /// Resolve the effective DebounceConfig for a fault code.
  /// `base` is what the layers below already produced (the global config, then
  /// any entity override). Fields the code does not set are left as `base` has
  /// them. A code with no override returns `base` unchanged.
  DebounceConfig resolve(const std::string & fault_code, const DebounceConfig & base) const;

  /// Number of configured fault-code entries.
  size_t size() const;

  /// Load fault-code threshold overrides from a YAML file.
  /// Returns an empty vector on parse error (logs warning via rcutils).
  /// YAML format: map of fault code -> {confirmation_threshold, healing_enabled, healing_threshold}
  static std::vector<FaultCodeDebounceOverride> load_from_yaml(const std::string & path);

 private:
  std::unordered_map<std::string, FaultCodeDebounceOverride> entries_;
};

/// Whether two configs debounce a fault the same way.
///
/// Compares only the three fields an override can carry. `auto_confirm_after_sec`
/// is global-only, so it is the same for every source by construction and would
/// only add noise to the comparison.
bool debounce_policy_equal(const DebounceConfig & a, const DebounceConfig & b);

}  // namespace ros2_medkit_fault_manager
