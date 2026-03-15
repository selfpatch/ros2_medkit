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

}  // namespace ros2_medkit_fault_manager
