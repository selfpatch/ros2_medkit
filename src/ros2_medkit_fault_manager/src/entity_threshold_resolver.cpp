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

#include "ros2_medkit_fault_manager/entity_threshold_resolver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>

#include "rcutils/logging_macros.h"

namespace ros2_medkit_fault_manager {

EntityThresholdResolver::EntityThresholdResolver(std::vector<EntityDebounceOverride> entries)
  : entries_(std::move(entries)) {
  // Sort by prefix length descending so longest-prefix match is found first
  std::sort(entries_.begin(), entries_.end(), [](const EntityDebounceOverride & a, const EntityDebounceOverride & b) {
    return a.prefix.size() > b.prefix.size();
  });
}

DebounceConfig EntityThresholdResolver::resolve(const std::string & source_id,
                                                const DebounceConfig & global_default) const {
  for (const auto & entry : entries_) {
    // Check if source_id starts with the prefix
    if (source_id.size() >= entry.prefix.size() && source_id.compare(0, entry.prefix.size(), entry.prefix) == 0) {
      // Merge: entry overrides take precedence, unset fields inherit global
      DebounceConfig result = global_default;
      if (entry.confirmation_threshold.has_value()) {
        result.confirmation_threshold = *entry.confirmation_threshold;
      }
      if (entry.healing_enabled.has_value()) {
        result.healing_enabled = *entry.healing_enabled;
      }
      if (entry.healing_threshold.has_value()) {
        result.healing_threshold = *entry.healing_threshold;
      }
      return result;
    }
  }
  return global_default;
}

size_t EntityThresholdResolver::size() const {
  return entries_.size();
}

std::vector<EntityDebounceOverride> EntityThresholdResolver::load_from_yaml(const std::string & path) {
  std::vector<EntityDebounceOverride> entries;

  if (!std::filesystem::exists(path)) {
    RCUTILS_LOG_ERROR_NAMED("entity_threshold_resolver", "Entity thresholds config file not found: %s", path.c_str());
    return entries;
  }

  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root.IsMap()) {
      RCUTILS_LOG_ERROR_NAMED("entity_threshold_resolver", "Entity thresholds config must be a YAML map, got %d in %s",
                              root.Type(), path.c_str());
      return entries;
    }

    for (const auto & item : root) {
      EntityDebounceOverride entry;
      entry.prefix = item.first.as<std::string>();

      if (!item.second.IsMap()) {
        RCUTILS_LOG_WARN_NAMED("entity_threshold_resolver", "Skipping non-map entry for prefix '%s' in %s",
                               entry.prefix.c_str(), path.c_str());
        continue;
      }

      auto node = item.second;
      if (node["confirmation_threshold"]) {
        auto val = node["confirmation_threshold"].as<int>();
        if (val > 0) {
          RCUTILS_LOG_WARN_NAMED("entity_threshold_resolver",
                                 "confirmation_threshold for '%s' should be <= 0, got %d. Using %d.",
                                 entry.prefix.c_str(), val, -val);
          val = -val;
        }
        entry.confirmation_threshold = static_cast<int32_t>(val);
      }

      if (node["healing_enabled"]) {
        entry.healing_enabled = node["healing_enabled"].as<bool>();
      }

      if (node["healing_threshold"]) {
        auto val = node["healing_threshold"].as<int>();
        if (val < 0) {
          RCUTILS_LOG_WARN_NAMED("entity_threshold_resolver",
                                 "healing_threshold for '%s' should be >= 0, got %d. Using %d.", entry.prefix.c_str(),
                                 val, -val);
          val = -val;
        }
        entry.healing_threshold = static_cast<int32_t>(val);
      }

      entries.push_back(std::move(entry));
    }

    RCUTILS_LOG_INFO_NAMED("entity_threshold_resolver", "Loaded %zu entity threshold entries from %s", entries.size(),
                           path.c_str());

  } catch (const YAML::Exception & e) {
    RCUTILS_LOG_ERROR_NAMED("entity_threshold_resolver", "Failed to parse entity thresholds config %s: %s",
                            path.c_str(), e.what());
  }

  return entries;
}

}  // namespace ros2_medkit_fault_manager
