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

#include "ros2_medkit_fault_manager/threshold_resolver.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>

#include "rcutils/logging_macros.h"

namespace ros2_medkit_fault_manager {

namespace {

/// The three fields an override may carry, as read from one YAML map node.
struct OverrideFields {
  std::optional<int32_t> confirmation_threshold;
  std::optional<bool> healing_enabled;
  std::optional<int32_t> healing_threshold;
};

/// Read the three override fields out of a YAML map node, correcting a sign an
/// operator is likely to have written the other way round. `log_name` is the
/// rcutils logger name and `key` names the entry in any message.
OverrideFields parse_override_fields(const YAML::Node & node, const char * log_name, const std::string & key) {
  OverrideFields fields;

  if (node["confirmation_threshold"]) {
    auto val = node["confirmation_threshold"].as<int>();
    if (val > 0) {
      RCUTILS_LOG_WARN_NAMED(log_name, "confirmation_threshold for '%s' should be <= 0, got %d. Using %d.", key.c_str(),
                             val, -val);
      val = -val;
    }
    fields.confirmation_threshold = static_cast<int32_t>(val);
  }

  if (node["healing_enabled"]) {
    fields.healing_enabled = node["healing_enabled"].as<bool>();
  }

  if (node["healing_threshold"]) {
    auto val = node["healing_threshold"].as<int>();
    if (val < 0) {
      RCUTILS_LOG_WARN_NAMED(log_name, "healing_threshold for '%s' should be >= 0, got %d. Using %d.", key.c_str(), val,
                             -val);
      val = -val;
    }
    fields.healing_threshold = static_cast<int32_t>(val);
  }

  return fields;
}

/// Open a threshold config file and hand back its top-level map, or an undefined
/// node when the file is missing, unparseable or not a map. Every rejection is
/// logged here so both loaders report the same way.
YAML::Node load_override_map(const std::string & path, const char * log_name, const char * what) {
  if (!std::filesystem::exists(path)) {
    RCUTILS_LOG_ERROR_NAMED(log_name, "%s config file not found: %s", what, path.c_str());
    return YAML::Node(YAML::NodeType::Undefined);
  }

  try {
    YAML::Node root = YAML::LoadFile(path);
    if (!root.IsMap()) {
      RCUTILS_LOG_ERROR_NAMED(log_name, "%s config must be a YAML map, got %d in %s", what, root.Type(), path.c_str());
      return YAML::Node(YAML::NodeType::Undefined);
    }
    return root;
  } catch (const YAML::Exception & e) {
    RCUTILS_LOG_ERROR_NAMED(log_name, "Failed to parse %s config %s: %s", what, path.c_str(), e.what());
    return YAML::Node(YAML::NodeType::Undefined);
  }
}

constexpr const char * kEntityLogName = "entity_threshold_resolver";
constexpr const char * kFaultCodeLogName = "fault_code_threshold_resolver";

}  // namespace

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
    // Check if source_id starts with the prefix at a path boundary
    if (source_id.size() >= entry.prefix.size() && source_id.compare(0, entry.prefix.size(), entry.prefix) == 0 &&
        (source_id.size() == entry.prefix.size() || source_id[entry.prefix.size()] == '/')) {
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

  YAML::Node root = load_override_map(path, kEntityLogName, "Entity thresholds");
  if (!root.IsMap()) {
    return entries;
  }

  for (const auto & item : root) {
    EntityDebounceOverride entry;
    entry.prefix = item.first.as<std::string>();

    if (!item.second.IsMap()) {
      RCUTILS_LOG_WARN_NAMED(kEntityLogName, "Skipping non-map entry for prefix '%s' in %s", entry.prefix.c_str(),
                             path.c_str());
      continue;
    }

    auto fields = parse_override_fields(item.second, kEntityLogName, entry.prefix);
    entry.confirmation_threshold = fields.confirmation_threshold;
    entry.healing_enabled = fields.healing_enabled;
    entry.healing_threshold = fields.healing_threshold;

    entries.push_back(std::move(entry));
  }

  RCUTILS_LOG_INFO_NAMED(kEntityLogName, "Loaded %zu entity threshold entries from %s", entries.size(), path.c_str());

  return entries;
}

FaultCodeThresholdResolver::FaultCodeThresholdResolver(std::vector<FaultCodeDebounceOverride> entries) {
  for (auto & entry : entries) {
    std::string key = entry.fault_code;
    entries_.emplace(std::move(key), std::move(entry));
  }
}

DebounceConfig FaultCodeThresholdResolver::resolve(const std::string & fault_code, const DebounceConfig & base) const {
  auto it = entries_.find(fault_code);
  if (it == entries_.end()) {
    return base;
  }

  // Merge on top of what the layers below produced: the code's own fields win,
  // the rest stay as the entity override (or the global default) left them.
  DebounceConfig result = base;
  const auto & entry = it->second;
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

size_t FaultCodeThresholdResolver::size() const {
  return entries_.size();
}

std::vector<FaultCodeDebounceOverride> FaultCodeThresholdResolver::load_from_yaml(const std::string & path) {
  std::vector<FaultCodeDebounceOverride> entries;

  YAML::Node root = load_override_map(path, kFaultCodeLogName, "Fault thresholds");
  if (!root.IsMap()) {
    return entries;
  }

  for (const auto & item : root) {
    FaultCodeDebounceOverride entry;
    entry.fault_code = item.first.as<std::string>();

    if (!item.second.IsMap()) {
      RCUTILS_LOG_WARN_NAMED(kFaultCodeLogName, "Skipping non-map entry for fault code '%s' in %s",
                             entry.fault_code.c_str(), path.c_str());
      continue;
    }

    auto fields = parse_override_fields(item.second, kFaultCodeLogName, entry.fault_code);
    entry.confirmation_threshold = fields.confirmation_threshold;
    entry.healing_enabled = fields.healing_enabled;
    entry.healing_threshold = fields.healing_threshold;

    entries.push_back(std::move(entry));
  }

  RCUTILS_LOG_INFO_NAMED(kFaultCodeLogName, "Loaded %zu fault-code threshold entries from %s", entries.size(),
                         path.c_str());

  return entries;
}

bool debounce_policy_equal(const DebounceConfig & a, const DebounceConfig & b) {
  return a.confirmation_threshold == b.confirmation_threshold && a.healing_enabled == b.healing_enabled &&
         a.healing_threshold == b.healing_threshold;
}

}  // namespace ros2_medkit_fault_manager
