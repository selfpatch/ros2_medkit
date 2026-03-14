// Copyright 2026 selfpatch GmbH
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

#include "ros2_medkit_beacon_common/beacon_entity_mapper.hpp"

#include <algorithm>
#include <chrono>

namespace ros2_medkit_beacon {

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::Function;
using ros2_medkit_gateway::IntrospectionInput;
using ros2_medkit_gateway::IntrospectionResult;
using HintStatus = BeaconHintStore::HintStatus;

BeaconEntityMapper::BeaconEntityMapper() : config_() {
}

BeaconEntityMapper::BeaconEntityMapper(Config config) : config_(config) {
}

IntrospectionResult BeaconEntityMapper::map(const std::vector<BeaconHintStore::StoredHint> & hints,
                                            const IntrospectionInput & current_entities) const {
  IntrospectionResult result;

  for (const auto & stored : hints) {
    const auto & hint = stored.hint;
    const auto & entity_id = hint.entity_id;

    // Look up entity_id in current_entities - scan apps first, then components
    bool found_as_app = false;
    bool found_as_component = false;

    for (const auto & app : current_entities.apps) {
      if (app.id == entity_id) {
        found_as_app = true;
        break;
      }
    }

    if (!found_as_app) {
      for (const auto & comp : current_entities.components) {
        if (comp.id == entity_id) {
          found_as_component = true;
          break;
        }
      }
    }

    if (found_as_app) {
      // Create shadow App entity in result
      App shadow;
      shadow.id = entity_id;
      if (!hint.display_name.empty()) {
        shadow.name = hint.display_name;
      }
      if (!hint.component_id.empty()) {
        shadow.component_id = hint.component_id;
      }
      result.new_entities.apps.push_back(shadow);
    } else if (found_as_component) {
      // Create shadow Component entity in result
      Component shadow;
      shadow.id = entity_id;
      if (!hint.display_name.empty()) {
        shadow.name = hint.display_name;
      }
      result.new_entities.components.push_back(shadow);
    } else if (config_.allow_new_entities) {
      // Create new App for unknown entity
      App new_app;
      new_app.id = entity_id;
      new_app.source = "beacon";
      if (!hint.display_name.empty()) {
        new_app.name = hint.display_name;
      }
      if (!hint.component_id.empty()) {
        new_app.component_id = hint.component_id;
      }
      result.new_entities.apps.push_back(new_app);
    } else {
      // Unknown entity and new entities disabled - skip
      continue;
    }

    // Build and set metadata for this entity
    result.metadata[entity_id] = build_metadata(stored);

    // Apply function membership
    apply_function_membership(hint, result, current_entities);
  }

  return result;
}

nlohmann::json BeaconEntityMapper::build_metadata(const BeaconHintStore::StoredHint & stored) const {
  nlohmann::json meta;
  const auto & hint = stored.hint;

  // Freeform metadata FIRST - structured fields below will overwrite any collisions,
  // ensuring a freeform key like "status" cannot override x-medkit-beacon-status.
  for (const auto & [key, value] : hint.metadata) {
    meta["x-medkit-beacon-" + key] = value;
  }

  // Status
  meta["x-medkit-beacon-status"] = (stored.status == HintStatus::ACTIVE) ? "active" : "stale";

  // Age in seconds since last_seen
  auto now = std::chrono::steady_clock::now();
  auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - stored.last_seen);
  meta["x-medkit-beacon-age-sec"] = static_cast<double>(age.count()) / 1000.0;

  // Optional transport fields
  if (!hint.transport_type.empty()) {
    meta["x-medkit-beacon-transport-type"] = hint.transport_type;
  }
  if (!hint.negotiated_format.empty()) {
    meta["x-medkit-beacon-negotiated-format"] = hint.negotiated_format;
  }

  // Optional process diagnostics
  if (hint.process_id > 0) {
    meta["x-medkit-beacon-process-id"] = hint.process_id;
  }
  if (!hint.process_name.empty()) {
    meta["x-medkit-beacon-process-name"] = hint.process_name;
  }
  if (!hint.hostname.empty()) {
    meta["x-medkit-beacon-hostname"] = hint.hostname;
  }

  // Optional identity
  if (!hint.stable_id.empty()) {
    meta["x-medkit-beacon-stable-id"] = hint.stable_id;
  }

  // Optional depends_on
  if (!hint.depends_on.empty()) {
    meta["x-medkit-beacon-depends-on"] = hint.depends_on;
  }

  // Optional function_ids
  if (!hint.function_ids.empty()) {
    meta["x-medkit-beacon-functions"] = hint.function_ids;
  }

  return meta;
}

void BeaconEntityMapper::apply_function_membership(const BeaconHint & hint, IntrospectionResult & result,
                                                   const IntrospectionInput & current) const {
  for (const auto & func_id : hint.function_ids) {
    // Check if this function exists in current_entities
    bool found = false;
    for (const auto & func : current.functions) {
      if (func.id == func_id) {
        found = true;
        break;
      }
    }

    if (!found) {
      // Function doesn't exist in current entities - skip (metadata already records it)
      continue;
    }

    // Find or create this function in result.new_entities.functions
    auto it = std::find_if(result.new_entities.functions.begin(), result.new_entities.functions.end(),
                           [&func_id](const Function & f) {
                             return f.id == func_id;
                           });

    if (it != result.new_entities.functions.end()) {
      // Function already in result - add entity_id to hosts if not already there
      auto host_it = std::find(it->hosts.begin(), it->hosts.end(), hint.entity_id);
      if (host_it == it->hosts.end()) {
        it->hosts.push_back(hint.entity_id);
      }
    } else {
      // Create new Function entry in result
      Function func;
      func.id = func_id;
      // Copy name from current entities
      for (const auto & current_func : current.functions) {
        if (current_func.id == func_id) {
          func.name = current_func.name;
          break;
        }
      }
      func.hosts.push_back(hint.entity_id);
      result.new_entities.functions.push_back(func);
    }
  }
}

}  // namespace ros2_medkit_beacon
