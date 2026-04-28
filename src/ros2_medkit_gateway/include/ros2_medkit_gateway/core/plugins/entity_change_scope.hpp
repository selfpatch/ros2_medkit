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

#include <optional>
#include <string>

namespace ros2_medkit_gateway {

/**
 * @brief Scope hint passed to `PluginContext::notify_entities_changed`.
 *
 * Identifies which part of the entity tree a plugin has just mutated so the
 * gateway can limit the reload / rediscovery work. All fields are optional;
 * when every field is empty the gateway performs a full-surface refresh.
 *
 * The scope is a HINT. The gateway is free to do more work than requested if
 * the implementation is simpler that way - the only guarantee to the plugin
 * is "by the time this returns, the tree reflects my changes".
 */
struct EntityChangeScope {
  /// Area identifier whose contents changed. When set, the gateway reloads
  /// this area's subtree (components / apps / functions underneath it).
  std::optional<std::string> area_id;

  /// Component identifier whose apps / subcomponents changed. May be set
  /// without `area_id` when the plugin does not know or care which area
  /// owns the component.
  std::optional<std::string> component_id;

  /// Convenience predicate. Equivalent to "no hint - refresh everything".
  bool is_full_refresh() const noexcept {
    return !area_id.has_value() && !component_id.has_value();
  }

  /// Produce a `full_refresh()` sentinel scope. Sugar for callers that want
  /// to be explicit at the call site.
  static EntityChangeScope full_refresh() noexcept {
    return EntityChangeScope{};
  }

  /// Scope to a single component id.
  static EntityChangeScope for_component(std::string id) {
    EntityChangeScope s;
    s.component_id = std::move(id);
    return s;
  }

  /// Scope to a single area id.
  static EntityChangeScope for_area(std::string id) {
    EntityChangeScope s;
    s.area_id = std::move(id);
    return s;
  }
};

}  // namespace ros2_medkit_gateway
