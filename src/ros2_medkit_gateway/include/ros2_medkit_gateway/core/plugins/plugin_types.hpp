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

#include <nlohmann/json.hpp>
#include <string>

/// Visibility macro for plugin extern "C" exports.
/// Ensures symbols are exported even with -fvisibility=hidden builds.
#ifdef _WIN32
#define GATEWAY_PLUGIN_EXPORT __declspec(dllexport)
#else
#define GATEWAY_PLUGIN_EXPORT __attribute__((visibility("default")))
#endif

namespace ros2_medkit_gateway {

/// Current plugin API version. Plugins must export this value from plugin_api_version().
///
/// Version history:
/// - v6: ScriptProvider, locking API, extended PluginContext (entity snapshot,
///       fault listing, sampler registration).
/// - v7: PluginContext::notify_entities_changed(EntityChangeScope) for plugins
///       that mutate the entity surface at runtime. Default implementation is
///       a no-op so plugin SOURCE written against v6 compiles unchanged
///       against v7 headers (source-compatible). Binary compatibility is not
///       provided - `plugin_loader` uses strict equality against this value,
///       so a pre-compiled v6 `.so` is rejected. Out-of-tree plugins must be
///       recompiled against v7 headers; in-tree plugins that `return
///       PLUGIN_API_VERSION` pick up the bump automatically.
/// - v8: ResourceChange and NotifierFilter carry the entity type of the change
///       (SovdEntityType), and ResourceChangeNotifier::notify takes it as its
///       second argument. An id names an entity within its type only, so a
///       change of an Area no longer fires the triggers of a Function that
///       shares its id. Source-incompatible for a plugin that calls notify():
///       pass the type of the entity the change is of (UNKNOWN for a change
///       that names no entity type, which triggers match by id alone). The
///       Area and Function models gain `available` (false while every
///       gateway contributing the entity is silent), which changes their
///       layout in IntrospectionInput / NewEntities.
///       PluginContext::register_sampler takes a sampler of (SovdEntityType,
///       entity_id, resource_path): the entity the subscription was created
///       on, by type and id (source-incompatible for a sampler). PluginContext
///       gains typed calls - get_entity(type, id), list_entity_faults(type,
///       id), register_entity_capability(type, id, name),
///       get_entity_capabilities(type, id), check_lock / acquire_lock /
///       release_lock(type, ...) - which plugins should use; they answer
///       through protected *_typed hooks (vtable change) whose defaults call
///       the id-only forms, so mocks that override only those still build.
///       The id-only forms resolve a bare id only while exactly one entity
///       holds it and refuse it otherwise (one WARN per ambiguity); a lock
///       call on an id no entity holds, or typed with no entity type, takes
///       no lock (entity-not-found; check_lock: allowed). The
///       gateway owns a plugin's entities by type and id, and drops the later
///       of two entities one plugin introspects under one id with different
///       types. As for every bump, a pre-compiled v7 `.so` is rejected.
///
/// The version is bumped when the PluginContext vtable changes, when a
/// GatewayPlugin or provider interface changes incompatibly, and when a type
/// that crosses the plugin boundary changes layout.
constexpr int PLUGIN_API_VERSION = 8;

/// Log severity levels for plugin logging callback
enum class PluginLogLevel { kInfo, kWarn, kError };

/// Configuration for a single plugin loaded from YAML
struct PluginConfig {
  std::string name;       ///< Plugin key from YAML (used for parameter namespace)
  std::string path;       ///< Path to .so file
  nlohmann::json config;  ///< Per-plugin configuration (passed to configure())
};

}  // namespace ros2_medkit_gateway
