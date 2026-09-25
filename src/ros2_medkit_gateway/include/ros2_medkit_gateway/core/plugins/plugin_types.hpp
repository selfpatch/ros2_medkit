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
/// - v8: FaultProvider::clear_fault_record(entity_id, fault_code, owner), which
///       the gateway now calls to clear one fault record. Its default
///       implementation calls the unchanged two-argument clear_fault(), so
///       plugin SOURCE written against v7 compiles unchanged against v8
///       headers (source-compatible). The new virtual changes the
///       FaultProvider vtable, so a pre-compiled v7 `.so` is rejected by the
///       strict equality check and must be recompiled against v8 headers.
///       GatewayPlugin also gains log_sink(), a copy of the log sink for work
///       that can outlive the plugin, and set_logger() becomes protected so a
///       plugin hosted without a PluginManager can wire its own. Both are
///       non-virtual and change no layout.
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
