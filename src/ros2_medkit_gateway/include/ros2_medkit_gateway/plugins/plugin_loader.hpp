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

#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"

#include <memory>
#include <string>

#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

class UpdateProvider;
class IntrospectionProvider;
class LogProvider;

/**
 * @brief Result of loading a gateway plugin.
 *
 * RAII wrapper guaranteeing correct destruction order: provider pointers
 * are invalidated first, then the plugin is destroyed, then dlclose is called.
 * Move-only (no copy).
 */
struct GatewayPluginLoadResult {
  GatewayPluginLoadResult() = default;
  ~GatewayPluginLoadResult();

  GatewayPluginLoadResult(GatewayPluginLoadResult && other) noexcept;
  GatewayPluginLoadResult & operator=(GatewayPluginLoadResult && other) noexcept;

  GatewayPluginLoadResult(const GatewayPluginLoadResult &) = delete;
  GatewayPluginLoadResult & operator=(const GatewayPluginLoadResult &) = delete;

  std::unique_ptr<GatewayPlugin> plugin;

  /// Non-owning pointer to UpdateProvider interface (null if plugin doesn't provide updates).
  /// Lifetime tied to plugin - do not use after plugin is destroyed.
  UpdateProvider * update_provider = nullptr;

  /// Non-owning pointer to IntrospectionProvider interface (null if not provided).
  /// Lifetime tied to plugin - do not use after plugin is destroyed.
  IntrospectionProvider * introspection_provider = nullptr;

  /// Non-owning pointer to LogProvider interface (null if not provided).
  /// Lifetime tied to plugin - do not use after plugin is destroyed.
  LogProvider * log_provider = nullptr;

 private:
  friend class PluginLoader;
  void * handle_ = nullptr;  // dlopen handle, destroyed after plugin
};

/**
 * @brief Loads a GatewayPlugin from a shared library (.so).
 *
 * The .so must export (with GATEWAY_PLUGIN_EXPORT visibility):
 *   extern "C" int plugin_api_version();       // must return PLUGIN_API_VERSION
 *   extern "C" GatewayPlugin* create_plugin(); // factory
 *
 * Optionally, for provider interface discovery (avoids RTTI across dlopen boundary):
 *   extern "C" UpdateProvider* get_update_provider(GatewayPlugin* plugin);
 *   extern "C" IntrospectionProvider* get_introspection_provider(GatewayPlugin* plugin);
 *   extern "C" LogProvider* get_log_provider(GatewayPlugin* plugin);
 *
 * Path requirements: must be absolute, have .so extension, and resolve to a real file.
 */
class PluginLoader {
 public:
  /// Load plugin from .so path. Returns plugin + handle, or error string.
  static tl::expected<GatewayPluginLoadResult, std::string> load(const std::string & plugin_path);
};

}  // namespace ros2_medkit_gateway
