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

#include "ros2_medkit_gateway/plugins/plugin_loader.hpp"

#include "ros2_medkit_gateway/plugins/plugin_types.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/providers/update_provider.hpp"

#include <dlfcn.h>

#include <filesystem>

namespace ros2_medkit_gateway {

// --- GatewayPluginLoadResult RAII ---

GatewayPluginLoadResult::~GatewayPluginLoadResult() {
  update_provider = nullptr;
  introspection_provider = nullptr;
  plugin.reset();
  if (handle_) {
    dlclose(handle_);
  }
}

GatewayPluginLoadResult::GatewayPluginLoadResult(GatewayPluginLoadResult && other) noexcept
  : plugin(std::move(other.plugin))
  , update_provider(other.update_provider)
  , introspection_provider(other.introspection_provider)
  , handle_(other.handle_) {
  other.update_provider = nullptr;
  other.introspection_provider = nullptr;
  other.handle_ = nullptr;
}

GatewayPluginLoadResult & GatewayPluginLoadResult::operator=(GatewayPluginLoadResult && other) noexcept {
  if (this != &other) {
    // Destroy current state in correct order
    update_provider = nullptr;
    introspection_provider = nullptr;
    plugin.reset();
    if (handle_) {
      dlclose(handle_);
    }

    // Move from other
    plugin = std::move(other.plugin);
    update_provider = other.update_provider;
    introspection_provider = other.introspection_provider;
    handle_ = other.handle_;

    other.update_provider = nullptr;
    other.introspection_provider = nullptr;
    other.handle_ = nullptr;
  }
  return *this;
}

// --- PluginLoader ---

tl::expected<GatewayPluginLoadResult, std::string> PluginLoader::load(const std::string & plugin_path) {
  // --- Validate path ---
  std::filesystem::path fs_path(plugin_path);

  if (!fs_path.is_absolute()) {
    return tl::make_unexpected("Plugin path must be absolute: " + plugin_path);
  }

  if (fs_path.extension() != ".so") {
    return tl::make_unexpected("Plugin path must have .so extension: " + plugin_path);
  }

  std::error_code ec;
  auto canonical_path = std::filesystem::canonical(fs_path, ec);
  if (ec) {
    return tl::make_unexpected("Plugin path does not exist or is not accessible: " + plugin_path);
  }

  // --- dlopen ---
  void * handle = dlopen(canonical_path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!handle) {
    return tl::make_unexpected("Failed to load plugin: " + std::string(dlerror()));
  }

  // Scope guard: dlclose on any error path. Released on success.
  auto handle_guard = std::unique_ptr<void, int (*)(void *)>(handle, dlclose);

  // Clear any existing error
  dlerror();

  // --- Check API version ---
  using VersionFn = int (*)();
  auto version_fn = reinterpret_cast<VersionFn>(dlsym(handle, "plugin_api_version"));

  const char * error = dlerror();
  if (error) {
    return tl::make_unexpected(std::string("Failed to find 'plugin_api_version': ") + std::string(error));
  }

  int version = 0;
  try {
    version = version_fn();
  } catch (...) {
    return tl::make_unexpected("'plugin_api_version' threw exception in plugin");
  }

  if (version != PLUGIN_API_VERSION) {
    return tl::make_unexpected("API version mismatch: plugin=" + std::to_string(version) +
                               " expected=" + std::to_string(PLUGIN_API_VERSION) +
                               ". Rebuild the plugin against matching gateway headers.");
  }

  // --- Call factory ---
  dlerror();

  using FactoryFn = GatewayPlugin * (*)();
  auto factory = reinterpret_cast<FactoryFn>(dlsym(handle, "create_plugin"));

  error = dlerror();
  if (error) {
    return tl::make_unexpected(std::string("Failed to find 'create_plugin': ") + std::string(error));
  }

  GatewayPlugin * raw_plugin = nullptr;
  try {
    raw_plugin = factory();
  } catch (const std::exception & e) {
    return tl::make_unexpected(std::string("Factory 'create_plugin' threw exception: ") + e.what());
  } catch (...) {
    return tl::make_unexpected("Factory 'create_plugin' threw unknown exception");
  }

  if (!raw_plugin) {
    return tl::make_unexpected("Factory 'create_plugin' returned null");
  }

  // --- Query provider interfaces via extern "C" (no RTTI across dlopen boundary) ---
  GatewayPluginLoadResult result;
  result.plugin = std::unique_ptr<GatewayPlugin>(raw_plugin);

  using UpdateProviderFn = UpdateProvider * (*)(GatewayPlugin *);
  auto update_fn = reinterpret_cast<UpdateProviderFn>(dlsym(handle, "get_update_provider"));
  if (update_fn) {
    result.update_provider = update_fn(raw_plugin);
  }

  using IntrospectionProviderFn = IntrospectionProvider * (*)(GatewayPlugin *);
  auto introspection_fn = reinterpret_cast<IntrospectionProviderFn>(dlsym(handle, "get_introspection_provider"));
  if (introspection_fn) {
    result.introspection_provider = introspection_fn(raw_plugin);
  }

  // Transfer handle ownership to result (disarm scope guard)
  result.handle_ = handle_guard.release();
  return result;
}

}  // namespace ros2_medkit_gateway
