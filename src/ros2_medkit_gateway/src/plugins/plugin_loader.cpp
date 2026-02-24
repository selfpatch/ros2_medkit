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

#include <dlfcn.h>

namespace ros2_medkit_gateway {

tl::expected<GatewayPluginLoadResult, std::string> PluginLoader::load(const std::string & plugin_path) {
  void * handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
  if (!handle) {
    return tl::make_unexpected("Failed to load plugin '" + plugin_path + "': " + std::string(dlerror()));
  }

  // Clear any existing error
  dlerror();

  // --- Check API version ---
  using VersionFn = int (*)();
  auto version_fn = reinterpret_cast<VersionFn>(dlsym(handle, "plugin_api_version"));

  const char * error = dlerror();
  if (error) {
    dlclose(handle);
    return tl::make_unexpected("Failed to find 'plugin_api_version' in '" + plugin_path + "': " + std::string(error));
  }

  int version = 0;
  try {
    version = version_fn();
  } catch (...) {
    dlclose(handle);
    return tl::make_unexpected("'plugin_api_version' threw exception in '" + plugin_path + "'");
  }

  if (version != PLUGIN_API_VERSION) {
    dlclose(handle);
    return tl::make_unexpected("API version mismatch in '" + plugin_path + "': plugin=" + std::to_string(version) +
                               " expected=" + std::to_string(PLUGIN_API_VERSION));
  }

  // --- Call factory ---
  dlerror();

  using FactoryFn = GatewayPlugin * (*)();
  auto factory = reinterpret_cast<FactoryFn>(dlsym(handle, "create_plugin"));

  error = dlerror();
  if (error) {
    dlclose(handle);
    return tl::make_unexpected("Failed to find 'create_plugin' in '" + plugin_path + "': " + std::string(error));
  }

  GatewayPlugin * raw_plugin = nullptr;
  try {
    raw_plugin = factory();
  } catch (const std::exception & e) {
    dlclose(handle);
    return tl::make_unexpected("Factory 'create_plugin' threw exception in '" + plugin_path + "': " + e.what());
  } catch (...) {
    dlclose(handle);
    return tl::make_unexpected("Factory 'create_plugin' threw unknown exception in '" + plugin_path + "'");
  }

  if (!raw_plugin) {
    dlclose(handle);
    return tl::make_unexpected("Factory 'create_plugin' returned null in '" + plugin_path + "'");
  }

  GatewayPluginLoadResult result;
  result.plugin = std::unique_ptr<GatewayPlugin>(raw_plugin);
  result.handle = handle;
  return result;
}

}  // namespace ros2_medkit_gateway
