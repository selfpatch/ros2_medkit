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

#include "ros2_medkit_gateway/updates/plugin_loader.hpp"

#include <dlfcn.h>

namespace ros2_medkit_gateway {

tl::expected<PluginLoadResult, std::string> UpdatePluginLoader::load(const std::string & plugin_path) {
  void * handle = dlopen(plugin_path.c_str(), RTLD_LAZY);
  if (!handle) {
    return tl::make_unexpected("Failed to load plugin '" + plugin_path + "': " + std::string(dlerror()));
  }

  // Clear any existing error
  dlerror();

  using FactoryFn = UpdateBackend * (*)();
  auto factory = reinterpret_cast<FactoryFn>(dlsym(handle, "create_update_backend"));

  const char * error = dlerror();
  if (error) {
    dlclose(handle);
    return tl::make_unexpected("Failed to find 'create_update_backend' in '" + plugin_path +
                               "': " + std::string(error));
  }

  UpdateBackend * raw_backend = factory();
  if (!raw_backend) {
    dlclose(handle);
    return tl::make_unexpected("Factory 'create_update_backend' returned null in '" + plugin_path + "'");
  }

  PluginLoadResult result;
  result.backend = std::unique_ptr<UpdateBackend>(raw_backend);
  result.handle = handle;
  return result;
}

}  // namespace ros2_medkit_gateway
