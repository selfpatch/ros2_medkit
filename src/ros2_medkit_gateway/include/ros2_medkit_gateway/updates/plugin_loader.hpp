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

#include <memory>
#include <string>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/updates/update_backend.hpp"

namespace ros2_medkit_gateway {

/// Result of loading a plugin: backend + dlopen handle (for cleanup)
struct PluginLoadResult {
  std::unique_ptr<UpdateBackend> backend;
  void * handle = nullptr;  // dlopen handle, pass to UpdateManager
};

/**
 * @brief Loads an UpdateBackend plugin from a shared library (.so).
 *
 * The .so must export: extern "C" UpdateBackend* create_update_backend();
 */
class UpdatePluginLoader {
 public:
  /// Load plugin from .so path. Returns backend + handle.
  static tl::expected<PluginLoadResult, std::string> load(const std::string & plugin_path);
};

}  // namespace ros2_medkit_gateway
