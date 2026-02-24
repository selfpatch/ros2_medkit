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

/// Result of loading a gateway plugin: plugin instance + dlopen handle (for cleanup)
struct GatewayPluginLoadResult {
  std::unique_ptr<GatewayPlugin> plugin;
  void * handle = nullptr;  // dlopen handle, caller owns dlclose
};

/**
 * @brief Loads a GatewayPlugin from a shared library (.so).
 *
 * The .so must export:
 *   extern "C" int plugin_api_version();       // must return PLUGIN_API_VERSION
 *   extern "C" GatewayPlugin* create_plugin(); // factory
 */
class PluginLoader {
 public:
  /// Load plugin from .so path. Returns plugin + handle, or error string.
  static tl::expected<GatewayPluginLoadResult, std::string> load(const std::string & plugin_path);
};

}  // namespace ros2_medkit_gateway
