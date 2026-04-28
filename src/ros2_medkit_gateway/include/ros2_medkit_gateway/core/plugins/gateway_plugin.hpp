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

#include "ros2_medkit_gateway/core/plugins/plugin_http_types.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_types.hpp"

#include <functional>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

class PluginContext;

/**
 * @brief Base class for all gateway plugins
 *
 * Plugins are loaded as shared libraries (.so) via dlopen/dlsym.
 * Each .so must export two extern "C" functions:
 *   int plugin_api_version();           // must return PLUGIN_API_VERSION
 *   GatewayPlugin* create_plugin();     // factory
 *
 * Plugins implement this base class plus one or more provider interfaces
 * (UpdateProvider, IntrospectionProvider) via multiple inheritance.
 *
 * @see PluginManager for loading and lifecycle orchestration
 * @see UpdateProvider, IntrospectionProvider for typed interfaces
 */
class GatewayPlugin {
 public:
  /// Describes a single REST route registered by a plugin
  struct PluginRoute {
    std::string method;   ///< HTTP method ("GET", "POST", "PUT", "DELETE")
    std::string pattern;  ///< Regex pattern relative to api_prefix
    std::function<void(const PluginRequest &, PluginResponse &)> handler;
  };

  virtual ~GatewayPlugin() = default;

  /**
   * @brief Unique name for this plugin
   * @return Plugin name (e.g., "systemd", "procfs", "mender_ota")
   */
  virtual std::string name() const = 0;

  /**
   * @brief Configure the plugin
   *
   * Called once after loading with per-plugin config from YAML.
   *
   * @param config JSON configuration object
   */
  virtual void configure(const nlohmann::json & config) = 0;

  /**
   * @brief Receive gateway context
   *
   * Called after configure(). Provides access to the ROS 2 node,
   * entity cache, fault data, and HTTP handler utilities.
   * Store the reference if needed during runtime.
   *
   * @param context Gateway plugin context (outlives this plugin)
   */
  virtual void set_context(PluginContext & /*context*/) {
  }

  /**
   * @brief Return custom REST routes for this plugin
   *
   * Called once during REST server setup. The gateway registers the returned
   * routes on the HTTP server, wrapping PluginRequest/PluginResponse around
   * the underlying library types.
   *
   * @return Routes to register (method, pattern, handler)
   */
  virtual std::vector<PluginRoute> get_routes() {
    return {};
  }

  /**
   * @brief Shutdown hook for cleanup
   *
   * Called before the plugin is destroyed. Use for releasing
   * resources, closing connections, etc.
   */
  virtual void shutdown() {
  }

 protected:
  /// Log an informational message (routed to gateway's ROS 2 logger)
  void log_info(const std::string & msg) const {
    if (log_fn_) {
      log_fn_(PluginLogLevel::kInfo, msg);
    }
  }

  /// Log a warning message
  void log_warn(const std::string & msg) const {
    if (log_fn_) {
      log_fn_(PluginLogLevel::kWarn, msg);
    }
  }

  /// Log an error message
  void log_error(const std::string & msg) const {
    if (log_fn_) {
      log_fn_(PluginLogLevel::kError, msg);
    }
  }

 private:
  friend class PluginManager;  // Sets log_fn_ after construction

  /// Logging callback set by PluginManager. Routes to rclcpp::get_logger("plugin.<name>").
  std::function<void(PluginLogLevel, const std::string &)> log_fn_;

  /// Called by PluginManager to wire up logging
  void set_logger(std::function<void(PluginLogLevel, const std::string &)> fn) {
    log_fn_ = std::move(fn);
  }
};

}  // namespace ros2_medkit_gateway
