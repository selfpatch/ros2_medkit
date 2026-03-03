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

#include "ros2_medkit_gateway/plugins/plugin_manager.hpp"

#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_gateway {

namespace {
auto logger() {
  return rclcpp::get_logger("plugin_manager");
}
}  // namespace

void PluginManager::setup_plugin_logging(GatewayPlugin & plugin) {
  std::string plugin_name = plugin.name();
  plugin.set_logger([plugin_name](PluginLogLevel level, const std::string & msg) {
    auto log = rclcpp::get_logger("plugin." + plugin_name);
    switch (level) {
      case PluginLogLevel::kInfo:
        RCLCPP_INFO(log, "%s", msg.c_str());
        break;
      case PluginLogLevel::kWarn:
        RCLCPP_WARN(log, "%s", msg.c_str());
        break;
      case PluginLogLevel::kError:
        RCLCPP_ERROR(log, "%s", msg.c_str());
        break;
    }
  });
}

PluginManager::~PluginManager() {
  shutdown_all();
  // GatewayPluginLoadResult RAII handles destruction order
  plugins_.clear();
}

void PluginManager::add_plugin(std::unique_ptr<GatewayPlugin> plugin) {
  LoadedPlugin lp;
  lp.config = nlohmann::json::object();

  // For in-process plugins, use dynamic_cast (safe within same binary)
  lp.update_provider = dynamic_cast<UpdateProvider *>(plugin.get());
  lp.introspection_provider = dynamic_cast<IntrospectionProvider *>(plugin.get());
  lp.log_provider = dynamic_cast<LogProvider *>(plugin.get());

  // Cache first UpdateProvider, warn on duplicates
  if (lp.update_provider) {
    if (!first_update_provider_) {
      first_update_provider_ = lp.update_provider;
    } else {
      RCLCPP_WARN(logger(), "Multiple UpdateProvider plugins loaded - ignoring '%s'", plugin->name().c_str());
    }
  }

  // Cache first LogProvider; additional LogProvider plugins are observers only
  if (lp.log_provider) {
    if (!first_log_provider_) {
      first_log_provider_ = lp.log_provider;
    } else {
      RCLCPP_DEBUG(logger(), "LogProvider plugin '%s' registered as observer only", plugin->name().c_str());
    }
  }

  setup_plugin_logging(*plugin);
  lp.load_result.plugin = std::move(plugin);
  plugins_.push_back(std::move(lp));
}

size_t PluginManager::load_plugins(const std::vector<PluginConfig> & configs) {
  size_t loaded = 0;
  for (const auto & cfg : configs) {
    auto result = PluginLoader::load(cfg.path);
    if (result) {
      RCLCPP_INFO(logger(), "Loaded plugin '%s' from %s", result->plugin->name().c_str(), cfg.path.c_str());

      LoadedPlugin lp;
      lp.config = cfg.config;

      // Provider pointers from extern "C" query functions (safe across dlopen boundary)
      lp.update_provider = result->update_provider;
      lp.introspection_provider = result->introspection_provider;
      // LogProvider: discovered via dynamic_cast across the dlopen boundary.
      // Limitation: with -fvisibility=hidden the RTTI symbol for LogProvider may exist in
      // both the gateway binary and the .so, causing dynamic_cast to silently return nullptr
      // even when the plugin does implement LogProvider. The safe fix is an extern "C" query
      // function (like get_update_provider_ptr()), but that requires an ABI change deferred
      // to a future release. For now, LogProvider plugins must be built with matching visibility
      // settings (e.g. -fvisibility=default or explicit attribute((visibility("default"))) on
      // the LogProvider vtable).
      lp.log_provider = dynamic_cast<LogProvider *>(result->plugin.get());

      // Cache first UpdateProvider, warn on duplicates
      if (lp.update_provider) {
        if (!first_update_provider_) {
          first_update_provider_ = lp.update_provider;
        } else {
          RCLCPP_WARN(logger(), "Multiple UpdateProvider plugins loaded - ignoring '%s'",
                      result->plugin->name().c_str());
        }
      }

      // Cache first LogProvider; additional LogProvider plugins are observers only
      if (lp.log_provider) {
        if (!first_log_provider_) {
          first_log_provider_ = lp.log_provider;
        } else {
          RCLCPP_DEBUG(logger(), "LogProvider plugin '%s' registered as observer only", result->plugin->name().c_str());
        }
      }

      setup_plugin_logging(*result->plugin);
      lp.load_result = std::move(*result);
      plugins_.push_back(std::move(lp));
      ++loaded;
    } else {
      RCLCPP_ERROR(logger(), "Failed to load plugin from %s: %s", cfg.path.c_str(), result.error().c_str());
    }
  }
  return loaded;
}

void PluginManager::disable_plugin(LoadedPlugin & lp) {
  if (lp.load_result.plugin) {
    try {
      lp.load_result.plugin->shutdown();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw during shutdown(): %s", lp.load_result.plugin->name().c_str(), e.what());
    } catch (...) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw unknown exception during shutdown()",
                  lp.load_result.plugin->name().c_str());
    }
  }
  // Invalidate cached provider if this plugin was the source, re-scan for next
  if (first_update_provider_ && lp.update_provider == first_update_provider_) {
    first_update_provider_ = nullptr;
    for (const auto & other : plugins_) {
      if (&other != &lp && other.load_result.plugin && other.update_provider) {
        first_update_provider_ = other.update_provider;
        break;
      }
    }
  }
  if (first_log_provider_ && lp.log_provider == first_log_provider_) {
    first_log_provider_ = nullptr;
    for (const auto & other : plugins_) {
      if (&other != &lp && other.load_result.plugin && other.log_provider) {
        first_log_provider_ = other.log_provider;
        break;
      }
    }
  }
  lp.update_provider = nullptr;
  lp.introspection_provider = nullptr;
  lp.log_provider = nullptr;
  lp.load_result.update_provider = nullptr;
  lp.load_result.introspection_provider = nullptr;
  lp.load_result.plugin.reset();
}

void PluginManager::configure_plugins() {
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    try {
      lp.load_result.plugin->configure(lp.config);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw during configure(): %s", lp.load_result.plugin->name().c_str(),
                   e.what());
      disable_plugin(lp);
    } catch (...) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw unknown exception during configure()",
                   lp.load_result.plugin->name().c_str());
      disable_plugin(lp);
    }
  }
}

void PluginManager::set_context(PluginContext & context) {
  context_ = &context;
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    try {
      lp.load_result.plugin->set_context(context);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw during set_context(): %s - disabling",
                   lp.load_result.plugin->name().c_str(), e.what());
      disable_plugin(lp);
    } catch (...) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw unknown exception during set_context() - disabling",
                   lp.load_result.plugin->name().c_str());
      disable_plugin(lp);
    }
  }
}

void PluginManager::register_routes(httplib::Server & server, const std::string & api_prefix) {
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    try {
      lp.load_result.plugin->register_routes(server, api_prefix);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw during register_routes(): %s - disabling",
                   lp.load_result.plugin->name().c_str(), e.what());
      disable_plugin(lp);
    } catch (...) {
      RCLCPP_ERROR(logger(), "Plugin '%s' threw unknown exception during register_routes() - disabling",
                   lp.load_result.plugin->name().c_str());
      disable_plugin(lp);
    }
  }
}

void PluginManager::shutdown_all() {
  if (shutdown_called_) {
    return;
  }
  shutdown_called_ = true;
  for (auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    try {
      lp.load_result.plugin->shutdown();
    } catch (const std::exception & e) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw during shutdown(): %s", lp.load_result.plugin->name().c_str(), e.what());
    } catch (...) {
      RCLCPP_WARN(logger(), "Plugin '%s' threw unknown exception during shutdown()",
                  lp.load_result.plugin->name().c_str());
    }
  }
}

UpdateProvider * PluginManager::get_update_provider() const {
  return first_update_provider_;
}

std::vector<IntrospectionProvider *> PluginManager::get_introspection_providers() const {
  std::vector<IntrospectionProvider *> result;
  for (const auto & lp : plugins_) {
    if (!lp.load_result.plugin) {
      continue;
    }
    if (lp.introspection_provider) {
      result.push_back(lp.introspection_provider);
    }
  }
  return result;
}

LogProvider * PluginManager::get_log_provider() const {
  return first_log_provider_;
}

std::vector<LogProvider *> PluginManager::get_log_observers() const {
  // NOTE: plugins_ is read here (called from the ROS 2 executor thread via LogManager::on_rosout)
  // while disable_plugin() can write plugins_ from the HTTP handler thread. This is the same
  // pre-existing race as get_introspection_providers() and get_update_provider(). In practice
  // disable_plugin() only nulls pointers/resets unique_ptrs and does not resize plugins_, so the
  // race is benign on all current platforms. A proper fix would require a shared_mutex or
  // copying the observer list at subscription time.
  std::vector<LogProvider *> result;
  for (const auto & lp : plugins_) {
    if (lp.log_provider) {
      result.push_back(lp.log_provider);
    }
  }
  return result;
}

bool PluginManager::has_plugins() const {
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin) {
      return true;
    }
  }
  return false;
}

std::vector<std::string> PluginManager::plugin_names() const {
  std::vector<std::string> names;
  for (const auto & lp : plugins_) {
    if (lp.load_result.plugin) {
      names.push_back(lp.load_result.plugin->name());
    }
  }
  return names;
}

}  // namespace ros2_medkit_gateway
