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

#include <vector>

namespace ros2_medkit_gateway {

class LogProvider;  // forward declaration

/**
 * @brief Port: lookup of LogProvider plugins for the LogManager.
 *
 * Decouples LogManager from PluginManager so the manager body can live in the
 * middleware-neutral build layer alongside the other managers. The production
 * adapter is PluginManager itself (which implements this interface inline);
 * unit tests can supply a lightweight mock instead.
 *
 * The two methods correspond to the two LogManager-internal usages:
 * - primary(): consulted at construction (manages_ingestion short-circuit)
 *              and at every get_logs / get_config / update_config call to
 *              decide whether to delegate to a plugin.
 * - observers(): consulted on every source-emitted entry so the observer
 *                pattern can fan out to all loaded plugins.
 */
class LogProviderRegistry {
 public:
  LogProviderRegistry() = default;
  LogProviderRegistry(const LogProviderRegistry &) = delete;
  LogProviderRegistry & operator=(const LogProviderRegistry &) = delete;
  LogProviderRegistry(LogProviderRegistry &&) = delete;
  LogProviderRegistry & operator=(LogProviderRegistry &&) = delete;
  virtual ~LogProviderRegistry() = default;

  /// First-loaded LogProvider plugin (used as the primary backend for queries
  /// and config) or nullptr when no such plugin is registered.
  virtual LogProvider * primary_log_provider() const = 0;

  /// All loaded LogProvider plugins (observer pattern: each receives every
  /// source-emitted LogEntry). Order is implementation-defined; LogManager
  /// iterates the full list.
  virtual std::vector<LogProvider *> log_observers() const = 0;
};

}  // namespace ros2_medkit_gateway
