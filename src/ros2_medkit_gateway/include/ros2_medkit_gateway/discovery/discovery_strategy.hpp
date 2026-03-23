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

#include <string>
#include <vector>

namespace ros2_medkit_gateway {

// Forward declarations
struct Area;
struct Component;
struct App;
struct Function;

namespace discovery {

/**
 * @brief Interface for discovery strategies (Strategy Pattern)
 *
 * Allows different discovery implementations:
 * - RuntimeDiscoveryStrategy: Discovers from ROS 2 graph (current behavior)
 * - ManifestDiscoveryStrategy: Discovers from YAML manifest (future)
 * - HybridDiscoveryStrategy: Combines both (future)
 *
 * @see RuntimeDiscoveryStrategy
 */
class DiscoveryStrategy {
 public:
  virtual ~DiscoveryStrategy() = default;

  /// Discover all areas
  virtual std::vector<Area> discover_areas() = 0;

  /// Discover all components
  virtual std::vector<Component> discover_components() = 0;

  /// Discover all apps (empty for runtime-only strategy)
  virtual std::vector<App> discover_apps() = 0;

  /// Discover all functions (empty for runtime-only strategy)
  virtual std::vector<Function> discover_functions() = 0;

  /// Get strategy name for logging
  virtual std::string get_name() const = 0;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
