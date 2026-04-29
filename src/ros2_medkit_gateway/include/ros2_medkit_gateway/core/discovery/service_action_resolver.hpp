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

#include <optional>
#include <string>

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Lookup interface for resolving services / actions by component
 *        namespace + operation name.
 *
 * Decouples OperationManager from DiscoveryManager so the manager body
 * compiles inside the neutral core layer without pulling rclcpp through
 * the discovery_manager.hpp transitive include chain. DiscoveryManager
 * implements this interface, but mock resolvers in unit tests can also.
 */
class ServiceActionResolver {
 public:
  ServiceActionResolver() = default;
  ServiceActionResolver(const ServiceActionResolver &) = delete;
  ServiceActionResolver & operator=(const ServiceActionResolver &) = delete;
  ServiceActionResolver(ServiceActionResolver &&) = delete;
  ServiceActionResolver & operator=(ServiceActionResolver &&) = delete;
  virtual ~ServiceActionResolver() = default;

  /**
   * @brief Resolve a service by component namespace + operation name.
   * @return ServiceInfo if a match exists, std::nullopt otherwise.
   */
  virtual std::optional<ServiceInfo> find_service(const std::string & component_ns,
                                                  const std::string & operation_name) const = 0;

  /**
   * @brief Resolve an action by component namespace + operation name.
   * @return ActionInfo if a match exists, std::nullopt otherwise.
   */
  virtual std::optional<ActionInfo> find_action(const std::string & component_ns,
                                                const std::string & operation_name) const = 0;
};

}  // namespace ros2_medkit_gateway
