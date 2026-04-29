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

#include <atomic>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_gateway/core/configuration/parameter_types.hpp"
#include "ros2_medkit_gateway/core/transports/parameter_transport.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

/**
 * @brief Application service for ROS 2 node parameters.
 *
 * Pure C++; ROS-side I/O is performed by the injected ParameterTransport
 * adapter (typically Ros2ParameterTransport). All rclcpp::SyncParametersClient
 * usage, the rclcpp::Parameter defaults cache, the negative-cache for
 * unreachable nodes, the spin_mutex serialising parameter-client spins, and
 * the JSON <-> rclcpp::ParameterValue conversion helpers live in the adapter.
 *
 * The manager retains the high-level orchestration:
 *   - shutdown ordering contract (idempotent shutdown forwards to transport)
 *   - reset orchestration (compose set_parameter with cached default values)
 *   - the public CRUD surface consumed by handler_context handlers
 *
 * Self-node short-circuiting (avoiding IPC for the gateway's own parameters)
 * is delegated to the transport because identifying the gateway's own FQN
 * requires the rclcpp node handle.
 */
class ConfigurationManager {
 public:
  /**
   * @param transport Concrete ParameterTransport adapter. Manager takes
   *                  shared ownership.
   */
  explicit ConfigurationManager(std::shared_ptr<ParameterTransport> transport);

  ~ConfigurationManager();

  ConfigurationManager(const ConfigurationManager &) = delete;
  ConfigurationManager & operator=(const ConfigurationManager &) = delete;
  ConfigurationManager(ConfigurationManager &&) = delete;
  ConfigurationManager & operator=(ConfigurationManager &&) = delete;

  /// Idempotent teardown. Forwards to the transport's shutdown so cached
  /// parameter clients drop their references before rclcpp::shutdown() runs.
  /// Must be called before rclcpp::shutdown() to prevent use-after-free of
  /// the parameter-client internal node held by the transport.
  void shutdown();

  /// List all parameters for a node.
  /// @param node_name Fully qualified node name (e.g., "/powertrain/engine/engine_temp_sensor").
  /// @return ParameterResult with array of {name, value, type} objects.
  ParameterResult list_parameters(const std::string & node_name);

  /// Get a specific parameter value.
  /// @param node_name Fully qualified node name.
  /// @param param_name Parameter name.
  /// @return ParameterResult with {name, value, type, description, read_only}.
  ParameterResult get_parameter(const std::string & node_name, const std::string & param_name);

  /// Set a parameter value.
  /// @param node_name Fully qualified node name.
  /// @param param_name Parameter name.
  /// @param value New value (JSON type will be converted to the appropriate ROS 2 type by the transport).
  /// @return ParameterResult with {name, value, type}.
  ParameterResult set_parameter(const std::string & node_name, const std::string & param_name, const json & value);

  /// Reset a specific parameter to its default (initial) value.
  /// @param node_name Fully qualified node name.
  /// @param param_name Parameter name.
  /// @return ParameterResult with reset parameter info.
  ParameterResult reset_parameter(const std::string & node_name, const std::string & param_name);

  /// Reset all parameters of a node to their default (initial) values.
  /// @param node_name Fully qualified node name.
  /// @return ParameterResult with count of reset parameters.
  ParameterResult reset_all_parameters(const std::string & node_name);

 private:
  /// Return SHUT_DOWN error result.
  static ParameterResult shut_down_result();

  std::shared_ptr<ParameterTransport> transport_;

  /// Shutdown flag - prevents delegation to the transport after teardown.
  std::atomic<bool> shutdown_{false};
};

}  // namespace ros2_medkit_gateway
