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

#include <chrono>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/configuration/parameter_types.hpp"

namespace ros2_medkit_gateway {

/// Single parameter descriptor returned by list_parameters.
struct ParameterDescriptor {
  std::string name;
  std::string type;  ///< "bool", "integer", "double", "string", "byte_array",
                     ///< "bool_array", "integer_array", "double_array", "string_array"
  json value = nullptr;
  std::string description;
  bool read_only = false;
};

class ParameterTransport {
 public:
  ParameterTransport() = default;
  ParameterTransport(const ParameterTransport &) = delete;
  ParameterTransport & operator=(const ParameterTransport &) = delete;
  ParameterTransport(ParameterTransport &&) = delete;
  ParameterTransport & operator=(ParameterTransport &&) = delete;
  virtual ~ParameterTransport() = default;

  /// True iff `node_name` resolves to the gateway's own node (self-parameter
  /// lookup short-circuit). Manager uses this to avoid IPC for own params.
  virtual bool is_self_node(const std::string & node_name) const = 0;

  /// List all parameters with descriptors for `node_name`.
  virtual ParameterResult list_parameters(const std::string & node_name) = 0;

  /// Get one parameter with descriptor.
  virtual ParameterResult get_parameter(const std::string & node_name, const std::string & param_name) = 0;

  /// Set one parameter; result.data echoes the committed descriptor.
  virtual ParameterResult set_parameter(const std::string & node_name, const std::string & param_name,
                                        const json & value) = 0;

  /// Self-node variants. Read-fast path that does not go via IPC.
  virtual ParameterResult list_own_parameters() = 0;
  virtual ParameterResult get_own_parameter(const std::string & param_name) = 0;

  /// Look up the cached default (initial) value for a single parameter.
  /// On success, `result.data` is the JSON value of the default. The transport
  /// owns the defaults cache; manager-level reset operations compose this with
  /// `set_parameter` to perform the actual reset.
  ///
  /// Error codes follow the same contract as the rest of the surface:
  ///   - NO_DEFAULTS_CACHED: nothing cached for `node_name` yet
  ///   - NOT_FOUND:          the node has cached defaults but not for `param_name`
  ///   - SERVICE_UNAVAILABLE / TIMEOUT / SHUT_DOWN: as per IPC reachability
  virtual ParameterResult get_default(const std::string & node_name, const std::string & param_name) = 0;

  /// List all cached default (initial) values for a node. On success,
  /// `result.data` is a JSON array of `{"name", "value", "type"}` entries.
  /// Used by `reset_all_parameters` to drive a chain of `set_parameter` calls.
  virtual ParameterResult list_defaults(const std::string & node_name) = 0;

  /// Service availability check (same semantics as today's negative cache).
  virtual bool is_node_available(const std::string & node_name) const = 0;

  /// Drop client/cached state for a given node (called by the manager when
  /// reset / refresh is requested).
  virtual void invalidate(const std::string & node_name) = 0;

  /// Tear down all cached clients before rclcpp::shutdown(). Idempotent.
  virtual void shutdown() = 0;
};

}  // namespace ros2_medkit_gateway
