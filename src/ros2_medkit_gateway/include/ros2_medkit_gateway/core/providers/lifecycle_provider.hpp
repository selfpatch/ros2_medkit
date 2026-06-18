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
#include <string_view>
#include <variant>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/dto/lifecycle.hpp"

namespace ros2_medkit_gateway {

enum class LifecycleProviderError {
  EntityNotFound,
  Unsupported,
  PreconditionFailed,
  AccessDenied,
  TransportError,
  Internal
};

struct LifecycleProviderErrorInfo {
  LifecycleProviderError code;
  std::string message;
  int http_status{500};
};

/// Per-entity provider for SOVD status/lifecycle control. A substrate-owning
/// plugin (ROS lifecycle, process/container, host) implements this; the
/// lifecycle handler routes to it. Mirrors OperationProvider's typed-DTO +
/// per-provider-error-struct style.
class LifecycleProvider {
 public:
  virtual ~LifecycleProvider() = default;

  virtual tl::expected<dto::LifecycleStatusResponse, LifecycleProviderErrorInfo>
  get_status(const std::string & entity_id) = 0;

  /// Initiate a transition ("start"|"restart"|"force-restart"|"shutdown"|
  /// "force-shutdown"). Async: returns on accept; the client polls GET status.
  virtual tl::expected<std::monostate, LifecycleProviderErrorInfo> request_transition(const std::string & entity_id,
                                                                                      std::string_view transition) = 0;
};

}  // namespace ros2_medkit_gateway
