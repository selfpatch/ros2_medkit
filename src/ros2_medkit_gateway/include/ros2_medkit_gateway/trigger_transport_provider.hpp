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

#include <nlohmann/json.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Describes a single trigger event to be delivered to a subscriber.
struct TriggerEventDelivery {
  std::string trigger_id;         ///< ID of the trigger that fired
  std::string entity_id;          ///< Entity the trigger is scoped to
  std::string protocol;           ///< Delivery protocol (e.g. "sse", "x-custom")
  nlohmann::json event_envelope;  ///< {timestamp, payload} or {timestamp, error}
};

/// Abstract transport backend for delivering trigger events to subscribers.
///
/// Plugins implement this interface to provide alternative delivery protocols
/// beyond the built-in SSE transport. Register via
/// PluginContext::register_trigger_transport().
///
/// @par Thread Safety
/// deliver_event() may be called from multiple threads concurrently.
/// Implementations must be thread-safe.
class TriggerTransportProvider {
 public:
  virtual ~TriggerTransportProvider() = default;

  /// Return the protocol name this provider handles (e.g. "sse", "x-mqtt").
  /// Used to match against TriggerInfo::protocol during event dispatch.
  virtual std::string protocol_name() const = 0;

  /// Deliver a trigger event to all subscribers of the given trigger.
  /// @param event The event to deliver, including envelope and metadata.
  /// @return void on success, error string on failure.
  virtual tl::expected<void, std::string> deliver_event(const TriggerEventDelivery & event) = 0;
};

}  // namespace ros2_medkit_gateway
