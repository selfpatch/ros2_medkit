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

#include <httplib.h>

#include <memory>
#include <string>
#include <unordered_map>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/resource_sampler.hpp"
#include "ros2_medkit_gateway/subscription_manager.hpp"

namespace ros2_medkit_gateway {

class GatewayNode;

/// Virtual interface for pluggable subscription transport protocols.
///
/// SSE is the built-in default (SOVD-compliant). Non-SSE protocols
/// (MQTT, WebSocket, Zenoh) are vendor extensions.
class SubscriptionTransportProvider {
 public:
  virtual ~SubscriptionTransportProvider() = default;

  /// Protocol identifier: "sse" (standard), or vendor extension
  virtual std::string protocol() const = 0;

  /// Start delivery for a subscription. Returns event_source URI for client.
  virtual tl::expected<std::string, std::string> start(const CyclicSubscriptionInfo & info,
                                                       ResourceSamplerFn json_sampler, GatewayNode * node) = 0;

  /// Subscription interval/duration changed. Transport must re-read info.
  virtual void notify_update(const std::string & sub_id) = 0;

  /// Stop delivery. MUST be synchronous - return only when all threads stopped.
  virtual void stop(const std::string & sub_id) = 0;

  /// Handle HTTP client connection (SSE: chunked provider, WebSocket: upgrade).
  /// MQTT/Zenoh: return false (not HTTP-based).
  virtual bool handle_client_connect(const std::string & sub_id, const httplib::Request & req,
                                     httplib::Response & res) {
    (void)sub_id;
    (void)req;
    (void)res;
    return false;
  }
};

/// Registry of transport providers keyed by protocol name.
class TransportRegistry {
 public:
  void register_transport(std::unique_ptr<SubscriptionTransportProvider> provider);
  SubscriptionTransportProvider * get_transport(const std::string & protocol) const;
  bool has_transport(const std::string & protocol) const;

  /// Stop all active subscriptions across all transports.
  void shutdown_all(SubscriptionManager & sub_mgr);

 private:
  std::unordered_map<std::string, std::unique_ptr<SubscriptionTransportProvider>> transports_;
};

}  // namespace ros2_medkit_gateway
