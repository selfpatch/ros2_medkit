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
#include <shared_mutex>
#include <string>
#include <unordered_map>

#include <tl/expected.hpp>

#include "ros2_medkit_gateway/core/managers/subscription_manager.hpp"
#include "ros2_medkit_gateway/core/models/error_info.hpp"
#include "ros2_medkit_gateway/core/resource_sampler.hpp"
#include "ros2_medkit_gateway/http/response_types.hpp"

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

  /// Stop delivery for a subscription.
  ///
  /// For HTTP-based transports (SSE), the HTTP server thread join handles
  /// synchronous stop - stop() removes state so the streaming loop exits
  /// on its next is_active() check. GatewayNode::~GatewayNode() calls
  /// stop_rest_server() before shutdown_all() to ensure this ordering.
  ///
  /// Non-HTTP transports MUST ensure all delivery threads have stopped
  /// before returning from this method.
  virtual void stop(const std::string & sub_id) = 0;

  /// Open a new client stream for the given subscription. HTTP-based transports
  /// (SSE) build an `http::SseStream` whose `next_event` callback the typed
  /// router drives via cpp-httplib's chunked content provider. Non-HTTP
  /// transports (MQTT, WebSocket, Zenoh) return an `ErrorInfo` so the typed
  /// handler renders a SOVD GenericError.
  ///
  /// On failure (unknown sub_id, client-limit exhausted, ...) the
  /// implementation returns a fully-formed `ErrorInfo` whose `http_status`
  /// drives the wire status code. On success the returned `SseStream`'s
  /// lifetime owns the transport-side client-tracker handle (e.g. via a
  /// shared_ptr deleter on a captured tracker token) so the slot is released
  /// when the framework destroys the stream - either on client disconnect or
  /// end-of-stream.
  virtual tl::expected<http::SseStream, ErrorInfo> make_sse_stream(const std::string & sub_id) {
    (void)sub_id;
    ErrorInfo err;
    err.code = "transport-not-streamable";
    err.message = "Transport does not produce HTTP streams";
    err.http_status = 501;
    return tl::make_unexpected(std::move(err));
  }
};

/// Registry of transport providers keyed by protocol name.
class TransportRegistry {
 public:
  void register_transport(std::unique_ptr<SubscriptionTransportProvider> provider);
  SubscriptionTransportProvider * get_transport(const std::string & protocol) const;
  bool has_transport(const std::string & protocol) const;

  /// Shutdown all subscriptions by delegating to SubscriptionManager::shutdown().
  /// Transport cleanup happens via the manager's on_removed callback, which
  /// calls SubscriptionTransportProvider::stop() for each active subscription.
  void shutdown_all(SubscriptionManager & sub_mgr);

 private:
  mutable std::shared_mutex mutex_;
  std::unordered_map<std::string, std::unique_ptr<SubscriptionTransportProvider>> transports_;
};

}  // namespace ros2_medkit_gateway
