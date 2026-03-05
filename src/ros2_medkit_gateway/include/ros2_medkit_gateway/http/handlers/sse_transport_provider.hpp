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

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "ros2_medkit_gateway/http/sse_client_tracker.hpp"
#include "ros2_medkit_gateway/subscription_transport.hpp"

namespace ros2_medkit_gateway {

/// Built-in SSE transport provider. Refactored from handle_events() logic.
class SseTransportProvider : public SubscriptionTransportProvider {
 public:
  SseTransportProvider(SubscriptionManager & sub_mgr, std::shared_ptr<SSEClientTracker> client_tracker);

  std::string protocol() const override {
    return "sse";
  }

  tl::expected<std::string, std::string> start(const CyclicSubscriptionInfo & info, ResourceSamplerFn json_sampler,
                                               GatewayNode * node) override;

  void notify_update(const std::string & sub_id) override;
  void stop(const std::string & sub_id) override;

  bool handle_client_connect(const std::string & sub_id, const httplib::Request & req,
                             httplib::Response & res) override;

 private:
  struct StreamState {
    ResourceSamplerFn sampler;
  };

  SubscriptionManager & sub_mgr_;
  std::shared_ptr<SSEClientTracker> client_tracker_;
  std::mutex mutex_;
  std::unordered_map<std::string, StreamState> streams_;

  static constexpr int kKeepaliveIntervalSec = 15;
};

}  // namespace ros2_medkit_gateway
