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

#include "ros2_medkit_gateway/subscription_transport.hpp"

#include <stdexcept>

namespace ros2_medkit_gateway {

void TransportRegistry::register_transport(std::unique_ptr<SubscriptionTransportProvider> provider) {
  std::unique_lock<std::shared_mutex> lock(mutex_);
  auto proto = provider->protocol();
  if (transports_.count(proto) > 0) {
    throw std::runtime_error("Transport already registered for protocol: " + proto);
  }
  transports_[proto] = std::move(provider);
}

SubscriptionTransportProvider * TransportRegistry::get_transport(const std::string & protocol) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  auto it = transports_.find(protocol);
  if (it == transports_.end()) {
    return nullptr;
  }
  return it->second.get();
}

bool TransportRegistry::has_transport(const std::string & protocol) const {
  std::shared_lock<std::shared_mutex> lock(mutex_);
  return transports_.count(protocol) > 0;
}

void TransportRegistry::shutdown_all(SubscriptionManager & sub_mgr) {
  // Delegate to sub_mgr.shutdown() which triggers on_removed callbacks
  // for each active subscription, causing transport->stop() for each.
  sub_mgr.shutdown();
}

}  // namespace ros2_medkit_gateway
