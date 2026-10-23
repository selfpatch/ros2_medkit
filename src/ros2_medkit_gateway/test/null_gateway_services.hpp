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

#include "ros2_medkit_gateway/core/gateway_services.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"

/// Support for this package's tests and the tests of packages that extend it.
/// Not part of the gateway's supported API.
namespace ros2_medkit_gateway::test_support {

/// GatewayServices with no service configured and an empty entity cache.
class NullGatewayServices final : public GatewayServices {
 public:
  const ThreadSafeEntityCache & entity_cache() const override {
    return cache_;
  }
  PluginManager * plugin_manager() override {
    return nullptr;
  }
  DiscoveryManager * discovery() override {
    return nullptr;
  }
  ScriptManager * script_manager() override {
    return nullptr;
  }
  FaultManager * fault_manager() override {
    return nullptr;
  }
  LockManager * lock_manager() override {
    return nullptr;
  }
  ConfigurationManager * configuration_manager() override {
    return nullptr;
  }
  DataAccessManager * data_access_manager() override {
    return nullptr;
  }
  OperationManager * operation_manager() override {
    return nullptr;
  }
  UpdateManager * update_manager() override {
    return nullptr;
  }
  LogManager * log_manager() override {
    return nullptr;
  }
  BulkDataStore * bulk_data_store() override {
    return nullptr;
  }
  FaultTriggerEngine * fault_trigger_engine() override {
    return nullptr;
  }
  EntityFreezeFrameCapture * freeze_frames() override {
    return nullptr;
  }
  TopicDataProvider * topic_data_provider() override {
    return nullptr;
  }
  TriggerManager * trigger_manager() override {
    return nullptr;
  }
  SubscriptionManager * subscription_manager() override {
    return nullptr;
  }
  ResourceSamplerRegistry * sampler_registry() override {
    return nullptr;
  }
  TransportRegistry * transport_registry() override {
    return nullptr;
  }
  std::shared_ptr<SSEClientTracker> sse_client_tracker() override {
    return nullptr;
  }
  GatewayNode * ros_node() override {
    return nullptr;
  }

 private:
  ThreadSafeEntityCache cache_;
};

/// Shared instance for fixtures that build a HandlerContext without a node.
inline GatewayServices & null_services() {
  static NullGatewayServices instance;
  return instance;
}

}  // namespace ros2_medkit_gateway::test_support
