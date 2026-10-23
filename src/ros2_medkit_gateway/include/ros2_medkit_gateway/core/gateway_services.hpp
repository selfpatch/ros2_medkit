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

namespace ros2_medkit_gateway {

class ThreadSafeEntityCache;
class PluginManager;
class EntityDiscovery;
class ScriptManager;
class FaultManager;
class LockManager;
class ConfigurationManager;
class DataAccessManager;
class OperationManager;
class UpdateManager;
class LogManager;
class BulkDataStore;
class FaultTriggerEngine;
class FaultEventSource;
class FreezeFrameSource;
class TopicDataProvider;
class TriggerManager;
class SubscriptionManager;
class ResourceSamplerRegistry;
class TransportRegistry;
class SSEClientTracker;
class TypeSchemaProvider;
class GatewayNode;

/// The gateway services the HTTP layer reaches. Pointers stay valid for the
/// lifetime of the implementer. A getter marked "never null" is dereferenced
/// without a check, so a host must provide it. A getter marked "may be null"
/// is checked at every use.
class GatewayServices {
 public:
  GatewayServices() = default;
  virtual ~GatewayServices() = default;
  GatewayServices(const GatewayServices &) = delete;
  GatewayServices & operator=(const GatewayServices &) = delete;
  GatewayServices(GatewayServices &&) = delete;
  GatewayServices & operator=(GatewayServices &&) = delete;

  // Never null.
  virtual const ThreadSafeEntityCache & entity_cache() const = 0;
  virtual EntityDiscovery * discovery() = 0;
  virtual FaultManager * fault_manager() = 0;
  virtual ConfigurationManager * configuration_manager() = 0;
  virtual DataAccessManager * data_access_manager() = 0;
  virtual OperationManager * operation_manager() = 0;
  virtual SubscriptionManager * subscription_manager() = 0;
  virtual ResourceSamplerRegistry * sampler_registry() = 0;
  virtual TransportRegistry * transport_registry() = 0;
  virtual TypeSchemaProvider * type_schemas() = 0;
  virtual std::shared_ptr<SSEClientTracker> sse_client_tracker() = 0;
  /// Never null while SSEFaultHandler subscribes through it. Otherwise only
  /// handed to subscription transports.
  virtual GatewayNode * ros_node() = 0;

  // May be null.
  virtual PluginManager * plugin_manager() = 0;
  virtual ScriptManager * script_manager() = 0;
  virtual LockManager * lock_manager() = 0;
  virtual UpdateManager * update_manager() = 0;
  virtual LogManager * log_manager() = 0;
  virtual BulkDataStore * bulk_data_store() = 0;
  virtual FaultTriggerEngine * fault_trigger_engine() = 0;
  virtual FreezeFrameSource * freeze_frames() = 0;
  // May be null: a host without a fault manager has no fault stream.
  virtual FaultEventSource * fault_events() = 0;
  virtual TopicDataProvider * topic_data_provider() = 0;
  virtual TriggerManager * trigger_manager() = 0;
};

}  // namespace ros2_medkit_gateway
