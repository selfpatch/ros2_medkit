// Copyright 2026 mfaferek93
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
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <open62541pp/open62541pp.hpp>
#include <tl/expected.hpp>

namespace ros2_medkit_gateway {

/// Variant type for OPC-UA values exposed to the plugin
using OpcuaValue = std::variant<bool, int32_t, int64_t, float, double, std::string>;

/// Result of reading a single OPC-UA node
struct ReadResult {
  std::string node_id;
  OpcuaValue value;
  std::chrono::system_clock::time_point timestamp;
  bool good{true};
};

/// Callback for subscription data changes
using DataChangeCallback = std::function<void(const std::string & node_id, const OpcuaValue & value)>;

/// Configuration for OPC-UA connection
struct OpcuaClientConfig {
  std::string endpoint_url = "opc.tcp://localhost:4840";
  std::chrono::milliseconds connect_timeout{5000};
  std::chrono::milliseconds reconnect_interval{3000};
  // TODO: OPC-UA security (certificates, Basic256Sha256)
};

/// RAII wrapper around open62541pp::Client with auto-reconnect
class OpcuaClient {
 public:
  OpcuaClient();
  ~OpcuaClient();

  OpcuaClient(const OpcuaClient &) = delete;
  OpcuaClient & operator=(const OpcuaClient &) = delete;

  /// Connect to OPC-UA server
  /// @return true if connected successfully
  bool connect(const OpcuaClientConfig & config);

  /// Disconnect and stop reconnect attempts
  void disconnect();

  /// Check if currently connected
  bool is_connected() const;

  /// Get the endpoint URL (for status reporting)
  std::string endpoint_url() const;

  /// Get the current config (for reconnection)
  OpcuaClientConfig current_config() const;

  /// Browse child nodes of a given node
  /// @return Vector of child node ID strings (e.g., "ns=1;s=TankLevel")
  std::vector<std::string> browse(const opcua::NodeId & parent_node);

  /// Read a single value
  ReadResult read_value(const opcua::NodeId & node_id);

  /// Read multiple values
  std::vector<ReadResult> read_values(const std::vector<opcua::NodeId> & node_ids);

  /// OPC-UA write error classification
  enum class WriteError { NotConnected, TypeMismatch, AccessDenied, NodeNotFound, TransportError };

  /// Detailed write error info
  struct WriteErrorInfo {
    WriteError code;
    std::string message;
  };

  /// Write a value to a node
  /// @param data_type_hint If non-empty, skip readValue type-probe and use this hint
  ///        ("bool", "int", "float", "string") to select the write coercion directly.
  ///        Halves mutex hold time by eliminating a round-trip to the server.
  /// @return void on success, WriteErrorInfo on failure with specific error code
  tl::expected<void, WriteErrorInfo> write_value(const opcua::NodeId & node_id, const OpcuaValue & value,
                                                 const std::string & data_type_hint = "");

  /// Create a subscription with data change notifications
  /// @return Subscription ID, or 0 on failure
  uint32_t create_subscription(double publish_interval_ms, DataChangeCallback callback);

  /// Add a monitored item to a subscription
  /// @return true if item was added
  bool add_monitored_item(uint32_t subscription_id, const opcua::NodeId & node_id);

  /// Remove all subscriptions
  void remove_subscriptions();

  /// One element of an OPC-UA SimpleAttributeOperand browse path.
  struct EventField {
    uint16_t namespace_index{0};
    std::string name;
  };

  /// Browse path for an event field, e.g. ``{{0, "EnabledState"}, {0, "Id"}}``.
  using EventBrowsePath = std::vector<EventField>;

  /// Full SimpleAttributeOperand spec - every clause in an EventFilter must
  /// have ``typeDefinitionId`` set to the type that *directly* defines the
  /// browse path's first segment (open62541 servers reject inherited
  /// lookups with BadNodeIdUnknown). ConditionId is the documented edge
  /// case (Part 9 §5.5.2.13): empty browse path + AttributeId=NodeId.
  struct EventFieldSpec {
    opcua::NodeId type_definition_id;
    EventBrowsePath browse_path;
    uint32_t attribute_id{13};  // UA_ATTRIBUTEID_VALUE
  };

  /// Callback invoked when an OPC-UA event arrives on a monitored item.
  /// @param select_values Values for caller-requested fields, in the order of
  ///        ``select_specs`` passed to ``add_event_monitored_item``.
  /// @param source_node Always-included SourceNode (extracted from the event
  ///        payload; null NodeId if the server omitted it).
  /// @param event_type Always-included EventType (null NodeId if absent).
  /// @param condition_id NodeId of the condition instance that emitted the
  ///        event (Part 9 §5.5.2.13). Null NodeId for non-condition events.
  using EventCallback =
      std::function<void(const std::vector<opcua::Variant> & select_values, const opcua::NodeId & source_node,
                         const opcua::NodeId & event_type, const opcua::NodeId & condition_id)>;

  /// Get the current subscription generation. Increments on every detected
  /// disconnect (clean ``disconnect()`` or transport-level drop). Used by the
  /// internal event trampoline to drop callbacks fired from defunct
  /// subscriptions.
  uint64_t current_generation() const;

  /// Run a single iteration of the open62541 client main loop. Required to
  /// dispatch incoming subscription notifications (events, data changes)
  /// to their callbacks. The poller calls this every iteration to keep
  /// AlarmCondition events flowing.
  void run_iterate(uint16_t timeout_ms = 100);

  /// Add an event-based monitored item to an existing subscription.
  ///
  /// Wraps ``UA_Client_MonitoredItems_createEvent`` from the open62541 C API
  /// because ``open62541pp`` v0.16 has no native EventFilter / event
  /// subscription support. ``EventType``, ``SourceNode`` and a ConditionId
  /// SAO (empty BrowsePath, AttributeId=NodeId) are always prepended; they
  /// are extracted from the event payload and delivered as separate callback
  /// parameters, not in ``select_values``.
  ///
  /// @return Server-assigned monitored item ID, or 0 on failure.
  uint32_t add_event_monitored_item(uint32_t subscription_id, const opcua::NodeId & source_node,
                                    const std::vector<EventFieldSpec> & select_specs, EventCallback callback);

  /// Remove a previously-added event monitored item. The server is asked to
  /// delete the item synchronously; the callback context is freed only after
  /// the server ACK so in-flight C callbacks cannot dangle.
  /// @return true if the item was found and removed cleanly.
  bool remove_event_monitored_item(uint32_t subscription_id, uint32_t mi_id);

  /// OPC-UA Method call error classification.
  enum class MethodError { NotConnected, MethodNotFound, InvalidArgument, MethodTimeout, TransportError };

  /// Detailed Method call error info.
  struct MethodErrorInfo {
    MethodError code;
    std::string message;
  };

  /// Synchronously call an OPC-UA Method on a target object.
  /// Used by ConditionRefresh, Acknowledge, and Confirm operations on
  /// AlarmConditionType nodes (issue #386).
  /// @return Output arguments on success, MethodErrorInfo on failure.
  tl::expected<std::vector<opcua::Variant>, MethodErrorInfo>
  call_method(const opcua::NodeId & object_id, const opcua::NodeId & method_id,
              const std::vector<opcua::Variant> & input_args);

  /// Map an OPC-UA StatusCode (from an attempted method call or a
  /// per-argument validation result) to a ``MethodError`` category.
  /// Exposed as a public static helper so the classification table is
  /// covered by unit tests without needing a live OPC-UA connection.
  static MethodErrorInfo status_to_method_error(uint32_t code, const std::string & message);

  /// Classify the full result of a Call service exchange per OPC-UA
  /// Part 4 §5.11.2: overall ``statusCode`` covers transport / method
  /// resolution; ``inputArgumentResults`` covers per-argument validation.
  /// Returns success when both are Good. The first non-Good code wins:
  /// overall statusCode takes precedence, then arg_results in order.
  /// AlarmConditionType.Acknowledge surfaces ``BadEventIdUnknown`` in
  /// ``arg_results[0]`` when the EventId we cached has been superseded.
  /// Exposed as a static for unit-test coverage of the per-arg branch.
  static tl::expected<void, MethodErrorInfo> classify_call_result(uint32_t overall_status_code,
                                                                  const std::vector<uint32_t> & arg_results);

  /// Get server description string (for status endpoint)
  std::string server_description() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_medkit_gateway
