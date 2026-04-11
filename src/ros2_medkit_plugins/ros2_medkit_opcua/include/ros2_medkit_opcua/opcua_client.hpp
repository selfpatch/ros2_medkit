// Copyright 2026 mfaferek
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

  /// Write a value to a node
  /// @return true if write succeeded
  bool write_value(const opcua::NodeId & node_id, const OpcuaValue & value);

  /// Create a subscription with data change notifications
  /// @return Subscription ID, or 0 on failure
  uint32_t create_subscription(double publish_interval_ms, DataChangeCallback callback);

  /// Add a monitored item to a subscription
  /// @return true if item was added
  bool add_monitored_item(uint32_t subscription_id, const opcua::NodeId & node_id);

  /// Remove all subscriptions
  void remove_subscriptions();

  /// Get server description string (for status endpoint)
  std::string server_description() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace ros2_medkit_gateway
