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

#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"
#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <ros2_medkit_gateway/discovery/introspection_provider.hpp>
#include <ros2_medkit_gateway/plugins/gateway_plugin.hpp>
#include <ros2_medkit_gateway/plugins/plugin_context.hpp>
#include <ros2_medkit_gateway/plugins/plugin_http_types.hpp>
#include <ros2_medkit_gateway/providers/data_provider.hpp>
#include <ros2_medkit_gateway/providers/fault_provider.hpp>
#include <ros2_medkit_gateway/providers/operation_provider.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

namespace ros2_medkit_gateway {

/// OPC-UA Gateway Plugin - bridges OPC-UA PLCs into the SOVD entity tree
///
/// Implements GatewayPlugin (lifecycle, routes), IntrospectionProvider
/// (entity discovery from OPC-UA address space), and the typed provider
/// interfaces (DataProvider, OperationProvider, FaultProvider) so that
/// standard SOVD endpoints (/data, /operations, /faults) work for PLC
/// entities alongside the vendor x-plc-* extensions.
///
/// Standard SOVD endpoints (via provider interfaces):
///   GET  /{type}/{id}/data                    - DataProvider::list_data
///   GET  /{type}/{id}/data/{name}             - DataProvider::read_data
///   PUT  /{type}/{id}/data/{name}             - DataProvider::write_data
///   GET  /{type}/{id}/operations              - OperationProvider::list_operations
///   POST /{type}/{id}/operations/{name}       - OperationProvider::execute_operation
///   GET  /{type}/{id}/faults                  - FaultProvider::list_faults
///   GET  /{type}/{id}/faults/{code}           - FaultProvider::get_fault
///   DELETE /{type}/{id}/faults/{code}         - FaultProvider::clear_fault
///
/// Vendor REST extensions (via get_routes):
///   GET  /apps/{id}/x-plc-data          - All OPC-UA values for entity
///   GET  /apps/{id}/x-plc-data/{node}   - Single node value
///   POST /apps/{id}/x-plc-operations/{op} - Write value to PLC
///   GET  /components/{id}/x-plc-status  - Connection state and stats
class OpcuaPlugin : public ros2_medkit_gateway::GatewayPlugin,
                    public ros2_medkit_gateway::IntrospectionProvider,
                    public ros2_medkit_gateway::DataProvider,
                    public ros2_medkit_gateway::OperationProvider,
                    public ros2_medkit_gateway::FaultProvider {
 public:
  OpcuaPlugin();
  ~OpcuaPlugin() override;

  // -- GatewayPlugin interface --
  std::string name() const override {
    return "opcua";
  }
  void configure(const nlohmann::json & config) override;
  void set_context(ros2_medkit_gateway::PluginContext & context) override;
  std::vector<PluginRoute> get_routes() override;
  void shutdown() override;

  // -- IntrospectionProvider interface --
  ros2_medkit_gateway::IntrospectionResult introspect(const ros2_medkit_gateway::IntrospectionInput & input) override;

  // -- DataProvider interface --
  tl::expected<nlohmann::json, DataProviderErrorInfo> list_data(const std::string & entity_id) override;
  tl::expected<nlohmann::json, DataProviderErrorInfo> read_data(const std::string & entity_id,
                                                                const std::string & resource_name) override;
  tl::expected<nlohmann::json, DataProviderErrorInfo>
  write_data(const std::string & entity_id, const std::string & resource_name, const nlohmann::json & value) override;

  // -- OperationProvider interface --
  tl::expected<nlohmann::json, OperationProviderErrorInfo> list_operations(const std::string & entity_id) override;
  tl::expected<nlohmann::json, OperationProviderErrorInfo>
  execute_operation(const std::string & entity_id, const std::string & operation_name,
                    const nlohmann::json & parameters) override;

  // -- FaultProvider interface --
  tl::expected<nlohmann::json, FaultProviderErrorInfo> list_faults(const std::string & entity_id) override;
  tl::expected<nlohmann::json, FaultProviderErrorInfo> get_fault(const std::string & entity_id,
                                                                 const std::string & fault_code) override;
  tl::expected<nlohmann::json, FaultProviderErrorInfo> clear_fault(const std::string & entity_id,
                                                                   const std::string & fault_code) override;

 private:
  // Route handlers
  void handle_plc_data(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);
  void handle_plc_data_single(const ros2_medkit_gateway::PluginRequest & req,
                              ros2_medkit_gateway::PluginResponse & res);
  void handle_plc_operations(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);
  void handle_plc_status(const ros2_medkit_gateway::PluginRequest & req, ros2_medkit_gateway::PluginResponse & res);

  // Alarm -> Fault bridge
  void on_alarm_change(const std::string & fault_code, const AlarmConfig & config, bool active);

  // Report/clear fault via ROS 2 service (private helpers, not the FaultProvider overrides)
  void send_report_fault(const std::string & entity_id, const std::string & fault_code,
                         const std::string & severity_str, const std::string & message);
  void send_clear_fault(const std::string & fault_code);

  // Publish PLC values to ROS 2 topics (called after each poll)
  void publish_values(const PollSnapshot & snap);

  // Build JSON response for data endpoint
  nlohmann::json build_data_response(const std::string & entity_id) const;

  std::atomic<bool> shutdown_requested_{false};
  ros2_medkit_gateway::PluginContext * ctx_{nullptr};
  OpcuaClientConfig client_config_;
  PollerConfig poller_config_;
  std::string node_map_path_;

  std::unique_ptr<OpcuaClient> client_;
  NodeMap node_map_;

  // ROS 2 service clients for fault reporting
  struct FaultClients;
  std::unique_ptr<FaultClients> fault_clients_;

  // Tracks which non-numeric nodes have already been warned about (avoids log spam).
  // Instance member instead of static to survive plugin reload (dlclose/dlopen).
  std::unordered_set<std::string> warned_non_numeric_;

  // ROS 2 publishers for PLC value bridging (node_id_str -> publisher).
  // Declared before poller_ so that C++ reverse-destruction-order guarantees
  // the poller thread is joined before publishers are destroyed.
  std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> publishers_;

  // Must be declared last among resources used by the poll thread.
  // ~OpcuaPoller() calls stop() which joins the thread.
  std::unique_ptr<OpcuaPoller> poller_;
};

}  // namespace ros2_medkit_gateway
