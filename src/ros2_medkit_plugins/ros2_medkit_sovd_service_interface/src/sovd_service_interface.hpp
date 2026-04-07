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
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/plugins/gateway_plugin.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_msgs/msg/entity_info.hpp"
#include "ros2_medkit_msgs/srv/get_capabilities.hpp"
#include "ros2_medkit_msgs/srv/get_entity_data.hpp"
#include "ros2_medkit_msgs/srv/list_entities.hpp"
#include "ros2_medkit_msgs/srv/list_faults_for_entity.hpp"

namespace ros2_medkit_gateway {

/// SOVD Service Interface plugin.
///
/// Exposes medkit entity tree, fault data, and capabilities via ROS 2
/// services. Designed for consumption by VDA 5050 agent, BT.CPP,
/// PlotJuggler, RTMaps, and any other ROS 2 node that needs SOVD data
/// without going through HTTP.
class SovdServiceInterface : public GatewayPlugin {
 public:
  std::string name() const override;
  void configure(const nlohmann::json & config) override;
  void set_context(PluginContext & context) override;
  void shutdown() override;
  ~SovdServiceInterface();

 private:
  void handle_list_entities(const std::shared_ptr<ros2_medkit_msgs::srv::ListEntities::Request> request,
                            std::shared_ptr<ros2_medkit_msgs::srv::ListEntities::Response> response);

  void handle_list_entity_faults(const std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Request> request,
                                 std::shared_ptr<ros2_medkit_msgs::srv::ListFaultsForEntity::Response> response);

  void handle_get_entity_data(const std::shared_ptr<ros2_medkit_msgs::srv::GetEntityData::Request> request,
                              std::shared_ptr<ros2_medkit_msgs::srv::GetEntityData::Response> response);

  void handle_get_capabilities(const std::shared_ptr<ros2_medkit_msgs::srv::GetCapabilities::Request> request,
                               std::shared_ptr<ros2_medkit_msgs::srv::GetCapabilities::Response> response);

  PluginContext * context_{nullptr};
  std::string service_prefix_{"/medkit"};
  std::atomic<bool> shutdown_requested_{false};

  rclcpp::Service<ros2_medkit_msgs::srv::ListEntities>::SharedPtr list_entities_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ListFaultsForEntity>::SharedPtr list_faults_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetEntityData>::SharedPtr get_data_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::GetCapabilities>::SharedPtr get_capabilities_srv_;
};

}  // namespace ros2_medkit_gateway
