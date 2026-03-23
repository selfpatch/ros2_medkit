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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/list_parameters_result.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ros2_medkit_param_beacon {

/// Abstract interface for parameter service client operations.
/// Enables GMock-based testing of ParameterBeaconPlugin without live ROS 2 nodes.
class ParameterClientInterface {
 public:
  virtual ~ParameterClientInterface() = default;

  virtual bool wait_for_service(std::chrono::duration<double> timeout) = 0;

  virtual rcl_interfaces::msg::ListParametersResult list_parameters(const std::vector<std::string> & prefixes,
                                                                    uint64_t depth) = 0;

  virtual std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) = 0;
};

/// Production implementation wrapping rclcpp::SyncParametersClient.
class RealParameterClient : public ParameterClientInterface {
 public:
  RealParameterClient(rclcpp::Node::SharedPtr node, const std::string & target_node)
    : client_(std::make_shared<rclcpp::SyncParametersClient>(node, target_node)) {
  }

  bool wait_for_service(std::chrono::duration<double> timeout) override {
    return client_->wait_for_service(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout));
  }

  rcl_interfaces::msg::ListParametersResult list_parameters(const std::vector<std::string> & prefixes,
                                                            uint64_t depth) override {
    return client_->list_parameters(prefixes, depth);
  }

  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) override {
    return client_->get_parameters(names);
  }

 private:
  std::shared_ptr<rclcpp::SyncParametersClient> client_;
};

/// Factory function type for creating parameter clients.
using ParameterClientFactory =
    std::function<std::shared_ptr<ParameterClientInterface>(const std::string & target_node)>;

}  // namespace ros2_medkit_param_beacon
