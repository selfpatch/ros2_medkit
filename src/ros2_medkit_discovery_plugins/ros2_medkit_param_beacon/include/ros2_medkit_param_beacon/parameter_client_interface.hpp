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

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/list_parameters_result.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

namespace ros2_medkit_param_beacon {

/// Abstract interface for parameter service client operations.
/// Enables GMock-based testing of ParameterBeaconPlugin without live ROS 2 nodes.
class ParameterClientInterface {
 public:
  virtual ~ParameterClientInterface() = default;

  virtual bool wait_for_service(std::chrono::duration<double> timeout) = 0;

  /// Throws when no answer arrives in time.
  virtual rcl_interfaces::msg::ListParametersResult list_parameters(const std::vector<std::string> & prefixes,
                                                                    uint64_t depth) = 0;

  /// Throws when no answer arrives in time. An answer without values gives an empty vector.
  virtual std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) = 0;
};

/// Production implementation: a client per parameter service, spun on its own executor.
///
/// Every call waits at most `timeout`. A request that gets no answer in time is removed from its client.
class RealParameterClient : public ParameterClientInterface {
 public:
  RealParameterClient(const rclcpp::Node::SharedPtr & node, const std::string & target_node,
                      std::chrono::duration<double> timeout)
    : node_(node->get_node_base_interface())
    , list_client_(create_client<rcl_interfaces::srv::ListParameters>(*node, target_node + "/list_parameters"))
    , get_client_(create_client<rcl_interfaces::srv::GetParameters>(*node, target_node + "/get_parameters"))
    , timeout_(std::chrono::duration_cast<std::chrono::nanoseconds>(timeout)) {
  }

  bool wait_for_service(std::chrono::duration<double> timeout) override {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::nanoseconds>(timeout);
    if (!list_client_->wait_for_service(
            std::max(std::chrono::steady_clock::duration::zero(), deadline - std::chrono::steady_clock::now()))) {
      return false;
    }
    return get_client_->wait_for_service(
        std::max(std::chrono::steady_clock::duration::zero(), deadline - std::chrono::steady_clock::now()));
  }

  rcl_interfaces::msg::ListParametersResult list_parameters(const std::vector<std::string> & prefixes,
                                                            uint64_t depth) override {
    auto request = std::make_shared<rcl_interfaces::srv::ListParameters::Request>();
    request->prefixes = prefixes;
    request->depth = depth;
    return call(*list_client_, request)->result;
  }

  std::vector<rclcpp::Parameter> get_parameters(const std::vector<std::string> & names) override {
    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = names;
    const auto response = call(*get_client_, request);
    std::vector<rclcpp::Parameter> parameters;
    // A service that cannot get one of the names answers with no values.
    if (response->values.size() != names.size()) {
      return parameters;
    }
    parameters.reserve(names.size());
    for (std::size_t i = 0; i < names.size(); ++i) {
      parameters.emplace_back(names[i], rclcpp::ParameterValue(response->values[i]));
    }
    return parameters;
  }

  /// Drops the requests still waiting for an answer and returns how many there were.
  std::size_t prune_pending_requests() {
    return list_client_->prune_pending_requests() + get_client_->prune_pending_requests();
  }

 private:
  template <typename ServiceT>
  static typename rclcpp::Client<ServiceT>::SharedPtr create_client(rclcpp::Node & node, const std::string & name) {
#if RCLCPP_VERSION_MAJOR >= 28
    return node.create_client<ServiceT>(name, rclcpp::ParametersQoS());
#else
    return node.create_client<ServiceT>(name, rmw_qos_profile_parameters);
#endif
  }

  template <typename ServiceT>
  typename ServiceT::Response::SharedPtr call(rclcpp::Client<ServiceT> & client,
                                              const typename ServiceT::Request::SharedPtr & request) {
    auto future = client.async_send_request(request);
    executor_.add_node(node_);
    rclcpp::FutureReturnCode code = rclcpp::FutureReturnCode::INTERRUPTED;
    try {
      code = executor_.spin_until_future_complete(future, timeout_);
    } catch (...) {
      executor_.remove_node(node_);
      client.remove_pending_request(future);
      throw;
    }
    executor_.remove_node(node_);
    if (code != rclcpp::FutureReturnCode::SUCCESS) {
      client.remove_pending_request(future);
      throw std::runtime_error(std::string("No answer from ") + client.get_service_name() + " in time");
    }
    return future.get();
  }

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
  rclcpp::Client<rcl_interfaces::srv::ListParameters>::SharedPtr list_client_;
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_client_;
  std::chrono::nanoseconds timeout_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

/// Factory function type for creating parameter clients.
using ParameterClientFactory =
    std::function<std::shared_ptr<ParameterClientInterface>(const std::string & target_node)>;

}  // namespace ros2_medkit_param_beacon
