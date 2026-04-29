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
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <shared_mutex>
#include <string>

#include "ros2_medkit_gateway/compat/generic_client_compat.hpp"
#include "ros2_medkit_gateway/core/transports/service_transport.hpp"
#include "ros2_medkit_serialization/json_serializer.hpp"

namespace ros2_medkit_gateway::ros2 {

/**
 * @brief rclcpp adapter implementing ServiceTransport.
 *
 * Owns the GenericServiceClient cache and the JsonSerializer that
 * OperationManager previously held directly. Service-call body lifted
 * verbatim from the legacy OperationManager::call_service.
 */
class Ros2ServiceTransport : public ServiceTransport {
 public:
  /**
   * @param node Non-owning ROS node used for client creation.
   */
  explicit Ros2ServiceTransport(rclcpp::Node * node);

  ~Ros2ServiceTransport() override;

  Ros2ServiceTransport(const Ros2ServiceTransport &) = delete;
  Ros2ServiceTransport & operator=(const Ros2ServiceTransport &) = delete;
  Ros2ServiceTransport(Ros2ServiceTransport &&) = delete;
  Ros2ServiceTransport & operator=(Ros2ServiceTransport &&) = delete;

  ServiceCallResult call(const std::string & service_path, const std::string & service_type, const json & request,
                         std::chrono::duration<double> timeout) override;

 private:
  static std::string make_client_key(const std::string & service_path, const std::string & service_type);

  compat::GenericServiceClient::SharedPtr get_or_create_client(const std::string & service_path,
                                                               const std::string & service_type);

  rclcpp::Node * node_;
  std::shared_ptr<ros2_medkit_serialization::JsonSerializer> serializer_;

  mutable std::shared_mutex clients_mutex_;
  std::map<std::string, compat::GenericServiceClient::SharedPtr> clients_;
};

}  // namespace ros2_medkit_gateway::ros2
