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

#include "ros2_medkit_gateway/ros2/status/ros2_lifecycle_state_reader.hpp"

#include <algorithm>

#include <lifecycle_msgs/srv/get_state.hpp>

namespace ros2_medkit_gateway {

Ros2LifecycleStateReader::Ros2LifecycleStateReader(rclcpp::Node * host, std::chrono::duration<double> timeout)
  : timeout_(timeout) {
  client_node_ = std::make_shared<rclcpp::Node>(std::string(host->get_name()) + "_lifecycle_state_reader");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);
}

Ros2LifecycleStateReader::~Ros2LifecycleStateReader() {
  if (executor_ && client_node_) {
    executor_->remove_node(client_node_);
  }
}

std::optional<std::string> Ros2LifecycleStateReader::get_state(const std::string & get_state_service_path) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto clamped = std::chrono::duration<double>(std::max(timeout_.count(), 0.0));

  auto client = client_node_->create_client<lifecycle_msgs::srv::GetState>(get_state_service_path);
  if (!client->wait_for_service(clamped)) {
    return std::nullopt;
  }
  auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
  auto future = client->async_send_request(request);
  if (executor_->spin_until_future_complete(future, clamped) != rclcpp::FutureReturnCode::SUCCESS) {
    client->remove_pending_request(future.request_id);
    return std::nullopt;
  }
  return future.get()->current_state.label;
}

}  // namespace ros2_medkit_gateway
