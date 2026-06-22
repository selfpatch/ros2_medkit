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
#include <exception>

#include <lifecycle_msgs/srv/get_state.hpp>

namespace ros2_medkit_gateway {

Ros2LifecycleStateReader::Ros2LifecycleStateReader(rclcpp::Node * host, std::chrono::duration<double> timeout)
  : timeout_(timeout) {
  client_node_ = std::make_shared<rclcpp::Node>(std::string(host->get_name()) + "_lifecycle_state_reader");
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(client_node_);
}

Ros2LifecycleStateReader::~Ros2LifecycleStateReader() {
  // Tear down under the mutex so that an in-flight get_state() (which holds the
  // mutex across async_send_request + spin) has finished first: remove_node
  // during an active spin is undefined behavior. Mirrors the executor-mutex
  // teardown in ros2_fault_service_transport.cpp.
  std::lock_guard<std::mutex> lock(mutex_);
  if (executor_ && client_node_) {
    executor_->remove_node(client_node_);
  }
}

std::optional<std::string> Ros2LifecycleStateReader::get_state(const std::string & get_state_service_path) {
  // A malformed or empty cached service path makes create_client throw
  // (rclcpp::exceptions::InvalidServiceNameError). The default (no-provider)
  // status branch in the handler does not wrap this call in try/catch, so
  // degrade to "no reading" here instead of letting it escape onto the HTTP
  // handler thread.
  if (get_state_service_path.empty()) {
    return std::nullopt;
  }
  const auto clamped = std::chrono::duration<double>(std::max(timeout_.count(), 0.0));

  // create_client (and ~Client at the tail) mutate client_node_'s registry, so
  // they are serialized by mutex_. wait_for_service is backed by an independent
  // graph listener and does not touch the executor, so it runs OUTSIDE the
  // mutex: otherwise a slow or unreachable lifecycle node would hold the mutex
  // for the whole timeout and head-of-line-block every other concurrent
  // /status read (ros2_fault_service_transport.cpp keeps wait_for_service
  // outside its executor mutex for the same reason).
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      client = client_node_->create_client<lifecycle_msgs::srv::GetState>(get_state_service_path);
    } catch (const std::exception & e) {
      RCLCPP_WARN(client_node_->get_logger(), "GetState client creation failed for '%s': %s",
                  get_state_service_path.c_str(), e.what());
      return std::nullopt;
    }
  }

  std::optional<std::string> label;
  if (client->wait_for_service(clamped)) {
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    std::lock_guard<std::mutex> lock(mutex_);
    auto future = client->async_send_request(request);
    if (executor_->spin_until_future_complete(future, clamped) == rclcpp::FutureReturnCode::SUCCESS) {
      label = future.get()->current_state.label;
    } else {
      client->remove_pending_request(future.request_id);
    }
  }

  // Destroy the client under the mutex: like create_client, ~Client mutates
  // client_node_'s registry and would race a concurrent create_client.
  std::lock_guard<std::mutex> lock(mutex_);
  client.reset();
  return label;
}

}  // namespace ros2_medkit_gateway
