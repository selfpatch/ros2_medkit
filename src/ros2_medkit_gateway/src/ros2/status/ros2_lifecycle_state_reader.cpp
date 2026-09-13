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

#include "ros2_medkit_gateway/ros2_common/callback_groups.hpp"

#include <algorithm>
#include <exception>

#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/version.h>

namespace ros2_medkit_gateway {

namespace {

/// create_client() with an explicit callback group, spelled the way each
/// supported distro wants it. Jazzy is rclcpp 28 and takes rclcpp::QoS; older
/// distros only offer the rmw_qos_profile_t form, where it is not deprecated.
rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr
create_get_state_client(rclcpp::Node * node, const std::string & service_name,
                        const rclcpp::CallbackGroup::SharedPtr & group) {
#if defined(RCLCPP_VERSION_MAJOR) && RCLCPP_VERSION_MAJOR >= 28
  return node->create_client<lifecycle_msgs::srv::GetState>(service_name, rclcpp::ServicesQoS(), group);
#else
  return node->create_client<lifecycle_msgs::srv::GetState>(service_name, rmw_qos_profile_services_default, group);
#endif
}

}  // namespace

Ros2LifecycleStateReader::Ros2LifecycleStateReader(rclcpp::Node * host, std::chrono::duration<double> timeout)
  : timeout_(timeout) {
  client_node_ = std::make_shared<rclcpp::Node>(std::string(host->get_name()) + "_lifecycle_state_reader");
  // Registered with the context's GraphListener here, while the context is
  // known valid. Left to the first graph use, NodeGraph::get_graph_event()
  // would spend should_add_to_graph_listener_ before add_node() throws
  // GraphListenerShutdownError on a stopped listener; the node is then marked
  // as registered while absent from the listener's list, and ~NodeGraph turns
  // that into a NodeNotFoundError thrown out of a noexcept destructor. The
  // window is narrowed, not closed: a shutdown between the make_shared above
  // and this line spends the flag the same way, and rclcpp offers no way to
  // un-spend it.
  (void)client_node_->get_graph_event();
}

Ros2LifecycleStateReader::~Ros2LifecycleStateReader() {
  // A call spends most of its life outside mutex_, so holding the mutex here
  // would prove nothing about the node being idle. Refuse new calls, then wait
  // for the ones already running: each of them owns an executor that holds a
  // reference to client_node_, and that executor is destroyed before the call
  // decrements in_flight_.
  std::unique_lock<std::mutex> lock(mutex_);
  stopping_ = true;
  idle_cv_.wait(lock, [this] {
    return in_flight_ == 0;
  });
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

  // Making the call's callback group and its client mutates registries on the
  // shared node that rclcpp does not serialize, and so does destroying them.
  // Those four moments are all mutex_ covers. Everything that can block - the
  // service wait, the request, the spin - happens outside it, because one
  // instance serves every caller in the process and an unanswering target must
  // not hold up a caller asking about a different node.
  rclcpp::CallbackGroup::SharedPtr group;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr client;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (stopping_) {
      return std::nullopt;
    }
    try {
      // An isolated group keeps this client out of any executor the private
      // node is added to, so the only thread that ever holds a reference to it
      // is the one running this call - which is what lets the client be
      // destroyed here rather than on an executor thread.
      group = ros2_common::create_isolated_callback_group(*client_node_);
      client = create_get_state_client(client_node_.get(), get_state_service_path, group);
    } catch (const std::exception & e) {
      RCLCPP_WARN(client_node_->get_logger(), "GetState client creation failed for '%s': %s",
                  get_state_service_path.c_str(), e.what());
      return std::nullopt;
    }
    ++in_flight_;
  }

  std::optional<std::string> label;
  {
    // This call's own executor, spun inline on this thread and destroyed on it.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_callback_group(group, client_node_->get_node_base_interface());
    if (client->wait_for_service(clamped)) {
      auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
      auto future = client->async_send_request(request);
      if (executor.spin_until_future_complete(future, clamped) == rclcpp::FutureReturnCode::SUCCESS) {
        label = future.get()->current_state.label;
      } else {
        // Drop the abandoned slot from the client's pending-request map.
        client->remove_pending_request(future.request_id);
      }
    }
    executor.remove_callback_group(group);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    client.reset();
    group.reset();
    --in_flight_;
  }
  idle_cv_.notify_all();
  return label;
}

}  // namespace ros2_medkit_gateway
