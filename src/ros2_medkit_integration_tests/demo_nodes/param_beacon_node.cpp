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

#include <csignal>

#include <rclcpp/rclcpp.hpp>

/// Demo node that declares ros2_medkit.discovery.* parameters.
/// Used by integration tests for the ParameterBeaconPlugin.
/// The gateway polls these parameters to build BeaconHints.
class ParamBeaconNode : public rclcpp::Node {
 public:
  ParamBeaconNode() : Node("param_beacon_node") {
    // Declare beacon discovery parameters - the gateway polls these
    declare_parameter("ros2_medkit.discovery.entity_id", "");
    declare_parameter("ros2_medkit.discovery.transport_type", "");
    declare_parameter("ros2_medkit.discovery.process_name", "");
    declare_parameter("ros2_medkit.discovery.hostname", "");
    declare_parameter("ros2_medkit.discovery.process_id", 0);
    declare_parameter("ros2_medkit.discovery.stable_id", "");

    auto entity_id = get_parameter("ros2_medkit.discovery.entity_id").as_string();
    if (entity_id.empty()) {
      RCLCPP_WARN(get_logger(), "No entity_id set - beacon parameters will be ignored by plugin");
    } else {
      RCLCPP_INFO(get_logger(), "ParamBeaconNode: entity_id='%s'", entity_id.c_str());
    }
  }
};

int main(int argc, char * argv[]) {
  // Block SIGINT/SIGTERM until the executor has allocated its guard
  // condition; a signal arriving mid-init invalidates the rcl context and
  // causes rcl_* calls to throw RCLError. Unblocking after add_node() lets
  // any queued signal be handled as a normal shutdown.
  sigset_t mask, old;
  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &mask, &old);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParamBeaconNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  pthread_sigmask(SIG_SETMASK, &old, nullptr);

  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
