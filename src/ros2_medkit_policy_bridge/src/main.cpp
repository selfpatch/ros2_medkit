// Copyright 2026 ahmedsleem109
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

#include <pthread.h>

#include <csignal>
#include <cstdio>
#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_policy_bridge/policy_bridge_node.hpp"

int main(int argc, char * argv[]) {
  // SIGINT and SIGTERM stay blocked until the node and its executor are built: rclcpp does not
  // support a context shut down while either is constructed. Threads started meanwhile inherit
  // the mask.
  sigset_t shutdown_signals;
  sigset_t previous_mask;
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &shutdown_signals, &previous_mask) != 0) {
    fprintf(stderr, "[policy_bridge] Fatal: could not block SIGINT and SIGTERM\n");
    return 1;
  }

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  std::shared_ptr<ros2_medkit_policy_bridge::PolicyBridgeNode> node;
  try {
    node = std::make_shared<ros2_medkit_policy_bridge::PolicyBridgeNode>();
  } catch (const std::exception & e) {
    // A setting nothing can fall back from, such as an invalid introspection_topic.
    RCLCPP_FATAL(rclcpp::get_logger("policy_bridge"), "Cannot start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  // One thread: the node's callbacks share state without a lock (policy_bridge_node.hpp).
  auto executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);

  // A signal sent during startup is delivered at the unblock and shuts the context down, so spin
  // returns at once. init() with None already counts as installed.
  rclcpp::uninstall_signal_handlers();
  const bool handling_signals =
      rclcpp::install_signal_handlers() && pthread_sigmask(SIG_SETMASK, &previous_mask, nullptr) == 0;
  int code = 0;
  if (handling_signals) {
    try {
      executor->spin();
    } catch (const std::exception & e) {
      // An rcl call that a shutdown made fail on its way out is not an error.
      if (rclcpp::ok()) {
        RCLCPP_FATAL(node->get_logger(), "Stopped: %s", e.what());
        code = 1;
      }
    }
  } else {
    fprintf(stderr, "[policy_bridge] Fatal: could not install the SIGINT and SIGTERM handlers\n");
    code = 1;
  }

  // The executor goes before the node it holds.
  executor->remove_node(node);
  executor.reset();
  node.reset();
  rclcpp::shutdown();
  return code;
}
