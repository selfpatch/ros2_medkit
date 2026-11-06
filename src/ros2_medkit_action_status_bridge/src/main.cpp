// Copyright 2026 mfaferek93, bburda
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
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_action_status_bridge/action_status_bridge_node.hpp"

int main(int argc, char * argv[]) {
  // SIGINT and SIGTERM stay blocked until the node is built: rclcpp does not support a context
  // shut down under a node's constructor. Threads started meanwhile inherit the mask.
  sigset_t shutdown_signals;
  sigset_t previous_mask;
  sigemptyset(&shutdown_signals);
  sigaddset(&shutdown_signals, SIGINT);
  sigaddset(&shutdown_signals, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &shutdown_signals, &previous_mask) != 0) {
    fprintf(stderr, "[action_status_bridge] Fatal: could not block SIGINT and SIGTERM\n");
    return 1;
  }

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto node = std::make_shared<ros2_medkit_action_status_bridge::ActionStatusBridgeNode>();

  // A signal sent during startup is delivered at the unblock and shuts the context down, so spin
  // returns at once. init() with None already counts as installed.
  rclcpp::uninstall_signal_handlers();
  if (!rclcpp::install_signal_handlers() || pthread_sigmask(SIG_SETMASK, &previous_mask, nullptr) != 0) {
    fprintf(stderr, "[action_status_bridge] Fatal: could not install the SIGINT and SIGTERM handlers\n");
    node.reset();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
