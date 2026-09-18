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

/**
 * @file endpointless_node.cpp
 * @brief A running node with no endpoints of its own
 *
 * Test fixture: a bare rcl node with /rosout off, because an rclcpp::Node always
 * subscribes to /parameter_events. Set the name with `-r __node:=... -r __ns:=...`.
 * Runs until SIGINT or SIGTERM.
 */

#include <csignal>
#include <cstdio>

#include <rcl/error_handling.h>
#include <rcl/node.h>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_integration_tests/crash_backtrace.hpp"

int main(int argc, char ** argv) {
  ros2_medkit_integration_tests::install_crash_backtrace();

  // Blocked before any thread exists, so every thread inherits the mask and the
  // signal waits for sigwait() below instead of rclcpp's handler.
  sigset_t mask;
  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &mask, nullptr) != 0) {
    return 1;
  }
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  rcl_node_t node = rcl_get_zero_initialized_node();
  rcl_node_options_t options = rcl_node_get_default_options();
  options.enable_rosout = false;
  auto context = rclcpp::contexts::get_global_default_context()->get_rcl_context();
  if (rcl_node_init(&node, "endpointless_node", "", context.get(), &options) != RCL_RET_OK) {
    std::fprintf(stderr, "rcl_node_init failed: %s\n", rcl_get_error_string().str);
    rclcpp::shutdown();
    return 1;
  }

  int signum = 0;
  while (sigwait(&mask, &signum) != 0) {
    // Interrupted before a signal was taken: wait again.
  }

  int exit_code = 0;
  if (rcl_node_fini(&node) != RCL_RET_OK) {
    std::fprintf(stderr, "rcl_node_fini failed: %s\n", rcl_get_error_string().str);
    exit_code = 1;
  }
  if (rcl_node_options_fini(&options) != RCL_RET_OK) {
    exit_code = 1;
  }
  rclcpp::shutdown();
  return exit_code;
}
