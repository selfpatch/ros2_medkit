// Copyright 2025 mfaferek93
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
 * @file calibration_service.cpp
 * @brief Demo calibration service
 *
 * Provides calibration service that:
 * - Exposes /powertrain/engine/calibrate service (Trigger type)
 * - Returns success with calibration message
 * - Used to test POST /services/{service} endpoint
 */

#include <csignal>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class CalibrationService : public rclcpp::Node {
 public:
  CalibrationService() : Node("calibration_service"), calibration_count_(0) {
    // Create calibration service
    calibration_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate",
        std::bind(&CalibrationService::calibrate_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Calibration service started");
  }

  ~CalibrationService() {
    calibration_srv_.reset();
  }

 private:
  void calibrate_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    (void)request;  // Trigger has no request fields

    calibration_count_++;

    // Simulate calibration
    response->success = true;
    response->message = "Engine calibrated successfully (count: " + std::to_string(calibration_count_) + ")";

    RCLCPP_INFO(this->get_logger(), "Calibration requested: %s", response->message.c_str());
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibration_srv_;
  int calibration_count_;
};

int main(int argc, char * argv[]) {
  // Block SIGINT/SIGTERM until all rcl resources (node, executor guard
  // condition) are allocated. A signal arriving during init / executor setup
  // invalidates the context mid-call, causing rcl_* to throw RCLError. By
  // holding the block through executor->add_node() the guard condition is
  // created while the context is still valid; unblock fires any queued
  // signal which rclcpp then handles as a normal shutdown.
  sigset_t mask, old;
  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  pthread_sigmask(SIG_BLOCK, &mask, &old);

  rclcpp::init(argc, argv);
  auto node = std::make_shared<CalibrationService>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  pthread_sigmask(SIG_SETMASK, &old, nullptr);

  executor.spin();
  executor.remove_node(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
