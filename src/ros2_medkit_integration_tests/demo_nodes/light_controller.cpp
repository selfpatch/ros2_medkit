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
 * @file light_controller.cpp
 * @brief Demo light controller
 *
 * Simulates light system that:
 * - Subscribes to command topic (Bool - on/off)
 * - Publishes status topic (Bool - current state)
 * - Used to test PUT /data/{topic} endpoint with boolean values
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

using namespace std::chrono_literals;

class LightController : public rclcpp::Node {
 public:
  LightController() : Node("light_controller"), light_on_(false) {
    // Subscribe to command topic
    cmd_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "command", 10, std::bind(&LightController::command_callback, this, std::placeholders::_1));

    // Publish status topic
    status_pub_ = this->create_publisher<std_msgs::msg::Bool>("status", 10);

    // Timer to periodically publish status
    timer_ = this->create_wall_timer(100ms, std::bind(&LightController::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Light controller started");
  }

 private:
  void command_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    light_on_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "Light command: %s", light_on_ ? "ON" : "OFF");
  }

  void timer_callback() {
    // Publish current status
    auto msg = std_msgs::msg::Bool();
    msg.data = light_on_;
    status_pub_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool light_on_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LightController>());
  rclcpp::shutdown();
  return 0;
}
