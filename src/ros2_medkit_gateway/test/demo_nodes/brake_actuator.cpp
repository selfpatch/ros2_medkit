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
 * @file brake_actuator.cpp
 * @brief Demo brake actuator
 *
 * Simulates brake system that:
 * - Subscribes to command topic (Float32)
 * - Publishes actual pressure (simulated gradual response)
 * - Used to test PUT /data/{topic} endpoint
 */

#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

using namespace std::chrono_literals;

class BrakeActuator : public rclcpp::Node {
 public:
  BrakeActuator() : Node("brake_actuator"), current_pressure_(0.0f), target_pressure_(0.0f) {
    // Subscribe to command topic
    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "command", 10, std::bind(&BrakeActuator::command_callback, this, std::placeholders::_1));

    // Publish pressure topic
    pressure_pub_ = this->create_publisher<std_msgs::msg::Float32>("pressure", 10);

    // Timer to simulate gradual pressure change
    timer_ = this->create_wall_timer(100ms, std::bind(&BrakeActuator::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Brake actuator started");
  }

 private:
  void command_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    target_pressure_ = msg->data;

    // Clamp to valid range (0-100 bar)
    if (target_pressure_ < 0.0f) {
      target_pressure_ = 0.0f;
    }
    if (target_pressure_ > 100.0f) {
      target_pressure_ = 100.0f;
    }

    RCLCPP_INFO(this->get_logger(), "Received command: %.2f bar", target_pressure_);
  }

  void timer_callback() {
    // Simulate gradual pressure change (5 bar/sec = 0.5 bar per 100ms)
    const float step = 0.5f;

    if (std::abs(current_pressure_ - target_pressure_) < step) {
      current_pressure_ = target_pressure_;
    } else if (current_pressure_ < target_pressure_) {
      current_pressure_ += step;
    } else {
      current_pressure_ -= step;
    }

    // Publish current pressure
    auto msg = std_msgs::msg::Float32();
    msg.data = current_pressure_;
    pressure_pub_->publish(msg);
  }

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr cmd_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressure_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  float current_pressure_;
  float target_pressure_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrakeActuator>());
  rclcpp::shutdown();
  return 0;
}
