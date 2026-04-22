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

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

class BrakePressureSensor : public rclcpp::Node {
 public:
  BrakePressureSensor() : Node("brake_pressure_sensor") {
    pressure_pub_ = this->create_publisher<std_msgs::msg::Float32>("pressure", 10);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&BrakePressureSensor::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "Brake pressure sensor started");
  }

  ~BrakePressureSensor() {
    timer_->cancel();
    std::lock_guard<std::mutex> lock(callback_mutex_);
    timer_.reset();
    pressure_pub_.reset();
  }

 private:
  void publish_data() {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (!pressure_pub_) {
      return;
    }
    current_pressure_ += 5.0;
    if (current_pressure_ > 100.0) {
      current_pressure_ = 0.0;
    }

    auto pressure_msg = std_msgs::msg::Float32();
    pressure_msg.data = static_cast<float>(current_pressure_);

    pressure_pub_->publish(pressure_msg);

    RCLCPP_INFO(this->get_logger(), "Brake Pressure: %.1f bar", current_pressure_);
  }

  std::mutex callback_mutex_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pressure_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double current_pressure_ = 0.0;
};

int main(int argc, char ** argv) {
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, []() -> std::shared_ptr<rclcpp::Node> {
    return std::make_shared<BrakePressureSensor>();
  });
}
