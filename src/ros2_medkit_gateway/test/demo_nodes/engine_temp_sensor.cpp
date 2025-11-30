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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

class EngineTempSensor : public rclcpp::Node {
 public:
  EngineTempSensor() : Node("engine_temp_sensor") {
    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&EngineTempSensor::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "Engine temperature sensor started");
  }

 private:
  void publish_data() {
    current_temp_ += 0.5;
    if (current_temp_ > 95.0) {
      current_temp_ = 85.0;
    }

    auto temp_msg = sensor_msgs::msg::Temperature();
    temp_msg.header.stamp = this->now();
    temp_msg.header.frame_id = "engine";
    temp_msg.temperature = current_temp_;
    temp_msg.variance = 0.5;

    temp_pub_->publish(temp_msg);

    RCLCPP_INFO(this->get_logger(), "Temperature: %.1f°C", current_temp_);
  }

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temp_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double current_temp_ = 85.0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EngineTempSensor>());
  rclcpp::shutdown();
  return 0;
}
