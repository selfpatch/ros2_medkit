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
#include <std_msgs/msg/float32.hpp>

class RPMSensor : public rclcpp::Node {
 public:
  RPMSensor() : Node("rpm_sensor") {
    rpm_pub_ = this->create_publisher<std_msgs::msg::Float32>("rpm", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RPMSensor::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "RPM sensor started");
  }

 private:
  void publish_data() {
    current_rpm_ += 50.0;
    if (current_rpm_ > 3000.0) {
      current_rpm_ = 1000.0;
    }

    auto rpm_msg = std_msgs::msg::Float32();
    rpm_msg.data = current_rpm_;

    rpm_pub_->publish(rpm_msg);

    RCLCPP_INFO(this->get_logger(), "RPM: %.0f", current_rpm_);
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rpm_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double current_rpm_ = 1000.0;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RPMSensor>());
  rclcpp::shutdown();
  return 0;
}
