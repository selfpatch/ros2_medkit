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
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

class DoorStatusSensor : public rclcpp::Node {
 public:
  DoorStatusSensor() : Node("door_status_sensor") {
    is_open_pub_ = this->create_publisher<std_msgs::msg::Bool>("is_open", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::String>("state", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&DoorStatusSensor::publish_data, this));

    RCLCPP_INFO(this->get_logger(), "Door status sensor started");
  }

 private:
  void publish_data() {
    // Toggle door state
    is_open_ = !is_open_;

    auto is_open_msg = std_msgs::msg::Bool();
    is_open_msg.data = is_open_;
    is_open_pub_->publish(is_open_msg);

    auto state_msg = std_msgs::msg::String();
    state_msg.data = is_open_ ? "open" : "closed";
    state_pub_->publish(state_msg);

    RCLCPP_INFO(this->get_logger(), "Door: %s", state_msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr is_open_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool is_open_ = false;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoorStatusSensor>());
  rclcpp::shutdown();
  return 0;
}
