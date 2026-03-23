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

#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_medkit_msgs/msg/medkit_discovery_hint.hpp>

#include <unistd.h>

#include <chrono>
#include <string>
#include <vector>

class BeaconPublisher : public rclcpp::Node {
 public:
  BeaconPublisher() : Node("beacon_publisher") {
    declare_parameter("beacon_rate_hz", 1.0);
    declare_parameter("beacon_entity_id", "");
    declare_parameter("beacon_transport_type", "");
    declare_parameter("beacon_function_ids", std::vector<std::string>{});
    declare_parameter("beacon_pause", false);
    declare_parameter("beacon_topic", "/ros2_medkit/discovery");
    declare_parameter("beacon_process_name", "beacon_publisher");

    auto topic = get_parameter("beacon_topic").as_string();
    publisher_ = create_publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>(topic, 10);

    auto rate_hz = get_parameter("beacon_rate_hz").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate_hz);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(period), [this]() {
      publish_beacon();
    });

    RCLCPP_INFO(get_logger(), "BeaconPublisher started on '%s' at %.1f Hz", topic.c_str(), rate_hz);
  }

 private:
  void publish_beacon() {
    if (get_parameter("beacon_pause").as_bool()) {
      return;
    }

    auto msg = ros2_medkit_msgs::msg::MedkitDiscoveryHint();
    msg.entity_id = get_parameter("beacon_entity_id").as_string();
    msg.transport_type = get_parameter("beacon_transport_type").as_string();
    msg.function_ids = get_parameter("beacon_function_ids").as_string_array();
    msg.process_id = static_cast<uint32_t>(getpid());
    msg.process_name = get_parameter("beacon_process_name").as_string();
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) == 0) {
      msg.hostname = hostname;
    }
    msg.stamp = now();

    publisher_->publish(msg);
  }

  rclcpp::Publisher<ros2_medkit_msgs::msg::MedkitDiscoveryHint>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BeaconPublisher>());
  rclcpp::shutdown();
  return 0;
}
