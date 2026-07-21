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
 * @file engine_temp_monitor.cpp
 * @brief Demo engine temperature monitor
 *
 * Subscribes to the temperature topic published by engine_temp_sensor and
 * tracks the latest reading against an overheat threshold.
 *
 * This node exists so the demo stack contains a real publisher/subscriber pair
 * on one topic inside one SOVD Function. Dataflow-graph resources need such a
 * pair to produce an edge; without it the graph is always empty.
 */

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "ros2_medkit_integration_tests/demo_node_main.hpp"

class EngineTempMonitor : public rclcpp::Node {
 public:
  EngineTempMonitor() : Node("engine_temp_monitor") {
    rcl_interfaces::msg::ParameterDescriptor threshold_desc;
    threshold_desc.description = "Overheat threshold in degrees Celsius";
    this->declare_parameter("overheat_threshold", 110.0, threshold_desc);

    // Relative name: resolves against the node namespace, so under
    // /powertrain/engine this subscribes to /powertrain/engine/temperature.
    temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
        "temperature", 10, std::bind(&EngineTempMonitor::temperature_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Engine temperature monitor started");
  }

  ~EngineTempMonitor() override {
    // Reset the subscription before implicit member destruction so the
    // callback cannot fire on a partially destroyed object.
    std::lock_guard<std::mutex> lock(callback_mutex_);
    temp_sub_.reset();
  }

  EngineTempMonitor(const EngineTempMonitor &) = delete;
  EngineTempMonitor & operator=(const EngineTempMonitor &) = delete;
  EngineTempMonitor(EngineTempMonitor &&) = delete;
  EngineTempMonitor & operator=(EngineTempMonitor &&) = delete;

 private:
  void temperature_callback(const sensor_msgs::msg::Temperature::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(callback_mutex_);
    last_temperature_ = msg->temperature;
    ++sample_count_;

    const double threshold = this->get_parameter("overheat_threshold").as_double();
    if (last_temperature_ > threshold) {
      RCLCPP_WARN(this->get_logger(), "Engine temperature %.2f C above threshold %.2f C", last_temperature_, threshold);
    }
  }

  std::mutex callback_mutex_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
  double last_temperature_{0.0};
  uint64_t sample_count_{0};
};

int main(int argc, char * argv[]) {
  return ros2_medkit_integration_tests::run_demo_node(argc, argv, []() -> std::shared_ptr<rclcpp::Node> {
    return std::make_shared<EngineTempMonitor>();
  });
}
