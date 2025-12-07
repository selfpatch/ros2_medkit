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

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/temperature.hpp>

class EngineTempSensor : public rclcpp::Node {
 public:
  EngineTempSensor() : Node("engine_temp_sensor") {
    // Declare parameters with defaults and descriptions
    auto publish_rate_desc = rcl_interfaces::msg::ParameterDescriptor();
    publish_rate_desc.description = "Publishing rate in Hz";
    publish_rate_desc.floating_point_range.resize(1);
    publish_rate_desc.floating_point_range[0].from_value = 0.1;
    publish_rate_desc.floating_point_range[0].to_value = 100.0;
    publish_rate_desc.floating_point_range[0].step = 0.0;
    this->declare_parameter("publish_rate", 2.0, publish_rate_desc);

    auto min_temp_desc = rcl_interfaces::msg::ParameterDescriptor();
    min_temp_desc.description = "Minimum temperature value in Celsius";
    this->declare_parameter("min_temp", 85.0, min_temp_desc);

    auto max_temp_desc = rcl_interfaces::msg::ParameterDescriptor();
    max_temp_desc.description = "Maximum temperature value in Celsius";
    this->declare_parameter("max_temp", 95.0, max_temp_desc);

    auto temp_step_desc = rcl_interfaces::msg::ParameterDescriptor();
    temp_step_desc.description = "Temperature increment per publish cycle";
    this->declare_parameter("temp_step", 0.5, temp_step_desc);

    // Get parameter values
    publish_rate_ = this->get_parameter("publish_rate").as_double();
    min_temp_ = this->get_parameter("min_temp").as_double();
    max_temp_ = this->get_parameter("max_temp").as_double();
    temp_step_ = this->get_parameter("temp_step").as_double();
    current_temp_ = min_temp_;

    temp_pub_ = this->create_publisher<sensor_msgs::msg::Temperature>("temperature", 10);

    int period_ms = static_cast<int>(1000.0 / publish_rate_);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&EngineTempSensor::publish_data, this));

    // Set up parameter callback for dynamic updates
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&EngineTempSensor::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Engine temperature sensor started (rate: %.1f Hz, range: %.1f-%.1f°C)",
                publish_rate_, min_temp_, max_temp_);
  }

 private:
  rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "publish_rate") {
        double new_rate = param.as_double();
        if (new_rate < 0.1 || new_rate > 100.0) {
          result.successful = false;
          result.reason = "publish_rate must be between 0.1 and 100.0 Hz";
          return result;
        }
        publish_rate_ = new_rate;
        // Recreate timer with new rate
        int period_ms = static_cast<int>(1000.0 / publish_rate_);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(period_ms),
                                         std::bind(&EngineTempSensor::publish_data, this));
        RCLCPP_INFO(this->get_logger(), "Publish rate changed to %.1f Hz", publish_rate_);
      } else if (param.get_name() == "min_temp") {
        min_temp_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Min temp changed to %.1f°C", min_temp_);
      } else if (param.get_name() == "max_temp") {
        max_temp_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Max temp changed to %.1f°C", max_temp_);
      } else if (param.get_name() == "temp_step") {
        temp_step_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "Temp step changed to %.1f°C", temp_step_);
      }
    }

    return result;
  }

  void publish_data() {
    current_temp_ += temp_step_;
    if (current_temp_ > max_temp_) {
      current_temp_ = min_temp_;
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
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  double publish_rate_;
  double min_temp_;
  double max_temp_;
  double temp_step_;
  double current_temp_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EngineTempSensor>());
  rclcpp::shutdown();
  return 0;
}
