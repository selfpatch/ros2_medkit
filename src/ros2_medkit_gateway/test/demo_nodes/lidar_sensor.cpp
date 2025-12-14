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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

/// Demo LIDAR sensor node that reports faults based on parameter validation.
/// Used to demonstrate the fault management workflow:
/// 1. Node detects parameter issues and reports faults
/// 2. Client reads faults via REST API
/// 3. Client fixes parameters via REST API
/// 4. Client clears faults via REST API
/// 5. Client runs calibration via REST API
class LidarSensor : public rclcpp::Node {
 public:
  LidarSensor() : Node("lidar_sensor") {
    // Declare parameters with defaults and descriptions
    auto min_range_desc = rcl_interfaces::msg::ParameterDescriptor();
    min_range_desc.description = "Minimum detection range in meters";
    this->declare_parameter("min_range", 0.1, min_range_desc);

    auto max_range_desc = rcl_interfaces::msg::ParameterDescriptor();
    max_range_desc.description = "Maximum detection range in meters";
    this->declare_parameter("max_range", 30.0, max_range_desc);

    auto scan_frequency_desc = rcl_interfaces::msg::ParameterDescriptor();
    scan_frequency_desc.description = "Scan frequency in Hz (max supported: 20.0)";
    this->declare_parameter("scan_frequency", 10.0, scan_frequency_desc);

    auto angular_resolution_desc = rcl_interfaces::msg::ParameterDescriptor();
    angular_resolution_desc.description = "Angular resolution in degrees";
    this->declare_parameter("angular_resolution", 0.25, angular_resolution_desc);

    // Get initial parameter values
    min_range_ = this->get_parameter("min_range").as_double();
    max_range_ = this->get_parameter("max_range").as_double();
    scan_frequency_ = this->get_parameter("scan_frequency").as_double();
    angular_resolution_ = this->get_parameter("angular_resolution").as_double();

    // Create publisher for laser scan data
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

    // Create calibration service
    calibrate_srv_ = this->create_service<std_srvs::srv::Trigger>(
        "calibrate", std::bind(&LidarSensor::handle_calibrate, this, std::placeholders::_1, std::placeholders::_2));

    // Create fault reporting client
    report_fault_client_ = this->create_client<ros2_medkit_msgs::srv::ReportFault>("/fault_manager/report_fault");

    // Set up parameter callback for dynamic updates
    param_callback_handle_ =
        this->add_on_set_parameters_callback(std::bind(&LidarSensor::on_parameter_change, this, std::placeholders::_1));

    // Timer for publishing scan data
    int period_ms = static_cast<int>(1000.0 / scan_frequency_);
    scan_timer_ =
        this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&LidarSensor::publish_scan, this));

    // Timer for periodic fault checking (every 2 seconds)
    fault_check_timer_ =
        this->create_wall_timer(std::chrono::seconds(2), std::bind(&LidarSensor::check_and_report_faults, this));

    RCLCPP_INFO(this->get_logger(), "LIDAR sensor started (freq: %.1f Hz, range: %.1f-%.1f m)", scan_frequency_,
                min_range_, max_range_);

    // Initial fault check after short delay (allow fault_manager to start)
    initial_check_timer_ = this->create_wall_timer(std::chrono::seconds(3), [this]() {
      check_and_report_faults();
      initial_check_timer_->cancel();  // Only run once
    });
  }

 private:
  rcl_interfaces::msg::SetParametersResult on_parameter_change(const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters) {
      if (param.get_name() == "min_range") {
        min_range_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "min_range changed to %.2f m", min_range_);
      } else if (param.get_name() == "max_range") {
        max_range_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "max_range changed to %.2f m", max_range_);
      } else if (param.get_name() == "scan_frequency") {
        double new_freq = param.as_double();
        if (new_freq <= 0.0) {
          result.successful = false;
          result.reason = "scan_frequency must be positive";
          return result;
        }
        scan_frequency_ = new_freq;
        // Recreate timer with new frequency
        int period_ms = static_cast<int>(1000.0 / scan_frequency_);
        scan_timer_ =
            this->create_wall_timer(std::chrono::milliseconds(period_ms), std::bind(&LidarSensor::publish_scan, this));
        RCLCPP_INFO(this->get_logger(), "scan_frequency changed to %.1f Hz", scan_frequency_);
      } else if (param.get_name() == "angular_resolution") {
        angular_resolution_ = param.as_double();
        RCLCPP_INFO(this->get_logger(), "angular_resolution changed to %.2f deg", angular_resolution_);
      }
    }

    // Trigger fault check after parameter change
    check_and_report_faults();

    return result;
  }

  void handle_calibrate(const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Calibration requested...");

    // Simulate calibration process
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    is_calibrated_ = true;
    last_calibration_time_ = this->now();

    response->success = true;
    response->message = "LIDAR calibration completed successfully";

    RCLCPP_INFO(this->get_logger(), "Calibration completed");
  }

  void check_and_report_faults() {
    if (!report_fault_client_->service_is_ready()) {
      RCLCPP_DEBUG(this->get_logger(), "Fault manager service not available, skipping fault check");
      return;
    }

    // Check for range configuration error
    if (min_range_ >= max_range_) {
      report_fault("LIDAR_RANGE_INVALID", ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR,
                   "Invalid range configuration: min_range (" + std::to_string(min_range_) + ") >= max_range (" +
                       std::to_string(max_range_) + ")");
    }

    // Check for unsupported frequency
    if (scan_frequency_ > 20.0) {
      report_fault("LIDAR_FREQ_UNSUPPORTED", ros2_medkit_msgs::msg::Fault::SEVERITY_WARN,
                   "Scan frequency " + std::to_string(scan_frequency_) + " Hz exceeds maximum supported (20.0 Hz)");
    }

    // Check for calibration status
    if (!is_calibrated_) {
      report_fault("LIDAR_CALIBRATION_REQUIRED", ros2_medkit_msgs::msg::Fault::SEVERITY_INFO,
                   "LIDAR sensor requires calibration before accurate operation");
    }
  }

  void report_fault(const std::string & fault_code, uint8_t severity, const std::string & description) {
    auto request = std::make_shared<ros2_medkit_msgs::srv::ReportFault::Request>();
    request->fault_code = fault_code;
    request->severity = severity;
    request->description = description;
    request->source_id = this->get_fully_qualified_name();

    auto future = report_fault_client_->async_send_request(request);

    // Don't block - fire and forget
    RCLCPP_DEBUG(this->get_logger(), "Reported fault: %s", fault_code.c_str());
  }

  void publish_scan() {
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = "lidar_link";

    scan_msg.angle_min = 0.0F;
    scan_msg.angle_max = static_cast<float>(2.0 * M_PI);
    scan_msg.angle_increment = static_cast<float>(angular_resolution_ * M_PI / 180.0);
    scan_msg.time_increment = 0.0F;
    scan_msg.scan_time = static_cast<float>(1.0 / scan_frequency_);
    scan_msg.range_min = static_cast<float>(min_range_);
    scan_msg.range_max = static_cast<float>(max_range_);

    // Generate dummy scan data (simulated distances)
    size_t num_readings = static_cast<size_t>((scan_msg.angle_max - scan_msg.angle_min) / scan_msg.angle_increment);
    scan_msg.ranges.resize(num_readings);
    scan_msg.intensities.resize(num_readings);

    for (size_t i = 0; i < num_readings; ++i) {
      // Simulate varying distances
      double idx = static_cast<double>(i);
      scan_msg.ranges[i] = static_cast<float>(min_range_ + (max_range_ - min_range_) * (0.5 + 0.3 * sin(idx * 0.1)));
      scan_msg.intensities[i] = 100.0F;
    }

    scan_pub_->publish(scan_msg);
  }

  // Publishers and services
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr calibrate_srv_;
  rclcpp::Client<ros2_medkit_msgs::srv::ReportFault>::SharedPtr report_fault_client_;

  // Timers
  rclcpp::TimerBase::SharedPtr scan_timer_;
  rclcpp::TimerBase::SharedPtr fault_check_timer_;
  rclcpp::TimerBase::SharedPtr initial_check_timer_;

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // Parameters
  double min_range_;
  double max_range_;
  double scan_frequency_;
  double angular_resolution_;

  // State
  bool is_calibrated_{false};
  rclcpp::Time last_calibration_time_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSensor>());
  rclcpp::shutdown();
  return 0;
}
