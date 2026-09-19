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

#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_manager_node.hpp"

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<ros2_medkit_fault_manager::FaultManagerNode> node;
  // A node that cannot start is a configuration error: log why and exit with code 1.
  try {
    node = std::make_shared<ros2_medkit_fault_manager::FaultManagerNode>();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("fault_manager"), "Fault manager did not start: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
