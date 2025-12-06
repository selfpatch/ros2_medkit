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
 * @file long_calibration_action.cpp
 * @brief Demo long-running calibration action server
 *
 * Provides long_calibration action that:
 * - Exposes /powertrain/engine/long_calibration action (Fibonacci type)
 * - Simulates a long-running calibration process
 * - Publishes progress feedback during execution
 * - Used to test async operation endpoints
 *
 * Uses example_interfaces/action/Fibonacci:
 * - Goal: int32 order (number of calibration steps)
 * - Feedback: int32[] sequence (calibration progress)
 * - Result: int32[] sequence (final calibration values)
 */

#include <example_interfaces/action/fibonacci.hpp>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <thread>

class LongCalibrationAction : public rclcpp::Node {
 public:
  using Fibonacci = example_interfaces::action::Fibonacci;
  using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  LongCalibrationAction() : Node("long_calibration") {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<Fibonacci>(
        this, "long_calibration", std::bind(&LongCalibrationAction::handle_goal, this, _1, _2),
        std::bind(&LongCalibrationAction::handle_cancel, this, _1),
        std::bind(&LongCalibrationAction::handle_accepted, this, _1));

    RCLCPP_INFO(get_logger(), "Long calibration action server started");
  }

 private:
  rclcpp_action::Server<Fibonacci>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                          std::shared_ptr<const Fibonacci::Goal> goal) {
    (void)uuid;
    RCLCPP_INFO(get_logger(), "Received calibration goal request with order %d", goal->order);

    // Reject if order is too large (to prevent very long calibrations)
    if (goal->order > 50) {
      RCLCPP_WARN(get_logger(), "Goal rejected: order too large (max 50)");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel calibration");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    // Execute in a separate thread to avoid blocking
    std::thread{std::bind(&LongCalibrationAction::execute, this, goal_handle)}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFibonacci> goal_handle) {
    RCLCPP_INFO(get_logger(), "Executing calibration...");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Fibonacci::Feedback>();
    auto & sequence = feedback->sequence;
    auto result = std::make_shared<Fibonacci::Result>();

    // Initialize Fibonacci sequence
    sequence.push_back(0);
    sequence.push_back(1);

    // Simulate calibration steps with 500ms delays
    rclcpp::Rate loop_rate(2);  // 2 Hz = 500ms between iterations

    for (int i = 1; i < goal->order && rclcpp::ok(); ++i) {
      // Check if cancelled
      if (goal_handle->is_canceling()) {
        result->sequence = sequence;
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "Calibration canceled");
        return;
      }

      // Compute next Fibonacci number (simulating calibration step)
      sequence.push_back(sequence[i] + sequence[i - 1]);

      // Publish feedback
      goal_handle->publish_feedback(feedback);
      RCLCPP_DEBUG(get_logger(), "Publishing calibration feedback: step %d/%d", static_cast<int>(sequence.size()),
                   goal->order);

      loop_rate.sleep();
    }

    // Complete the calibration
    if (rclcpp::ok()) {
      result->sequence = sequence;
      goal_handle->succeed(result);
      RCLCPP_INFO(get_logger(), "Calibration completed successfully with %zu steps", sequence.size());
    }
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LongCalibrationAction>());
  rclcpp::shutdown();
  return 0;
}
