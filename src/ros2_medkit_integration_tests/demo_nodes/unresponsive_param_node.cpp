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
 * @file unresponsive_param_node.cpp
 * @brief Demo node whose parameter service is discoverable but never replies
 *
 * Regression fixture for #531 (end-to-end counterpart of
 * ros2_medkit_gateway/test/test_ros2_parameter_transport.cpp). Parameter
 * services are NOT auto-started (`start_parameter_services(false)`);
 * instead all six are created manually so that a `SyncParametersClient`'s
 * `wait_for_service()` succeeds immediately (the services genuinely exist
 * in the ROS graph), while `~/list_parameters` - the first round-trip call
 * the gateway makes - never returns.
 *
 * Used by test_param_service_hang.test.py to prove the gateway's REST API
 * stays responsive - bounded 503 errors instead of hanging forever - when a
 * backing node's parameter service is discoverable but unresponsive.
 */

#include <csignal>

#include <chrono>
#include <memory>
#include <thread>

#include <rcl_interfaces/srv/describe_parameters.hpp>
#include <rcl_interfaces/srv/get_parameter_types.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rcl_interfaces/srv/list_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/set_parameters_atomically.hpp>

#include <rclcpp/rclcpp.hpp>

class UnresponsiveParamNode : public rclcpp::Node {
 public:
  UnresponsiveParamNode() : Node("unresponsive_param_node", rclcpp::NodeOptions().start_parameter_services(false)) {
    list_parameters_service_ = create_service<rcl_interfaces::srv::ListParameters>(
        "~/list_parameters", [](const std::shared_ptr<rcl_interfaces::srv::ListParameters::Request> /*request*/,
                                std::shared_ptr<rcl_interfaces::srv::ListParameters::Response> /*response*/) {
          // Never actually replies. Loops on rclcpp::ok() (not a fixed
          // sleep) rather than a condition variable tied to node state, so
          // it needs no member access at all: main() flips rclcpp::ok() to
          // false on SIGINT/SIGTERM before asking the executor to stop, so
          // this callback - and therefore the process - still exits
          // promptly instead of hanging the executor forever.
          while (rclcpp::ok()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
          }
        });

    // The remaining parameter services only need to exist in the ROS graph
    // so that wait_for_service() (discovery only, not responsiveness)
    // succeeds for every SyncParametersClient call the gateway might issue.
    // They are not expected to be invoked in practice: list_parameters is
    // the first round-trip call in the gateway's configuration round trip
    // and it never returns within the gateway's bounded timeout.
    get_parameters_service_ = create_service<rcl_interfaces::srv::GetParameters>(
        "~/get_parameters", [](const std::shared_ptr<rcl_interfaces::srv::GetParameters::Request> /*request*/,
                               std::shared_ptr<rcl_interfaces::srv::GetParameters::Response> /*response*/) {});
    set_parameters_service_ = create_service<rcl_interfaces::srv::SetParameters>(
        "~/set_parameters", [](const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> /*request*/,
                               std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> /*response*/) {});
    describe_parameters_service_ = create_service<rcl_interfaces::srv::DescribeParameters>(
        "~/describe_parameters",
        [](const std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::DescribeParameters::Response> /*response*/) {});
    get_parameter_types_service_ = create_service<rcl_interfaces::srv::GetParameterTypes>(
        "~/get_parameter_types", [](const std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Request> /*request*/,
                                    std::shared_ptr<rcl_interfaces::srv::GetParameterTypes::Response> /*response*/) {});
    set_parameters_atomically_service_ = create_service<rcl_interfaces::srv::SetParametersAtomically>(
        "~/set_parameters_atomically",
        [](const std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Request> /*request*/,
           std::shared_ptr<rcl_interfaces::srv::SetParametersAtomically::Response> /*response*/) {});

    RCLCPP_INFO(get_logger(),
                "UnresponsiveParamNode started: parameter services are registered but "
                "list_parameters will never reply (regression fixture for #531)");
  }

  ~UnresponsiveParamNode() override {
    list_parameters_service_.reset();
    get_parameters_service_.reset();
    set_parameters_service_.reset();
    describe_parameters_service_.reset();
    get_parameter_types_service_.reset();
    set_parameters_atomically_service_.reset();
  }

  UnresponsiveParamNode(const UnresponsiveParamNode &) = delete;
  UnresponsiveParamNode & operator=(const UnresponsiveParamNode &) = delete;
  UnresponsiveParamNode(UnresponsiveParamNode &&) = delete;
  UnresponsiveParamNode & operator=(UnresponsiveParamNode &&) = delete;

 private:
  rclcpp::Service<rcl_interfaces::srv::ListParameters>::SharedPtr list_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameters>::SharedPtr get_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr set_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::DescribeParameters>::SharedPtr describe_parameters_service_;
  rclcpp::Service<rcl_interfaces::srv::GetParameterTypes>::SharedPtr get_parameter_types_service_;
  rclcpp::Service<rcl_interfaces::srv::SetParametersAtomically>::SharedPtr set_parameters_atomically_service_;
};

int main(int argc, char ** argv) {
  // Deliberately NOT ros2_medkit_integration_tests::run_demo_node(): that
  // helper calls rclcpp::shutdown() only after executor.spin() has already
  // returned, which works for every other demo node but would deadlock this
  // one - the list_parameters callback above blocks the executor's worker on
  // rclcpp::ok(), so shutdown() must be called BEFORE the executor is asked
  // to stop, not after, or spin() never returns.
  //
  // Same building block as run_demo_node() otherwise: block SIGINT/SIGTERM
  // up front and consume them on a dedicated sigwait thread (a normal thread
  // context) rather than rclcpp's own async-signal-context handler, which is
  // what makes it safe to call rclcpp::shutdown() from that thread at all.
  sigset_t mask;
  sigset_t old_mask;
  sigemptyset(&mask);
  sigaddset(&mask, SIGINT);
  sigaddset(&mask, SIGTERM);
  if (pthread_sigmask(SIG_BLOCK, &mask, &old_mask) != 0) {
    return 1;
  }

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);

  {
    auto node = std::make_shared<UnresponsiveParamNode>();

    // MultiThreadedExecutor (not the SingleThreadedExecutor other demo nodes
    // use): several test HTTP workers can each trigger a concurrent
    // list_parameters round trip against this node, and each must get its
    // own blocked worker thread here rather than queuing behind a single
    // stuck one - matching the "several requests genuinely in flight at
    // once" scenario the /health-stays-responsive test exercises.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    std::thread signal_thread([&mask, &executor] {
      int signum = 0;
      while (true) {
        const int rc = sigwait(&mask, &signum);
        if (rc == 0) {
          break;
        }
        // EINTR can theoretically fire; retry.
      }
      // Flip rclcpp::ok() to false first so the blocking list_parameters
      // callback notices and returns; only then ask the executor to stop.
      rclcpp::shutdown();
      executor.cancel();
    });

    executor.spin();

    if (signal_thread.joinable()) {
      signal_thread.join();
    }

    executor.remove_node(node);
    node.reset();
  }

  // rclcpp::shutdown() already ran on the signal thread above.
  pthread_sigmask(SIG_SETMASK, &old_mask, nullptr);
  return 0;
}
