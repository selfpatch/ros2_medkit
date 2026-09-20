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

// A node joins rclcpp's graph listener on its first graph wait. A first wait after
// rclcpp::shutdown() leaves the node half-registered, and ~NodeGraph then aborts the
// process. Each case runs in a child process that exits 0 only when teardown survives.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <exception>
#include <memory>

#include <lifecycle_msgs/srv/get_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/ros2/status/ros2_lifecycle_state_reader.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_fault_service_transport.hpp"

namespace ros2_medkit_gateway {

struct Ros2LifecycleStateReaderTestAccess {
  static rclcpp::Node::SharedPtr client_node(Ros2LifecycleStateReader & reader) {
    return reader.client_node_;
  }
};

}  // namespace ros2_medkit_gateway

namespace {

using ros2_medkit_gateway::Ros2LifecycleStateReader;
using ros2_medkit_gateway::Ros2LifecycleStateReaderTestAccess;
using ros2_medkit_gateway::ros2::Ros2FaultServiceTransport;

constexpr std::chrono::milliseconds kWait{100};

/// Starts rclcpp and a host node that has joined the graph listener, as GatewayNode has.
std::shared_ptr<rclcpp::Node> start_host(const char * name) {
  rclcpp::init(0, nullptr);
  auto host = std::make_shared<rclcpp::Node>(name);
  static_cast<void>(host->get_graph_event());
  return host;
}

void fault_transport_waits_after_shutdown() {
  auto host = start_host("graph_join_fault_host");
  auto transport = std::make_unique<Ros2FaultServiceTransport>(host.get());
  rclcpp::shutdown();
  try {
    // No fault manager runs, so the wait takes the graph-event path.
    static_cast<void>(transport->wait_for_services(kWait));
  } catch (const std::exception &) {
    // The gateway's callers catch it too. The abort comes at teardown.
  }
  transport.reset();
  host.reset();
  std::exit(0);
}

void lifecycle_reader_waits_after_shutdown() {
  auto host = start_host("graph_join_lifecycle_host");
  auto reader = std::make_unique<Ros2LifecycleStateReader>(host.get(), kWait);
  // get_state() creates its client, then waits: a shutdown between the two.
  auto client = Ros2LifecycleStateReaderTestAccess::client_node(*reader)->create_client<lifecycle_msgs::srv::GetState>(
      "/graph_join_absent_node/get_state");
  rclcpp::shutdown();
  try {
    static_cast<void>(client->wait_for_service(kWait));
  } catch (const std::exception &) {
    // Swallowed like the fault transport case: only the teardown is under test.
  }
  client.reset();
  reader.reset();
  host.reset();
  std::exit(0);
}

}  // namespace

// Re-executes the binary for each child, so no rclcpp or DDS thread is forked.
class GraphListenerJoinTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ::testing::GTEST_FLAG(death_test_style) = "threadsafe";
  }
};

TEST_F(GraphListenerJoinTest, FaultTransportSurvivesAServiceWaitAfterShutdown) {
  EXPECT_EXIT(fault_transport_waits_after_shutdown(), ::testing::ExitedWithCode(0), "");
}

TEST_F(GraphListenerJoinTest, LifecycleStateReaderSurvivesAServiceWaitAfterShutdown) {
  EXPECT_EXIT(lifecycle_reader_waits_after_shutdown(), ::testing::ExitedWithCode(0), "");
}
