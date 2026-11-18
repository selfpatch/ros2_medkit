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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

#include "ros2_medkit_gateway/ros2_common/helper_node.hpp"

using ros2_medkit_gateway::ros2_common::make_helper_node;

namespace {

/// A context initialised with @p args as process-wide arguments, as launch_ros passes them.
class ProcessContext {
 public:
  explicit ProcessContext(std::vector<std::string> args) : args_(std::move(args)) {
    args_.insert(args_.begin(), "gateway_node");
    std::vector<const char *> argv;
    argv.reserve(args_.size());
    for (const auto & arg : args_) {
      argv.push_back(arg.c_str());
    }
    context_ = std::make_shared<rclcpp::Context>();
    context_->init(static_cast<int>(argv.size()), argv.data());
  }

  ~ProcessContext() {
    context_->shutdown("test done");
  }

  ProcessContext(const ProcessContext &) = delete;
  ProcessContext & operator=(const ProcessContext &) = delete;
  ProcessContext(ProcessContext &&) = delete;
  ProcessContext & operator=(ProcessContext &&) = delete;

  rclcpp::NodeOptions options() const {
    return rclcpp::NodeOptions().context(context_);
  }

 private:
  std::vector<std::string> args_;
  std::shared_ptr<rclcpp::Context> context_;
};

/// Names of the nodes that publish on /rosout, once @p expected is among them.
std::vector<std::string> rosout_publishers(rclcpp::Node & observer, const std::string & expected) {
  std::vector<std::string> names;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    names.clear();
    for (const auto & info : observer.get_publishers_info_by_topic("/rosout")) {
      names.push_back(info.node_name());
    }
    if (std::find(names.begin(), names.end(), expected) != names.end()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  return names;
}

}  // namespace

TEST(HelperNode, KeepsOwnNameUnderProcessWideNodeRemap) {
  ProcessContext process({"--ros-args", "-r", "__node:=gateway", "-r", "__ns:=/robot1", "-r", "/in:=/out"});
  auto host = std::make_shared<rclcpp::Node>("gateway_node", process.options());
  ASSERT_STREQ(host->get_name(), "gateway");

  auto helper = make_helper_node(*host, "_clients");

  EXPECT_STREQ(helper->get_name(), "_gateway_clients");
  EXPECT_STREQ(helper->get_namespace(), "/robot1");
  // Process-wide remaps other than the node name still apply.
  EXPECT_EQ(helper->get_node_topics_interface()->resolve_topic_name("/in"), "/out");
}

TEST(HelperNode, InheritsUseSimTimeFromParametersKeyedToHost) {
  char path[] = "/tmp/helper_node_params_XXXXXX";
  const int fd = mkstemp(path);
  ASSERT_NE(fd, -1);
  close(fd);
  {
    std::ofstream file(path);
    file << "/robot1/gateway:\n  ros__parameters:\n    use_sim_time: true\n";
  }
  ProcessContext process({"--ros-args", "-r", "__node:=gateway", "-r", "__ns:=/robot1", "--params-file", path});
  auto host = std::make_shared<rclcpp::Node>("gateway_node", process.options());
  ASSERT_TRUE(host->get_parameter("use_sim_time").as_bool());

  auto helper = make_helper_node(*host, "_clients");

  EXPECT_TRUE(helper->get_parameter("use_sim_time").as_bool());
  std::remove(path);
}

TEST(HelperNode, UsesNamespaceGivenToHostConstructor) {
  ProcessContext process({});
  auto host = std::make_shared<rclcpp::Node>("gateway", "/robot2", process.options());

  auto helper = make_helper_node(*host, "_sub");

  EXPECT_STREQ(helper->get_name(), "_gateway_sub");
  EXPECT_STREQ(helper->get_namespace(), "/robot2");
  EXPECT_FALSE(helper->get_parameter("use_sim_time").as_bool());
}

TEST(HelperNode, ServesNoParameters) {
  ProcessContext process({"--ros-args", "-r", "__node:=gateway"});
  auto host = std::make_shared<rclcpp::Node>("gateway_node", process.options());

  auto helper = make_helper_node(*host, "_sub");

  EXPECT_FALSE(helper->get_node_options().start_parameter_services());
  EXPECT_FALSE(helper->get_node_options().start_parameter_event_publisher());
}

TEST(HelperNode, PublishesNoRosout) {
  ProcessContext process({});
  auto host = std::make_shared<rclcpp::Node>("gateway", process.options());

  auto helper = make_helper_node(*host, "_sub");

  const auto names = rosout_publishers(*host, "gateway");
  ASSERT_EQ(std::count(names.begin(), names.end(), "gateway"), 1) << "the host's /rosout is not in the graph";
  EXPECT_EQ(std::count(names.begin(), names.end(), "_gateway_sub"), 0) << "the helper publishes on /rosout";
}

// Context shutdown finalises every /rosout publisher on the shutting-down thread.
// Under TSan this test reports a race if the helper has one while another thread
// creates and destroys entities on the helper. One round catches it about half
// the time, so the test runs several.
TEST(HelperNode, ContextShutdownLeavesHelperEntitiesAlone) {
  constexpr int kRounds = 10;
  for (int round = 0; round < kRounds; ++round) {
    auto process = std::make_unique<ProcessContext>(std::vector<std::string>{});
    auto host = std::make_shared<rclcpp::Node>("gateway", process->options());
    auto helper = make_helper_node(*host, "_sub");

    std::atomic<bool> churning{true};
    std::atomic<int> cycles{0};
    std::thread churn([&] {
      while (churning.load()) {
        try {
          // Same type as /rosout, so both threads use one type cache entry.
          auto sub = helper->create_subscription<rcl_interfaces::msg::Log>("churn", 10,
                                                                           [](const rcl_interfaces::msg::Log &) {});
        } catch (const std::exception &) {
          return;  // The context is shut down.
        }
        ++cycles;
      }
    });

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (cycles.load() < 3 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::yield();
    }
    process.reset();
    churning = false;
    churn.join();
    ASSERT_GE(cycles.load(), 3) << "round " << round;
  }
}
