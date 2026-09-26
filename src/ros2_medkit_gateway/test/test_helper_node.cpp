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

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
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
