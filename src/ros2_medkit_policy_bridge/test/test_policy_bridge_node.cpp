// Copyright 2026 ahmedsleem109
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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"
#include "ros2_medkit_policy_bridge/policy_bridge_node.hpp"

using ros2_medkit_msgs::srv::ReportFault;
using ros2_medkit_policy_bridge::PolicyBridgeNode;
using ros2_medkit_policy_msgs::msg::PolicyIntrospection;

namespace {

// Budgets are multiplied by MEDKIT_TEST_TIME_SCALE, as in the launch tests.
std::chrono::milliseconds scaled(std::chrono::milliseconds base) {
  const char * env = std::getenv("MEDKIT_TEST_TIME_SCALE");
  // Below 1 or unparseable is 1, as ros2_medkit_test_utils reads it.
  const double value = env ? std::atof(env) : 1.0;
  const double scale = value >= 1.0 ? value : 1.0;
  return std::chrono::milliseconds(static_cast<int64_t>(static_cast<double>(base.count()) * scale));
}

// A ReportFault server that records what it is sent, standing in for the
// FaultManager.
struct FakeFaultManager {
  struct Event {
    std::string code;
    bool failed;
    std::string source;
    uint8_t severity;
  };

  explicit FakeFaultManager(const rclcpp::Node::SharedPtr & host) : node(host) {
    start();
  }

  void start() {
    service = node->create_service<ReportFault>(
        "/fault_manager/report_fault",
        // The signature rclcpp's service callback takes.
        [this](const std::shared_ptr<ReportFault::Request> request,  // NOLINT(performance-unnecessary-value-param)
               std::shared_ptr<ReportFault::Response> response) {    // NOLINT(performance-unnecessary-value-param)
          events.push_back({request->fault_code, request->event_type == ReportFault::Request::EVENT_FAILED,
                            request->source_id, request->severity});
          response->accepted = true;
        });
  }

  void stop() {
    service.reset();
  }

  int count(const std::string & code, bool failed) const {
    int n = 0;
    for (const auto & e : events) {
      n += (e.code == code && e.failed == failed) ? 1 : 0;
    }
    return n;
  }

  rclcpp::Node::SharedPtr node;
  rclcpp::Service<ReportFault>::SharedPtr service;
  std::vector<Event> events;
};

class PolicyBridgeNodeTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  // Bridge on `topic` with the spread check raising on the first unsure
  // reading, a publisher, a host for the fake FaultManager, one executor.
  void SetUp() override {
    static int n = 0;
    topic_ = "/test_bridge_" + std::to_string(++n) + "/introspection";
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        {"introspection_topic", topic_},
        {"spread_threshold", 0.05},
        {"raise_after", int64_t{1}},
        {"retry_period_sec", 0.05},
    });
    bridge_ = std::make_shared<PolicyBridgeNode>(options);
    publisher_node_ = std::make_shared<rclcpp::Node>("test_policy_" + std::to_string(n));
    pub_ = publisher_node_->create_publisher<PolicyIntrospection>(topic_, 10);
    host_ = std::make_shared<rclcpp::Node>("test_fault_manager_host_" + std::to_string(n));
    probe_ = host_->create_client<ReportFault>("/fault_manager/report_fault");
    exec_.add_node(bridge_);
    exec_.add_node(publisher_node_);
    exec_.add_node(host_);
    ASSERT_TRUE(spin_until([this]() {
      return pub_->get_subscription_count() > 0;
    }));
  }

  void TearDown() override {
    exec_.remove_node(bridge_);
    exec_.remove_node(publisher_node_);
    exec_.remove_node(host_);
  }

  // In-process discovery and service calls take milliseconds; 3 s is ample.
  bool spin_until(const std::function<bool()> & done, std::chrono::milliseconds budget = std::chrono::seconds(3)) {
    const auto deadline = std::chrono::steady_clock::now() + scaled(budget);
    while (std::chrono::steady_clock::now() < deadline) {
      exec_.spin_some(std::chrono::milliseconds(20));
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return done();
  }

  void spin_for(std::chrono::milliseconds time) {
    spin_until(
        []() {
          return false;
        },
        time);
  }

  // Two chunks that disagree by 0.4 on every value: a spread far above 0.05.
  void publish_unsure(uint32_t episode) {
    for (uint32_t k = 0; k < 2; ++k) {
      PolicyIntrospection msg;
      msg.episode = episode;
      msg.step = 5 * k;
      msg.chunk_length = 10;
      msg.action_dim = 1;
      for (uint32_t t = 0; t < 10; ++t) {
        msg.chunk.push_back(0.01f * static_cast<float>(msg.step + t) + (k ? 0.2f : -0.2f));
      }
      pub_->publish(msg);
      spin_for(std::chrono::milliseconds(100));
    }
  }

  std::string topic_;
  std::shared_ptr<PolicyBridgeNode> bridge_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<PolicyIntrospection>::SharedPtr pub_;
  rclcpp::Node::SharedPtr host_;
  rclcpp::Client<ReportFault>::SharedPtr probe_;
  rclcpp::executors::SingleThreadedExecutor exec_;
};

}  // namespace

TEST(PolicyBridgeFaultCode, PrefixIsUpperSnake) {
  EXPECT_EQ(PolicyBridgeNode::fault_code("POLICY", "CHUNK_DISAGREEMENT"), "POLICY_CHUNK_DISAGREEMENT");
  EXPECT_EQ(PolicyBridgeNode::fault_code("left arm/act", "CHUNK_DISAGREEMENT"), "LEFT_ARM_ACT_CHUNK_DISAGREEMENT");
  EXPECT_EQ(PolicyBridgeNode::fault_code("__x--y__", "SLOW_INFERENCE"), "X_Y_SLOW_INFERENCE");
}

TEST(PolicyBridgeFaultCode, EmptyPrefixLeavesTheSuffix) {
  EXPECT_EQ(PolicyBridgeNode::fault_code("", "CHUNK_DISAGREEMENT"), "CHUNK_DISAGREEMENT");
  EXPECT_EQ(PolicyBridgeNode::fault_code("///", "CHUNK_DISAGREEMENT"), "CHUNK_DISAGREEMENT");
}

TEST(PolicyBridgeFaultCode, LongPrefixIsCutAndSuffixKept) {
  const std::string suffix = "_CHUNK_DISAGREEMENT";
  const std::string code = PolicyBridgeNode::fault_code(std::string(200, 'A'), suffix.substr(1));
  EXPECT_EQ(code.size(), 64u);
  EXPECT_EQ(code.substr(code.size() - suffix.size()), suffix);
}

TEST(PolicyBridgeToSample, CopiesEveryField) {
  PolicyIntrospection msg;
  msg.episode = 3;
  msg.step = 17;
  msg.chunk_length = 2;
  msg.action_dim = 1;
  msg.chunk = {0.5f, 0.25f};
  msg.inference_ms = 12.5f;
  auto s = PolicyBridgeNode::to_sample(msg);
  EXPECT_EQ(s.episode, 3u);
  EXPECT_EQ(s.step, 17u);
  EXPECT_EQ(s.chunk_length, 2u);
  EXPECT_EQ(s.action_dim, 1u);
  EXPECT_EQ(s.chunk, msg.chunk);
  EXPECT_FLOAT_EQ(s.inference_ms, 12.5f);
}

// A chunk over the cap is not copied; the sample keeps its size for the reason.
TEST(PolicyBridgeToSample, ChunkOverTheCapIsNotCopied) {
  PolicyIntrospection msg;
  msg.chunk_length = 10;
  msg.action_dim = 2;
  msg.chunk.assign(ros2_medkit_policy_bridge::core::kMaxChunkValues + 1, 0.0f);
  auto s = PolicyBridgeNode::to_sample(msg);
  EXPECT_TRUE(s.chunk.empty());
  EXPECT_EQ(s.dropped_chunk_values, ros2_medkit_policy_bridge::core::kMaxChunkValues + 1);
  msg.chunk.resize(20);
  s = PolicyBridgeNode::to_sample(msg);
  EXPECT_EQ(s.chunk.size(), 20u);
  EXPECT_EQ(s.dropped_chunk_values, 0u);
}

TEST(PolicyBridgeToSample, UnsetInferenceTimeIsNotMeasured) {
  PolicyIntrospection msg;  // inference_ms left at its default
  EXPECT_FALSE(ros2_medkit_policy_bridge::core::has_inference_time(PolicyBridgeNode::to_sample(msg)));
}

TEST_F(PolicyBridgeNodeTest, DefaultsAreTheDocumentedOnes) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"introspection_topic", topic_ + "_defaults"}});
  auto node = std::make_shared<PolicyBridgeNode>(options);
  const auto & c = node->config();
  EXPECT_EQ(c.spread_threshold, 0.0);
  EXPECT_EQ(c.max_inference_ms, 0.0);
  EXPECT_EQ(c.spread_history, 4u);
  EXPECT_EQ(c.spread_horizon, 20u);
  EXPECT_EQ(c.raise_after, 3u);
  EXPECT_EQ(c.clear_after, 0u);
  EXPECT_TRUE(c.clear_on_new_episode);
  EXPECT_EQ(node->chunk_disagreement_severity(), 1);
  EXPECT_EQ(node->slow_inference_severity(), 1);
  EXPECT_DOUBLE_EQ(node->sync_period_sec(), 0.1);
  EXPECT_EQ(node->chunk_disagreement_code(), "POLICY_CHUNK_DISAGREEMENT");
  EXPECT_EQ(node->slow_inference_code(), "POLICY_SLOW_INFERENCE");
  EXPECT_FALSE(node->get_parameter("fault_reporter.local_filtering.enabled").as_bool());
  // A PASSED for each code waits for the FaultManager (none runs here).
  EXPECT_EQ(node->due_reports(), 2u);
}

// Values of the wrong type are replaced by their default and values out of
// range are clamped; none of them stops the node.
TEST_F(PolicyBridgeNodeTest, WrongTypesAndRangesFallBack) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({
      {"introspection_topic", topic_ + "_bad"},
      {"spread_threshold", int64_t{1}},             // an integer is a number
      {"max_inference_ms", std::string("fast")},    // not a number: default 0
      {"raise_after", 3.0},                         // a whole double is an integer
      {"clear_after", 2.5},                         // not whole: default 0
      {"spread_history", int64_t{4294967296}},      // clamped to 64
      {"chunk_disagreement_severity", int64_t{9}},  // default 1
      {"retry_period_sec", -1.0},                   // default 0.1
      {"joint_scale", std::vector<int64_t>{1, 2}},
      {"clear_on_new_episode", std::string("yes")},
      {"code_prefix", int64_t{5}},
  });
  std::shared_ptr<PolicyBridgeNode> node;
  ASSERT_NO_THROW(node = std::make_shared<PolicyBridgeNode>(options));
  const auto & c = node->config();
  EXPECT_DOUBLE_EQ(c.spread_threshold, 1.0);
  EXPECT_EQ(c.max_inference_ms, 0.0);
  EXPECT_EQ(c.raise_after, 3u);
  EXPECT_EQ(c.clear_after, 0u);
  EXPECT_EQ(c.spread_history, 64u);
  EXPECT_EQ(node->chunk_disagreement_severity(), 1);
  EXPECT_DOUBLE_EQ(node->sync_period_sec(), 0.1);
  EXPECT_EQ(c.joint_scale, (std::vector<double>{1.0, 2.0}));
  EXPECT_TRUE(c.clear_on_new_episode);
  EXPECT_EQ(node->chunk_disagreement_code(), "POLICY_CHUNK_DISAGREEMENT");
}

// Nothing can fall back from a topic that is not a valid name: the constructor
// throws, and main() logs it and exits 1.
TEST_F(PolicyBridgeNodeTest, InvalidTopicNameThrows) {
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"introspection_topic", std::string("/policy-introspection")}});
  EXPECT_ANY_THROW({ auto node = std::make_shared<PolicyBridgeNode>(options); });
}

// The FaultManager starts after the policy raised: the raise waits and is sent
// when the service appears, with the passes of start-up before it.
TEST_F(PolicyBridgeNodeTest, FaultManagerThatAppearsLaterGetsTheRaise) {
  publish_unsure(0);
  spin_for(std::chrono::milliseconds(300));
  EXPECT_EQ(bridge_->due_reports(), 2u);  // the raise and the other code's pass

  FakeFaultManager fm(host_);
  ASSERT_TRUE(spin_until([&]() {
    return fm.count(bridge_->chunk_disagreement_code(), true) == 1 &&
           fm.count(bridge_->slow_inference_code(), false) == 1;
  })) << "events: "
      << fm.events.size();
  EXPECT_EQ(bridge_->due_reports(), 0u);
  EXPECT_EQ(fm.count(bridge_->chunk_disagreement_code(), false), 0);  // no pass of the raised code
  EXPECT_EQ(fm.events.front().source, std::string("/") + publisher_node_->get_name());
  spin_for(std::chrono::milliseconds(300));
  EXPECT_EQ(fm.count(bridge_->chunk_disagreement_code(), true), 1);  // sent once, not on every sync
}

// The FaultManager goes away and comes back (a restart, memory storage): the
// raise that is still active is sent again.
TEST_F(PolicyBridgeNodeTest, FaultManagerThatComesBackGetsTheRaiseAgain) {
  FakeFaultManager fm(host_);
  ASSERT_TRUE(spin_until([&]() {
    return fm.count(bridge_->slow_inference_code(), false) == 1;
  }));
  publish_unsure(0);
  ASSERT_TRUE(spin_until([&]() {
    return fm.count(bridge_->chunk_disagreement_code(), true) == 1;
  }));

  fm.stop();
  ASSERT_TRUE(spin_until([&]() {
    return !probe_->service_is_ready();
  }));
  spin_for(std::chrono::milliseconds(300));  // several sync periods with the service gone
  fm.start();
  // Both codes are sent again: the active raise and the other code's pass.
  ASSERT_TRUE(spin_until([&]() {
    return fm.count(bridge_->chunk_disagreement_code(), true) == 2 &&
           fm.count(bridge_->slow_inference_code(), false) == 2;
  })) << "state not sent again after the FaultManager came back";
}

// A new episode passes the raised fault.
TEST_F(PolicyBridgeNodeTest, NewEpisodePassesTheRaise) {
  FakeFaultManager fm(host_);
  ASSERT_TRUE(spin_until([&]() {
    return fm.count(bridge_->chunk_disagreement_code(), false) == 1;  // the start-up pass
  }));
  publish_unsure(0);
  ASSERT_TRUE(spin_until([&]() {
    return fm.count(bridge_->chunk_disagreement_code(), true) == 1;
  }));
  PolicyIntrospection msg;
  msg.episode = 1;
  msg.chunk_length = 1;
  msg.action_dim = 1;
  msg.chunk = {0.0f};
  pub_->publish(msg);
  ASSERT_TRUE(spin_until([&]() {
    return fm.count(bridge_->chunk_disagreement_code(), false) == 2;  // the start-up pass and this one
  }));
}

// source_id and the severities are taken from the parameters.
TEST_F(PolicyBridgeNodeTest, SourceIdAndSeverityAreTheConfiguredOnes) {
  rclcpp::NodeOptions options;
  const std::string topic = topic_ + "_configured";
  options.parameter_overrides({
      {"introspection_topic", topic},
      {"source_id", std::string("/arm")},
      {"code_prefix", std::string("ARM")},
      {"spread_threshold", 0.05},
      {"raise_after", int64_t{1}},
      {"chunk_disagreement_severity", int64_t{2}},
      {"retry_period_sec", 0.05},
  });
  auto bridge = std::make_shared<PolicyBridgeNode>(options);
  auto pub = publisher_node_->create_publisher<PolicyIntrospection>(topic, 10);
  exec_.add_node(bridge);
  FakeFaultManager fm(host_);
  ASSERT_TRUE(spin_until([&]() {
    return pub->get_subscription_count() > 0 && fm.count("ARM_CHUNK_DISAGREEMENT", false) == 1;
  }));
  for (uint32_t k = 0; k < 2; ++k) {
    PolicyIntrospection msg;
    msg.step = 5 * k;
    msg.chunk_length = 10;
    msg.action_dim = 1;
    for (uint32_t t = 0; t < 10; ++t) {
      msg.chunk.push_back(0.01f * static_cast<float>(msg.step + t) + (k ? 0.2f : -0.2f));
    }
    pub->publish(msg);
    spin_for(std::chrono::milliseconds(100));
  }
  ASSERT_TRUE(spin_until([&]() {
    return fm.count("ARM_CHUNK_DISAGREEMENT", true) == 1;
  }));
  for (const auto & e : fm.events) {
    if (e.code == "ARM_CHUNK_DISAGREEMENT" && e.failed) {
      EXPECT_EQ(e.source, "/arm");
      EXPECT_EQ(e.severity, 2);
    }
  }
  exec_.remove_node(bridge);
}
