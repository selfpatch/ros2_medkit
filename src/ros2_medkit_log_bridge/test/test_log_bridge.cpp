// Copyright 2026 mfaferek93, bburda
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

#include "ros2_medkit_log_bridge/log_bridge_node.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

using Fault = ros2_medkit_msgs::msg::Fault;
using ros2_medkit_log_bridge::LogBridgeNode;

namespace {
constexpr uint8_t kDebug = 10;
constexpr uint8_t kInfo = 20;
constexpr uint8_t kWarn = 30;
constexpr uint8_t kError = 40;
constexpr uint8_t kFatal = 50;
}  // namespace

class LogBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

// --- Level -> severity mapping (default floor WARN) ---

TEST_F(LogBridgeTest, MapLevel_DebugAndInfoDropped) {
  uint8_t sev = 255;
  EXPECT_FALSE(LogBridgeNode::map_level_to_severity(kDebug, kWarn, &sev));
  EXPECT_FALSE(LogBridgeNode::map_level_to_severity(kInfo, kWarn, &sev));
}

TEST_F(LogBridgeTest, MapLevel_WarnErrorFatal) {
  uint8_t sev = 255;
  ASSERT_TRUE(LogBridgeNode::map_level_to_severity(kWarn, kWarn, &sev));
  EXPECT_EQ(sev, Fault::SEVERITY_WARN);
  ASSERT_TRUE(LogBridgeNode::map_level_to_severity(kError, kWarn, &sev));
  EXPECT_EQ(sev, Fault::SEVERITY_ERROR);
  ASSERT_TRUE(LogBridgeNode::map_level_to_severity(kFatal, kWarn, &sev));
  EXPECT_EQ(sev, Fault::SEVERITY_CRITICAL);
}

TEST_F(LogBridgeTest, MapLevel_FloorRaisedToError) {
  // With floor=ERROR, WARN is dropped, ERROR/FATAL still pass.
  uint8_t sev = 255;
  EXPECT_FALSE(LogBridgeNode::map_level_to_severity(kWarn, kError, &sev));
  ASSERT_TRUE(LogBridgeNode::map_level_to_severity(kError, kError, &sev));
  EXPECT_EQ(sev, Fault::SEVERITY_ERROR);
}

TEST_F(LogBridgeTest, MapLevel_UnknownAboveFloorTreatedAsError) {
  uint8_t sev = 255;
  ASSERT_TRUE(LogBridgeNode::map_level_to_severity(45, kWarn, &sev));
  EXPECT_EQ(sev, Fault::SEVERITY_ERROR);
}

// --- Message normalization (stable template across varying numbers) ---

TEST_F(LogBridgeTest, NormalizeMessage_StripsNumbersAndPunctuation) {
  // Same logical message with different numbers must normalize identically.
  const auto a = LogBridgeNode::normalize_message("worldToMap failed: mx,my: 2200,2200");
  const auto b = LogBridgeNode::normalize_message("worldToMap failed: mx,my: 1500,1500");
  EXPECT_EQ(a, b);
  EXPECT_EQ(a, "worldtomap failed mx my");
}

TEST_F(LogBridgeTest, NormalizeMessage_CollapsesNumbersAndHex) {
  // Same device, differing only in the trailing index and the hex errno:
  // both collapse to the same template.
  const auto a = LogBridgeNode::normalize_message("Failed to open /dev/ttyACM0 (errno 0x19)");
  const auto b = LogBridgeNode::normalize_message("Failed to open /dev/ttyACM5 (errno 0x05)");
  EXPECT_EQ(a, b);
  // A genuinely different device name stays distinct (ACM vs USB are not noise).
  const auto c = LogBridgeNode::normalize_message("Failed to open /dev/ttyUSB0 (errno 0x19)");
  EXPECT_NE(a, c);
}

// --- Fault-code generation ---

TEST_F(LogBridgeTest, GenerateFaultCode_StableAndWellFormed) {
  auto node = std::make_shared<LogBridgeNode>();
  const auto c1 = node->generate_fault_code("/planner_server", "failed to create plan to (100.0, 100.0)");
  const auto c2 = node->generate_fault_code("/planner_server", "failed to create plan to (5.0, 5.0)");
  // Same node + same template -> same code regardless of the coordinates.
  EXPECT_EQ(c1, c2);
  // Prefix + node basename present, charset respected, length bounded.
  EXPECT_EQ(c1.rfind("LOG_PLANNER_SERVER_", 0), 0u);
  EXPECT_LE(c1.size(), 64u);
  for (char ch : c1) {
    const bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_';
    EXPECT_TRUE(ok) << "bad char in fault_code: " << ch;
  }
}

TEST_F(LogBridgeTest, GenerateFaultCode_DifferentMessagesDiffer) {
  auto node = std::make_shared<LogBridgeNode>();
  const auto a = node->generate_fault_code("/ctrl", "command interface not available");
  const auto b = node->generate_fault_code("/ctrl", "could not find controller");
  EXPECT_NE(a, b);
}

// --- Node eligibility (include/exclude) ---

TEST_F(LogBridgeTest, NodeEligibility_DefaultAllows) {
  auto node = std::make_shared<LogBridgeNode>();
  EXPECT_TRUE(node->node_is_eligible("/any_node"));
}

// --- source_id normalization to node FQN (entity association) ---

TEST_F(LogBridgeTest, NodeSourceId_PrependsLeadingSlash) {
  EXPECT_EQ(LogBridgeNode::node_source_id("bt_navigator"), "/bt_navigator");
  EXPECT_EQ(LogBridgeNode::node_source_id("planner_server"), "/planner_server");
}

TEST_F(LogBridgeTest, NodeSourceId_KeepsExistingSlash) {
  EXPECT_EQ(LogBridgeNode::node_source_id("/amcl"), "/amcl");
}

TEST_F(LogBridgeTest, NodeSourceId_StripsSubLoggerSuffix) {
  // A logger name with a sub-logger suffix maps to the node FQN.
  EXPECT_EQ(LogBridgeNode::node_source_id("controller_manager.resource_manager"),
            "/controller_manager");
}

TEST_F(LogBridgeTest, NodeCreation) {
  auto node = std::make_shared<LogBridgeNode>();
  EXPECT_NE(node, nullptr);
  EXPECT_STREQ(node->get_name(), "log_bridge");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
