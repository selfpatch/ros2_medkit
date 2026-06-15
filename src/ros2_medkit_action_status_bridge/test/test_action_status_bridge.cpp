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

#include <array>

#include "ros2_medkit_action_status_bridge/action_status_bridge_node.hpp"

using ros2_medkit_action_status_bridge::ActionStatusBridgeNode;

class ActionStatusBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }
};

// --- action name extraction from status topic ---

TEST_F(ActionStatusBridgeTest, ActionNameFromStatusTopic_Valid) {
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/navigate_to_pose/_action/status"),
            "/navigate_to_pose");
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/move_action/_action/status"),
            "/move_action");
}

TEST_F(ActionStatusBridgeTest, ActionNameFromStatusTopic_NotAStatusTopic) {
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/some/topic"), "");
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/navigate_to_pose/_action/feedback"), "");
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/_action/status"), "");
}

// --- fault code generation ---

TEST_F(ActionStatusBridgeTest, AbortedFaultCode_WellFormed) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  const auto code = node->aborted_fault_code("/navigate_to_pose");
  EXPECT_EQ(code, "ACTION_NAVIGATE_TO_POSE_ABORTED");
  EXPECT_LE(code.size(), 64u);
  for (char ch : code) {
    const bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_';
    EXPECT_TRUE(ok) << "bad char: " << ch;
  }
}

TEST_F(ActionStatusBridgeTest, AbortedFaultCode_MoveAction) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_EQ(node->aborted_fault_code("/move_action"), "ACTION_MOVE_ACTION_ABORTED");
}

// --- uuid hex ---

TEST_F(ActionStatusBridgeTest, UuidToHex) {
  std::array<uint8_t, 16> uuid{};
  uuid[0] = 0xAB;
  uuid[1] = 0x01;
  uuid[15] = 0xFF;
  const auto hex = ActionStatusBridgeNode::uuid_to_hex(uuid);
  EXPECT_EQ(hex.size(), 32u);
  EXPECT_EQ(hex.substr(0, 4), "ab01");
  EXPECT_EQ(hex.substr(30, 2), "ff");
}

TEST_F(ActionStatusBridgeTest, NodeCreation) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_NE(node, nullptr);
  EXPECT_STREQ(node->get_name(), "action_status_bridge");
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
