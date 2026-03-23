// Copyright 2026 mfaferek93
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

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "ros2_medkit_diagnostic_bridge/diagnostic_bridge_node.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
using Fault = ros2_medkit_msgs::msg::Fault;
using ros2_medkit_diagnostic_bridge::DiagnosticBridgeNode;

class DiagnosticBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }

  void TearDown() override {
    rclcpp::shutdown();
  }
};

// Test severity mapping
TEST_F(DiagnosticBridgeTest, MapToSeverity_Warn) {
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::WARN);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_WARN);
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Error) {
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::ERROR);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_ERROR);
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Stale) {
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::STALE);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_CRITICAL);
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Ok) {
  // OK should return nullopt (use is_ok_level and send PASSED instead)
  auto result = DiagnosticBridgeNode::map_to_severity(DiagStatus::OK);
  EXPECT_FALSE(result.has_value());
}

TEST_F(DiagnosticBridgeTest, MapToSeverity_Unknown) {
  // Unknown level defaults to ERROR
  auto result = DiagnosticBridgeNode::map_to_severity(99);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(*result, Fault::SEVERITY_ERROR);
}

// Test is_ok_level
TEST_F(DiagnosticBridgeTest, IsOkLevel_True) {
  EXPECT_TRUE(DiagnosticBridgeNode::is_ok_level(DiagStatus::OK));
}

TEST_F(DiagnosticBridgeTest, IsOkLevel_False) {
  EXPECT_FALSE(DiagnosticBridgeNode::is_ok_level(DiagStatus::WARN));
  EXPECT_FALSE(DiagnosticBridgeNode::is_ok_level(DiagStatus::ERROR));
  EXPECT_FALSE(DiagnosticBridgeNode::is_ok_level(DiagStatus::STALE));
}

// Node creation test
TEST_F(DiagnosticBridgeTest, NodeCreation) {
  auto node = std::make_shared<DiagnosticBridgeNode>();
  EXPECT_NE(node, nullptr);
  EXPECT_STREQ(node->get_name(), "diagnostic_bridge");
}

// Test fault code mapping with auto-generate
TEST_F(DiagnosticBridgeTest, MapToFaultCode_AutoGenerate) {
  auto node = std::make_shared<DiagnosticBridgeNode>();

  // Auto-generated codes
  EXPECT_EQ(node->map_to_fault_code("motor temp"), "MOTOR_TEMP");
  EXPECT_EQ(node->map_to_fault_code("motor_temperature"), "MOTOR_TEMPERATURE");
  EXPECT_EQ(node->map_to_fault_code("motor: Status"), "MOTOR_STATUS");
  EXPECT_EQ(node->map_to_fault_code("/robot/sensor"), "ROBOT_SENSOR");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_SpecialCharacters) {
  auto node = std::make_shared<DiagnosticBridgeNode>();

  // Multiple separators collapse to single underscore
  EXPECT_EQ(node->map_to_fault_code("motor::temp"), "MOTOR_TEMP");
  EXPECT_EQ(node->map_to_fault_code("motor  temp"), "MOTOR_TEMP");
  EXPECT_EQ(node->map_to_fault_code("motor - temp"), "MOTOR_TEMP");
}

TEST_F(DiagnosticBridgeTest, MapToFaultCode_LeadingTrailing) {
  auto node = std::make_shared<DiagnosticBridgeNode>();

  // Leading/trailing separators are removed
  EXPECT_EQ(node->map_to_fault_code("/motor"), "MOTOR");
  EXPECT_EQ(node->map_to_fault_code("motor/"), "MOTOR");
  EXPECT_EQ(node->map_to_fault_code("  motor  "), "MOTOR");
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
