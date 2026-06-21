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

// The trailing "_<8 hex>" hash segment is lowercase hex by spec; the prefix and
// node parts must be [A-Z0-9_]. Validate accordingly.
void ExpectWellFormed(const std::string & code) {
  ASSERT_GE(code.size(), 9u);  // at least "_<8hex>"
  const size_t hash_start = code.size() - 8;
  ASSERT_EQ(code[hash_start - 1], '_');
  for (size_t i = 0; i < hash_start - 1; ++i) {
    const char ch = code[i];
    const bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_';
    EXPECT_TRUE(ok) << "bad char in fault_code prefix/node: " << ch;
  }
  for (size_t i = hash_start; i < code.size(); ++i) {
    const char ch = code[i];
    const bool ok = (ch >= 'a' && ch <= 'f') || (ch >= '0' && ch <= '9');
    EXPECT_TRUE(ok) << "bad char in fault_code hash: " << ch;
  }
}
}  // namespace

class LogBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
  }
  void TearDown() override {
    rclcpp::shutdown();
  }

  // Build a node with parameter overrides (declare_parameter reads these).
  static std::shared_ptr<LogBridgeNode> make_node_with(const std::vector<rclcpp::Parameter> & overrides) {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(overrides);
    return std::make_shared<LogBridgeNode>(opts);
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
  // mx/my are two-char tokens, kept; numbers and punctuation gone.
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

TEST_F(LogBridgeTest, NormalizeMessage_DropsIsolatedSingleLetters) {
  // "host A" vs "host B" must normalize to the same template (single letters dropped).
  const auto a = LogBridgeNode::normalize_message("connection lost to host A");
  const auto b = LogBridgeNode::normalize_message("connection lost to host B");
  EXPECT_EQ(a, b);
  EXPECT_EQ(a, "connection lost to host");
}

TEST_F(LogBridgeTest, NormalizeMessage_EmptyForAllPunctOrDigits) {
  EXPECT_EQ(LogBridgeNode::normalize_message(""), "");
  EXPECT_EQ(LogBridgeNode::normalize_message("12345"), "");
  EXPECT_EQ(LogBridgeNode::normalize_message("--- 0x1f ::"), "");
}

// --- Stable hash (fixed FNV-1a spec, known constant) ---

TEST_F(LogBridgeTest, Fnv1aHex_KnownConstants) {
  // Fixed FNV-1a 32-bit, 8 lowercase hex. Constants are part of the contract.
  EXPECT_EQ(LogBridgeNode::fnv1a_hex(""), "811c9dc5");
  EXPECT_EQ(LogBridgeNode::fnv1a_hex("a"), "e40c292c");
  EXPECT_EQ(LogBridgeNode::fnv1a_hex("foobar"), "bf9cf968");
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
  ExpectWellFormed(c1);
}

TEST_F(LogBridgeTest, GenerateFaultCode_KnownConstant) {
  auto node = std::make_shared<LogBridgeNode>();
  // Pin the full code for a fixed input: hash of normalize_message("oops 42") =
  // hash of "oops" since the digits drop. fnv1a_hex("oops") asserted below.
  const std::string expected_hash = LogBridgeNode::fnv1a_hex("oops");
  EXPECT_EQ(node->generate_fault_code("/ctrl", "oops 42"), "LOG_CTRL_" + expected_hash);
}

TEST_F(LogBridgeTest, GenerateFaultCode_DifferentMessagesDiffer) {
  auto node = std::make_shared<LogBridgeNode>();
  const auto a = node->generate_fault_code("/ctrl", "command interface not available");
  const auto b = node->generate_fault_code("/ctrl", "could not find controller");
  EXPECT_NE(a, b);
}

TEST_F(LogBridgeTest, GenerateFaultCode_EmptyMessageGetsSentinel) {
  auto node = std::make_shared<LogBridgeNode>();
  // All-punct and empty messages share the _NOMSG sentinel hash, but two
  // genuinely different real messages must not collide with it.
  const auto a = node->generate_fault_code("/ctrl", "");
  const auto b = node->generate_fault_code("/ctrl", "!!!");
  const auto real = node->generate_fault_code("/ctrl", "actual failure");
  EXPECT_EQ(a, b);
  EXPECT_NE(a, real);
}

TEST_F(LogBridgeTest, GenerateFaultCode_NamespaceDisambiguates) {
  auto node = std::make_shared<LogBridgeNode>();
  // Same node name under different namespaces must not collide.
  const auto a = node->generate_fault_code("/robot1/planner_server", "no plan");
  const auto b = node->generate_fault_code("/robot2/planner_server", "no plan");
  EXPECT_NE(a, b);
  EXPECT_EQ(a.rfind("LOG_ROBOT1_PLANNER_SERVER_", 0), 0u);
  EXPECT_EQ(b.rfind("LOG_ROBOT2_PLANNER_SERVER_", 0), 0u);
}

TEST_F(LogBridgeTest, GenerateFaultCode_NeverTruncatesHash_LongNode) {
  auto node = std::make_shared<LogBridgeNode>();
  const std::string long_ns = "/" + std::string(200, 'x') + "/planner_server";
  const auto code = node->generate_fault_code(long_ns, "boom");
  EXPECT_LE(code.size(), 64u);
  ExpectWellFormed(code);
  // The full 8-hex hash survives intact at the tail.
  EXPECT_EQ(code.substr(code.size() - 8), LogBridgeNode::fnv1a_hex("boom"));
}

// --- Node eligibility (include/exclude) ---

TEST_F(LogBridgeTest, NodeEligibility_DefaultAllows) {
  auto node = std::make_shared<LogBridgeNode>();
  EXPECT_TRUE(node->node_is_eligible("/any_node"));
}

TEST_F(LogBridgeTest, NodeEligibility_ExcludeSubstring) {
  auto node = make_node_with({rclcpp::Parameter("exclude_nodes", std::vector<std::string>{"chatty"})});
  EXPECT_FALSE(node->node_is_eligible("/chatty_logger"));
  EXPECT_TRUE(node->node_is_eligible("/planner_server"));
}

TEST_F(LogBridgeTest, NodeEligibility_IncludeOnlySubstring) {
  auto node = make_node_with({rclcpp::Parameter("include_only_nodes", std::vector<std::string>{"planner"})});
  EXPECT_TRUE(node->node_is_eligible("/planner_server"));
  EXPECT_FALSE(node->node_is_eligible("/amcl"));
}

TEST_F(LogBridgeTest, NodeEligibility_ExcludesMedkitStackByDefault) {
  auto node = make_node_with({});
  // medkit's own infrastructure must not feed its own logs back as faults
  EXPECT_FALSE(node->node_is_eligible("/fault_manager"));
  EXPECT_FALSE(node->node_is_eligible("/robot1/fault_manager"));
  EXPECT_FALSE(node->node_is_eligible("/ros2_medkit_gateway"));
  EXPECT_FALSE(node->node_is_eligible("/diagnostic_bridge"));
  EXPECT_FALSE(node->node_is_eligible("/action_status_bridge"));
  // ordinary application nodes are still promoted
  EXPECT_TRUE(node->node_is_eligible("/bt_navigator"));
}

TEST_F(LogBridgeTest, NodeEligibility_MedkitStackExclusionDisablable) {
  auto node = make_node_with({rclcpp::Parameter("exclude_medkit_stack", false)});
  EXPECT_TRUE(node->node_is_eligible("/fault_manager"));
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
  EXPECT_EQ(LogBridgeNode::node_source_id("controller_manager.resource_manager"), "/controller_manager");
}

// --- Per-code forward cooldown ---

TEST_F(LogBridgeTest, Cooldown_FirstPassesThenSuppressedThenReopens) {
  auto node = std::make_shared<LogBridgeNode>();
  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  // First occurrence forwarded; same code+severity within the 5s window suppressed.
  EXPECT_TRUE(node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_ERROR, t0));
  EXPECT_FALSE(
      node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_ERROR, t0 + rclcpp::Duration::from_seconds(1.0)));
  // A different code is independent.
  EXPECT_TRUE(node->cooldown_allows("LOG_X_bbbb2222", Fault::SEVERITY_ERROR, t0 + rclcpp::Duration::from_seconds(1.0)));
  // After the window the original code reopens.
  EXPECT_TRUE(node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_ERROR, t0 + rclcpp::Duration::from_seconds(6.0)));
}

TEST_F(LogBridgeTest, Cooldown_KeyedBySeverity_EscalationNotSuppressed) {
  auto node = std::make_shared<LogBridgeNode>();
  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  // A same-code escalation to a higher severity is a distinct (code, severity)
  // and must pass even within the window.
  EXPECT_TRUE(node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_ERROR, t0));
  EXPECT_TRUE(
      node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_CRITICAL, t0 + rclcpp::Duration::from_seconds(1.0)));
  // ...but a repeat of the same (code, severity) is still bounded.
  EXPECT_FALSE(
      node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_ERROR, t0 + rclcpp::Duration::from_seconds(1.0)));
}

TEST_F(LogBridgeTest, Cooldown_ZeroDisables) {
  auto node = make_node_with({rclcpp::Parameter("report_cooldown_sec", 0.0)});
  const rclcpp::Time t0(0, 0, RCL_ROS_TIME);
  // With cooldown disabled, every occurrence passes.
  EXPECT_TRUE(node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_ERROR, t0));
  EXPECT_TRUE(node->cooldown_allows("LOG_X_aaaa1111", Fault::SEVERITY_ERROR, t0));
}

// --- Per-node reporter lifecycle (reuse + bounded LRU eviction) ---

TEST_F(LogBridgeTest, ReporterReuse_SameSourceReturnsSameInstance) {
  auto node = std::make_shared<LogBridgeNode>();
  auto * a = node->reporter_for("/planner_server");
  auto * b = node->reporter_for("/planner_server");
  EXPECT_EQ(a, b);
  EXPECT_EQ(node->tracked_reporter_count(), 1u);
  auto * c = node->reporter_for("/amcl");
  EXPECT_NE(a, c);
  EXPECT_EQ(node->tracked_reporter_count(), 2u);
}

TEST_F(LogBridgeTest, ReporterBound_LruEviction) {
  auto node = make_node_with({rclcpp::Parameter("max_tracked_nodes", 2)});
  node->reporter_for("/n1");
  node->reporter_for("/n2");
  // Touch n1 so n2 becomes least-recently-used.
  node->reporter_for("/n1");
  node->reporter_for("/n3");  // evicts n2
  EXPECT_EQ(node->tracked_reporter_count(), 2u);
  // n1 survived the eviction: fetching it again returns the cached reporter
  // without growing the count.
  auto * n1_again = node->reporter_for("/n1");
  EXPECT_NE(n1_again, nullptr);
  EXPECT_EQ(node->tracked_reporter_count(), 2u);
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
