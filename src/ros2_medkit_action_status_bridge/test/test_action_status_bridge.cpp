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
#include <cstdint>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "ros2_medkit_action_status_bridge/action_status_bridge_node.hpp"

using ros2_medkit_action_status_bridge::ActionStatusBridgeNode;
using ros2_medkit_action_status_bridge::ActionStatusBridgeTestAccess;
using GoalStatus = action_msgs::msg::GoalStatus;
using GoalStatusArray = action_msgs::msg::GoalStatusArray;
using State = ActionStatusBridgeNode::ActionState;

namespace {

// Build a GoalStatusArray from a list of (goal_byte, status) pairs. goal_byte
// seeds a distinct UUID so each goal has its own id.
GoalStatusArray make_array(const std::vector<std::pair<uint8_t, int8_t>> & goals) {
  GoalStatusArray arr;
  for (const auto & [goal_byte, status] : goals) {
    GoalStatus gs;
    gs.goal_info.goal_id.uuid[0] = goal_byte;
    gs.status = status;
    arr.status_list.push_back(gs);
  }
  return arr;
}

GoalStatusArray one_goal(uint8_t goal_byte, int8_t status) {
  return make_array({{goal_byte, status}});
}

}  // namespace

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
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/move_action/_action/status"), "/move_action");
}

TEST_F(ActionStatusBridgeTest, ActionNameFromStatusTopic_NotAStatusTopic) {
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/some/topic"), "");
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/navigate_to_pose/_action/feedback"), "");
  EXPECT_EQ(ActionStatusBridgeNode::action_name_from_status_topic("/_action/status"), "");
}

// --- server FQN resolution (placeholder handling during DDS discovery) ---

TEST_F(ActionStatusBridgeTest, ServerFqnFromEndpoint_Resolved) {
  EXPECT_EQ(ActionStatusBridgeNode::server_fqn_from_endpoint("planner_server", "/"), "/planner_server");
  EXPECT_EQ(ActionStatusBridgeNode::server_fqn_from_endpoint("planner_server", ""), "/planner_server");
  EXPECT_EQ(ActionStatusBridgeNode::server_fqn_from_endpoint("planner_server", "/nav"), "/nav/planner_server");
}

TEST_F(ActionStatusBridgeTest, ServerFqnFromEndpoint_UnresolvedReturnsEmpty) {
  // During discovery rcl reports these placeholders; they must not become a source_id.
  EXPECT_EQ(ActionStatusBridgeNode::server_fqn_from_endpoint("_NODE_NAME_UNKNOWN_", "_NODE_NAMESPACE_UNKNOWN_"), "");
  EXPECT_EQ(ActionStatusBridgeNode::server_fqn_from_endpoint("_NODE_NAME_UNKNOWN_", "/nav"), "");
  EXPECT_EQ(ActionStatusBridgeNode::server_fqn_from_endpoint("planner_server", "_NODE_NAMESPACE_UNKNOWN_"), "");
  EXPECT_EQ(ActionStatusBridgeNode::server_fqn_from_endpoint("", "/"), "");
}

// --- fault code generation ---

TEST_F(ActionStatusBridgeTest, FaultCode_Aborted_WellFormed) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  const auto code = node->fault_code_for("/navigate_to_pose", false);
  EXPECT_EQ(code, "ACTION_NAVIGATE_TO_POSE_ABORTED");
  EXPECT_LE(code.size(), 64u);
  for (char ch : code) {
    const bool ok = (ch >= 'A' && ch <= 'Z') || (ch >= '0' && ch <= '9') || ch == '_';
    EXPECT_TRUE(ok) << "bad char: " << ch;
  }
}

TEST_F(ActionStatusBridgeTest, FaultCode_Canceled_UsesCanceledSuffix) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_EQ(node->fault_code_for("/move_action", true), "ACTION_MOVE_ACTION_CANCELED");
}

TEST_F(ActionStatusBridgeTest, FaultCode_LongName_StatusSuffixSurvivesClamp) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  const std::string long_name = "/" + std::string(80, 'a');
  const auto aborted = node->fault_code_for(long_name, false);
  const auto canceled = node->fault_code_for(long_name, true);
  EXPECT_LE(aborted.size(), 64u);
  EXPECT_LE(canceled.size(), 64u);
  EXPECT_NE(aborted.find("_ABORTED"), std::string::npos) << aborted;
  EXPECT_NE(canceled.find("_CANCELED"), std::string::npos) << canceled;
  EXPECT_NE(aborted.back(), '_');
}

TEST_F(ActionStatusBridgeTest, FaultCode_SharedLongPrefix_NoCollision) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  // Two names share the first 60 chars but differ afterwards.
  const std::string base = "/" + std::string(60, 'x');
  const auto a = node->fault_code_for(base + "_alpha", false);
  const auto b = node->fault_code_for(base + "_bravo", false);
  EXPECT_NE(a, b) << a << " vs " << b;
  EXPECT_LE(a.size(), 64u);
  EXPECT_LE(b.size(), 64u);
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

// --- derive_state: net state from a whole array ---

TEST_F(ActionStatusBridgeTest, DeriveState_AbortedIsFailed) {
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(one_goal(1, GoalStatus::STATUS_ABORTED), false), State::kFailed);
}

TEST_F(ActionStatusBridgeTest, DeriveState_SucceededIsHealthy) {
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(one_goal(1, GoalStatus::STATUS_SUCCEEDED), false), State::kHealthy);
}

TEST_F(ActionStatusBridgeTest, DeriveState_NoTerminalIsUnknown) {
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(one_goal(1, GoalStatus::STATUS_EXECUTING), false), State::kUnknown);
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(GoalStatusArray{}, false), State::kUnknown);
}

TEST_F(ActionStatusBridgeTest, DeriveState_CanceledRespectsFlag) {
  const auto arr = one_goal(1, GoalStatus::STATUS_CANCELED);
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(arr, false), State::kHealthy);
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(arr, true), State::kFailed);
}

TEST_F(ActionStatusBridgeTest, DeriveState_OrderIndependent) {
  // ABORTED before and after SUCCEEDED must both yield FAILED.
  const auto a = make_array({{1, GoalStatus::STATUS_ABORTED}, {2, GoalStatus::STATUS_SUCCEEDED}});
  const auto b = make_array({{2, GoalStatus::STATUS_SUCCEEDED}, {1, GoalStatus::STATUS_ABORTED}});
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(a, false), State::kFailed);
  EXPECT_EQ(ActionStatusBridgeNode::derive_state(b, false), State::kFailed);
}

// --- apply_message: transitions only (nullptr reporter = decision-only seam) ---

TEST_F(ActionStatusBridgeTest, Apply_AbortRaisesThenSucceedHeals) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_ABORTED), nullptr), State::kFailed);
  EXPECT_EQ(node->apply_message("/nav", one_goal(2, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kHealthy);
}

TEST_F(ActionStatusBridgeTest, Apply_DedupNoDoubleRaiseOrHeal) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_ABORTED), nullptr), State::kFailed);
  // Same failed array again: no transition.
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_ABORTED), nullptr), State::kUnknown);
  EXPECT_EQ(node->apply_message("/nav", one_goal(2, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kHealthy);
  // Same healthy array again: no transition.
  EXPECT_EQ(node->apply_message("/nav", one_goal(2, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kUnknown);
}

TEST_F(ActionStatusBridgeTest, Apply_SucceededFirstNoSpuriousHeal) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  // First terminal is SUCCEEDED with no prior fault: healthy is the net state
  // but there is nothing to heal, so no failed->healthy transition fires.
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kUnknown);
}

TEST_F(ActionStatusBridgeTest, Apply_MultiGoalOneArray_OrderIndependent) {
  auto n1 = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_EQ(n1->apply_message("/nav", make_array({{1, GoalStatus::STATUS_ABORTED}, {2, GoalStatus::STATUS_SUCCEEDED}}),
                              nullptr),
            State::kFailed);
  auto n2 = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_EQ(n2->apply_message("/nav", make_array({{2, GoalStatus::STATUS_SUCCEEDED}, {1, GoalStatus::STATUS_ABORTED}}),
                              nullptr),
            State::kFailed);
}

TEST_F(ActionStatusBridgeTest, Apply_HealOnlyWhenFailingGoalAgesOut) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  // Failing goal still retained alongside a fresh success: stays failed.
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_ABORTED), nullptr), State::kFailed);
  EXPECT_EQ(node->apply_message(
                "/nav", make_array({{1, GoalStatus::STATUS_ABORTED}, {2, GoalStatus::STATUS_SUCCEEDED}}), nullptr),
            State::kUnknown);
  // Failing goal aged out, only the success remains: heals.
  EXPECT_EQ(node->apply_message("/nav", one_goal(2, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kHealthy);
}

TEST_F(ActionStatusBridgeTest, Apply_Flapping_OnlyNetStateChangesTransition) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  int transitions = 0;
  // Retries issue a fresh goal_id each attempt; the action stays failed.
  for (uint8_t g = 10; g < 14; ++g) {
    if (node->apply_message("/nav", one_goal(g, GoalStatus::STATUS_ABORTED), nullptr) != State::kUnknown) {
      ++transitions;
    }
  }
  EXPECT_EQ(transitions, 1) << "only the first abort should transition";
  EXPECT_EQ(node->apply_message("/nav", one_goal(99, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kHealthy);
}

// --- canceled-as-fault semantics ---

TEST_F(ActionStatusBridgeTest, Apply_CanceledNotFaultByDefault) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  // Default canceled_is_fault=false: a lone CANCELED is a non-failing terminal,
  // healthy with nothing to heal -> no transition.
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_CANCELED), nullptr), State::kUnknown);
}

TEST_F(ActionStatusBridgeTest, Apply_CanceledIsFault_RaisesThenHealsOnSuccess) {
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("canceled_is_fault", true);
  auto node = std::make_shared<ActionStatusBridgeNode>(opts);
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_CANCELED), nullptr), State::kFailed);
  // Canceled goal aged out, success remains -> heals.
  EXPECT_EQ(node->apply_message("/nav", one_goal(2, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kHealthy);
}

TEST_F(ActionStatusBridgeTest, Apply_HealDisabled_NoHealTransition) {
  rclcpp::NodeOptions opts;
  opts.append_parameter_override("heal_on_succeeded", false);
  auto node = std::make_shared<ActionStatusBridgeNode>(opts);
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_ABORTED), nullptr), State::kFailed);
  EXPECT_EQ(node->apply_message("/nav", one_goal(2, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kUnknown);
}

// --- per-action isolation: one action failing must not heal another ---

TEST_F(ActionStatusBridgeTest, Apply_PerActionIsolation) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_ABORTED), nullptr), State::kFailed);
  EXPECT_EQ(node->apply_message("/arm", one_goal(1, GoalStatus::STATUS_ABORTED), nullptr), State::kFailed);
  // Healing /arm leaves /nav failed.
  EXPECT_EQ(node->apply_message("/arm", one_goal(2, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kHealthy);
  EXPECT_EQ(node->apply_message("/nav", one_goal(3, GoalStatus::STATUS_SUCCEEDED), nullptr), State::kHealthy);
}

// --- reporter stickiness: created once, source fixed, never swapped ---

TEST_F(ActionStatusBridgeTest, ReporterFor_StickyCreatedOnceNeverSwapped) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  ActionStatusBridgeTestAccess access(node.get());
  // No publisher exists, so the server FQN is unresolved and the reporter falls
  // back to the action name. It must be created once and reused, never swapped:
  // reporting_sources is append-only, so a provisional source cannot be undone.
  const void * first = access.reporter_identity("/nav");
  EXPECT_NE(first, nullptr);
  EXPECT_EQ(access.reporter_identity("/nav"), first);
}

// --- deferred delivery: a fault that cannot reach the FaultManager yet (its
// service is not discovered) must be kept pending and retried, never silently
// dropped. This is the startup discovery race a latched ABORTED status hits. ---

TEST_F(ActionStatusBridgeTest, Reconcile_FaultDeferredWhenServiceUnavailable) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  ActionStatusBridgeTestAccess access(node.get());
  auto * reporter = access.reporter_for("/nav");
  ASSERT_NE(reporter, nullptr);
  // No FaultManager runs in this unit test, so the report service is not ready.
  ASSERT_FALSE(reporter->is_service_ready());

  // An ABORTED whose report cannot be delivered must NOT commit the reported
  // state (otherwise a later SUCCEEDED would "heal" a fault the manager never
  // received) and must stay pending so a rescan can retry it once the service
  // appears.
  EXPECT_EQ(node->apply_message("/nav", one_goal(1, GoalStatus::STATUS_ABORTED), reporter), State::kUnknown);
  EXPECT_FALSE(access.reported_failed("/nav"));  // not falsely recorded as delivered
  EXPECT_TRUE(access.pending_failed("/nav"));    // remembered for retry, not dropped
}

// --- rescan add + prune ---

TEST_F(ActionStatusBridgeTest, RescanPrune_DropsVanishedAction) {
  auto node = std::make_shared<ActionStatusBridgeNode>();
  ActionStatusBridgeTestAccess access(node.get());

  access.add_watched("/nav");
  access.add_watched("/arm");
  EXPECT_TRUE(access.is_watched("/nav"));
  EXPECT_TRUE(access.is_watched("/arm"));
  EXPECT_TRUE(access.has_state("/arm"));

  // /arm vanished from the graph; /nav remains.
  access.prune_to({"/nav"});
  EXPECT_TRUE(access.is_watched("/nav"));
  EXPECT_TRUE(access.has_state("/nav"));
  EXPECT_FALSE(access.is_watched("/arm"));
  EXPECT_FALSE(access.has_state("/arm"));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
