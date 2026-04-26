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

#include "ros2_medkit_opcua/alarm_state_machine.hpp"

#include <gtest/gtest.h>

namespace ros2_medkit_gateway {

namespace {

AlarmEventInput live_event(bool active) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = active;
  return in;
}

}  // namespace

// -- Rule 1: branch events ---------------------------------------------------

TEST(AlarmStateMachineTest, BranchEventNeverAdvancesStatus) {
  AlarmEventInput in = live_event(true);
  in.branch_id_present = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Cleared, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, BranchEventDoesNotClearActiveAlarm) {
  // A branch event is informational only; it must NOT clear an active fault
  // even if its ActiveState=false bit would have triggered HEALED otherwise.
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.branch_id_present = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Confirmed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

// -- Rule 2: EnabledState=false ---------------------------------------------

TEST(AlarmStateMachineTest, DisabledClearsActiveAlarm) {
  AlarmEventInput in;
  in.enabled_state = false;
  in.active_state = true;  // ignored once disabled
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::ClearFault);
}

TEST(AlarmStateMachineTest, DisabledNoOpWhenAlreadySuppressed) {
  AlarmEventInput in;
  in.enabled_state = false;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Suppressed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Suppressed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

// -- Rule 3: Shelving --------------------------------------------------------

TEST(AlarmStateMachineTest, ShelvedClearsActiveAlarm) {
  AlarmEventInput in = live_event(true);
  in.shelved = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::ClearFault);
}

TEST(AlarmStateMachineTest, ShelvedNoOpWhenAlreadySuppressed) {
  AlarmEventInput in = live_event(false);
  in.shelved = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Suppressed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Suppressed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

// -- Rule 4: ActiveState=true ------------------------------------------------

TEST(AlarmStateMachineTest, ActiveAlarmReportsConfirmedFromCleared) {
  AlarmEventInput in = live_event(true);
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Cleared, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Confirmed);
  EXPECT_EQ(out.action, AlarmAction::ReportConfirmed);
}

TEST(AlarmStateMachineTest, ActiveAlarmIdempotentWhenAlreadyConfirmed) {
  AlarmEventInput in = live_event(true);
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Confirmed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, ReFireFromHealedReturnsToConfirmed) {
  AlarmEventInput in = live_event(true);
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Healed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Confirmed);
  EXPECT_EQ(out.action, AlarmAction::ReportConfirmed);
}

// -- Rule 5: ActiveState=false (the HEALED / CLEARED branch) ----------------

TEST(AlarmStateMachineTest, InactiveUnackedReportsHealed) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.acked_state = false;
  in.confirmed_state = false;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Healed);
  EXPECT_EQ(out.action, AlarmAction::ReportHealed);
}

TEST(AlarmStateMachineTest, AckedButNotConfirmedRemainsHealed) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.acked_state = true;
  in.confirmed_state = false;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Healed);
  EXPECT_EQ(out.action, AlarmAction::ReportHealed);
}

TEST(AlarmStateMachineTest, AckedAndConfirmedClearsFromConfirmed) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.acked_state = true;
  in.confirmed_state = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::ClearFault);
}

TEST(AlarmStateMachineTest, AckedAndConfirmedClearsFromHealed) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.acked_state = true;
  in.confirmed_state = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Healed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::ClearFault);
}

TEST(AlarmStateMachineTest, AckedAndConfirmedNoOpFromCleared) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.acked_state = true;
  in.confirmed_state = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Cleared, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, IdempotentHealedNoSecondReport) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Healed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Healed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

// -- Rule precedence ---------------------------------------------------------

TEST(AlarmStateMachineTest, BranchTakesPrecedenceOverDisabled) {
  AlarmEventInput in;
  in.enabled_state = false;
  in.branch_id_present = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  // Branch wins: live status untouched.
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Confirmed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, DisabledTakesPrecedenceOverShelving) {
  AlarmEventInput in;
  in.enabled_state = false;
  in.shelved = true;
  in.active_state = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Confirmed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::ClearFault);
}

TEST(AlarmStateMachineTest, ShelvedTakesPrecedenceOverActive) {
  AlarmEventInput in = live_event(true);
  in.shelved = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Cleared, in);
  // Shelving suppresses the alarm; never promotes to CONFIRMED.
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Suppressed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

// -- Coverage of remaining transition matrix cells (issue #389 follow-up).
// The above tests exercise the obvious paths; these cover the corners where
// prev_status is Healed or Suppressed and an exit / re-entry rule fires.

TEST(AlarmStateMachineTest, DisabledClearsHealedAlarm) {
  // Operator disables an already-latched (Healed) alarm: must transition to
  // Cleared with a ClearFault action so the latched fault disappears from
  // /faults instead of orphaning between HEALED and Cleared forever.
  AlarmEventInput in;
  in.enabled_state = false;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Healed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::ClearFault);
}

TEST(AlarmStateMachineTest, DisabledTransitionsClearedToSuppressedNoOp) {
  // Disabled-while-Cleared: status DOES change (Cleared -> Suppressed) but
  // no callback fires (NoOp action). Naming reflects both halves so a
  // future reader does not misread "NoOp" as "no transition".
  AlarmEventInput in;
  in.enabled_state = false;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Cleared, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Suppressed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, ShelvedClearsHealedAlarm) {
  // Same exit shape as Disabled, via the shelving rule.
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.shelved = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Healed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::ClearFault);
}

TEST(AlarmStateMachineTest, ShelvedNoOpWhenAlreadyCleared) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.shelved = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Cleared, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Suppressed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, ActiveAlarmReportsConfirmedFromSuppressed) {
  // Operator unshelves / re-enables an alarm whose underlying source is
  // still active: the next event has active=true and the state machine
  // must promote Suppressed -> Confirmed with a ReportConfirmed action.
  AlarmEventInput in = live_event(true);
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Suppressed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Confirmed);
  EXPECT_EQ(out.action, AlarmAction::ReportConfirmed);
}

TEST(AlarmStateMachineTest, BranchEventFromHealedNoOp) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.branch_id_present = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Healed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Healed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, BranchEventFromSuppressedNoOp) {
  AlarmEventInput in;
  in.enabled_state = true;
  in.branch_id_present = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Suppressed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Suppressed);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, AckedAndConfirmedNoOpFromSuppressed) {
  // Suppressed alarm receives a fully-cleared event (active=false,
  // acked=true, confirmed=true). Was already not-active per the
  // suppression; no ClearFault to issue, but next_status should track
  // to Cleared so a later re-fire re-promotes correctly. This is the
  // ``was_active=false`` branch of rule 5.
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.acked_state = true;
  in.confirmed_state = true;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Suppressed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Cleared);
  EXPECT_EQ(out.action, AlarmAction::NoOp);
}

TEST(AlarmStateMachineTest, InactiveUnackedFromSuppressedReportsHealed) {
  // Suppressed alarm sees an inactive+unacked event (operator unshelved
  // while the source had already self-cleared but wasn't acked). The
  // state machine must surface this as Healed so the operator sees the
  // pending ack/confirm workflow item rather than silently forgetting it.
  AlarmEventInput in;
  in.enabled_state = true;
  in.active_state = false;
  in.acked_state = false;
  in.confirmed_state = false;
  auto out = AlarmStateMachine::compute(SovdAlarmStatus::Suppressed, in);
  EXPECT_EQ(out.next_status, SovdAlarmStatus::Healed);
  EXPECT_EQ(out.action, AlarmAction::ReportHealed);
}

}  // namespace ros2_medkit_gateway
