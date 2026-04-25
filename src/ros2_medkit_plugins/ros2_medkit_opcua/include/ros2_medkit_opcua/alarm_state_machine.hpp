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

#pragma once

#include <cstdint>
#include <string>

namespace ros2_medkit_gateway {

/// SOVD fault statuses surfaced by the OPC-UA AlarmCondition bridge.
///
/// PREFAILED is reserved for the threshold-polling pre-trigger path and is
/// never produced by ``AlarmStateMachine::compute`` from native event input -
/// OPC-UA Part 9 has no equivalent state.
enum class SovdAlarmStatus { Suppressed, Confirmed, Healed, Cleared };

/// Action the poller should take after running the state machine on one event.
///
/// ``ReportConfirmed`` / ``ReportHealed`` correspond to ``send_report_fault``
/// (FAILED event_type for Confirmed; PASSED-but-still-tracked for Healed).
/// ``ClearFault`` issues ``clear_fault`` against fault_manager. ``NoOp`` means
/// the event was redundant (same as last_known_status) and we suppress
/// downstream noise.
enum class AlarmAction { NoOp, ReportConfirmed, ReportHealed, ClearFault };

/// Inputs from a single AlarmConditionType event payload.
///
/// Fields populated by the trampoline from EventFilter select clauses.
/// ``branch_id_present`` is true when ``BranchId`` is non-null - per
/// Part 9 §5.5.2.12 those events refer to historical branches and must
/// never advance the live SOVD status (we route them to the fault_manager
/// event log only).
struct AlarmEventInput {
  bool enabled_state{true};
  bool active_state{false};
  bool acked_state{false};
  bool confirmed_state{false};
  /// True iff ShelvingState != Unshelved.
  bool shelved{false};
  /// True iff BranchId is non-null (historical branch event).
  bool branch_id_present{false};
};

/// Pure, side-effect-free state machine bridging OPC-UA AlarmConditionType
/// state combinations to SOVD fault lifecycle. The poller owns the
/// ``last_known_status`` per (ConditionId) and feeds it back as
/// ``prev_status`` on every event.
///
/// Decision order (first match wins) follows the design table in
/// design/index.rst and issue #386:
///
///   1. BranchId != null              -> Suppressed + NoOp (history only)
///   2. EnabledState == false         -> Suppressed (clear if was active)
///   3. ShelvingState != Unshelved    -> Suppressed (clear if was active)
///   4. ActiveState == true           -> Confirmed (idempotent re-report)
///   5. ActiveState == false          -> Healed or Cleared based on
///                                       Acked + Confirmed
///
/// Retain is intentionally NOT used here. Per Part 9 §5.5.2.10 it controls
/// visibility during ConditionRefresh bursts, not lifecycle - the poller
/// strips Retain=false events delivered between RefreshStartEvent and
/// RefreshEndEvent before invoking compute().
class AlarmStateMachine {
 public:
  struct Outcome {
    SovdAlarmStatus next_status;
    AlarmAction action;
  };

  static Outcome compute(SovdAlarmStatus prev_status, const AlarmEventInput & in) {
    // Rule 1: branch events are recorded in the fault_manager event log
    // (caller's responsibility) but never advance the primary lifecycle.
    if (in.branch_id_present) {
      return {prev_status, AlarmAction::NoOp};
    }

    const bool was_active = (prev_status == SovdAlarmStatus::Confirmed || prev_status == SovdAlarmStatus::Healed);

    // Rule 2: an alarm with EnabledState=false is administratively switched
    // off in the PLC. Treat the same as a clear.
    if (!in.enabled_state) {
      if (was_active) {
        return {SovdAlarmStatus::Cleared, AlarmAction::ClearFault};
      }
      return {SovdAlarmStatus::Suppressed, AlarmAction::NoOp};
    }

    // Rule 3: shelving is operator-driven suppression. We mirror it as a
    // soft clear in SOVD - operator who unshelves will receive a fresh
    // CONFIRMED event from the next live notification.
    if (in.shelved) {
      if (was_active) {
        return {SovdAlarmStatus::Cleared, AlarmAction::ClearFault};
      }
      return {SovdAlarmStatus::Suppressed, AlarmAction::NoOp};
    }

    // Rule 4: live alarm condition.
    if (in.active_state) {
      if (prev_status == SovdAlarmStatus::Confirmed) {
        // Same state - the underlying event still re-fires for fault_manager
        // occurrence_count tracking but does NOT trigger another Confirmed
        // report. Caller increments the count via a separate path.
        return {SovdAlarmStatus::Confirmed, AlarmAction::NoOp};
      }
      return {SovdAlarmStatus::Confirmed, AlarmAction::ReportConfirmed};
    }

    // Rule 5: ActiveState=false. Cleared only when both Acked AND Confirmed
    // have been completed by the operator; until then the alarm is latched
    // (HEALED) so the operator sees the unfinished workflow item.
    if (in.acked_state && in.confirmed_state) {
      if (prev_status == SovdAlarmStatus::Cleared || prev_status == SovdAlarmStatus::Suppressed) {
        return {SovdAlarmStatus::Cleared, AlarmAction::NoOp};
      }
      return {SovdAlarmStatus::Cleared, AlarmAction::ClearFault};
    }
    if (prev_status == SovdAlarmStatus::Healed) {
      return {SovdAlarmStatus::Healed, AlarmAction::NoOp};
    }
    return {SovdAlarmStatus::Healed, AlarmAction::ReportHealed};
  }
};

}  // namespace ros2_medkit_gateway
