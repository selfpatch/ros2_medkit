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

/// Internal lifecycle states the AlarmCondition bridge tracks per condition.
///
/// **Public-facing vs internal**: not every value here surfaces in the SOVD
/// ``/faults`` payload (bburda review on PR #387):
/// - ``Suppressed``: condition exists but is administratively masked
///   (``EnabledState=false`` or shelved); no entry in ``/faults``.
/// - ``Confirmed``: visible as ``status=CONFIRMED`` in ``/faults``.
/// - ``Cleared``: removed from ``/faults`` via ``clear_fault``.
/// - ``Healed``: **internal-only**. Means "ActiveState=false but operator
///   workflow incomplete (ack and/or confirm pending)". This is NOT exposed
///   as a separate ``/faults`` status - the bridge keeps the entry at
///   ``CONFIRMED`` until ``Cleared`` fires (see ``ReportHealed`` below).
///   ``ros2_medkit_msgs/srv/ReportFault`` has no HEALED verb today; routing
///   ``EVENT_PASSED`` here would let ``fault_manager``'s debounce engine
///   auto-clear the fault, defeating Part 9's mandatory ack/confirm
///   contract. A future ``STATUS_LATCHED`` extension to the message would
///   make this state externally visible; until then it stays internal.
///
/// PREFAILED is reserved for the threshold-polling pre-trigger path and is
/// never produced by ``AlarmStateMachine::compute`` from native event input -
/// OPC-UA Part 9 has no equivalent state.
enum class SovdAlarmStatus { Suppressed, Confirmed, Healed, Cleared };

/// Action the poller should take after running the state machine on one event.
///
/// ``ReportConfirmed`` triggers ``send_report_fault`` (FAILED event_type).
/// ``ClearFault`` issues ``clear_fault`` against fault_manager.
/// ``ReportHealed`` is currently a no-op in ``OpcuaPlugin::on_event_alarm`` -
/// see ``SovdAlarmStatus::Healed`` above for the rationale and the future
/// ``STATUS_LATCHED`` extension that would make this surface externally.
/// ``NoOp`` means the event was redundant (same as last_known_status) and we
/// suppress downstream noise.
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
/// Retain is intentionally not modeled by this state machine and does not
/// affect ``compute()``. Per Part 9 §5.5.2.10 it controls visibility during
/// ConditionRefresh bursts rather than the lifecycle mapping implemented
/// here. The current EventFilter does not include Retain in its select
/// clauses; if/when ConditionRefresh-with-Retain filtering is added (issue
/// #389), it will live in the poller's pre-compute path, not in this
/// pure-function table. (Copilot review on PR #387.)
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
