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

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_fault_detection/fault_detection.hpp"
#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"

namespace ros2_medkit_gateway {

/// Turns the poller's cached point values into fault raise/clear edges.
///
/// Holds everything the polled detection path keeps between passes: the
/// raise/clear tracker, the raise qualifier of each point, and which values
/// are new since the last pass. No OPC UA session and no locking: the poller
/// calls it under its snapshot mutex, and tests drive it directly.
///
/// What counts as a new sample depends on the transport:
///
///  * Polling reads every point each cycle. A value that did not arrive this
///    cycle is left over from a failed read and is skipped, so a dead sensor's
///    last reading neither counts nor completes a wait, and the point keeps its
///    state.
///  * A subscription reports changes only, so a cached value is the current
///    one. Each data change is a sample, and while a value holds, each timer
///    pass (``Pass::kTick``, once per poll interval) is one more. A pass caused
///    by another node's change does not count for this point.
///
/// Points without a raise qualifier raise and clear exactly as before.
class DetectionPipeline {
 public:
  using Clock = ros2_medkit::fault_detection::RaiseQualifierFilter::Clock;

  /// Why ``evaluate`` runs.
  enum class Pass {
    kValues,  // new values arrived (a poll cycle or a data change)
    kTick,    // the poll loop's timer, only under a subscription
  };

  struct Change {
    std::string entity_id;
    ros2_medkit::fault_detection::FaultSignal signal;
  };

  /// A good value of ``node_id`` arrived.
  void record_sample(const std::string & node_id);

  /// Evaluate every entry against ``values`` and return the raise/clear edges.
  /// ``alarms`` is updated with each fault's current state as the fault manager
  /// sees it, so a fault still waiting to qualify reads as not active.
  std::vector<Change> evaluate(const std::vector<const NodeMapEntry *> & entries,
                               const std::unordered_map<std::string, OpcuaValue> & values, bool subscribed, Pass pass,
                               Clock::time_point now, std::unordered_map<std::string, bool> & alarms);

  /// The link is down: nothing is observed until it is back, so faults that
  /// are still waiting start over. Raised faults stay raised.
  void link_lost();

  /// True while a fault waits to qualify, so a subscription needs timer passes.
  bool has_pending() const;

  /// Translate an OPC UA value into the evaluator's value type.
  static ros2_medkit::fault_detection::Value to_fd_value(const OpcuaValue & v);

 private:
  ros2_medkit::fault_detection::FaultTransitionTracker tracker_;
  ros2_medkit::fault_detection::RaiseQualifierFilter qualifier_;
  // Good values received per node, and the count each node had when it was
  // last evaluated. ``evaluated_`` is rebuilt every pass from the entries
  // evaluated, so it never holds more than the current detection entries.
  std::unordered_map<std::string, std::uint64_t> received_;
  std::unordered_map<std::string, std::uint64_t> evaluated_;
};

}  // namespace ros2_medkit_gateway
