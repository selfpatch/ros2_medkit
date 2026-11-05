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

#include <chrono>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
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
///    one. A data change that starts a wait is its first sample. While the wait
///    is open, a data change or a timer pass (``Pass::kTick``, once per poll
///    interval) counts as one more sample only when at least the minimum sample
///    spacing (the poll interval) passed since that fault's last counted
///    sample. So a fault gets at most one sample per poll interval, as under
///    polling, whether its value holds or keeps moving. A pass caused by another
///    node's change, and a recheck (``Pass::kRecheck``), count no sample; they
///    only let a ``raise_after_ms`` wait end on time. A node not seen since the
///    link was lost is skipped on every pass, so its old value neither starts,
///    completes nor clears anything.
///
/// Points without a raise qualifier raise and clear exactly as before.
class DetectionPipeline {
 public:
  using Clock = ros2_medkit::fault_detection::RaiseQualifierFilter::Clock;

  /// Why ``evaluate`` runs.
  enum class Pass {
    kValues,   // new values arrived (a poll cycle or a data change)
    kTick,     // the poll loop's timer, only under a subscription
    kRecheck,  // time passed between ticks, only under a subscription
  };

  struct Change {
    std::string entity_id;
    ros2_medkit::fault_detection::FaultSignal signal;
  };

  /// A good value of ``node_id`` arrived.
  void record_sample(const std::string & node_id);

  /// Under a subscription, the least time between two counted samples of one
  /// waiting fault. The poller sets it to the poll interval. Zero (the default)
  /// counts every data change and every tick.
  void set_min_sample_spacing(std::chrono::milliseconds spacing) {
    min_spacing_ = spacing;
  }

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

  /// True while a fault with a ``raise_after_ms`` rule waits to qualify, so a
  /// recheck between ticks can end it. A samples-only wait needs no recheck.
  bool has_pending_duration() const;

  /// Translate an OPC UA value into the evaluator's value type.
  static ros2_medkit::fault_detection::Value to_fd_value(const OpcuaValue & v);

 private:
  ros2_medkit::fault_detection::FaultTransitionTracker tracker_;
  ros2_medkit::fault_detection::RaiseQualifierFilter qualifier_;
  // What the pipeline knows about the samples of one node.
  struct NodeSamples {
    std::uint64_t received{0};    // good values received
    std::uint64_t evaluated{0};   // ``received`` when the node was last evaluated
    bool seen_since_link{false};  // got a sample since the link was last lost
  };
  // One entry per node the poller records a sample for, so it is bounded by
  // the node map. Updated in place on every pass.
  std::unordered_map<std::string, NodeSamples> nodes_;
  // Fault codes of entries with a ``raise_after_ms`` rule, bounded by the
  // fault codes the node map declares.
  std::unordered_set<std::string> duration_codes_;
  // Under a subscription: when each fault last counted a sample, and the least
  // time between two. Bounded by the fault codes the node map declares.
  std::unordered_map<std::string, Clock::time_point> last_counted_;
  std::chrono::milliseconds min_spacing_{0};
};

}  // namespace ros2_medkit_gateway
