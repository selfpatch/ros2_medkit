// Copyright 2026 bburda
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

namespace ros2_medkit_gateway {

/// Outcome of one graph-check tick's debounce decision.
struct GraphRefreshDecision {
  bool refresh;  ///< run the (expensive) refresh_cache() pipeline now
  bool dirty;    ///< next value of the pending-change latch
};

/// Pure debounce decision for graph-event-driven cache refresh (issue #442).
///
/// The graph event fires many times per second under graph churn; each refresh
/// runs the full discovery pipeline, so graph changes are coalesced into at most
/// one refresh per `debounce_ms`. Kept free of ROS/clock state so the coalescing,
/// 0 ms (disabled), and cold-start cases are unit-testable.
///
/// @param event_fired    a graph event was observed on this tick
/// @param currently_dirty a graph change is pending (latched, not yet serviced)
/// @param elapsed_ms      milliseconds since the last graph-driven refresh
/// @param debounce_ms     minimum interval between refreshes; 0 disables debouncing
///
/// Cold start: callers seed the last-refresh time to the clock epoch, so the
/// first pending change sees a large `elapsed_ms` and refreshes immediately.
inline GraphRefreshDecision decide_graph_refresh(bool event_fired, bool currently_dirty, std::int64_t elapsed_ms,
                                                 int debounce_ms) {
  const bool dirty = currently_dirty || event_fired;
  if (!dirty) {
    return {false, false};  // nothing pending
  }
  if (elapsed_ms < static_cast<std::int64_t>(debounce_ms)) {
    return {false, true};  // pending, but still inside the debounce window
  }
  return {true, false};  // service the pending change and clear the latch
}

}  // namespace ros2_medkit_gateway
