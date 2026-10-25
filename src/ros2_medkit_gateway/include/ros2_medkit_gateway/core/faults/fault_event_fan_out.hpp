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

#include <memory>
#include <mutex>
#include <vector>

#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"

namespace ros2_medkit_gateway {

/// Shares one upstream `FaultEventSource` between several consumers.
///
/// `FaultEventSource::subscribe` replaces the earlier callback, so two consumers
/// (the fault SSE stream and the freeze-frame capture, for example) cannot share
/// one source. The fan-out subscribes upstream once and forwards every event to
/// each branch that is subscribed. A host with a single source passes a branch to
/// each consumer instead of the source itself.
///
/// Thread-safe. The upstream is subscribed while at least one branch is, and
/// released when the last one leaves. A branch must not outlive the fan-out, and
/// the fan-out must not outlive the upstream. A branch callback must not subscribe
/// or unsubscribe any branch of the same fan-out.
class FaultEventFanOut {
 public:
  explicit FaultEventFanOut(FaultEventSource & upstream);
  ~FaultEventFanOut();
  FaultEventFanOut(const FaultEventFanOut &) = delete;
  FaultEventFanOut & operator=(const FaultEventFanOut &) = delete;
  FaultEventFanOut(FaultEventFanOut &&) = delete;
  FaultEventFanOut & operator=(FaultEventFanOut &&) = delete;

  /// A new source for one consumer. `subscribe` and `unsubscribe` on it affect this consumer only.
  std::unique_ptr<FaultEventSource> make_branch();

 private:
  class Branch;
  void attach(Branch * branch);
  void detach(Branch * branch);
  void deliver(const ros2_medkit::fault_detection::FaultEventRecord & event);

  FaultEventSource & upstream_;
  /// Held for a whole attach or detach, so the upstream is never subscribed and released at once.
  std::mutex upstream_mutex_;
  std::mutex mutex_;  ///< guards branches_, and is held while events are delivered
  std::vector<Branch *> branches_;
};

}  // namespace ros2_medkit_gateway
