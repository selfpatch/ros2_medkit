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

#include <functional>

#include "ros2_medkit_fault_detection/fault_types.hpp"

namespace ros2_medkit_gateway {

/// Push side of the fault stream: delivers each fault event the fault manager
/// publishes. Implementations must be thread-safe.
class FaultEventSource {
 public:
  using Callback = std::function<void(const ros2_medkit::fault_detection::FaultEventRecord &)>;

  FaultEventSource() = default;
  virtual ~FaultEventSource() = default;
  FaultEventSource(const FaultEventSource &) = delete;
  FaultEventSource & operator=(const FaultEventSource &) = delete;
  FaultEventSource(FaultEventSource &&) = delete;
  FaultEventSource & operator=(FaultEventSource &&) = delete;

  /// Start delivering events to `callback`; replaces any earlier callback.
  /// The callback may run on any thread.
  virtual void subscribe(Callback callback) = 0;

  /// Stop delivery. Returns only when no callback is running any more.
  virtual void unsubscribe() = 0;
};

}  // namespace ros2_medkit_gateway
