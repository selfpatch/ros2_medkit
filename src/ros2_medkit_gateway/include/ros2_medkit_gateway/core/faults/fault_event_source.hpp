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
#include <utility>

#include "ros2_medkit_fault_detection/fault_types.hpp"

namespace ros2_medkit_gateway {

/// One subscriber's registration with a FaultEventSource. Move-only.
/// Destroying or reset()ing it ends delivery to that subscriber only and
/// returns once its callback is not running.
class FaultEventSubscription {
 public:
  FaultEventSubscription() = default;

  /// @param cancel Removes the callback and returns once it is not running.
  explicit FaultEventSubscription(std::function<void()> cancel) : cancel_(std::move(cancel)) {
  }

  ~FaultEventSubscription() {
    reset();
  }

  FaultEventSubscription(FaultEventSubscription && other) noexcept : cancel_(std::exchange(other.cancel_, nullptr)) {
  }

  /// Ends the current subscription first.
  FaultEventSubscription & operator=(FaultEventSubscription && other) noexcept {
    if (this != &other) {
      reset();
      cancel_ = std::exchange(other.cancel_, nullptr);
    }
    return *this;
  }

  FaultEventSubscription(const FaultEventSubscription &) = delete;
  FaultEventSubscription & operator=(const FaultEventSubscription &) = delete;

  /// End delivery to this subscriber. No-op on an empty handle.
  void reset() {
    if (cancel_) {
      auto cancel = std::exchange(cancel_, nullptr);
      cancel();
    }
  }

  /// True while the subscription is live.
  explicit operator bool() const {
    return static_cast<bool>(cancel_);
  }

 private:
  std::function<void()> cancel_;
};

/// Push side of the fault stream: delivers each fault event the fault manager
/// publishes to every live subscription. Implementations must be thread-safe.
/// The source must outlive every handle it returned, and a handle must not be
/// reset from inside its own callback.
class FaultEventSource {
 public:
  using Callback = std::function<void(const ros2_medkit::fault_detection::FaultEventRecord &)>;

  FaultEventSource() = default;
  virtual ~FaultEventSource() = default;
  FaultEventSource(const FaultEventSource &) = delete;
  FaultEventSource & operator=(const FaultEventSource &) = delete;
  FaultEventSource(FaultEventSource &&) = delete;
  FaultEventSource & operator=(FaultEventSource &&) = delete;

  /// Deliver events to `callback` until the returned handle ends.
  /// The callback may run on any thread.
  /// It must not throw: the exception escapes into the delivering thread and later subscribers miss the event.
  [[nodiscard]] virtual FaultEventSubscription subscribe(Callback callback) = 0;
};

}  // namespace ros2_medkit_gateway
