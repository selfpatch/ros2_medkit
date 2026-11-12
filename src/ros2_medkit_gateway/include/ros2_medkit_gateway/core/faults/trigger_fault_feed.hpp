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
#include <string>

#include "ros2_medkit_gateway/core/faults/fault_event_source.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"

namespace ros2_medkit_gateway {

/// Turns the events of a `FaultEventSource` into "faults" notifications for the trigger manager.
///
/// Which triggers a change reaches is decided from the record's owner (its first reporting source) by the trigger
/// manager's "faults" scope function, not from the change's entity id. The entity id is the app the resolver maps the
/// owner to, or empty when no app owns it, and is informational. The resource path is "/" plus the fault code; the
/// value is the fault JSON plus `event_type`. An event without an owner is dropped. The feed holds one subscription on
/// the source, so other consumers can subscribe to the same source.
class TriggerFaultFeed {
 public:
  /// Maps a record's owner (its reporting source) to the id of the app whose scope holds it, or "" when no app does.
  using FaultOwnerFn = std::function<std::string(const std::string & owner)>;

  /// Subscribes to `source` at once. `source` and `notifier` must outlive the feed.
  /// Destroying the feed ends its subscription and waits for a running callback.
  TriggerFaultFeed(FaultEventSource & source, ResourceChangeNotifier & notifier, FaultOwnerFn resolver = {});
  ~TriggerFaultFeed();
  TriggerFaultFeed(const TriggerFaultFeed &) = delete;
  TriggerFaultFeed & operator=(const TriggerFaultFeed &) = delete;
  TriggerFaultFeed(TriggerFaultFeed &&) = delete;
  TriggerFaultFeed & operator=(TriggerFaultFeed &&) = delete;

  /// Replaces the resolver. Call it before events can arrive: it is not synchronised with delivery.
  void set_fault_owner_resolver(FaultOwnerFn resolver);

 private:
  void on_event(const ros2_medkit::fault_detection::FaultEventRecord & event);

  ResourceChangeNotifier & notifier_;
  FaultOwnerFn resolver_;
  FaultEventSubscription subscription_;
};

}  // namespace ros2_medkit_gateway
