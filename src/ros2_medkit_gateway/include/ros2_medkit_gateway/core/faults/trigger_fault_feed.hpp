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
/// The entity of an event is the first reporting source, mapped by the resolver when it knows the source and by its
/// last path segment otherwise. The resource path is "/" plus the fault code; the value is the fault JSON plus
/// `event_type`. A host gives it its own branch of a `FaultEventFanOut` when other consumers share the source.
class TriggerFaultFeed {
 public:
  /// Maps a reporting source to an entity id. Returns an empty string for a source it does not know.
  using NodeToEntityFn = std::function<std::string(const std::string &)>;

  /// Subscribes to `source` at once. `source` and `notifier` must outlive the feed.
  TriggerFaultFeed(FaultEventSource & source, ResourceChangeNotifier & notifier, NodeToEntityFn resolver = {});
  ~TriggerFaultFeed();
  TriggerFaultFeed(const TriggerFaultFeed &) = delete;
  TriggerFaultFeed & operator=(const TriggerFaultFeed &) = delete;
  TriggerFaultFeed(TriggerFaultFeed &&) = delete;
  TriggerFaultFeed & operator=(TriggerFaultFeed &&) = delete;

  /// Replaces the resolver. Call it before events can arrive: it is not synchronised with delivery.
  void set_node_to_entity_resolver(NodeToEntityFn resolver);

 private:
  void on_event(const ros2_medkit::fault_detection::FaultEventRecord & event);

  FaultEventSource & source_;
  ResourceChangeNotifier & notifier_;
  NodeToEntityFn resolver_;
};

}  // namespace ros2_medkit_gateway
