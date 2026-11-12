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

#include "ros2_medkit_gateway/core/faults/trigger_fault_feed.hpp"

#include <utility>

#include "ros2_medkit_gateway/core/faults/fault_json.hpp"

namespace ros2_medkit_gateway {

namespace fd = ros2_medkit::fault_detection;

TriggerFaultFeed::TriggerFaultFeed(FaultEventSource & source, ResourceChangeNotifier & notifier, FaultOwnerFn resolver)
  : notifier_(notifier)
  , resolver_(std::move(resolver))
  , subscription_(source.subscribe([this](const fd::FaultEventRecord & event) {
    on_event(event);
  })) {
}

TriggerFaultFeed::~TriggerFaultFeed() {
  subscription_.reset();
}

void TriggerFaultFeed::set_fault_owner_resolver(FaultOwnerFn resolver) {
  resolver_ = std::move(resolver);
}

void TriggerFaultFeed::on_event(const fd::FaultEventRecord & event) {
  // A record without an owner is in no entity's scope.
  if (event.fault.reporting_sources.empty() || event.fault.reporting_sources.front().empty()) {
    return;
  }
  const auto & owner = event.fault.reporting_sources.front();

  nlohmann::json fault_json = fault_record_to_json(event.fault);
  fault_json["event_type"] = event.event_type;

  std::string entity_id;
  if (resolver_) {
    entity_id = resolver_(owner);
  }

  // Both fault_cleared and a heal remove the fault from the active set. The payload "status" tells them apart.
  ChangeType change_type = ChangeType::UPDATED;
  if (event.event_type == fd::fault_event_type::kConfirmed) {
    change_type = ChangeType::CREATED;
  } else if (event.event_type == fd::fault_event_type::kCleared) {
    change_type = ChangeType::DELETED;
  }

  // The leading slash matches the resource_path that parse_resource_uri() gives a trigger.
  notifier_.notify("faults", entity_id, "/" + event.fault.fault_code, fault_json, change_type);
}

}  // namespace ros2_medkit_gateway
