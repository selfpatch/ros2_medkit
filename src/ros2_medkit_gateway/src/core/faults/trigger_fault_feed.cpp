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

TriggerFaultFeed::TriggerFaultFeed(FaultEventSource & source, ResourceChangeNotifier & notifier,
                                   NodeToEntityFn resolver)
  : source_(source), notifier_(notifier), resolver_(std::move(resolver)) {
  source_.subscribe([this](const fd::FaultEventRecord & event) {
    on_event(event);
  });
}

TriggerFaultFeed::~TriggerFaultFeed() {
  source_.unsubscribe();
}

void TriggerFaultFeed::set_node_to_entity_resolver(NodeToEntityFn resolver) {
  resolver_ = std::move(resolver);
}

void TriggerFaultFeed::on_event(const fd::FaultEventRecord & event) {
  nlohmann::json fault_json = fault_record_to_json(event.fault);
  fault_json["event_type"] = event.event_type;

  // The resolver maps a source to a manifest entity id. Without a mapping the last path segment is the entity id.
  std::string entity_id;
  if (!event.fault.reporting_sources.empty()) {
    const auto & source = event.fault.reporting_sources[0];
    if (resolver_) {
      entity_id = resolver_(source);
    }
    if (entity_id.empty()) {
      const auto last_slash = source.rfind('/');
      entity_id = (last_slash != std::string::npos) ? source.substr(last_slash + 1) : source;
    }
  }
  // No entity: a catch-all match would be unintended.
  if (entity_id.empty()) {
    return;
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
