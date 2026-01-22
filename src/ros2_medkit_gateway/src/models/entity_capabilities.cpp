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

#include "ros2_medkit_gateway/models/entity_capabilities.hpp"

namespace ros2_medkit_gateway {

EntityCapabilities EntityCapabilities::for_type(SovdEntityType type) {
  EntityCapabilities caps;

  switch (type) {
    case SovdEntityType::SERVER:
      // SERVER supports all collections
      caps.collections_ = {
          ResourceCollection::CONFIGURATIONS,
          ResourceCollection::DATA,
          ResourceCollection::FAULTS,
          ResourceCollection::OPERATIONS,
          ResourceCollection::BULK_DATA,
          ResourceCollection::DATA_LISTS,
          ResourceCollection::LOCKS,
          ResourceCollection::MODES,
          ResourceCollection::CYCLIC_SUBSCRIPTIONS,
          ResourceCollection::COMMUNICATION_LOGS,
          ResourceCollection::TRIGGERS,
          ResourceCollection::SCRIPTS,
          ResourceCollection::UPDATES,
      };
      // SERVER resources
      caps.resources_ = {"docs", "version-info", "logs", "belongs-to", "depends-on", "data-categories", "data-groups"};
      break;

    case SovdEntityType::AREA:
      // AREA does NOT support resource collections per SOVD spec
      // Only navigation/relationship resources
      caps.collections_ = {};
      caps.resources_ = {"docs", "contains", "subareas", "related-components"};
      break;

    case SovdEntityType::COMPONENT:
      // COMPONENT supports most collections
      caps.collections_ = {
          ResourceCollection::CONFIGURATIONS,
          ResourceCollection::DATA,
          ResourceCollection::FAULTS,
          ResourceCollection::OPERATIONS,
          ResourceCollection::BULK_DATA,
          ResourceCollection::DATA_LISTS,
          ResourceCollection::LOCKS,
          ResourceCollection::MODES,
          ResourceCollection::CYCLIC_SUBSCRIPTIONS,
          ResourceCollection::COMMUNICATION_LOGS,
          ResourceCollection::TRIGGERS,
          ResourceCollection::SCRIPTS,
          ResourceCollection::UPDATES,
      };
      caps.resources_ = {"docs",       "logs",          "hosts",           "belongs-to",
                         "depends-on", "subcomponents", "data-categories", "data-groups"};
      break;

    case SovdEntityType::APP:
      // APP supports most collections
      caps.collections_ = {
          ResourceCollection::CONFIGURATIONS,
          ResourceCollection::DATA,
          ResourceCollection::FAULTS,
          ResourceCollection::OPERATIONS,
          ResourceCollection::BULK_DATA,
          ResourceCollection::DATA_LISTS,
          ResourceCollection::LOCKS,
          ResourceCollection::MODES,
          ResourceCollection::CYCLIC_SUBSCRIPTIONS,
          ResourceCollection::COMMUNICATION_LOGS,
          ResourceCollection::TRIGGERS,
          ResourceCollection::SCRIPTS,
          ResourceCollection::UPDATES,
      };
      caps.resources_ = {"docs", "logs", "is-located-on", "belongs-to", "depends-on", "data-categories", "data-groups"};
      break;

    case SovdEntityType::FUNCTION:
      // FUNCTION only supports data and operations (aggregated from Apps)
      caps.collections_ = {
          ResourceCollection::DATA,
          ResourceCollection::OPERATIONS,
      };
      // Mark these as aggregated
      caps.aggregated_collections_ = {
          ResourceCollection::DATA,
          ResourceCollection::OPERATIONS,
      };
      caps.resources_ = {"docs", "hosts", "depends-on"};
      break;

    case SovdEntityType::UNKNOWN:
    default:
      // Unknown type has no capabilities
      caps.collections_ = {};
      caps.resources_ = {};
      break;
  }

  return caps;
}

bool EntityCapabilities::supports_collection(ResourceCollection col) const {
  return collections_.count(col) > 0;
}

bool EntityCapabilities::supports_resource(const std::string & name) const {
  return resources_.count(name) > 0;
}

std::vector<ResourceCollection> EntityCapabilities::collections() const {
  return std::vector<ResourceCollection>(collections_.begin(), collections_.end());
}

std::vector<std::string> EntityCapabilities::resources() const {
  return std::vector<std::string>(resources_.begin(), resources_.end());
}

bool EntityCapabilities::is_aggregated(ResourceCollection col) const {
  return aggregated_collections_.count(col) > 0;
}

}  // namespace ros2_medkit_gateway
