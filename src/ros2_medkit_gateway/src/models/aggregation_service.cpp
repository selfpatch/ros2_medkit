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

#include "ros2_medkit_gateway/models/aggregation_service.hpp"

#include "ros2_medkit_gateway/models/entity_capabilities.hpp"

namespace ros2_medkit_gateway {

AggregationService::AggregationService(const ThreadSafeEntityCache * cache) : cache_(cache) {
}

AggregatedOperations AggregationService::get_operations(SovdEntityType type, const std::string & entity_id) const {
  switch (type) {
    case SovdEntityType::APP:
      return cache_->get_app_operations(entity_id);

    case SovdEntityType::COMPONENT:
      return cache_->get_component_operations(entity_id);

    case SovdEntityType::AREA:
      return cache_->get_area_operations(entity_id);

    case SovdEntityType::FUNCTION:
      return cache_->get_function_operations(entity_id);

    case SovdEntityType::SERVER: {
      // SERVER operations would aggregate from all entities
      // For now, return empty - can be extended later
      AggregatedOperations result;
      result.aggregation_level = "server";
      result.is_aggregated = true;
      return result;
    }

    case SovdEntityType::UNKNOWN:
    default:
      return AggregatedOperations{};
  }
}

AggregatedOperations AggregationService::get_operations_by_id(const std::string & entity_id) const {
  auto type = cache_->get_entity_type(entity_id);
  return get_operations(type, entity_id);
}

nlohmann::json AggregationService::build_x_medkit(const AggregatedOperations & result) {
  nlohmann::json x_medkit;

  x_medkit["aggregated"] = result.is_aggregated;

  if (result.is_aggregated && !result.source_ids.empty()) {
    x_medkit["aggregation_sources"] = result.source_ids;
  }

  if (!result.aggregation_level.empty()) {
    x_medkit["aggregation_level"] = result.aggregation_level;
  }

  return x_medkit;
}

bool AggregationService::supports_operations(SovdEntityType type) {
  // Use EntityCapabilities to check
  auto caps = EntityCapabilities::for_type(type);
  return caps.supports_collection(ResourceCollection::OPERATIONS);
}

bool AggregationService::should_aggregate(SovdEntityType type) {
  switch (type) {
    case SovdEntityType::COMPONENT:
    case SovdEntityType::AREA:
    case SovdEntityType::FUNCTION:
    case SovdEntityType::SERVER:
      return true;

    case SovdEntityType::APP:
    case SovdEntityType::UNKNOWN:
    default:
      return false;
  }
}

}  // namespace ros2_medkit_gateway
