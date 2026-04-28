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

#include "ros2_medkit_gateway/core/models/aggregation_service.hpp"

#include "ros2_medkit_gateway/core/models/entity_capabilities.hpp"

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

std::vector<std::string> AggregationService::get_child_app_ids(SovdEntityType type,
                                                               const std::string & entity_id) const {
  switch (type) {
    case SovdEntityType::APP:
      return {entity_id};

    case SovdEntityType::COMPONENT:
      return cache_->get_apps_for_component(entity_id);

    case SovdEntityType::FUNCTION: {
      auto func = cache_->get_function(entity_id);
      if (!func) {
        return {};
      }
      return func->hosts;
    }

    case SovdEntityType::AREA: {
      std::vector<std::string> app_ids;
      auto comp_ids = cache_->get_components_for_area(entity_id);
      for (const auto & comp_id : comp_ids) {
        auto comp_apps = cache_->get_apps_for_component(comp_id);
        app_ids.insert(app_ids.end(), comp_apps.begin(), comp_apps.end());
      }
      return app_ids;
    }

    case SovdEntityType::SERVER:
    case SovdEntityType::UNKNOWN:
    default:
      return {};
  }
}

nlohmann::json AggregationService::build_collection_x_medkit(SovdEntityType type, const std::string & entity_id) const {
  nlohmann::json x_medkit;
  bool aggregated = should_aggregate(type);
  x_medkit["aggregated"] = aggregated;

  if (aggregated) {
    auto child_ids = get_child_app_ids(type, entity_id);
    if (!child_ids.empty()) {
      x_medkit["aggregation_sources"] = child_ids;
    }
  }

  // Set aggregation_level
  switch (type) {
    case SovdEntityType::APP:
      x_medkit["aggregation_level"] = "app";
      break;
    case SovdEntityType::COMPONENT:
      x_medkit["aggregation_level"] = "component";
      break;
    case SovdEntityType::AREA:
      x_medkit["aggregation_level"] = "area";
      break;
    case SovdEntityType::FUNCTION:
      x_medkit["aggregation_level"] = "function";
      break;
    case SovdEntityType::SERVER:
      x_medkit["aggregation_level"] = "server";
      break;
    case SovdEntityType::UNKNOWN:
    default:
      break;
  }

  return x_medkit;
}

}  // namespace ros2_medkit_gateway
