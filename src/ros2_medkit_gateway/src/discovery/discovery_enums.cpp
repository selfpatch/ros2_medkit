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

#include "ros2_medkit_gateway/discovery/discovery_enums.hpp"

namespace ros2_medkit_gateway {

DiscoveryMode parse_discovery_mode(const std::string & str) {
  if (str == "manifest_only") {
    return DiscoveryMode::MANIFEST_ONLY;
  }
  if (str == "hybrid") {
    return DiscoveryMode::HYBRID;
  }
  return DiscoveryMode::RUNTIME_ONLY;
}

std::string discovery_mode_to_string(DiscoveryMode mode) {
  switch (mode) {
    case DiscoveryMode::MANIFEST_ONLY:
      return "manifest_only";
    case DiscoveryMode::HYBRID:
      return "hybrid";
    default:
      return "runtime_only";
  }
}

ComponentGroupingStrategy parse_grouping_strategy(const std::string & str) {
  if (str == "namespace") {
    return ComponentGroupingStrategy::NAMESPACE;
  }
  return ComponentGroupingStrategy::NONE;
}

std::string grouping_strategy_to_string(ComponentGroupingStrategy strategy) {
  switch (strategy) {
    case ComponentGroupingStrategy::NAMESPACE:
      return "namespace";
    default:
      return "none";
  }
}

TopicOnlyPolicy parse_topic_only_policy(const std::string & str) {
  if (str == "ignore") {
    return TopicOnlyPolicy::IGNORE;
  }
  if (str == "create_area_only") {
    return TopicOnlyPolicy::CREATE_AREA_ONLY;
  }
  return TopicOnlyPolicy::CREATE_COMPONENT;
}

std::string topic_only_policy_to_string(TopicOnlyPolicy policy) {
  switch (policy) {
    case TopicOnlyPolicy::IGNORE:
      return "ignore";
    case TopicOnlyPolicy::CREATE_AREA_ONLY:
      return "create_area_only";
    default:
      return "create_component";
  }
}

}  // namespace ros2_medkit_gateway
