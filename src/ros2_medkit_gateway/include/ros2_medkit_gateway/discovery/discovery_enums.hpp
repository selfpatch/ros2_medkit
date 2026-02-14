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

#include <string>

namespace ros2_medkit_gateway {

/**
 * @brief Discovery mode determining which strategy is used
 */
enum class DiscoveryMode {
  RUNTIME_ONLY,   ///< Traditional ROS graph introspection only
  MANIFEST_ONLY,  ///< Only expose manifest-declared entities
  HYBRID          ///< Manifest as source of truth + runtime linking
};

/**
 * @brief Parse DiscoveryMode from string
 * @param str Mode string: "runtime_only", "manifest_only", or "hybrid"
 * @return Parsed mode (defaults to RUNTIME_ONLY)
 */
DiscoveryMode parse_discovery_mode(const std::string & str);

/**
 * @brief Convert DiscoveryMode to string
 * @param mode Discovery mode
 * @return String representation
 */
std::string discovery_mode_to_string(DiscoveryMode mode);

/**
 * @brief Strategy for grouping nodes into synthetic components
 */
enum class ComponentGroupingStrategy {
  NONE,       ///< Each node = 1 component (current behavior)
  NAMESPACE,  ///< Group by first namespace segment (area)
  // PROCESS  // Group by OS process (future - requires R&D)
};

/**
 * @brief Parse ComponentGroupingStrategy from string
 * @param str Strategy string: "none" or "namespace"
 * @return Parsed strategy (defaults to NONE)
 */
ComponentGroupingStrategy parse_grouping_strategy(const std::string & str);

/**
 * @brief Convert ComponentGroupingStrategy to string
 * @param strategy Grouping strategy
 * @return String representation
 */
std::string grouping_strategy_to_string(ComponentGroupingStrategy strategy);

/**
 * @brief Policy for handling namespaces with topics but no nodes
 *
 * When topics exist in a namespace without any ROS 2 nodes (common with
 * Isaac Sim, external bridges), this policy determines what entities to create.
 */
enum class TopicOnlyPolicy {
  IGNORE,            ///< Don't create any entity for topic-only namespaces
  CREATE_COMPONENT,  ///< Create component with source="topic" (default)
  CREATE_AREA_ONLY   ///< Only create the area, no component
};

/**
 * @brief Parse TopicOnlyPolicy from string
 * @param str Policy string: "ignore", "create_component", or "create_area_only"
 * @return Parsed policy (defaults to CREATE_COMPONENT)
 */
TopicOnlyPolicy parse_topic_only_policy(const std::string & str);

/**
 * @brief Convert TopicOnlyPolicy to string
 * @param policy Topic-only policy
 * @return String representation
 */
std::string topic_only_policy_to_string(TopicOnlyPolicy policy);

}  // namespace ros2_medkit_gateway

