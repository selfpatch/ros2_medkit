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

}  // namespace ros2_medkit_gateway
