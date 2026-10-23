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

#include <string>

#include "ros2_medkit_gateway/core/discovery/manifest/manifest.hpp"

namespace ros2_medkit_gateway {
namespace discovery {

// ManifestConfig helper implementations
ManifestConfig::UnmanifestedNodePolicy ManifestConfig::parse_policy(const std::string & str) {
  if (str == "ignore") {
    return UnmanifestedNodePolicy::IGNORE;
  }
  if (str == "error") {
    return UnmanifestedNodePolicy::ERROR;
  }
  if (str == "include_as_orphan") {
    return UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN;
  }
  return UnmanifestedNodePolicy::WARN;  // Default
}

std::string ManifestConfig::policy_to_string(UnmanifestedNodePolicy policy) {
  switch (policy) {
    case UnmanifestedNodePolicy::IGNORE:
      return "ignore";
    case UnmanifestedNodePolicy::ERROR:
      return "error";
    case UnmanifestedNodePolicy::INCLUDE_AS_ORPHAN:
      return "include_as_orphan";
    case UnmanifestedNodePolicy::WARN:
    default:
      return "warn";
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
