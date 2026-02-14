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

#include "ros2_medkit_gateway/discovery/manifest/manifest.hpp"
#include "ros2_medkit_gateway/discovery/manifest/validation_error.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Validates manifest structure and references
 *
 * Applies validation rules R001-R011 to detect errors and warnings
 * in the manifest structure.
 */
class ManifestValidator {
 public:
  /**
   * @brief Validate a manifest
   * @param manifest The manifest to validate
   * @return Validation result with errors and warnings
   */
  ValidationResult validate(const Manifest & manifest) const;

 private:
  void validate_version(const Manifest & manifest, ValidationResult & result) const;
  void validate_unique_ids(const Manifest & manifest, ValidationResult & result) const;
  void validate_area_references(const Manifest & manifest, ValidationResult & result) const;
  void validate_component_references(const Manifest & manifest, ValidationResult & result) const;
  void validate_app_references(const Manifest & manifest, ValidationResult & result) const;
  void validate_function_references(const Manifest & manifest, ValidationResult & result) const;
  void validate_ros_bindings(const Manifest & manifest, ValidationResult & result) const;
  void validate_circular_dependencies(const Manifest & manifest, ValidationResult & result) const;

  /// Check if ID exists in any entity collection
  bool entity_exists(const Manifest & manifest, const std::string & id) const;

  /// Detect circular dependencies using DFS
  bool has_cycle(const std::string & start,
                 const std::unordered_map<std::string, std::vector<std::string>> & graph) const;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway

