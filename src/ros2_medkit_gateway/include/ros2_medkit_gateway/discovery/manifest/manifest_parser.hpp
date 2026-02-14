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

#include <string>
#include <vector>

// Forward declare YAML::Node to avoid including yaml-cpp in header
namespace YAML {
class Node;
}

namespace ros2_medkit_gateway {
namespace discovery {

/**
 * @brief Parses manifest YAML files into Manifest structure
 *
 * The parser handles YAML loading and converts the document into
 * internal model structures (Area, Component, App, Function).
 */
class ManifestParser {
 public:
  /**
   * @brief Parse manifest from file
   * @param file_path Path to YAML file
   * @return Parsed manifest
   * @throws std::runtime_error if file cannot be read or parsed
   */
  Manifest parse_file(const std::string & file_path) const;

  /**
   * @brief Parse manifest from YAML string
   * @param yaml_content YAML content as string
   * @return Parsed manifest
   * @throws std::runtime_error if YAML is malformed
   */
  Manifest parse_string(const std::string & yaml_content) const;

 private:
  /// Recursively parse area and its nested subareas
  void parse_area_recursive(const YAML::Node & node, const std::string & parent_id, std::vector<Area> & areas) const;
  Component parse_component(const YAML::Node & node) const;
  App parse_app(const YAML::Node & node) const;
  Function parse_function(const YAML::Node & node) const;
  ManifestConfig parse_config(const YAML::Node & node) const;
  ManifestMetadata parse_metadata(const YAML::Node & node) const;

  /// Get string value with default
  std::string get_string(const YAML::Node & node, const std::string & key, const std::string & default_val = "") const;

  /// Get string vector
  std::vector<std::string> get_string_vector(const YAML::Node & node, const std::string & key) const;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway

