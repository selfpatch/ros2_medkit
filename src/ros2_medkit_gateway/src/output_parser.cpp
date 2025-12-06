// Copyright 2025 mfaferek93
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

#include "ros2_medkit_gateway/output_parser.hpp"

#include <vector>

namespace ros2_medkit_gateway {

json OutputParser::parse_yaml(const std::string & yaml_str) {
  try {
    std::vector<YAML::Node> docs = YAML::LoadAll(yaml_str);

    if (docs.empty()) {
      throw std::runtime_error("No YAML documents found in input");
    }

    // Parse only the first document (ros2 topic echo outputs single document)
    return yaml_to_json(docs[0]);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("Failed to parse YAML: " + std::string(e.what()));
  }
}

json OutputParser::yaml_to_json(const YAML::Node & node) {
  switch (node.Type()) {
    case YAML::NodeType::Null:
      return nullptr;

    case YAML::NodeType::Scalar: {
      // Try bool first (most specific)
      try {
        return node.as<bool>();
      } catch (...) {
      }

      // Try int first (preserves exact integer values)
      try {
        return node.as<int>();
      } catch (...) {
      }

      // Try double for floating-point numbers
      try {
        return node.as<double>();
      } catch (...) {
      }

      // Fallback to string
      return node.as<std::string>();
    }

    case YAML::NodeType::Sequence: {
      json arr = json::array();
      for (const auto & item : node) {
        arr.push_back(yaml_to_json(item));
      }
      return arr;
    }

    case YAML::NodeType::Map: {
      json obj = json::object();
      for (const auto & pair : node) {
        obj[pair.first.as<std::string>()] = yaml_to_json(pair.second);
      }
      return obj;
    }

    default:
      return nullptr;
  }
}

}  // namespace ros2_medkit_gateway
