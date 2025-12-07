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

#include <sstream>
#include <vector>

namespace ros2_medkit_gateway {

namespace {

/**
 * @brief Preprocess ros2 topic echo output to extract clean YAML
 *
 * ros2 topic echo can output warning messages like:
 * - "A message was lost!!!" with count info
 * - Other diagnostic messages
 *
 * This function extracts the last valid YAML document from the output.
 */
std::string preprocess_topic_echo_output(const std::string & raw_output) {
  // Split by document separator "---"
  std::vector<std::string> documents;
  std::istringstream stream(raw_output);
  std::string line;
  std::string current_doc;

  while (std::getline(stream, line)) {
    // Skip warning lines from ros2 topic echo
    if (line.find("A message was lost") != std::string::npos || line.find("total count") != std::string::npos) {
      continue;
    }

    // Document separator
    if (line == "---") {
      if (!current_doc.empty()) {
        documents.push_back(current_doc);
        current_doc.clear();
      }
      continue;
    }

    // Accumulate non-empty lines
    if (!line.empty()) {
      current_doc += line + "\n";
    }
  }

  // Don't forget the last document if there's no trailing ---
  if (!current_doc.empty()) {
    documents.push_back(current_doc);
  }

  // Return the last valid document (the actual message data)
  if (documents.empty()) {
    return "";
  }

  return documents.back();
}

}  // namespace

json OutputParser::parse_yaml(const std::string & yaml_str) {
  try {
    // Preprocess to remove warning messages and get clean YAML
    std::string clean_yaml = preprocess_topic_echo_output(yaml_str);

    if (clean_yaml.empty()) {
      throw std::runtime_error("No valid YAML content found in input");
    }

    std::vector<YAML::Node> docs = YAML::LoadAll(clean_yaml);

    if (docs.empty()) {
      throw std::runtime_error("No YAML documents found in input");
    }

    // Parse only the first document (should be the message data after preprocessing)
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
