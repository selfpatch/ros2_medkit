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

#include "ros2_medkit_gateway/discovery/manifest/manifest_parser.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <sstream>

namespace ros2_medkit_gateway {
namespace discovery {

Manifest ManifestParser::parse_file(const std::string & file_path) const {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open manifest file: " + file_path);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return parse_string(buffer.str());
}

Manifest ManifestParser::parse_string(const std::string & yaml_content) const {
  YAML::Node root;
  try {
    root = YAML::Load(yaml_content);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error("YAML parse error: " + std::string(e.what()));
  }

  Manifest manifest;

  // Parse version (required)
  manifest.manifest_version = get_string(root, "manifest_version");
  if (manifest.manifest_version.empty()) {
    throw std::runtime_error("Missing required field: manifest_version");
  }

  // Parse metadata
  if (root["metadata"]) {
    manifest.metadata = parse_metadata(root["metadata"]);
  }

  // Parse discovery config
  if (root["discovery"]) {
    manifest.config = parse_config(root["discovery"]);
  }

  // Parse areas (with recursive subareas)
  if (root["areas"] && root["areas"].IsSequence()) {
    for (const auto & node : root["areas"]) {
      parse_area_recursive(node, "", manifest.areas);
    }
  }

  // Parse components
  if (root["components"] && root["components"].IsSequence()) {
    for (const auto & node : root["components"]) {
      manifest.components.push_back(parse_component(node));
    }
  }

  // Parse apps
  if (root["apps"] && root["apps"].IsSequence()) {
    for (const auto & node : root["apps"]) {
      manifest.apps.push_back(parse_app(node));
    }
  }

  // Parse functions
  if (root["functions"] && root["functions"].IsSequence()) {
    for (const auto & node : root["functions"]) {
      manifest.functions.push_back(parse_function(node));
    }
  }

  // Parse capabilities (optional map)
  if (root["capabilities"] && root["capabilities"].IsMap()) {
    for (const auto & it : root["capabilities"]) {
      std::string entity_id = it.first.as<std::string>();
      // Store as empty JSON object for now
      // Full YAML->JSON conversion can be added if needed
      manifest.capabilities[entity_id] = json::object();
    }
  }

  return manifest;
}

ManifestMetadata ManifestParser::parse_metadata(const YAML::Node & node) const {
  ManifestMetadata meta;
  meta.name = get_string(node, "name");
  meta.description = get_string(node, "description");
  meta.version = get_string(node, "version");
  meta.created_at = get_string(node, "created_at");
  return meta;
}

ManifestConfig ManifestParser::parse_config(const YAML::Node & node) const {
  ManifestConfig config;

  std::string policy = get_string(node, "unmanifested_nodes", "warn");
  config.unmanifested_nodes = ManifestConfig::parse_policy(policy);

  if (node["inherit_runtime_resources"]) {
    config.inherit_runtime_resources = node["inherit_runtime_resources"].as<bool>();
  }
  if (node["allow_manifest_override"]) {
    config.allow_manifest_override = node["allow_manifest_override"].as<bool>();
  }

  return config;
}

void ManifestParser::parse_area_recursive(const YAML::Node & node, const std::string & parent_id,
                                          std::vector<Area> & areas) const {
  Area area;
  area.id = get_string(node, "id");
  area.name = get_string(node, "name", area.id);  // Default to id if no name
  area.namespace_path = get_string(node, "namespace", "/" + area.id);
  area.translation_id = get_string(node, "translation_id");
  area.description = get_string(node, "description");
  area.tags = get_string_vector(node, "tags");
  // Set parent from recursive call, or from explicit parent_area field
  area.parent_area_id = parent_id.empty() ? get_string(node, "parent_area") : parent_id;

  areas.push_back(area);

  // Recursively parse nested subareas
  if (node["subareas"] && node["subareas"].IsSequence()) {
    for (const auto & subarea_node : node["subareas"]) {
      parse_area_recursive(subarea_node, area.id, areas);
    }
  }
}

Component ManifestParser::parse_component(const YAML::Node & node) const {
  Component comp;
  comp.id = get_string(node, "id");
  comp.name = get_string(node, "name", comp.id);
  comp.namespace_path = get_string(node, "namespace");
  comp.area = get_string(node, "area");
  comp.translation_id = get_string(node, "translation_id");
  comp.description = get_string(node, "description");
  comp.variant = get_string(node, "variant");
  comp.tags = get_string_vector(node, "tags");
  comp.parent_component_id = get_string(node, "parent_component_id");
  comp.depends_on = get_string_vector(node, "depends_on");
  comp.source = "manifest";

  // Parse type if provided (e.g., "controller", "sensor", "actuator")
  std::string type_val = get_string(node, "type");
  if (!type_val.empty()) {
    comp.type = type_val;
  }

  // Compute FQN if namespace and id are provided
  if (!comp.namespace_path.empty()) {
    comp.fqn = comp.namespace_path + "/" + comp.id;
  } else {
    comp.fqn = "/" + comp.id;
  }

  return comp;
}

App ManifestParser::parse_app(const YAML::Node & node) const {
  App app;
  app.id = get_string(node, "id");
  app.name = get_string(node, "name", app.id);
  app.translation_id = get_string(node, "translation_id");
  app.description = get_string(node, "description");
  app.component_id = get_string(node, "is_located_on");
  app.depends_on = get_string_vector(node, "depends_on");
  app.tags = get_string_vector(node, "tags");
  app.external = node["external"] ? node["external"].as<bool>() : false;
  app.source = "manifest";

  // Parse ros_binding
  if (node["ros_binding"]) {
    App::RosBinding binding;
    binding.node_name = get_string(node["ros_binding"], "node_name");
    binding.namespace_pattern = get_string(node["ros_binding"], "namespace", "*");
    binding.topic_namespace = get_string(node["ros_binding"], "topic_namespace");
    app.ros_binding = binding;
  }

  return app;
}

Function ManifestParser::parse_function(const YAML::Node & node) const {
  Function func;
  func.id = get_string(node, "id");
  func.name = get_string(node, "name", func.id);
  func.translation_id = get_string(node, "translation_id");
  func.description = get_string(node, "description");
  // Support both "hosted_by" (manifest) and "hosts" (internal)
  func.hosts = get_string_vector(node, "hosted_by");
  if (func.hosts.empty()) {
    func.hosts = get_string_vector(node, "hosts");
  }
  func.depends_on = get_string_vector(node, "depends_on");
  func.tags = get_string_vector(node, "tags");
  func.source = "manifest";

  return func;
}

std::string ManifestParser::get_string(const YAML::Node & node, const std::string & key,
                                       const std::string & default_val) const {
  if (node[key]) {
    return node[key].as<std::string>();
  }
  return default_val;
}

std::vector<std::string> ManifestParser::get_string_vector(const YAML::Node & node, const std::string & key) const {
  std::vector<std::string> result;
  if (node[key] && node[key].IsSequence()) {
    for (const auto & item : node[key]) {
      result.push_back(item.as<std::string>());
    }
  }
  return result;
}

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
    default:
      return "warn";
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
