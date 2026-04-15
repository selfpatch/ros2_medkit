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

#include <algorithm>
#include <fstream>
#include <sstream>

#include "ros2_medkit_serialization/json_serializer.hpp"

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

Manifest ManifestParser::parse_fragment_file(const std::string & file_path) const {
  // Read the yaml into a string, inject a synthetic manifest_version if the
  // fragment omits one (fragments are not required to declare it), then
  // reuse the main parse_string pipeline so every field is parsed with the
  // exact same logic as a full manifest. Anything forbidden in fragments
  // (areas, metadata, scripts, ...) is still parsed, which lets the caller
  // detect and reject it with a specific error message.
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open manifest fragment: " + file_path);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  std::string contents = buffer.str();

  // Version prefix is appended-only when the source yaml does not set one.
  // Looking for the bare word as the start of a line is sufficient for the
  // yaml dialect we accept (no flow-style top-level document).
  auto has_version_field = [](const std::string & s) {
    // Scan line-by-line for a line beginning with "manifest_version:".
    size_t pos = 0;
    while (pos < s.size()) {
      size_t end = s.find('\n', pos);
      std::string line = s.substr(pos, end == std::string::npos ? std::string::npos : end - pos);
      size_t first = line.find_first_not_of(" \t");
      if (first != std::string::npos && line.compare(first, sizeof("manifest_version:") - 1,
                                                      "manifest_version:") == 0) {
        return true;
      }
      if (end == std::string::npos) break;
      pos = end + 1;
    }
    return false;
  };
  if (!has_version_field(contents)) {
    contents = "manifest_version: \"1.0\"\n" + contents;
  }
  return parse_string(contents);
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
      auto comp = parse_component(node);
      if (node["lock"] && node["lock"].IsMap()) {
        manifest.lock_overrides[comp.id] = parse_lock_config(node["lock"]);
      }
      manifest.components.push_back(std::move(comp));
    }
  }

  // Parse apps
  if (root["apps"] && root["apps"].IsSequence()) {
    for (const auto & node : root["apps"]) {
      auto app = parse_app(node);
      if (node["lock"] && node["lock"].IsMap()) {
        manifest.lock_overrides[app.id] = parse_lock_config(node["lock"]);
      }
      manifest.apps.push_back(std::move(app));
    }
  }

  // Parse functions
  if (root["functions"] && root["functions"].IsSequence()) {
    for (const auto & node : root["functions"]) {
      manifest.functions.push_back(parse_function(node));
    }
  }

  // Parse scripts
  if (root["scripts"] && root["scripts"].IsSequence()) {
    for (const auto & node : root["scripts"]) {
      manifest.scripts.push_back(parse_script_entry(node));
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
  area.source = "manifest";

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
    case UnmanifestedNodePolicy::WARN:
    default:
      return "warn";
  }
}

ros2_medkit_gateway::ScriptEntryConfig ManifestParser::parse_script_entry(const YAML::Node & node) const {
  ros2_medkit_gateway::ScriptEntryConfig entry;
  entry.id = get_string(node, "id");
  if (entry.id.empty()) {
    throw std::runtime_error("Script entry missing required field: id");
  }
  entry.name = get_string(node, "name", entry.id);
  entry.description = get_string(node, "description");
  entry.path = get_string(node, "path");
  if (entry.path.empty()) {
    throw std::runtime_error("Script entry '" + entry.id + "' missing required field: path");
  }
  entry.format = get_string(node, "format");
  if (entry.format.empty()) {
    throw std::runtime_error("Script entry '" + entry.id + "' missing required field: format");
  }
  if (entry.format != "bash" && entry.format != "python" && entry.format != "sh") {
    throw std::runtime_error("Script entry '" + entry.id + "' has unknown format: '" + entry.format +
                             "' (expected: bash, python, sh)");
  }
  if (node["timeout_sec"]) {
    entry.timeout_sec = std::max(1, node["timeout_sec"].as<int>());
  }
  entry.entity_filter = get_string_vector(node, "entity_filter");

  // Parse env map (skip non-scalar values to avoid yaml-cpp exceptions)
  if (node["env"] && node["env"].IsMap()) {
    for (const auto & it : node["env"]) {
      if (it.second.IsScalar()) {
        entry.env[it.first.as<std::string>()] = it.second.as<std::string>();
      }
    }
  }

  // Parse args (JSON array of {name, type, flag} objects)
  if (node["args"] && node["args"].IsSequence()) {
    entry.args = json::array();
    for (const auto & arg_node : node["args"]) {
      json arg_obj;
      if (arg_node["name"]) {
        arg_obj["name"] = arg_node["name"].as<std::string>();
      }
      if (arg_node["type"]) {
        arg_obj["type"] = arg_node["type"].as<std::string>();
      }
      if (arg_node["flag"]) {
        arg_obj["flag"] = arg_node["flag"].as<std::string>();
      }
      entry.args.push_back(arg_obj);
    }
  }

  // Parse parameters_schema via recursive YAML-to-JSON conversion
  if (node["parameters_schema"]) {
    auto schema = ros2_medkit_serialization::JsonSerializer::yaml_to_json(node["parameters_schema"]);
    if (!schema.is_null() && !schema.empty()) {
      entry.parameters_schema = schema;
    }
  }

  return entry;
}

ManifestLockConfig ManifestParser::parse_lock_config(const YAML::Node & node) const {
  ManifestLockConfig config;
  config.required_scopes = get_string_vector(node, "required_scopes");
  if (node["breakable"]) {
    config.breakable = node["breakable"].as<bool>();
  }
  if (node["max_expiration"]) {
    config.max_expiration = std::max(0, node["max_expiration"].as<int>());
  }
  return config;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
