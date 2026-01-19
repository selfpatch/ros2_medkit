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

#include "ros2_medkit_gateway/discovery/manifest/manifest_validator.hpp"

#include <set>
#include <stack>

namespace ros2_medkit_gateway {
namespace discovery {

ValidationResult ManifestValidator::validate(const Manifest & manifest) const {
  ValidationResult result;

  validate_version(manifest, result);
  validate_unique_ids(manifest, result);
  validate_area_references(manifest, result);
  validate_component_references(manifest, result);
  validate_app_references(manifest, result);
  validate_function_references(manifest, result);
  validate_ros_bindings(manifest, result);
  validate_circular_dependencies(manifest, result);

  return result;
}

void ManifestValidator::validate_version(const Manifest & manifest, ValidationResult & result) const {
  if (manifest.manifest_version != "1.0") {
    result.add_error("R001", "Invalid manifest_version: '" + manifest.manifest_version + "', expected '1.0'",
                     "manifest_version");
  }
}

void ManifestValidator::validate_unique_ids(const Manifest & manifest, ValidationResult & result) const {
  std::set<std::string> seen_ids;

  // Check area IDs
  for (const auto & area : manifest.areas) {
    if (seen_ids.count(area.id)) {
      result.add_error("R002", "Duplicate area ID: " + area.id, "areas");
    }
    seen_ids.insert(area.id);
  }

  // Check component IDs
  for (const auto & comp : manifest.components) {
    if (seen_ids.count(comp.id)) {
      result.add_error("R003", "Duplicate component ID: " + comp.id, "components");
    }
    seen_ids.insert(comp.id);
  }

  // Check app IDs
  for (const auto & app : manifest.apps) {
    if (seen_ids.count(app.id)) {
      result.add_error("R004", "Duplicate app ID: " + app.id, "apps");
    }
    seen_ids.insert(app.id);
  }

  // Check function IDs
  for (const auto & func : manifest.functions) {
    if (seen_ids.count(func.id)) {
      result.add_error("R005", "Duplicate function ID: " + func.id, "functions");
    }
    seen_ids.insert(func.id);
  }
}

void ManifestValidator::validate_area_references(const Manifest & manifest, ValidationResult & result) const {
  std::set<std::string> area_ids;
  for (const auto & area : manifest.areas) {
    area_ids.insert(area.id);
  }

  // Check parent_area references
  for (const auto & area : manifest.areas) {
    if (!area.parent_area_id.empty() && !area_ids.count(area.parent_area_id)) {
      result.add_error("R006", "Area '" + area.id + "' references non-existent parent_area: " + area.parent_area_id,
                       "areas/" + area.id + "/parent_area");
    }
  }
}

void ManifestValidator::validate_component_references(const Manifest & manifest, ValidationResult & result) const {
  std::set<std::string> area_ids;
  for (const auto & area : manifest.areas) {
    area_ids.insert(area.id);
  }

  std::set<std::string> comp_ids;
  for (const auto & comp : manifest.components) {
    comp_ids.insert(comp.id);
  }

  for (const auto & comp : manifest.components) {
    // Check area reference
    if (!comp.area.empty() && !area_ids.count(comp.area)) {
      result.add_error("R006", "Component '" + comp.id + "' references non-existent area: " + comp.area,
                       "components/" + comp.id + "/area");
    }

    // Check parent_component reference
    if (!comp.parent_component_id.empty() && !comp_ids.count(comp.parent_component_id)) {
      result.add_error(
          "R006", "Component '" + comp.id + "' references non-existent parent_component: " + comp.parent_component_id,
          "components/" + comp.id + "/parent_component");
    }
  }
}

void ManifestValidator::validate_app_references(const Manifest & manifest, ValidationResult & result) const {
  std::set<std::string> comp_ids;
  for (const auto & comp : manifest.components) {
    comp_ids.insert(comp.id);
  }

  std::set<std::string> app_ids;
  for (const auto & app : manifest.apps) {
    app_ids.insert(app.id);
  }

  for (const auto & app : manifest.apps) {
    // Check component reference
    if (!app.component_id.empty() && !comp_ids.count(app.component_id)) {
      result.add_error("R007", "App '" + app.id + "' references non-existent component: " + app.component_id,
                       "apps/" + app.id + "/component");
    }

    // Check depends_on references (warning only)
    for (const auto & dep : app.depends_on) {
      if (!app_ids.count(dep)) {
        result.add_warning("R008", "App '" + app.id + "' depends on non-existent app: " + dep,
                           "apps/" + app.id + "/depends_on");
      }
    }
  }
}

void ManifestValidator::validate_function_references(const Manifest & manifest, ValidationResult & result) const {
  std::set<std::string> func_ids;
  for (const auto & func : manifest.functions) {
    func_ids.insert(func.id);
  }

  for (const auto & func : manifest.functions) {
    // Check hosts references (warning only)
    for (const auto & host : func.hosts) {
      if (!entity_exists(manifest, host)) {
        result.add_warning("R009", "Function '" + func.id + "' hosts non-existent entity: " + host,
                           "functions/" + func.id + "/hosts");
      }
    }

    // Check depends_on references
    for (const auto & dep : func.depends_on) {
      if (!func_ids.count(dep)) {
        result.add_warning("R008", "Function '" + func.id + "' depends on non-existent function: " + dep,
                           "functions/" + func.id + "/depends_on");
      }
    }
  }
}

void ManifestValidator::validate_ros_bindings(const Manifest & manifest, ValidationResult & result) const {
  std::set<std::string> seen_bindings;

  for (const auto & app : manifest.apps) {
    if (app.ros_binding.has_value() && !app.ros_binding->is_empty()) {
      std::string binding_key = app.ros_binding->node_name + "@" + app.ros_binding->namespace_pattern;

      // For topic namespace bindings
      if (!app.ros_binding->topic_namespace.empty()) {
        binding_key = "topic:" + app.ros_binding->topic_namespace;
      }

      // Only flag exact duplicates (not wildcards)
      if (seen_bindings.count(binding_key) && binding_key.find('*') == std::string::npos) {
        result.add_error("R010", "Duplicate ROS binding: " + binding_key + " (app: " + app.id + ")",
                         "apps/" + app.id + "/ros_binding");
      }
      seen_bindings.insert(binding_key);
    }
  }
}

void ManifestValidator::validate_circular_dependencies(const Manifest & manifest, ValidationResult & result) const {
  // Build dependency graph for apps
  std::unordered_map<std::string, std::vector<std::string>> app_graph;
  for (const auto & app : manifest.apps) {
    app_graph[app.id] = app.depends_on;
  }

  // Check for cycles in app dependencies
  for (const auto & app : manifest.apps) {
    if (has_cycle(app.id, app_graph)) {
      result.add_error("R011", "Circular dependency detected involving app: " + app.id,
                       "apps/" + app.id + "/depends_on");
      break;  // One cycle error is enough
    }
  }

  // Build and check function dependency graph
  std::unordered_map<std::string, std::vector<std::string>> func_graph;
  for (const auto & func : manifest.functions) {
    func_graph[func.id] = func.depends_on;
  }

  for (const auto & func : manifest.functions) {
    if (has_cycle(func.id, func_graph)) {
      result.add_error("R011", "Circular dependency detected involving function: " + func.id,
                       "functions/" + func.id + "/depends_on");
      break;
    }
  }
}

bool ManifestValidator::entity_exists(const Manifest & manifest, const std::string & id) const {
  for (const auto & a : manifest.areas) {
    if (a.id == id) {
      return true;
    }
  }
  for (const auto & c : manifest.components) {
    if (c.id == id) {
      return true;
    }
  }
  for (const auto & a : manifest.apps) {
    if (a.id == id) {
      return true;
    }
  }
  for (const auto & f : manifest.functions) {
    if (f.id == id) {
      return true;
    }
  }
  return false;
}

bool ManifestValidator::has_cycle(const std::string & start,
                                  const std::unordered_map<std::string, std::vector<std::string>> & graph) const {
  std::set<std::string> visited;
  std::set<std::string> in_stack;
  std::stack<std::string> stack;

  stack.push(start);
  in_stack.insert(start);

  while (!stack.empty()) {
    std::string current = stack.top();

    auto it = graph.find(current);
    if (it == graph.end() || visited.count(current)) {
      stack.pop();
      in_stack.erase(current);
      visited.insert(current);
      continue;
    }

    bool found_unvisited = false;
    for (const auto & dep : it->second) {
      if (in_stack.count(dep)) {
        return true;  // Cycle detected
      }
      if (!visited.count(dep) && graph.count(dep)) {
        stack.push(dep);
        in_stack.insert(dep);
        found_unvisited = true;
        break;
      }
    }

    if (!found_unvisited) {
      stack.pop();
      in_stack.erase(current);
      visited.insert(current);
    }
  }

  return false;
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
