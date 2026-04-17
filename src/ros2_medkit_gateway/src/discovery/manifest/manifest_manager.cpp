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

#include "ros2_medkit_gateway/discovery/manifest/manifest_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>

namespace ros2_medkit_gateway {
namespace discovery {

ManifestManager::ManifestManager(rclcpp::Node * node) : node_(node) {
}

bool ManifestManager::load_manifest(const std::string & file_path, bool strict) {
  std::lock_guard<std::mutex> lock(mutex_);

  strict_mode_ = strict;
  manifest_path_ = file_path;

  try {
    log_info("Loading manifest from: " + file_path);
    Manifest loaded = parser_.parse_file(file_path);

    // Apply any fragments dropped in the fragments directory on top of the
    // base manifest before validation. `apply_fragments` appends its own
    // errors to validation_result_ so they surface in the normal flow.
    validation_result_ = ValidationResult{};
    if (!apply_fragments(loaded)) {
      // apply_fragments already recorded the errors; fall through to the
      // regular validation step so the full error set is reported.
    }

    // Validate the merged manifest.
    auto merge_errors = std::move(validation_result_);
    validation_result_ = validator_.validate(loaded);
    // Preserve any fragment errors recorded above.
    for (auto & err : merge_errors.errors) {
      validation_result_.errors.push_back(std::move(err));
    }
    for (auto & warn : merge_errors.warnings) {
      validation_result_.warnings.push_back(std::move(warn));
    }

    // Always fail on ERRORs (broken references, circular deps, duplicate bindings)
    // These indicate a fundamentally broken manifest that would cause runtime issues
    if (validation_result_.has_errors()) {
      for (const auto & err : validation_result_.errors) {
        log_error("Manifest error: " + err.to_string());
      }
      log_error("Manifest validation failed: " + std::to_string(validation_result_.errors.size()) + " errors found");
      return false;
    }

    // In strict mode, also fail on warnings; otherwise just log them
    for (const auto & warn : validation_result_.warnings) {
      log_warn("Manifest warning: " + warn.to_string());
    }
    if (strict && validation_result_.has_warnings()) {
      log_error("Manifest validation failed (strict mode): warnings treated as errors");
      return false;
    }

    manifest_ = std::move(loaded);
    build_lookup_maps();

    auto msg = "Manifest loaded successfully: " + std::to_string(manifest_->areas.size()) + " areas, " +
               std::to_string(manifest_->components.size()) + " components, " + std::to_string(manifest_->apps.size()) +
               " apps, " + std::to_string(manifest_->functions.size()) + " functions";
    if (!manifest_->scripts.empty()) {
      msg += ", " + std::to_string(manifest_->scripts.size()) + " scripts";
    }
    log_info(msg);
    return true;

  } catch (const std::exception & e) {
    log_error("Failed to load manifest: " + std::string(e.what()));
    validation_result_.add_error("LOAD", e.what(), file_path);
    return false;
  }
}

bool ManifestManager::load_manifest_from_string(const std::string & yaml_content, bool strict) {
  std::lock_guard<std::mutex> lock(mutex_);

  strict_mode_ = strict;
  manifest_path_ = "<string>";

  try {
    Manifest loaded = parser_.parse_string(yaml_content);
    validation_result_ = validator_.validate(loaded);

    // Always fail on ERRORs (broken references, circular deps, duplicate bindings)
    if (validation_result_.has_errors()) {
      for (const auto & err : validation_result_.errors) {
        log_error("Manifest error: " + err.to_string());
      }
      log_error("Manifest validation failed: " + std::to_string(validation_result_.errors.size()) + " errors found");
      return false;
    }

    // In strict mode, also fail on warnings; otherwise just log them
    for (const auto & warn : validation_result_.warnings) {
      log_warn("Manifest warning: " + warn.to_string());
    }
    if (strict && validation_result_.has_warnings()) {
      log_error("Manifest validation failed (strict mode): warnings treated as errors");
      return false;
    }

    manifest_ = std::move(loaded);
    build_lookup_maps();
    return true;

  } catch (const std::exception & e) {
    log_error("Failed to parse manifest: " + std::string(e.what()));
    validation_result_.add_error("LOAD", e.what(), "<string>");
    return false;
  }
}

bool ManifestManager::reload_manifest() {
  // Note: This method acquires lock internally via load_manifest
  if (manifest_path_.empty() || manifest_path_ == "<string>") {
    log_warn("No manifest file path to reload");
    return false;
  }

  // Store current state
  std::optional<Manifest> old_manifest;
  bool old_strict_mode;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    old_manifest = std::move(manifest_);
    old_strict_mode = strict_mode_;
    manifest_.reset();
  }

  std::string path_to_reload = manifest_path_;

  if (!load_manifest(path_to_reload, old_strict_mode)) {
    // Restore old manifest on failure
    std::lock_guard<std::mutex> lock(mutex_);
    manifest_ = std::move(old_manifest);
    log_warn("Manifest reload failed, keeping previous version");
    return false;
  }

  log_info("Manifest reloaded successfully");
  return true;
}

void ManifestManager::unload_manifest() {
  std::lock_guard<std::mutex> lock(mutex_);

  manifest_.reset();
  manifest_path_.clear();
  validation_result_ = ValidationResult{};
  area_index_.clear();
  component_index_.clear();
  app_index_.clear();
  function_index_.clear();

  log_info("Manifest unloaded");
}

bool ManifestManager::is_manifest_active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return manifest_.has_value() && manifest_->is_loaded();
}

std::string ManifestManager::get_manifest_path() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return manifest_path_;
}

ValidationResult ManifestManager::get_validation_result() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return validation_result_;
}

std::optional<ManifestMetadata> ManifestManager::get_metadata() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->metadata;
  }
  return std::nullopt;
}

ManifestConfig ManifestManager::get_config() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->config;
  }
  return ManifestConfig{};
}

std::unordered_map<std::string, ManifestLockConfig> ManifestManager::get_lock_overrides() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->lock_overrides;
  }
  return {};
}

std::vector<ScriptEntryConfig> ManifestManager::get_scripts() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->scripts;
  }
  return {};
}

std::vector<Area> ManifestManager::get_areas() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->areas;
  }
  return {};
}

std::vector<Component> ManifestManager::get_components() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->components;
  }
  return {};
}

std::vector<App> ManifestManager::get_apps() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->apps;
  }
  return {};
}

std::vector<Function> ManifestManager::get_functions() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (manifest_) {
    return manifest_->functions;
  }
  return {};
}

std::optional<Area> ManifestManager::get_area(const std::string & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!manifest_) {
    return std::nullopt;
  }

  auto it = area_index_.find(id);
  if (it != area_index_.end()) {
    return manifest_->areas[it->second];
  }
  return std::nullopt;
}

std::optional<Component> ManifestManager::get_component(const std::string & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!manifest_) {
    return std::nullopt;
  }

  auto it = component_index_.find(id);
  if (it != component_index_.end()) {
    return manifest_->components[it->second];
  }
  return std::nullopt;
}

std::optional<App> ManifestManager::get_app(const std::string & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!manifest_) {
    return std::nullopt;
  }

  auto it = app_index_.find(id);
  if (it != app_index_.end()) {
    return manifest_->apps[it->second];
  }
  return std::nullopt;
}

std::optional<Function> ManifestManager::get_function(const std::string & id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!manifest_) {
    return std::nullopt;
  }

  auto it = function_index_.find(id);
  if (it != function_index_.end()) {
    return manifest_->functions[it->second];
  }
  return std::nullopt;
}

std::vector<Component> ManifestManager::get_components_for_area(const std::string & area_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Component> result;
  if (!manifest_) {
    return result;
  }

  // Collect all area IDs that are descendants of area_id (including area_id itself)
  std::vector<std::string> area_ids;
  area_ids.push_back(area_id);

  // Find all descendant areas (areas whose parent is in our list)
  bool found_new = true;
  while (found_new) {
    found_new = false;
    for (const auto & area : manifest_->areas) {
      // Check if this area's parent is in our list
      if (!area.parent_area_id.empty()) {
        bool parent_in_list = std::find(area_ids.begin(), area_ids.end(), area.parent_area_id) != area_ids.end();
        bool area_in_list = std::find(area_ids.begin(), area_ids.end(), area.id) != area_ids.end();
        if (parent_in_list && !area_in_list) {
          area_ids.push_back(area.id);
          found_new = true;
        }
      }
    }
  }

  // Now collect components from all matching areas
  for (const auto & comp : manifest_->components) {
    if (std::find(area_ids.begin(), area_ids.end(), comp.area) != area_ids.end()) {
      result.push_back(comp);
    }
  }
  return result;
}

std::vector<App> ManifestManager::get_apps_for_component(const std::string & component_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<App> result;
  if (!manifest_) {
    return result;
  }

  for (const auto & app : manifest_->apps) {
    if (app.component_id == component_id) {
      result.push_back(app);
    }
  }
  return result;
}

std::vector<std::string> ManifestManager::get_hosts_for_function(const std::string & function_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!manifest_) {
    return {};
  }

  auto it = function_index_.find(function_id);
  if (it != function_index_.end()) {
    return manifest_->functions[it->second].hosts;
  }
  return {};
}

std::vector<Area> ManifestManager::get_subareas(const std::string & area_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Area> result;
  if (!manifest_) {
    return result;
  }

  for (const auto & area : manifest_->areas) {
    if (area.parent_area_id == area_id) {
      result.push_back(area);
    }
  }
  return result;
}

std::vector<Component> ManifestManager::get_subcomponents(const std::string & component_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<Component> result;
  if (!manifest_) {
    return result;
  }

  for (const auto & comp : manifest_->components) {
    if (comp.parent_component_id == component_id) {
      result.push_back(comp);
    }
  }
  return result;
}

std::optional<json> ManifestManager::get_capabilities_override(const std::string & entity_id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!manifest_) {
    return std::nullopt;
  }

  auto it = manifest_->capabilities.find(entity_id);
  if (it != manifest_->capabilities.end()) {
    return std::optional<json>{it->second};
  }
  return std::nullopt;
}

json ManifestManager::get_status_json() const {
  std::lock_guard<std::mutex> lock(mutex_);

  bool active = manifest_.has_value() && manifest_->is_loaded();

  json status = {{"active", active}, {"path", manifest_path_}};

  if (manifest_) {
    status["metadata"] = {{"name", manifest_->metadata.name},
                          {"version", manifest_->metadata.version},
                          {"description", manifest_->metadata.description}};
    status["manifest_version"] = manifest_->manifest_version;
    status["entity_counts"] = {{"areas", manifest_->areas.size()},
                               {"components", manifest_->components.size()},
                               {"apps", manifest_->apps.size()},
                               {"functions", manifest_->functions.size()},
                               {"scripts", manifest_->scripts.size()}};
  }

  status["validation"] = {{"is_valid", validation_result_.is_valid},
                          {"error_count", validation_result_.errors.size()},
                          {"warning_count", validation_result_.warnings.size()}};

  return status;
}

void ManifestManager::build_lookup_maps() {
  area_index_.clear();
  component_index_.clear();
  app_index_.clear();
  function_index_.clear();

  if (!manifest_) {
    return;
  }

  for (size_t i = 0; i < manifest_->areas.size(); ++i) {
    area_index_[manifest_->areas[i].id] = i;
  }
  for (size_t i = 0; i < manifest_->components.size(); ++i) {
    component_index_[manifest_->components[i].id] = i;
  }
  for (size_t i = 0; i < manifest_->apps.size(); ++i) {
    app_index_[manifest_->apps[i].id] = i;
  }
  for (size_t i = 0; i < manifest_->functions.size(); ++i) {
    function_index_[manifest_->functions[i].id] = i;
  }
}

void ManifestManager::set_fragments_dir(const std::string & dir) {
  std::lock_guard<std::mutex> lock(mutex_);
  fragments_dir_ = dir;
}

std::string ManifestManager::get_fragments_dir() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return fragments_dir_;
}

bool ManifestManager::apply_fragments(Manifest & base) {
  // Called with mutex_ held by the caller.
  if (fragments_dir_.empty()) {
    return true;
  }
  std::filesystem::path dir(fragments_dir_);
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec) || !std::filesystem::is_directory(dir, ec)) {
    // A missing directory is "no fragments", not an error. This lets the
    // deploy side create the directory lazily on first install.
    return true;
  }

  // Collect + sort file paths so the merge order is deterministic. Without
  // this the validator's "duplicate id" error could point at a different
  // fragment run to run, which is nightmarish to diagnose.
  std::vector<std::filesystem::path> files;
  for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
    if (ec) {
      break;
    }
    if (!entry.is_regular_file(ec)) {
      continue;
    }
    auto ext = entry.path().extension().string();
    if (ext == ".yaml" || ext == ".yml") {
      files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());

  bool ok = true;
  for (const auto & path : files) {
    log_info("Merging manifest fragment: " + path.string());
    Manifest fragment;
    try {
      fragment = parser_.parse_fragment_file(path.string());
    } catch (const std::exception & e) {
      validation_result_.add_error("FRAGMENT_PARSE", e.what(), path.string());
      log_error("Failed to parse fragment " + path.string() + ": " + e.what());
      ok = false;
      continue;
    }

    // Fragments may only contribute apps, components, functions. Reject any
    // attempt to redeclare top-level properties owned by the base manifest.
    auto reject = [&](const std::string & field) {
      validation_result_.add_error("FRAGMENT_FORBIDDEN_FIELD", "fragment may not declare '" + field + "'",
                                   path.string());
      log_error("Fragment " + path.string() + " declares forbidden field '" + field + "'");
      ok = false;
    };
    if (!fragment.areas.empty()) {
      reject("areas");
    }
    // manifest_version is auto-injected by parse_fragment_file when the
    // fragment omits it, so we don't check that field here - fragments are
    // free to declare or omit it without penalty.
    if (!fragment.metadata.name.empty() || !fragment.metadata.version.empty() ||
        !fragment.metadata.description.empty() || !fragment.metadata.created_at.empty()) {
      reject("metadata");
    }
    if (!fragment.scripts.empty()) {
      reject("scripts");
    }
    if (!fragment.capabilities.empty()) {
      reject("capabilities");
    }
    if (!fragment.lock_overrides.empty()) {
      reject("lock_overrides");
    }

    // ManifestConfig defaults (unmanifested_nodes=WARN, inherit_runtime_resources=true,
    // allow_manifest_override=true) are indistinguishable from "not declared" on the
    // parsed struct, so detect a `discovery:` top-level key by re-reading the raw YAML.
    try {
      YAML::Node raw = YAML::LoadFile(path.string());
      if (raw && raw["discovery"]) {
        reject("discovery");
      }
    } catch (const YAML::Exception & e) {
      validation_result_.add_error("FRAGMENT_PARSE", e.what(), path.string());
      log_error("Failed to re-parse fragment " + path.string() + " for discovery check: " + e.what());
      ok = false;
    }

    // Append allowed entity lists. Duplicate IDs are caught by the regular
    // validator run once all fragments have been merged.
    for (auto & comp : fragment.components) {
      base.components.push_back(std::move(comp));
    }
    for (auto & app : fragment.apps) {
      base.apps.push_back(std::move(app));
    }
    for (auto & fn : fragment.functions) {
      base.functions.push_back(std::move(fn));
    }
  }
  return ok;
}

void ManifestManager::log_info(const std::string & msg) const {
  if (node_) {
    RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
  }
}

void ManifestManager::log_warn(const std::string & msg) const {
  if (node_) {
    RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
  }
}

void ManifestManager::log_error(const std::string & msg) const {
  if (node_) {
    RCLCPP_ERROR(node_->get_logger(), "%s", msg.c_str());
  }
}

}  // namespace discovery
}  // namespace ros2_medkit_gateway
