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
#include "ros2_medkit_gateway/discovery/manifest/manifest_parser.hpp"
#include "ros2_medkit_gateway/discovery/manifest/manifest_validator.hpp"
#include "ros2_medkit_gateway/discovery/manifest/validation_error.hpp"

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace ros2_medkit_gateway {
namespace discovery {

using json = nlohmann::json;

/**
 * @brief Manages manifest loading, validation, and entity access
 *
 * Thread-safe class for managing the manifest lifecycle.
 * Provides access to parsed entities and relationship queries.
 */
class ManifestManager {
 public:
  /**
   * @brief Construct ManifestManager
   * @param node ROS node for logging (can be nullptr for testing)
   */
  explicit ManifestManager(rclcpp::Node * node = nullptr);

  // === Manifest Loading ===

  /**
   * @brief Load manifest from file
   * @param file_path Path to manifest YAML file
   * @param strict If true, also fail on warnings; errors always cause failure
   * @return true if loaded and validated successfully
   *
   * @note ERRORs (broken references, circular deps, duplicate bindings) always
   *       cause failure regardless of strict mode, as they indicate a fundamentally
   *       broken manifest. Only WARNINGs are affected by the strict flag.
   */
  bool load_manifest(const std::string & file_path, bool strict = true);

  /**
   * @brief Load manifest from YAML string (useful for testing)
   * @param yaml_content YAML content
   * @param strict If true, also fail on warnings; errors always cause failure
   * @return true if loaded and validated successfully
   *
   * @note See load_manifest() for behavior details.
   */
  bool load_manifest_from_string(const std::string & yaml_content, bool strict = true);

  /**
   * @brief Reload manifest from previously loaded file path
   *
   * Re-parses the base manifest file AND re-scans the fragments directory
   * (if one has been configured via `set_fragments_dir`). Fragments are
   * merged on top of the base manifest before validation runs. A missing
   * fragments directory is treated as "no fragments" and does not fail the
   * reload.
   *
   * @return true if reloaded successfully
   */
  bool reload_manifest();

  /**
   * @brief Configure a directory containing manifest fragment files.
   *
   * When set, every `load_manifest` / `reload_manifest` call scans the
   * directory for files whose name ends in `.yaml` or `.yml` (recursion is
   * not performed), parses each one with the standard `ManifestParser`, and
   * merges the declared entities on top of the base manifest before
   * validation. Plugins that deploy new nodes at runtime can drop a
   * per-deploy fragment alongside their deployed files without editing
   * the base manifest.
   *
   * Merge rules:
   *   * apps / components / functions are appended; duplicate IDs across
   *     the merged manifest cause validation to fail the same way they
   *     would in a single file.
   *   * fragments may not declare top-level `areas`, `metadata`,
   *     `discovery`, `scripts`, `capabilities`, or `lock_overrides` - those
   *     must stay in the base manifest. Each forbidden field a fragment
   *     declares is recorded as a `FRAGMENT_FORBIDDEN_FIELD` validation
   *     error and causes manifest loading / reloading to fail.
   *
   * Call with an empty string to disable fragment scanning.
   *
   * @param dir Absolute or gateway-relative path to a directory. Does not
   *            need to exist at call time; a missing directory on load is
   *            treated as "no fragments found".
   */
  void set_fragments_dir(const std::string & dir);

  /**
   * @brief Get the currently configured fragments directory (empty if unset).
   */
  std::string get_fragments_dir() const;

  /**
   * @brief Unload current manifest (revert to runtime-only mode)
   */
  void unload_manifest();

  // === Status ===

  /**
   * @brief Check if manifest is loaded and active
   */
  bool is_manifest_active() const;

  /**
   * @brief Get the loaded manifest file path
   */
  std::string get_manifest_path() const;

  /**
   * @brief Get last validation result
   */
  ValidationResult get_validation_result() const;

  /**
   * @brief Get manifest metadata
   */
  std::optional<ManifestMetadata> get_metadata() const;

  /**
   * @brief Get manifest config
   */
  ManifestConfig get_config() const;

  /**
   * @brief Get per-entity lock configuration overrides from manifest
   */
  std::unordered_map<std::string, ManifestLockConfig> get_lock_overrides() const;

  /**
   * @brief Get script entries defined in manifest
   */
  std::vector<ScriptEntryConfig> get_scripts() const;

  // === Entity Access ===

  /**
   * @brief Get all areas from manifest
   */
  std::vector<Area> get_areas() const;

  /**
   * @brief Get all components from manifest
   */
  std::vector<Component> get_components() const;

  /**
   * @brief Get all apps from manifest
   */
  std::vector<App> get_apps() const;

  /**
   * @brief Get all functions from manifest
   */
  std::vector<Function> get_functions() const;

  /**
   * @brief Get area by ID
   */
  std::optional<Area> get_area(const std::string & id) const;

  /**
   * @brief Get component by ID
   */
  std::optional<Component> get_component(const std::string & id) const;

  /**
   * @brief Get app by ID
   */
  std::optional<App> get_app(const std::string & id) const;

  /**
   * @brief Get function by ID
   */
  std::optional<Function> get_function(const std::string & id) const;

  // === Relationship Queries ===

  /**
   * @brief Get components belonging to an area
   */
  std::vector<Component> get_components_for_area(const std::string & area_id) const;

  /**
   * @brief Get apps located on a component
   */
  std::vector<App> get_apps_for_component(const std::string & component_id) const;

  /**
   * @brief Get entities hosted by a function
   */
  std::vector<std::string> get_hosts_for_function(const std::string & function_id) const;

  /**
   * @brief Get subareas of an area
   */
  std::vector<Area> get_subareas(const std::string & area_id) const;

  /**
   * @brief Get subcomponents of a component
   */
  std::vector<Component> get_subcomponents(const std::string & component_id) const;

  // === Capabilities ===

  /**
   * @brief Get custom capabilities override for an entity
   */
  std::optional<json> get_capabilities_override(const std::string & entity_id) const;

  // === Status JSON for REST API ===

  /**
   * @brief Get manifest status as JSON (for /manifest/status endpoint)
   */
  json get_status_json() const;

 private:
  /// Build lookup maps after loading manifest
  void build_lookup_maps();

  /// Log info message (handles null node)
  void log_info(const std::string & msg) const;
  /// Log warning message
  void log_warn(const std::string & msg) const;
  /// Log error message
  void log_error(const std::string & msg) const;

  rclcpp::Node * node_;
  mutable std::mutex mutex_;

  /// Merge the fragments found in `fragments_dir_` on top of `base`.
  /// Returns true on success (or no fragments); false if any fragment
  /// failed to parse or declared disallowed top-level fields.
  /// Duplicate / conflicting IDs are not detected here - they are surfaced
  /// by the subsequent `ManifestValidator::validate()` pass over the merged
  /// manifest. Validation errors from individual fragments are appended to
  /// `validation_result_` so callers see them in the normal error flow.
  bool apply_fragments(Manifest & base);

  std::optional<Manifest> manifest_;
  std::string manifest_path_;
  std::string fragments_dir_;
  ValidationResult validation_result_;
  bool strict_mode_{true};

  ManifestParser parser_;
  ManifestValidator validator_;

  // Lookup maps for fast access by ID
  std::unordered_map<std::string, size_t> area_index_;
  std::unordered_map<std::string, size_t> component_index_;
  std::unordered_map<std::string, size_t> app_index_;
  std::unordered_map<std::string, size_t> function_index_;
};

}  // namespace discovery
}  // namespace ros2_medkit_gateway
