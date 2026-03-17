// Copyright 2026 Bartlomiej Burda
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

#include "ros2_medkit_gateway/default_script_provider.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace ros2_medkit_gateway {

DefaultScriptProvider::DefaultScriptProvider(const ScriptsConfig & config) : config_(config) {
  // Populate manifest_scripts_ from config entries
  for (const auto & entry : config_.entries) {
    manifest_scripts_[entry.id] = entry;
  }

  // Ensure the scripts directory exists
  if (!config_.scripts_dir.empty()) {
    std::error_code ec;
    std::filesystem::create_directories(config_.scripts_dir, ec);
  }
}

tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo>
DefaultScriptProvider::list_scripts(const std::string & entity_id) {
  std::lock_guard<std::mutex> lock(fs_mutex_);

  std::vector<ScriptInfo> result;

  // 1. Collect manifest scripts matching the entity filter
  for (const auto & [id, entry] : manifest_scripts_) {
    if (matches_entity_filter(entity_id, entry.entity_filter)) {
      ScriptInfo info;
      info.id = entry.id;
      info.name = entry.name;
      info.description = entry.description;
      info.managed = true;
      info.proximity_proof_required = false;
      info.parameters_schema = entry.parameters_schema;
      result.push_back(std::move(info));
    }
  }

  // 2. Scan filesystem for uploaded scripts: scripts_dir/{entity_id}/*/metadata.json
  if (!config_.scripts_dir.empty()) {
    auto entity_dir = std::filesystem::path(config_.scripts_dir) / entity_id;
    std::error_code ec;
    if (std::filesystem::is_directory(entity_dir, ec)) {
      for (const auto & dir_entry : std::filesystem::directory_iterator(entity_dir, ec)) {
        if (!dir_entry.is_directory()) {
          continue;
        }
        auto meta_path = dir_entry.path() / "metadata.json";
        if (!std::filesystem::exists(meta_path, ec)) {
          continue;
        }
        try {
          std::ifstream meta_file(meta_path);
          auto meta = nlohmann::json::parse(meta_file);
          ScriptInfo info;
          info.id = dir_entry.path().filename().string();
          info.name = meta.value("name", info.id);
          info.description = meta.value("description", "");
          info.managed = false;
          info.proximity_proof_required = false;
          if (meta.contains("parameters_schema") && !meta["parameters_schema"].is_null()) {
            info.parameters_schema = meta["parameters_schema"];
          }
          result.push_back(std::move(info));
        } catch (const std::exception &) {
          // Skip malformed metadata files
          continue;
        }
      }
    }
  }

  return result;
}

tl::expected<ScriptInfo, ScriptBackendErrorInfo> DefaultScriptProvider::get_script(const std::string & entity_id,
                                                                                   const std::string & script_id) {
  // Check manifest first
  auto manifest_it = manifest_scripts_.find(script_id);
  if (manifest_it != manifest_scripts_.end()) {
    const auto & entry = manifest_it->second;
    if (matches_entity_filter(entity_id, entry.entity_filter)) {
      ScriptInfo info;
      info.id = entry.id;
      info.name = entry.name;
      info.description = entry.description;
      info.managed = true;
      info.proximity_proof_required = false;
      info.parameters_schema = entry.parameters_schema;
      return info;
    }
  }

  // Check filesystem
  if (!config_.scripts_dir.empty()) {
    auto meta_path = script_dir_path(entity_id, script_id) / "metadata.json";
    std::error_code ec;
    if (std::filesystem::exists(meta_path, ec)) {
      try {
        std::ifstream meta_file(meta_path);
        auto meta = nlohmann::json::parse(meta_file);
        ScriptInfo info;
        info.id = script_id;
        info.name = meta.value("name", script_id);
        info.description = meta.value("description", "");
        info.managed = false;
        info.proximity_proof_required = false;
        if (meta.contains("parameters_schema") && !meta["parameters_schema"].is_null()) {
          info.parameters_schema = meta["parameters_schema"];
        }
        return info;
      } catch (const std::exception &) {
        return tl::make_unexpected(
            ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to read script metadata"});
      }
    }
  }

  return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found: " + script_id});
}

tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
DefaultScriptProvider::upload_script(const std::string & entity_id, const std::string & filename,
                                     const std::string & content, const std::optional<nlohmann::json> & metadata) {
  std::lock_guard<std::mutex> lock(fs_mutex_);

  if (config_.scripts_dir.empty()) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, "Scripts directory not configured"});
  }

  // Check file size
  auto max_bytes = static_cast<size_t>(config_.max_file_size_mb) * 1024 * 1024;
  if (content.size() > max_bytes) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::FileTooLarge, "Script file exceeds maximum size of " +
                                                                     std::to_string(config_.max_file_size_mb) + " MB"});
  }

  // Generate a unique script ID
  auto script_id = generate_id();

  // Detect format from filename
  auto format = detect_format(filename);

  // Create directory
  auto dir = script_dir_path(entity_id, script_id);
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  if (ec) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to create script directory: " + ec.message()});
  }

  // Determine extension from format
  std::string ext;
  if (format == "python") {
    ext = ".py";
  } else if (format == "bash") {
    ext = ".bash";
  } else {
    ext = ".sh";
  }

  // Write script file
  auto script_path = dir / ("script" + ext);
  {
    std::ofstream script_file(script_path, std::ios::binary);
    if (!script_file) {
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to write script file"});
    }
    script_file.write(content.data(), static_cast<std::streamsize>(content.size()));
  }

  // Build metadata JSON
  std::string name = filename;
  std::string description;
  std::optional<nlohmann::json> parameters_schema;

  if (metadata.has_value()) {
    const auto & meta = metadata.value();
    if (meta.contains("name") && meta["name"].is_string()) {
      name = meta["name"].get<std::string>();
    }
    if (meta.contains("description") && meta["description"].is_string()) {
      description = meta["description"].get<std::string>();
    }
    if (meta.contains("parameters_schema") && !meta["parameters_schema"].is_null()) {
      parameters_schema = meta["parameters_schema"];
    }
  }

  nlohmann::json meta_json;
  meta_json["name"] = name;
  meta_json["description"] = description;
  meta_json["filename"] = filename;
  meta_json["format"] = format;
  meta_json["created_at"] = now_iso8601();
  if (parameters_schema.has_value()) {
    meta_json["parameters_schema"] = parameters_schema.value();
  } else {
    meta_json["parameters_schema"] = nullptr;
  }

  // Write metadata.json
  auto meta_path = dir / "metadata.json";
  {
    std::ofstream meta_file(meta_path);
    if (!meta_file) {
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to write metadata file"});
    }
    meta_file << meta_json.dump(2);
  }

  return ScriptUploadResult{script_id, name};
}

tl::expected<void, ScriptBackendErrorInfo> DefaultScriptProvider::delete_script(const std::string & entity_id,
                                                                                const std::string & script_id) {
  // Check if it's a manifest script - cannot delete those
  if (is_manifest_script(script_id)) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::ManagedScript, "Cannot delete managed script: " + script_id});
  }

  std::lock_guard<std::mutex> lock(fs_mutex_);

  auto dir = script_dir_path(entity_id, script_id);
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec)) {
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found: " + script_id});
  }

  std::filesystem::remove_all(dir, ec);
  if (ec) {
    return tl::make_unexpected(
        ScriptBackendErrorInfo{ScriptBackendError::Internal, "Failed to delete script: " + ec.message()});
  }

  return {};
}

// --- Execution methods (stubbed, implemented in Task 7) ---

tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
DefaultScriptProvider::start_execution(const std::string & /*entity_id*/, const std::string & /*script_id*/,
                                       const ExecutionRequest & /*request*/) {
  return tl::make_unexpected(
      ScriptBackendErrorInfo{ScriptBackendError::Internal, "Script execution not yet implemented"});
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
DefaultScriptProvider::get_execution(const std::string & /*entity_id*/, const std::string & /*script_id*/,
                                     const std::string & /*execution_id*/) {
  return tl::make_unexpected(
      ScriptBackendErrorInfo{ScriptBackendError::Internal, "Script execution not yet implemented"});
}

tl::expected<ExecutionInfo, ScriptBackendErrorInfo>
DefaultScriptProvider::control_execution(const std::string & /*entity_id*/, const std::string & /*script_id*/,
                                         const std::string & /*execution_id*/, const std::string & /*action*/) {
  return tl::make_unexpected(
      ScriptBackendErrorInfo{ScriptBackendError::Internal, "Script execution not yet implemented"});
}

tl::expected<void, ScriptBackendErrorInfo>
DefaultScriptProvider::delete_execution(const std::string & /*entity_id*/, const std::string & /*script_id*/,
                                        const std::string & /*execution_id*/) {
  return tl::make_unexpected(
      ScriptBackendErrorInfo{ScriptBackendError::Internal, "Script execution not yet implemented"});
}

// --- Private helpers ---

std::filesystem::path DefaultScriptProvider::script_dir_path(const std::string & entity_id,
                                                             const std::string & script_id) const {
  return std::filesystem::path(config_.scripts_dir) / entity_id / script_id;
}

std::string DefaultScriptProvider::generate_id() {
  return "script_" + std::to_string(id_counter_++);
}

bool DefaultScriptProvider::is_manifest_script(const std::string & script_id) const {
  return manifest_scripts_.find(script_id) != manifest_scripts_.end();
}

bool DefaultScriptProvider::matches_entity_filter(const std::string & entity_id,
                                                  const std::vector<std::string> & filters) {
  // Empty filter list matches everything
  if (filters.empty()) {
    return true;
  }

  for (const auto & filter : filters) {
    // Wildcard matches everything
    if (filter == "*") {
      return true;
    }

    // Check for glob pattern ending with /* (e.g. "components/*")
    if (filter.size() >= 2 && filter.substr(filter.size() - 2) == "/*") {
      // The prefix before /* is informational (entity type hint) - since entity_id
      // alone does not carry type info, a trailing wildcard matches any entity
      return true;
    }

    // Exact match
    if (filter == entity_id) {
      return true;
    }
  }

  return false;
}

std::string DefaultScriptProvider::detect_format(const std::string & filename) {
  auto dot_pos = filename.rfind('.');
  if (dot_pos == std::string::npos) {
    return "sh";
  }

  auto ext = filename.substr(dot_pos);
  if (ext == ".py") {
    return "python";
  }
  if (ext == ".bash") {
    return "bash";
  }
  return "sh";
}

std::string DefaultScriptProvider::now_iso8601() {
  auto now = std::chrono::system_clock::now();
  auto time_t_now = std::chrono::system_clock::to_time_t(now);
  std::tm tm_buf{};
  gmtime_r(&time_t_now, &tm_buf);
  std::ostringstream oss;
  oss << std::put_time(&tm_buf, "%Y-%m-%dT%H:%M:%SZ");
  return oss.str();
}

}  // namespace ros2_medkit_gateway
