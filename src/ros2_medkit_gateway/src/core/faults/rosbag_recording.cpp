// Copyright 2026 mfaferek93
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

#include "ros2_medkit_gateway/core/faults/rosbag_recording.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <set>
#include <string_view>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/core/faults/fault_scope.hpp"
#include "ros2_medkit_gateway/core/managers/fault_manager.hpp"

namespace ros2_medkit_gateway {
namespace faults {

namespace {

constexpr std::uintmax_t kMaxManifestBytes = 8 * 1024 * 1024;

/// Digits only, no sign, no overflow.
std::optional<uint64_t> parse_decimal(std::string_view text) {
  if (text.empty() || text.size() > 19) {
    return std::nullopt;
  }
  uint64_t value = 0;
  for (const char c : text) {
    if (c < '0' || c > '9') {
      return std::nullopt;
    }
    value = value * 10 + static_cast<uint64_t>(c - '0');
  }
  return value;
}

/// A link is not a file of the recording: it is never followed.
bool is_plain_file(const std::filesystem::path & path) {
  std::error_code ec;
  return std::filesystem::is_regular_file(std::filesystem::symlink_status(path, ec));
}

}  // namespace

bool recording_has_manifest(const std::string & recording_path) {
  return is_plain_file(std::filesystem::path(recording_path) / kRecordingManifestFile);
}

std::optional<std::string> read_recording_manifest(const std::string & recording_path) {
  const auto manifest_path = std::filesystem::path(recording_path) / kRecordingManifestFile;
  if (!is_plain_file(manifest_path)) {
    return std::nullopt;
  }
  std::error_code ec;
  const auto size = std::filesystem::file_size(manifest_path, ec);
  if (ec || size > kMaxManifestBytes) {
    return std::nullopt;
  }
  std::ifstream file(manifest_path, std::ios::binary);
  if (!file.is_open()) {
    return std::nullopt;
  }
  std::string text(static_cast<std::size_t>(size), '\0');
  file.read(text.data(), static_cast<std::streamsize>(text.size()));
  if (static_cast<std::size_t>(file.gcount()) != text.size()) {
    return std::nullopt;
  }
  return text;
}

std::optional<std::size_t> parse_recording_file_index(const std::string & text) {
  if (text.size() > 9) {
    return std::nullopt;
  }
  const auto value = parse_decimal(text);
  if (!value) {
    return std::nullopt;
  }
  return static_cast<std::size_t>(*value);
}

std::optional<std::string> recording_file_path(const std::string & recording_path, const std::string & manifest_text,
                                               std::size_t index) {
  const auto manifest = nlohmann::json::parse(manifest_text, nullptr, false);
  if (manifest.is_discarded() || !manifest.is_object()) {
    return std::nullopt;
  }
  if (!manifest.contains("files") || !manifest["files"].is_array() || index >= manifest["files"].size()) {
    return std::nullopt;
  }
  const auto & entry = manifest["files"][index];
  if (!entry.is_object() || !entry.contains("name") || !entry["name"].is_string()) {
    return std::nullopt;
  }
  const std::string name = entry["name"].get<std::string>();
  // A plain file name only: the manifest must not be able to point outside its directory.
  const std::filesystem::path as_path(name);
  if (name.empty() || name == "." || name == ".." || as_path.filename().string() != name ||
      name.find('\\') != std::string::npos) {
    return std::nullopt;
  }
  const auto file_path = std::filesystem::path(recording_path) / name;
  if (!is_plain_file(file_path)) {
    return std::nullopt;
  }
  return file_path.string();
}

std::vector<AttachedRecord> attached_records(const nlohmann::json & rosbag) {
  std::vector<AttachedRecord> records;
  const auto codes = rosbag.find("fault_codes");
  if (codes == rosbag.end() || !codes->is_array()) {
    return records;
  }
  // `owners` runs parallel to `fault_codes`. A fault manager that predates owners
  // sends none, and every record it holds is shared, so the code alone names it.
  const auto owners = rosbag.find("owners");
  const bool has_owners = owners != rosbag.end() && owners->is_array() && owners->size() == codes->size();
  records.reserve(codes->size());
  for (std::size_t i = 0; i < codes->size(); ++i) {
    const auto & code = (*codes)[i];
    if (!code.is_string()) {
      continue;
    }
    std::string owner;
    if (has_owners && (*owners)[i].is_string()) {
      owner = (*owners)[i].get<std::string>();
    }
    records.push_back(AttachedRecord{code.get<std::string>(), std::move(owner)});
  }
  return records;
}

bool attached_record_in_scope(const std::vector<AttachedRecord> & records, const nlohmann::json & faults,
                              const std::set<std::string> & scope) {
  if (!faults.is_array()) {
    return false;
  }
  for (const auto & fault : faults) {
    if (!fault.is_object() || !fault_in_source_scope(fault, scope)) {
      continue;
    }
    const std::string code = fault.value("fault_code", std::string{});
    const std::string owner = stored_owner(fault);
    if (std::any_of(records.begin(), records.end(), [&](const AttachedRecord & record) {
          return record.fault_code == code && (record.owner.empty() || record.owner == owner);
        })) {
      return true;
    }
  }
  return false;
}

RecordingLookup find_recording(FaultManager & fault_manager, const std::string & recording_id,
                               const std::vector<std::string> & source_filters) {
  RecordingLookup lookup;

  const auto rosbag = fault_manager.get_rosbag(recording_id);
  if (!rosbag.success && rosbag.failure == FaultFailure::Unavailable) {
    return lookup;
  }

  if (rosbag.success) {
    const std::string resolved = rosbag.data.value("recording_id", std::string{});
    if (resolved.empty() || !rosbag.data.contains("file_path") || !rosbag.data["file_path"].is_string()) {
      lookup.status = RecordingLookupStatus::kNotInEntity;
      return lookup;
    }
    if (resolved != recording_id) {
      // The fault manager found no recording under this id and fell back to reading it
      // as a fault code. These routes take a recording id only.
      lookup.status = RecordingLookupStatus::kNotFound;
      return lookup;
    }

    // The recording is the entity's when one of the records it is attached to is in scope.
    // Every status and muted records: a recording outlives its record's active state.
    const std::set<std::string> scope(source_filters.begin(), source_filters.end());
    const auto held = fault_manager.list_faults("", /*include_prefailed=*/true, /*include_confirmed=*/true,
                                                /*include_cleared=*/true, /*include_healed=*/true,
                                                /*include_muted=*/true);
    if (!held.success) {
      return lookup;
    }
    if (!attached_record_in_scope(attached_records(rosbag.data), held.data.value("faults", nlohmann::json::array()),
                                  scope)) {
      lookup.status = RecordingLookupStatus::kNotInEntity;
      return lookup;
    }
    lookup.status = RecordingLookupStatus::kFound;
    lookup.path = rosbag.data["file_path"].get<std::string>();
    return lookup;
  }

  // Declined. That covers "no such recording" and also a store that could not be read,
  // and the answer does not say which. A complete listing settles it.
  if (source_filters.empty()) {
    // Nothing to list, so nothing proves the recording absent. An entity with no source
    // owns no recording either way.
    lookup.status = RecordingLookupStatus::kNotInEntity;
    return lookup;
  }
  for (const auto & source_filter : source_filters) {
    const auto listed = fault_manager.list_rosbags(source_filter);
    if (!listed.success) {
      return lookup;
    }
    if (!listed.data.contains("rosbags") || !listed.data["rosbags"].is_array()) {
      continue;
    }
    for (const auto & row : listed.data["rosbags"]) {
      if (row.is_object() && row.value("recording_id", std::string{}) == recording_id) {
        return lookup;
      }
    }
  }
  lookup.status = RecordingLookupStatus::kNotFound;
  return lookup;
}

}  // namespace faults
}  // namespace ros2_medkit_gateway
