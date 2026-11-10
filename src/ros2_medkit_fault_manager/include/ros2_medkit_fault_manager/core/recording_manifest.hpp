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

#pragma once

#include <cstdint>
#include <optional>
#include <string>

namespace ros2_medkit_fault_manager::core {

/// File a closed recording directory carries next to `metadata.yaml`.
inline constexpr const char * kRecordingManifestFile = "manifest.json";

/// Lowercase hex sha256 of the file at @p path, or std::nullopt when it cannot be read.
std::optional<std::string> sha256_file_hex(const std::string & path);

/// Build the manifest of the closed recording at @p bag_path.
///
/// The storage files are listed in the order `metadata.yaml` names them
/// (`relative_file_paths`), each with its size and sha256. `metadata.yaml` itself is
/// neither listed nor counted in `total_bytes`.
///
/// @return The manifest text, or std::nullopt with @p error set.
std::optional<std::string> build_recording_manifest(const std::string & bag_path, int64_t created_at_ns,
                                                    std::string & error);

/// Write `manifest.json` into the closed recording at @p bag_path: a temporary file in
/// the same directory, renamed into place. An existing manifest is never replaced.
///
/// @return std::nullopt on success, the reason otherwise. On failure the directory has
///         no manifest.
std::optional<std::string> write_recording_manifest(const std::string & bag_path, int64_t created_at_ns);

}  // namespace ros2_medkit_fault_manager::core
