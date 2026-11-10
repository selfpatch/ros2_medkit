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

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

class FaultManager;

namespace faults {

/// File the fault manager writes into a closed recording directory.
inline constexpr const char * kRecordingManifestFile = "manifest.json";

/// Whether the recording directory at @p recording_path carries a manifest.
bool recording_has_manifest(const std::string & recording_path);

/// The bytes of the recording's `manifest.json`, or std::nullopt when it is absent,
/// unreadable or larger than a manifest can be.
std::optional<std::string> read_recording_manifest(const std::string & recording_path);

/// A decimal index into the manifest's `files`: digits only.
std::optional<std::size_t> parse_recording_file_index(const std::string & text);

/// Path of storage file `files[index]` of @p manifest_text inside @p recording_path.
///
/// The name comes from the manifest and must be a plain file name that exists in the
/// directory. std::nullopt for a manifest that cannot be read that way, an index past
/// the list, or a file missing on disk.
std::optional<std::string> recording_file_path(const std::string & recording_path, const std::string & manifest_text,
                                               std::size_t index);

/// Whether every range of a `Range` header value lies inside a file of @p size bytes.
/// False for a first byte at or past the end, a last byte past the end, an empty
/// suffix, and a value that is not a byte range.
bool range_within_file(const std::string & range_header, uint64_t size);

enum class RecordingLookupStatus {
  /// The recording exists and one of its faults is in the entity's scope.
  kFound,
  /// The fault manager answered and holds no recording with this id.
  kNotFound,
  /// The recording exists, but not for this entity.
  kNotInEntity,
  /// The fault manager did not give an answer that settles the question.
  kUnavailable,
};

struct RecordingLookup {
  RecordingLookupStatus status{RecordingLookupStatus::kUnavailable};
  /// Recording directory. Set when status is kFound.
  std::string path;
};

/// Resolve @p recording_id for an entity whose fault sources are @p source_filters.
///
/// The id names a recording only, never a fault code. kNotFound is returned only on
/// positive evidence: the fault manager resolved the id to no recording, or it
/// declined the id and a complete listing of the entity's recordings does not carry
/// it. Anything short of that is kUnavailable.
RecordingLookup find_recording(FaultManager & fault_manager, const std::string & recording_id,
                               const std::vector<std::string> & source_filters);

}  // namespace faults
}  // namespace ros2_medkit_gateway
