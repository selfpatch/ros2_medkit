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

#include <cstddef>
#include <string>

namespace ros2_medkit_fault_manager::core {

/// Size of the storage file a download of the recording at @p bag_path transfers: the one file
/// its metadata.yaml names in relative_file_paths.
///
/// Returns @p stored_total_bytes (the directory total) when metadata.yaml is missing or not the
/// rosbag2 shape, names other than exactly one file, or names something other than a regular
/// .db3 or .mcap file directly inside the bag (no symlinks). The gateway applies the same rule in
/// bulkdata_handlers.cpp. docs/api/rest.rst, "One recording, one size", states the contract.
size_t rosbag_served_bytes(const std::string & bag_path, size_t stored_total_bytes);

}  // namespace ros2_medkit_fault_manager::core
