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

#include "ros2_medkit_fault_manager/core/rosbag_served_size.hpp"

#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <optional>
#include <system_error>
#include <vector>

namespace ros2_medkit_fault_manager::core {

namespace {

/// A regular file that is not a symlink. A link inside a bag could point at any file on the host.
bool is_plain_file(const std::filesystem::path & path) {
  std::error_code ec;
  return std::filesystem::symlink_status(path, ec).type() == std::filesystem::file_type::regular && !ec;
}

/// rosbag2_bagfile_information.relative_file_paths from <bag>/metadata.yaml. nullopt when the
/// file is missing, a symlink, unreadable, or not that shape.
std::optional<std::vector<std::string>> relative_file_paths(const std::filesystem::path & bag) {
  const std::filesystem::path metadata_path = bag / "metadata.yaml";
  if (!is_plain_file(metadata_path)) {
    return std::nullopt;
  }
  try {
    const YAML::Node root = YAML::LoadFile(metadata_path.string());
    if (!root.IsMap()) {
      return std::nullopt;
    }
    const YAML::Node info = root["rosbag2_bagfile_information"];
    if (!info || !info.IsMap()) {
      return std::nullopt;
    }
    const YAML::Node paths = info["relative_file_paths"];
    if (!paths || !paths.IsSequence()) {
      return std::nullopt;
    }
    std::vector<std::string> names;
    names.reserve(paths.size());
    for (const auto & entry : paths) {
      if (!entry.IsScalar()) {
        return std::nullopt;
      }
      names.push_back(entry.as<std::string>());
    }
    return names;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

/// @p name inside @p bag when it is a direct child with a .db3 or .mcap extension. An absolute
/// or `..` name would otherwise pick any file on the host.
std::optional<std::filesystem::path> named_storage_file(const std::filesystem::path & bag, const std::string & name) {
  std::filesystem::path named = (bag / name).lexically_normal();
  if (named.parent_path() != bag) {
    return std::nullopt;
  }
  const std::string extension = named.extension().string();
  if (extension != ".db3" && extension != ".mcap") {
    return std::nullopt;
  }
  return named;
}

}  // namespace

size_t rosbag_served_bytes(const std::string & bag_path, size_t stored_total_bytes) {
  std::filesystem::path bag = std::filesystem::path(bag_path).lexically_normal();
  if (!bag.has_filename()) {
    bag = bag.parent_path();  // trailing slash
  }

  // metadata.yaml and the storage files are written once, when the recording closes.
  const auto names = relative_file_paths(bag);
  if (!names || names->empty()) {
    return stored_total_bytes;
  }
  size_t total = 0;
  for (const auto & name : *names) {
    const auto file = named_storage_file(bag, name);
    if (!file || !is_plain_file(*file)) {
      return stored_total_bytes;
    }
    std::error_code ec;
    const auto size = std::filesystem::file_size(*file, ec);
    if (ec) {
      return stored_total_bytes;
    }
    total += static_cast<size_t>(size);
  }
  return total;
}

}  // namespace ros2_medkit_fault_manager::core
