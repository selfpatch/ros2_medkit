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

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_fault_manager/core/rosbag_retention.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"

/// Store rosbag rows as they are, through the storage primitive: no cap, no quota, no unlink.
inline void seed_rosbag_rows(ros2_medkit_fault_manager::FaultStorage & storage,
                             std::vector<ros2_medkit_fault_manager::RosbagFileInfo> rows) {
  ros2_medkit_fault_manager::RosbagChangeSet changes;
  changes.insert = std::move(rows);
  storage.apply_rosbag_changes(changes);
}

/// The paths one retention call hands over: each still exists when the call returns, and is
/// listed once.
inline void expect_unlinkable(const std::vector<std::string> & paths) {
  for (const auto & path : paths) {
    EXPECT_TRUE(std::filesystem::exists(path)) << path << " was removed before the caller could";
    EXPECT_EQ(std::count(paths.begin(), paths.end(), path), 1) << path << " is listed more than once";
  }
}

inline bool listed(const std::vector<std::string> & paths, const std::string & path) {
  return std::find(paths.begin(), paths.end(), path) != paths.end();
}

inline std::vector<std::string> paths_of(const std::vector<ros2_medkit_fault_manager::core::FreedRecording> & freed) {
  std::vector<std::string> paths;
  paths.reserve(freed.size());
  for (const auto & recording : freed) {
    paths.push_back(recording.path);
  }
  return paths;
}

inline void expect_unlinkable(const std::vector<ros2_medkit_fault_manager::core::FreedRecording> & freed) {
  expect_unlinkable(paths_of(freed));
}

inline bool listed(const std::vector<ros2_medkit_fault_manager::core::FreedRecording> & freed,
                   const std::string & path) {
  return listed(paths_of(freed), path);
}
