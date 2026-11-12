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

#include <functional>
#include <string>
#include <utility>

#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"

namespace ros2_medkit_fault_manager::ros2 {

/// Runs the node's snapshot and rosbag capture for the core's capture calls. Each call forwards to
/// the function given at construction; the node reads its capture members when it runs.
class RosCaptureHooks : public core::CaptureHooks {
 public:
  using CaptureFn = std::function<core::CaptureResult(const FaultId &)>;
  using RecordFn = std::function<void(const FaultId &)>;

  RosCaptureHooks(CaptureFn capture_now, RecordFn on_prefailed, RecordFn on_cleared)
    : capture_now_(std::move(capture_now)), on_prefailed_(std::move(on_prefailed)), on_cleared_(std::move(on_cleared)) {
  }

  core::CaptureResult capture_now(const FaultId & id) override {
    return capture_now_(id);
  }
  void on_prefailed(const FaultId & id) override {
    on_prefailed_(id);
  }
  void on_cleared(const FaultId & id) override {
    on_cleared_(id);
  }

 private:
  CaptureFn capture_now_;
  RecordFn on_prefailed_;
  RecordFn on_cleared_;
};

}  // namespace ros2_medkit_fault_manager::ros2
