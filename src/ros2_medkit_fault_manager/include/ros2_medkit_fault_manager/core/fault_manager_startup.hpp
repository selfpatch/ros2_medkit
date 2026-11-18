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

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "ros2_medkit_fault_detection/fault_types.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_config.hpp"
#include "ros2_medkit_fault_manager/core/log_sink.hpp"
#include "ros2_medkit_fault_manager/correlation/correlation_engine.hpp"
#include "ros2_medkit_fault_manager/fault_audit_log.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/threshold_resolver.hpp"

namespace ros2_medkit_fault_manager::core {

/// What the fault manager builds from a validated `FaultManagerConfig` without any ROS handle.
struct CoreParts {
  std::unique_ptr<FaultStorage> storage;
  std::unique_ptr<FaultAuditLog> audit;                             ///< nullptr when disabled
  std::unique_ptr<correlation::CorrelationEngine> correlation;      ///< nullptr when disabled or invalid
  DebounceConfig global_debounce;                                   ///< After sanitizing
  std::unique_ptr<EntityThresholdResolver> entity_resolver;         ///< nullptr when none loaded
  std::unique_ptr<FaultCodeThresholdResolver> fault_code_resolver;  ///< nullptr when none loaded
};

/// Opens the storage and the audit log, applies the storage caps, loads the threshold files and the
/// correlation config. Logs through @p sink. Throws when the storage or the audit log cannot be opened.
CoreParts build_core_parts(const FaultManagerConfig & config, LogSink & sink);

/// Appends the audit event of one reclassified fault. May throw to fail the startup.
using ReclassifyAuditFn =
    std::function<void(const ros2_medkit::fault_detection::FaultRecord &, int64_t occurred_at_ns)>;

/// Turns stale HEALED rows into CLEARED for codes whose healing is now disabled. Calls @p audit for
/// each flipped fault when @p audit_enabled. Storage that is unreachable skips it with a warning.
void reclassify_stale_healed(FaultStorage & storage, const FaultCodeThresholdResolver * fault_code_resolver,
                             const DebounceConfig & global_debounce, bool audit_enabled, LogSink & sink,
                             const std::string & logger, const ReclassifyAuditFn & audit);

}  // namespace ros2_medkit_fault_manager::core
