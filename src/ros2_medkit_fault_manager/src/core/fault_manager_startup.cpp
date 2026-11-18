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

#include "ros2_medkit_fault_manager/core/fault_manager_startup.hpp"

#include <chrono>
#include <cinttypes>
#include <filesystem>
#include <stdexcept>
#include <utility>
#include <vector>

#include "ros2_medkit_fault_manager/correlation/config_parser.hpp"
#ifdef POSTGRES_SUPPORT
#include "ros2_medkit_fault_manager/postgres_fault_storage.hpp"
#endif
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"

namespace ros2_medkit_fault_manager::core {

namespace {

int64_t wall_clock_ns() {
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
}

std::unique_ptr<FaultStorage> create_storage(const FaultManagerConfig & config, LogSink & sink) {
  const std::string & logger = config.logger_name;
  if (config.storage_type == "memory") {
    logf(sink, LogLevel::kInfo, logger, "Using in-memory fault storage");
    return std::make_unique<InMemoryFaultStorage>();
  }

  if (config.storage_type == "sqlite") {
    if (config.database_path != ":memory:") {
      std::filesystem::path db_path(config.database_path);
      auto parent_dir = db_path.parent_path();
      std::string parent_dir_str = parent_dir.string();
      if (!parent_dir_str.empty() && !std::filesystem::exists(parent_dir)) {
        try {
          std::filesystem::create_directories(parent_dir);
          logf(sink, LogLevel::kInfo, logger, "Created database directory: %s", parent_dir_str.c_str());
        } catch (const std::filesystem::filesystem_error & e) {
          logf(sink, LogLevel::kError, logger,
               "Failed to create database directory for fault manager storage at '%s': %s", parent_dir_str.c_str(),
               e.what());
          throw;
        }
      }
    }

    logf(sink, LogLevel::kInfo, logger, "Using SQLite fault storage: %s", config.database_path.c_str());
    return std::make_unique<SqliteFaultStorage>(config.database_path, sink);
  }

#ifdef POSTGRES_SUPPORT
  if (config.storage_type == "postgres") {
    auto postgres_fault_storage = std::make_unique<PgFaultStorage>(config.database_url, sink);
    logf(sink, LogLevel::kInfo, logger, "Using PostgreSQL fault storage (%s)",
         postgres_fault_storage->target().c_str());
    if (!postgres_fault_storage->connected()) {
      logf(sink, LogLevel::kWarn, logger, "PostgreSQL is unreachable: running without fault storage until it connects");
    }
    return postgres_fault_storage;
  }
#else
  if (config.storage_type == "postgres") {
    throw std::runtime_error("storage_type 'postgres' needs a build with -DPOSTGRES_SUPPORT=ON");
  }
#endif

  logf(sink, LogLevel::kError, logger, "Unknown storage_type '%s', falling back to in-memory",
       config.storage_type.c_str());
  return std::make_unique<InMemoryFaultStorage>();
}

std::unique_ptr<FaultAuditLog> create_audit_log(const FaultManagerConfig & config, LogSink & sink) {
  const std::string & logger = config.logger_name;
  if (!config.audit_log_enabled) {
    return nullptr;  // No table, no file, no write overhead when off.
  }

  const bool confirmed_only = (config.audit_log_transitions == "confirmed_only");
  const int64_t retention = config.audit_log_retention_max_records;

  // Path: explicit override, else a sibling of the fault DB, else :memory:.
  std::string audit_path = config.audit_log_database_path;
  if (audit_path.empty()) {
    if (config.database_path == ":memory:" || (config.storage_type != "sqlite" && config.storage_type != "postgres")) {
      audit_path = ":memory:";
    } else {
      std::filesystem::path base(config.database_path);
      audit_path = (base.parent_path() / "fault_audit.db").string();
    }
  }

  if (audit_path != ":memory:") {
    std::filesystem::path p(audit_path);
    auto parent = p.parent_path();
    if (!parent.empty() && !std::filesystem::exists(parent)) {
      try {
        std::filesystem::create_directories(parent);
      } catch (const std::filesystem::filesystem_error & e) {
        logf(sink, LogLevel::kError, logger, "Failed to create audit log directory '%s': %s", parent.string().c_str(),
             e.what());
        throw;
      }
    }
  }

  try {
    auto log = std::make_unique<FaultAuditLog>(audit_path, retention);
    logf(sink, LogLevel::kInfo, logger,
         "Fault audit log enabled: %s (transitions=%s, retention=%" PRId64 ", fail_closed=%s, resume_seq=%" PRId64 ")",
         audit_path.c_str(), confirmed_only ? "confirmed_only" : "all", retention,
         config.audit_log_fail_closed ? "true" : "false", log->head().seq);

    // Appended directly, so it is recorded even in confirmed_only mode. A failure does not abort startup.
    AuditEvent marker;
    marker.fault_code = "__audit__";
    marker.transition = kTransitionLoggingActivated;
    marker.status = "ACTIVE";
    marker.source_id = "fault_manager";
    marker.description = "audit logging activated";
    marker.occurred_at_ns = wall_clock_ns();
    try {
      log->append(marker);
    } catch (const std::exception & e) {
      logf(sink, LogLevel::kWarn, logger, "Failed to append audit 'logging_activated' marker: %s", e.what());
    }
    return log;
  } catch (const std::exception & e) {
    logf(sink, LogLevel::kError, logger, "Failed to open fault audit log '%s': %s", audit_path.c_str(), e.what());
    throw;
  }
}

std::unique_ptr<correlation::CorrelationEngine> create_correlation(const FaultManagerConfig & config, LogSink & sink) {
  const std::string & logger = config.logger_name;
  const std::string & config_file = config.correlation_config_file;

  if (config_file.empty()) {
    logf(sink, LogLevel::kDebug, logger, "Correlation disabled: no config_file specified");
    return nullptr;
  }

  if (!std::filesystem::exists(config_file)) {
    logf(sink, LogLevel::kError, logger, "Correlation config file not found: %s", config_file.c_str());
    return nullptr;
  }

  try {
    logf(sink, LogLevel::kInfo, logger, "Loading correlation config from: %s", config_file.c_str());
    auto parsed = correlation::parse_config_file(config_file);

    if (!parsed.enabled) {
      logf(sink, LogLevel::kInfo, logger, "Correlation explicitly disabled in config");
      return nullptr;
    }

    auto validation = correlation::validate_config(parsed);
    for (const auto & warning : validation.warnings) {
      logf(sink, LogLevel::kWarn, logger, "Correlation config: %s", warning.c_str());
    }
    if (!validation.valid) {
      for (const auto & error : validation.errors) {
        logf(sink, LogLevel::kError, logger, "Correlation config error: %s", error.c_str());
      }
      return nullptr;
    }

    logf(sink, LogLevel::kInfo, logger, "Correlation engine enabled (default_window=%ums, patterns=%zu, rules=%zu)",
         parsed.default_window_ms, parsed.patterns.size(), parsed.rules.size());

    return std::make_unique<correlation::CorrelationEngine>(parsed);

  } catch (const std::exception & e) {
    logf(sink, LogLevel::kError, logger, "Failed to load correlation config: %s", e.what());
    return nullptr;
  }
}

/// Merges @p entry over @p global field by field and resets an invalid merge to the safe defaults.
/// Returns false when the merge was invalid.
template <typename Entry>
bool sanitize_override(const DebounceConfig & global, Entry & entry) {
  DebounceConfig merged = global;
  if (entry.confirmation_threshold) {
    merged.confirmation_threshold = *entry.confirmation_threshold;
  }
  if (entry.healing_threshold) {
    merged.healing_threshold = *entry.healing_threshold;
  }
  if (sanitize_debounce_config(merged)) {
    return true;
  }
  if (entry.confirmation_threshold) {
    entry.confirmation_threshold = merged.confirmation_threshold;
  }
  if (entry.healing_threshold) {
    entry.healing_threshold = merged.healing_threshold;
  }
  return false;
}

}  // namespace

CoreParts build_core_parts(const FaultManagerConfig & config, LogSink & sink) {
  const std::string & logger = config.logger_name;
  CoreParts parts;

  parts.storage = create_storage(config, sink);
  parts.audit = create_audit_log(config, sink);

  if (config.snapshots_max_per_fault > 0) {
    parts.storage->set_max_snapshots_per_fault(static_cast<size_t>(config.snapshots_max_per_fault));
  }
  // Independent of the cap: max_per_fault 0 means unlimited.
  parts.storage->set_retain_snapshots_on_clear(config.snapshots_retain_on_clear);

  if (config.snapshots_rosbag_enabled) {
    // The cap lives in the storage backend: it has to be atomic with the insert, and the unlink
    // has to happen after the backend's commit.
    parts.storage->set_max_rosbags_per_fault(static_cast<size_t>(config.snapshots_rosbag_max_bags_per_fault));
  }

  // Applying the near-miss bound also trims a series an earlier run left over it, which deletes
  // history for good. Unreachable storage skips the trim; the bound still applies to new entries.
  size_t evicted_near_misses = 0;
  try {
    evicted_near_misses =
        parts.storage->set_max_near_misses_per_fault(static_cast<size_t>(config.near_miss_max_per_fault));
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    logf(sink, LogLevel::kWarn, logger, "Skipped the startup near-miss trim, fault storage unavailable: %s", e.what());
  }
  if (evicted_near_misses > 0) {
    logf(sink, LogLevel::kWarn, logger,
         "near_miss.max_per_fault=%ld dropped %zu stored near-miss entries that exceeded the bound. "
         "Raise the bound or set it to 0 before restarting if that history was needed.",
         static_cast<long>(config.near_miss_max_per_fault), evicted_near_misses);
  }

  DebounceConfig & global = parts.global_debounce;
  global.confirmation_threshold = static_cast<int32_t>(config.confirmation_threshold);
  global.healing_enabled = config.healing_enabled;
  global.healing_threshold = static_cast<int32_t>(config.healing_threshold);
  global.auto_confirm_after_sec = config.auto_confirm_after_sec;
  if (!sanitize_debounce_config(global)) {
    logf(sink, LogLevel::kWarn, logger,
         "Invalid debounce thresholds (need confirmation_threshold < 0 <= healing_threshold); using safe "
         "defaults: confirmation=%d healing=%d",
         global.confirmation_threshold, global.healing_threshold);
  }
  parts.storage->set_debounce_config(global);

  if (!config.entity_thresholds_config_file.empty()) {
    auto entries = EntityThresholdResolver::load_from_yaml(config.entity_thresholds_config_file, sink);
    // The merge is field by field, so an override can break confirmation_threshold < 0 <= healing_threshold
    // even when the global config is valid.
    for (auto & entry : entries) {
      if (!sanitize_override(global, entry)) {
        logf(sink, LogLevel::kWarn, logger,
             "Entity '%s' debounce thresholds invalid (need confirmation_threshold < 0 <= healing_threshold); "
             "using safe defaults",
             entry.prefix.c_str());
      }
    }
    if (!entries.empty()) {
      parts.entity_resolver = std::make_unique<EntityThresholdResolver>(std::move(entries));
      logf(sink, LogLevel::kInfo, logger, "Loaded %zu per-entity threshold overrides from %s",
           parts.entity_resolver->size(), config.entity_thresholds_config_file.c_str());
      if (config.auto_confirm_after_sec > 0.0) {
        logf(sink, LogLevel::kWarn, logger,
             "Per-entity thresholds are configured but auto_confirm_after_sec=%.1f is also set. "
             "Auto-confirmation will bypass entity-specific debounce policies for PREFAILED faults.",
             config.auto_confirm_after_sec);
      }
    }
  }

  // Applied after the entity layer, so a code listed here debounces the same whichever source reports it.
  if (!config.fault_thresholds_config_file.empty()) {
    auto entries = FaultCodeThresholdResolver::load_from_yaml(config.fault_thresholds_config_file, sink);
    for (auto & entry : entries) {
      if (!sanitize_override(global, entry)) {
        logf(sink, LogLevel::kWarn, logger,
             "Fault code '%s' debounce thresholds invalid (need confirmation_threshold < 0 <= "
             "healing_threshold); using safe defaults",
             entry.fault_code.c_str());
      }
    }
    if (!entries.empty()) {
      parts.fault_code_resolver = std::make_unique<FaultCodeThresholdResolver>(std::move(entries));
      logf(sink, LogLevel::kInfo, logger, "Loaded %zu per-fault_code threshold overrides from %s",
           parts.fault_code_resolver->size(), config.fault_thresholds_config_file.c_str());
      if (config.auto_confirm_after_sec > 0.0) {
        logf(sink, LogLevel::kWarn, logger,
             "Per-fault_code thresholds are configured but auto_confirm_after_sec=%.1f is also set. "
             "Auto-confirmation will bypass fault-code debounce policies for PREFAILED faults.",
             config.auto_confirm_after_sec);
      }
    }
  }

  parts.correlation = create_correlation(config, sink);
  return parts;
}

void reclassify_stale_healed(FaultStorage & storage, const FaultCodeThresholdResolver * fault_code_resolver,
                             const DebounceConfig & global_debounce, bool audit_enabled, LogSink & sink,
                             const std::string & logger, const ReclassifyAuditFn & audit) {
  try {
    const auto reclassified =
        storage.reclassify_healed_as_cleared([fault_code_resolver, &global_debounce](const std::string & fault_code) {
          if (fault_code_resolver) {
            return fault_code_resolver->resolve(fault_code, global_debounce).healing_enabled;
          }
          return global_debounce.healing_enabled;
        });
    if (!reclassified.empty()) {
      if (audit_enabled) {
        const int64_t reclassified_at_ns = wall_clock_ns();
        for (const auto & fault_code : reclassified) {
          auto fault = storage.get_fault(fault_code);
          if (fault) {
            audit(*fault, reclassified_at_ns);
          }
        }
      }
      logf(sink, LogLevel::kInfo, logger,
           "Reclassified %zu stale HEALED fault(s) as CLEARED, because healing is disabled for their codes",
           reclassified.size());
    }
  } catch (const FaultStorage::IgnorableConnectionException & e) {
    logf(sink, LogLevel::kWarn, logger, "Startup HEALED reclassification not done, fault storage unavailable: %s",
         e.what());
  }
}

}  // namespace ros2_medkit_fault_manager::core
