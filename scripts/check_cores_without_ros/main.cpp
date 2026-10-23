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

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/core/fault_manager_config.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_startup.hpp"
#include "ros2_medkit_fault_manager/core/log_sink.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_fault_reporter/core/reporter_core.hpp"

namespace core = ros2_medkit_fault_reporter::core;
using ros2_medkit_fault_reporter::FilterConfig;

namespace {
class StderrSink : public ros2_medkit_fault_manager::core::LogSink {
 public:
  void log(ros2_medkit_fault_manager::core::LogLevel /*level*/, const std::string & logger,
           const std::string & message) override {
    std::fprintf(stderr, "[%s] %s\n", logger.c_str(), message.c_str());
  }
};

class RecordingEventSink : public ros2_medkit_fault_manager::core::EventSink {
 public:
  void publish(const ros2_medkit::fault_detection::FaultEventRecord & event) override {
    types.push_back(event.event_type);
  }
  std::vector<std::string> types;
};

class IgnoreCaptureHooks : public ros2_medkit_fault_manager::core::CaptureHooks {
 public:
  void on_confirmed(const std::string & /*code*/) override {
  }
  void on_prefailed(const std::string & /*code*/) override {
  }
  void on_cleared(const std::string & /*code*/) override {
  }
  void forget_cooldown(const std::string & /*code*/) override {
  }
};

class FixedWallClock : public ros2_medkit_fault_manager::core::WallClock {
 public:
  int64_t now_ns() const override {
    return 2000000000LL;
  }
};

bool check(bool ok, const char * what) {
  std::printf("%s: %s\n", ok ? "ok  " : "FAIL", what);
  return ok;
}
}  // namespace

static int run() {
  int sunk = 0;
  FilterConfig cfg;
  cfg.default_threshold = 2;
  core::ReporterCore reporter("no_ros", cfg, [&sunk](const ros2_medkit::fault_detection::FaultReport &) {
    ++sunk;
  });

  bool ok = true;
  ok &= check(!reporter.report("X", 1, "d"), "first report below threshold is filtered");
  ok &= check(reporter.report("X", 1, "d"), "second report reaches the sink");
  ok &= check(sunk == 1, "exactly one report reached the sink");

  int before = sunk;
  ok &= check(!reporter.report_passed("X"), "one PASSED below threshold is filtered");
  ok &= check(sunk == before, "no PASSED reached the sink");
  ok &= check(!reporter.report("", 1, "d"), "empty fault code is not forwarded");

  // Fault manager core: a FAILED report with threshold -1 confirms and lists as a neutral record.
  namespace fm = ros2_medkit_fault_manager;
  namespace fd = ros2_medkit::fault_detection;
  const auto db_path = std::filesystem::temp_directory_path() / "check_cores_without_ros_faults.db";
  std::filesystem::remove(db_path);
  {
    StderrSink sink;
    fm::SqliteFaultStorage storage(db_path.string(), sink);
    fm::DebounceConfig debounce;
    debounce.confirmation_threshold = -1;
    ok &= check(storage.report_fault_event("NO_ROS_FAULT", fd::ReportKind::kFailed, fd::severity::kError, "d", "/src",
                                           1000000000LL, debounce),
                "a FAILED report creates the fault");
    const auto faults = storage.list_faults(false, 0, {});
    ok &= check(faults.size() == 1, "list_faults returns one record");
    ok &= check(!faults.empty() && faults[0].status == fd::fault_status::kConfirmed, "the record is CONFIRMED");
    ok &= check(!faults.empty() && faults[0].first_occurred_ns == 1000000000LL, "the record keeps its nanosecond time");
  }
  std::filesystem::remove(db_path);

  // Fault manager config rules and startup parts.
  {
    fm::core::FaultManagerConfig config;
    config.confirmation_threshold = 5;
    config.auto_confirm_after_sec = std::nan("");
    const auto notices = config.validate();
    ok &= check(config.confirmation_threshold == -5, "confirmation_threshold 5 becomes -5");
    ok &= check(config.auto_confirm_after_sec == 0.0, "NaN auto_confirm_after_sec falls back to 0");
    ok &= check(notices.size() == 2, "two notices: the sign flip and the NaN fallback");

    const auto parts_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_parts.db";
    std::filesystem::remove(parts_db);
    StderrSink sink;
    fm::core::FaultManagerConfig sqlite_config;
    sqlite_config.storage_type = "sqlite";
    sqlite_config.database_path = parts_db.string();
    {
      auto parts = fm::core::build_core_parts(sqlite_config, sink);
      ok &= check(parts.storage != nullptr, "build_core_parts opens a sqlite storage");
      ok &= check(parts.audit == nullptr && parts.correlation == nullptr, "audit and correlation stay off by default");
    }
    std::filesystem::remove(parts_db);
  }

  // Fault manager core: report, list and clear through the orchestration, SQLite storage.
  {
    namespace fmc = ros2_medkit_fault_manager::core;
    const auto core_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_core.db";
    std::filesystem::remove(core_db);
    StderrSink sink;
    fmc::FaultManagerConfig core_config;
    core_config.storage_type = "sqlite";
    core_config.database_path = core_db.string();
    core_config.confirmation_threshold = -1;
    RecordingEventSink events;
    IgnoreCaptureHooks hooks;
    FixedWallClock clock;
    {
      fmc::FaultManagerCore fault_core(core_config, fmc::build_core_parts(core_config, sink), events, hooks, clock,
                                       sink);
      fd::FaultReport report;
      report.fault_code = "CORE_FAULT";
      report.kind = fd::ReportKind::kFailed;
      report.severity = fd::severity::kError;
      report.description = "d";
      report.source_id = "/src";
      ok &= check(fault_core.report(report).accepted, "core accepts a FAILED report");
      const auto listed = fault_core.list(fmc::ListQuery{});
      ok &= check(listed.faults.size() == 1 && listed.faults[0].status == fd::fault_status::kConfirmed,
                  "core list returns the fault CONFIRMED");
      ok &= check(events.types.size() == 1 && events.types[0] == fd::fault_event_type::kConfirmed,
                  "the sink saw exactly one fault_confirmed");
      const auto cleared = fault_core.clear("CORE_FAULT", false);
      ok &= check(cleared.success, "core clear succeeds");
      ok &= check(events.types.size() == 2 && events.types[1] == fd::fault_event_type::kCleared,
                  "the sink saw fault_cleared after the clear");
    }
    std::filesystem::remove(core_db);
  }

  return ok ? 0 : 1;
}

int main() {
  try {
    return run();
  } catch (...) {
    std::printf("FAIL: the check threw an exception\n");
    return 1;
  }
}
