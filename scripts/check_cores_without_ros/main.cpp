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
#include <fstream>
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
  ros2_medkit_fault_manager::core::CaptureResult capture_now(const std::string & code) override {
    captured.push_back(code);
    return {ros2_medkit_fault_manager::core::CaptureEnqueue::kAccepted, {}};
  }
  void on_prefailed(const std::string & /*code*/) override {
  }
  void on_cleared(const std::string & /*code*/) override {
  }
  std::vector<std::string> captured;
};

class SettableWallClock : public ros2_medkit_fault_manager::core::WallClock {
 public:
  int64_t now_ns() const override {
    return now;
  }
  int64_t now{0};
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

    fm::core::FaultManagerConfig wide;
    wide.confirmation_threshold = -4294967297LL;
    wide.healing_threshold = 4294967296LL;
    const auto wide_notices = wide.validate();
    ok &= check(wide.confirmation_threshold == -2147483647LL && wide.healing_threshold == 2147483647LL,
                "int64 thresholds outside the int32 range clamp to its ends");
    ok &= check(wide_notices.size() == 2 &&
                    wide_notices[0].text.find("confirmation_threshold must be in [-2147483647, 2147483647]") == 0,
                "the clamp notice names the range");

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

  // Fault manager core: tick drives auto-confirmation and correlation cleanup, each on its own period.
  {
    namespace fmc = ros2_medkit_fault_manager::core;
    constexpr int64_t kMs = 1000000LL;
    const int64_t t0 = 10000000000LL;
    const auto tick_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_tick.db";
    const auto corr_yaml = std::filesystem::temp_directory_path() / "check_cores_without_ros_corr.yaml";
    std::filesystem::remove(tick_db);
    {
      std::ofstream out(corr_yaml.string());
      out << "correlation:\n  enabled: true\n  default_window_ms: 500\n  patterns:\n    sensor_errors:\n"
             "      codes: [\"SENSOR_*\"]\n  rules:\n    - id: r1\n      name: R1\n      mode: hierarchical\n"
             "      root_cause:\n        codes: [\"ESTOP_001\"]\n      symptoms:\n"
             "        - pattern: sensor_errors\n      window_ms: 2000\n      mute_symptoms: true\n";
    }
    StderrSink sink;
    fmc::FaultManagerConfig tick_config;
    tick_config.storage_type = "sqlite";
    tick_config.database_path = tick_db.string();
    tick_config.confirmation_threshold = -3;
    tick_config.auto_confirm_after_sec = 1.0;
    tick_config.correlation_config_file = corr_yaml.string();
    tick_config.correlation_cleanup_interval_sec = 5.0;
    RecordingEventSink events;
    IgnoreCaptureHooks hooks;
    SettableWallClock clock;
    clock.now = t0;
    {
      fmc::FaultManagerCore fault_core(tick_config, fmc::build_core_parts(tick_config, sink), events, hooks, clock,
                                       sink);
      fd::FaultReport report;
      report.fault_code = "TICK_FAULT";
      report.kind = fd::ReportKind::kFailed;
      report.severity = fd::severity::kError;
      report.description = "d";
      report.source_id = "/src";
      ok &= check(fault_core.report(report).accepted, "tick: core accepts a FAILED report");
      fmc::ListQuery all_statuses;
      all_statuses.statuses = {fd::fault_status::kPrefailed};
      const auto before = fault_core.list(all_statuses);
      ok &= check(before.faults.size() == 1 && before.faults[0].status == fd::fault_status::kPrefailed,
                  "tick: the fault is PREFAILED");

      fault_core.tick(t0 + 999 * kMs);
      ok &= check(events.types.empty() && hooks.captured.empty(), "tick at 999 ms: no event, no capture");
      ok &= check(fault_core.auto_confirm_runs() == 0, "tick at 999 ms: auto-confirm job has not run");
      fault_core.tick(t0 + 1001 * kMs);
      ok &= check(events.types.size() == 1 && events.types[0] == fd::fault_event_type::kConfirmed,
                  "tick at 1001 ms: one fault_confirmed");
      ok &= check(hooks.captured.size() == 1 && hooks.captured[0] == "TICK_FAULT",
                  "tick at 1001 ms: capture requested once");
      ok &= check(fault_core.auto_confirm_runs() == 1, "tick at 1001 ms: auto-confirm job ran once");

      ok &= check(fault_core.correlation_cleanup_runs() == 0, "cleanup has not run before its interval");
      fault_core.tick(t0 + 4999 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 0, "tick at 4999 ms: cleanup not due");
      fault_core.tick(t0 + 5001 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 1, "tick at 5001 ms: cleanup ran once");
      fault_core.tick(t0 + 5002 * kMs);
      fault_core.tick(t0 + 9000 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 1, "ticks inside the interval: cleanup does not repeat");
      fault_core.tick(t0 + 10001 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 2, "tick at 10001 ms: cleanup ran a second time");
      ok &= check(events.types.size() == 1 && hooks.captured.size() == 1, "later ticks emit no second confirmation");

      // A tick earlier than the last one runs nothing and pulls the due times back to now + interval.
      const uint64_t confirm_runs = fault_core.auto_confirm_runs();
      fault_core.tick(t0 + 2000 * kMs);
      ok &= check(fault_core.auto_confirm_runs() == confirm_runs && fault_core.correlation_cleanup_runs() == 2,
                  "backwards tick: no job runs");
      ok &= check(events.types.size() == 1 && hooks.captured.size() == 1, "backwards tick: no event, no capture");
      fault_core.tick(t0 + 6999 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 2, "after a backwards tick: cleanup waits one interval");
      fault_core.tick(t0 + 7001 * kMs);
      ok &= check(fault_core.correlation_cleanup_runs() == 3, "after a backwards tick: cleanup resumes");
      ok &= check(events.types.size() == 1, "after a backwards tick: still one fault_confirmed");
    }
    std::filesystem::remove(tick_db);
    std::filesystem::remove(corr_yaml);
  }

  // A cleanup interval near the validation maximum must not overflow the due time.
  {
    namespace fmc = ros2_medkit_fault_manager::core;
    const int64_t epoch_now = 1790000000LL * 1000000000LL;
    const auto huge_db = std::filesystem::temp_directory_path() / "check_cores_without_ros_huge.db";
    const auto corr_yaml = std::filesystem::temp_directory_path() / "check_cores_without_ros_corr_huge.yaml";
    std::filesystem::remove(huge_db);
    {
      std::ofstream out(corr_yaml.string());
      out << "correlation:\n  enabled: true\n  default_window_ms: 500\n  patterns:\n    sensor_errors:\n"
             "      codes: [\"SENSOR_*\"]\n  rules:\n    - id: r1\n      name: R1\n      mode: hierarchical\n"
             "      root_cause:\n        codes: [\"ESTOP_001\"]\n      symptoms:\n"
             "        - pattern: sensor_errors\n      window_ms: 2000\n      mute_symptoms: true\n";
    }
    StderrSink sink;
    fmc::FaultManagerConfig huge_config;
    huge_config.storage_type = "sqlite";
    huge_config.database_path = huge_db.string();
    huge_config.correlation_config_file = corr_yaml.string();
    huge_config.correlation_cleanup_interval_sec = 9.0e9;
    RecordingEventSink events;
    IgnoreCaptureHooks hooks;
    SettableWallClock clock;
    clock.now = epoch_now;
    {
      fmc::FaultManagerCore fault_core(huge_config, fmc::build_core_parts(huge_config, sink), events, hooks, clock,
                                       sink);
      fault_core.tick(epoch_now + 1000000000LL);
      ok &= check(fault_core.correlation_cleanup_runs() == 0, "cleanup interval 9e9 s: no run one second after start");
    }
    std::filesystem::remove(huge_db);
    std::filesystem::remove(corr_yaml);
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
