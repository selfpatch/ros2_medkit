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

#include <cstdio>
#include <filesystem>
#include <string>

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
