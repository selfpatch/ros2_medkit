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

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "ros2_medkit_fault_manager/core/fault_manager_core.hpp"
#include "ros2_medkit_fault_manager/core/fault_manager_startup.hpp"

namespace fd = ros2_medkit::fault_detection;
namespace core = ros2_medkit_fault_manager::core;

namespace {

class NullLogSink : public core::LogSink {
 public:
  void log(core::LogLevel /*level*/, const std::string & /*logger*/, const std::string & /*message*/) override {
  }
};

class RecordingEventSink : public core::EventSink {
 public:
  void publish(const fd::FaultEventRecord & event) override {
    events.push_back(event);
  }
  std::vector<fd::FaultEventRecord> events;
};

class NoCaptureHooks : public core::CaptureHooks {
 public:
  core::CaptureResult capture_now(const std::string & /*code*/) override {
    return {};
  }
  void on_prefailed(const std::string & /*code*/) override {
  }
  void on_cleared(const std::string & /*code*/) override {
  }
};

class FixedClock : public core::WallClock {
 public:
  int64_t now_ns() const override {
    return 1000000000LL;
  }
};

/// One core on in-memory storage: the first FAILED confirms, the first PASSED heals, and one
/// hierarchical rule makes SYMPTOM_* a symptom of ROOT.
struct Fixture {
  std::string config_path;
  NullLogSink log;
  RecordingEventSink events;
  NoCaptureHooks hooks;
  FixedClock clock;
  std::unique_ptr<core::FaultManagerCore> core;

  explicit Fixture(bool auto_clear_with_root, int confirmation_threshold = -1) {
    config_path = testing::TempDir() + "heal_correlation_" + (auto_clear_with_root ? "on" : "off") + ".yaml";
    std::ofstream file(config_path);
    file << "correlation:\n"
            "  enabled: true\n"
            "  patterns:\n"
            "    symptoms:\n"
            "      codes: [\"SYMPTOM_*\"]\n"
            "  rules:\n"
            "    - id: root_rule\n"
            "      mode: hierarchical\n"
            "      window_ms: 60000\n"
            "      root_cause:\n"
            "        codes: [\"ROOT\"]\n"
            "      symptoms:\n"
            "        - pattern: symptoms\n"
            "      mute_symptoms: true\n"
            "      auto_clear_with_root: "
         << (auto_clear_with_root ? "true" : "false") << "\n";
    file.close();

    core::FaultManagerConfig config;
    config.storage_type = "memory";
    config.confirmation_threshold = confirmation_threshold;
    config.healing_enabled = true;
    config.healing_threshold = 0;
    config.correlation_config_file = config_path;
    core = std::make_unique<core::FaultManagerCore>(config, core::build_core_parts(config, log), events, hooks, clock,
                                                    log);
  }

  ~Fixture() {
    std::remove(config_path.c_str());
  }

  void report(const std::string & code, fd::ReportKind kind) {
    fd::FaultReport r;
    r.fault_code = code;
    r.kind = kind;
    r.severity = fd::severity::kError;
    r.description = "test";
    r.source_id = "/test";
    ASSERT_TRUE(core->report(r).accepted);
  }

  core::ListOutcome list() {
    core::ListQuery query;
    query.statuses = {fd::fault_status::kConfirmed};
    query.include_muted = true;
    return core->list(query);
  }
};

TEST(FaultManagerCoreCorrelation, HealedRootClearsItsSymptoms) {
  Fixture f(true);
  ASSERT_NE(f.core->correlation(), nullptr);
  f.report("ROOT", fd::ReportKind::kFailed);
  f.report("SYMPTOM_A", fd::ReportKind::kFailed);
  f.report("SYMPTOM_B", fd::ReportKind::kFailed);
  ASSERT_EQ(f.list().muted_count, 2u);

  f.report("ROOT", fd::ReportKind::kPassed);

  auto after = f.list();
  EXPECT_TRUE(after.faults.empty());
  EXPECT_EQ(after.muted_count, 0u);
  EXPECT_EQ(f.core->get("ROOT").fault.status, fd::fault_status::kHealed);
  EXPECT_EQ(f.core->get("SYMPTOM_A").fault.status, fd::fault_status::kCleared);
  EXPECT_EQ(f.core->get("SYMPTOM_B").fault.status, fd::fault_status::kCleared);

  // The root's end event names the symptoms that went with it, as a clear does.
  ASSERT_FALSE(f.events.events.empty());
  const auto & last = f.events.events.back();
  EXPECT_EQ(last.fault.fault_code, "ROOT");
  EXPECT_EQ(last.event_type, fd::fault_event_type::kCleared);
  EXPECT_EQ(last.auto_cleared_codes.size(), 2u);
}

TEST(FaultManagerCoreCorrelation, HealedRootWithoutAutoClearUnmutesSymptoms) {
  Fixture f(false);
  f.report("ROOT", fd::ReportKind::kFailed);
  f.report("SYMPTOM_A", fd::ReportKind::kFailed);
  ASSERT_EQ(f.list().muted_count, 1u);

  f.report("ROOT", fd::ReportKind::kPassed);

  auto after = f.list();
  EXPECT_EQ(after.muted_count, 0u);
  ASSERT_EQ(after.faults.size(), 1u);
  EXPECT_EQ(after.faults[0].fault_code, "SYMPTOM_A");
  EXPECT_TRUE(f.events.events.back().auto_cleared_codes.empty());
}

TEST(FaultManagerCoreCorrelation, HealedSymptomLeavesTheMutedList) {
  Fixture f(true);
  f.report("ROOT", fd::ReportKind::kFailed);
  f.report("SYMPTOM_A", fd::ReportKind::kFailed);
  ASSERT_EQ(f.list().muted_count, 1u);

  f.report("SYMPTOM_A", fd::ReportKind::kPassed);

  auto after = f.list();
  EXPECT_EQ(after.muted_count, 0u);
  ASSERT_EQ(after.faults.size(), 1u);
  EXPECT_EQ(after.faults[0].fault_code, "ROOT");
}

TEST(FaultManagerCoreCorrelation, HealedSymptomIsNotClearedWithItsFormerRoot) {
  Fixture f(true);
  f.report("ROOT", fd::ReportKind::kFailed);
  f.report("SYMPTOM_A", fd::ReportKind::kFailed);
  f.report("SYMPTOM_A", fd::ReportKind::kPassed);

  f.report("ROOT", fd::ReportKind::kPassed);

  EXPECT_EQ(f.core->get("SYMPTOM_A").fault.status, fd::fault_status::kHealed);
  EXPECT_TRUE(f.events.events.back().auto_cleared_codes.empty());
}

TEST(FaultManagerCoreCorrelation, FailedReportThatKeepsAFaultHealedDoesNotMuteIt) {
  // Threshold -2: one FAILED after healing leaves the status HEALED.
  Fixture f(true, -2);
  f.report("ROOT", fd::ReportKind::kFailed);
  f.report("ROOT", fd::ReportKind::kFailed);
  f.report("SYMPTOM_A", fd::ReportKind::kFailed);
  f.report("SYMPTOM_A", fd::ReportKind::kFailed);
  ASSERT_EQ(f.list().muted_count, 1u);
  f.report("SYMPTOM_A", fd::ReportKind::kPassed);
  f.report("SYMPTOM_A", fd::ReportKind::kPassed);
  ASSERT_EQ(f.core->get("SYMPTOM_A").fault.status, fd::fault_status::kHealed);
  ASSERT_EQ(f.list().muted_count, 0u);

  f.report("SYMPTOM_A", fd::ReportKind::kFailed);

  EXPECT_EQ(f.core->get("SYMPTOM_A").fault.status, fd::fault_status::kHealed);
  EXPECT_EQ(f.list().muted_count, 0u);
}

}  // namespace
