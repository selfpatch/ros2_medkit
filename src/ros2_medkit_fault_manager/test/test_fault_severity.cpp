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

/// Severity contract of a fault record, run against both storage backends.
///
/// A record (fault_code, owner) has one severity: the highest any FAILED report from its owner has
/// carried, within an occurrence and across occurrences. With critical_immediate_confirm, a report
/// on a record whose stored severity is CRITICAL confirms it, so no stored record is CRITICAL in
/// PREFAILED or PREPASSED. Another owner's record of the same code is untouched.

#include <gtest/gtest.h>

#include <sqlite3.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros2_medkit_fault_manager/fault_storage.hpp"
#include "ros2_medkit_fault_manager/sqlite_fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "test_log_sink.hpp"

namespace {

using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::FaultId;
using ros2_medkit_fault_manager::FaultStorage;
using ros2_medkit_fault_manager::InMemoryFaultStorage;
using ros2_medkit_fault_manager::SqliteFaultStorage;
using ros2_medkit_msgs::msg::Fault;

constexpr const char * kOwner = "/reporter";

constexpr int64_t kBaseNs = 1'700'000'000'000'000'000LL;

int64_t at(int64_t second) {
  return kBaseNs + second * 1'000'000'000LL;
}

DebounceConfig confirm_at(int32_t confirmation_threshold) {
  DebounceConfig config;
  config.confirmation_threshold = confirmation_threshold;
  return config;
}

bool pending(const std::string & status) {
  return status == Fault::STATUS_PREFAILED || status == Fault::STATUS_PREPASSED;
}

std::filesystem::path unique_temp_dir() {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dist;
  return std::filesystem::temp_directory_path() / ("medkit_severity_" + std::to_string(dist(gen)));
}

template <typename Backend>
struct BackendFactory;

template <>
struct BackendFactory<InMemoryFaultStorage> {
  static std::unique_ptr<FaultStorage> make(const std::filesystem::path & /*dir*/) {
    return std::make_unique<InMemoryFaultStorage>();
  }
};

template <>
struct BackendFactory<SqliteFaultStorage> {
  static std::unique_ptr<FaultStorage> make(const std::filesystem::path & dir) {
    return std::make_unique<SqliteFaultStorage>((dir / "faults.db").string(), test_log_sink());
  }
};

template <typename Backend>
class FaultSeverityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = unique_temp_dir();
    std::filesystem::create_directories(dir_);
    storage_ = BackendFactory<Backend>::make(dir_);
  }

  void TearDown() override {
    storage_.reset();
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  bool failed(const std::string & code, uint8_t severity, int64_t second, const DebounceConfig & config,
              const std::string & owner = kOwner) {
    return storage_->report_fault_event(FaultId{code, owner}, fd::ReportKind::kFailed, severity, "report", owner,
                                        at(second), config);
  }

  void passed(const std::string & code, int64_t second, const DebounceConfig & config,
              const std::string & owner = kOwner) {
    storage_->report_fault_event(FaultId{code, owner}, fd::ReportKind::kPassed, 0, "", owner, at(second), config);
  }

  bool clear(const std::string & code, const std::string & owner = kOwner) {
    return storage_->clear_fault(FaultId{code, owner});
  }

  fd::FaultRecord fault(const std::string & code, const std::string & owner = kOwner) {
    auto f = storage_->get_fault(FaultId{code, owner});
    EXPECT_TRUE(f.has_value()) << code << " " << owner;
    return f.value_or(fd::FaultRecord{});
  }

  /// Every stored CRITICAL record in a pending status.
  std::vector<fd::FaultRecord> pending_critical() {
    return storage_->list_faults(true, Fault::SEVERITY_CRITICAL, {Fault::STATUS_PREFAILED, Fault::STATUS_PREPASSED});
  }

  std::filesystem::path dir_;
  std::unique_ptr<FaultStorage> storage_;
};

using Backends = ::testing::Types<InMemoryFaultStorage, SqliteFaultStorage>;
TYPED_TEST_SUITE(FaultSeverityTest, Backends);

TYPED_TEST(FaultSeverityTest, CriticalThenClearedWarnConfirmsAtEveryThreshold) {
  for (const int32_t threshold : {-1, -2, -3}) {
    const auto config = confirm_at(threshold);
    ASSERT_TRUE(config.critical_immediate_confirm);
    const std::string code = "CRIT_CLEAR_WARN_T" + std::to_string(-threshold);
    SCOPED_TRACE(code);

    this->failed(code, Fault::SEVERITY_CRITICAL, 0, config);
    ASSERT_EQ(this->fault(code).status, Fault::STATUS_CONFIRMED);
    ASSERT_TRUE(this->clear(code));

    EXPECT_TRUE(this->failed(code, Fault::SEVERITY_WARN, 1, config)) << "a report on a CLEARED record is an occurrence";
    const auto f = this->fault(code);
    EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL) << "the record keeps the highest severity ever reported";
    EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED) << "a CRITICAL record confirms on its first report";
    EXPECT_EQ(f.occurrence_count, 2u);
    EXPECT_TRUE(this->pending_critical().empty());
  }
}

TYPED_TEST(FaultSeverityTest, TimeBasedConfirmationFindsACriticalRecordConfirmedAtTheReport) {
  auto config = confirm_at(-3);
  config.auto_confirm_after_sec = 5.0;
  this->storage_->set_debounce_config(config);

  this->failed("TIMED_CRIT", Fault::SEVERITY_CRITICAL, 0, config);
  ASSERT_TRUE(this->clear("TIMED_CRIT"));
  this->failed("TIMED_CRIT", Fault::SEVERITY_WARN, 1, config);
  this->failed("TIMED_WARN", Fault::SEVERITY_WARN, 1, config);
  EXPECT_EQ(this->fault("TIMED_CRIT").status, Fault::STATUS_CONFIRMED) << "confirmed by the report, not the sweep";
  EXPECT_EQ(this->fault("TIMED_WARN").status, Fault::STATUS_PREFAILED);

  EXPECT_TRUE(this->storage_->check_time_based_confirmation(at(3)).empty()) << "before the window ends";
  const auto swept = this->storage_->check_time_based_confirmation(at(10));
  ASSERT_EQ(swept.size(), 1u) << "the sweep confirms the pending record only";
  EXPECT_EQ(swept[0].fault_code, "TIMED_WARN");
  EXPECT_EQ(this->fault("TIMED_WARN").status, Fault::STATUS_CONFIRMED);
  const auto f = this->fault("TIMED_CRIT");
  EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(f.occurrence_count, 2u);
}

TYPED_TEST(FaultSeverityTest, AnotherOwnersCriticalDoesNotConfirmThisRecord) {
  const auto config = confirm_at(-3);

  this->failed("SHARED_CRIT", Fault::SEVERITY_CRITICAL, 0, config, "/a");
  ASSERT_TRUE(this->clear("SHARED_CRIT", "/a"));

  EXPECT_TRUE(this->failed("SHARED_CRIT", Fault::SEVERITY_WARN, 1, config, "/b")) << "/b owns no record yet";
  auto b = this->fault("SHARED_CRIT", "/b");
  EXPECT_EQ(b.severity, Fault::SEVERITY_WARN) << "severity is per record";
  EXPECT_EQ(b.status, Fault::STATUS_PREFAILED);

  this->failed("SHARED_CRIT", Fault::SEVERITY_WARN, 2, config, "/a");
  const auto a = this->fault("SHARED_CRIT", "/a");
  EXPECT_EQ(a.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(a.status, Fault::STATUS_CONFIRMED);
  b = this->fault("SHARED_CRIT", "/b");
  EXPECT_EQ(b.status, Fault::STATUS_PREFAILED) << "confirming /a leaves /b's record as it was";
  EXPECT_TRUE(this->pending_critical().empty());
}

TYPED_TEST(FaultSeverityTest, WarnCriticalWarnWithinAnOccurrence) {
  const auto config = confirm_at(-3);

  this->failed("W_C_W", Fault::SEVERITY_WARN, 0, config);
  EXPECT_EQ(this->fault("W_C_W").status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(this->fault("W_C_W").severity, Fault::SEVERITY_WARN);

  this->failed("W_C_W", Fault::SEVERITY_CRITICAL, 1, config);
  EXPECT_EQ(this->fault("W_C_W").status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(this->fault("W_C_W").severity, Fault::SEVERITY_CRITICAL) << "escalation within an occurrence";

  this->failed("W_C_W", Fault::SEVERITY_WARN, 2, config);
  const auto f = this->fault("W_C_W");
  EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL) << "a lower report does not lower the record";
  EXPECT_EQ(f.occurrence_count, 1u);
}

TYPED_TEST(FaultSeverityTest, WarnCriticalClearWarnConfirmsTheNewOccurrence) {
  const auto config = confirm_at(-3);

  this->failed("W_C_CLR_W", Fault::SEVERITY_WARN, 0, config);
  this->failed("W_C_CLR_W", Fault::SEVERITY_CRITICAL, 1, config);
  ASSERT_TRUE(this->clear("W_C_CLR_W"));

  this->failed("W_C_CLR_W", Fault::SEVERITY_WARN, 2, config);
  const auto f = this->fault("W_C_CLR_W");
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(f.occurrence_count, 2u);
  EXPECT_TRUE(this->pending_critical().empty());
}

TYPED_TEST(FaultSeverityTest, HealedCriticalRecordConfirmsOnTheNextFailed) {
  // Healing at 1: one FAILED confirms at -1, two PASSED reach 1 and heal.
  DebounceConfig config = confirm_at(-3);
  config.healing_enabled = true;
  config.healing_threshold = 1;

  this->failed("HEAL_CRIT", Fault::SEVERITY_CRITICAL, 0, config);
  this->passed("HEAL_CRIT", 1, config);
  this->passed("HEAL_CRIT", 2, config);
  ASSERT_EQ(this->fault("HEAL_CRIT").status, Fault::STATUS_HEALED);

  EXPECT_FALSE(this->failed("HEAL_CRIT", Fault::SEVERITY_WARN, 3, config)) << "HEALED is not an occurrence boundary";
  const auto f = this->fault("HEAL_CRIT");
  EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED) << "the latch does not hold a CRITICAL record back";
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(f.occurrence_count, 1u);
  EXPECT_EQ(f.first_occurred_ns, at(0));
}

TYPED_TEST(FaultSeverityTest, HealedWarnRecordStillWalksBackThroughTheLatch) {
  // Control for the case above: a non-CRITICAL record keeps the latch.
  DebounceConfig config = confirm_at(-3);
  config.healing_enabled = true;
  config.healing_threshold = 1;

  this->failed("HEAL_WARN", Fault::SEVERITY_WARN, 0, config);
  this->failed("HEAL_WARN", Fault::SEVERITY_WARN, 1, config);
  this->failed("HEAL_WARN", Fault::SEVERITY_WARN, 2, config);
  ASSERT_EQ(this->fault("HEAL_WARN").status, Fault::STATUS_CONFIRMED);
  for (int64_t s = 3; s < 7; ++s) {
    this->passed("HEAL_WARN", s, config);
  }
  ASSERT_EQ(this->fault("HEAL_WARN").status, Fault::STATUS_HEALED);

  this->failed("HEAL_WARN", Fault::SEVERITY_WARN, 7, config);
  EXPECT_EQ(this->fault("HEAL_WARN").status, Fault::STATUS_HEALED);
}

TYPED_TEST(FaultSeverityTest, CriticalBypassOffLeavesTheDebounceInCharge) {
  DebounceConfig config = confirm_at(-3);
  config.critical_immediate_confirm = false;

  this->failed("NO_BYPASS", Fault::SEVERITY_CRITICAL, 0, config);
  EXPECT_EQ(this->fault("NO_BYPASS").status, Fault::STATUS_PREFAILED);
  this->failed("NO_BYPASS", Fault::SEVERITY_WARN, 1, config);
  EXPECT_EQ(this->fault("NO_BYPASS").status, Fault::STATUS_PREFAILED);
}

TYPED_TEST(FaultSeverityTest, CriticalBypassOffLeavesTheReactivationToTheDebounce) {
  DebounceConfig config = confirm_at(-3);
  config.critical_immediate_confirm = false;

  for (const int64_t second : {0, 1, 2}) {
    this->failed("NO_BYPASS_REACT", Fault::SEVERITY_CRITICAL, second, config);
  }
  ASSERT_EQ(this->fault("NO_BYPASS_REACT").status, Fault::STATUS_CONFIRMED) << "confirmed by the debounce";
  ASSERT_TRUE(this->clear("NO_BYPASS_REACT"));

  this->failed("NO_BYPASS_REACT", Fault::SEVERITY_WARN, 3, config);
  auto f = this->fault("NO_BYPASS_REACT");
  EXPECT_EQ(f.status, Fault::STATUS_PREFAILED) << "without the bypass a CRITICAL record reactivates like any other";
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(f.occurrence_count, 2u);

  this->failed("NO_BYPASS_REACT", Fault::SEVERITY_WARN, 4, config);
  EXPECT_EQ(this->fault("NO_BYPASS_REACT").status, Fault::STATUS_PREFAILED);
  this->failed("NO_BYPASS_REACT", Fault::SEVERITY_WARN, 5, config);
  EXPECT_EQ(this->fault("NO_BYPASS_REACT").status, Fault::STATUS_CONFIRMED);
}

TYPED_TEST(FaultSeverityTest, SeverityNeverDropsWithinOrAcrossOccurrences) {
  for (const int32_t threshold : {-1, -3}) {
    const auto config = confirm_at(threshold);
    const std::string code = "NO_DROP_T" + std::to_string(-threshold);
    SCOPED_TRACE(code);

    this->failed(code, Fault::SEVERITY_ERROR, 0, config);
    this->failed(code, Fault::SEVERITY_WARN, 1, config);
    this->failed(code, Fault::SEVERITY_INFO, 2, config);
    EXPECT_EQ(this->fault(code).severity, Fault::SEVERITY_ERROR);

    ASSERT_TRUE(this->clear(code));
    this->failed(code, Fault::SEVERITY_WARN, 3, config);
    const auto f = this->fault(code);
    EXPECT_EQ(f.severity, Fault::SEVERITY_ERROR);
    EXPECT_EQ(f.occurrence_count, 2u);
    EXPECT_TRUE(
        this->storage_->list_faults(true, Fault::SEVERITY_WARN, {Fault::STATUS_PREFAILED, Fault::STATUS_CONFIRMED})
            .empty())
        << "the severity filter matches the stored severity";
  }
}

TYPED_TEST(FaultSeverityTest, NearMissRowsKeepTheReportedSeverity) {
  const auto config = confirm_at(-3);

  this->failed("NEAR", Fault::SEVERITY_ERROR, 0, config);
  this->failed("NEAR", Fault::SEVERITY_WARN, 1, config);
  ASSERT_EQ(this->fault("NEAR").severity, Fault::SEVERITY_ERROR);

  const auto series = this->storage_->get_near_misses(FaultId{"NEAR", kOwner});
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[0].severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(series[1].severity, Fault::SEVERITY_WARN) << "a near-miss row records its report";
}

// ---------------------------------------------------------------------------
// SQLite only: restart, the migration of a database without owners, and the confirmation time
// the in-memory backend does not store.
// ---------------------------------------------------------------------------

/// The faults table as releases before record identity created it: one row per code.
constexpr const char * kLegacyFaultsDdl =
    "CREATE TABLE faults (fault_code TEXT PRIMARY KEY, severity INTEGER NOT NULL, "
    "description TEXT NOT NULL, first_occurred_ns INTEGER NOT NULL, "
    "last_occurred_ns INTEGER NOT NULL, occurrence_count INTEGER NOT NULL, "
    "status TEXT NOT NULL, reporting_sources TEXT NOT NULL, "
    "debounce_counter INTEGER NOT NULL DEFAULT 0, "
    "last_failed_ns INTEGER NOT NULL DEFAULT 0, last_passed_ns INTEGER NOT NULL DEFAULT 0, "
    "confirmed_at_ns INTEGER NOT NULL DEFAULT 0);";

class SqliteSeverityTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = unique_temp_dir();
    std::filesystem::create_directories(dir_);
    reopen();
  }

  void TearDown() override {
    storage_.reset();
    std::error_code ec;
    std::filesystem::remove_all(dir_, ec);
  }

  /// Close the database and open it again, as a fault manager restart does.
  void reopen() {
    storage_.reset();
    storage_ = std::make_unique<SqliteFaultStorage>(db_path(), test_log_sink());
  }

  std::string db_path() const {
    return (dir_ / "faults.db").string();
  }

  struct LegacyRow {
    std::string code;
    int severity;
    std::string status;
    int64_t confirmed_at_ns;
  };

  /// Replace the database with a legacy one holding these rows, then open it, which migrates it.
  void open_legacy_database(const std::vector<LegacyRow> & rows) {
    storage_.reset();
    std::error_code ec;
    std::filesystem::remove(db_path(), ec);
    sqlite3 * raw = nullptr;
    EXPECT_EQ(sqlite3_open(db_path().c_str(), &raw), SQLITE_OK);
    const int64_t t1 = at(1);
    std::ostringstream sql;
    sql << kLegacyFaultsDdl;
    for (const auto & row : rows) {
      sql << "INSERT INTO faults VALUES ('" << row.code << "', " << row.severity << ", 'legacy', " << t1 << ", " << t1
          << ", 2, '" << row.status << "', '[\"" << kOwner << "\"]', -1, " << t1 << ", 0, " << row.confirmed_at_ns
          << ");";
    }
    EXPECT_EQ(sqlite3_exec(raw, sql.str().c_str(), nullptr, nullptr, nullptr), SQLITE_OK) << sqlite3_errmsg(raw);
    sqlite3_close(raw);
    record_owner_ = ros2_medkit_fault_manager::kSharedOwner;  // a migrated row is the shared record of its code
    reopen();
  }

  void open_legacy_database(const std::string & code, int severity, const std::string & status,
                            int64_t confirmed_at_ns) {
    open_legacy_database({LegacyRow{code, severity, status, confirmed_at_ns}});
  }

  void failed(const std::string & code, uint8_t severity, int64_t second, const DebounceConfig & config) {
    storage_->report_fault_event(FaultId{code, record_owner_}, fd::ReportKind::kFailed, severity, "report", kOwner,
                                 at(second), config);
  }

  void passed(const std::string & code, int64_t second, const DebounceConfig & config) {
    storage_->report_fault_event(FaultId{code, record_owner_}, fd::ReportKind::kPassed, 0, "", kOwner, at(second),
                                 config);
  }

  bool clear(const std::string & code) {
    return storage_->clear_fault(FaultId{code, record_owner_});
  }

  fd::FaultRecord fault(const std::string & code) {
    auto f = storage_->get_fault(FaultId{code, record_owner_});
    EXPECT_TRUE(f.has_value()) << code;
    return f.value_or(fd::FaultRecord{});
  }

  int64_t confirmed_at(const std::string & code) const {
    sqlite3 * raw = nullptr;
    EXPECT_EQ(sqlite3_open(db_path().c_str(), &raw), SQLITE_OK);
    sqlite3_stmt * stmt = nullptr;
    EXPECT_EQ(sqlite3_prepare_v2(raw, "SELECT confirmed_at_ns FROM faults WHERE fault_code = ? AND owner = ?", -1,
                                 &stmt, nullptr),
              SQLITE_OK);
    sqlite3_bind_text(stmt, 1, code.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(stmt, 2, record_owner_.c_str(), -1, SQLITE_TRANSIENT);
    int64_t value = -1;
    if (sqlite3_step(stmt) == SQLITE_ROW) {
      value = sqlite3_column_int64(stmt, 0);
    }
    sqlite3_finalize(stmt);
    sqlite3_close(raw);
    return value;
  }

  std::filesystem::path dir_;
  std::unique_ptr<SqliteFaultStorage> storage_;
  /// Owner of the records the helpers address: the test's reporter, or the shared owner after a legacy open.
  std::string record_owner_{kOwner};
};

TEST_F(SqliteSeverityTest, CriticalClearRestartWarnConfirms) {
  const auto config = confirm_at(-3);
  failed("CRIT_RESTART", Fault::SEVERITY_CRITICAL, 0, config);
  ASSERT_TRUE(clear("CRIT_RESTART"));

  reopen();
  failed("CRIT_RESTART", Fault::SEVERITY_WARN, 1, config);

  const auto f = fault("CRIT_RESTART");
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(f.occurrence_count, 2u);
  EXPECT_EQ(confirmed_at("CRIT_RESTART"), at(1));
}

TEST_F(SqliteSeverityTest, ReactivationHasNoConfirmationTimeUntilItConfirms) {
  const auto config = confirm_at(-3);
  failed("CONF_AT", Fault::SEVERITY_ERROR, 0, config);
  failed("CONF_AT", Fault::SEVERITY_ERROR, 1, config);
  failed("CONF_AT", Fault::SEVERITY_ERROR, 2, config);
  ASSERT_EQ(fault("CONF_AT").status, Fault::STATUS_CONFIRMED);
  ASSERT_EQ(confirmed_at("CONF_AT"), at(2));
  ASSERT_TRUE(clear("CONF_AT"));
  EXPECT_EQ(confirmed_at("CONF_AT"), at(2)) << "a CLEARED record keeps the time it confirmed";

  failed("CONF_AT", Fault::SEVERITY_ERROR, 3, config);
  ASSERT_EQ(fault("CONF_AT").status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(confirmed_at("CONF_AT"), 0) << "this occurrence has not confirmed";

  failed("CONF_AT", Fault::SEVERITY_ERROR, 4, config);
  EXPECT_EQ(confirmed_at("CONF_AT"), 0);

  failed("CONF_AT", Fault::SEVERITY_ERROR, 5, config);
  ASSERT_EQ(fault("CONF_AT").status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(confirmed_at("CONF_AT"), at(5));
}

TEST_F(SqliteSeverityTest, ReactivationConfirmationTimeSurvivesRestarts) {
  const auto config = confirm_at(-2);
  failed("CONF_AT_RESTART", Fault::SEVERITY_ERROR, 0, config);
  failed("CONF_AT_RESTART", Fault::SEVERITY_ERROR, 1, config);
  ASSERT_EQ(confirmed_at("CONF_AT_RESTART"), at(1));
  ASSERT_TRUE(clear("CONF_AT_RESTART"));

  reopen();
  failed("CONF_AT_RESTART", Fault::SEVERITY_ERROR, 2, config);
  ASSERT_EQ(fault("CONF_AT_RESTART").status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(confirmed_at("CONF_AT_RESTART"), 0);

  reopen();
  EXPECT_EQ(confirmed_at("CONF_AT_RESTART"), 0);
  failed("CONF_AT_RESTART", Fault::SEVERITY_ERROR, 3, config);
  ASSERT_EQ(fault("CONF_AT_RESTART").status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(confirmed_at("CONF_AT_RESTART"), at(3));
}

TEST_F(SqliteSeverityTest, ReactivationThatConfirmsAtOnceRecordsItsOwnTime) {
  const auto config = confirm_at(-1);
  failed("CONF_AT_NOW", Fault::SEVERITY_WARN, 0, config);
  ASSERT_EQ(confirmed_at("CONF_AT_NOW"), at(0));
  ASSERT_TRUE(clear("CONF_AT_NOW"));

  failed("CONF_AT_NOW", Fault::SEVERITY_WARN, 1, config);
  EXPECT_EQ(confirmed_at("CONF_AT_NOW"), at(1));

  // A CRITICAL record confirms a debounced reactivation at once, and records that instant.
  const auto debounced = confirm_at(-3);
  failed("CONF_AT_CRIT", Fault::SEVERITY_CRITICAL, 2, debounced);
  ASSERT_TRUE(clear("CONF_AT_CRIT"));
  failed("CONF_AT_CRIT", Fault::SEVERITY_INFO, 3, debounced);
  EXPECT_EQ(confirmed_at("CONF_AT_CRIT"), at(3));
}

TEST_F(SqliteSeverityTest, MigratedClearedRecordHasNoConfirmationTimeUntilItConfirms) {
  const auto config = confirm_at(-3);
  open_legacy_database("MIGRATED", Fault::SEVERITY_ERROR, Fault::STATUS_CLEARED, at(0));
  ASSERT_EQ(fault("MIGRATED").status, Fault::STATUS_CLEARED);
  EXPECT_EQ(confirmed_at("MIGRATED"), at(0)) << "the migration keeps the cleared occurrence's time";

  failed("MIGRATED", Fault::SEVERITY_ERROR, 2, config);
  ASSERT_EQ(fault("MIGRATED").status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(fault("MIGRATED").occurrence_count, 3u);
  EXPECT_EQ(confirmed_at("MIGRATED"), 0) << "this occurrence has not confirmed";

  reopen();
  EXPECT_EQ(confirmed_at("MIGRATED"), 0);
  failed("MIGRATED", Fault::SEVERITY_ERROR, 3, config);
  failed("MIGRATED", Fault::SEVERITY_ERROR, 4, config);
  ASSERT_EQ(fault("MIGRATED").status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(confirmed_at("MIGRATED"), at(4));
}

TEST_F(SqliteSeverityTest, MigratedClearedCriticalRecordConfirmsOnAWarn) {
  const auto config = confirm_at(-3);
  open_legacy_database("MIGRATED_CRIT", Fault::SEVERITY_CRITICAL, Fault::STATUS_CLEARED, at(0));

  failed("MIGRATED_CRIT", Fault::SEVERITY_WARN, 2, config);
  const auto f = fault("MIGRATED_CRIT");
  EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(confirmed_at("MIGRATED_CRIT"), at(2));
}

TEST_F(SqliteSeverityTest, PendingCriticalRowFromAnEarlierReleaseStaysUntilAReport) {
  // The row an earlier release left after CRITICAL, clear, WARN at threshold -3: CRITICAL in
  // PREFAILED, occurrence 2, still holding the first occurrence's confirmation time.
  open_legacy_database("LEGACY", Fault::SEVERITY_CRITICAL, Fault::STATUS_PREFAILED, at(0));
  auto f = fault("LEGACY");
  EXPECT_EQ(f.status, Fault::STATUS_PREFAILED) << "the migration does not rewrite the status";
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
  EXPECT_EQ(confirmed_at("LEGACY"), 0) << "the migration drops the earlier occurrence's time";

  reopen();
  f = fault("LEGACY");
  EXPECT_EQ(f.status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(f.occurrence_count, 2u);
  EXPECT_EQ(confirmed_at("LEGACY"), 0);
}

TEST_F(SqliteSeverityTest, MigrationKeepsAConfirmationTimeOnlyOnRowsThatConfirmed) {
  // Under the latch a row that confirmed stays CONFIRMED or HEALED until a clear, so a pending row
  // with a time carries it from the occurrence before a reactivation.
  const int64_t earlier = at(0);
  open_legacy_database({
      {"PENDING_FAILED", Fault::SEVERITY_ERROR, Fault::STATUS_PREFAILED, earlier},
      {"PENDING_PASSED", Fault::SEVERITY_WARN, Fault::STATUS_PREPASSED, earlier},
      {"PENDING_NEVER", Fault::SEVERITY_ERROR, Fault::STATUS_PREFAILED, 0},
      {"CONFIRMED", Fault::SEVERITY_ERROR, Fault::STATUS_CONFIRMED, earlier},
      {"HEALED", Fault::SEVERITY_ERROR, Fault::STATUS_HEALED, earlier},
      {"CLEARED", Fault::SEVERITY_ERROR, Fault::STATUS_CLEARED, earlier},
  });

  const std::vector<std::pair<std::string, int64_t>> expected{
      {"PENDING_FAILED", 0},  {"PENDING_PASSED", 0}, {"PENDING_NEVER", 0},
      {"CONFIRMED", earlier}, {"HEALED", earlier},   {"CLEARED", earlier},
  };
  for (int open = 0; open < 2; ++open) {
    for (const auto & [code, confirmed] : expected) {
      SCOPED_TRACE(code + " open " + std::to_string(open));
      EXPECT_EQ(confirmed_at(code), confirmed);
    }
    reopen();
  }
  EXPECT_EQ(fault("PENDING_FAILED").status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(fault("PENDING_PASSED").status, Fault::STATUS_PREPASSED);
  EXPECT_EQ(fault("CONFIRMED").status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault("HEALED").status, Fault::STATUS_HEALED);
  EXPECT_EQ(fault("CLEARED").status, Fault::STATUS_CLEARED);
}

TEST_F(SqliteSeverityTest, SweepRecordsTheConfirmationTimeOfAReactivatedOccurrence) {
  auto config = confirm_at(-3);
  config.auto_confirm_after_sec = 5.0;
  storage_->set_debounce_config(config);

  for (const int64_t second : {0, 1, 2}) {
    failed("SWEPT", Fault::SEVERITY_ERROR, second, config);
  }
  ASSERT_EQ(fault("SWEPT").status, Fault::STATUS_CONFIRMED);
  ASSERT_EQ(confirmed_at("SWEPT"), at(2));
  ASSERT_TRUE(clear("SWEPT"));
  failed("SWEPT", Fault::SEVERITY_ERROR, 3, config);
  ASSERT_EQ(fault("SWEPT").status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(confirmed_at("SWEPT"), 0) << "this occurrence has not confirmed";

  reopen();
  storage_->set_debounce_config(config);
  EXPECT_EQ(confirmed_at("SWEPT"), 0);
  EXPECT_TRUE(storage_->check_time_based_confirmation(at(5)).empty()) << "before the window ends";
  ASSERT_EQ(storage_->check_time_based_confirmation(at(10)).size(), 1u);
  EXPECT_EQ(fault("SWEPT").status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(confirmed_at("SWEPT"), at(10)) << "the sweep confirmed this occurrence";
}

TEST_F(SqliteSeverityTest, TimeBasedConfirmationKeepsTheConfirmationTimeOfTheReport) {
  auto config = confirm_at(-3);
  config.auto_confirm_after_sec = 5.0;
  storage_->set_debounce_config(config);

  failed("TIMED", Fault::SEVERITY_CRITICAL, 0, config);
  ASSERT_TRUE(clear("TIMED"));
  failed("TIMED", Fault::SEVERITY_WARN, 1, config);
  ASSERT_EQ(fault("TIMED").status, Fault::STATUS_CONFIRMED);
  ASSERT_EQ(confirmed_at("TIMED"), at(1)) << "confirmed by the report";

  EXPECT_TRUE(storage_->check_time_based_confirmation(at(10)).empty());
  EXPECT_EQ(confirmed_at("TIMED"), at(1)) << "the sweep does not rewrite it";

  reopen();
  storage_->set_debounce_config(config);
  EXPECT_TRUE(storage_->check_time_based_confirmation(at(20)).empty());
  EXPECT_EQ(confirmed_at("TIMED"), at(1));
}

TEST_F(SqliteSeverityTest, PendingCriticalRowConfirmsOnTheNextFailedOfAnySeverity) {
  const auto config = confirm_at(-3);
  for (const uint8_t severity :
       {Fault::SEVERITY_INFO, Fault::SEVERITY_WARN, Fault::SEVERITY_ERROR, Fault::SEVERITY_CRITICAL}) {
    const std::string code = "LEGACY_FAILED_" + std::to_string(severity);
    SCOPED_TRACE(code);
    open_legacy_database(code, Fault::SEVERITY_CRITICAL, Fault::STATUS_PREFAILED, at(0));

    failed(code, severity, 2, config);
    const auto f = fault(code);
    EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED);
    EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
    EXPECT_EQ(f.occurrence_count, 2u) << "the row is not CLEARED, so this is the same occurrence";
    EXPECT_EQ(confirmed_at(code), at(2));
  }
}

TEST_F(SqliteSeverityTest, PendingCriticalRowCanMoveToPrepassedOnPassed) {
  const auto config = confirm_at(-3);
  open_legacy_database("LEGACY_PASSED", Fault::SEVERITY_CRITICAL, Fault::STATUS_PREFAILED, at(0));

  passed("LEGACY_PASSED", 2, config);
  EXPECT_EQ(fault("LEGACY_PASSED").status, Fault::STATUS_PREFAILED) << "counter 0 keeps the status";
  passed("LEGACY_PASSED", 3, config);
  auto f = fault("LEGACY_PASSED");
  EXPECT_EQ(f.status, Fault::STATUS_PREPASSED) << "a PASSED report does not apply the CRITICAL bypass";
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);

  failed("LEGACY_PASSED", Fault::SEVERITY_WARN, 4, config);
  f = fault("LEGACY_PASSED");
  EXPECT_EQ(f.status, Fault::STATUS_CONFIRMED) << "the next FAILED confirms it";
  EXPECT_EQ(f.severity, Fault::SEVERITY_CRITICAL);
}

// ---------------------------------------------------------------------------
// Exhaustive sweep: every event sequence of a given length over a small alphabet, applied to both
// backends and checked after every step, for both records of one code.
// ---------------------------------------------------------------------------

enum class Kind { kFailed, kPassed, kClear };

struct Event {
  Kind kind;
  uint8_t severity;
  const char * owner;
};

constexpr std::array<const char *, 2> kOwners = {"/a", "/b"};

constexpr std::array<Event, 8> kAlphabet = {{
    {Kind::kFailed, Fault::SEVERITY_INFO, "/a"},
    {Kind::kFailed, Fault::SEVERITY_WARN, "/a"},
    {Kind::kFailed, Fault::SEVERITY_CRITICAL, "/a"},
    {Kind::kFailed, Fault::SEVERITY_WARN, "/b"},
    {Kind::kFailed, Fault::SEVERITY_CRITICAL, "/b"},
    {Kind::kPassed, 0, "/a"},
    {Kind::kClear, 0, "/a"},
    {Kind::kClear, 0, "/b"},
}};

/// The fields this contract fixes for one record, independent of the debounce state machine.
struct Model {
  bool exists{false};
  bool cleared{false};
  uint8_t severity{0};
  uint32_t occurrence_count{0};

  void apply(const Event & e) {
    switch (e.kind) {
      case Kind::kFailed:
        if (!exists || cleared) {
          ++occurrence_count;
        }
        severity = exists ? std::max(severity, e.severity) : e.severity;
        exists = true;
        cleared = false;
        break;
      case Kind::kPassed:
        break;
      case Kind::kClear:
        cleared = exists;
        break;
    }
  }
};

std::string describe(const std::vector<size_t> & seq, int upto) {
  std::ostringstream out;
  for (int i = 0; i <= upto; ++i) {
    const auto & e = kAlphabet[seq[static_cast<size_t>(i)]];
    if (i > 0) {
      out << ", ";
    }
    switch (e.kind) {
      case Kind::kFailed:
        out << "FAILED(" << static_cast<int>(e.severity) << "," << e.owner << ")";
        break;
      case Kind::kPassed:
        out << "PASSED(" << e.owner << ")";
        break;
      case Kind::kClear:
        out << "CLEAR(" << e.owner << ")";
        break;
    }
  }
  return out.str();
}

/// Empty when the record holds the contract after @p e; otherwise what broke.
/// @param addressed Whether @p e was addressed to this record
std::string check(const fd::FaultRecord & f, const Model & m, const Event & e, bool addressed,
                  const DebounceConfig & config, const std::string & where) {
  std::ostringstream out;
  if (f.severity != m.severity) {
    out << where << " severity " << static_cast<int>(f.severity) << " != " << static_cast<int>(m.severity) << "; ";
  }
  if (f.occurrence_count != m.occurrence_count) {
    out << where << " occurrence_count " << f.occurrence_count << " != " << m.occurrence_count << "; ";
  }
  if (m.cleared != (f.status == Fault::STATUS_CLEARED)) {
    out << where << " status " << f.status << "; ";
  }
  if (config.critical_immediate_confirm && f.severity == Fault::SEVERITY_CRITICAL) {
    if (pending(f.status)) {
      out << where << " CRITICAL in " << f.status << "; ";
    }
    if (addressed && e.kind == Kind::kFailed && f.status != Fault::STATUS_CONFIRMED) {
      out << where << " FAILED on a CRITICAL record left " << f.status << "; ";
    }
  }
  return out.str();
}

std::string check_parity(const fd::FaultRecord & mem, const fd::FaultRecord & sql, const std::string & owner) {
  std::ostringstream out;
  if (mem.severity != sql.severity || mem.status != sql.status || mem.description != sql.description ||
      mem.occurrence_count != sql.occurrence_count || mem.reporting_sources != sql.reporting_sources ||
      mem.first_occurred_ns != sql.first_occurred_ns || mem.last_occurred_ns != sql.last_occurred_ns ||
      mem.last_passed_ns != sql.last_passed_ns) {
    out << owner << " backends disagree (memory: sev=" << static_cast<int>(mem.severity) << " status=" << mem.status
        << " occ=" << mem.occurrence_count << "; sqlite: sev=" << static_cast<int>(sql.severity)
        << " status=" << sql.status << " occ=" << sql.occurrence_count << "); ";
  }
  return out.str();
}

/// @param length Events per sequence; long enough to reach HEALED and report FAILED on it.
void run_sweep(const DebounceConfig & config, const std::string & tag, int length) {
  InMemoryFaultStorage memory;
  SqliteFaultStorage sqlite(":memory:", test_log_sink());

  size_t total = 1;
  for (int i = 0; i < length; ++i) {
    total *= kAlphabet.size();
  }

  size_t failures = 0;
  std::string first_failure;
  std::vector<size_t> seq(static_cast<size_t>(length), 0);
  for (size_t index = 0; index < total; ++index) {
    size_t rest = index;
    for (int i = 0; i < length; ++i) {
      seq[static_cast<size_t>(i)] = rest % kAlphabet.size();
      rest /= kAlphabet.size();
    }
    const std::string code = tag + "_" + std::to_string(index);
    std::array<Model, kOwners.size()> models{};
    for (int step = 0; step < length; ++step) {
      const auto & e = kAlphabet[seq[static_cast<size_t>(step)]];
      const auto t = at(step);
      if (e.kind == Kind::kClear) {
        memory.clear_fault(FaultId{code, e.owner});
        sqlite.clear_fault(FaultId{code, e.owner});
      } else {
        const fd::ReportKind type = e.kind == Kind::kFailed ? fd::ReportKind::kFailed : fd::ReportKind::kPassed;
        memory.report_fault_event(FaultId{code, e.owner}, type, e.severity, "step " + std::to_string(step), e.owner, t,
                                  config);
        sqlite.report_fault_event(FaultId{code, e.owner}, type, e.severity, "step " + std::to_string(step), e.owner, t,
                                  config);
      }

      std::string diff;
      for (size_t o = 0; o < kOwners.size(); ++o) {
        const std::string owner = kOwners[o];
        const bool addressed = owner == e.owner;
        if (addressed) {
          models[o].apply(e);
        }
        const auto mem = memory.get_fault(FaultId{code, owner});
        const auto sql = sqlite.get_fault(FaultId{code, owner});
        if (mem.has_value() != models[o].exists || sql.has_value() != models[o].exists) {
          diff += owner + " existence differs; ";
        } else if (models[o].exists) {
          diff += check(*mem, models[o], e, addressed, config, "memory " + owner) +
                  check(*sql, models[o], e, addressed, config, "sqlite " + owner) + check_parity(*mem, *sql, owner);
        }
      }
      if (!diff.empty()) {
        if (failures == 0) {
          first_failure = "[" + describe(seq, step) + "] " + diff;
        }
        ++failures;
        break;
      }
    }
  }

  EXPECT_EQ(failures, 0u) << failures << " of " << total << " sequences diverged; first: " << first_failure;
}

TEST(FaultSeveritySweep, ImmediateConfirmation) {
  run_sweep(DebounceConfig{}, "IC", 4);
}

TEST(FaultSeveritySweep, ImmediateConfirmationHealOnSinglePassed) {
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 0;
  run_sweep(config, "IH", 4);
}

TEST(FaultSeveritySweep, DebouncedWithShortHealing) {
  // HEALED within three events, so a FAILED on a HEALED record fits in four.
  DebounceConfig config;
  config.confirmation_threshold = -2;
  config.healing_enabled = true;
  config.healing_threshold = 1;
  run_sweep(config, "DS", 4);
}

TEST(FaultSeveritySweep, DeepDebounceWithHealing) {
  // CRITICAL confirms at -1 and heals after three PASSED, so a FAILED on it needs five events.
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_enabled = true;
  config.healing_threshold = 2;
  run_sweep(config, "DH", 5);
}

TEST(FaultSeveritySweep, CriticalBypassOff) {
  // Without the bypass nothing forces a CRITICAL record to confirm; the other checks still hold.
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.critical_immediate_confirm = false;
  run_sweep(config, "NB", 4);
}

}  // namespace
