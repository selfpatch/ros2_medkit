// Copyright 2026 gstavrinos
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
#include <libpq-fe.h>

#include <pqxx/pqxx>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging.h"
#include "ros2_medkit_fault_manager/postgres_fault_storage.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"
#include "ros2_medkit_msgs/srv/report_fault.hpp"

using ros2_medkit_fault_manager::DebounceConfig;
using ros2_medkit_fault_manager::PgFaultStorage;
using ros2_medkit_fault_manager::RosbagFileInfo;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::srv::ReportFault;

/// Default debounce config for tests (matches DebounceConfig defaults: threshold=-1, no healing)
static DebounceConfig default_config() {
  return DebounceConfig{};
}

/// Connection string of the server the tests run against. Unlike SQLite, PostgreSQL
/// needs a live server. Set the ROS2_MEDKIT_TEST_PG_CONN to a testing database url.
/// The included docker container uses the default url, so using the environment
/// variable is not necessary.
std::string base_conn_info() {
  if (const char * env = std::getenv("ROS2_MEDKIT_TEST_PG_CONN")) {
    return env;
  }
  return "postgresql://user:password@localhost:5432/ros2_medkit_faults_database";
}

/// SQLite isolates tests with a unique temp file; the PostgreSQL equivalent is to isolate them with a unique schema and
/// a search_path attached to it. Thus, reopening "the same database" (same .db file in SQLite) needs the construction
/// of a second PgFaultStorage on the same conninfo.
class PgFaultStorageTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a unique schema for each test using random_device for better entropy
    std::random_device rd;
    std::mt19937_64 gen(rd());
    std::uniform_int_distribution<uint64_t> dist;
    schema_ = "test_faults_" + std::to_string(dist(gen));

    try {
      db_conn_ = std::make_unique<pqxx::connection>(base_conn_info());
    } catch (const std::exception & e) {
      GTEST_FAIL() << "No PostgreSQL server reachable (set ROS2_MEDKIT_TEST_PG_CONN): " << e.what();
    }

    {
      pqxx::work tx(*db_conn_);
      tx.exec("CREATE SCHEMA " + tx.quote_name(schema_));
      tx.commit();
    }

    conn_info_ = base_conn_info() + "?options=-csearch_path%3D" + schema_;
    storage_ = std::make_unique<PgFaultStorage>(conn_info_);

    temp_root_ = std::filesystem::temp_directory_path() / schema_;
    std::filesystem::create_directories(temp_root_);
  }

  void TearDown() override {
    storage_.reset();
    if (db_conn_) {
      pqxx::work tx(*db_conn_);
      try {
        tx.exec("DROP SCHEMA IF EXISTS " + tx.quote_name(schema_) + " CASCADE");
        tx.commit();
      } catch (...) {
        tx.abort();
      }
      db_conn_.reset();
    }
    std::error_code ec;
    std::filesystem::remove_all(temp_root_, ec);
  }

  // Fresh connection pinned to this test's schema, for pokes behind the storage's back.
  std::unique_ptr<pqxx::connection> raw_connection() const {
    return std::make_unique<pqxx::connection>(conn_info_);
  }

  // Helper function to quickly execute sql commands
  void exec_raw(const std::string & sql) const {
    auto conn = raw_connection();
    pqxx::work tx(*conn);
    tx.exec(sql);
    tx.commit();
  }

  // confirmed_at_ns column (consumed by the compliance timeline exporter).
  int64_t read_confirmed_at(const std::string & fault_code) {
    auto conn = raw_connection();
    pqxx::work tx(*conn);
    auto res = tx.exec("SELECT confirmed_at_ns FROM faults WHERE fault_code = $1", pqxx::params{fault_code});
    tx.commit();
    return res.empty() ? -1 : res[0][0].as<int64_t>();
  }

  std::string schema_;
  std::string conn_info_;
  std::filesystem::path temp_root_;
  std::unique_ptr<pqxx::connection> db_conn_;
  std::unique_ptr<PgFaultStorage> storage_;
};

TEST_F(PgFaultStorageTest, ReportNewFaultEvent) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED,
                                             Fault::SEVERITY_ERROR, "Motor temperature exceeded threshold",
                                             "/powertrain/motor", timestamp, default_config());

  EXPECT_TRUE(is_new);
  EXPECT_EQ(storage_->size(), 1u);
  EXPECT_TRUE(storage_->contains("MOTOR_OVERHEAT"));
}

TEST_F(PgFaultStorageTest, PassedEventForNonExistentFaultIgnored) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  bool is_new = storage_->report_fault_event("NON_EXISTENT", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR,
                                             "Test", "/node1", timestamp, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_->size(), 0u);
}

TEST_F(PgFaultStorageTest, ReportExistingFaultEventUpdates) {
  rclcpp::Clock clock;
  auto timestamp1 = clock.now();
  auto timestamp2 = clock.now();

  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Initial report", "/powertrain/motor1", timestamp1, default_config());

  bool is_new =
      storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                   "Second report", "/powertrain/motor2", timestamp2, default_config());

  EXPECT_FALSE(is_new);
  EXPECT_EQ(storage_->size(), 1u);

  auto fault = storage_->get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  // Still the same continuous occurrence (not CLEARED in between): occurrence_count
  // does not bump on every report, only severity/sources/description update.
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->severity, Fault::SEVERITY_ERROR);  // Updated to higher severity
  EXPECT_EQ(fault->reporting_sources.size(), 2u);
}

TEST_F(PgFaultStorageTest, ContinuouslyActiveFaultDoesNotInflateOccurrenceCount) {
  rclcpp::Clock clock;

  // Simulate a level-triggered poller re-reporting the same still-true condition
  // every cycle (issue #11): occurrence_count must stay at 1, not grow per poll.
  for (int i = 0; i < 25; ++i) {
    storage_->report_fault_event("TANK_OVERFILL", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                 "level = 95 > 80", "/tank", clock.now(), default_config());
  }

  auto fault = storage_->get_fault("TANK_OVERFILL");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(PgFaultStorageTest, ListFaultsDefaultReturnsConfirmedOnly) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, single report confirms immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, default_config());

  // Default query should return the CONFIRMED fault
  auto faults = storage_->list_faults(false, 0, {});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

TEST_F(PgFaultStorageTest, ListFaultsWithPrefailedStatus) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Set threshold to -3 to test PREFAILED status
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, config);

  // Query with PREFAILED status
  auto faults = storage_->list_faults(false, 0, {Fault::STATUS_PREFAILED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_1");
  EXPECT_EQ(faults[0].status, Fault::STATUS_PREFAILED);
}

TEST_F(PgFaultStorageTest, ListFaultsFilterBySeverity) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  storage_->report_fault_event("FAULT_INFO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Info", "/node1",
                               timestamp, default_config());
  storage_->report_fault_event("FAULT_ERROR", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Error",
                               "/node1", timestamp, default_config());

  // Filter by ERROR severity (query CONFIRMED since that's the default status now)
  auto faults = storage_->list_faults(true, Fault::SEVERITY_ERROR, {Fault::STATUS_CONFIRMED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].fault_code, "FAULT_ERROR");
}

TEST_F(PgFaultStorageTest, ClearFault) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                               "/node1", timestamp, default_config());

  bool cleared = storage_->clear_fault("MOTOR_OVERHEAT");
  EXPECT_TRUE(cleared);

  auto fault = storage_->get_fault("MOTOR_OVERHEAT");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);
}

TEST_F(PgFaultStorageTest, ClearNonExistentFault) {
  bool cleared = storage_->clear_fault("NON_EXISTENT");
  EXPECT_FALSE(cleared);
}

TEST_F(PgFaultStorageTest, PassedEventDoesNotAdvanceLastOccurred) {
  // Same contract as the in-memory and SQLite backends: a PASSED event ends a
  // fault, it does not re-date it. Guards against a stale CONFIRMED fault reading
  // as freshly active in /faults and in the SSE payload.
  const rclcpp::Time failed_at(1000, 0, RCL_SYSTEM_TIME);
  const rclcpp::Time passed_at(9000, 0, RCL_SYSTEM_TIME);

  storage_->report_fault_event("FAULT_LO", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               failed_at, default_config());
  storage_->report_fault_event("FAULT_LO", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", passed_at,
                               default_config());

  auto fault = storage_->get_fault("FAULT_LO");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // healing disabled: latched, by design
  EXPECT_EQ(rclcpp::Time(fault->last_occurred).nanoseconds(), failed_at.nanoseconds());
  // The PASSED instant is not lost: it rides on the wire as last_passed.
  EXPECT_EQ(rclcpp::Time(fault->last_passed).nanoseconds(), passed_at.nanoseconds());
}

TEST_F(PgFaultStorageTest, GetClearedFaults) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, default_config());
  storage_->clear_fault("FAULT_1");

  // Query cleared faults
  auto faults = storage_->list_faults(false, 0, {Fault::STATUS_CLEARED});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CLEARED);
}

TEST_F(PgFaultStorageTest, InvalidStatusDefaultsToConfirmed) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, fault is immediately CONFIRMED
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               timestamp, default_config());

  // Query with invalid status - defaults to CONFIRMED, which now matches our fault
  auto faults = storage_->list_faults(false, 0, {"INVALID_STATUS"});
  EXPECT_EQ(faults.size(), 1u);
  EXPECT_EQ(faults[0].status, Fault::STATUS_CONFIRMED);
}

// PostgreSQL-specific persistence test
TEST_F(PgFaultStorageTest, PersistenceAcrossRestarts) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // With default threshold=-1, faults are immediately CONFIRMED
  // Report some faults
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Persistent fault 1", "/node1", timestamp, default_config());
  storage_->report_fault_event("FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Persistent fault 2", "/node2", timestamp, default_config());
  storage_->clear_fault("FAULT_2");

  // Verify initial state
  EXPECT_EQ(storage_->size(), 2u);

  // Drop the connection
  storage_.reset();

  // Reconnect to the same schema
  storage_ = std::make_unique<PgFaultStorage>(conn_info_);

  // Verify faults persisted
  EXPECT_EQ(storage_->size(), 2u);
  EXPECT_TRUE(storage_->contains("FAULT_1"));
  EXPECT_TRUE(storage_->contains("FAULT_2"));

  auto fault1 = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault1.has_value());
  EXPECT_EQ(fault1->severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(fault1->status, Fault::STATUS_CONFIRMED);  // Immediately confirmed with threshold=-1
  EXPECT_EQ(fault1->description, "Persistent fault 1");

  auto fault2 = storage_->get_fault("FAULT_2");
  ASSERT_TRUE(fault2.has_value());
  EXPECT_EQ(fault2->status, Fault::STATUS_CLEARED);
}

// Test timestamp precision
TEST_F(PgFaultStorageTest, TimestampPrecision) {
  // Create a timestamp with nanosecond precision
  int64_t test_ns = 1735312456123456789LL;  // Specific nanosecond timestamp
  rclcpp::Time timestamp(test_ns, RCL_SYSTEM_TIME);

  storage_->report_fault_event("FAULT_TS", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_INFO, "Timestamp test",
                               "/node1", timestamp, default_config());

  auto fault = storage_->get_fault("FAULT_TS");
  ASSERT_TRUE(fault.has_value());

  // Convert builtin_interfaces::msg::Time back to rclcpp::Time for comparison
  rclcpp::Time first_ts(fault->first_occurred);
  rclcpp::Time last_ts(fault->last_occurred);

  // Verify nanosecond precision is preserved
  EXPECT_EQ(first_ts.nanoseconds(), test_ns);
  EXPECT_EQ(last_ts.nanoseconds(), test_ns);
}

namespace {

/// One keyword of base_conn_info() as libpq reads it; empty when absent.
std::string base_conn_option(const std::string & keyword) {
  char * err = nullptr;
  PQconninfoOption * options = PQconninfoParse(base_conn_info().c_str(), &err);
  if (err != nullptr) {
    PQfreemem(err);
  }
  std::string value;
  for (const PQconninfoOption * o = options; o != nullptr && o->keyword != nullptr; ++o) {
    if (keyword == o->keyword && o->val != nullptr) {
      value = o->val;
    }
  }
  PQconninfoFree(options);
  return value;
}

/// key=value connection string to the test server with the given user, password and extra options.
std::string conn_as(const std::string & user, const std::string & password, const std::string & extra = "") {
  return "host=" + base_conn_option("host") + " port=" + base_conn_option("port") +
         " dbname=" + base_conn_option("dbname") + " user=" + user + " password=" + password + extra;
}

std::string unique_suffix() {
  std::random_device rd;
  std::mt19937_64 gen(rd());
  return std::to_string(std::uniform_int_distribution<uint64_t>()(gen));
}

}  // namespace

// A database that does not exist yet is not a configuration error: the storage starts without it.
TEST(PgFaultStorageConnectionTest, StartsWithoutAnUnreachableDatabase) {
  PgFaultStorage storage(base_conn_info() + "_g4rb4g3");
  EXPECT_FALSE(storage.connected());
  EXPECT_THROW(storage.size(), ros2_medkit_fault_manager::FaultStorage::IgnorableConnectionException);
}

// The schema is created on the first connection that succeeds, after the constructor.
TEST(PgFaultStorageConnectionTest, CreatesTheSchemaWhenTheDatabaseAppears) {
  std::unique_ptr<pqxx::connection> admin;
  try {
    admin = std::make_unique<pqxx::connection>(base_conn_info());
  } catch (const std::exception & e) {
    GTEST_FAIL() << "No PostgreSQL server reachable (set ROS2_MEDKIT_TEST_PG_CONN): " << e.what();
  }
  const std::string db = "medkit_late_" + unique_suffix();
  const std::string base = base_conn_info();
  const std::string late_url = base.substr(0, base.rfind('/') + 1) + db;

  {
    PgFaultStorage storage(late_url);
    ASSERT_FALSE(storage.connected());
    EXPECT_THROW(storage.size(), ros2_medkit_fault_manager::FaultStorage::IgnorableConnectionException);

    {
      pqxx::nontransaction ntx(*admin);
      ntx.exec("CREATE DATABASE " + ntx.quote_name(db));
    }

    // The next connection round starts once the backoff after the failed one has passed.
    rclcpp::Clock clock;
    bool stored = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!stored && std::chrono::steady_clock::now() < deadline) {
      try {
        storage.report_fault_event("LATE", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "late", "/node",
                                   clock.now(), default_config());
        stored = true;
      } catch (const ros2_medkit_fault_manager::FaultStorage::IgnorableConnectionException &) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
      }
    }
    ASSERT_TRUE(stored) << "the storage never connected after the database was created";
    EXPECT_TRUE(storage.connected());
    EXPECT_EQ(storage.size(), 1u);
  }

  pqxx::nontransaction ntx(*admin);
  ntx.exec("DROP DATABASE IF EXISTS " + ntx.quote_name(db) + " WITH (FORCE)");
}

/// Collects every rcutils log message while alive.
class LogCapture {
 public:
  LogCapture() : previous_(rcutils_logging_get_output_handler()) {
    std::lock_guard<std::mutex> lock(mutex());
    captured().clear();
    rcutils_logging_set_output_handler(&LogCapture::handler);
  }
  ~LogCapture() {
    rcutils_logging_set_output_handler(previous_);
  }
  LogCapture(const LogCapture &) = delete;
  LogCapture & operator=(const LogCapture &) = delete;
  LogCapture(LogCapture &&) = delete;
  LogCapture & operator=(LogCapture &&) = delete;

  std::string text() const {
    std::lock_guard<std::mutex> lock(mutex());
    return captured();
  }

 private:
  static void handler(const rcutils_log_location_t * /*location*/, int /*severity*/, const char * /*name*/,
                      rcutils_time_point_value_t /*timestamp*/, const char * format, va_list * args) {
    va_list copy;
    va_copy(copy, *args);
    std::array<char, 4096> buffer{};
    // The format comes from the logging call site. clang reports -Wformat-nonliteral here, GCC does not.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wformat-nonliteral"
    std::vsnprintf(buffer.data(), buffer.size(), format, copy);
#pragma GCC diagnostic pop
    va_end(copy);
    std::lock_guard<std::mutex> lock(mutex());
    captured() += buffer.data();
    captured() += '\n';
  }
  static std::mutex & mutex() {
    static std::mutex m;
    return m;
  }
  static std::string & captured() {
    static std::string text;
    return text;
  }

  rcutils_logging_output_handler_t previous_;
};

/// TCP server on 127.0.0.1 that counts the connections it accepts and answers per mode.
class CountingListener {
 public:
  enum class Mode {
    kClose,         ///< close at once
    kHold,          ///< keep open, never answer, until release()
    kEchoPassword,  ///< ask for a clear-text password, then fail with a message that contains it
    kCloseOnQuery,  ///< complete the handshake, close on the first query
  };

  explicit CountingListener(Mode mode = Mode::kClose) : mode_(mode) {
    fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    socklen_t len = sizeof(addr);
    if (fd_ < 0 || ::bind(fd_, reinterpret_cast<sockaddr *>(&addr), len) != 0 || ::listen(fd_, 16) != 0 ||
        ::getsockname(fd_, reinterpret_cast<sockaddr *>(&addr), &len) != 0) {
      throw std::runtime_error("cannot open the test listener");
    }
    port_ = ntohs(addr.sin_port);
    thread_ = std::thread([this] {
      run();
    });
  }
  ~CountingListener() {
    stop_ = true;
    thread_.join();
    release();
    for (const int client : watched_) {
      ::close(client);
    }
    ::close(fd_);
  }
  CountingListener(const CountingListener &) = delete;
  CountingListener & operator=(const CountingListener &) = delete;
  CountingListener(CountingListener &&) = delete;
  CountingListener & operator=(CountingListener &&) = delete;

  /// Closes the held connections.
  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const int client : held_) {
      ::close(client);
    }
    held_.clear();
  }

  int port() const {
    return port_;
  }
  int accepted() const {
    return accepted_;
  }
  std::vector<std::chrono::steady_clock::time_point> accept_times() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return accept_times_;
  }

 private:
  static std::string be32(uint32_t value) {
    return {static_cast<char>((value >> 24) & 0xFF), static_cast<char>((value >> 16) & 0xFF),
            static_cast<char>((value >> 8) & 0xFF), static_cast<char>(value & 0xFF)};
  }
  static std::string message(char type, const std::string & body) {
    return std::string(1, type) + be32(static_cast<uint32_t>(body.size() + 4)) + body;
  }
  static std::string cstr(const std::string & text) {
    return text + std::string(1, '\0');
  }
  /// Reads @p size bytes, or fewer when the peer closes or stays silent for 2 s.
  static std::string read_bytes(int fd, size_t size) {
    std::string out;
    while (out.size() < size) {
      pollfd pfd{fd, POLLIN, 0};
      if (::poll(&pfd, 1, 2000) <= 0) {
        break;
      }
      std::array<char, 512> buffer{};
      const ssize_t got = ::recv(fd, buffer.data(), std::min(buffer.size(), size - out.size()), 0);
      if (got <= 0) {
        break;
      }
      out.append(buffer.data(), static_cast<size_t>(got));
    }
    return out;
  }
  /// Body of the next message; @p startup for the untyped startup packet.
  static std::string read_message(int fd, bool startup) {
    if (!startup) {
      read_bytes(fd, 1);
    }
    const std::string header = read_bytes(fd, 4);
    if (header.size() < 4) {
      return {};
    }
    const auto byte = [&header](size_t i) {
      return static_cast<uint32_t>(static_cast<unsigned char>(header[i]));
    };
    const uint32_t size = (byte(0) << 24) | (byte(1) << 16) | (byte(2) << 8) | byte(3);
    return size < 4 ? std::string{} : read_bytes(fd, size - 4);
  }
  static void send_all(int fd, const std::string & data) {
    size_t sent = 0;
    while (sent < data.size()) {
      const ssize_t n = ::send(fd, data.data() + sent, data.size() - sent, MSG_NOSIGNAL);
      if (n <= 0) {
        return;
      }
      sent += static_cast<size_t>(n);
    }
  }

  void run() {
    while (!stop_) {
      std::vector<pollfd> fds{{fd_, POLLIN, 0}};
      for (const int client : watched_) {
        fds.push_back({client, POLLIN, 0});
      }
      if (::poll(fds.data(), fds.size(), 20) <= 0) {
        continue;
      }
      for (size_t i = 1; i < fds.size(); ++i) {
        if (fds[i].revents != 0) {
          ::close(fds[i].fd);
          watched_.erase(std::find(watched_.begin(), watched_.end(), fds[i].fd));
        }
      }
      if ((fds[0].revents & POLLIN) != 0) {
        const int client = ::accept(fd_, nullptr, nullptr);
        if (client >= 0) {
          serve(client);
        }
      }
    }
  }

  void serve(int client) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      accept_times_.push_back(std::chrono::steady_clock::now());
    }
    ++accepted_;
    switch (mode_) {
      case Mode::kClose:
        ::close(client);
        return;
      case Mode::kHold: {
        std::lock_guard<std::mutex> lock(mutex_);
        held_.push_back(client);
        return;
      }
      case Mode::kEchoPassword: {
        read_message(client, true);
        send_all(client, message('R', be32(3)));
        std::string password = read_message(client, false);
        if (!password.empty() && password.back() == '\0') {
          password.pop_back();
        }
        send_all(client, message('E', cstr("SFATAL") + cstr("VFATAL") + cstr("C28P01") +
                                          cstr("Mauthentication rejected; supplied password was " + password) +
                                          std::string(1, '\0')));
        ::close(client);
        return;
      }
      case Mode::kCloseOnQuery: {
        read_message(client, true);
        std::string out = message('R', be32(0));
        for (const auto & [key, value] :
             std::vector<std::pair<std::string, std::string>>{{"server_version", "16.0"},
                                                              {"server_encoding", "UTF8"},
                                                              {"client_encoding", "UTF8"},
                                                              {"standard_conforming_strings", "on"},
                                                              {"integer_datetimes", "on"},
                                                              {"DateStyle", "ISO, MDY"}}) {
          out += message('S', cstr(key) + cstr(value));
        }
        out += message('K', be32(1) + be32(2));
        out += message('Z', "I");
        send_all(client, out);
        watched_.push_back(client);
        return;
      }
    }
  }

  Mode mode_;
  int fd_{-1};
  int port_{0};
  std::atomic<bool> stop_{false};
  std::atomic<int> accepted_{0};
  mutable std::mutex mutex_;
  std::vector<int> held_;
  std::vector<std::chrono::steady_clock::time_point> accept_times_;
  std::vector<int> watched_;  ///< only touched by thread_
  std::thread thread_;
};

/// Sets or unsets an environment variable for one scope.
class ScopedEnv {
 public:
  ScopedEnv(const char * name, const char * value) : name_(name) {
    if (const char * old = std::getenv(name)) {
      old_ = old;
    }
    if (value != nullptr) {
      setenv(name, value, 1);
    } else {
      unsetenv(name);
    }
  }
  ~ScopedEnv() {
    if (old_) {
      setenv(name_.c_str(), old_->c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }
  ScopedEnv(const ScopedEnv &) = delete;
  ScopedEnv & operator=(const ScopedEnv &) = delete;
  ScopedEnv(ScopedEnv &&) = delete;
  ScopedEnv & operator=(ScopedEnv &&) = delete;

 private:
  std::string name_;
  std::optional<std::string> old_;
};

/// Connection string to @p server without TLS or GSS negotiation.
std::string local_conn(const CountingListener & server) {
  return "host=127.0.0.1 port=" + std::to_string(server.port()) + " dbname=x user=x sslmode=disable gssencmode=disable";
}

struct UnansweredStart {
  std::chrono::steady_clock::duration elapsed;
  bool connected;
  std::string target;
};

/// Starts a storage against a held @p server, gives it at most 8 s, then releases the server.
UnansweredStart start_unanswered(const std::string & conn, CountingListener & server) {
  const auto start = std::chrono::steady_clock::now();
  auto result = std::async(std::launch::async, [&conn] {
    PgFaultStorage storage(conn, 0, 0);
    return std::make_pair(storage.connected(), storage.target());
  });
  const auto status = result.wait_for(std::chrono::seconds(8));
  const auto elapsed = std::chrono::steady_clock::now() - start;
  server.release();
  EXPECT_EQ(status, std::future_status::ready) << "the attempt did not time out";
  const auto [connected, target] = result.get();
  return {elapsed, connected, target};
}

// Connection attempts with the node's retry policy, counted and timed at the server: two 500 ms
// apart when the storage starts, none during the 5 s backoff, then one per round.
TEST(PgFaultStorageConnectionTest, OneAttemptPerRoundAfterTheBackoff) {
  using ros2_medkit_fault_manager::FaultStorage;
  CountingListener server;
  PgFaultStorage storage(local_conn(server));
  ASSERT_FALSE(storage.connected());
  const auto attempts = server.accept_times();
  ASSERT_EQ(attempts.size(), 2u);
  EXPECT_GE(attempts[1] - attempts[0], std::chrono::milliseconds(450));

  const auto start = std::chrono::steady_clock::now();
  EXPECT_THROW(storage.size(), FaultStorage::IgnorableConnectionException);
  EXPECT_THROW(storage.size(), FaultStorage::IgnorableConnectionException);
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::milliseconds(100)) << "did not fail at once";
  EXPECT_EQ(server.accepted(), 2);

  std::this_thread::sleep_until(start + std::chrono::milliseconds(4500));
  EXPECT_THROW(storage.size(), FaultStorage::IgnorableConnectionException);
  EXPECT_EQ(server.accepted(), 2) << "the backoff ended before 5 s";

  std::this_thread::sleep_until(start + std::chrono::milliseconds(5500));
  EXPECT_THROW(storage.size(), FaultStorage::IgnorableConnectionException);
  EXPECT_EQ(server.accepted(), 3);
}

// A server that accepts the connection and never answers. Without connect_timeout in database_url
// the attempt still ends after the default of 2 s.
TEST(PgFaultStorageConnectionTest, AnUnansweredAttemptEndsAfterTheDefaultTimeout) {
  ScopedEnv no_env_timeout("PGCONNECT_TIMEOUT", nullptr);
  CountingListener server(CountingListener::Mode::kHold);
  const auto result = start_unanswered(local_conn(server), server);
  EXPECT_FALSE(result.connected);
  EXPECT_LT(result.elapsed, std::chrono::seconds(4));
  EXPECT_EQ(server.accepted(), 1);
}

// connect_timeout in database_url, or PGCONNECT_TIMEOUT, replaces the default of 2 s.
TEST(PgFaultStorageConnectionTest, AGivenConnectTimeoutReplacesTheDefault) {
  for (const bool from_env : {false, true}) {
    ScopedEnv env_timeout("PGCONNECT_TIMEOUT", from_env ? "3" : nullptr);
    CountingListener server(CountingListener::Mode::kHold);
    const auto result = start_unanswered(local_conn(server) + (from_env ? "" : " connect_timeout=3"), server);
    EXPECT_GE(result.elapsed, std::chrono::milliseconds(2500)) << (from_env ? "PGCONNECT_TIMEOUT" : "database_url");
    EXPECT_LT(result.elapsed, std::chrono::milliseconds(4500)) << (from_env ? "PGCONNECT_TIMEOUT" : "database_url");
  }
}

// With a libpq service, the service file sets the timeout. The storage adds no default over it, and
// the target names the service.
TEST(PgFaultStorageConnectionTest, AServiceKeepsItsOwnConnectTimeout) {
  CountingListener server(CountingListener::Mode::kHold);
  const auto file = std::filesystem::temp_directory_path() / ("medkit_pg_service_" + unique_suffix() + ".conf");
  {
    std::ofstream out(file);
    out << "[alpha]\nhost=127.0.0.1\nport=" << server.port()
        << "\ndbname=x\nuser=x\nsslmode=disable\ngssencmode=disable\nconnect_timeout=3\n";
  }
  ScopedEnv service_file("PGSERVICEFILE", file.c_str());
  ScopedEnv no_env_timeout("PGCONNECT_TIMEOUT", nullptr);
  const auto result = start_unanswered("service=alpha", server);
  std::filesystem::remove(file);
  EXPECT_EQ(server.accepted(), 1) << "the service was not used";
  EXPECT_GE(result.elapsed, std::chrono::milliseconds(2500));
  EXPECT_LT(result.elapsed, std::chrono::milliseconds(4500));
  EXPECT_EQ(result.target, "service=alpha");
}

// An empty database_url takes the server from the libpq environment variables.
TEST(PgFaultStorageConnectionTest, AnEmptyDatabaseUrlUsesTheEnvironment) {
  CountingListener server;
  const std::string port = std::to_string(server.port());
  ScopedEnv host("PGHOST", "127.0.0.1");
  ScopedEnv port_env("PGPORT", port.c_str());
  ScopedEnv dbname("PGDATABASE", "x");
  ScopedEnv user("PGUSER", "x");
  ScopedEnv ssl("PGSSLMODE", "disable");
  ScopedEnv gss("PGGSSENCMODE", "disable");
  PgFaultStorage storage("", 0, 0);
  EXPECT_EQ(server.accepted(), 1);
  EXPECT_NE(storage.target().find("host=127.0.0.1 port=" + port + " dbname=x user=x"), std::string::npos)
      << storage.target();
}

// An empty host or port in database_url overrides PGHOST and PGPORT, in libpq and in the target.
TEST(PgFaultStorageConnectionTest, TheTargetShowsAnEmptyOverride) {
  ScopedEnv host("PGHOST", "env-host.invalid");
  ScopedEnv port("PGPORT", "6543");
  PgFaultStorage storage("host='' port='' dbname=x user=x connect_timeout=2", 0, 0);
  EXPECT_EQ(storage.target().find("env-host.invalid"), std::string::npos) << storage.target();
  EXPECT_EQ(storage.target().find("6543"), std::string::npos) << storage.target();
}

// The server writes the error text, and a hostile one can put the password it received in it.
// The storage removes the password it knows before it logs or reports that text.
TEST(PgFaultStorageConnectionTest, APasswordEchoedByTheServerIsRemoved) {
  const std::string secret = "Echo-Secret-" + unique_suffix();
  for (const bool from_env : {false, true}) {
    CountingListener server(CountingListener::Mode::kEchoPassword);
    ScopedEnv env_password("PGPASSWORD", from_env ? secret.c_str() : nullptr);
    LogCapture log;
    PgFaultStorage storage(local_conn(server) + (from_env ? "" : " password=" + secret), 0, 0);
    std::string reported;
    try {
      storage.size();
    } catch (const ros2_medkit_fault_manager::FaultStorage::IgnorableConnectionException & e) {
      reported = e.what();
    }
    const std::string logged = log.text();
    const char * source = from_env ? "PGPASSWORD" : "database_url";
    EXPECT_NE(logged.find("supplied password was"), std::string::npos) << source << ": no server message";
    EXPECT_EQ(logged.find(secret), std::string::npos) << source << ": " << logged;
    EXPECT_NE(reported.find("supplied password was"), std::string::npos) << source << ": " << reported;
    EXPECT_EQ(reported.find(secret), std::string::npos) << source << ": " << reported;
  }
}

// A server that drops every connection on its first query. When the transaction retries run out,
// the backoff starts: the next request fails at once and opens no connection.
TEST(PgFaultStorageConnectionTest, ATransactionThatKeepsFailingStartsTheBackoff) {
  CountingListener server(CountingListener::Mode::kCloseOnQuery);
  PgFaultStorage storage(local_conn(server), 1, 50);
  ASSERT_FALSE(storage.connected());
  const int after_start = server.accepted();
  EXPECT_EQ(after_start, 2);
  EXPECT_THROW(storage.size(), ros2_medkit_fault_manager::FaultStorage::IgnorableConnectionException);
  EXPECT_EQ(server.accepted(), after_start);
}

// database_url is passed to libpq in key='value' form. A URI password with a quote, a backslash
// and a space must survive that.
TEST(PgFaultStorageConnectionTest, ConnectsWithAPasswordThatNeedsEscaping) {
  std::unique_ptr<pqxx::connection> admin;
  try {
    admin = std::make_unique<pqxx::connection>(base_conn_info());
  } catch (const std::exception & e) {
    GTEST_FAIL() << "No PostgreSQL server reachable (set ROS2_MEDKIT_TEST_PG_CONN): " << e.what();
  }
  const std::string suffix = unique_suffix();
  const std::string role = "medkit_esc_" + suffix;
  const std::string schema = "medkit_esc_schema_" + suffix;
  const std::string password = "p'a\\ss w";
  {
    pqxx::nontransaction ntx(*admin);
    ntx.exec("CREATE ROLE " + ntx.quote_name(role) + " LOGIN PASSWORD " + ntx.quote(password));
    ntx.exec("CREATE SCHEMA " + ntx.quote_name(schema) + " AUTHORIZATION " + ntx.quote_name(role));
  }
  const std::string uri = "postgresql://" + role + ":p%27a%5Css%20w@" + base_conn_option("host") + ":" +
                          base_conn_option("port") + "/" + base_conn_option("dbname") + "?options=-csearch_path%3D" +
                          schema;
  {
    PgFaultStorage storage(uri);
    EXPECT_TRUE(storage.connected());
    EXPECT_EQ(storage.size(), 0u);
  }
  pqxx::nontransaction ntx(*admin);
  ntx.exec("DROP SCHEMA IF EXISTS " + ntx.quote_name(schema) + " CASCADE");
  ntx.exec("DROP ROLE IF EXISTS " + ntx.quote_name(role));
}

// A wrong password is not decidable from libpq's text alone, so the storage starts without the
// database. Neither the errors, the log nor the target carries the password. The test server must
// require a password (the CI service and the compose files do).
TEST(PgFaultStorageConnectionTest, AWrongPasswordIsNeverEchoed) {
  const std::string secret = "Wr0ng-Secret-" + unique_suffix();
  LogCapture log;
  PgFaultStorage storage(conn_as(base_conn_option("user"), secret));
  ASSERT_FALSE(storage.connected()) << "the test server accepted a wrong password";
  EXPECT_EQ(storage.target().find(secret), std::string::npos);
  try {
    storage.size();
    ADD_FAILURE() << "size() reached a server that rejected the password";
  } catch (const ros2_medkit_fault_manager::FaultStorage::IgnorableConnectionException & e) {
    EXPECT_EQ(std::string(e.what()).find(secret), std::string::npos) << e.what();
  }
  const std::string logged = log.text();
  EXPECT_NE(logged.find("password authentication failed"), std::string::npos) << "no connection error logged";
  EXPECT_EQ(logged.find(secret), std::string::npos) << logged;
}

// A string libpq cannot parse is a wrong configuration. libpq's own message would quote the
// fragment after the space, which here is the tail of the password.
TEST(PgFaultStorageConnectionTest, AMalformedConnectionStringStopsWithoutEchoingIt) {
  const std::string tail = "tail-of-the-password";
  try {
    PgFaultStorage storage("host=localhost password=first " + tail);
    ADD_FAILURE() << "a malformed connection string was accepted";
  } catch (const std::invalid_argument & e) {
    EXPECT_EQ(std::string(e.what()).find(tail), std::string::npos) << e.what();
    EXPECT_EQ(std::string(e.what()).find("first"), std::string::npos) << e.what();
  }
}

// A server that accepts the connection but refuses the schema is a wrong configuration.
TEST(PgFaultStorageConnectionTest, AServerThatRefusesTheSchemaStopsTheStartup) {
  std::unique_ptr<pqxx::connection> admin;
  try {
    admin = std::make_unique<pqxx::connection>(base_conn_info());
  } catch (const std::exception & e) {
    GTEST_FAIL() << "No PostgreSQL server reachable (set ROS2_MEDKIT_TEST_PG_CONN): " << e.what();
  }
  const std::string suffix = unique_suffix();
  const std::string role = "medkit_ro_" + suffix;
  const std::string schema = "medkit_ro_schema_" + suffix;
  {
    pqxx::nontransaction ntx(*admin);
    ntx.exec("CREATE ROLE " + ntx.quote_name(role) + " LOGIN PASSWORD 'ro-pass'");
    ntx.exec("CREATE SCHEMA " + ntx.quote_name(schema));
    ntx.exec("GRANT USAGE ON SCHEMA " + ntx.quote_name(schema) + " TO " + ntx.quote_name(role));
  }

  EXPECT_THROW(PgFaultStorage(conn_as(role, "ro-pass", " options='-csearch_path=" + schema + "'")), std::runtime_error);

  pqxx::nontransaction ntx(*admin);
  ntx.exec("DROP SCHEMA IF EXISTS " + ntx.quote_name(schema) + " CASCADE");
  ntx.exec("DROP ROLE IF EXISTS " + ntx.quote_name(role));
}

// A faults table of another layout is a wrong configuration: the storage does not start.
TEST(PgFaultStorageConnectionTest, ATableOfAnotherLayoutStopsTheStartup) {
  std::unique_ptr<pqxx::connection> admin;
  try {
    admin = std::make_unique<pqxx::connection>(base_conn_info());
  } catch (const std::exception & e) {
    GTEST_FAIL() << "No PostgreSQL server reachable (set ROS2_MEDKIT_TEST_PG_CONN): " << e.what();
  }
  const std::string schema = "medkit_layout_" + unique_suffix();
  {
    pqxx::nontransaction ntx(*admin);
    ntx.exec("CREATE SCHEMA " + ntx.quote_name(schema));
    ntx.exec("CREATE TABLE " + ntx.quote_name(schema) + ".faults (fault_code TEXT PRIMARY KEY, status TEXT NOT NULL)");
  }

  EXPECT_THROW(PgFaultStorage(conn_as(base_conn_option("user"), base_conn_option("password"),
                                      " options='-csearch_path=" + schema + "'")),
               std::runtime_error);

  pqxx::nontransaction ntx(*admin);
  ntx.exec("DROP SCHEMA IF EXISTS " + ntx.quote_name(schema) + " CASCADE");
}

// A connection the server ends is opened again, and the schema is created again on the new one.
TEST(PgFaultStorageConnectionTest, ALostConnectionIsOpenedAgainWithTheSchema) {
  std::unique_ptr<pqxx::connection> admin;
  try {
    admin = std::make_unique<pqxx::connection>(base_conn_info());
  } catch (const std::exception & e) {
    GTEST_FAIL() << "No PostgreSQL server reachable (set ROS2_MEDKIT_TEST_PG_CONN): " << e.what();
  }
  const std::string suffix = unique_suffix();
  const std::string schema = "medkit_reconnect_" + suffix;
  const std::string application = "medkit_reconnect_" + suffix;
  {
    pqxx::nontransaction ntx(*admin);
    ntx.exec("CREATE SCHEMA " + ntx.quote_name(schema));
  }
  rclcpp::Clock clock;
  {
    PgFaultStorage storage(conn_as(base_conn_option("user"), base_conn_option("password"),
                                   " application_name=" + application + " options='-csearch_path=" + schema + "'"));
    ASSERT_TRUE(storage.connected());
    storage.report_fault_event("LOST_A", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "a", "/node",
                               clock.now(), default_config());
    EXPECT_EQ(storage.size(), 1u);

    // A fresh, empty schema behind the next connection, and the current connection ended.
    {
      pqxx::nontransaction ntx(*admin);
      ntx.exec("DROP SCHEMA " + ntx.quote_name(schema) + " CASCADE");
      ntx.exec("CREATE SCHEMA " + ntx.quote_name(schema));
      ntx.exec("SELECT pg_terminate_backend(pid) FROM pg_stat_activity WHERE application_name = " +
               ntx.quote(application));
    }
    EXPECT_EQ(storage.size(), 0u);
    storage.report_fault_event("LOST_B", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "b", "/node",
                               clock.now(), default_config());
    EXPECT_EQ(storage.size(), 1u);
  }
  pqxx::nontransaction ntx(*admin);
  ntx.exec("DROP SCHEMA IF EXISTS " + ntx.quote_name(schema) + " CASCADE");
}

// Test reporting sources JSON handling
TEST_F(PgFaultStorageTest, ReportingSourcesJsonHandling) {
  rclcpp::Clock clock;
  auto timestamp = clock.now();

  // Add multiple sources for the same fault
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/node/path/with/slashes", timestamp, default_config());
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/another/node", timestamp, default_config());
  storage_->report_fault_event("MULTI_SRC", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Multi-source",
                               "/special\"chars", timestamp, default_config());

  auto fault = storage_->get_fault("MULTI_SRC");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->reporting_sources.size(), 3u);

  // Verify all sources are present (order may vary due to set)
  std::set<std::string> sources(fault->reporting_sources.begin(), fault->reporting_sources.end());
  EXPECT_TRUE(sources.count("/node/path/with/slashes") > 0);
  EXPECT_TRUE(sources.count("/another/node") > 0);
  EXPECT_TRUE(sources.count("/special\"chars") > 0);
}

// Test connection string accessor (equivalent of sqlite's DbPathAccessor).
TEST_F(PgFaultStorageTest, ConnInfoAccessor) {
  EXPECT_EQ(storage_->conn_info(), conn_info_);
}

// Debounce config tests for PostgreSQL storage
TEST_F(PgFaultStorageTest, DefaultDebounceConfig) {
  auto config = storage_->get_debounce_config();
  EXPECT_EQ(config.confirmation_threshold, -1);
  EXPECT_FALSE(config.healing_enabled);
  EXPECT_EQ(config.healing_threshold, 3);
  EXPECT_TRUE(config.critical_immediate_confirm);
}

TEST_F(PgFaultStorageTest, SetDebounceConfig) {
  DebounceConfig config;
  config.confirmation_threshold = -5;
  config.healing_enabled = true;
  config.healing_threshold = 5;
  config.critical_immediate_confirm = false;

  storage_->set_debounce_config(config);
  auto retrieved = storage_->get_debounce_config();

  EXPECT_EQ(retrieved.confirmation_threshold, -5);
  EXPECT_TRUE(retrieved.healing_enabled);
  EXPECT_EQ(retrieved.healing_threshold, 5);
  EXPECT_FALSE(retrieved.critical_immediate_confirm);
}

TEST_F(PgFaultStorageTest, FaultStaysPrefailedAboveThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (2 FAILED events should stay PREFAILED)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Still debouncing towards confirmation, same occurrence
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);
}

TEST_F(PgFaultStorageTest, FaultConfirmsAtThreshold) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior (3 FAILED events should confirm)
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);  // Debounce build-up is still one occurrence
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(PgFaultStorageTest, ImmediateConfirmationWithThresholdZero) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = 0;  // Immediate confirmation
  storage_->set_debounce_config(config);

  // Single report should confirm immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(PgFaultStorageTest, CriticalSeverityBypassesDebounce) {
  rclcpp::Clock clock;

  // CRITICAL severity should confirm immediately
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "Critical test",
                               "/node1", clock.now(), default_config());

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(PgFaultStorageTest, ClearedFaultCanBeReactivated) {
  rclcpp::Clock clock;

  // Report to confirm (with default threshold=-1, single report confirms)
  auto first_ts = clock.now();
  bool is_new = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                             "Initial", "/node1", first_ts, default_config());
  EXPECT_TRUE(is_new);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(fault->occurrence_count, 1u);
  EXPECT_EQ(rclcpp::Time(fault->first_occurred).nanoseconds(), first_ts.nanoseconds());

  // Clear the fault
  storage_->clear_fault("FAULT_1");
  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);

  // Report again after a gap - should reactivate as a new cycle
  rclcpp::Time second_ts(first_ts.nanoseconds() + 1'000'000'000LL);  // +1s
  is_new = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                                        "Reactivated", "/node2", second_ts, default_config());
  EXPECT_TRUE(is_new);  // Should return true like a new fault

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Should be reconfirmed
  EXPECT_EQ(fault->occurrence_count, 2u);             // Should increment
  EXPECT_EQ(fault->reporting_sources.size(), 2u);     // Both sources
  EXPECT_EQ(fault->description, "Reactivated");       // Updated description
  // #25: first_occurred must reflect the new cycle, not the outage that already cleared.
  EXPECT_EQ(rclcpp::Time(fault->first_occurred).nanoseconds(), second_ts.nanoseconds());
}

TEST_F(PgFaultStorageTest, PassedEventForClearedFaultIgnored) {
  rclcpp::Clock clock;

  // Report and confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), default_config());

  // Clear the fault
  storage_->clear_fault("FAULT_1");

  // PASSED event should be ignored for CLEARED fault
  bool result = storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1",
                                             clock.now(), default_config());
  EXPECT_FALSE(result);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CLEARED);  // Should stay cleared
}

TEST_F(PgFaultStorageTest, ClearedFaultReactivationRestartsDebounce) {
  rclcpp::Clock clock;

  // Set threshold to -3 to test debounce behavior
  DebounceConfig config;
  config.confirmation_threshold = -3;
  storage_->set_debounce_config(config);

  // Report 3 times to confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now(), config);

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // Clear the fault
  storage_->clear_fault("FAULT_1");

  // Reactivate - should start in PREFAILED with counter=-1
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node4",
                               clock.now(), config);

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREFAILED);  // Not yet confirmed, needs 2 more FAILED

  // Report 2 more times to re-confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node5",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node6",
                               clock.now(), config);

  fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);  // Now confirmed
}

TEST_F(PgFaultStorageTest, ConfirmationPersistsAfterReopen) {
  rclcpp::Clock clock;

  // Report 3 times to confirm
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), default_config());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), default_config());
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node3",
                               clock.now(), default_config());

  // Close and reopen storage
  storage_.reset();
  storage_ = std::make_unique<PgFaultStorage>(conn_info_);

  // Verify status persisted
  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

TEST_F(PgFaultStorageTest, PassedEventIncrementsCounter) {
  rclcpp::Clock clock;
  // confirmation_threshold = -3 so 2 FAILED stays PREFAILED (not CONFIRMED). A
  // confirmed fault is latched and would not move to PREPASSED on a heal.
  DebounceConfig config;
  config.confirmation_threshold = -3;

  // Report 2 FAILED events (counter -2, PREFAILED)
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node2",
                               clock.now(), config);

  // Report 3 PASSED events (counter -2 -> +1)
  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_PREPASSED);  // Counter > 0, never confirmed so not latched
}

// Regression for #428: a periodic heal heartbeat on a healthy system used to push
// the debounce counter toward INT32_MAX, so a real fault then took a huge number of
// reports to confirm. The counter must clamp at the thresholds, and a confirmed or
// healed status must latch (one report must not flip it).
TEST_F(PgFaultStorageTest, HeartbeatHealClampedAndStatusLatched) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  ASSERT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);

  // Long heal heartbeat: counter clamps at healing_threshold instead of running off.
  for (int i = 0; i < 100; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }
  auto healed = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(healed.has_value());
  EXPECT_EQ(healed->status, Fault::STATUS_HEALED);

  // Re-confirmation is now bounded to (healing_threshold - confirmation_threshold)
  // reports, and the healed status latches until the counter walks all the way down.
  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                                 clock.now(), config);
    EXPECT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_HEALED) << "latch broke after " << (i + 1);
  }
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  auto reconfirmed = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(reconfirmed.has_value());
  EXPECT_EQ(reconfirmed->status, Fault::STATUS_CONFIRMED);
}

// A confirmed fault must not be un-confirmed by a heal heartbeat when auto-healing
// is off, and the counter must stay bounded.
TEST_F(PgFaultStorageTest, ConfirmedFaultSurvivesHealHeartbeat) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_enabled = false;
  config.healing_threshold = 3;  // upper clamp on the counter, even with healing disabled

  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  ASSERT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);

  for (int i = 0; i < 20; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
    EXPECT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED) << "un-confirmed after " << (i + 1);
  }

  // The counter is now positive (clamped at +3). One FAILED must keep it CONFIRMED, not flip it to
  // PREPASSED via the `counter > 0` branch - that was the asymmetric-latch bug.
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);
  EXPECT_EQ(storage_->get_fault("FAULT_1")->status, Fault::STATUS_CONFIRMED);
}

// Latch must hold in BOTH directions and identically on all backends (regression for the
// asymmetric per-branch latch). A CONFIRMED fault with a positive counter stays CONFIRMED on FAILED.
TEST_F(PgFaultStorageTest, ConfirmedLatchSurvivesFailedAtPositiveCounter) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_threshold = 3;
  config.critical_immediate_confirm = true;

  // CRITICAL confirms immediately at counter -1.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL, "crit", "/n",
                               clock.now(), config);
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  // PASSED events raise the counter into positive territory; latch keeps it CONFIRMED.
  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  // One normal FAILED: counter still positive, must NOT become PREPASSED.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
}

// A HEALED fault with a negative counter stays HEALED on PASSED (opposite direction of the bug above).
TEST_F(PgFaultStorageTest, HealedLatchSurvivesPassedAtNegativeCounter) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  for (int i = 0; i < 4; ++i) {  // counter -1 -> +3 -> HEALED
    storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED);
  // FAILED events drive the counter negative while HEALED is latched (not yet at confirmation -3).
  for (int i = 0; i < 5; ++i) {  // +3 -> -2
    storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                                 config);
    ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED) << "latch broke after " << (i + 1);
  }
  // One PASSED: counter goes -2 -> -1, still negative; must stay HEALED, not flip to PREFAILED.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED);
}

// A database written by an older build can hold a runaway counter. It must be clamped back on first
// touch so re-confirmation is bounded, not ~INT32 events away.
TEST_F(PgFaultStorageTest, RunawayCounterFromOldRowRecovers) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -1;
  config.healing_threshold = 3;

  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  // Simulate the old bug: poke a huge counter directly into the table behind the storage's back.
  exec_raw("UPDATE faults SET debounce_counter = 100000, status = 'PREPASSED'");

  // The counter is clamped back into range on first touch, so a bounded number of FAILED events
  // re-confirms (healing_threshold - confirmation_threshold = 4), not the ~100000 the runaway implied.
  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                                 config);
  }
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
}

TEST_F(PgFaultStorageTest, ReclassifyHealedAsCleared) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;

  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", clock.now(),
                               config);
  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("F", ReportFault::Request::EVENT_PASSED, 0, "", "/n", clock.now(), config);
  }
  ASSERT_EQ(storage_->get_fault("F")->status, Fault::STATUS_HEALED);

  const auto reclassified = storage_->reclassify_healed_as_cleared();
  ASSERT_EQ(reclassified.size(), 1u);
  EXPECT_EQ(reclassified[0], "F");
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CLEARED);
  EXPECT_TRUE(storage_->reclassify_healed_as_cleared().empty());
}

TEST_F(PgFaultStorageTest, HealingWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 3;
  storage_->set_debounce_config(config);

  // Report 1 FAILED event
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               clock.now(), config);

  // Report 4 PASSED events (counter = -1 + 4 = +3, reaches healing threshold)
  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_PASSED, 0, "", "/node1", clock.now(), config);
  }

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_HEALED);
}

TEST_F(PgFaultStorageTest, TimeBasedConfirmationWhenEnabled) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;  // Need debounce so fault stays PREFAILED
  config.auto_confirm_after_sec = 10.0;
  storage_->set_debounce_config(config);

  auto now = clock.now();
  storage_->report_fault_event("FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               now, config);

  // Check before timeout - should not confirm
  auto before_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(5e9));
  auto confirmed_early = storage_->check_time_based_confirmation(before_timeout);
  EXPECT_TRUE(confirmed_early.empty());

  // Check after timeout - should confirm
  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  auto confirmed = storage_->check_time_based_confirmation(after_timeout);
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0], "FAULT_1");

  auto fault = storage_->get_fault("FAULT_1");
  ASSERT_TRUE(fault.has_value());
  EXPECT_EQ(fault->status, Fault::STATUS_CONFIRMED);
}

// The returned codes are exactly the rows confirmed: the node audits and publishes only those.
TEST_F(PgFaultStorageTest, TimeBasedConfirmationConfirmsOnlyReturnedFaults) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.auto_confirm_after_sec = 10.0;
  storage_->set_debounce_config(config);

  auto now = clock.now();
  storage_->report_fault_event("TIMED", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test", "/node1",
                               now, config);
  storage_->report_fault_event("NO_FAILURE_TIME", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "Test",
                               "/node1", now, config);
  // A PREFAILED row without a failure time is not eligible.
  exec_raw("UPDATE faults SET last_failed_ns = 0 WHERE fault_code = 'NO_FAILURE_TIME'");

  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  auto confirmed = storage_->check_time_based_confirmation(after_timeout);
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(confirmed[0], "TIMED");

  auto skipped = storage_->get_fault("NO_FAILURE_TIME");
  ASSERT_TRUE(skipped.has_value());
  EXPECT_EQ(skipped->status, Fault::STATUS_PREFAILED);
}

TEST_F(PgFaultStorageTest, ConfirmedAtRecordedOnImmediateConfirmation) {
  rclcpp::Clock clock;
  auto t = clock.now();
  // Default config confirms on the first FAILED event.
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t,
                               default_config());
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(read_confirmed_at("F"), t.nanoseconds());

  // A later FAILED on an already-confirmed fault must NOT move the timestamp.
  auto t2 = rclcpp::Time(t.nanoseconds() + static_cast<int64_t>(5e9));
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t2,
                               default_config());
  EXPECT_EQ(read_confirmed_at("F"), t.nanoseconds());
}

TEST_F(PgFaultStorageTest, ConfirmedAtRecordedOnDebouncedConfirmation) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -2;  // second FAILED event confirms

  auto t1 = clock.now();
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t1, config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_PREFAILED);
  EXPECT_EQ(read_confirmed_at("F"), 0) << "not confirmed yet: no confirmation timestamp";

  auto t2 = rclcpp::Time(t1.nanoseconds() + static_cast<int64_t>(1e9));
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t2, config);
  EXPECT_EQ(storage_->get_fault("F")->status, Fault::STATUS_CONFIRMED);
  EXPECT_EQ(read_confirmed_at("F"), t2.nanoseconds());
}

TEST_F(PgFaultStorageTest, ConfirmedAtRecordedOnTimeBasedConfirmation) {
  rclcpp::Clock clock;
  DebounceConfig config;
  config.confirmation_threshold = -3;
  config.auto_confirm_after_sec = 10.0;
  storage_->set_debounce_config(config);

  auto now = clock.now();
  storage_->report_fault_event("F", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", now, config);

  auto after_timeout = rclcpp::Time(now.nanoseconds() + static_cast<int64_t>(15e9));
  auto confirmed = storage_->check_time_based_confirmation(after_timeout);
  ASSERT_EQ(confirmed.size(), 1u);
  EXPECT_EQ(read_confirmed_at("F"), after_timeout.nanoseconds());
}

// A row inserted without confirmed_at_ns gets the column default 0.
TEST_F(PgFaultStorageTest, ConfirmedAtDefaultsToZeroForExternallyInsertedRow) {
  exec_raw(
      "INSERT INTO faults (fault_code, severity, description, first_occurred_ns, last_occurred_ns, "
      "occurrence_count, status, reporting_sources) "
      "VALUES ('OLD', 2, 'd', 1, 1, 1, 'CONFIRMED', '[\"/n\"]')");

  EXPECT_TRUE(storage_->contains("OLD"));
  EXPECT_EQ(read_confirmed_at("OLD"), 0) << "rows carrying no confirmation time default to 0";

  rclcpp::Clock clock;
  auto t = clock.now();
  storage_->report_fault_event("NEW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "e", "/n", t,
                               default_config());
  EXPECT_EQ(read_confirmed_at("NEW"), t.nanoseconds());
}

// --- #620: many recordings per fault -----------------------------------------

namespace {

RosbagFileInfo make_rosbag(const std::string & code, const std::string & path, int64_t created_ns,
                           size_t bytes = 1024) {
  RosbagFileInfo info;
  info.fault_code = code;
  info.file_path = path;
  info.format = "mcap";
  info.duration_sec = 6.0;
  info.size_bytes = bytes;
  info.created_at_ns = created_ns;
  return info;  // recording_id deliberately left empty - the backend must fill it
}

}  // namespace
TEST_F(PgFaultStorageTest, OneFaultKeepsSeveralRecordingsWhenTheCapAllows) {
  storage_->set_max_rosbags_per_fault(3);
  storage_->store_rosbag_file(make_rosbag("FLAP", "/bags/fault_FLAP_100", 100));
  storage_->store_rosbag_file(make_rosbag("FLAP", "/bags/fault_FLAP_200", 200));

  const auto rows = storage_->get_rosbag_files("FLAP");
  ASSERT_EQ(rows.size(), 2u) << "the second recording must not replace the first";
  EXPECT_EQ(rows[0].recording_id, "fault_FLAP_200") << "newest first";
  EXPECT_EQ(rows[1].recording_id, "fault_FLAP_100");

  // get_rosbag_file is "the newest", deterministically.
  const auto newest = storage_->get_rosbag_file("FLAP");
  ASSERT_TRUE(newest.has_value());
  EXPECT_EQ(newest->recording_id, "fault_FLAP_200");
}

TEST_F(PgFaultStorageTest, CapKeepsTheNewestAndUnlinksTheEvictedBag) {
  const auto dir_a = temp_root_ / "fault_CAP_100";
  const auto dir_b = temp_root_ / "fault_CAP_200";
  std::filesystem::create_directories(dir_a);
  std::filesystem::create_directories(dir_b);

  storage_->set_max_rosbags_per_fault(1);
  storage_->store_rosbag_file(make_rosbag("CAP", dir_a.string(), 100));
  storage_->store_rosbag_file(make_rosbag("CAP", dir_b.string(), 200));

  const auto rows = storage_->get_rosbag_files("CAP");
  ASSERT_EQ(rows.size(), 1u) << "cap 1 is the historical behaviour: one recording per fault";
  EXPECT_EQ(rows[0].file_path, dir_b.string());
  EXPECT_FALSE(std::filesystem::exists(dir_a)) << "the evicted bag must be unlinked";
  EXPECT_TRUE(std::filesystem::exists(dir_b));
}

TEST_F(PgFaultStorageTest, EvictingOneFaultsLinkKeepsABagASiblingStillReferences) {
  // A burst shares one recording. Evicting it for one fault must not take the bytes
  // the other fault still points at.
  const auto shared = temp_root_ / "fault_SHARED_100";
  const auto later = temp_root_ / "fault_A_200";
  std::filesystem::create_directories(shared);
  std::filesystem::create_directories(later);

  storage_->set_max_rosbags_per_fault(1);
  storage_->store_rosbag_files({make_rosbag("A", shared.string(), 100), make_rosbag("B", shared.string(), 100)});
  // A re-confirms with its own new bag, so A's link to the shared one is evicted.
  storage_->store_rosbag_file(make_rosbag("A", later.string(), 200));

  EXPECT_TRUE(std::filesystem::exists(shared)) << "B still references it";
  const auto b_rows = storage_->get_rosbag_files("B");
  ASSERT_EQ(b_rows.size(), 1u);
  EXPECT_EQ(b_rows[0].file_path, shared.string());
}

TEST_F(PgFaultStorageTest, RecordingLookupReturnsEveryFaultOfTheBurst) {
  storage_->store_rosbag_files({make_rosbag("A", "/bags/fault_A_100", 100), make_rosbag("B", "/bags/fault_A_100", 100),
                                make_rosbag("C", "/bags/fault_A_100", 100)});

  const auto rows = storage_->get_rosbag_files_by_recording("fault_A_100");
  ASSERT_EQ(rows.size(), 3u) << "the download authorizes against this set";
  EXPECT_EQ(rows[0].fault_code, "A");
  EXPECT_EQ(rows[2].fault_code, "C");
  EXPECT_TRUE(storage_->get_rosbag_files_by_recording("fault_NOPE_1").empty());
}

TEST_F(PgFaultStorageTest, DeleteRecordingRemovesEveryLinkAndTheBag) {
  const auto dir = temp_root_ / "fault_A_100";
  std::filesystem::create_directories(dir);
  storage_->store_rosbag_files({make_rosbag("A", dir.string(), 100), make_rosbag("B", dir.string(), 100)});

  EXPECT_EQ(storage_->delete_rosbag_recording("fault_A_100"), 2u);
  EXPECT_TRUE(storage_->get_rosbag_files("A").empty());
  EXPECT_TRUE(storage_->get_rosbag_files("B").empty());
  EXPECT_FALSE(std::filesystem::exists(dir));
}

TEST_F(PgFaultStorageTest, DeletingAFaultDropsAllItsRecordings) {
  storage_->set_max_rosbags_per_fault(0);
  storage_->store_rosbag_file(make_rosbag("A", "/bags/fault_A_100", 100));
  storage_->store_rosbag_file(make_rosbag("A", "/bags/fault_A_200", 200));

  EXPECT_TRUE(storage_->delete_rosbag_file("A"));
  EXPECT_TRUE(storage_->get_rosbag_files("A").empty()) << "auto_cleanup drops the fault's whole history";
}

TEST_F(PgFaultStorageTest, SharedRecordingStillCountsOnceTowardsStorageWithSeveralRecordings) {
  storage_->set_max_rosbags_per_fault(0);
  storage_->store_rosbag_files(
      {make_rosbag("A", "/bags/fault_A_100", 100, 4096), make_rosbag("B", "/bags/fault_A_100", 100, 4096)});
  storage_->store_rosbag_file(make_rosbag("A", "/bags/fault_A_200", 200, 1024));

  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 4096u + 1024u) << "bytes belong to file_path, not to the row";
}

// Snapshot storage tests
// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, StoreAndRetrieveSnapshot) {
  using ros2_medkit_fault_manager::SnapshotData;

  // First, create a fault to associate the snapshot with
  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  // Store a snapshot
  SnapshotData snapshot;
  snapshot.fault_code = "MOTOR_OVERHEAT";
  snapshot.topic = "/motor/temperature";
  snapshot.message_type = "sensor_msgs/msg/Temperature";
  snapshot.data = R"({"temperature": 85.5, "variance": 0.1})";
  snapshot.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot);

  // Retrieve snapshots
  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");
  ASSERT_EQ(snapshots.size(), 1u);

  EXPECT_EQ(snapshots[0].fault_code, "MOTOR_OVERHEAT");
  EXPECT_EQ(snapshots[0].topic, "/motor/temperature");
  EXPECT_EQ(snapshots[0].message_type, "sensor_msgs/msg/Temperature");
  EXPECT_EQ(snapshots[0].data, R"({"temperature": 85.5, "variance": 0.1})");
  EXPECT_EQ(snapshots[0].captured_at_ns, snapshot.captured_at_ns);
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, MultipleSnapshotsForSameFault) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  // Store multiple snapshots for the same fault
  SnapshotData snapshot1;
  snapshot1.fault_code = "MOTOR_OVERHEAT";
  snapshot1.topic = "/motor/temperature";
  snapshot1.message_type = "sensor_msgs/msg/Temperature";
  snapshot1.data = R"({"temperature": 85.5})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();

  SnapshotData snapshot2;
  snapshot2.fault_code = "MOTOR_OVERHEAT";
  snapshot2.topic = "/motor/rpm";
  snapshot2.message_type = "std_msgs/msg/Float64";
  snapshot2.data = R"({"data": 5500.0})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot1);
  storage_->store_snapshot(snapshot2);

  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");
  EXPECT_EQ(snapshots.size(), 2u);
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, FilterSnapshotsByTopic) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  SnapshotData snapshot1;
  snapshot1.fault_code = "MOTOR_OVERHEAT";
  snapshot1.topic = "/motor/temperature";
  snapshot1.message_type = "sensor_msgs/msg/Temperature";
  snapshot1.data = R"({"temperature": 85.5})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();

  SnapshotData snapshot2;
  snapshot2.fault_code = "MOTOR_OVERHEAT";
  snapshot2.topic = "/motor/rpm";
  snapshot2.message_type = "std_msgs/msg/Float64";
  snapshot2.data = R"({"data": 5500.0})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();

  storage_->store_snapshot(snapshot1);
  storage_->store_snapshot(snapshot2);

  // Filter by topic
  auto filtered = storage_->get_snapshots("MOTOR_OVERHEAT", "/motor/temperature");
  ASSERT_EQ(filtered.size(), 1u);
  EXPECT_EQ(filtered[0].topic, "/motor/temperature");
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, NoSnapshotsForUnknownFault) {
  auto snapshots = storage_->get_snapshots("UNKNOWN_FAULT");
  EXPECT_TRUE(snapshots.empty());
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, ClearFaultDeletesAssociatedSnapshots) {
  using ros2_medkit_fault_manager::SnapshotData;
  rclcpp::Clock clock;

  // Create a fault using report_fault_event
  storage_->report_fault_event("SNAPSHOT_CLEAR_TEST", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Test fault for snapshot cleanup", "/test_node", clock.now(), default_config());

  // Store snapshots for this fault
  SnapshotData snapshot1;
  snapshot1.fault_code = "SNAPSHOT_CLEAR_TEST";
  snapshot1.topic = "/test/topic1";
  snapshot1.message_type = "std_msgs/msg/String";
  snapshot1.data = R"({"data": "test1"})";
  snapshot1.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot1);

  SnapshotData snapshot2;
  snapshot2.fault_code = "SNAPSHOT_CLEAR_TEST";
  snapshot2.topic = "/test/topic2";
  snapshot2.message_type = "std_msgs/msg/String";
  snapshot2.data = R"({"data": "test2"})";
  snapshot2.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot2);

  // Verify snapshots exist
  auto snapshots_before = storage_->get_snapshots("SNAPSHOT_CLEAR_TEST");
  ASSERT_EQ(snapshots_before.size(), 2u);

  // Clear the fault
  bool cleared = storage_->clear_fault("SNAPSHOT_CLEAR_TEST");
  EXPECT_TRUE(cleared);

  // Verify snapshots are deleted
  auto snapshots_after = storage_->get_snapshots("SNAPSHOT_CLEAR_TEST");
  EXPECT_TRUE(snapshots_after.empty());
}

// Freeze-frame storage tests
// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, ClearFaultKeepsSnapshotsWhenEvidenceIsRetained) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("KEEP", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "keep", "/n",
                               clock.now(), default_config());
  storage_->set_retain_snapshots_on_clear(true);
  EXPECT_TRUE(storage_->retains_snapshots_on_clear());

  SnapshotData row;
  row.fault_code = "KEEP";
  row.topic = "/t";
  row.message_type = "std_msgs/msg/Float64";
  row.data = R"({"data": 1.0})";
  row.captured_at_ns = 1000;
  row.capture_id = 1;
  storage_->store_snapshots({row});

  ASSERT_TRUE(storage_->clear_fault("KEEP"));

  // Recordings survive an acknowledgement once a history is configured, so the
  // readings captured beside them have to as well - otherwise the fault is left
  // holding bags whose values are gone, which is worse than losing both.
  EXPECT_EQ(storage_->get_snapshots("KEEP").size(), 1u);
}

TEST_F(PgFaultStorageTest, StoreAndRetrieveFreezeFrame) {
  using ros2_medkit_fault_manager::FreezeFrameData;
  rclcpp::Clock clock;

  storage_->report_fault_event("PLC_PRESSURE_HIGH", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Pressure high", "/plc_node", clock.now(), default_config());

  FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4},"/plc/valve":{"data":true}})";
  frame.captured_at_ns = clock.now().nanoseconds();
  storage_->store_freeze_frame(frame);

  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->fault_code, "PLC_PRESSURE_HIGH");
  EXPECT_EQ(retrieved->data, frame.data);
  EXPECT_EQ(retrieved->captured_at_ns, frame.captured_at_ns);
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, NoFreezeFrameForUnknownFault) {
  auto retrieved = storage_->get_freeze_frame("NEVER_CAPTURED");
  EXPECT_FALSE(retrieved.has_value());
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, FreezeFrameReplacedOnRecapture) {
  using ros2_medkit_fault_manager::FreezeFrameData;

  FreezeFrameData first;
  first.fault_code = "PLC_PRESSURE_HIGH";
  first.data = R"({"/plc/pressure":{"data":8.4}})";
  first.captured_at_ns = 1000;
  storage_->store_freeze_frame(first);

  FreezeFrameData second;
  second.fault_code = "PLC_PRESSURE_HIGH";
  second.data = R"({"/plc/pressure":{"data":9.9}})";
  second.captured_at_ns = 2000;
  storage_->store_freeze_frame(second);

  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, second.data);
  EXPECT_EQ(retrieved->captured_at_ns, 2000);
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, FreezeFrameSurvivesClearFault) {
  using ros2_medkit_fault_manager::FreezeFrameData;
  using ros2_medkit_fault_manager::SnapshotData;
  rclcpp::Clock clock;

  storage_->report_fault_event("PLC_PRESSURE_HIGH", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Pressure high", "/plc_node", clock.now(), default_config());

  // A per-topic snapshot (removed on clear) plus a freeze-frame (retained on clear).
  SnapshotData snapshot;
  snapshot.fault_code = "PLC_PRESSURE_HIGH";
  snapshot.topic = "/plc/pressure";
  snapshot.message_type = "std_msgs/msg/Float64";
  snapshot.data = R"({"data":8.4})";
  snapshot.captured_at_ns = clock.now().nanoseconds();
  storage_->store_snapshot(snapshot);

  FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4}})";
  frame.captured_at_ns = clock.now().nanoseconds();
  storage_->store_freeze_frame(frame);

  ASSERT_TRUE(storage_->clear_fault("PLC_PRESSURE_HIGH"));

  // Snapshots are wiped on clear, the freeze-frame is retained and still retrievable.
  EXPECT_TRUE(storage_->get_snapshots("PLC_PRESSURE_HIGH").empty());
  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, frame.data);
}

// @verifies REQ_INTEROP_088
TEST_F(PgFaultStorageTest, FreezeFramePersistsAcrossReopen) {
  using ros2_medkit_fault_manager::FreezeFrameData;

  FreezeFrameData frame;
  frame.fault_code = "PLC_PRESSURE_HIGH";
  frame.data = R"({"/plc/pressure":{"data":8.4}})";
  frame.captured_at_ns = 4242;
  storage_->store_freeze_frame(frame);

  // Reconnect to the same database.
  storage_.reset();
  storage_ = std::make_unique<PgFaultStorage>(conn_info_);

  auto retrieved = storage_->get_freeze_frame("PLC_PRESSURE_HIGH");
  ASSERT_TRUE(retrieved.has_value());
  EXPECT_EQ(retrieved->data, frame.data);
  EXPECT_EQ(retrieved->captured_at_ns, 4242);
}

// Rosbag entity-scoped listing tests

// @verifies REQ_INTEROP_071
TEST_F(PgFaultStorageTest, ListRosbagsForEntityFiltersCorrectly) {
  using ros2_medkit_fault_manager::RosbagFileInfo;
  rclcpp::Clock clock;

  // Create fault with reporting source for entity
  storage_->report_fault_event("ENTITY_FAULT_1", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Fault from entity", "/powertrain/motor", clock.now(), default_config());

  // Create another fault with different reporting source
  storage_->report_fault_event("ENTITY_FAULT_2", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "Fault from other entity", "/chassis/brake", clock.now(), default_config());

  // Store rosbags for both faults
  RosbagFileInfo info1;
  info1.fault_code = "ENTITY_FAULT_1";
  info1.file_path = "/tmp/entity1.mcap";
  info1.format = "mcap";
  info1.duration_sec = 5.0;
  info1.size_bytes = 1024;
  info1.created_at_ns = clock.now().nanoseconds();
  storage_->store_rosbag_file(info1);

  RosbagFileInfo info2;
  info2.fault_code = "ENTITY_FAULT_2";
  info2.file_path = "/tmp/entity2.mcap";
  info2.format = "mcap";
  info2.duration_sec = 3.0;
  info2.size_bytes = 512;
  info2.created_at_ns = clock.now().nanoseconds();
  storage_->store_rosbag_file(info2);

  // Get rosbags for motor entity
  auto rosbags = storage_->list_rosbags_for_entity("/powertrain/motor");
  ASSERT_EQ(rosbags.size(), 1u);
  EXPECT_EQ(rosbags[0].fault_code, "ENTITY_FAULT_1");

  // Get rosbags for brake entity
  auto brake_rosbags = storage_->list_rosbags_for_entity("/chassis/brake");
  ASSERT_EQ(brake_rosbags.size(), 1u);
  EXPECT_EQ(brake_rosbags[0].fault_code, "ENTITY_FAULT_2");

  // Get rosbags for unknown entity
  auto unknown_rosbags = storage_->list_rosbags_for_entity("/unknown/entity");
  EXPECT_TRUE(unknown_rosbags.empty());
}

TEST_F(PgFaultStorageTest, AFailingRowDeleteKeepsTheBagOnDisk) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  // delete_rosbag_file() has to survive a DELETE that throws - a lock timeout, a
  // full disk, a read-only standby - without having already unlinked the bag. A
  // surviving row whose directory is gone fails every later retrieval and keeps its
  // bytes charged against the quota, which sums rows.
  const auto bag_dir = temp_root_ / "failing_delete_bag";
  std::filesystem::create_directories(bag_dir);
  { std::ofstream(bag_dir / "payload.mcap") << "data"; }

  storage_->store_rosbag_file(make_rosbag("DELETE_FAILS", bag_dir.string(), 1000, 4));

  // Make the DELETE, and only the DELETE, fail. A dropped or renamed table would
  // take the whole statement with it, and the ordering under test is between the
  // row change and the unlink.
  exec_raw(
      "CREATE FUNCTION block_rosbag_delete() RETURNS trigger AS $fn$ "
      "BEGIN RAISE EXCEPTION 'delete blocked'; END; $fn$ LANGUAGE plpgsql");
  exec_raw(
      "CREATE TRIGGER block_rosbag_delete BEFORE DELETE ON rosbag_files "
      "FOR EACH ROW EXECUTE PROCEDURE block_rosbag_delete()");

  EXPECT_THROW(storage_->delete_rosbag_file("DELETE_FAILS"), std::runtime_error);

  exec_raw("DROP TRIGGER block_rosbag_delete ON rosbag_files");

  EXPECT_TRUE(storage_->get_rosbag_file("DELETE_FAILS").has_value()) << "the DELETE failed, so its row must remain";
  EXPECT_TRUE(std::filesystem::exists(bag_dir))
      << "the bag was unlinked before its row was deleted, so the surviving row now points at nothing";
}

// @verifies REQ_INTEROP_073
TEST_F(PgFaultStorageTest, GetAllRosbagFilesReturnsSortedByCreatedAt) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  RosbagFileInfo info1;
  info1.fault_code = "FAULT_A";
  info1.file_path = "/tmp/a.mcap";
  info1.format = "mcap";
  info1.duration_sec = 1.0;
  info1.size_bytes = 100;
  info1.created_at_ns = 1000;
  storage_->store_rosbag_file(info1);

  RosbagFileInfo info2;
  info2.fault_code = "FAULT_B";
  info2.file_path = "/tmp/b.mcap";
  info2.format = "mcap";
  info2.duration_sec = 2.0;
  info2.size_bytes = 200;
  info2.created_at_ns = 2000;
  storage_->store_rosbag_file(info2);

  auto all_rosbags = storage_->get_all_rosbag_files();
  ASSERT_EQ(all_rosbags.size(), 2u);

  // Should be sorted by created_at_ns (oldest first)
  EXPECT_EQ(all_rosbags[0].fault_code, "FAULT_A");
  EXPECT_EQ(all_rosbags[1].fault_code, "FAULT_B");
}

// Shared-recording tests: a burst of correlated faults confirming inside one
// post-roll window all reference the same bag, so the file must outlive every
// record but one, and the quota must not count it once per fault.

TEST_F(PgFaultStorageTest, SharedRosbagSurvivesUntilTheLastFaultIsDeleted) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  auto bag_path = (temp_root_ / "_shared_bag").string();
  std::filesystem::create_directories(bag_path);

  RosbagFileInfo info;
  info.fault_code = "ROOT_CAUSE";
  info.file_path = bag_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 4096;
  info.created_at_ns = 1000;
  storage_->store_rosbag_file(info);

  info.fault_code = "CORRELATED";
  storage_->store_rosbag_file(info);

  EXPECT_TRUE(storage_->delete_rosbag_file("CORRELATED"));
  EXPECT_TRUE(std::filesystem::exists(bag_path));
  EXPECT_TRUE(storage_->get_rosbag_file("ROOT_CAUSE").has_value());

  EXPECT_TRUE(storage_->delete_rosbag_file("ROOT_CAUSE"));
  EXPECT_FALSE(std::filesystem::exists(bag_path));
}

TEST_F(PgFaultStorageTest, SharedRosbagCountsOnceTowardsStorageTotal) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  // Rows of one recording can transiently disagree on size while it is being
  // finalised, so the total takes MAX(size_bytes) per path. Two shared bags
  // with opposite insert orderings, so neither first- nor last-write-wins can
  // fake the MAX (mirrors the in-memory SharedBagTotalTakesTheLargestRowPerPath).
  auto store = [this](const char * code, const std::string & path, size_t bytes) {
    RosbagFileInfo info;
    info.fault_code = code;
    info.file_path = path;
    info.format = "mcap";
    info.duration_sec = 5.0;
    info.size_bytes = bytes;
    info.created_at_ns = 1000;
    storage_->store_rosbag_file(info);
  };

  store("BIG_FIRST", "/tmp/shared_a.mcap", 1000);
  store("SMALL_SECOND", "/tmp/shared_a.mcap", 300);
  store("SMALL_FIRST", "/tmp/shared_b.mcap", 200);
  store("BIG_SECOND", "/tmp/shared_b.mcap", 800);
  store("UNRELATED", "/tmp/other.mcap", 500);

  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 1000u + 800u + 500u);
}

// Re-store guard: a fault re-confirming stores a row with a new path, and the
// old bag must be unlinked only when no sibling fault still references it.

TEST_F(PgFaultStorageTest, RestoreWithNewPathUnlinksTheOldExclusiveBag) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  const auto old_path = (temp_root_ / "_exclusive_bag").string();
  std::filesystem::create_directories(old_path);

  RosbagFileInfo info;
  info.fault_code = "X";
  info.file_path = old_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 100;
  info.created_at_ns = 1000;
  storage_->store_rosbag_file(info);

  info.file_path = old_path + "_new";
  storage_->store_rosbag_file(info);

  EXPECT_FALSE(std::filesystem::exists(old_path)) << "nobody references the old bag, it must be unlinked";
  auto row = storage_->get_rosbag_file("X");
  ASSERT_TRUE(row.has_value());
  EXPECT_EQ(row->file_path, old_path + "_new");
}

TEST_F(PgFaultStorageTest, RestoreWithNewPathKeepsTheBagASiblingStillReferences) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  const auto shared_path = (temp_root_ / "_shared_bag").string();
  std::filesystem::create_directories(shared_path);

  RosbagFileInfo info;
  info.file_path = shared_path;
  info.format = "mcap";
  info.duration_sec = 5.0;
  info.size_bytes = 100;
  info.created_at_ns = 1000;
  info.fault_code = "X";
  storage_->store_rosbag_file(info);
  info.fault_code = "Y";
  storage_->store_rosbag_file(info);

  info.fault_code = "X";
  info.file_path = shared_path + "_new";
  storage_->store_rosbag_file(info);

  EXPECT_TRUE(std::filesystem::exists(shared_path)) << "the sibling fault still owns the shared bag";
  auto sibling = storage_->get_rosbag_file("Y");
  ASSERT_TRUE(sibling.has_value());
  EXPECT_EQ(sibling->file_path, shared_path);
}

// Burst-level batch operations (one PostgreSQL transaction per burst).

TEST_F(PgFaultStorageTest, BulkStoreRegistersEveryFaultOfTheBurst) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  RosbagFileInfo info;
  info.file_path = "/tmp/burst_bag";
  info.format = "mcap";
  info.duration_sec = 6.0;
  info.size_bytes = 4096;
  info.created_at_ns = 1000;

  std::vector<RosbagFileInfo> rows;
  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A", "CORRELATED_B"}) {
    info.fault_code = code;
    rows.push_back(info);
  }
  storage_->store_rosbag_files(rows);

  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A", "CORRELATED_B"}) {
    auto row = storage_->get_rosbag_file(code);
    ASSERT_TRUE(row.has_value()) << code;
    EXPECT_EQ(row->file_path, "/tmp/burst_bag");
  }
  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 4096u);
}

TEST_F(PgFaultStorageTest, BulkDeleteRemovesTheBurstAndUnlinksTheBagOnce) {
  using ros2_medkit_fault_manager::RosbagFileInfo;

  const auto bag_path = (temp_root_ / "_burst_bag").string();
  std::filesystem::create_directories(bag_path);

  RosbagFileInfo info;
  info.file_path = bag_path;
  info.format = "mcap";
  info.duration_sec = 6.0;
  info.size_bytes = 4096;
  info.created_at_ns = 1000;
  std::vector<RosbagFileInfo> rows;
  for (const char * code : {"ROOT_CAUSE", "CORRELATED_A", "CORRELATED_B"}) {
    info.fault_code = code;
    rows.push_back(info);
  }
  storage_->store_rosbag_files(rows);

  // A partial delete leaves the bag on disk for the remaining fault.
  EXPECT_EQ(storage_->delete_rosbag_files({"CORRELATED_A", "CORRELATED_B", "NEVER_STORED"}), 2u);
  EXPECT_TRUE(std::filesystem::exists(bag_path));
  EXPECT_TRUE(storage_->get_rosbag_file("ROOT_CAUSE").has_value());
  EXPECT_FALSE(storage_->get_rosbag_file("CORRELATED_A").has_value());

  // The last reference going away unlinks the directory.
  EXPECT_EQ(storage_->delete_rosbag_files({"ROOT_CAUSE"}), 1u);
  EXPECT_FALSE(std::filesystem::exists(bag_path));
  EXPECT_EQ(storage_->get_total_rosbag_storage_bytes(), 0u);
}

// =============================================================================
// Snapshot limit tests (issue #308)
// =============================================================================

TEST_F(PgFaultStorageTest, SnapshotCapKeepsTheNewestCaptureWholeAndDropsTheOldest) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "Motor overheated", "/motor_node", clock.now(), default_config());

  // Two topics per capture, room for two captures.
  storage_->set_max_snapshots_per_fault(4);

  const auto capture = [](int64_t id, int64_t at) {
    std::vector<SnapshotData> rows;
    for (const char * topic : {"/motor/temp", "/motor/rpm"}) {
      SnapshotData row;
      row.fault_code = "MOTOR_OVERHEAT";
      row.topic = topic;
      row.message_type = "std_msgs/msg/Float64";
      row.data = R"({"data": 1.0})";
      row.captured_at_ns = at;
      row.capture_id = id;
      rows.push_back(row);
    }
    return rows;
  };

  storage_->store_snapshots(capture(1, 1000));
  storage_->store_snapshots(capture(2, 2000));
  storage_->store_snapshots(capture(3, 3000));

  auto snapshots = storage_->get_snapshots("MOTOR_OVERHEAT");

  // The third capture is stored WHOLE and the first goes whole. The old rule
  // counted rows and rejected the new one once full, so capture 3 landed with one
  // topic present and the other silently missing - a freeze frame with a hole in
  // it that reads exactly like "that topic was not publishing".
  ASSERT_EQ(snapshots.size(), 4u);
  std::set<int64_t> captures;
  for (const auto & s : snapshots) {
    captures.insert(s.capture_id);
  }
  EXPECT_EQ(captures, (std::set<int64_t>{2, 3}));
  for (int64_t id : {2, 3}) {
    EXPECT_EQ(std::count_if(snapshots.begin(), snapshots.end(),
                            [id](const SnapshotData & s) {
                              return s.capture_id == id;
                            }),
              2)
        << "capture " << id << " was stored in part";
  }
}

TEST_F(PgFaultStorageTest, ACaptureLargerThanTheCapIsKeptWholeRatherThanTorn) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("WIDE", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "many topics", "/n",
                               clock.now(), default_config());
  storage_->set_max_snapshots_per_fault(2);

  std::vector<SnapshotData> rows;
  for (int i = 0; i < 4; ++i) {
    SnapshotData row;
    row.fault_code = "WIDE";
    row.topic = "/t" + std::to_string(i);
    row.message_type = "std_msgs/msg/Float64";
    row.data = "{}";
    row.captured_at_ns = 1000;
    row.capture_id = 7;
    rows.push_back(row);
  }
  storage_->store_snapshots(rows);

  // The cap is smaller than this fault's topic count. Trimming to it would mean
  // storing the reading with holes, which is the failure being fixed; the capture
  // stays whole and the operator can see the cap is too small.
  EXPECT_EQ(storage_->get_snapshots("WIDE").size(), 4u);
}

TEST_F(PgFaultStorageTest, SnapshotLimitZeroMeansUnlimited) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("FAULT_A", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "desc", "/node",
                               clock.now(), default_config());

  // Default (0) = unlimited
  storage_->set_max_snapshots_per_fault(0);

  for (int i = 0; i < 20; ++i) {
    SnapshotData snap;
    snap.fault_code = "FAULT_A";
    snap.topic = "/topic";
    snap.message_type = "std_msgs/msg/String";
    snap.data = "{}";
    snap.captured_at_ns = i * 1000;
    storage_->store_snapshot(snap);
  }

  auto snapshots = storage_->get_snapshots("FAULT_A");
  EXPECT_EQ(snapshots.size(), 20u) << "Unlimited mode should store all snapshots";
}

TEST_F(PgFaultStorageTest, SnapshotLimitPerFaultNotGlobal) {
  using ros2_medkit_fault_manager::SnapshotData;

  rclcpp::Clock clock;
  storage_->report_fault_event("FAULT_A", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "desc", "/node",
                               clock.now(), default_config());
  storage_->report_fault_event("FAULT_B", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "desc", "/node",
                               clock.now(), default_config());

  storage_->set_max_snapshots_per_fault(1);

  SnapshotData snap_a;
  snap_a.fault_code = "FAULT_A";
  snap_a.topic = "/topic";
  snap_a.message_type = "std_msgs/msg/String";
  snap_a.data = "{}";
  snap_a.captured_at_ns = 1000;

  SnapshotData snap_b = snap_a;
  snap_b.fault_code = "FAULT_B";

  storage_->store_snapshot(snap_a);
  storage_->store_snapshot(snap_b);

  // Both faults should have 1 snapshot each (limit is per-fault)
  EXPECT_EQ(storage_->get_snapshots("FAULT_A").size(), 1u);
  EXPECT_EQ(storage_->get_snapshots("FAULT_B").size(), 1u);
}

// --- Near-miss series ---
//
// A near miss is a FAILED report that moved the debounce counter without the fault ending up
// CONFIRMED. The series is append-only and must survive clear_fault, because acknowledging a
// fault cycle must not erase how often that code approached confirmation.

/// Debounce config that takes four FAILED reports to confirm, leaving three near misses first.
static DebounceConfig four_strike_config() {
  DebounceConfig config;
  config.confirmation_threshold = -4;
  config.critical_immediate_confirm = false;
  return config;
}

/// Deterministic timestamps 1 ms apart, so series ordering is checkable.
static rclcpp::Time nth_report_time(int index) {
  constexpr int64_t kBaseNs = 1700000000000000000LL;
  return rclcpp::Time(kBaseNs + static_cast<int64_t>(index) * 1000000LL);
}

TEST_F(PgFaultStorageTest, NearMissSeriesIsAppendedNotOverwritten) {
  const auto config = four_strike_config();

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto fault = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(fault.has_value());
  ASSERT_NE(fault->status, Fault::STATUS_CONFIRMED) << "test setup: these reports must not confirm";

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "each near miss must append an entry, not overwrite the last";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[1].debounce_counter, -2);
  EXPECT_EQ(series[2].debounce_counter, -3);
  EXPECT_EQ(series[0].confirmation_threshold, -4);
  EXPECT_EQ(series[0].fault_code, "PUMP_PRESSURE_LOW");
  EXPECT_EQ(series[0].source_id, "/hydraulics/pump");
  EXPECT_EQ(series[0].severity, Fault::SEVERITY_WARN);
  EXPECT_EQ(series[0].occurred_at_ns, nth_report_time(0).nanoseconds());
  EXPECT_LT(series[0].occurred_at_ns, series[2].occurred_at_ns);
}

TEST_F(PgFaultStorageTest, ConfirmingReportIsNotANearMiss) {
  const auto config = four_strike_config();

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto fault = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(fault.has_value());
  ASSERT_EQ(fault->status, Fault::STATUS_CONFIRMED);

  // The fourth report is the fault happening, not nearly happening.
  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);
}

TEST_F(PgFaultStorageTest, NearMissSeriesSurvivesClearFault) {
  const auto config = four_strike_config();

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);

  ASSERT_TRUE(storage_->clear_fault("PUMP_PRESSURE_LOW"));

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "acknowledging the fault destroyed the near-miss record";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[2].debounce_counter, -3);
}

TEST_F(PgFaultStorageTest, NearMissSeriesContinuesAcrossReactivation) {
  const auto config = four_strike_config();

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_TRUE(storage_->clear_fault("PUMP_PRESSURE_LOW"));

  // A new outage cycle starts: the reactivating report resets the counter to -1 without confirming.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping again", "/hydraulics/pump", nth_report_time(10), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 4u) << "the series must span fault cycles, one entry per occurrence";
  EXPECT_EQ(series[3].debounce_counter, -1);
  EXPECT_EQ(series[3].occurred_at_ns, nth_report_time(10).nanoseconds());
}

TEST_F(PgFaultStorageTest, NearMissSeriesSurvivesReopen) {
  const auto config = four_strike_config();

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_TRUE(storage_->clear_fault("PUMP_PRESSURE_LOW"));

  storage_.reset();
  storage_ = std::make_unique<PgFaultStorage>(conn_info_);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "the series must outlive the process, not just the fault cycle";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[2].debounce_counter, -3);
}

TEST_F(PgFaultStorageTest, PassedReportIsNotANearMiss) {
  const auto config = four_strike_config();

  for (int i = 0; i < 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);

  // A PASSED report moves the counter in the healing direction: the fault receding, not nearing.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                               "/hydraulics/pump", nth_report_time(2), config);

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);
}

TEST_F(PgFaultStorageTest, CriticalImmediateConfirmIsNotANearMiss) {
  DebounceConfig config = four_strike_config();
  config.critical_immediate_confirm = true;

  storage_->report_fault_event("BATTERY_THERMAL_RUNAWAY", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_CRITICAL,
                               "cell over temperature", "/power/bms", nth_report_time(0), config);

  auto fault = storage_->get_fault("BATTERY_THERMAL_RUNAWAY");
  ASSERT_TRUE(fault.has_value());
  ASSERT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_TRUE(storage_->get_near_misses("BATTERY_THERMAL_RUNAWAY").empty());
}

TEST_F(PgFaultStorageTest, ImmediateConfirmThresholdRecordsNoNearMiss) {
  // Endpoint of the documented range: confirmation_threshold = -1 confirms on the first report,
  // so a fault under this config never has a near miss to record.
  const auto config = default_config();

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "pressure lost", "/hydraulics/pump", nth_report_time(0), config);

  auto fault = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(fault.has_value());
  ASSERT_EQ(fault->status, Fault::STATUS_CONFIRMED);
  EXPECT_TRUE(storage_->get_near_misses("PUMP_PRESSURE_LOW").empty());
}

TEST_F(PgFaultStorageTest, FailedReportUnderHealedLatchIsNearMiss) {
  DebounceConfig config = four_strike_config();
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);

  for (int i = 1; i <= 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                                 "/hydraulics/pump", nth_report_time(i), config);
  }
  auto healed = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";

  // The latch keeps the status at HEALED, but the counter moved toward confirmation: a near miss.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(3), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[1].debounce_counter, 0);
}

TEST_F(PgFaultStorageTest, NearMissSeriesBoundedKeepingNewest) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(3);

  for (int i = 0; i < 5; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u);
  // Oldest-first eviction: a series frozen at boot would say nothing about a trend.
  EXPECT_EQ(series[0].debounce_counter, -3);
  EXPECT_EQ(series[1].debounce_counter, -4);
  EXPECT_EQ(series[2].debounce_counter, -5);
}

TEST_F(PgFaultStorageTest, NearMissBoundOfOneKeepsLatestOnly) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(1);

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 1u);
  EXPECT_EQ(series[0].debounce_counter, -4);
}

TEST_F(PgFaultStorageTest, NearMissBoundIsPerFaultCode) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(2);

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
    storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "temperature rising", "/powertrain/motor", nth_report_time(i), config);
  }

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 2u);
  EXPECT_EQ(storage_->get_near_misses("MOTOR_OVERHEAT").size(), 2u);
}

TEST_F(PgFaultStorageTest, NearMissBoundZeroIsUnlimited) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -200;
  storage_->set_max_near_misses_per_fault(0);

  for (int i = 0; i < 150; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 150u);
}

TEST_F(PgFaultStorageTest, NearMissSeriesSurvivesHealedReclassification) {
  // Startup reclassification is the other place that drops a fault's captured data; it must
  // leave the series alone for the same reason clear_fault does.
  DebounceConfig config = four_strike_config();
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);
  for (int i = 1; i <= 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                                 "/hydraulics/pump", nth_report_time(i), config);
  }
  auto healed = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);

  ASSERT_EQ(storage_->reclassify_healed_as_cleared().size(), 1u);

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 1u);
}

TEST_F(PgFaultStorageTest, NearMissSeriesUsesArrivalOrderNotTimestamps) {
  // Reporters carry their own clocks, so a report can arrive with a timestamp behind one already
  // stored. Ordering eviction by timestamp would delete the row that was just appended.
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(2);

  for (int i = 0; i < 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(10 + i), config);
  }
  // Arrives third, but carries the earliest timestamp of the three.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[0].occurred_at_ns, nth_report_time(11).nanoseconds());
  EXPECT_EQ(series[1].occurred_at_ns, nth_report_time(0).nanoseconds())
      << "the report that just arrived must not be the one evicted";
  EXPECT_EQ(series[1].debounce_counter, -3);
}

TEST_F(PgFaultStorageTest, NearMissEntriesDescribeTheirOwnReport) {
  // Each entry must describe the report that produced it, not the fault's current state, or a
  // series spanning a threshold change or a new reporting source reads as if nothing changed.
  DebounceConfig first = four_strike_config();
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), first);

  DebounceConfig second = four_strike_config();
  second.confirmation_threshold = -8;
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR,
                               "pressure dipping", "/hydraulics/backup_pump", nth_report_time(1), second);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u);
  EXPECT_EQ(series[0].confirmation_threshold, -4);
  EXPECT_EQ(series[0].severity, Fault::SEVERITY_WARN);
  EXPECT_EQ(series[0].source_id, "/hydraulics/pump");
  EXPECT_EQ(series[1].confirmation_threshold, -8);
  EXPECT_EQ(series[1].severity, Fault::SEVERITY_ERROR);
  EXPECT_EQ(series[1].source_id, "/hydraulics/backup_pump");
}

TEST_F(PgFaultStorageTest, NearMissSeriesContinuesAfterReopen) {
  const auto config = four_strike_config();

  for (int i = 0; i < 2; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  storage_.reset();
  storage_ = std::make_unique<PgFaultStorage>(conn_info_);

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(2), config);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 3u) << "a restart must extend the series, not restart or reorder it";
  EXPECT_EQ(series[0].debounce_counter, -1);
  EXPECT_EQ(series[1].debounce_counter, -2);
  EXPECT_EQ(series[2].debounce_counter, -3);
  EXPECT_EQ(series[2].occurred_at_ns, nth_report_time(2).nanoseconds());
}

TEST_F(PgFaultStorageTest, ApplyingSmallerBoundTrimsExistingSeries) {
  // A database that grew under a larger bound, or none, must come back inside the bound as soon
  // as it is applied. Waiting for the next near miss leaves a quiet fault code over the bound for
  // good.
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(0);

  for (int i = 0; i < 5; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
    storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "temperature rising", "/powertrain/motor", nth_report_time(i), config);
  }
  ASSERT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 5u);

  storage_->set_max_near_misses_per_fault(2);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 2u) << "applying the bound left the stored series over it";
  EXPECT_EQ(series[0].debounce_counter, -4);
  EXPECT_EQ(series[1].debounce_counter, -5);
  EXPECT_EQ(storage_->get_near_misses("MOTOR_OVERHEAT").size(), 2u) << "the bound is applied per fault code";
}

TEST_F(PgFaultStorageTest, ApplyingUnlimitedBoundKeepsExistingSeries) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(4);

  for (int i = 0; i < 4; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  storage_->set_max_near_misses_per_fault(0);

  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 4u);
}

TEST_F(PgFaultStorageTest, UnlimitedBoundSpeltAsSizeMaxKeepsTheSeries) {
  // SIZE_MAX is the idiomatic spelling of "no limit". Bound straight into a BIGINT parameter it
  // becomes -1, and every row then compares as beyond the bound, which would empty the table.
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;

  for (int i = 0; i < 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }

  EXPECT_EQ(storage_->set_max_near_misses_per_fault(std::numeric_limits<size_t>::max()), 0u);
  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 3u);

  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(3), config);
  EXPECT_EQ(storage_->get_near_misses("PUMP_PRESSURE_LOW").size(), 4u);
}

TEST_F(PgFaultStorageTest, ApplyingBoundReportsHowManyEntriesItDropped) {
  DebounceConfig config = four_strike_config();
  config.confirmation_threshold = -20;
  storage_->set_max_near_misses_per_fault(0);

  for (int i = 0; i < 5; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
    storage_->report_fault_event("MOTOR_OVERHEAT", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "temperature rising", "/powertrain/motor", nth_report_time(i), config);
  }

  // Two codes, five entries each, bound of 2: three dropped per code.
  EXPECT_EQ(storage_->set_max_near_misses_per_fault(2), 6u);
  // Applying the same bound again has nothing left to drop.
  EXPECT_EQ(storage_->set_max_near_misses_per_fault(2), 0u);
}

TEST_F(PgFaultStorageTest, PassedReportOnUnknownFaultWritesNothing) {
  // A heal heartbeat for a fault that does not exist must stay a read: it writes no row.
  EXPECT_FALSE(storage_->report_fault_event("NEVER_REPORTED", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN,
                                            "", "/test_node", nth_report_time(0), default_config()));
  EXPECT_EQ(storage_->size(), 0u);
  EXPECT_TRUE(storage_->get_near_misses("NEVER_REPORTED").empty());
}

TEST_F(PgFaultStorageTest, NearMissSeparatesApproachFromRampBackIntoAFault) {
  // The HEALED latch holds the status all the way down to confirmation, so every FAILED report on
  // the way back into a fault that DOES confirm looks like an approach. resulting_status is what
  // tells the two apart afterwards.
  DebounceConfig config = four_strike_config();
  config.healing_enabled = true;
  config.healing_threshold = 2;

  // An approach that recedes: counter moves, nothing is latched.
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);
  for (int i = 1; i <= 3; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_WARN, "",
                                 "/hydraulics/pump", nth_report_time(i), config);
  }
  auto healed = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";

  // The ramp back down: the latch keeps reporting HEALED until the fault actually confirms.
  // From the healing threshold (+2) it takes six FAILED reports to reach the confirmation
  // threshold (-4).
  for (int i = 4; i <= 9; ++i) {
    storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                                 "pressure dipping", "/hydraulics/pump", nth_report_time(i), config);
  }
  auto confirmed = storage_->get_fault("PUMP_PRESSURE_LOW");
  ASSERT_TRUE(confirmed.has_value());
  ASSERT_EQ(confirmed->status, Fault::STATUS_CONFIRMED) << "test setup: the ramp must end in a real fault";

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_FALSE(series.empty());
  EXPECT_EQ(series[0].resulting_status, Fault::STATUS_PREFAILED) << "the first report was a genuine approach";

  size_t approaches = 0;
  for (const auto & entry : series) {
    EXPECT_NE(entry.resulting_status, Fault::STATUS_CONFIRMED) << "a confirmed report is not a near miss";
    if (entry.resulting_status == Fault::STATUS_PREFAILED) {
      ++approaches;
    }
  }
  EXPECT_EQ(approaches, 1u) << "the ramp into a real fault must not read as an approach that receded";
}

TEST_F(PgFaultStorageTest, NearMissResultingStatusSurvivesReopen) {
  const auto config = four_strike_config();
  storage_->report_fault_event("PUMP_PRESSURE_LOW", ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_WARN,
                               "pressure dipping", "/hydraulics/pump", nth_report_time(0), config);

  storage_.reset();
  storage_ = std::make_unique<PgFaultStorage>(conn_info_);

  auto series = storage_->get_near_misses("PUMP_PRESSURE_LOW");
  ASSERT_EQ(series.size(), 1u);
  EXPECT_EQ(series[0].resulting_status, Fault::STATUS_PREFAILED);
}

TEST_F(PgFaultStorageTest, NearMissSeriesEmptyForUnknownFault) {
  EXPECT_TRUE(storage_->get_near_misses("NEVER_REPORTED").empty());
}

// --- Snapshot retention through the startup reclassification ---
//
// clear_fault is not the only place that drops a fault's snapshots: reclassifying a HEALED fault
// as CLEARED at startup does it too, and snapshots.retain_on_clear has to reach both.

/// Store @p count snapshots for @p fault_code, one per topic.
static void store_snapshots_for(ros2_medkit_fault_manager::FaultStorage & storage, const std::string & fault_code,
                                int count) {
  for (int i = 0; i < count; ++i) {
    ros2_medkit_fault_manager::SnapshotData snapshot;
    snapshot.fault_code = fault_code;
    snapshot.topic = "/test/topic" + std::to_string(i);
    snapshot.message_type = "std_msgs/msg/String";
    snapshot.data = R"({"data": "value"})";
    snapshot.captured_at_ns = 1000 + i;
    storage.store_snapshot(snapshot);
  }
}

/// Drive @p fault_code to a latched HEALED state.
static void drive_to_healed(ros2_medkit_fault_manager::FaultStorage & storage, const std::string & fault_code) {
  DebounceConfig config;
  config.healing_enabled = true;
  config.healing_threshold = 1;

  storage.report_fault_event(fault_code, ReportFault::Request::EVENT_FAILED, Fault::SEVERITY_ERROR, "fault",
                             "/test_node", nth_report_time(0), config);
  store_snapshots_for(storage, fault_code, 2);
  // Two PASSED reports: the first only lifts the counter to 0, which the CONFIRMED latch holds.
  for (int i = 1; i <= 2; ++i) {
    storage.report_fault_event(fault_code, ReportFault::Request::EVENT_PASSED, Fault::SEVERITY_ERROR, "", "/test_node",
                               nth_report_time(i), config);
  }
}

TEST_F(PgFaultStorageTest, SnapshotsRetainedThroughHealedReclassification) {
  storage_->set_retain_snapshots_on_clear(true);
  drive_to_healed(*storage_, "SNAPSHOT_RETAIN_TEST");

  auto healed = storage_->get_fault("SNAPSHOT_RETAIN_TEST");
  ASSERT_TRUE(healed.has_value());
  ASSERT_EQ(healed->status, Fault::STATUS_HEALED) << "test setup: the fault must be latched HEALED";
  ASSERT_EQ(storage_->get_snapshots("SNAPSHOT_RETAIN_TEST").size(), 2u);

  ASSERT_EQ(storage_->reclassify_healed_as_cleared().size(), 1u);

  EXPECT_EQ(storage_->get_snapshots("SNAPSHOT_RETAIN_TEST").size(), 2u)
      << "the restart reclassification deleted snapshots the configuration asked to keep";
}

TEST_F(PgFaultStorageTest, SnapshotsDroppedByHealedReclassificationByDefault) {
  drive_to_healed(*storage_, "SNAPSHOT_DROP_TEST");
  ASSERT_EQ(storage_->reclassify_healed_as_cleared().size(), 1u);

  EXPECT_TRUE(storage_->get_snapshots("SNAPSHOT_DROP_TEST").empty());
}

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
