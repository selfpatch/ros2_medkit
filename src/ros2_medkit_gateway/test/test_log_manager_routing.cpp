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

// LogManager routing test: links exclusively against gateway_core + GTest.
// No rclcpp/ament dependencies on the link line, proving that the manager
// body lives in the middleware-neutral build layer. The /rosout subscription
// and the rcl_interfaces::msg::Log -> LogEntry conversion are provided by the
// Ros2LogSource adapter; here we exercise the manager against a mock source
// that emits LogEntry directly.

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/log_types.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider_registry.hpp"
#include "ros2_medkit_gateway/core/transports/log_source.hpp"

namespace ros2_medkit_gateway {
namespace {

/// Mock LogSource that records start()/stop() calls and exposes a manual
/// emit() helper so tests can route LogEntry instances through the manager
/// without any middleware dependency.
class MockLogSource : public LogSource {
 public:
  MockLogSource() = default;
  ~MockLogSource() override = default;
  MockLogSource(const MockLogSource &) = delete;
  MockLogSource & operator=(const MockLogSource &) = delete;
  MockLogSource(MockLogSource &&) = delete;
  MockLogSource & operator=(MockLogSource &&) = delete;

  void start(EntryCallback callback) override {
    ++start_calls_;
    callback_ = std::move(callback);
  }

  void stop() override {
    ++stop_calls_;
    callback_ = nullptr;
  }

  /// Test-side helper: emit a LogEntry as if rclcpp had just delivered one.
  /// Returns false if the manager never started the source (in which case
  /// the entry would never have been delivered in the real adapter either).
  bool emit(const LogEntry & entry) {
    if (!callback_) {
      return false;
    }
    callback_(entry);
    return true;
  }

  int start_calls() const {
    return start_calls_;
  }

  int stop_calls() const {
    return stop_calls_;
  }

 private:
  EntryCallback callback_;
  int start_calls_ = 0;
  int stop_calls_ = 0;
};

/// Helper: construct a LogEntry without an id (the manager assigns one).
LogEntry make_entry(const std::string & name, uint8_t level, const std::string & msg) {
  LogEntry e{};
  e.id = 0;
  e.stamp_sec = 1000;
  e.stamp_nanosec = 0;
  e.level = level;
  e.name = name;
  e.msg = msg;
  return e;
}

/// LogProvider that fully owns ingestion (manages_ingestion() == true).
class MockIngestionProvider : public LogProvider {
 public:
  bool manages_ingestion() const override {
    return true;
  }

  std::vector<LogEntry> get_logs(const std::vector<std::string> & /*node_fqns*/, bool /*prefix_match*/,
                                 const std::string & /*min_severity*/, const std::string & /*context_filter*/,
                                 const std::string & /*entity_id*/) override {
    return {};
  }

  LogConfig get_config(const std::string & /*entity_id*/) const override {
    return LogConfig{};
  }

  std::string update_config(const std::string & /*entity_id*/, const std::optional<std::string> & /*severity_filter*/,
                            const std::optional<size_t> & /*max_entries*/) override {
    return "";
  }
};

/// Observer-mode LogProvider: counts on_log_entry() calls and remembers the
/// most recent entry so the routing test can assert the manager forwards.
class MockObserverProvider : public LogProvider {
 public:
  std::vector<LogEntry> get_logs(const std::vector<std::string> & /*node_fqns*/, bool /*prefix_match*/,
                                 const std::string & /*min_severity*/, const std::string & /*context_filter*/,
                                 const std::string & /*entity_id*/) override {
    return {};
  }

  LogConfig get_config(const std::string & /*entity_id*/) const override {
    return LogConfig{};
  }

  std::string update_config(const std::string & /*entity_id*/, const std::optional<std::string> & /*severity_filter*/,
                            const std::optional<size_t> & /*max_entries*/) override {
    return "";
  }

  bool on_log_entry(const LogEntry & entry) override {
    ++calls_;
    last_name_ = entry.name;
    last_msg_ = entry.msg;
    return false;
  }

  int calls() const {
    return calls_;
  }

  const std::string & last_name() const {
    return last_name_;
  }

  const std::string & last_msg() const {
    return last_msg_;
  }

 private:
  int calls_ = 0;
  std::string last_name_;
  std::string last_msg_;
};

/// Trivial LogProviderRegistry adapter: holds a primary provider + a flat
/// observer list. Mirrors what PluginManager exposes through the registry
/// port without pulling in the rclcpp-coupled implementation.
class MockProviderRegistry : public LogProviderRegistry {
 public:
  void set_primary(LogProvider * primary) {
    primary_ = primary;
  }

  void add_observer(LogProvider * observer) {
    observers_.push_back(observer);
  }

  LogProvider * primary_log_provider() const override {
    return primary_;
  }

  std::vector<LogProvider *> log_observers() const override {
    return observers_;
  }

 private:
  LogProvider * primary_ = nullptr;
  std::vector<LogProvider *> observers_;
};

}  // namespace

// ============================================================
// Routing tests
// ============================================================

TEST(LogManagerRouting, SourceEmittedEntryRoutesToBuffer) {
  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/10);
  EXPECT_EQ(source->start_calls(), 1);

  ASSERT_TRUE(source->emit(make_entry("my_node", /*level=*/20, "hello")));

  auto result = manager.get_logs({"/my_node"}, /*prefix_match=*/false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0]["context"]["node"], "my_node");
  EXPECT_EQ((*result)[0]["message"], "hello");
  // Manager assigns ids monotonically starting at 1.
  EXPECT_EQ((*result)[0]["id"], "log_1");
}

TEST(LogManagerRouting, ManagesIngestionTrueSkipsSourceStart) {
  MockIngestionProvider ingestion;
  MockProviderRegistry registry;
  registry.set_primary(&ingestion);

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, &registry, /*max_buffer_size=*/10);

  // Primary LogProvider declares manages_ingestion() -> manager must NOT have
  // started the source (preserving today's "no /rosout subscription created"
  // behaviour for full-ingestion plugins).
  EXPECT_EQ(source->start_calls(), 0);
}

TEST(LogManagerRouting, ManagesIngestionFalseStartsSource) {
  MockObserverProvider observer;
  MockProviderRegistry registry;
  registry.set_primary(&observer);
  registry.add_observer(&observer);

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, &registry, /*max_buffer_size=*/10);

  // Primary LogProvider is observer-mode (default manages_ingestion()=false)
  // so the manager must call start() exactly once.
  EXPECT_EQ(source->start_calls(), 1);
}

TEST(LogManagerRouting, PluginObserverFiresOnEachEntry) {
  MockObserverProvider observer1;
  MockObserverProvider observer2;
  MockProviderRegistry registry;
  registry.add_observer(&observer1);
  registry.add_observer(&observer2);

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, &registry, /*max_buffer_size=*/10);

  ASSERT_TRUE(source->emit(make_entry("alpha", /*level=*/30, "first")));
  ASSERT_TRUE(source->emit(make_entry("beta", /*level=*/40, "second")));

  EXPECT_EQ(observer1.calls(), 2);
  EXPECT_EQ(observer2.calls(), 2);
  EXPECT_EQ(observer1.last_name(), "beta");
  EXPECT_EQ(observer1.last_msg(), "second");
  EXPECT_EQ(observer2.last_name(), "beta");
  EXPECT_EQ(observer2.last_msg(), "second");
}

TEST(LogManagerRouting, RingBufferOverflowEvictsOldest) {
  auto source = std::make_shared<MockLogSource>();
  // Tiny ring buffer to exercise eviction quickly.
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/3);

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(source->emit(make_entry("n", /*level=*/20, "msg" + std::to_string(i))));
  }

  auto result = manager.get_logs({"/n"}, /*prefix_match=*/false, "", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 3u);
  // Manager assigns ids 1..5; oldest 2 must be gone (3, 4, 5 remain).
  EXPECT_EQ((*result)[0]["id"], "log_3");
  EXPECT_EQ((*result)[1]["id"], "log_4");
  EXPECT_EQ((*result)[2]["id"], "log_5");
}

TEST(LogManagerRouting, SeverityFilterApplied) {
  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/10);

  ASSERT_TRUE(source->emit(make_entry("n", /*level=*/10, "debug-msg")));    // debug
  ASSERT_TRUE(source->emit(make_entry("n", /*level=*/20, "info-msg")));     // info
  ASSERT_TRUE(source->emit(make_entry("n", /*level=*/30, "warning-msg")));  // warning
  ASSERT_TRUE(source->emit(make_entry("n", /*level=*/40, "error-msg")));    // error

  // min_severity=warning -> only warning + error returned.
  auto result = manager.get_logs({"/n"}, /*prefix_match=*/false, "warning", "", "");
  ASSERT_TRUE(result.has_value());
  ASSERT_EQ(result->size(), 2u);
  EXPECT_EQ((*result)[0]["severity"], "warning");
  EXPECT_EQ((*result)[1]["severity"], "error");
}

TEST(LogManagerRouting, ShutdownStopsSource) {
  auto source = std::make_shared<MockLogSource>();
  {
    LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/10);
    EXPECT_EQ(source->start_calls(), 1);
    EXPECT_EQ(source->stop_calls(), 0);
  }  // manager destructs here

  // Manager destructor must call source->stop() so the adapter can release
  // its rclcpp subscription deterministically.
  EXPECT_GE(source->stop_calls(), 1);
}

}  // namespace ros2_medkit_gateway
