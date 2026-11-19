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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/log_types.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider_registry.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
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

/// LogProvider that throws std::runtime_error from every provider method.
/// Used to exercise the LogManager's exception handlers and verify that
/// diagnostics flow through the injected log_sink (not std::cerr).
class ThrowingLogProvider : public LogProvider {
 public:
  bool manages_ingestion() const override {
    return false;
  }

  std::vector<LogEntry> get_logs(const std::vector<std::string> & /*node_fqns*/, bool /*prefix_match*/,
                                 const std::string & /*min_severity*/, const std::string & /*context_filter*/,
                                 const std::string & /*entity_id*/) override {
    throw std::runtime_error("boom-get_logs");
  }

  LogConfig get_config(const std::string & /*entity_id*/) const override {
    throw std::runtime_error("boom-get_config");
  }

  std::string update_config(const std::string & /*entity_id*/, const std::optional<std::string> & /*severity_filter*/,
                            const std::optional<size_t> & /*max_entries*/) override {
    throw std::runtime_error("boom-update_config");
  }

  bool on_log_entry(const LogEntry & /*entry*/) override {
    throw std::runtime_error("boom-on_log_entry");
  }
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

// ============================================================
// log_sink callback tests (provider exceptions -> observability sink)
// ============================================================

TEST(LogManagerRoutingTest, NewNodeBeyondCapLogsOnceAndIncrementsCounter) {
  // Buffer cap = max_buffer_size_ * 10. With max=2, cap=20 distinct nodes.
  std::vector<std::pair<int, std::string>> sink_calls;
  auto sink = [&sink_calls](int level, std::string_view msg) {
    sink_calls.emplace_back(level, std::string(msg));
  };

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/2, sink);

  // Fill exactly to the distinct-node cap (20).
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(source->emit(make_entry("node_" + std::to_string(i), /*level=*/20, "in-cap")));
  }
  // dropped_entries_count() must still be zero - we are at the cap, not over.
  EXPECT_EQ(manager.dropped_entries_count(), 0u);
  EXPECT_TRUE(sink_calls.empty());

  // Push two entries from a brand-new node beyond the cap -> first emits a
  // WARN, second only increments the counter.
  ASSERT_TRUE(source->emit(make_entry("over_cap_node", /*level=*/20, "msg-1")));
  ASSERT_TRUE(source->emit(make_entry("over_cap_node", /*level=*/20, "msg-2")));

  EXPECT_EQ(manager.dropped_entries_count(), 2u);
  ASSERT_EQ(sink_calls.size(), 1u);
  EXPECT_EQ(sink_calls[0].first, LogManager::kLogLevelWarn);
  EXPECT_NE(sink_calls[0].second.find("over_cap_node"), std::string::npos);
  EXPECT_NE(sink_calls[0].second.find("buffer cap reached"), std::string::npos);

  // A second new-node-beyond-cap WARNs once for that node too.
  ASSERT_TRUE(source->emit(make_entry("another_over_cap_node", /*level=*/20, "msg-x")));
  EXPECT_EQ(manager.dropped_entries_count(), 3u);
  ASSERT_EQ(sink_calls.size(), 2u);
  EXPECT_NE(sink_calls[1].second.find("another_over_cap_node"), std::string::npos);
}

TEST(LogManagerRouting, ProviderGetLogsExceptionRoutedToLogSink) {
  ThrowingLogProvider throwing_provider;
  MockProviderRegistry registry;
  registry.set_primary(&throwing_provider);

  std::vector<std::pair<int, std::string>> sink_calls;
  auto sink = [&sink_calls](int level, std::string_view msg) {
    sink_calls.emplace_back(level, std::string(msg));
  };

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, &registry, /*max_buffer_size=*/10, sink);

  auto result = manager.get_logs({"/my_node"}, /*prefix_match=*/false, "", "", "");
  EXPECT_FALSE(result.has_value());

  ASSERT_FALSE(sink_calls.empty());
  EXPECT_EQ(sink_calls[0].first, LogManager::kLogLevelError);
  EXPECT_NE(sink_calls[0].second.find("LogProvider::get_logs threw"), std::string::npos);
  EXPECT_NE(sink_calls[0].second.find("boom-get_logs"), std::string::npos);
}

TEST(LogManagerRouting, ProviderGetConfigExceptionRoutedToLogSink) {
  ThrowingLogProvider throwing_provider;
  MockProviderRegistry registry;
  registry.set_primary(&throwing_provider);

  std::vector<std::pair<int, std::string>> sink_calls;
  auto sink = [&sink_calls](int level, std::string_view msg) {
    sink_calls.emplace_back(level, std::string(msg));
  };

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, &registry, /*max_buffer_size=*/10, sink);

  auto result = manager.get_config("entity1");
  EXPECT_FALSE(result.has_value());

  ASSERT_FALSE(sink_calls.empty());
  EXPECT_EQ(sink_calls[0].first, LogManager::kLogLevelError);
  EXPECT_NE(sink_calls[0].second.find("LogProvider::get_config threw"), std::string::npos);
}

TEST(LogManagerRouting, ProviderUpdateConfigExceptionRoutedToLogSink) {
  ThrowingLogProvider throwing_provider;
  MockProviderRegistry registry;
  registry.set_primary(&throwing_provider);

  std::vector<std::pair<int, std::string>> sink_calls;
  auto sink = [&sink_calls](int level, std::string_view msg) {
    sink_calls.emplace_back(level, std::string(msg));
  };

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, &registry, /*max_buffer_size=*/10, sink);

  std::string err = manager.update_config("entity1", std::optional<std::string>("warning"), std::nullopt);
  EXPECT_NE(err.find("LogProvider plugin error"), std::string::npos);

  ASSERT_FALSE(sink_calls.empty());
  EXPECT_EQ(sink_calls[0].first, LogManager::kLogLevelError);
  EXPECT_NE(sink_calls[0].second.find("LogProvider::update_config threw"), std::string::npos);
}

TEST(LogManagerRouting, ProviderOnLogEntryExceptionRoutedToLogSinkAsWarn) {
  ThrowingLogProvider throwing_provider;
  MockProviderRegistry registry;
  registry.add_observer(&throwing_provider);

  std::vector<std::pair<int, std::string>> sink_calls;
  auto sink = [&sink_calls](int level, std::string_view msg) {
    sink_calls.emplace_back(level, std::string(msg));
  };

  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, &registry, /*max_buffer_size=*/10, sink);

  ASSERT_TRUE(source->emit(make_entry("my_node", /*level=*/20, "hi")));

  ASSERT_FALSE(sink_calls.empty());
  EXPECT_EQ(sink_calls[0].first, LogManager::kLogLevelWarn);
  EXPECT_NE(sink_calls[0].second.find("LogProvider::on_log_entry threw"), std::string::npos);
  EXPECT_NE(sink_calls[0].second.find("boom-on_log_entry"), std::string::npos);
}

TEST(LogManagerRouting, NullLogSinkIsSafe) {
  ThrowingLogProvider throwing_provider;
  MockProviderRegistry registry;
  registry.set_primary(&throwing_provider);
  registry.add_observer(&throwing_provider);

  auto source = std::make_shared<MockLogSource>();
  // No sink passed - falls back to the default nullptr, which must be a no-op
  // rather than a crash. This preserves call sites that don't yet wire a sink.
  LogManager manager(source, &registry, /*max_buffer_size=*/10);

  // Trigger all exception paths; expect no crash and well-formed error results.
  auto get_result = manager.get_logs({"/my_node"}, /*prefix_match=*/false, "", "", "");
  EXPECT_FALSE(get_result.has_value());

  auto cfg_result = manager.get_config("entity1");
  EXPECT_FALSE(cfg_result.has_value());

  std::string err = manager.update_config("entity1", std::optional<std::string>("warning"), std::nullopt);
  EXPECT_NE(err.find("LogProvider plugin error"), std::string::npos);

  ASSERT_TRUE(source->emit(make_entry("my_node", /*level=*/20, "hi")));
}

// ============================================================
// Trigger notifications
// ============================================================

/// Records (entity_id, logger name) of every "logs" change, in delivery order.
class LogChangeRecorder {
 public:
  using Change = std::pair<std::string, std::string>;

  explicit LogChangeRecorder(ResourceChangeNotifier & notifier) {
    notifier.subscribe(NotifierFilter{"logs", "", ""}, [this](const ResourceChange & change) {
      std::lock_guard<std::mutex> lock(mutex_);
      changes_.emplace_back(change.entity_id, change.value["context"].value("node", ""));
      cv_.notify_all();
    });
  }

  /// Wait until `logger` was recorded `count` times; return everything
  /// recorded so far.
  std::vector<Change> wait_for(const std::string & logger, std::ptrdiff_t count = 1) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::seconds(5), [&] {
      return std::count_if(changes_.begin(), changes_.end(), [&](const Change & c) {
               return c.second == logger;
             }) >= count;
    });
    return changes_;
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<Change> changes_;
};

TEST(LogManagerRouting, ChangeCarriesTheOwnerAndTheLoggerName) {
  ResourceChangeNotifier notifier;
  LogChangeRecorder recorder(notifier);
  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/10);
  manager.set_logger_owner_fn([](const std::string & logger) {
    return logger == "powertrain.engine.temp_sensor" ? std::string("engine-temp") : std::string();
  });
  manager.set_notifier(&notifier);

  ASSERT_TRUE(source->emit(make_entry("powertrain.engine.temp_sensor", /*level=*/20, "owned")));
  ASSERT_TRUE(source->emit(make_entry("powertrain/engine", /*level=*/20, "unowned")));
  manager.add_log_entry("powertrain.engine.temp_sensor", "warning", "programmatic", json::object());

  auto seen = recorder.wait_for("powertrain.engine.temp_sensor", 2);
  EXPECT_EQ(seen, (std::vector<LogChangeRecorder::Change>{{"engine-temp", "powertrain.engine.temp_sensor"},
                                                          {"", "powertrain/engine"},
                                                          {"engine-temp", "powertrain.engine.temp_sensor"}}));
  notifier.shutdown();
}

TEST(LogManagerRouting, ChangeNamesNoEntityWithoutAnOwnerFunction) {
  ResourceChangeNotifier notifier;
  LogChangeRecorder recorder(notifier);
  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/10);
  manager.set_notifier(&notifier);

  ASSERT_TRUE(source->emit(make_entry("temp_sensor", /*level=*/20, "root")));

  auto seen = recorder.wait_for("temp_sensor");
  EXPECT_EQ(seen, (std::vector<LogChangeRecorder::Change>{{"", "temp_sensor"}}));
  notifier.shutdown();
}

TEST(LogManagerRouting, EntryLevelIsExactForLevelsWithoutASeverityName) {
  ResourceChangeNotifier notifier;
  std::mutex mutex;
  std::vector<json> values;
  notifier.subscribe(NotifierFilter{"logs", "", ""}, [&](const ResourceChange & change) {
    std::lock_guard<std::mutex> lock(mutex);
    values.push_back(change.value);
  });
  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/10);
  manager.set_notifier(&notifier);

  const std::vector<uint8_t> levels = {25, 0, 99, 9, 20, 50, 255};
  for (auto level : levels) {
    ASSERT_TRUE(source->emit(make_entry("node", level, "level " + std::to_string(level))));
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (values.size() == levels.size()) {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  notifier.shutdown();
  ASSERT_EQ(values.size(), levels.size());
  for (std::size_t i = 0; i < levels.size(); ++i) {
    EXPECT_EQ(manager.entry_level(values[i]), levels[i]) << values[i].dump();
  }
}

TEST(LogManagerRouting, EntryLevelFallsBackToTheSeverityNamePastWhatTheNotifierHolds) {
  ResourceChangeNotifier notifier;
  notifier.set_max_queue_size(5);
  ASSERT_EQ(notifier.max_in_flight(), 10u);
  auto source = std::make_shared<MockLogSource>();
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/10);
  manager.set_notifier(&notifier);

  for (std::size_t i = 0; i <= notifier.max_in_flight(); ++i) {
    ASSERT_TRUE(source->emit(make_entry("node", /*level=*/25, "unnamed")));
  }
  notifier.shutdown();
  // Ids start at 1: the first entry is past what the notifier holds, the second is not.
  EXPECT_EQ(manager.entry_level(json{{"id", "log_1"}, {"severity", "debug"}}), 10);
  EXPECT_EQ(manager.entry_level(json{{"id", "log_2"}, {"severity", "debug"}}), 25);
  // Not an entry id: the severity name decides.
  EXPECT_EQ(manager.entry_level(json{{"id", "log_2x"}, {"severity", "error"}}), 40);
  EXPECT_EQ(manager.entry_level(json{{"severity", "fatal"}}), 50);
  EXPECT_EQ(manager.entry_level(json(7)), 0);
}

TEST(LogManagerRouting, EntryDroppedByTheCapEmitsNoChange) {
  ResourceChangeNotifier notifier;
  LogChangeRecorder recorder(notifier);
  auto source = std::make_shared<MockLogSource>();
  // Cap = 1 x 10 distinct logger names.
  LogManager manager(source, /*plugin_mgr=*/nullptr, /*max_buffer_size=*/1);
  manager.set_notifier(&notifier);

  for (int i = 0; i < 10; ++i) {
    ASSERT_TRUE(source->emit(make_entry("node_" + std::to_string(i), /*level=*/20, "in-cap")));
  }
  ASSERT_TRUE(source->emit(make_entry("over_cap_node", /*level=*/20, "dropped")));
  ASSERT_TRUE(source->emit(make_entry("node_0", /*level=*/20, "kept")));
  ASSERT_EQ(manager.dropped_entries_count(), 1u);

  // The worker delivers in order, so node_0's second change comes after
  // anything the dropped entry could have emitted.
  auto seen = recorder.wait_for("node_0", 2);
  auto count = [&](const std::string & logger) {
    return std::count_if(seen.begin(), seen.end(), [&](const LogChangeRecorder::Change & c) {
      return c.second == logger;
    });
  };
  EXPECT_EQ(count("node_0"), 2);
  EXPECT_EQ(count("over_cap_node"), 0);
  notifier.shutdown();
}

}  // namespace ros2_medkit_gateway
