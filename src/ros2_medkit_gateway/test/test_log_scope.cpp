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

#include <gtest/gtest.h>

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/logs/log_scope.hpp"
#include "ros2_medkit_gateway/core/managers/log_manager.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider_registry.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/sqlite_trigger_store.hpp"
#include "ros2_medkit_gateway/core/transports/log_source.hpp"

using json = nlohmann::json;
using namespace ros2_medkit_gateway;

namespace {

class NullLogSource : public LogSource {
 public:
  void start(EntryCallback /*callback*/) override {
  }
  void stop() override {
  }
};

/// Source that hands each emitted entry to the LogManager, as /rosout does.
class EmittingLogSource : public LogSource {
 public:
  void start(EntryCallback callback) override {
    callback_ = std::move(callback);
  }
  void stop() override {
    callback_ = nullptr;
  }
  void emit(const LogEntry & entry) {
    if (callback_) {
      callback_(entry);
    }
  }

 private:
  EntryCallback callback_;
};

std::set<std::string> messages_of(const json & items) {
  std::set<std::string> out;
  for (const auto & item : items) {
    out.insert(item.value("message", ""));
  }
  return out;
}

}  // namespace

// =============================================================================
// logs::query_entity_logs
//
// Pins the shared entity -> log-source scope used by GET /{entity}/logs and
// the cyclic subscription "logs" sampler: exact sources follow the fault-scope
// rule (external app -> bare id, external component owns its own id), while
// COMPONENT / AREA merge the namespace-prefix query on top.
// =============================================================================

class LogScopeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Manifest component with a namespace hosting one external app and one
    // ROS-bound app.
    Component cell;
    cell.id = "plant_cell";
    cell.name = "Plant Cell";
    cell.namespace_path = "/plant";
    cell.fqn = "/plant";
    cell.area = "cell_area";

    // External component with no apps (protocol bridge asset).
    Component box;
    box.id = "plc-box";
    box.name = "PLC Box";
    box.external = true;

    App ext;
    ext.id = "ext-plc";
    ext.name = "External PLC";
    ext.component_id = "plant_cell";
    ext.external = true;

    App temp;
    temp.id = "temp";
    temp.name = "Temp Sensor";
    temp.component_id = "plant_cell";
    App::RosBinding rb;
    rb.node_name = "temp";
    rb.namespace_pattern = "/plant";
    temp.ros_binding = rb;

    Area area;
    area.id = "cell_area";
    area.name = "Cell Area";
    area.namespace_path = "/plant";

    Function line;
    line.id = "line_func";
    line.name = "Line Function";
    line.hosts = {"plant_cell"};  // Component host, not an app

    cache_.update_all({area}, {cell, box}, {ext, temp}, {line});

    mgr_.add_log_entry("ext-plc", "info", "plc entry", {});
    mgr_.add_log_entry("/plant/node1", "info", "node1 entry", {});
    mgr_.add_log_entry("/plant/temp", "info", "temp entry", {});
    mgr_.add_log_entry("plc-box", "info", "box entry", {});
    mgr_.add_log_entry("/other/node", "info", "other entry", {});
  }

  ThreadSafeEntityCache cache_;
  LogManager mgr_{std::make_shared<NullLogSource>()};
};

// COMPONENT merges the exact query over resolved sources (bare id for the
// external app, FQN for the bound app) with the namespace-prefix query.
// Before the merge, a non-empty resolved set disabled the prefix path and the
// /plant/* node entries silently vanished.
TEST_F(LogScopeTest, ComponentMergesExternalAppAndNamespaceEntries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plant_cell", "/plant", "", "");
  ASSERT_TRUE(result.has_value());
  auto msgs = messages_of(*result);
  EXPECT_TRUE(msgs.count("plc entry"));
  EXPECT_TRUE(msgs.count("node1 entry"));
  EXPECT_TRUE(msgs.count("temp entry"));
  EXPECT_FALSE(msgs.count("other entry"));
}

// The bound app's entry is hit by both the exact and the prefix query - it
// must appear exactly once, and the merged list stays sorted by id.
TEST_F(LogScopeTest, ComponentDedupesEntriesMatchedByBothQueries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plant_cell", "/plant", "", "");
  ASSERT_TRUE(result.has_value());
  int temp_count = 0;
  long long prev = -1;
  for (const auto & item : *result) {
    if (item.value("message", "") == "temp entry") {
      ++temp_count;
    }
    const long long n = std::stoll(item.value("id", "log_0").substr(4));
    EXPECT_GT(n, prev);
    prev = n;
  }
  EXPECT_EQ(temp_count, 1);
}

// An external component owns entries stored under its own entity id even
// with no hosted apps.
TEST_F(LogScopeTest, ExternalComponentOwnsItsOwnIdEntries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plc-box", "", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(messages_of(*result), std::set<std::string>{"box entry"});
}

// An external app has an empty effective_fqn; its entries live under its bare
// entity id. The raw {entity.fqn} query returned nothing for it.
TEST_F(LogScopeTest, ExternalAppResolvesToBareEntityId) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::APP, "ext-plc", "", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(messages_of(*result), std::set<std::string>{"plc entry"});
}

// A ROS-bound app keeps its exact-FQN scope - no namespace bleed.
TEST_F(LogScopeTest, BoundAppKeepsExactFqnScope) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::APP, "temp", "/plant/temp", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(messages_of(*result), std::set<std::string>{"temp entry"});
}

// A function hosting a Component resolves the component's apps (incl. the
// external one) but never runs a namespace query.
TEST_F(LogScopeTest, FunctionFollowsComponentHosts) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::FUNCTION, "line_func", "", "", "");
  ASSERT_TRUE(result.has_value());
  auto msgs = messages_of(*result);
  EXPECT_TRUE(msgs.count("plc entry"));
  EXPECT_TRUE(msgs.count("temp entry"));
  EXPECT_FALSE(msgs.count("node1 entry"));
}

// AREA resolves hosted apps through its components and merges the namespace
// prefix query, mirroring COMPONENT.
TEST_F(LogScopeTest, AreaMergesHostedAppsAndNamespaceEntries) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::AREA, "cell_area", "/plant", "", "");
  ASSERT_TRUE(result.has_value());
  auto msgs = messages_of(*result);
  EXPECT_TRUE(msgs.count("plc entry"));
  EXPECT_TRUE(msgs.count("node1 entry"));
  EXPECT_TRUE(msgs.count("temp entry"));
  EXPECT_FALSE(msgs.count("other entry"));
}

// The resolved_sources out-param carries the exact-match set for x-medkit
// metadata.
TEST_F(LogScopeTest, ResolvedSourcesOutParamExposesExactSet) {
  std::set<std::string> resolved;
  auto result =
      logs::query_entity_logs(mgr_, cache_, SovdEntityType::COMPONENT, "plant_cell", "/plant", "", "", &resolved);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(resolved.count("ext-plc"));
  EXPECT_TRUE(resolved.count("/plant/temp"));
}

// Unknown entity types resolve to nothing.
TEST_F(LogScopeTest, UnknownTypeReturnsEmpty) {
  auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::UNKNOWN, "whatever", "", "", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->empty());
}

// =============================================================================
// logs::entity_lists_log_entry
//
// The trigger-side predicate must agree with the query for every entity type
// and logger-name form: a logger is in scope exactly when query_entity_logs
// lists its entries.
// =============================================================================

TEST_F(LogScopeTest, PredicateAgreesWithQueryForEveryLoggerForm) {
  const std::vector<std::string> loggers = {
      "plant.temp", "plant.temp.child", "plant/temp/child", "plant.node2", "plantx/node",      "plant",
      "temp",       "ext-plc.child",    "plc-box/sub",      "other.node",  "plant_cell/child", "plant_cell"};
  for (const auto & name : loggers) {
    mgr_.add_log_entry(name, "info", "entry of " + name, {});
  }

  const std::vector<std::pair<SovdEntityType, std::string>> entities = {{SovdEntityType::APP, "temp"},
                                                                        {SovdEntityType::APP, "ext-plc"},
                                                                        {SovdEntityType::COMPONENT, "plant_cell"},
                                                                        {SovdEntityType::COMPONENT, "plc-box"},
                                                                        {SovdEntityType::AREA, "cell_area"},
                                                                        {SovdEntityType::FUNCTION, "line_func"}};

  std::vector<std::string> all_loggers = loggers;
  for (const auto & name : {"ext-plc", "plant/node1", "plant/temp", "plc-box", "other/node"}) {
    all_loggers.emplace_back(name);
  }

  for (const auto & [type, id] : entities) {
    auto result = logs::query_entity_logs(mgr_, cache_, type, id, logs::entity_log_fqn(cache_, type, id), "", "");
    ASSERT_TRUE(result.has_value());
    std::set<std::string> listed;
    for (const auto & item : *result) {
      listed.insert(item["context"].value("node", ""));
    }
    for (const auto & name : all_loggers) {
      EXPECT_EQ(logs::entity_lists_log_entry(mgr_, cache_, type, id, name, 20), listed.count(name) == 1)
          << "entity " << id << ", logger " << name;
    }
  }
}

TEST_F(LogScopeTest, PredicateHonoursTheEntitySeverityFilter) {
  ASSERT_EQ(mgr_.update_config("temp", std::string("warning"), std::nullopt), "");
  EXPECT_FALSE(logs::entity_lists_log_entry(mgr_, cache_, SovdEntityType::APP, "temp", "plant/temp", 20));
  EXPECT_TRUE(logs::entity_lists_log_entry(mgr_, cache_, SovdEntityType::APP, "temp", "plant/temp", 30));
  // The filter belongs to the entity: the component still lists info entries.
  EXPECT_TRUE(logs::entity_lists_log_entry(mgr_, cache_, SovdEntityType::COMPONENT, "plant_cell", "plant/temp", 20));
}

TEST_F(LogScopeTest, PredicateNeverUsesTheLastPathSegment) {
  // "other/node/temp" ends in the id of app temp; the app owns only its FQN.
  EXPECT_FALSE(logs::entity_lists_log_entry(mgr_, cache_, SovdEntityType::APP, "temp", "other/node/temp", 20));
  EXPECT_FALSE(logs::entity_lists_log_entry(mgr_, cache_, SovdEntityType::APP, "temp", "temp", 20));
}

TEST_F(LogScopeTest, ChangePredicateReadsLoggerAndSeverityFromTheEntry) {
  const json entry = {{"id", "log_1"}, {"severity", "info"}, {"context", {{"node", "plant.temp"}}}};
  EXPECT_TRUE(logs::entity_lists_log_change(mgr_, cache_, SovdEntityType::APP, "temp", entry));
  EXPECT_FALSE(logs::entity_lists_log_change(mgr_, cache_, SovdEntityType::APP, "ext-plc", entry));
  // Values a plugin may notify under "logs" without an entry's shape.
  for (const json & odd : {json(42), json::object(), json{{"context", "plant.temp"}},
                           json{{"context", {{"node", 7}}}, {"severity", "info"}}}) {
    EXPECT_FALSE(logs::entity_lists_log_change(mgr_, cache_, SovdEntityType::APP, "temp", odd)) << odd.dump();
  }
}

// =============================================================================
// logs::LoggerOwnerIndex
// =============================================================================

TEST_F(LogScopeTest, OwnerIsTheOneAppWhoseListingHoldsTheLogger) {
  const std::vector<std::string> loggers = {
      "plant.temp",    "plant/temp", "plant.temp.child", "temp",       "ext-plc",
      "ext-plc.child", "plant",      "plc-box",          "other/node", "plant/node1"};
  for (const auto & name : loggers) {
    mgr_.add_log_entry(name, "info", "entry of " + name, {});
  }
  logs::LoggerOwnerIndex index;
  for (const auto & name : loggers) {
    std::vector<std::string> listing_apps;
    for (const auto & app : cache_.get_apps()) {
      auto result = logs::query_entity_logs(mgr_, cache_, SovdEntityType::APP, app.id,
                                            logs::entity_log_fqn(cache_, SovdEntityType::APP, app.id), "", "");
      ASSERT_TRUE(result.has_value());
      for (const auto & item : *result) {
        if (item["context"].value("node", "") == name) {
          listing_apps.push_back(app.id);
          break;
        }
      }
    }
    const std::string expected = listing_apps.size() == 1 ? listing_apps.front() : "";
    EXPECT_EQ(index.find_owner(cache_, name), expected) << "logger " << name;
  }
}

TEST_F(LogScopeTest, OwnerIsEmptyWhenTwoAppsHoldTheLogger) {
  logs::LoggerOwnerIndex index;
  ASSERT_EQ(index.find_owner(cache_, "plant.temp"), "temp");

  // A second app bound to the same node, published by a later discovery pass.
  auto apps = cache_.get_apps();
  App twin = apps.front().id == "temp" ? apps.front() : apps.back();
  twin.id = "temp-twin";
  apps.push_back(twin);
  cache_.update_all(cache_.get_areas(), cache_.get_components(), apps, cache_.get_functions());

  EXPECT_EQ(index.find_owner(cache_, "plant.temp"), "");
  EXPECT_EQ(index.find_owner(cache_, "plant/temp"), "");
  EXPECT_EQ(index.find_owner(cache_, "ext-plc"), "ext-plc");
}

// =============================================================================
// Exact levels of changes waiting for dispatch
// =============================================================================

// The notifier's worker drains its whole queue as one batch and the queue
// refills while the batch is dispatched, so a batch and a full queue can wait
// at once. Every one of them must still be judged on its exact level.
TEST_F(LogScopeTest, LevelsOfEveryWaitingChangeReachTheTriggerDecision) {
  const std::size_t queue_size = ResourceChangeNotifier::kDefaultMaxQueueSize;
  auto source = std::make_shared<EmittingLogSource>();
  LogManager mgr(source, nullptr, /*max_buffer_size=*/2 * queue_size + 1);
  ASSERT_EQ(mgr.update_config("temp", std::string("info"), 4 * queue_size), "");

  // The first change of each of the first two batches blocks the worker until
  // the test releases it.
  std::mutex mutex;
  std::condition_variable cv;
  int calls = 0;
  int entered = 0;
  int released = 0;
  std::set<std::string> admitted;
  std::size_t decided = 0;
  ResourceChangeNotifier notifier;
  mgr.set_notifier(&notifier);
  // Unblocks the worker on every exit path, before the notifier joins it.
  // Not an aggregate under C++20: the deleted copy and move make the constructor necessary.
  struct Unblock {
    Unblock(std::mutex & mutex_ref, std::condition_variable & cv_ref, int & released_ref)
      : m(mutex_ref), c(cv_ref), r(released_ref) {
    }
    std::mutex & m;
    std::condition_variable & c;
    int & r;
    ~Unblock() {
      std::lock_guard<std::mutex> lock(m);
      r = 2;
      c.notify_all();
    }
    Unblock(const Unblock &) = delete;
    Unblock & operator=(const Unblock &) = delete;
    Unblock(Unblock &&) = delete;
    Unblock & operator=(Unblock &&) = delete;
  } unblock{mutex, cv, released};
  notifier.subscribe(NotifierFilter{"logs", "", ""}, [&](const ResourceChange & change) {
    std::unique_lock<std::mutex> lock(mutex);
    const int call = calls++;
    if (call < 2) {
      entered = call + 1;
      cv.notify_all();
      cv.wait(lock, [&] {
        return released > call;
      });
    }
    lock.unlock();
    const bool lists = logs::entity_lists_log_change(mgr, cache_, SovdEntityType::APP, "temp", change.value);
    lock.lock();
    if (lists) {
      admitted.insert(change.value.value("id", ""));
    }
    ++decided;
    cv.notify_all();
  });

  std::size_t emitted = 0;
  auto emit = [&]() {
    LogEntry entry{};
    entry.stamp_sec = 1;
    // 25 and 15 have no severity name; GET under "info" lists only 25.
    entry.level = emitted % 2 == 0 ? 25 : 15;
    entry.name = "plant.temp";
    entry.msg = "m" + std::to_string(emitted);
    source->emit(entry);
    ++emitted;
  };
  auto wait_for = [&](const std::function<bool()> & done) {
    std::unique_lock<std::mutex> lock(mutex);
    return cv.wait_for(lock, std::chrono::seconds(120), done);
  };
  auto release = [&](int n) {
    std::lock_guard<std::mutex> lock(mutex);
    released = n;
    cv.notify_all();
  };

  emit();
  ASSERT_TRUE(wait_for([&] {
    return entered == 1;
  }));
  for (std::size_t i = 0; i < queue_size; ++i) {
    emit();
  }
  release(1);
  ASSERT_TRUE(wait_for([&] {
    return entered == 2;
  }));
  for (std::size_t i = 0; i < queue_size; ++i) {
    emit();
  }
  release(2);
  ASSERT_TRUE(wait_for([&] {
    return decided == emitted;
  })) << "decided "
      << decided << " of " << emitted;
  notifier.shutdown();

  auto listed = logs::query_entity_logs(mgr, cache_, SovdEntityType::APP, "temp",
                                        logs::entity_log_fqn(cache_, SovdEntityType::APP, "temp"), "", "");
  ASSERT_TRUE(listed.has_value());
  std::set<std::string> listed_ids;
  for (const auto & item : *listed) {
    listed_ids.insert(item.value("id", ""));
  }
  EXPECT_EQ(listed_ids.size(), (emitted + 1) / 2);
  EXPECT_EQ(admitted.size(), listed_ids.size());
  EXPECT_TRUE(admitted == listed_ids) << "trigger decisions differ from the GET listing";
}

// =============================================================================
// logs::restore_replay_entries
// =============================================================================

namespace {

/// Primary LogProvider whose first get_logs call throws.
class FailOnceLogProvider : public LogProvider {
 public:
  explicit FailOnceLogProvider(std::vector<LogEntry> entries) : entries_(std::move(entries)) {
  }

  std::vector<LogEntry> get_logs(const std::vector<std::string> & /*node_fqns*/, bool /*prefix_match*/,
                                 const std::string & /*min_severity*/, const std::string & /*context_filter*/,
                                 const std::string & /*entity_id*/) override {
    if (calls_++ == 0) {
      throw std::runtime_error("log backend unavailable");
    }
    return entries_;
  }

  LogConfig get_config(const std::string & /*entity_id*/) const override {
    return LogConfig{};
  }

  std::string update_config(const std::string & /*entity_id*/, const std::optional<std::string> & /*severity_filter*/,
                            const std::optional<size_t> & /*max_entries*/) override {
    return "";
  }

  int calls() const {
    return calls_;
  }

 private:
  std::vector<LogEntry> entries_;
  int calls_{0};
};

class SingleProviderRegistry : public LogProviderRegistry {
 public:
  explicit SingleProviderRegistry(LogProvider * provider) : provider_(provider) {
  }
  LogProvider * primary_log_provider() const override {
    return provider_;
  }
  std::vector<LogProvider *> log_observers() const override {
    return {provider_};
  }

 private:
  LogProvider * provider_;
};

LogEntry provider_entry(int64_t id) {
  LogEntry entry{};
  entry.id = id;
  entry.stamp_sec = 1;
  entry.level = 20;
  entry.name = "plant.temp";
  entry.msg = "provider " + std::to_string(id);
  return entry;
}

}  // namespace

// The fixture's add_log_entry() entries are trigger markers: listed, not replayed.
TEST_F(LogScopeTest, ReplayListingWaitsForAnEntityOfItsTypeAndLeavesOutMarkers) {
  LogEntry real{};
  real.id = 1000;
  real.stamp_sec = 1;
  real.level = 20;
  real.name = "plant/temp";
  real.msg = "real temp entry";
  mgr_.inject_entry_for_testing(real);

  EXPECT_FALSE(logs::restore_replay_entries(mgr_, cache_, SovdEntityType::APP, "plant_cell").has_value())
      << "a Component with the id is not the App";
  auto entries = logs::restore_replay_entries(mgr_, cache_, SovdEntityType::APP, "temp");
  ASSERT_TRUE(entries.has_value());
  ASSERT_EQ(entries->size(), 1u);
  EXPECT_EQ((*entries)[0]["message"], "real temp entry");
  auto listed = logs::query_entity_logs(mgr_, cache_, SovdEntityType::APP, "temp", "/plant/temp", "", "");
  ASSERT_TRUE(listed.has_value());
  EXPECT_EQ(listed->size(), 2u) << "GET lists the marker too";
}

// A failed listing leaves the replay pending; the next sweep delivers each
// entry once.
TEST_F(LogScopeTest, FailedReplayListingIsRetried) {
  FailOnceLogProvider provider({provider_entry(1), provider_entry(2)});
  SingleProviderRegistry registry(&provider);
  LogManager mgr(std::make_shared<NullLogSource>(), &registry);

  TriggerInfo info;
  info.id = "trig_retry";
  info.entity_id = "temp";
  info.entity_type = "apps";
  info.resource_uri = "/api/v1/apps/temp/logs";
  info.collection = "logs";
  info.condition_type = "OnChange";
  info.condition_params = json::object();
  info.protocol = "sse";
  info.multishot = true;
  info.persistent = true;
  info.status = TriggerStatus::ACTIVE;
  info.created_at = std::chrono::system_clock::now();
  SqliteTriggerStore store(":memory:");
  ASSERT_TRUE(store.save(info).has_value());

  ResourceChangeNotifier notifier;
  ConditionRegistry conditions;
  conditions.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
  TriggerConfig config;
  config.max_triggers = 10;
  config.on_restart_behavior = "restore";
  TriggerManager triggers(notifier, conditions, store, config);
  triggers.set_restore_replay_fn("logs", [&](const std::string & /*entity_type*/, const std::string & entity_id) {
    return logs::restore_replay_entries(mgr, cache_, SovdEntityType::APP, entity_id);
  });
  triggers.set_entity_exists_fn([&](const std::string & entity_id, const std::string & /*entity_type*/) {
    return cache_.has_app(entity_id);
  });
  triggers.load_persistent_triggers();

  auto drain = [&] {
    std::vector<std::string> ids;
    while (auto event = triggers.consume_pending_event("trig_retry")) {
      ids.push_back((*event)["payload"].value("id", ""));
    }
    return ids;
  };
  triggers.sweep_orphaned_triggers();
  EXPECT_TRUE(drain().empty()) << "a failed listing delivered events";
  triggers.sweep_orphaned_triggers();
  EXPECT_EQ(drain(), (std::vector<std::string>{"log_1", "log_2"}));
  triggers.sweep_orphaned_triggers();
  EXPECT_TRUE(drain().empty()) << "replayed twice";
  EXPECT_EQ(provider.calls(), 2);

  triggers.shutdown();
  notifier.shutdown();
}
