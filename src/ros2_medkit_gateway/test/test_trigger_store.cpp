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

#include <algorithm>
#include <cstdio>
#include <string>

#include "ros2_medkit_gateway/sqlite_trigger_store.hpp"

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

// ---------------------------------------------------------------------------
// Helper: build a fully-populated TriggerInfo
// ---------------------------------------------------------------------------

static TriggerInfo make_trigger(const std::string & id, const std::string & entity = "temp_sensor",
                                const std::string & condition = "OnChange") {
  TriggerInfo t;
  t.id = id;
  t.entity_id = entity;
  t.entity_type = "apps";
  t.resource_uri = "/api/v1/apps/" + entity + "/data/temperature";
  t.collection = "data";
  t.resource_path = "temperature";
  t.path = "/value";
  t.condition_type = condition;
  t.condition_params = json{{"threshold", 42}};
  t.protocol = "sse";
  t.multishot = true;
  t.persistent = false;
  t.lifetime_sec = 3600;
  t.log_settings = json{{"level", "info"}};
  t.status = TriggerStatus::ACTIVE;
  t.created_at = std::chrono::system_clock::now();
  t.expires_at = t.created_at + std::chrono::seconds(3600);
  return t;
}

// ===========================================================================
// Empty database
// ===========================================================================

TEST(TriggerStore, LoadAllEmptyDb) {
  SqliteTriggerStore store(":memory:");
  auto result = store.load_all();
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(result->empty());
}

// ===========================================================================
// save + load_all round-trip
// ===========================================================================

// @verifies REQ_INTEROP_029
TEST(TriggerStore, SaveAndLoadRoundTrip) {
  SqliteTriggerStore store(":memory:");

  auto t1 = make_trigger("trig_001", "sensor_a");
  auto t2 = make_trigger("trig_002", "sensor_b", "LeaveRange");
  auto t3 = make_trigger("trig_003", "sensor_c", "x-custom");

  ASSERT_TRUE(store.save(t1).has_value());
  ASSERT_TRUE(store.save(t2).has_value());
  ASSERT_TRUE(store.save(t3).has_value());

  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 3u);

  // Find trig_002 and verify deep fields
  auto it = std::find_if(loaded->begin(), loaded->end(), [](const TriggerInfo & t) {
    return t.id == "trig_002";
  });
  ASSERT_NE(it, loaded->end());
  EXPECT_EQ(it->entity_id, "sensor_b");
  EXPECT_EQ(it->entity_type, "apps");
  EXPECT_EQ(it->collection, "data");
  EXPECT_EQ(it->resource_path, "temperature");
  EXPECT_EQ(it->path, "/value");
  EXPECT_EQ(it->condition_type, "LeaveRange");
  EXPECT_EQ(it->condition_params, json({{"threshold", 42}}));
  EXPECT_EQ(it->protocol, "sse");
  EXPECT_TRUE(it->multishot);
  EXPECT_FALSE(it->persistent);
  EXPECT_EQ(it->status, TriggerStatus::ACTIVE);
}

// ===========================================================================
// All TriggerInfo fields survive round-trip (including optionals)
// ===========================================================================

TEST(TriggerStore, AllFieldsSurviveRoundTrip) {
  SqliteTriggerStore store(":memory:");

  TriggerInfo t;
  t.id = "full_test";
  t.entity_id = "motor_1";
  t.entity_type = "components";
  t.resource_uri = "/api/v1/components/motor_1/data/speed";
  t.collection = "data";
  t.resource_path = "speed";
  t.path = "/data";
  t.condition_type = "LeaveRange";
  t.condition_params = json{{"lower_bound", 20}, {"upper_bound", 30}};
  t.protocol = "sse";
  t.multishot = false;
  t.persistent = true;
  t.lifetime_sec = 120;
  t.log_settings = json{{"severity", "warning"}, {"max_entries", 100}};
  t.status = TriggerStatus::TERMINATED;
  // Use a fixed time_point for deterministic testing (truncated to seconds)
  t.created_at = std::chrono::system_clock::from_time_t(1700000000);
  t.expires_at = std::chrono::system_clock::from_time_t(1700000120);

  ASSERT_TRUE(store.save(t).has_value());

  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);

  const auto & r = loaded->at(0);
  EXPECT_EQ(r.id, "full_test");
  EXPECT_EQ(r.entity_id, "motor_1");
  EXPECT_EQ(r.entity_type, "components");
  EXPECT_EQ(r.resource_uri, "/api/v1/components/motor_1/data/speed");
  EXPECT_EQ(r.collection, "data");
  EXPECT_EQ(r.resource_path, "speed");
  EXPECT_EQ(r.path, "/data");
  EXPECT_EQ(r.condition_type, "LeaveRange");
  EXPECT_EQ(r.condition_params, json({{"lower_bound", 20}, {"upper_bound", 30}}));
  EXPECT_EQ(r.protocol, "sse");
  EXPECT_FALSE(r.multishot);
  EXPECT_TRUE(r.persistent);
  ASSERT_TRUE(r.lifetime_sec.has_value());
  EXPECT_EQ(r.lifetime_sec.value(), 120);
  ASSERT_TRUE(r.log_settings.has_value());
  EXPECT_EQ(r.log_settings.value(), json({{"severity", "warning"}, {"max_entries", 100}}));
  EXPECT_EQ(r.status, TriggerStatus::TERMINATED);
  EXPECT_EQ(std::chrono::system_clock::to_time_t(r.created_at), 1700000000);
  ASSERT_TRUE(r.expires_at.has_value());
  EXPECT_EQ(std::chrono::system_clock::to_time_t(r.expires_at.value()), 1700000120);
}

// ===========================================================================
// Optional fields as nullopt
// ===========================================================================

TEST(TriggerStore, OptionalFieldsNullopt) {
  SqliteTriggerStore store(":memory:");

  TriggerInfo t = make_trigger("nullopt_test");
  t.lifetime_sec = std::nullopt;
  t.log_settings = std::nullopt;
  t.expires_at = std::nullopt;

  ASSERT_TRUE(store.save(t).has_value());

  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);

  const auto & r = loaded->at(0);
  EXPECT_FALSE(r.lifetime_sec.has_value());
  EXPECT_FALSE(r.log_settings.has_value());
  EXPECT_FALSE(r.expires_at.has_value());
}

// ===========================================================================
// Idempotent upsert: save same trigger twice
// ===========================================================================

TEST(TriggerStore, UpsertSameTrigger) {
  SqliteTriggerStore store(":memory:");

  auto t = make_trigger("dup_001");
  ASSERT_TRUE(store.save(t).has_value());

  // Modify a field and re-save
  t.condition_type = "LeaveRange";
  ASSERT_TRUE(store.save(t).has_value());

  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  EXPECT_EQ(loaded->at(0).condition_type, "LeaveRange");
}

// ===========================================================================
// update
// ===========================================================================

TEST(TriggerStore, UpdateFields) {
  SqliteTriggerStore store(":memory:");

  auto t = make_trigger("upd_001");
  ASSERT_TRUE(store.save(t).has_value());

  // Update status and lifetime_sec
  json fields = {{"status", "TERMINATED"}, {"lifetime_sec", "7200"}};
  auto result = store.update("upd_001", fields);
  ASSERT_TRUE(result.has_value()) << result.error();

  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  EXPECT_EQ(loaded->at(0).status, TriggerStatus::TERMINATED);
  ASSERT_TRUE(loaded->at(0).lifetime_sec.has_value());
  EXPECT_EQ(loaded->at(0).lifetime_sec.value(), 7200);
}

TEST(TriggerStore, UpdateDisallowedColumn) {
  SqliteTriggerStore store(":memory:");
  auto t = make_trigger("upd_deny");
  ASSERT_TRUE(store.save(t).has_value());

  // Attempt to change primary key - should be rejected
  nlohmann::json fields = {{"id", "new_id"}};
  auto result = store.update("upd_deny", fields);
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("not updatable"), std::string::npos);
}

TEST(TriggerStore, UpdateNonexistent) {
  SqliteTriggerStore store(":memory:");

  json fields = {{"status", "TERMINATED"}};
  auto result = store.update("no_such_id", fields);
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("not found"), std::string::npos);
}

TEST(TriggerStore, UpdateEmptyFields) {
  SqliteTriggerStore store(":memory:");

  auto t = make_trigger("upd_empty");
  ASSERT_TRUE(store.save(t).has_value());

  auto result = store.update("upd_empty", json::object());
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("non-empty"), std::string::npos);
}

// ===========================================================================
// remove
// ===========================================================================

TEST(TriggerStore, RemoveTrigger) {
  SqliteTriggerStore store(":memory:");

  ASSERT_TRUE(store.save(make_trigger("rem_001")).has_value());
  ASSERT_TRUE(store.save(make_trigger("rem_002")).has_value());

  auto result = store.remove("rem_001");
  ASSERT_TRUE(result.has_value()) << result.error();

  auto loaded = store.load_all();
  ASSERT_TRUE(loaded.has_value());
  ASSERT_EQ(loaded->size(), 1u);
  EXPECT_EQ(loaded->at(0).id, "rem_002");
}

TEST(TriggerStore, RemoveNonexistent) {
  SqliteTriggerStore store(":memory:");

  auto result = store.remove("ghost");
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("not found"), std::string::npos);
}

// ===========================================================================
// save_state + load_state round-trip
// ===========================================================================

TEST(TriggerStore, SaveAndLoadState) {
  SqliteTriggerStore store(":memory:");

  json state = json{{"previous", 42.5}};
  auto save_result = store.save_state("trig_001", state);
  ASSERT_TRUE(save_result.has_value()) << save_result.error();

  auto load_result = store.load_state("trig_001");
  ASSERT_TRUE(load_result.has_value()) << load_result.error();
  ASSERT_TRUE(load_result->has_value());
  EXPECT_EQ(load_result->value(), state);
}

TEST(TriggerStore, LoadStateUnknownTrigger) {
  SqliteTriggerStore store(":memory:");

  auto result = store.load_state("nonexistent");
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_FALSE(result->has_value());  // nullopt - no state
}

TEST(TriggerStore, SaveStateOverwrite) {
  SqliteTriggerStore store(":memory:");

  ASSERT_TRUE(store.save_state("trig_x", json{{"v", 1}}).has_value());
  ASSERT_TRUE(store.save_state("trig_x", json{{"v", 2}}).has_value());

  auto result = store.load_state("trig_x");
  ASSERT_TRUE(result.has_value());
  ASSERT_TRUE(result->has_value());
  EXPECT_EQ(result->value(), json({{"v", 2}}));
}

// ===========================================================================
// remove also deletes associated state
// ===========================================================================

TEST(TriggerStore, RemoveDeletesState) {
  SqliteTriggerStore store(":memory:");

  ASSERT_TRUE(store.save(make_trigger("st_001")).has_value());
  ASSERT_TRUE(store.save_state("st_001", json{{"prev", 10}}).has_value());

  ASSERT_TRUE(store.remove("st_001").has_value());

  auto state = store.load_state("st_001");
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(state->has_value());  // state cleaned up
}

// ===========================================================================
// File-based database persistence
// ===========================================================================

// @verifies REQ_INTEROP_029
TEST(TriggerStore, FilePersistence) {
  std::string path = "/tmp/test_trigger_store_persist.db";
  std::remove(path.c_str());

  // Scope 1: create and save
  {
    SqliteTriggerStore store(path);
    ASSERT_TRUE(store.save(make_trigger("persist_001")).has_value());
    ASSERT_TRUE(store.save_state("persist_001", json{{"val", 99}}).has_value());
  }

  // Scope 2: reopen and verify
  {
    SqliteTriggerStore store(path);
    auto loaded = store.load_all();
    ASSERT_TRUE(loaded.has_value());
    ASSERT_EQ(loaded->size(), 1u);
    EXPECT_EQ(loaded->at(0).id, "persist_001");

    auto state = store.load_state("persist_001");
    ASSERT_TRUE(state.has_value());
    ASSERT_TRUE(state->has_value());
    EXPECT_EQ(state->value(), json({{"val", 99}}));
  }

  std::remove(path.c_str());
}
