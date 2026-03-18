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

#include <atomic>
#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/condition_evaluator.hpp"
#include "ros2_medkit_gateway/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/sqlite_trigger_store.hpp"
#include "ros2_medkit_gateway/trigger_manager.hpp"

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

// ===========================================================================
// Test fixture
// ===========================================================================

class TriggerManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Register all built-in condition evaluators
    registry_.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());
    registry_.register_condition("OnChangeTo", std::make_shared<OnChangeToEvaluator>());
    registry_.register_condition("EnterRange", std::make_shared<EnterRangeEvaluator>());
    registry_.register_condition("LeaveRange", std::make_shared<LeaveRangeEvaluator>());

    TriggerConfig config;
    config.max_triggers = 10;
    config.on_restart_behavior = "reset";

    manager_ = std::make_unique<TriggerManager>(notifier_, registry_, store_, config);
  }

  void TearDown() override {
    if (manager_) {
      manager_->shutdown();
    }
    notifier_.shutdown();
  }

  TriggerCreateRequest make_request(const std::string & entity_id = "sensor",
                                    const std::string & condition_type = "OnChange",
                                    const json & condition_params = json::object()) {
    TriggerCreateRequest req;
    req.entity_id = entity_id;
    req.entity_type = "apps";
    req.resource_uri = "/api/v1/apps/" + entity_id + "/data/temperature";
    req.collection = "data";
    req.resource_path = "temperature";
    req.path = "";
    req.condition_type = condition_type;
    req.condition_params = condition_params;
    req.protocol = "sse";
    req.multishot = false;
    req.persistent = false;
    return req;
  }

  ResourceChangeNotifier notifier_;
  ConditionRegistry registry_;
  SqliteTriggerStore store_{":memory:"};
  std::unique_ptr<TriggerManager> manager_;
};

// ===========================================================================
// CRUD Tests
// ===========================================================================

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, Create_ValidOnChangeTrigger) {
  auto result = manager_->create(make_request());
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_FALSE(result->id.empty());
  EXPECT_EQ(result->entity_id, "sensor");
  EXPECT_EQ(result->entity_type, "apps");
  EXPECT_EQ(result->collection, "data");
  EXPECT_EQ(result->condition_type, "OnChange");
  EXPECT_EQ(result->status, TriggerStatus::ACTIVE);
}

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, Create_InvalidConditionType) {
  auto req = make_request("sensor", "NonexistentCondition");
  auto result = manager_->create(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("condition"), std::string::npos);
}

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, Create_InvalidParams) {
  // EnterRange requires lower_bound <= upper_bound
  json params = {{"lower_bound", 30}, {"upper_bound", 10}};
  auto req = make_request("sensor", "EnterRange", params);
  auto result = manager_->create(req);
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("lower_bound"), std::string::npos);
}

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, Create_MaxTriggersExceeded) {
  for (int i = 0; i < 10; ++i) {
    auto r = manager_->create(make_request("entity_" + std::to_string(i)));
    ASSERT_TRUE(r.has_value()) << "Trigger " << i << " should succeed: " << r.error();
  }
  auto result = manager_->create(make_request("overflow"));
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("capacity"), std::string::npos);
}

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, Get_Existing) {
  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  auto fetched = manager_->get(created->id);
  ASSERT_TRUE(fetched.has_value());
  EXPECT_EQ(fetched->id, created->id);
  EXPECT_EQ(fetched->entity_id, "sensor");
}

TEST_F(TriggerManagerTest, Get_NonExisting) {
  auto fetched = manager_->get("nonexistent");
  EXPECT_FALSE(fetched.has_value());
}

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, List_ReturnsTriggersForEntity) {
  (void)manager_->create(make_request("sensor"));
  (void)manager_->create(make_request("sensor"));
  (void)manager_->create(make_request("actuator"));

  auto sensor_triggers = manager_->list("sensor");
  EXPECT_EQ(sensor_triggers.size(), 2u);

  auto actuator_triggers = manager_->list("actuator");
  EXPECT_EQ(actuator_triggers.size(), 1u);

  auto empty = manager_->list("nonexistent");
  EXPECT_TRUE(empty.empty());
}

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, Update_ChangeLifetime) {
  auto req = make_request();
  req.lifetime_sec = 300;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  auto updated = manager_->update(created->id, 600);
  ASSERT_TRUE(updated.has_value()) << updated.error();
  EXPECT_EQ(updated->lifetime_sec, 600);
  EXPECT_TRUE(updated->expires_at.has_value());
}

TEST_F(TriggerManagerTest, Update_NonExisting) {
  auto result = manager_->update("nonexistent", 600);
  EXPECT_FALSE(result.has_value());
}

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, Remove_ExistingTrigger) {
  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  EXPECT_TRUE(manager_->remove(created->id));
  EXPECT_FALSE(manager_->get(created->id).has_value());
}

TEST_F(TriggerManagerTest, Remove_NonExisting) {
  EXPECT_FALSE(manager_->remove("nonexistent"));
}

// ===========================================================================
// Event Evaluation Tests
// ===========================================================================

// @verifies REQ_TRIGGER_002
TEST_F(TriggerManagerTest, SingleShot_NotifyMatchingChange) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = false;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify a matching resource change
  notifier_.notify("data", "sensor", "temperature", json(42.0));

  // Give the async notifier time to process
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  EXPECT_TRUE(event->contains("timestamp"));
  EXPECT_TRUE(event->contains("payload"));

  // Single-shot: should be terminated now
  auto info = manager_->get(created->id);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, TriggerStatus::TERMINATED);
}

// @verifies REQ_TRIGGER_002
TEST_F(TriggerManagerTest, Multishot_NotifyTwice) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // First change
  notifier_.notify("data", "sensor", "temperature", json(42.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event1 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event1.has_value());

  // Second change
  notifier_.notify("data", "sensor", "temperature", json(43.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event2 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event2.has_value());

  // Should still be active
  EXPECT_TRUE(manager_->is_active(created->id));
}

// @verifies REQ_TRIGGER_002
TEST_F(TriggerManagerTest, LifetimeExpiry) {
  auto req = make_request();
  req.lifetime_sec = 1;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  EXPECT_TRUE(manager_->is_active(created->id));

  std::this_thread::sleep_for(std::chrono::seconds(2));

  EXPECT_FALSE(manager_->is_active(created->id));
}

// ===========================================================================
// Hierarchy Matching Tests
// ===========================================================================

// @verifies REQ_TRIGGER_003
TEST_F(TriggerManagerTest, Hierarchy_ComponentToApp) {
  // Set up hierarchy: component "chassis" contains app "sensor"
  manager_->set_entity_children_fn(
      [](const std::string & entity_id, const std::string & /*entity_type*/) -> std::vector<std::string> {
        if (entity_id == "chassis") {
          return {"sensor", "actuator"};
        }
        return {};
      });

  auto req = make_request("chassis", "OnChange");
  req.entity_type = "components";
  req.resource_uri = "/api/v1/components/chassis/data/temperature";
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify from a child app "sensor" - should fire the trigger
  notifier_.notify("data", "sensor", "temperature", json(42.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
}

// @verifies REQ_TRIGGER_003
TEST_F(TriggerManagerTest, Hierarchy_AreaToApp) {
  // Set up hierarchy: area "navigation" contains apps
  manager_->set_entity_children_fn(
      [](const std::string & entity_id, const std::string & /*entity_type*/) -> std::vector<std::string> {
        if (entity_id == "navigation") {
          return {"lidar", "imu", "planner"};
        }
        return {};
      });

  auto req = make_request("navigation", "OnChange");
  req.entity_type = "areas";
  req.resource_uri = "/api/v1/areas/navigation/data/temperature";
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify from nested app "lidar"
  notifier_.notify("data", "lidar", "temperature", json(55.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
}

TEST_F(TriggerManagerTest, Hierarchy_NoMatchForUnrelatedEntity) {
  manager_->set_entity_children_fn(
      [](const std::string & entity_id, const std::string & /*entity_type*/) -> std::vector<std::string> {
        if (entity_id == "chassis") {
          return {"sensor"};
        }
        return {};
      });

  auto req = make_request("chassis", "OnChange");
  req.entity_type = "components";
  req.resource_uri = "/api/v1/components/chassis/data/temperature";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify from unrelated entity "other_entity"
  notifier_.notify("data", "other_entity", "temperature", json(42.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event.has_value());
}

// ===========================================================================
// Callback Tests
// ===========================================================================

// @verifies REQ_TRIGGER_001
TEST_F(TriggerManagerTest, OnRemovedCallback) {
  std::vector<std::string> removed_ids;
  manager_->set_on_removed([&](const std::string & trigger_id) {
    removed_ids.push_back(trigger_id);
  });

  auto created = manager_->create(make_request());
  ASSERT_TRUE(created.has_value());

  manager_->remove(created->id);

  ASSERT_EQ(removed_ids.size(), 1u);
  EXPECT_EQ(removed_ids[0], created->id);
}

// ===========================================================================
// Event Envelope Tests
// ===========================================================================

// @verifies REQ_TRIGGER_002
TEST_F(TriggerManagerTest, EventEnvelopeFormat) {
  auto req = make_request("sensor", "OnChange");
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  notifier_.notify("data", "sensor", "temperature", json(42.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  EXPECT_TRUE(event->contains("timestamp"));
  EXPECT_TRUE(event->contains("payload"));
  EXPECT_TRUE((*event)["timestamp"].is_string());
  // Timestamp should be ISO 8601 format
  std::string ts = (*event)["timestamp"].get<std::string>();
  EXPECT_NE(ts.find("T"), std::string::npos);
}

// ===========================================================================
// Consume/Wait Tests
// ===========================================================================

TEST_F(TriggerManagerTest, ConsumePendingEvent_ClearsEvent) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  notifier_.notify("data", "sensor", "temperature", json(42.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event1 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event1.has_value());

  // After consuming, should return nullopt until a new event arrives
  auto event2 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event2.has_value());
}

TEST_F(TriggerManagerTest, WaitForEvent_BlocksUntilEventArrives) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  std::atomic<bool> event_received{false};
  std::thread waiter([&]() {
    bool got_event = manager_->wait_for_event(created->id, std::chrono::milliseconds(2000));
    if (got_event) {
      event_received.store(true);
    }
  });

  // Give waiter thread time to start waiting
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // Send event
  notifier_.notify("data", "sensor", "temperature", json(42.0));

  waiter.join();
  EXPECT_TRUE(event_received.load());
}

TEST_F(TriggerManagerTest, WaitForEvent_TimesOutWhenNoEvent) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  auto start = std::chrono::steady_clock::now();
  bool got_event = manager_->wait_for_event(created->id, std::chrono::milliseconds(100));
  auto elapsed = std::chrono::steady_clock::now() - start;

  EXPECT_FALSE(got_event);
  EXPECT_GE(elapsed, std::chrono::milliseconds(80));
}

// ===========================================================================
// Shutdown Tests
// ===========================================================================

TEST_F(TriggerManagerTest, Shutdown_WakesWaitingThreads) {
  auto req = make_request("sensor", "OnChange");
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  std::atomic<bool> waiter_returned{false};
  std::thread waiter([&]() {
    manager_->wait_for_event(created->id, std::chrono::seconds(30));
    waiter_returned.store(true);
  });

  // Give waiter time to start
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  manager_->shutdown();

  waiter.join();
  EXPECT_TRUE(waiter_returned.load());
}

// ===========================================================================
// JSON Pointer Extraction Tests
// ===========================================================================

// @verifies REQ_TRIGGER_002
TEST_F(TriggerManagerTest, JsonPointer_ExtractsSubElement) {
  auto req = make_request("sensor", "OnChange");
  req.path = "/data";
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // The change value has a "data" field
  json value = {{"data", 42.0}, {"metadata", "ignored"}};
  notifier_.notify("data", "sensor", "temperature", value);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  // The payload should be the full change value
  EXPECT_TRUE(event->contains("payload"));
}

TEST_F(TriggerManagerTest, JsonPointer_EmptyPathUsesFullValue) {
  auto req = make_request("sensor", "OnChange");
  req.path = "";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  notifier_.notify("data", "sensor", "temperature", json(42.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  EXPECT_EQ((*event)["payload"], json(42.0));
}

// ===========================================================================
// Condition-Specific Tests
// ===========================================================================

TEST_F(TriggerManagerTest, OnChangeTo_FiresOnlyWhenTargetReached) {
  auto req = make_request("sensor", "OnChangeTo", {{"target_value", 100}});
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // First notification: value is 100, no previous -> fires (target match, first eval)
  notifier_.notify("data", "sensor", "temperature", json(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event1 = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event1.has_value());

  // Second notification: value stays 100 -> does NOT fire (no change)
  notifier_.notify("data", "sensor", "temperature", json(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event2 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event2.has_value());

  // Third notification: value goes to 50 -> does NOT fire (not target)
  notifier_.notify("data", "sensor", "temperature", json(50));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event3 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event3.has_value());

  // Fourth notification: back to 100 -> fires (transition to target)
  notifier_.notify("data", "sensor", "temperature", json(100));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event4 = manager_->consume_pending_event(created->id);
  EXPECT_TRUE(event4.has_value());
}

TEST_F(TriggerManagerTest, EnterRange_FiresOnTransition) {
  json params = {{"lower_bound", 20}, {"upper_bound", 30}};
  auto req = make_request("sensor", "EnterRange", params);
  req.multishot = true;
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // First notification: value = 10, no previous -> EnterRange returns false (needs transition)
  notifier_.notify("data", "sensor", "temperature", json(10));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event1 = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event1.has_value());

  // Second notification: 10 -> 25 (enters range) -> fires
  notifier_.notify("data", "sensor", "temperature", json(25));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  auto event2 = manager_->consume_pending_event(created->id);
  EXPECT_TRUE(event2.has_value());
}

// ===========================================================================
// Non-matching resource path / collection
// ===========================================================================

TEST_F(TriggerManagerTest, NoFireForDifferentCollection) {
  auto req = make_request("sensor", "OnChange");
  req.collection = "data";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify with different collection "faults"
  notifier_.notify("faults", "sensor", "temperature", json("fault_data"));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event.has_value());
}

TEST_F(TriggerManagerTest, NoFireForDifferentResourcePath) {
  auto req = make_request("sensor", "OnChange");
  req.resource_path = "temperature";
  auto created = manager_->create(req);
  ASSERT_TRUE(created.has_value());

  // Notify with different resource path "humidity"
  notifier_.notify("data", "sensor", "humidity", json(65.0));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  auto event = manager_->consume_pending_event(created->id);
  EXPECT_FALSE(event.has_value());
}
