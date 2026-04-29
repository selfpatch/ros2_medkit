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

// TriggerManager routing test: links exclusively against gateway_core +
// GTest. No rclcpp/ament dependencies on the link line, proving that the
// manager body lives in the middleware-neutral build layer. The
// rclcpp::GenericSubscription / type lookup / pending retry concerns belong
// to the Ros2TopicSubscriptionTransport adapter; here we exercise the
// manager against a mock transport that records subscribe/unsubscribe
// calls and lets the test emit JSON samples directly.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/condition_evaluator.hpp"
#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/core/sqlite_trigger_store.hpp"
#include "ros2_medkit_gateway/core/transports/topic_subscription_transport.hpp"

namespace ros2_medkit_gateway {
namespace {

// ----------------------------------------------------------------------------
// Mock transport
// ----------------------------------------------------------------------------

/// Mock TopicSubscriptionTransport recording every subscribe/unsubscribe so
/// the test can assert what the manager routes through it. Each subscribe()
/// returns a handle whose dtor flips a per-key "alive" flag, mirroring the
/// real adapter's contract.
class MockTopicSubscriptionTransport : public TopicSubscriptionTransport {
 public:
  struct Subscription {
    std::string topic_path;
    std::string msg_type;
    SampleCallback callback;
    std::shared_ptr<std::atomic<bool>> alive = std::make_shared<std::atomic<bool>>(true);
  };

  class Handle : public TopicSubscriptionHandle {
   public:
    Handle(MockTopicSubscriptionTransport * owner, std::string key) : owner_(owner), key_(std::move(key)) {
    }

    ~Handle() override {
      owner_->release(key_);
    }

    Handle(const Handle &) = delete;
    Handle & operator=(const Handle &) = delete;
    Handle(Handle &&) = delete;
    Handle & operator=(Handle &&) = delete;

    const std::string & key() const {
      return key_;
    }

   private:
    MockTopicSubscriptionTransport * owner_;
    std::string key_;
  };

  std::unique_ptr<TopicSubscriptionHandle> subscribe(const std::string & topic_path, const std::string & msg_type,
                                                     SampleCallback callback) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto key = "mock_" + std::to_string(next_id_++);
    Subscription entry;
    entry.topic_path = topic_path;
    entry.msg_type = msg_type;
    entry.callback = std::move(callback);
    subs_[key] = std::move(entry);
    subscribe_log_.emplace_back(key, topic_path);
    return std::make_unique<Handle>(this, key);
  }

  /// Test helper: emit a sample as if rclcpp had just delivered one.
  /// Returns true when the key is still alive (callback was invoked).
  bool emit(const std::string & key, const nlohmann::json & sample) {
    SampleCallback cb;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      auto it = subs_.find(key);
      if (it == subs_.end() || !it->second.alive->load()) {
        return false;
      }
      cb = it->second.callback;
    }
    if (!cb) {
      return false;
    }
    cb(sample);
    return true;
  }

  /// Test helper: returns the topic path most recently registered.
  std::string last_subscribed_topic() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return subscribe_log_.empty() ? std::string() : subscribe_log_.back().second;
  }

  /// Test helper: count of currently-alive subscriptions.
  size_t alive_count() const {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t n = 0;
    for (const auto & [_, sub] : subs_) {
      if (sub.alive->load()) {
        ++n;
      }
    }
    return n;
  }

  /// Test helper: total number of subscribe() calls observed.
  size_t total_subscribes() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return subscribe_log_.size();
  }

  /// Test helper: list of keys still alive.
  std::vector<std::string> alive_keys() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out;
    for (const auto & [key, sub] : subs_) {
      if (sub.alive->load()) {
        out.push_back(key);
      }
    }
    return out;
  }

 private:
  void release(const std::string & key) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = subs_.find(key);
    if (it == subs_.end()) {
      return;
    }
    it->second.alive->store(false);
    it->second.callback = nullptr;
  }

  mutable std::mutex mutex_;
  std::unordered_map<std::string, Subscription> subs_;
  std::vector<std::pair<std::string, std::string>> subscribe_log_;  // (key, topic)
  uint64_t next_id_ = 1;
};

// ----------------------------------------------------------------------------
// Fixture
// ----------------------------------------------------------------------------

class TriggerManagerRoutingTest : public ::testing::Test {
 protected:
  void SetUp() override {
    registry_.register_condition("OnChange", std::make_shared<OnChangeEvaluator>());

    transport_ = std::make_shared<MockTopicSubscriptionTransport>();

    TriggerConfig config;
    config.max_triggers = 50;
    config.on_restart_behavior = "reset";
    manager_ = std::make_unique<TriggerManager>(notifier_, registry_, store_, config, transport_);
  }

  void TearDown() override {
    if (manager_) {
      manager_->shutdown();
    }
    notifier_.shutdown();
  }

  TriggerCreateRequest make_data_request(const std::string & entity_id = "sensor",
                                         const std::string & resolved_topic = "/sensor/temperature",
                                         const std::string & resource_path = "/temperature") {
    TriggerCreateRequest req;
    req.entity_id = entity_id;
    req.entity_type = "apps";
    req.resource_uri = "/api/v1/apps/" + entity_id + "/data" + resource_path;
    req.collection = "data";
    req.resource_path = resource_path;
    req.resolved_topic_name = resolved_topic;
    req.path = "";
    req.condition_type = "OnChange";
    req.condition_params = nlohmann::json::object();
    req.protocol = "sse";
    req.multishot = true;
    req.persistent = false;
    return req;
  }

  ResourceChangeNotifier notifier_;
  ConditionRegistry registry_;
  SqliteTriggerStore store_{":memory:"};
  std::shared_ptr<MockTopicSubscriptionTransport> transport_;
  std::unique_ptr<TriggerManager> manager_;
};

// ----------------------------------------------------------------------------
// Routing tests (~6 cases)
// ----------------------------------------------------------------------------

TEST_F(TriggerManagerRoutingTest, RegisteredTriggerSubscribesToTransport) {
  auto created = manager_->create(make_data_request("sensor", "/sensor/temperature"));
  ASSERT_TRUE(created.has_value()) << created.error().message;

  EXPECT_EQ(transport_->total_subscribes(), 1u);
  EXPECT_EQ(transport_->alive_count(), 1u);
  EXPECT_EQ(transport_->last_subscribed_topic(), "/sensor/temperature");
}

TEST_F(TriggerManagerRoutingTest, RemovedTriggerDestructsHandle) {
  auto created = manager_->create(make_data_request());
  ASSERT_TRUE(created.has_value());
  ASSERT_EQ(transport_->alive_count(), 1u);

  ASSERT_TRUE(manager_->remove(created->id));
  EXPECT_EQ(transport_->alive_count(), 0u)
      << "Removing a data trigger must drop its TopicSubscriptionHandle, which must unsubscribe";
}

TEST_F(TriggerManagerRoutingTest, SampleCallbackRoutesToConditionEvaluator) {
  auto created = manager_->create(make_data_request("sensor", "/sensor/temperature", "/temperature"));
  ASSERT_TRUE(created.has_value());

  auto keys = transport_->alive_keys();
  ASSERT_EQ(keys.size(), 1u);

  // Route a sample through the mock transport - this must reach the
  // resource-change notifier and fire the OnChange evaluator.
  ASSERT_TRUE(transport_->emit(keys[0], nlohmann::json(42.0)));
  ASSERT_TRUE(manager_->wait_for_event(created->id, std::chrono::milliseconds(2000)));

  auto event = manager_->consume_pending_event(created->id);
  ASSERT_TRUE(event.has_value());
  EXPECT_TRUE(event->contains("payload"));
  EXPECT_EQ((*event)["payload"], nlohmann::json(42.0));
}

TEST_F(TriggerManagerRoutingTest, OrphanedTriggerSweepUnsubscribes) {
  auto alive = manager_->create(make_data_request("alive_app", "/alive_app/temperature"));
  auto gone = manager_->create(make_data_request("gone_app", "/gone_app/temperature"));
  ASSERT_TRUE(alive.has_value());
  ASSERT_TRUE(gone.has_value());
  ASSERT_EQ(transport_->alive_count(), 2u);

  manager_->set_entity_exists_fn([](const std::string & id, const std::string & /*type*/) {
    return id != "gone_app";
  });
  manager_->sweep_orphaned_triggers();

  EXPECT_EQ(transport_->alive_count(), 1u) << "Orphan sweep must unsubscribe gone_app's transport handle";
  EXPECT_TRUE(manager_->list("gone_app").empty());
  EXPECT_EQ(manager_->list("alive_app").size(), 1u);
}

TEST_F(TriggerManagerRoutingTest, MultipleTriggersIndependentSubscriptions) {
  auto t1 = manager_->create(make_data_request("sensor_a", "/sensor_a/temperature", "/temperature"));
  auto t2 = manager_->create(make_data_request("sensor_b", "/sensor_b/temperature", "/temperature"));
  ASSERT_TRUE(t1.has_value());
  ASSERT_TRUE(t2.has_value());

  // Two distinct subscribe calls, two alive handles.
  EXPECT_EQ(transport_->total_subscribes(), 2u);
  EXPECT_EQ(transport_->alive_count(), 2u);

  // Removing only one drops only that handle; the other survives.
  ASSERT_TRUE(manager_->remove(t1->id));
  EXPECT_EQ(transport_->alive_count(), 1u);
}

TEST_F(TriggerManagerRoutingTest, ShutdownClearsAllSubscriptions) {
  ASSERT_TRUE(manager_->create(make_data_request("a", "/a/x", "/x")).has_value());
  ASSERT_TRUE(manager_->create(make_data_request("b", "/b/x", "/x")).has_value());
  ASSERT_TRUE(manager_->create(make_data_request("c", "/c/x", "/x")).has_value());
  ASSERT_EQ(transport_->alive_count(), 3u);

  manager_->shutdown();
  EXPECT_EQ(transport_->alive_count(), 0u) << "Manager shutdown must drop every transport handle";
}

TEST_F(TriggerManagerRoutingTest, NonDataTriggerDoesNotSubscribe) {
  // Faults / configurations / etc. don't go through the topic transport.
  TriggerCreateRequest req;
  req.entity_id = "sensor";
  req.entity_type = "apps";
  req.resource_uri = "/api/v1/apps/sensor/faults/fault_001";
  req.collection = "faults";
  req.resource_path = "/fault_001";
  req.resolved_topic_name = "";
  req.path = "";
  req.condition_type = "OnChange";
  req.condition_params = nlohmann::json::object();
  req.protocol = "sse";
  req.multishot = true;
  req.persistent = false;

  ASSERT_TRUE(manager_->create(req).has_value());
  EXPECT_EQ(transport_->total_subscribes(), 0u) << "Non-data triggers must not register any topic subscription";
}

}  // namespace
}  // namespace ros2_medkit_gateway
