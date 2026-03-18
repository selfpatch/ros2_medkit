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
#include <future>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_gateway/resource_change_notifier.hpp"

using namespace ros2_medkit_gateway;

// --- Subscribe + Notify ---

TEST(ResourceChangeNotifier, SubscribeAndNotify) {
  ResourceChangeNotifier notifier;

  std::promise<ResourceChange> promise;
  auto future = promise.get_future();

  notifier.subscribe({"faults", "", ""}, [&promise](const ResourceChange & change) {
    promise.set_value(change);
  });

  nlohmann::json value = {{"severity", "high"}};
  notifier.notify("faults", "temp_sensor", "fault_003", value, ChangeType::CREATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);

  auto result = future.get();
  EXPECT_EQ(result.collection, "faults");
  EXPECT_EQ(result.entity_id, "temp_sensor");
  EXPECT_EQ(result.resource_path, "fault_003");
  EXPECT_EQ(result.value, value);
  EXPECT_EQ(result.change_type, ChangeType::CREATED);
}

// --- Filter by collection ---

TEST(ResourceChangeNotifier, FilterByCollection) {
  ResourceChangeNotifier notifier;

  std::promise<ResourceChange> promise;
  auto future = promise.get_future();

  // Subscribe to "data" collection only
  notifier.subscribe({"data", "", ""}, [&promise](const ResourceChange & change) {
    promise.set_value(change);
  });

  // Notify on "faults" - should NOT trigger callback
  notifier.notify("faults", "sensor", "f1", {{"x", 1}}, ChangeType::CREATED);

  // Notify on "data" - should trigger callback
  nlohmann::json data_value = {{"temperature", 42.5}};
  notifier.notify("data", "sensor", "temp", data_value, ChangeType::UPDATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);

  auto result = future.get();
  EXPECT_EQ(result.collection, "data");
  EXPECT_EQ(result.resource_path, "temp");
}

// --- Filter by entity_id: empty = all entities ---

TEST(ResourceChangeNotifier, EmptyEntityIdMatchesAll) {
  ResourceChangeNotifier notifier;

  std::atomic<int> call_count{0};
  std::promise<void> done;
  auto future = done.get_future();

  notifier.subscribe({"data", "", ""}, [&](const ResourceChange & /*change*/) {
    if (call_count.fetch_add(1) + 1 == 2) {
      done.set_value();
    }
  });

  notifier.notify("data", "sensor_a", "temp", {}, ChangeType::UPDATED);
  notifier.notify("data", "sensor_b", "temp", {}, ChangeType::UPDATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);
  EXPECT_EQ(call_count.load(), 2);
}

// --- Filter by entity_id: specific = only that entity ---

TEST(ResourceChangeNotifier, SpecificEntityIdFilters) {
  ResourceChangeNotifier notifier;

  std::promise<ResourceChange> promise;
  auto future = promise.get_future();

  notifier.subscribe({"data", "sensor_b", ""}, [&promise](const ResourceChange & change) {
    promise.set_value(change);
  });

  // Should NOT match - different entity
  notifier.notify("data", "sensor_a", "temp", {{"v", 1}}, ChangeType::UPDATED);

  // Should match
  nlohmann::json expected_val = {{"v", 2}};
  notifier.notify("data", "sensor_b", "temp", expected_val, ChangeType::UPDATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);

  auto result = future.get();
  EXPECT_EQ(result.entity_id, "sensor_b");
  EXPECT_EQ(result.value, expected_val);
}

// --- Filter by resource_path: empty = all resources ---

TEST(ResourceChangeNotifier, EmptyResourcePathMatchesAll) {
  ResourceChangeNotifier notifier;

  std::atomic<int> call_count{0};
  std::promise<void> done;
  auto future = done.get_future();

  notifier.subscribe({"data", "sensor", ""}, [&](const ResourceChange & /*change*/) {
    if (call_count.fetch_add(1) + 1 == 2) {
      done.set_value();
    }
  });

  notifier.notify("data", "sensor", "temperature", {}, ChangeType::UPDATED);
  notifier.notify("data", "sensor", "humidity", {}, ChangeType::UPDATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);
  EXPECT_EQ(call_count.load(), 2);
}

// --- Filter by resource_path: specific = only that resource ---

TEST(ResourceChangeNotifier, SpecificResourcePathFilters) {
  ResourceChangeNotifier notifier;

  std::promise<ResourceChange> promise;
  auto future = promise.get_future();

  notifier.subscribe({"data", "sensor", "humidity"}, [&promise](const ResourceChange & change) {
    promise.set_value(change);
  });

  // Should NOT match - different resource_path
  notifier.notify("data", "sensor", "temperature", {{"v", 1}}, ChangeType::UPDATED);

  // Should match
  nlohmann::json expected_val = {{"v", 2}};
  notifier.notify("data", "sensor", "humidity", expected_val, ChangeType::UPDATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);

  auto result = future.get();
  EXPECT_EQ(result.resource_path, "humidity");
  EXPECT_EQ(result.value, expected_val);
}

// --- Unsubscribe ---

TEST(ResourceChangeNotifier, Unsubscribe) {
  ResourceChangeNotifier notifier;

  std::atomic<int> call_count{0};

  auto id = notifier.subscribe({"data", "", ""}, [&](const ResourceChange & /*change*/) {
    call_count.fetch_add(1);
  });

  // First notify - should trigger
  notifier.notify("data", "sensor", "temp", {}, ChangeType::UPDATED);

  // Wait a bit for async delivery
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(call_count.load(), 1);

  // Unsubscribe
  notifier.unsubscribe(id);

  // Second notify - should NOT trigger
  notifier.notify("data", "sensor", "temp", {}, ChangeType::UPDATED);

  // Wait to confirm no additional callback
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(call_count.load(), 1);
}

// --- Async dispatch: notify() returns quickly even with slow callback ---

TEST(ResourceChangeNotifier, AsyncDispatchNonBlocking) {
  ResourceChangeNotifier notifier;

  std::promise<void> callback_started;
  auto started_future = callback_started.get_future();

  notifier.subscribe({"data", "", ""}, [&](const ResourceChange & /*change*/) {
    callback_started.set_value();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  });

  auto start = std::chrono::steady_clock::now();
  notifier.notify("data", "sensor", "temp", {}, ChangeType::UPDATED);
  auto elapsed = std::chrono::steady_clock::now() - start;

  // notify() should return almost immediately (well under 10ms)
  EXPECT_LT(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count(), 10);

  // Wait for the callback to actually start (proves it runs asynchronously)
  auto status = started_future.wait_for(std::chrono::seconds(2));
  EXPECT_EQ(status, std::future_status::ready);
}

// --- Multiple subscribers ---

TEST(ResourceChangeNotifier, MultipleSubscribersAllCalled) {
  ResourceChangeNotifier notifier;

  std::atomic<int> count_a{0};
  std::atomic<int> count_b{0};
  std::atomic<int> total{0};
  std::promise<void> done;
  auto future = done.get_future();

  auto check_done = [&]() {
    if (total.fetch_add(1) + 1 == 2) {
      done.set_value();
    }
  };

  notifier.subscribe({"faults", "", ""}, [&](const ResourceChange & /*change*/) {
    count_a.fetch_add(1);
    check_done();
  });

  notifier.subscribe({"faults", "", ""}, [&](const ResourceChange & /*change*/) {
    count_b.fetch_add(1);
    check_done();
  });

  notifier.notify("faults", "sensor", "f1", {}, ChangeType::CREATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);
  EXPECT_EQ(count_a.load(), 1);
  EXPECT_EQ(count_b.load(), 1);
}

// --- Exception in callback: other subscribers still called ---

TEST(ResourceChangeNotifier, ExceptionInCallbackDoesNotBlockOthers) {
  ResourceChangeNotifier notifier;

  std::promise<ResourceChange> promise;
  auto future = promise.get_future();

  // First subscriber throws
  notifier.subscribe({"data", "", ""}, [](const ResourceChange & /*change*/) {
    throw std::runtime_error("callback exploded");
  });

  // Second subscriber should still be called
  notifier.subscribe({"data", "", ""}, [&promise](const ResourceChange & change) {
    promise.set_value(change);
  });

  notifier.notify("data", "sensor", "temp", {{"v", 42}}, ChangeType::UPDATED);

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);

  auto result = future.get();
  EXPECT_EQ(result.entity_id, "sensor");
}

// --- ChangeType propagation ---

TEST(ResourceChangeNotifier, ChangeTypePropagation) {
  ResourceChangeNotifier notifier;

  for (auto ct : {ChangeType::CREATED, ChangeType::UPDATED, ChangeType::DELETED}) {
    std::promise<ChangeType> promise;
    auto future = promise.get_future();

    auto id = notifier.subscribe({"data", "", ""}, [&promise](const ResourceChange & change) {
      promise.set_value(change.change_type);
    });

    notifier.notify("data", "sensor", "temp", {}, ct);

    auto status = future.wait_for(std::chrono::seconds(2));
    ASSERT_EQ(status, std::future_status::ready);
    EXPECT_EQ(future.get(), ct);

    notifier.unsubscribe(id);
    // Brief pause to let the worker finish processing before next iteration
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

// --- Timestamp set correctly ---

TEST(ResourceChangeNotifier, TimestampWithinTolerance) {
  ResourceChangeNotifier notifier;

  std::promise<std::chrono::system_clock::time_point> promise;
  auto future = promise.get_future();

  notifier.subscribe({"data", "", ""}, [&promise](const ResourceChange & change) {
    promise.set_value(change.timestamp);
  });

  auto before = std::chrono::system_clock::now();
  notifier.notify("data", "sensor", "temp", {}, ChangeType::UPDATED);
  auto after = std::chrono::system_clock::now();

  auto status = future.wait_for(std::chrono::seconds(2));
  ASSERT_EQ(status, std::future_status::ready);

  auto ts = future.get();
  EXPECT_GE(ts, before);
  EXPECT_LE(ts, after + std::chrono::milliseconds(10));
}

// --- Notify after shutdown is a no-op ---

TEST(ResourceChangeNotifier, NotifyAfterShutdownIsNoop) {
  ResourceChangeNotifier notifier;

  std::atomic<int> call_count{0};

  notifier.subscribe({"data", "", ""}, [&](const ResourceChange & /*change*/) {
    call_count.fetch_add(1);
  });

  notifier.shutdown();

  // Should be a no-op
  notifier.notify("data", "sensor", "temp", {}, ChangeType::UPDATED);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(call_count.load(), 0);
}

// --- Double shutdown is safe ---

TEST(ResourceChangeNotifier, DoubleShutdownIsSafe) {
  ResourceChangeNotifier notifier;
  notifier.shutdown();
  EXPECT_NO_THROW(notifier.shutdown());
}

// --- Unsubscribe with invalid ID is safe ---

TEST(ResourceChangeNotifier, UnsubscribeInvalidIdIsSafe) {
  ResourceChangeNotifier notifier;
  EXPECT_NO_THROW(notifier.unsubscribe(999));
}
