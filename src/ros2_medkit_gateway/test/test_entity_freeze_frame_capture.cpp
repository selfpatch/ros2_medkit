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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <set>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/entity_freeze_frame_store.hpp"
#include "ros2_medkit_gateway/entity_freeze_frame_capture.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_gateway/ros2_common/ros2_subscription_executor.hpp"
#include "ros2_medkit_msgs/msg/fault_event.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using ros2_medkit_gateway::DataProvider;
using ros2_medkit_gateway::DataProviderError;
using ros2_medkit_gateway::DataProviderErrorInfo;
using ros2_medkit_gateway::EntityFreezeFrameCapture;
using ros2_medkit_gateway::InMemoryEntityFreezeFrameStore;
using ros2_medkit_gateway::StoredEntityFreezeFrame;
using ros2_medkit_gateway::handlers::FaultHandlers;
using ros2_medkit_gateway::ros2_common::Ros2SubscriptionExecutor;
using ros2_medkit_msgs::msg::Fault;
using ros2_medkit_msgs::msg::FaultEvent;

namespace {

/// Fake plugin DataProvider serving PLC-like values for one entity. The
/// temperature is settable so tests can prove a re-confirm re-samples.
class FakePlcDataProvider : public DataProvider {
 public:
  explicit FakePlcDataProvider(std::string entity_id) : entity_id_(std::move(entity_id)) {
  }

  void set_temperature(double value) {
    temperature_.store(value);
  }

  tl::expected<ros2_medkit_gateway::dto::DataListResult, DataProviderErrorInfo>
  list_data(const std::string & entity_id) override {
    if (entity_id != entity_id_) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::EntityNotFound, "not found", 404});
    }
    json items = json::array();
    items.push_back({{"id", "temperature"}, {"name", "Temperature"}, {"value", temperature_.load()}});
    items.push_back({{"id", "pressure"}, {"name", "Pressure"}, {"value", 3.2}});
    return ros2_medkit_gateway::dto::DataListResult{json{{"items", std::move(items)}}};
  }

  tl::expected<ros2_medkit_gateway::dto::DataValue, DataProviderErrorInfo>
  read_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "unused", 500});
  }

  tl::expected<ros2_medkit_gateway::dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/,
             const json & /*value*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read-only", 405});
  }

 private:
  std::string entity_id_;
  std::atomic<double> temperature_{42.5};
};

/// Fake DataProvider serving fixed list_data content (for gate tests).
class StaticContentDataProvider : public DataProvider {
 public:
  StaticContentDataProvider(std::string entity_id, json content)
    : entity_id_(std::move(entity_id)), content_(std::move(content)) {
  }

  tl::expected<ros2_medkit_gateway::dto::DataListResult, DataProviderErrorInfo>
  list_data(const std::string & entity_id) override {
    if (entity_id != entity_id_) {
      return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::EntityNotFound, "not found", 404});
    }
    return ros2_medkit_gateway::dto::DataListResult{content_};
  }

  tl::expected<ros2_medkit_gateway::dto::DataValue, DataProviderErrorInfo>
  read_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::Internal, "unused", 500});
  }

  tl::expected<ros2_medkit_gateway::dto::DataWriteResult, DataProviderErrorInfo>
  write_data(const std::string & /*entity_id*/, const std::string & /*resource_name*/,
             const json & /*value*/) override {
    return tl::make_unexpected(DataProviderErrorInfo{DataProviderError::ReadOnly, "read-only", 405});
  }

 private:
  std::string entity_id_;
  json content_;
};

FaultEvent make_confirmed_event(const std::string & fault_code, const std::vector<std::string> & sources) {
  FaultEvent event;
  event.event_type = FaultEvent::EVENT_CONFIRMED;
  event.fault.fault_code = fault_code;
  event.fault.severity = Fault::SEVERITY_ERROR;
  event.fault.status = Fault::STATUS_CONFIRMED;
  event.fault.reporting_sources = sources;
  return event;
}

class EntityFreezeFrameCaptureTest : public ::testing::Test {
 protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("entity_freeze_frame_test_node");
    // Publisher lives on a node the main executor never spins (see
    // Ros2SubscriptionSlotTest): publishers mutate their node's rcutils_hash_map
    // under the hood, which TSan otherwise flags against the spun node.
    publisher_node_ = std::make_shared<rclcpp::Node>("entity_freeze_frame_test_publisher");
    publisher_ = publisher_node_->create_publisher<FaultEvent>("/fault_manager/events", rclcpp::QoS(100).reliable());
    provider_ = std::make_unique<FakePlcDataProvider>("plc_app");

    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this] {
      executor_->spin();
    });
    // The capture creates its fault-events subscription on this executor's
    // dedicated _sub node, off the main node (issue #375).
    sub_exec_ = std::make_unique<Ros2SubscriptionExecutor>(node_);
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    sub_exec_.reset();
    executor_.reset();
    publisher_.reset();
    publisher_node_.reset();
    node_.reset();
  }

  /// Block until the capture's subscription has matched our publisher, so a
  /// reliable-QoS publish is not dropped by a late-joining subscriber.
  bool wait_for_match() {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (publisher_->get_subscription_count() > 0) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return false;
  }

  /// Publish an event repeatedly until the capture holds frames (or timeout).
  /// The callback runs on the subscription executor worker, not this thread.
  bool publish_and_wait(EntityFreezeFrameCapture & capture, const FaultEvent & event) {
    if (!wait_for_match()) {
      return false;
    }
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      publisher_->publish(event);
      if (!capture.frames_for(event.fault.fault_code).empty()) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::Node> publisher_node_;
  rclcpp::Publisher<FaultEvent>::SharedPtr publisher_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<Ros2SubscriptionExecutor> sub_exec_;
  std::unique_ptr<FakePlcDataProvider> provider_;
};

}  // namespace

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ConfirmedPluginFaultCapturesEntityValues) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_OVERPRESSURE", {"plc_app"})));

  auto frames = capture.frames_for("PLC_OVERPRESSURE");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("temperature", 0.0), 42.5);
  EXPECT_DOUBLE_EQ(frames[0].values.value("pressure", 0.0), 3.2);
  EXPECT_GT(frames[0].captured_at_ns, 0);
  EXPECT_FALSE(frames[0].startup_catchup);  // live confirm edge, not catch-up
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, NonPluginSourceCapturesNothing) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [](const std::string &) -> DataProvider * {
    return nullptr;  // ROS FQN sources are never plugin-owned
  });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("ROS_FAULT", {"/powertrain/engine/temp_sensor"});
  // Deliver repeatedly so the callback demonstrably ran and filtered it.
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("ROS_FAULT").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, NonConfirmedEventsAreIgnored) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("PLC_UPDATED_ONLY", {"plc_app"});
  event.event_type = FaultEvent::EVENT_UPDATED;
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_UPDATED_ONLY").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, StandingFaultsAreFramedWithoutAnyEvent) {
  // The startup case: the fault confirmed before this object subscribed, so no
  // event will ever arrive for it. The lister supplies it instead.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string &) -> std::optional<json> {
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", 7.0}}})}};
      },
      256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        return {{"PLC_STANDING", {"route_plc_app"}}};
      });

  // No publish at all - the frame must appear from the catch-up alone.
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (capture.frames_for("PLC_STANDING").empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  const auto frames = capture.frames_for("PLC_STANDING");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "route_plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("level", 0.0), 7.0);
  EXPECT_TRUE(frames[0].startup_catchup);  // consumers can tell catch-up frames apart
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, QueuedLiveConfirmDedupesStandingCatchUp) {
  // Both paths carry the same fault code and the confirm is already queued
  // when the catch-up snapshots the queue: the catch-up must skip that code
  // entirely (one plugin read per confirm, no double capture) and the drain
  // loop's live frame is the one that lands.
  std::atomic<bool> published{false};
  std::atomic<bool> standing_sampled{false};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [&standing_sampled](const std::string & entity_id) -> std::optional<json> {
        double level = 0.0;
        if (entity_id == "route_live_app") {
          level = 99.0;
        } else if (entity_id == "route_standing_app") {
          level = 1.0;
          standing_sampled.store(true);
        } else {
          return std::nullopt;
        }
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", level}}})}};
      },
      256,
      [&published](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        // Hold the catch-up until the live confirm has reached the queue so
        // the standing entry and the queued event genuinely overlap.
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!published.load() && std::chrono::steady_clock::now() < deadline) {
          std::this_thread::sleep_for(10ms);
        }
        std::this_thread::sleep_for(500ms);  // let the event reach the queue
        return {{"PLC_BOTH_PATHS", {"route_standing_app"}}};
      });

  ASSERT_TRUE(wait_for_match());
  const auto live = make_confirmed_event("PLC_BOTH_PATHS", {"route_live_app"});
  publisher_->publish(live);
  published.store(true);

  const auto deadline = std::chrono::steady_clock::now() + 10s;
  std::vector<EntityFreezeFrameCapture::Frame> frames;
  while (std::chrono::steady_clock::now() < deadline) {
    frames = capture.frames_for("PLC_BOTH_PATHS");
    if (!frames.empty()) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  // The queued confirm dedupes the catch-up: the standing entity was never
  // sampled and the only frame is the live one.
  EXPECT_FALSE(standing_sampled.load());
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "route_live_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("level", 0.0), 99.0);
  EXPECT_FALSE(frames[0].startup_catchup);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ThrowingListerDoesNotKillCaptureThread) {
  // The lister runs on the capture thread's entry path: an escaping exception
  // there is std::terminate. It must be swallowed, and the drain loop must
  // still capture live confirms afterwards.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      nullptr, 256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        throw std::runtime_error("fault services exploded");
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_AFTER_THROW", {"plc_app"})));
  EXPECT_FALSE(capture.frames_for("PLC_AFTER_THROW").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DestructionAbortsBlockedLister) {
  // A lister stuck waiting for the fault services must not hold the
  // destructor's join for its full timeout: should_abort turns true on stop.
  std::atomic<bool> entered{false};
  std::atomic<bool> abort_seen{false};
  auto capture = std::make_unique<EntityFreezeFrameCapture>(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      nullptr, 256,
      [&](const std::function<bool()> & should_abort) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        entered.store(true);
        while (!should_abort()) {  // stands in for the bounded service wait
          std::this_thread::sleep_for(10ms);
        }
        abort_seen.store(true);
        return {};
      });

  const auto entry_deadline = std::chrono::steady_clock::now() + 15s;
  while (!entered.load() && std::chrono::steady_clock::now() < entry_deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(entered.load());

  const auto start = std::chrono::steady_clock::now();
  capture.reset();
  EXPECT_TRUE(abort_seen.load());
  EXPECT_LT(std::chrono::steady_clock::now() - start, 5s);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ListerRunsOnlyAfterEventsPublisherMatched) {
  // ~/events is reliable but volatile on both ends: a confirm published after
  // the standing snapshot but before the reader matches would be missed by
  // both paths. The catch-up must therefore wait for a matched publisher
  // before calling the lister.
  publisher_.reset();                  // no events publisher exists yet
  std::this_thread::sleep_for(250ms);  // let the graph forget it
  std::atomic<bool> publisher_created{false};
  std::atomic<bool> lister_ran{false};
  std::atomic<bool> saw_publisher{false};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      nullptr, 256,
      [&](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        saw_publisher.store(publisher_created.load());
        lister_ran.store(true);
        return {};
      });

  // Ample time for an unordered catch-up to have run before any publisher.
  std::this_thread::sleep_for(500ms);
  publisher_created.store(true);
  publisher_ = publisher_node_->create_publisher<FaultEvent>("/fault_manager/events", rclcpp::QoS(100).reliable());

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!lister_ran.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(10ms);
  }
  ASSERT_TRUE(lister_ran.load());
  EXPECT_TRUE(saw_publisher.load());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, CatchUpCapsStoredFramesAtMaxFaults) {
  // max_faults = 2. The no-data fault must not consume the cap (nothing was
  // stored for it), and the fault past the cap must be skipped instead of
  // FIFO-evicting the catch-up's own earliest frame.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string & entity_id) -> std::optional<json> {
        if (entity_id == "route_no_data_app") {
          return json{{"connected", false}};  // dead row: no frame stored
        }
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", 7.0}}})}};
      },
      /*max_faults=*/2,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        return {{"PLC_CAP_NO_DATA", {"route_no_data_app"}},
                {"PLC_CAP_A", {"route_a"}},
                {"PLC_CAP_B", {"route_b"}},
                {"PLC_CAP_C", {"route_c"}}};
      });

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (capture.frames_for("PLC_CAP_B").empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  std::this_thread::sleep_for(300ms);  // would be enough for an uncapped C capture
  EXPECT_TRUE(capture.frames_for("PLC_CAP_NO_DATA").empty());
  EXPECT_FALSE(capture.frames_for("PLC_CAP_A").empty());  // not evicted by C
  EXPECT_FALSE(capture.frames_for("PLC_CAP_B").empty());
  EXPECT_TRUE(capture.frames_for("PLC_CAP_C").empty());  // past the cap
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, FramesRetainedAcrossClearAndOverwrittenOnReconfirm) {
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [this](const std::string & entity_id) -> DataProvider * {
    return entity_id == "plc_app" ? provider_.get() : nullptr;
  });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_CYCLING", {"plc_app"})));
  ASSERT_DOUBLE_EQ(capture.frames_for("PLC_CYCLING")[0].values.value("temperature", 0.0), 42.5);

  // Clearing retains the confirmed-state record, mirroring the fault_manager's
  // freeze-frame retention across clear_fault.
  auto cleared = make_confirmed_event("PLC_CYCLING", {"plc_app"});
  cleared.event_type = FaultEvent::EVENT_CLEARED;
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(cleared);
    std::this_thread::sleep_for(10ms);
  }
  auto retained = capture.frames_for("PLC_CYCLING");
  ASSERT_EQ(retained.size(), 1u);
  EXPECT_DOUBLE_EQ(retained[0].values.value("temperature", 0.0), 42.5);

  // The next confirm re-samples the plugin, replacing the retained frame, so
  // a new occurrence never serves the previous incident's values past confirm.
  provider_->set_temperature(99.0);
  const auto reconfirm = make_confirmed_event("PLC_CYCLING", {"plc_app"});
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool overwritten = false;
  while (std::chrono::steady_clock::now() < deadline && !overwritten) {
    publisher_->publish(reconfirm);
    std::this_thread::sleep_for(20ms);
    auto latest = capture.frames_for("PLC_CYCLING");
    overwritten = !latest.empty() && std::abs(latest[0].values.value("temperature", 0.0) - 99.0) < 1e-9;
  }
  EXPECT_TRUE(overwritten);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, RouteFallbackCapturesWhenPluginHasNoDataProvider) {
  // Mirror the commercial PLC bridges: no DataProvider, values only through
  // the plugin's x-plc-data route (dispatched in-process by the fetcher).
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string & entity_id) -> std::optional<json> {
        if (entity_id != "route_plc_app") {
          return std::nullopt;
        }
        return json{{"connected", true},
                    {"items", json::array({{{"name", "level"}, {"value", 87.5}, {"unit", "mm"}},
                                           {{"name", "alarm"}, {"value", true}}})}};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_ROUTE_LEVEL_HIGH", {"route_plc_app"})));

  auto frames = capture.frames_for("PLC_ROUTE_LEVEL_HIGH");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "route_plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("level", 0.0), 87.5);
  EXPECT_EQ(frames[0].values.value("alarm", false), true);
  EXPECT_GT(frames[0].captured_at_ns, 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, RouteFallbackSkipsDisconnectedPlcWithNoValues) {
  // A disconnected PLC with nothing cached must not freeze-frame a row of
  // nulls. One that still serves its last known values does get a frame - see
  // DisconnectedEntityWithLastKnownValuesIsCaptured.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string &) -> std::optional<json> {
        return json{{"connected", false}, {"items", json::array({{{"name", "level"}, {"value", nullptr}}})}};
      });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("PLC_DISCONNECTED", {"route_plc_app"});
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_DISCONNECTED").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DisconnectedEntityWithLastKnownValuesIsCaptured) {
  // The loss-of-comms case: the bridge reports the link down and still serves
  // the values it last read. That is what the fault is about, so it is frozen,
  // with the payload's link flag and own timestamp carried as provenance.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [](const std::string &) -> std::optional<json> {
        return json{{"connected", false},
                    {"items", json::array({{{"name", "level"}, {"value", 42.0}}})},
                    {"timestamp", 1234567890}};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_COMMS_LOST", {"route_plc_app"})));

  const auto frames = capture.frames_for("PLC_COMMS_LOST");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].values["level"], 42.0);
  ASSERT_TRUE(frames[0].connected.has_value());
  EXPECT_FALSE(*frames[0].connected);
  EXPECT_EQ(frames[0].source_timestamp, 1234567890);
  // Which path read the values. This capture had no DataProvider and went
  // through the route fallback, so the frame must name that path. The
  // DataProvider flavour of the same case asserts the other constant, which is
  // what stops the two from being swapped at their call sites unnoticed.
  EXPECT_EQ(frames[0].source, EntityFreezeFrameCapture::kSourceXPlcDataRoute);
  // The wire value itself, not just the symbol: swapping what the two constants
  // hold is an API break for every consumer of x-medkit.source, and comparing
  // symbol against symbol would not see it.
  EXPECT_EQ(frames[0].source, "plugin_x_plc_data_route");
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DataProviderWinsOverRouteFallback) {
  std::atomic<bool> fetcher_called{false};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      [&fetcher_called](const std::string &) -> std::optional<json> {
        fetcher_called = true;
        return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", 1.0}}})}};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_PROVIDER_FIRST", {"plc_app"})));

  auto frames = capture.frames_for("PLC_PROVIDER_FIRST");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_DOUBLE_EQ(frames[0].values.value("temperature", 0.0), 42.5);  // provider values, not route
  EXPECT_FALSE(fetcher_called.load());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DataProviderWithoutLiveValuesCapturesNothing) {
  // Same "no row = nothing captured" invariant as the route path: an empty
  // items array (cold poll cache) or all-null values must not freeze-frame a
  // row of {} / nulls, connected or not.
  StaticContentDataProvider cold_provider("cold_app", json{{"items", json::array()}});
  StaticContentDataProvider null_provider("null_app",
                                          json{{"connected", false}, {"items", json::array({{{"id", "level"}}})}});
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [&](const std::string & entity_id) -> DataProvider * {
    if (entity_id == "cold_app") {
      return &cold_provider;
    }
    if (entity_id == "null_app") {
      return &null_provider;
    }
    return nullptr;
  });

  ASSERT_TRUE(wait_for_match());
  auto event = make_confirmed_event("PLC_NO_LIVE_DATA", {"cold_app", "null_app"});
  for (int i = 0; i < 25; ++i) {
    publisher_->publish(event);
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_NO_LIVE_DATA").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, DisconnectedDataProviderWithLastKnownValuesIsCaptured) {
  // DataProvider flavour of the loss-of-comms case: list_data flags the link
  // down but still serves the last known values - captured, with the link
  // state on the frame.
  StaticContentDataProvider down_provider(
      "down_app", json{{"connected", false}, {"items", json::array({{{"id", "level"}, {"value", 5.5}}})}});
  EntityFreezeFrameCapture capture(node_.get(), *sub_exec_, [&](const std::string & entity_id) -> DataProvider * {
    return entity_id == "down_app" ? &down_provider : nullptr;
  });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_PROVIDER_COMMS_LOST", {"down_app"})));

  const auto frames = capture.frames_for("PLC_PROVIDER_COMMS_LOST");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].values["level"], 5.5);
  ASSERT_TRUE(frames[0].connected.has_value());
  EXPECT_FALSE(*frames[0].connected);
  EXPECT_TRUE(frames[0].source_timestamp.is_null());  // provider content has no timestamp field
  // The provider path names itself, and the route path (same case, above) names
  // the other constant: the pair is what makes a swap of the two call sites
  // visible. The literal pins the wire value the API reference documents.
  EXPECT_EQ(frames[0].source, EntityFreezeFrameCapture::kSourceDataProvider);
  EXPECT_EQ(frames[0].source, "plugin_data_provider");
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ComponentFramesFromItsHostedEntitiesViaRoute) {
  // The route-path flavour: the resolver below returns nullptr for EVERY
  // entity, so the component and both hosted apps are read through the
  // x-plc-data fetcher. That payload reports the link flag and its own
  // timestamp, which is why the frames here carry them. A bridge whose apps
  // do export a DataProvider takes the other path and carries neither - see
  // ComponentFramesHostedAppsThroughTheirDataProviders, which is the shipped
  // OPC UA shape.
  //
  // Common to both: the loss-of-comms fault is reported by the PLC runtime
  // component, which reads nothing of its own, so the frames come from the
  // apps it hosts - one entry per hosted app, named after the app, and none
  // for the component.
  std::atomic<int> component_reads{0};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;  // neither the component nor its apps export one
      },
      [&component_reads](const std::string & entity_id) -> std::optional<json> {
        if (entity_id == "plc_runtime") {
          component_reads.fetch_add(1);
          return std::nullopt;  // the route is app-only; a component gets an error
        }
        if (entity_id == "load_process") {
          return json{{"connected", false},
                      {"items", json::array({{{"name", "level"}, {"value", 42.0}}})},
                      {"timestamp", 1234567890}};
        }
        if (entity_id == "aux_process") {
          return json{{"connected", false}, {"items", json::array({{{"name", "flow"}, {"value", 7.5}}})}};
        }
        return std::nullopt;
      },
      // store / known_code_lister: this fixture does not persist.
      256, nullptr, nullptr, nullptr,
      [](const std::string & source_id) -> std::vector<std::string> {
        return source_id == "plc_runtime" ? std::vector<std::string>{"load_process", "aux_process"}
                                          : std::vector<std::string>{};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_COMMS_LOST_HOSTED", {"plc_runtime"})));

  const auto frames = capture.frames_for("PLC_COMMS_LOST_HOSTED");
  // Both hosted apps, and no entry for the component itself (nothing to read).
  ASSERT_EQ(frames.size(), 2u);
  EXPECT_EQ(frames[0].entity_id, "load_process");
  EXPECT_EQ(frames[0].values["level"], 42.0);
  ASSERT_TRUE(frames[0].connected.has_value());
  EXPECT_FALSE(*frames[0].connected);
  EXPECT_EQ(frames[0].source_timestamp, 1234567890);
  EXPECT_EQ(frames[0].source, EntityFreezeFrameCapture::kSourceXPlcDataRoute);
  EXPECT_EQ(frames[1].entity_id, "aux_process");
  EXPECT_EQ(frames[1].values["flow"], 7.5);
  ASSERT_TRUE(frames[1].connected.has_value());
  EXPECT_FALSE(*frames[1].connected);
  EXPECT_EQ(frames[1].source, EntityFreezeFrameCapture::kSourceXPlcDataRoute);
  // The component was read first. Hosting is the fallback, not the first move.
  EXPECT_GT(component_reads.load(), 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ComponentFramesHostedAppsThroughTheirDataProviders) {
  // The shipped OPC UA shape, and the reason a frame's link flag cannot be
  // promised: an app with node-map entries is data-bearing, so the gateway's
  // resolver hands back the plugin's DataProvider for it and nullptr for the
  // component (which has no data points of its own). list_data returns items
  // and nothing else, so the frames name their path and their capture time
  // and carry no link flag, even though the same values fetched over the
  // x-plc-data route would report connected: false - which the fetcher below
  // does, and which the provider path deliberately never consults.
  StaticContentDataProvider load_provider(
      "load_process", json{{"items", json::array({{{"id", "level"}, {"name", "Level"}, {"value", 42.0}},
                                                  {{"id", "status_word"}, {"name", "Status Word"}, {"value", 5}}})}});
  StaticContentDataProvider aux_provider(
      "aux_process", json{{"items", json::array({{{"id", "flow"}, {"name", "Flow"}, {"value", 7.5}}})}});
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [&load_provider, &aux_provider](const std::string & entity_id) -> DataProvider * {
        if (entity_id == "load_process") {
          return &load_provider;
        }
        if (entity_id == "aux_process") {
          return &aux_provider;
        }
        return nullptr;  // the component holds no data points, so no provider
      },
      [](const std::string & entity_id) -> std::optional<json> {
        if (entity_id == "plc_runtime") {
          return std::nullopt;  // the route is app-only, a component gets an error
        }
        // Reachable only if the provider path is skipped: the same values the
        // route would serve, link flag included.
        return json{{"connected", false},
                    {"items", json::array({{{"name", "level"}, {"value", 42.0}}})},
                    {"timestamp", 1234567890}};
      },
      // store / known_code_lister: this fixture does not persist.
      256, nullptr, nullptr, nullptr,
      [](const std::string & source_id) -> std::vector<std::string> {
        return source_id == "plc_runtime" ? std::vector<std::string>{"load_process", "aux_process"}
                                          : std::vector<std::string>{};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_COMMS_LOST_PROVIDER", {"plc_runtime"})));

  const auto frames = capture.frames_for("PLC_COMMS_LOST_PROVIDER");
  ASSERT_EQ(frames.size(), 2u);

  EXPECT_EQ(frames[0].entity_id, "load_process");
  EXPECT_EQ(frames[0].values["level"], 42.0);
  EXPECT_EQ(frames[0].values["status_word"], 5);
  EXPECT_EQ(frames[0].source, EntityFreezeFrameCapture::kSourceDataProvider);
  EXPECT_EQ(frames[0].source, "plugin_data_provider");
  EXPECT_GT(frames[0].captured_at_ns, 0);
  // What the docs may not promise: list_data reports no link flag and no
  // timestamp, so neither reaches x-medkit. Adding them to the plugin's
  // list_data payload is a separate change - every list_data consumer reads it.
  EXPECT_FALSE(frames[0].connected.has_value());
  EXPECT_TRUE(frames[0].source_timestamp.is_null());

  EXPECT_EQ(frames[1].entity_id, "aux_process");
  EXPECT_EQ(frames[1].values["flow"], 7.5);
  EXPECT_EQ(frames[1].source, EntityFreezeFrameCapture::kSourceDataProvider);
  EXPECT_FALSE(frames[1].connected.has_value());
  EXPECT_TRUE(frames[1].source_timestamp.is_null());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AlarmOnlyHostedAppContributesNoFrame) {
  // Not every hosted app has values. An App that exists only to host alarm
  // events (an event_alarms entry, or the auto_alarms fallback) is registered
  // under the component but owns no data points, so no DataProvider resolves
  // for it and its x-plc-data route answers 404. It contributes nothing, and
  // the count is what says so: one frame, from the app that does have values.
  std::atomic<int> alarm_app_reads{0};
  StaticContentDataProvider load_provider(
      "load_process", json{{"items", json::array({{{"id", "level"}, {"name", "Level"}, {"value", 42.0}}})}});
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [&load_provider](const std::string & entity_id) -> DataProvider * {
        // has_data() is false for an alarm-only app, so the manager hands back
        // no provider for it, exactly as it hands back none for the component.
        return entity_id == "load_process" ? &load_provider : nullptr;
      },
      [&alarm_app_reads](const std::string & entity_id) -> std::optional<json> {
        if (entity_id == "plc_runtime_alarms") {
          alarm_app_reads.fetch_add(1);
        }
        // The component's route answers 400 and the alarm-only app's answers
        // 404 (no data mapped), both of which reach the capture as nullopt.
        return std::nullopt;
      },
      // store / known_code_lister: this fixture does not persist.
      256, nullptr, nullptr, nullptr,
      [](const std::string & source_id) -> std::vector<std::string> {
        return source_id == "plc_runtime" ? std::vector<std::string>{"load_process", "plc_runtime_alarms"}
                                          : std::vector<std::string>{};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_COMMS_LOST_ALARM_HOST", {"plc_runtime"})));

  const auto frames = capture.frames_for("PLC_COMMS_LOST_ALARM_HOST");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "load_process");
  EXPECT_EQ(frames[0].values["level"], 42.0);
  for (const auto & frame : frames) {
    EXPECT_NE(frame.entity_id, "plc_runtime_alarms");
  }
  // The alarm-only app was read and yielded nothing, rather than never being
  // reached: without this a resolver that dropped it would pass too.
  EXPECT_GT(alarm_app_reads.load(), 0);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ComponentWithOwnProviderKeepsItsSingleFrame) {
  // A component that serves its own values is unchanged: one entry, read from
  // its DataProvider, and no descent into what it hosts. The hosted resolver
  // here names an entity the route fetcher would gladly serve, so a wrong
  // descent shows up as a second frame rather than as nothing at all.
  StaticContentDataProvider comp_provider(
      "plc_runtime", json{{"connected", false}, {"items", json::array({{{"id", "uptime"}, {"value", 900}}})}});
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [&comp_provider](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_runtime" ? &comp_provider : nullptr;
      },
      [](const std::string & entity_id) -> std::optional<json> {
        if (entity_id != "load_process") {
          return std::nullopt;
        }
        return json{{"connected", false}, {"items", json::array({{{"name", "level"}, {"value", 42.0}}})}};
      },
      // store / known_code_lister: this fixture does not persist.
      256, nullptr, nullptr, nullptr,
      [](const std::string &) -> std::vector<std::string> {
        return {"load_process"};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_COMMS_LOST_OWN_DATA", {"plc_runtime"})));

  const auto own = capture.frames_for("PLC_COMMS_LOST_OWN_DATA");
  ASSERT_EQ(own.size(), 1u);
  EXPECT_EQ(own[0].entity_id, "plc_runtime");
  EXPECT_EQ(own[0].values["uptime"], 900);
  EXPECT_EQ(own[0].source, EntityFreezeFrameCapture::kSourceDataProvider);

  // Positive control on this same capture: a source that reads nothing DOES
  // descend into what it hosts, so the single frame above is the provider
  // winning, not the hosted path being inert.
  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_COMMS_LOST_NO_DATA", {"other_runtime"})));
  const auto hosted = capture.frames_for("PLC_COMMS_LOST_NO_DATA");
  ASSERT_EQ(hosted.size(), 1u);
  EXPECT_EQ(hosted[0].entity_id, "load_process");
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AppFaultKeepsItsSingleOwnFrame) {
  // An app fault is untouched by the hosted fallback: the app reads its own
  // values, so the resolver's answer (an entity the route fetcher serves)
  // must not add a second frame.
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      [](const std::string & entity_id) -> std::optional<json> {
        if (entity_id != "load_process") {
          return std::nullopt;
        }
        return json{{"connected", false}, {"items", json::array({{{"name", "level"}, {"value", 42.0}}})}};
      },
      // store / known_code_lister: this fixture does not persist.
      256, nullptr, nullptr, nullptr,
      [](const std::string &) -> std::vector<std::string> {
        return {"load_process"};
      });

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_APP_OWN_FRAME", {"plc_app"})));

  const auto frames = capture.frames_for("PLC_APP_OWN_FRAME");
  ASSERT_EQ(frames.size(), 1u);
  EXPECT_EQ(frames[0].entity_id, "plc_app");
  EXPECT_DOUBLE_EQ(frames[0].values.value("temperature", 0.0), 42.5);
  EXPECT_EQ(frames[0].source, EntityFreezeFrameCapture::kSourceDataProvider);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, OldestFaultEvictedPastMaxFaults) {
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      nullptr, /*max_faults=*/2);

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_A", {"plc_app"})));
  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_B", {"plc_app"})));
  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_C", {"plc_app"})));

  EXPECT_TRUE(capture.frames_for("PLC_EVICT_A").empty());  // FIFO-evicted
  EXPECT_FALSE(capture.frames_for("PLC_EVICT_B").empty());
  EXPECT_FALSE(capture.frames_for("PLC_EVICT_C").empty());
}

// ===========================================================================
// Persistence: the frame outlives the process, so a restart serves what was
// frozen at fault time instead of re-reading the plant as it is now.
// ===========================================================================

namespace {

/// One stored row, shaped as the capture writes them.
StoredEntityFreezeFrame make_stored_row(const std::string & fault_code, const std::string & entity_id,
                                        int64_t captured_at_ns, double level = 7.0,
                                        const std::string & capture_origin = "") {
  StoredEntityFreezeFrame row;
  row.fault_code = fault_code;
  row.entity_id = entity_id;
  row.frame = json{{"values", {{"level", level}}}, {"connected", false}, {"source_timestamp", "2026-09-08T17:51:40Z"}};
  row.captured_at_ns = captured_at_ns;
  row.source = EntityFreezeFrameCapture::kSourceXPlcDataRoute;
  row.capture_origin = capture_origin;
  return row;
}

/// Store that counts what the capture asks of it, so a test can pin the ORDER
/// of a replacement. A stale frame must be swapped out by one write, never
/// erased first and re-taken afterwards if the plant happens to answer.
class CountingEntityFreezeFrameStore : public ros2_medkit_gateway::EntityFreezeFrameStore {
 public:
  tl::expected<void, std::string> replace_frames(const std::string & fault_code,
                                                 const std::vector<StoredEntityFreezeFrame> & frames) override {
    replaces_.fetch_add(1);
    return inner_.replace_frames(fault_code, frames);
  }

  tl::expected<void, std::string> erase_frames(const std::string & fault_code) override {
    erases_.fetch_add(1);
    return inner_.erase_frames(fault_code);
  }

  tl::expected<std::vector<StoredEntityFreezeFrame>, std::string> load_all() override {
    return inner_.load_all();
  }

  /// Forget the writes the test itself made while seeding.
  void reset_counts() {
    replaces_.store(0);
    erases_.store(0);
  }

  int replaces() const {
    return replaces_.load();
  }
  int erases() const {
    return erases_.load();
  }

 private:
  InMemoryEntityFreezeFrameStore inner_;
  std::atomic<int> replaces_{0};
  std::atomic<int> erases_{0};
};

/// Route fetcher that serves a fixed level and records which entities it read,
/// so a test can prove the plant was NOT re-read for a fault that already has
/// a frame.
class CountingRouteFetcher {
 public:
  explicit CountingRouteFetcher(double level) : level_(level) {
  }

  std::optional<json> operator()(const std::string & entity_id) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reads_[entity_id] += 1;
    }
    return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", level_}}})}};
  }

  int reads(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = reads_.find(entity_id);
    return it == reads_.end() ? 0 : it->second;
  }

 private:
  double level_;
  std::mutex mutex_;
  std::map<std::string, int> reads_;
};

}  // namespace

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, CaptureWritesTheFrameThroughToTheStore) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      nullptr, 256, nullptr, store);

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_PERSIST", {"plc_app"})));
  const auto served = capture.frames_for("PLC_PERSIST");
  ASSERT_EQ(served.size(), 1u);

  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_EQ((*rows)[0].fault_code, "PLC_PERSIST");
  EXPECT_EQ((*rows)[0].entity_id, "plc_app");
  EXPECT_EQ((*rows)[0].captured_at_ns, served[0].captured_at_ns);
  EXPECT_EQ((*rows)[0].frame["values"], served[0].values);
  EXPECT_EQ((*rows)[0].source, EntityFreezeFrameCapture::kSourceDataProvider);
  EXPECT_EQ((*rows)[0].capture_origin, "");  // captured on the confirm edge
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ReConfirmOverwritesTheStoredRow) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      nullptr, 256, nullptr, store);

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_RECONFIRM", {"plc_app"})));

  // The store must follow the map: today's semantics are one frame per fault,
  // so a stale row would resurrect the previous occurrence's values on restart.
  provider_->set_temperature(99.0);
  const auto reconfirm = make_confirmed_event("PLC_RECONFIRM", {"plc_app"});
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  bool overwritten = false;
  while (std::chrono::steady_clock::now() < deadline && !overwritten) {
    publisher_->publish(reconfirm);
    std::this_thread::sleep_for(20ms);
    auto rows = store->load_all();
    overwritten = rows.has_value() && rows->size() == 1u &&
                  std::abs((*rows)[0].frame["values"].value("temperature", 0.0) - 99.0) < 1e-9;
  }
  EXPECT_TRUE(overwritten);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AReloadedFrameIsServedExactlyAsItWasCaptured) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  json first_wire;
  int64_t captured_at_ns = 0;
  {
    CountingRouteFetcher fetcher(41.0);
    EntityFreezeFrameCapture capture(
        node_.get(), *sub_exec_,
        [](const std::string &) -> DataProvider * {
          return nullptr;
        },
        [&fetcher](const std::string & entity_id) {
          return fetcher(entity_id);
        },
        256, nullptr, store);
    ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_RESTART", {"route_plc_app"})));
    const auto frames = capture.frames_for("PLC_RESTART");
    ASSERT_EQ(frames.size(), 1u);
    captured_at_ns = frames[0].captured_at_ns;
    first_wire = FaultHandlers::merge_entity_freeze_frames(json{{"snapshots", json::array()}}, frames);
  }

  // A second gateway life on the same store, with the plant now reading
  // something else entirely: the served frame must still be the frozen one.
  CountingRouteFetcher moved_on(999.0);
  EntityFreezeFrameCapture restarted(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [&moved_on](const std::string & entity_id) {
        return moved_on(entity_id);
      },
      256, nullptr, store);

  const auto reloaded = restarted.frames_for("PLC_RESTART");
  ASSERT_EQ(reloaded.size(), 1u);
  EXPECT_EQ(reloaded[0].captured_at_ns, captured_at_ns);
  EXPECT_FALSE(reloaded[0].startup_catchup);
  const auto second_wire = FaultHandlers::merge_entity_freeze_frames(json{{"snapshots", json::array()}}, reloaded);
  EXPECT_EQ(second_wire, first_wire);  // byte for byte, marker included
  ASSERT_EQ(second_wire["snapshots"].size(), 1u);
  EXPECT_FALSE(second_wire["snapshots"][0].contains("capture_origin"));
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, CatchUpSkipsAReloadedCodeAndFramesOneWithoutARow) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(
      store->replace_frames("PLC_HAS_FRAME", {make_stored_row("PLC_HAS_FRAME", "route_stored_app", 4242)}).has_value());

  CountingRouteFetcher fetcher(7.0);
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [&fetcher](const std::string & entity_id) {
        return fetcher(entity_id);
      },
      256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        return {{"PLC_HAS_FRAME", {"route_stored_app"}}, {"PLC_NO_FRAME", {"route_fresh_app"}}};
      },
      store);

  // Positive control on the same harness: a standing fault with no stored row
  // still gets its startup frame, so an empty PLC_HAS_FRAME below would be a
  // broken catch-up rather than a working skip.
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (capture.frames_for("PLC_NO_FRAME").empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  const auto fresh = capture.frames_for("PLC_NO_FRAME");
  ASSERT_EQ(fresh.size(), 1u);
  EXPECT_TRUE(fresh[0].startup_catchup);

  const auto stored = capture.frames_for("PLC_HAS_FRAME");
  ASSERT_EQ(stored.size(), 1u);
  EXPECT_EQ(stored[0].captured_at_ns, 4242);  // the frozen moment, not this start
  EXPECT_FALSE(stored[0].startup_catchup);
  EXPECT_EQ(fetcher.reads("route_stored_app"), 0);  // the plant was never re-read for it
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, ReloadDropsAFaultTheManagerNoLongerHoldsAndKeepsAClearedOne) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_GONE", {make_stored_row("PLC_GONE", "route_a", 100)}).has_value());
  ASSERT_TRUE(store->replace_frames("PLC_CLEARED", {make_stored_row("PLC_CLEARED", "route_b", 200)}).has_value());

  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      nullptr, 256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        return {};  // nothing standing: only the reload and the prune run
      },
      store,
      // The fault manager reports the cleared fault and knows nothing of the
      // other: its own store was replaced under ours.
      [](const std::function<bool()> &) -> std::optional<std::unordered_set<std::string>> {
        return std::unordered_set<std::string>{"PLC_CLEARED"};
      });

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!capture.frames_for("PLC_GONE").empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_TRUE(capture.frames_for("PLC_GONE").empty());
  // A cleared fault keeps its frame: the gateway's retention across a clear is
  // exactly what persisting it is meant to preserve.
  EXPECT_FALSE(capture.frames_for("PLC_CLEARED").empty());
  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_EQ((*rows)[0].fault_code, "PLC_CLEARED");
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AnUnanswerableKnownFaultListerDropsNothing) {
  // Absence assertion, controlled by the test above: the same seeding with a
  // lister that CAN answer drops PLC_GONE, so "nothing dropped" here is the
  // "could not tell" rule and not a prune that never runs.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_GONE", {make_stored_row("PLC_GONE", "route_a", 100)}).has_value());
  ASSERT_TRUE(store->replace_frames("PLC_CLEARED", {make_stored_row("PLC_CLEARED", "route_b", 200)}).has_value());

  std::atomic<bool> asked{false};
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      nullptr, 256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        return {};
      },
      store,
      [&asked](const std::function<bool()> &) -> std::optional<std::unordered_set<std::string>> {
        asked.store(true);
        return std::nullopt;  // fault manager unreachable
      });

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (!asked.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_TRUE(asked.load());
  std::this_thread::sleep_for(300ms);  // would be enough for a prune to land
  EXPECT_FALSE(capture.frames_for("PLC_GONE").empty());
  EXPECT_FALSE(capture.frames_for("PLC_CLEARED").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, TheRetainedFrameBoundCountsReloadedFrames) {
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  ASSERT_TRUE(store->replace_frames("PLC_LOADED_A", {make_stored_row("PLC_LOADED_A", "route_a", 100)}).has_value());
  ASSERT_TRUE(store->replace_frames("PLC_LOADED_B", {make_stored_row("PLC_LOADED_B", "route_b", 200)}).has_value());

  const auto standing = [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
    return {{"PLC_FRESH", {"route_fresh"}}};
  };
  CountingRouteFetcher fetcher(7.0);
  const auto route = [&fetcher](const std::string & entity_id) {
    return fetcher(entity_id);
  };
  const auto no_provider = [](const std::string &) -> DataProvider * {
    return nullptr;
  };

  {
    // Bound of 2, already met by the two reloaded frames: catching PLC_FRESH up
    // would FIFO-evict one of them, so it is refused instead.
    EntityFreezeFrameCapture capped(node_.get(), *sub_exec_, no_provider, route, /*max_faults=*/2, standing, store);
    std::this_thread::sleep_for(1s);  // enough for an uncapped catch-up to land
    EXPECT_TRUE(capped.frames_for("PLC_FRESH").empty());
    EXPECT_FALSE(capped.frames_for("PLC_LOADED_A").empty());
    EXPECT_FALSE(capped.frames_for("PLC_LOADED_B").empty());
  }

  // Positive control on the same harness: one more slot and the same catch-up
  // frames it.
  EntityFreezeFrameCapture roomy(node_.get(), *sub_exec_, no_provider, route, /*max_faults=*/3, standing, store);
  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (roomy.frames_for("PLC_FRESH").empty() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_FALSE(roomy.frames_for("PLC_FRESH").empty());
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AnEvictedFaultLosesItsStoredRowToo) {
  // Otherwise the bound holds only within a process: an evicted frame would
  // come back on the next start and the file would grow without a limit.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [this](const std::string & entity_id) -> DataProvider * {
        return entity_id == "plc_app" ? provider_.get() : nullptr;
      },
      nullptr, /*max_faults=*/1, nullptr, store);

  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_FIRST", {"plc_app"})));
  ASSERT_TRUE(publish_and_wait(capture, make_confirmed_event("PLC_EVICT_SECOND", {"plc_app"})));

  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_EQ((*rows)[0].fault_code, "PLC_EVICT_SECOND");
}

namespace {

/// A capture whose store already holds one frame for PLC_REOCCUR, taken at
/// `stored_at_ns` with level 10.0, against a plant that now reads 99.0. The
/// standing lister reports the fault with `first_occurred_ns`, which is what
/// decides whether the stored frame belongs to the occurrence being served.
struct ReoccurrenceHarness {
  std::shared_ptr<CountingEntityFreezeFrameStore> store = std::make_shared<CountingEntityFreezeFrameStore>();
  CountingRouteFetcher plant{99.0};
  static constexpr int64_t kStoredAtNs = 1'000'000'000'000'000'000;
};

/// Route fetcher whose entity never answers: the plugin-unreachable shape, and
/// the one the whole feature exists for (a restart while the link is down).
class UnreachableRouteFetcher {
 public:
  std::optional<json> operator()(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    reads_[entity_id] += 1;
    return std::nullopt;
  }

  int reads(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = reads_.find(entity_id);
    return it == reads_.end() ? 0 : it->second;
  }

 private:
  std::mutex mutex_;
  std::map<std::string, int> reads_;
};

/// Standing lister reporting one fault whose occurrence began after the frame
/// the store holds for it.
EntityFreezeFrameCapture::StandingFaultLister reoccurred_lister(const std::string & code,
                                                                const std::vector<std::string> & sources) {
  return [code, sources](const std::function<bool()> &) {
    EntityFreezeFrameCapture::StandingFault fault;
    fault.fault_code = code;
    fault.reporting_sources = sources;
    fault.first_occurred_ns = ReoccurrenceHarness::kStoredAtNs + 60'000'000'000;
    return std::vector<EntityFreezeFrameCapture::StandingFault>{fault};
  };
}

/// Standing lister over an explicit list, so a test can fix the reply's order
/// and each fault's occurrence start independently.
EntityFreezeFrameCapture::StandingFaultLister
listed_faults(std::vector<EntityFreezeFrameCapture::StandingFault> faults) {
  return [faults](const std::function<bool()> &) {
    return faults;
  };
}

EntityFreezeFrameCapture::StandingFault standing_fault(const std::string & code, const std::vector<std::string> & srcs,
                                                       int64_t first_occurred_ns) {
  EntityFreezeFrameCapture::StandingFault fault;
  fault.fault_code = code;
  fault.reporting_sources = srcs;
  fault.first_occurred_ns = first_occurred_ns;
  return fault;
}

/// Route fetcher that answers for every entity except the named ones, so one
/// entity out of several can be the unreachable one.
class SelectiveRouteFetcher {
 public:
  SelectiveRouteFetcher(double level, std::set<std::string> unreachable)
    : level_(level), unreachable_(std::move(unreachable)) {
  }

  std::optional<json> operator()(const std::string & entity_id) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reads_[entity_id] += 1;
    }
    if (unreachable_.count(entity_id) != 0) {
      return std::nullopt;
    }
    return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", level_}}})}};
  }

  int reads(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = reads_.find(entity_id);
    return it == reads_.end() ? 0 : it->second;
  }

 private:
  double level_;
  std::set<std::string> unreachable_;
  std::mutex mutex_;
  std::map<std::string, int> reads_;
};

/// Route fetcher whose named entity fails its first N calls and answers after
/// that: a link that comes back between two reads inside one catch-up.
class FlakyRouteFetcher {
 public:
  FlakyRouteFetcher(double level, std::string flaky, int failures)
    : level_(level), flaky_(std::move(flaky)), failures_left_(failures) {
  }

  std::optional<json> operator()(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    reads_[entity_id] += 1;
    if (entity_id == flaky_ && failures_left_ > 0) {
      --failures_left_;
      return std::nullopt;
    }
    return json{{"connected", true}, {"items", json::array({{{"name", "level"}, {"value", level_}}})}};
  }

  int reads(const std::string & entity_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = reads_.find(entity_id);
    return it == reads_.end() ? 0 : it->second;
  }

 private:
  double level_;
  std::string flaky_;
  int failures_left_;
  std::mutex mutex_;
  std::map<std::string, int> reads_;
};

/// Fault codes present in the store, sorted, for a whole-store assertion.
std::vector<std::string> stored_codes(const std::shared_ptr<InMemoryEntityFreezeFrameStore> & store) {
  auto rows = store->load_all();
  std::vector<std::string> codes;
  if (rows) {
    for (const auto & row : *rows) {
      codes.push_back(row.fault_code);
    }
  }
  std::sort(codes.begin(), codes.end());
  return codes;
}

// The bound probe both tests below run. Two slots, one live reloaded frame
// (PLC_KEEP, the older of the two so it is the FIFO front), one stale reloaded
// frame (PLC_STALE) and one standing fault with no frame at all (PLC_NEW).
constexpr int64_t kKeepAtNs = ReoccurrenceHarness::kStoredAtNs;
constexpr int64_t kStaleAtNs = ReoccurrenceHarness::kStoredAtNs + 10'000'000'000;

std::vector<EntityFreezeFrameCapture::StandingFault> bound_probe_standing() {
  return {standing_fault("PLC_NEW", {"route_new"}, kStaleAtNs + 30'000'000'000),
          standing_fault("PLC_STALE", {"route_stale"}, kStaleAtNs + 60'000'000'000),  // after its frame, so stale
          standing_fault("PLC_KEEP", {"route_keep"}, kKeepAtNs - 60'000'000'000)};    // before its frame, so live
}

void seed_bound_probe(const std::shared_ptr<InMemoryEntityFreezeFrameStore> & store) {
  ASSERT_TRUE(
      store->replace_frames("PLC_KEEP", {make_stored_row("PLC_KEEP", "route_keep", kKeepAtNs, 10.0)}).has_value());
  ASSERT_TRUE(
      store->replace_frames("PLC_STALE", {make_stored_row("PLC_STALE", "route_stale", kStaleAtNs, 11.0)}).has_value());
}

}  // namespace

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AReloadedFrameFromAnEarlierOccurrenceIsReReadAndMarkedStartup) {
  // The fault cleared and confirmed again while the gateway was down, so the
  // stored frame holds the PREVIOUS incident's values. Serving it unmarked
  // would present last week's numbers as this occurrence's.
  ReoccurrenceHarness h;
  ASSERT_TRUE(
      h.store->replace_frames("PLC_REOCCUR", {make_stored_row("PLC_REOCCUR", "route_stored_app", h.kStoredAtNs, 10.0)})
          .has_value());
  h.store->reset_counts();

  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [&h](const std::string & entity_id) {
        return h.plant(entity_id);
      },
      256, reoccurred_lister("PLC_REOCCUR", {"route_stored_app"}), h.store);

  const auto deadline = std::chrono::steady_clock::now() + 15s;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto now = capture.frames_for("PLC_REOCCUR");
    if (!now.empty() && std::abs(now[0].values.value("level", 0.0) - 99.0) < 1e-9) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }
  const auto served = capture.frames_for("PLC_REOCCUR");
  ASSERT_EQ(served.size(), 1u);
  EXPECT_DOUBLE_EQ(served[0].values.value("level", 0.0), 99.0);  // this occurrence, not the last one
  EXPECT_TRUE(served[0].startup_catchup);                        // read at start, so it says so
  EXPECT_GT(served[0].captured_at_ns, ReoccurrenceHarness::kStoredAtNs);

  auto rows = h.store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_DOUBLE_EQ((*rows)[0].frame["values"].value("level", 0.0), 99.0);  // replaced on disk too
  EXPECT_EQ((*rows)[0].capture_origin, "startup");

  // The order, not just the outcome: the plant is read first and the row is
  // then swapped by ONE write. Erasing first and re-taking afterwards leaves
  // the fault with nothing whenever the entity cannot answer, which is exactly
  // the case this feature is for.
  EXPECT_EQ(h.store->erases(), 0);
  EXPECT_EQ(h.store->replaces(), 1);
  EXPECT_EQ(h.plant.reads("route_stored_app"), 1);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AStaleFrameWhoseEntityCannotBeReReadIsDroppedWithAWarning) {
  // The headline case: the gateway restarts while the PLC link is down, and the
  // fault re-confirmed in the meantime. The stored frame is from the previous
  // occurrence so it must not be served, and the re-read cannot replace it, so
  // the fault ends with no frame. That is a real loss of evidence and the log
  // is the only place the operator can learn about it.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  UnreachableRouteFetcher plant;
  ASSERT_TRUE(store
                  ->replace_frames("PLC_LINK_DOWN", {make_stored_row("PLC_LINK_DOWN", "route_stored_app",
                                                                     ReoccurrenceHarness::kStoredAtNs, 10.0)})
                  .has_value());

  testing::internal::CaptureStderr();
  {
    EntityFreezeFrameCapture capture(
        node_.get(), *sub_exec_,
        [](const std::string &) -> DataProvider * {
          return nullptr;
        },
        [&plant](const std::string & entity_id) {
          return plant(entity_id);
        },
        256, reoccurred_lister("PLC_LINK_DOWN", {"route_stored_app"}), store);

    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline && plant.reads("route_stored_app") == 0) {
      std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(300ms);             // let the drop that follows the failed read land
    EXPECT_GE(plant.reads("route_stored_app"), 1);  // the re-read WAS attempted
    EXPECT_TRUE(capture.frames_for("PLC_LINK_DOWN").empty());
  }
  const auto logs = testing::internal::GetCapturedStderr();

  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  EXPECT_TRUE(rows->empty());  // and the row is gone, not left to be served
  EXPECT_NE(logs.find("PLC_LINK_DOWN"), std::string::npos) << logs;
  EXPECT_NE(logs.find("route_stored_app"), std::string::npos) << logs;
  EXPECT_NE(logs.find("no freeze-frame"), std::string::npos) << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AStaleFrameWhoseFaultNamesNoEntityIsDroppedWithAWarning) {
  // The two eligibility tests have to agree. A standing fault with no reporting
  // sources is stale to the comparison and unreadable to the catch-up, so it is
  // a re-read that cannot even be attempted, and it gets the same treatment and
  // the same line rather than disappearing quietly.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  CountingRouteFetcher plant{99.0};
  ASSERT_TRUE(store
                  ->replace_frames("PLC_NO_ENTITY", {make_stored_row("PLC_NO_ENTITY", "route_stored_app",
                                                                     ReoccurrenceHarness::kStoredAtNs, 10.0)})
                  .has_value());

  testing::internal::CaptureStderr();
  {
    EntityFreezeFrameCapture capture(
        node_.get(), *sub_exec_,
        [](const std::string &) -> DataProvider * {
          return nullptr;
        },
        [&plant](const std::string & entity_id) {
          return plant(entity_id);
        },
        256, reoccurred_lister("PLC_NO_ENTITY", {}), store);

    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline && !capture.frames_for("PLC_NO_ENTITY").empty()) {
      std::this_thread::sleep_for(20ms);
    }
    EXPECT_TRUE(capture.frames_for("PLC_NO_ENTITY").empty());
    EXPECT_EQ(plant.reads("route_stored_app"), 0);  // nothing to read, and none was invented
  }
  const auto logs = testing::internal::GetCapturedStderr();

  auto rows = store->load_all();
  ASSERT_TRUE(rows.has_value());
  EXPECT_TRUE(rows->empty());
  EXPECT_NE(logs.find("PLC_NO_ENTITY"), std::string::npos) << logs;
  EXPECT_NE(logs.find("no entity to read"), std::string::npos) << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AReloadedFrameFromTheSameOccurrenceIsKeptUnmarked) {
  // Control for the test above on the same harness: the fault never cleared, so
  // its first_occurred predates the frame and the stored values are the ones
  // this occurrence froze. Re-reading the plant here is the whole defect.
  ReoccurrenceHarness h;
  ASSERT_TRUE(
      h.store->replace_frames("PLC_REOCCUR", {make_stored_row("PLC_REOCCUR", "route_stored_app", h.kStoredAtNs, 10.0)})
          .has_value());

  EntityFreezeFrameCapture capture(
      node_.get(), *sub_exec_,
      [](const std::string &) -> DataProvider * {
        return nullptr;
      },
      [&h](const std::string & entity_id) {
        return h.plant(entity_id);
      },
      256,
      [](const std::function<bool()> &) -> std::vector<EntityFreezeFrameCapture::StandingFault> {
        EntityFreezeFrameCapture::StandingFault fault;
        fault.fault_code = "PLC_REOCCUR";
        fault.reporting_sources = {"route_stored_app"};
        fault.first_occurred_ns = ReoccurrenceHarness::kStoredAtNs - 60'000'000'000;  // a minute before the frame
        return std::vector<EntityFreezeFrameCapture::StandingFault>{fault};
      },
      h.store);

  std::this_thread::sleep_for(1s);  // enough for a catch-up read to land
  const auto served = capture.frames_for("PLC_REOCCUR");
  ASSERT_EQ(served.size(), 1u);
  EXPECT_DOUBLE_EQ(served[0].values.value("level", 0.0), 10.0);
  EXPECT_EQ(served[0].captured_at_ns, ReoccurrenceHarness::kStoredAtNs);
  EXPECT_FALSE(served[0].startup_catchup);
  EXPECT_EQ(h.plant.reads("route_stored_app"), 0);  // the plant was never re-read

  auto rows = h.store->load_all();
  ASSERT_TRUE(rows.has_value());
  ASSERT_EQ(rows->size(), 1u);
  EXPECT_DOUBLE_EQ((*rows)[0].frame["values"].value("level", 0.0), 10.0);  // row untouched
  EXPECT_EQ((*rows)[0].captured_at_ns, ReoccurrenceHarness::kStoredAtNs);
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AStaleReReadAtTheBoundNeverCostsALiveFrame) {
  // Two slots, both taken by reloaded frames, and a third standing fault with
  // none. The stale one owns its slot and its replacement cannot exceed the
  // bound, so it must be re-read in place. The bare one has no slot, so it goes
  // unframed - and must not be let in against an occupancy that counts the
  // stale code as absent, because the FIFO would then evict PLC_KEEP, which is
  // a live frame for a fault that is still standing.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  seed_bound_probe(store);
  SelectiveRouteFetcher plant(99.0, {});

  testing::internal::CaptureStderr();
  {
    EntityFreezeFrameCapture capture(
        node_.get(), *sub_exec_,
        [](const std::string &) -> DataProvider * {
          return nullptr;
        },
        [&plant](const std::string & entity_id) {
          return plant(entity_id);
        },
        /*max_faults=*/2, listed_faults(bound_probe_standing()), store);

    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline && plant.reads("route_stale") == 0) {
      std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(500ms);  // enough for an unbounded pass to admit PLC_NEW

    const auto keep = capture.frames_for("PLC_KEEP");
    ASSERT_EQ(keep.size(), 1u) << "the live reloaded frame was evicted";
    EXPECT_DOUBLE_EQ(keep[0].values.value("level", 0.0), 10.0);
    EXPECT_EQ(keep[0].captured_at_ns, kKeepAtNs);
    EXPECT_FALSE(keep[0].startup_catchup);

    const auto stale = capture.frames_for("PLC_STALE");
    ASSERT_EQ(stale.size(), 1u);
    EXPECT_DOUBLE_EQ(stale[0].values.value("level", 0.0), 99.0);  // re-read in place
    EXPECT_TRUE(stale[0].startup_catchup);
    EXPECT_GT(stale[0].captured_at_ns, kStaleAtNs);

    EXPECT_TRUE(capture.frames_for("PLC_NEW").empty());  // no slot for it

    EXPECT_EQ(plant.reads("route_keep"), 0);   // a live frame is never re-read
    EXPECT_EQ(plant.reads("route_stale"), 1);  // the stale one is, exactly once
    EXPECT_EQ(plant.reads("route_new"), 0);    // refused before the plugin was touched
  }
  const auto logs = testing::internal::GetCapturedStderr();

  EXPECT_EQ(stored_codes(store), (std::vector<std::string>{"PLC_KEEP", "PLC_STALE"}));
  EXPECT_NE(logs.find("PLC_NEW"), std::string::npos) << logs;  // the truncation names it
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AFailedStaleReReadFreesItsSlotForAFaultWithNoFrame) {
  // Same probe, but the stale code's entity cannot answer. Its row is dropped,
  // which genuinely frees a slot, so the bare fault now fits - and PLC_KEEP is
  // still not the one that pays for it.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  seed_bound_probe(store);
  SelectiveRouteFetcher plant(99.0, {"route_stale"});

  testing::internal::CaptureStderr();
  {
    EntityFreezeFrameCapture capture(
        node_.get(), *sub_exec_,
        [](const std::string &) -> DataProvider * {
          return nullptr;
        },
        [&plant](const std::string & entity_id) {
          return plant(entity_id);
        },
        /*max_faults=*/2, listed_faults(bound_probe_standing()), store);

    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline && capture.frames_for("PLC_NEW").empty()) {
      std::this_thread::sleep_for(20ms);
    }

    const auto keep = capture.frames_for("PLC_KEEP");
    ASSERT_EQ(keep.size(), 1u) << "the live reloaded frame was evicted";
    EXPECT_EQ(keep[0].captured_at_ns, kKeepAtNs);
    EXPECT_FALSE(keep[0].startup_catchup);

    EXPECT_TRUE(capture.frames_for("PLC_STALE").empty());  // dropped, not served

    const auto fresh = capture.frames_for("PLC_NEW");
    ASSERT_EQ(fresh.size(), 1u) << "the freed slot was not reused";
    EXPECT_TRUE(fresh[0].startup_catchup);

    EXPECT_EQ(plant.reads("route_keep"), 0);
    EXPECT_EQ(plant.reads("route_stale"), 1);  // asked once before being dropped, never twice
    EXPECT_EQ(plant.reads("route_new"), 1);
  }
  const auto logs = testing::internal::GetCapturedStderr();

  EXPECT_EQ(stored_codes(store), (std::vector<std::string>{"PLC_KEEP", "PLC_NEW"}));
  EXPECT_NE(logs.find("PLC_STALE"), std::string::npos) << logs;
  EXPECT_NE(logs.find("route_stale"), std::string::npos) << logs;
  EXPECT_NE(logs.find("no freeze-frame"), std::string::npos) << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AStaleCodeIsAskedOnceEvenWithASlotToSpare) {
  // Three slots for three faults, so nothing competes and the bound never
  // speaks. A stale code whose entity cannot answer is dropped, which leaves a
  // fault with no frame and a free slot: exactly the shape in which a second
  // pass would go back and block on the same dead entity all over again.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  seed_bound_probe(store);
  SelectiveRouteFetcher plant(99.0, {"route_stale"});

  testing::internal::CaptureStderr();
  {
    EntityFreezeFrameCapture capture(
        node_.get(), *sub_exec_,
        [](const std::string &) -> DataProvider * {
          return nullptr;
        },
        [&plant](const std::string & entity_id) {
          return plant(entity_id);
        },
        /*max_faults=*/3, listed_faults(bound_probe_standing()), store);

    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline && capture.frames_for("PLC_NEW").empty()) {
      std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(500ms);  // enough for a second pass at route_stale to land

    EXPECT_EQ(plant.reads("route_stale"), 1);  // one blocking read per catch-up, whatever the capacity
    EXPECT_EQ(plant.reads("route_keep"), 0);
    EXPECT_EQ(plant.reads("route_new"), 1);
    EXPECT_TRUE(capture.frames_for("PLC_STALE").empty());  // dropped, and it stays dropped
    ASSERT_EQ(capture.frames_for("PLC_KEEP").size(), 1u);
    EXPECT_EQ(capture.frames_for("PLC_KEEP")[0].captured_at_ns, kKeepAtNs);
    ASSERT_EQ(capture.frames_for("PLC_NEW").size(), 1u);
  }
  const auto logs = testing::internal::GetCapturedStderr();

  EXPECT_EQ(stored_codes(store), (std::vector<std::string>{"PLC_KEEP", "PLC_NEW"}));
  EXPECT_NE(logs.find("PLC_STALE"), std::string::npos) << logs;
}

/// @verifies REQ_INTEROP_088
TEST_F(EntityFreezeFrameCaptureTest, AStaleCodeThatWouldAnswerOnASecondAskIsStillOnlyAskedOnce) {
  // The entity fails once and would answer if asked again. Asking again is what
  // must not happen: the warning has already told the operator the frame was
  // discarded and this occurrence has none, so a frame appearing anyway would
  // make the log a lie, the summary would count a discard that did not stick,
  // and the slot freed by the drop would go back to the code that just lost it
  // instead of to the fault still waiting for one.
  auto store = std::make_shared<InMemoryEntityFreezeFrameStore>();
  seed_bound_probe(store);
  FlakyRouteFetcher plant(99.0, "route_stale", /*failures=*/1);
  // STALE first in the reply, so nothing else can reach the bound before it.
  auto standing = bound_probe_standing();
  std::swap(standing[0], standing[1]);

  testing::internal::CaptureStderr();
  {
    EntityFreezeFrameCapture capture(
        node_.get(), *sub_exec_,
        [](const std::string &) -> DataProvider * {
          return nullptr;
        },
        [&plant](const std::string & entity_id) {
          return plant(entity_id);
        },
        /*max_faults=*/2, listed_faults(standing), store);

    const auto deadline = std::chrono::steady_clock::now() + 15s;
    while (std::chrono::steady_clock::now() < deadline && capture.frames_for("PLC_NEW").empty()) {
      std::this_thread::sleep_for(20ms);
    }
    std::this_thread::sleep_for(500ms);

    EXPECT_EQ(plant.reads("route_stale"), 1);
    EXPECT_TRUE(capture.frames_for("PLC_STALE").empty()) << "the discarded frame came back";
    ASSERT_EQ(capture.frames_for("PLC_KEEP").size(), 1u);
    EXPECT_EQ(capture.frames_for("PLC_KEEP")[0].captured_at_ns, kKeepAtNs);
    ASSERT_EQ(capture.frames_for("PLC_NEW").size(), 1u) << "the freed slot did not reach the waiting fault";
  }
  const auto logs = testing::internal::GetCapturedStderr();

  EXPECT_EQ(stored_codes(store), (std::vector<std::string>{"PLC_KEEP", "PLC_NEW"}));
  EXPECT_NE(logs.find("PLC_STALE"), std::string::npos) << logs;
  EXPECT_NE(logs.find("no freeze-frame"), std::string::npos) << logs;
  // The summary has to describe what actually happened, not what was attempted.
  EXPECT_NE(logs.find("0 reloaded frame(s) from an earlier occurrence re-read, 1 discarded"), std::string::npos)
      << logs;
}

TEST(ContentHasLiveData, GatesOnItemsNotOnTheLinkFlag) {
  using Capture = EntityFreezeFrameCapture;
  EXPECT_TRUE(Capture::content_has_live_data(
      json{{"connected", true}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  // No "connected" field: items alone are enough (plugin-defined shape).
  EXPECT_TRUE(Capture::content_has_live_data(json{{"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  // Disconnected but still serving its last known values: this is the
  // loss-of-comms case, and those values are the point of the frame.
  EXPECT_TRUE(Capture::content_has_live_data(
      json{{"connected", false}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  EXPECT_FALSE(Capture::content_has_live_data(json{{"connected", true}, {"items", json::array()}}));
  EXPECT_FALSE(Capture::content_has_live_data(json{{"connected", true}}));
  EXPECT_FALSE(Capture::content_has_live_data(json::array()));
}

TEST(ContentReportsDisconnected, TriggerPathGateDiscriminatesFromFreezeFramePath) {
  using Capture = EntityFreezeFrameCapture;
  // The loss-of-comms payload: link down, last known values still served.
  const json down_with_values{{"connected", false}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}};
  // The freeze-frame path (content_has_live_data alone) captures it...
  EXPECT_TRUE(Capture::content_has_live_data(down_with_values));
  // ...while the trigger value fetcher (gateway_node) also requires the link
  // up, so it yields nullopt and threshold rules hold state on the outage
  // instead of firing on a frozen number.
  EXPECT_TRUE(Capture::content_reports_disconnected(down_with_values));
  EXPECT_FALSE(Capture::content_reports_disconnected(
      json{{"connected", true}, {"items", json::array({{{"name", "a"}, {"value", 1}}})}}));
  // No flag (DataProvider list_data shape) or a non-boolean flag: not down.
  EXPECT_FALSE(Capture::content_reports_disconnected(json{{"items", json::array()}}));
  EXPECT_FALSE(Capture::content_reports_disconnected(json{{"connected", "no"}}));
  EXPECT_FALSE(Capture::content_reports_disconnected(json::array()));
}

TEST(ValuesHaveData, RejectsEmptyAndAllNull) {
  using Capture = EntityFreezeFrameCapture;
  EXPECT_TRUE(Capture::values_have_data(json{{"a", 1}}));
  EXPECT_TRUE(Capture::values_have_data(json{{"a", nullptr}, {"b", 0.0}}));
  EXPECT_FALSE(Capture::values_have_data(json::object()));
  EXPECT_FALSE(Capture::values_have_data(json{{"a", nullptr}, {"b", nullptr}}));
  EXPECT_FALSE(Capture::values_have_data(json(nullptr)));
}

TEST(ValuesFromListContent, BuildsCompactDictFromItems) {
  json content = {{"items", json::array({{{"id", "a"}, {"value", 1}},
                                         {{"name", "b"}, {"value", "on"}},  // falls back to "name" key
                                         {{"id", "c"}}})}};                 // no value -> null
  auto values = EntityFreezeFrameCapture::values_from_list_content(content);
  EXPECT_EQ(values["a"], 1);
  EXPECT_EQ(values["b"], "on");
  EXPECT_TRUE(values["c"].is_null());
}

TEST(ValuesFromListContent, NonItemsShapeKeptVerbatim) {
  json content = {{"status", "ok"}, {"raw", 7}};
  EXPECT_EQ(EntityFreezeFrameCapture::values_from_list_content(content), content);
}

TEST(ValuesFromListContent, NonStringIdOrNameNeverThrows) {
  // json::value() throws type_error.302 on present-but-non-string keys;
  // plugin content is untrusted, so the builder must be total instead.
  json content = {{"items", json::array({{{"id", 42}, {"name", "b"}, {"value", 2}},  // int id -> name fallback
                                         {{"id", 7}, {"value", 1}},                  // no usable key -> skipped
                                         {{"name", true}}})}};                       // no usable key -> skipped
  json values;
  ASSERT_NO_THROW(values = EntityFreezeFrameCapture::values_from_list_content(content));
  EXPECT_EQ(values, json({{"b", 2}}));
}

TEST(StandingFaultsFromListReply, ParsesWellFormedReply) {
  const json data = {{"faults", json::array({json{{"fault_code", "F1"}, {"reporting_sources", json::array({"a", "b"})}},
                                             json{{"fault_code", "F2"}, {"reporting_sources", json::array()}}})},
                     {"count", 2}};
  const auto standing = EntityFreezeFrameCapture::standing_faults_from_list_reply(data);
  ASSERT_TRUE(standing.has_value());
  ASSERT_EQ(standing->size(), 2u);
  EXPECT_EQ((*standing)[0].fault_code, "F1");
  EXPECT_EQ((*standing)[0].reporting_sources, (std::vector<std::string>{"a", "b"}));
  EXPECT_EQ((*standing)[1].fault_code, "F2");
  EXPECT_TRUE((*standing)[1].reporting_sources.empty());
}

TEST(StandingFaultsFromListReply, FirstOccurredIsReadInSecondsAndKeptInNanoseconds) {
  // The wire carries seconds (fault_msg_conversions), the comparison against a
  // frame's captured_at_ns needs nanoseconds. Anything that is not a positive
  // number leaves 0, which the caller reads as "cannot tell" and never lets
  // cost a stored frame.
  const json data = {
      {"faults",
       json::array({json{{"fault_code", "SECONDS"},
                         {"reporting_sources", json::array({"a"})},
                         {"first_occurred", 1788948705.5}},
                    json{{"fault_code", "ABSENT"}, {"reporting_sources", json::array({"a"})}},
                    json{{"fault_code", "NOT_A_NUMBER"},
                         {"reporting_sources", json::array({"a"})},
                         {"first_occurred", "yesterday"}},
                    json{{"fault_code", "ZERO"}, {"reporting_sources", json::array({"a"})}, {"first_occurred", 0}},
                    // Past what int64 holds once multiplied out, so the cast
                    // itself is undefined. What the platform then hands back
                    // decides the behaviour, which is the whole problem. On
                    // x86-64 it is INT64_MIN, which the non-positive check in
                    // stale_reloaded_codes happens to reject, so the frame
                    // survives by luck. A saturating target hands back a large
                    // POSITIVE value instead, which passes that check and sits
                    // above every captured_at, so the fault reads as re-occurred
                    // and a good frame is discarded. The range guard is what
                    // makes the outcome the same on both.
                    json{{"fault_code", "HUGE"}, {"reporting_sources", json::array({"a"})}, {"first_occurred", 1e19}},
                    json{{"fault_code", "INFINITE"},
                         {"reporting_sources", json::array({"a"})},
                         {"first_occurred", std::numeric_limits<double>::infinity()}},
                    json{{"fault_code", "NAN_SECONDS"},
                         {"reporting_sources", json::array({"a"})},
                         {"first_occurred", std::numeric_limits<double>::quiet_NaN()}}})}};
  const auto standing = EntityFreezeFrameCapture::standing_faults_from_list_reply(data);
  ASSERT_TRUE(standing.has_value());
  ASSERT_EQ(standing->size(), 7u);
  EXPECT_EQ((*standing)[0].fault_code, "SECONDS");
  EXPECT_EQ((*standing)[0].first_occurred_ns, 1788948705500000000);
  EXPECT_EQ((*standing)[1].first_occurred_ns, 0);  // absent
  EXPECT_EQ((*standing)[2].first_occurred_ns, 0);  // not a number
  EXPECT_EQ((*standing)[3].first_occurred_ns, 0);  // zero
  EXPECT_EQ((*standing)[4].fault_code, "HUGE");
  EXPECT_EQ((*standing)[4].first_occurred_ns, 0);
  EXPECT_EQ((*standing)[5].fault_code, "INFINITE");
  EXPECT_EQ((*standing)[5].first_occurred_ns, 0);
  EXPECT_EQ((*standing)[6].fault_code, "NAN_SECONDS");
  EXPECT_EQ((*standing)[6].first_occurred_ns, 0);
}

TEST(StandingFaultsFromListReply, RepliesNotShapedLikeListFaultsYieldNullopt) {
  // nullopt (vs empty vector) is what lets the caller warn on a malformed or
  // renamed reply instead of silently disabling the catch-up.
  using Capture = EntityFreezeFrameCapture;
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json::array()).has_value());
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json(3)).has_value());
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json(nullptr)).has_value());
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json::object()).has_value());  // no "faults" key
  EXPECT_FALSE(Capture::standing_faults_from_list_reply(json{{"faults", "nope"}}).has_value());
}

TEST(StandingFaultsFromListReply, MalformedItemsAreSkippedNeverThrownOn) {
  const json data = {
      {"faults",
       json::array({"not_an_object", json{{"reporting_sources", json::array({"a"})}},     // missing fault_code
                    json{{"fault_code", 42}, {"reporting_sources", json::array({"a"})}},  // non-string fault_code
                    json{{"fault_code", "NO_SOURCES"}},                                   // missing sources
                    json{{"fault_code", "BAD_SOURCES"}, {"reporting_sources", "x"}},      // non-array sources
                    json{{"fault_code", "MIXED"}, {"reporting_sources", json::array({"ok", 7, true, "also_ok"})}}})}};
  std::optional<std::vector<EntityFreezeFrameCapture::StandingFault>> standing;
  ASSERT_NO_THROW(standing = EntityFreezeFrameCapture::standing_faults_from_list_reply(data));
  ASSERT_TRUE(standing.has_value());
  ASSERT_EQ(standing->size(), 1u);
  EXPECT_EQ((*standing)[0].fault_code, "MIXED");
  EXPECT_EQ((*standing)[0].reporting_sources, (std::vector<std::string>{"ok", "also_ok"}));
}

TEST(MergeEntityFreezeFrames, AppendsWhenNoConfiguredFreezeFrame) {
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};
  frame.captured_at_ns = 1234;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  const auto & snap = merged["snapshots"][0];
  EXPECT_EQ(snap["type"], "freeze_frame");
  EXPECT_EQ(snap["name"], "plc_app");
  EXPECT_EQ(json::parse(snap["data"].get<std::string>())["temperature"], 42.5);
  EXPECT_EQ(snap["captured_at_ns"], 1234);
  EXPECT_FALSE(snap.contains("capture_origin"));  // confirm-edge frames carry no marker
}

TEST(MergeEntityFreezeFrames, CarriesCapturePathAsSource) {
  // An entity frame has no ROS topic, so topic/message_type are necessarily
  // empty, so "source" is the only field left saying where they came from.
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};
  frame.captured_at_ns = 1234;
  frame.source = EntityFreezeFrameCapture::kSourceXPlcDataRoute;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  const auto & snap = merged["snapshots"][0];
  EXPECT_EQ(snap["source"], EntityFreezeFrameCapture::kSourceXPlcDataRoute);
  EXPECT_EQ(snap["topic"], "");
  EXPECT_EQ(snap["message_type"], "");
}

TEST(MergeEntityFreezeFrames, OmitsSourceForAFrameThatNamesNoPath) {
  // A merge-helper contract, not a control for the capture tests: both capture
  // paths always name themselves (asserted from real captures in
  // Disconnected{Entity,DataProvider}WithLastKnownValuesIsCaptured), so this
  // frame is one only a caller can build. The helper must then leave the key
  // out rather than invent a provenance the wire consumer would trust.
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  EXPECT_FALSE(merged["snapshots"][0].contains("source"));
}

TEST(MergeEntityFreezeFrames, StartupCatchUpFrameCarriesCaptureOrigin) {
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};
  frame.captured_at_ns = 1234;
  frame.startup_catchup = true;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  EXPECT_EQ(merged["snapshots"][0]["capture_origin"], "startup");
}

TEST(MergeEntityFreezeFrames, ExplicitConfigWins) {
  // A fault_manager-captured freeze-frame (explicit snapshot config) must
  // suppress the zero-config entity frames.
  json env_data = {{"snapshots", json::array({{{"type", "freeze_frame"},
                                               {"snapshot_type", "freeze_frame"},
                                               {"name", "scan"},
                                               {"data", "{}"},
                                               {"topic", "/scan"},
                                               {"message_type", "sensor_msgs/msg/LaserScan"},
                                               {"captured_at_ns", 1}}})}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 1u);
  EXPECT_EQ(merged["snapshots"][0]["name"], "scan");
}

TEST(MergeEntityFreezeFrames, RosbagSnapshotDoesNotSuppressEntityFrames) {
  // Only a freeze-frame counts as "explicit config captured": a rosbag-only
  // environment still gets the entity frame appended.
  json env_data = {
      {"snapshots",
       json::array({{{"type", "rosbag"}, {"snapshot_type", "rosbag"}, {"name", "rosbag_X"}, {"fault_code", "X"}}})}};
  EntityFreezeFrameCapture::Frame frame;
  frame.entity_id = "plc_app";
  frame.values = {{"temperature", 42.5}};

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {frame});
  ASSERT_EQ(merged["snapshots"].size(), 2u);
  EXPECT_EQ(merged["snapshots"][1]["name"], "plc_app");
}

TEST(MergeEntityFreezeFrames, NoFramesNoChange) {
  json env_data = {{"snapshots", json::array()}};
  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {});
  EXPECT_EQ(merged, env_data);
}

TEST(MergeEntityFreezeFrames, CarriesLinkStateAndSourceTimestampWhenKnown) {
  // Loss-of-comms provenance: the payload's link flag and own timestamp ride
  // the snapshot entry through to the wire x-medkit block - and only when the
  // plugin actually reported them.
  json env_data = {{"snapshots", json::array()}};
  EntityFreezeFrameCapture::Frame down_frame;
  down_frame.entity_id = "plc_app";
  down_frame.values = {{"level", 42.0}};
  down_frame.captured_at_ns = 1234;
  down_frame.connected = false;
  down_frame.source_timestamp = 1234567890;
  EntityFreezeFrameCapture::Frame bare_frame;
  bare_frame.entity_id = "other_app";
  bare_frame.values = {{"temperature", 42.5}};
  bare_frame.captured_at_ns = 5678;

  auto merged = FaultHandlers::merge_entity_freeze_frames(env_data, {down_frame, bare_frame});
  ASSERT_EQ(merged["snapshots"].size(), 2u);
  EXPECT_EQ(merged["snapshots"][0]["connected"], false);
  EXPECT_EQ(merged["snapshots"][0]["source_timestamp"], 1234567890);
  EXPECT_FALSE(merged["snapshots"][1].contains("connected"));
  EXPECT_FALSE(merged["snapshots"][1].contains("source_timestamp"));

  // And onto the wire: build_sovd_fault_response surfaces them in x-medkit.
  const auto detail = FaultHandlers::build_sovd_fault_response(
      json{{"fault_code", "PLC_COMMS_LOST"}, {"status", "CONFIRMED"}}, merged, "/apps/plc_app");
  ASSERT_TRUE(detail.environment_data.snapshots.has_value());
  const auto & wire = *detail.environment_data.snapshots;
  ASSERT_EQ(wire.size(), 2u);
  EXPECT_EQ(wire[0]["x-medkit"]["connected"], false);
  EXPECT_EQ(wire[0]["x-medkit"]["source_timestamp"], 1234567890);
  EXPECT_FALSE(wire[1]["x-medkit"].contains("connected"));
  EXPECT_FALSE(wire[1]["x-medkit"].contains("source_timestamp"));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
