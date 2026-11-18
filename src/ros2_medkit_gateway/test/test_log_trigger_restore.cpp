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

// A persistent logs trigger restored by a real GatewayNode from a real trigger
// store, with real discovery and real /rosout entries.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <memory>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/sqlite_trigger_store.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using namespace std::chrono_literals;
using ros2_medkit_gateway::SqliteTriggerStore;
using ros2_medkit_gateway::TriggerInfo;
using ros2_medkit_gateway::TriggerStatus;

namespace {

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    ADD_FAILURE() << "socket: " << std::strerror(errno);
    return 0;
  }
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    ADD_FAILURE() << "bind: " << std::strerror(errno);
    close(sock);
    return 0;
  }
  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    ADD_FAILURE() << "getsockname: " << std::strerror(errno);
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

// A manifest Component with the id the runtime App of node /dup_node gets.
constexpr const char * kManifest = R"(
manifest_version: "1.0"
metadata:
  name: log-trigger-restore
  version: "0.0.1"
areas:
  - id: vehicle
    name: Vehicle
components:
  - id: dup_node
    name: Component sharing an app id
    area: vehicle
)";

constexpr const char * kTriggerId = "trig_dup";

TriggerInfo persistent_logs_trigger(const std::string & app_id, const std::string & id = kTriggerId) {
  TriggerInfo info;
  info.id = id;
  info.entity_id = app_id;
  info.entity_type = "apps";
  info.resource_uri = "/api/v1/apps/" + app_id + "/logs";
  info.collection = "logs";
  info.condition_type = "OnChange";
  info.condition_params = nlohmann::json::object();
  info.protocol = "sse";
  info.multishot = true;
  info.persistent = true;
  info.status = TriggerStatus::ACTIVE;
  info.created_at = std::chrono::system_clock::now();
  return info;
}

class LogTriggerRestoreTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    const auto * info = ::testing::UnitTest::GetInstance()->current_test_info();
    const std::string name = info ? std::string(info->name()) : std::string("unknown");
    work_dir_ = std::filesystem::temp_directory_path() /
                ("medkit-log-trigger-restore-" + std::to_string(::getpid()) + "-" + name);
    std::error_code ec;
    std::filesystem::remove_all(work_dir_, ec);
    std::filesystem::create_directories(work_dir_);
    const auto manifest_path = work_dir_ / "manifest.yaml";
    std::ofstream(manifest_path) << kManifest;
    db_path_ = work_dir_ / "triggers.db";
    const auto & db_path = db_path_;
    {
      SqliteTriggerStore store(db_path.string());
      for (const auto & trigger : stored_triggers()) {
        ASSERT_TRUE(store.save(trigger).has_value());
      }
    }

    const int port = reserve_local_port();
    ASSERT_GT(port, 0);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"discovery.mode", "hybrid"},
        {"discovery.manifest_path", manifest_path.string()},
        {"discovery.manifest_strict_validation", false},
        {"refresh_interval_ms", 500},
        {"triggers.storage.path", db_path.string()},
        {"triggers.on_restart_behavior", std::string("restore")},
        {"server.host", std::string("127.0.0.1")},
        {"server.port", port},
    });
    gateway_ = std::make_shared<ros2_medkit_gateway::GatewayNode>(opts);
    probe_ = std::make_shared<rclcpp::Node>("log_restore_probe", "/probe");
    rosout_ = probe_->create_publisher<rcl_interfaces::msg::Log>("/rosout", rclcpp::QoS(100).reliable());
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(gateway_);
    executor_->add_node(probe_);
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    executor_.reset();
    app_node_.reset();
    rosout_.reset();
    probe_.reset();
    gateway_.reset();
    std::error_code ec;
    std::filesystem::remove_all(work_dir_, ec);
  }

  /// Triggers in the store before the gateway starts.
  virtual std::vector<TriggerInfo> stored_triggers() const {
    return {persistent_logs_trigger("dup_node")};
  }

  void publish(const std::string & logger, const std::string & message) {
    rcl_interfaces::msg::Log msg;
    msg.stamp = probe_->now();
    msg.level = rcl_interfaces::msg::Log::INFO;
    msg.name = logger;
    msg.msg = message;
    rosout_->publish(msg);
  }

  bool spin_until(const std::function<bool()> & done, std::chrono::nanoseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      executor_->spin_some(50ms);
      if (done()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return done();
  }

  /// The orphan sweep and restore replay the gateway's refresh timer runs, run here.
  void sweep() {
    gateway_->get_trigger_manager()->sweep_orphaned_triggers();
  }

  /// Entries the gateway holds under logger dup_node, markers included.
  size_t buffered() {
    auto items = gateway_->get_log_manager()->get_logs({"/dup_node"}, false, "", "", "dup_node");
    return items ? items->size() : 0;
  }

  /// Moves the pending events of @p trigger_id into events_.
  void drain(const std::string & trigger_id) {
    while (auto event = gateway_->get_trigger_manager()->consume_pending_event(trigger_id)) {
      events_[trigger_id].push_back((*event)["payload"].value("message", ""));
    }
  }

  /// Spins until every trigger in @p trigger_ids has delivered @p message. Events reach a
  /// trigger in log order, so nothing logged before @p message can arrive after it.
  bool spin_until_delivered(const std::vector<std::string> & trigger_ids, const std::string & message) {
    return spin_until(
        [&] {
          bool all = true;
          for (const auto & id : trigger_ids) {
            drain(id);
            const auto & got = events_[id];
            all = all && std::find(got.begin(), got.end(), message) != got.end();
          }
          return all;
        },
        20s);
  }

  std::map<std::string, std::vector<std::string>> events_;

  std::filesystem::path work_dir_;
  std::filesystem::path db_path_;
  std::shared_ptr<ros2_medkit_gateway::GatewayNode> gateway_;
  std::shared_ptr<rclcpp::Node> probe_;
  std::shared_ptr<rclcpp::Node> app_node_;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr rosout_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

constexpr const char * kMarkerTriggerA = "trig_mark_a";
constexpr const char * kMarkerTriggerB = "trig_mark_b";

/// Two restored triggers with log_settings on the root app dup_node. Their
/// markers are logged under "dup_node", which the app's GET lists.
class LogTriggerRestoreMarkerTest : public LogTriggerRestoreTest {
 protected:
  std::vector<TriggerInfo> stored_triggers() const override {
    auto a = persistent_logs_trigger("dup_node", kMarkerTriggerA);
    a.log_settings = nlohmann::json{{"severity", "info"}, {"marker", "mark-a"}};
    auto b = persistent_logs_trigger("dup_node", kMarkerTriggerB);
    b.log_settings = nlohmann::json{{"severity", "info"}, {"marker", "mark-b"}};
    return {a, b};
  }

  /// Messages GET /apps/dup_node/logs lists.
  std::vector<std::string> listed() {
    std::vector<std::string> out;
    auto items = gateway_->get_log_manager()->get_logs({"/dup_node"}, false, "", "", "dup_node");
    if (items) {
      for (const auto & item : *items) {
        out.push_back(item.value("message", ""));
      }
    }
    return out;
  }
};

size_t count_prefix(const std::vector<std::string> & messages, const std::string & prefix) {
  return static_cast<size_t>(std::count_if(messages.begin(), messages.end(), [&](const std::string & m) {
    return m.rfind(prefix, 0) == 0;
  }));
}

}  // namespace

TEST_F(LogTriggerRestoreTest, AppReplayWaitsForTheAppNotForAComponentWithItsId) {
  const auto & cache = gateway_->get_thread_safe_cache();
  auto * triggers = gateway_->get_trigger_manager();
  ASSERT_NE(triggers, nullptr);
  ASSERT_TRUE(triggers->get(kTriggerId).has_value()) << "the trigger was not restored";
  ASSERT_TRUE(spin_until(
      [&] {
        return cache.has_component("dup_node") && rosout_->get_subscription_count() >= 1;
      },
      20s));
  ASSERT_FALSE(cache.has_app("dup_node"));

  // Logged while only the Component carries the id. Sweeps run, and the trigger waits.
  publish("dup_node", "early");
  ASSERT_TRUE(spin_until(
      [&] {
        return buffered() == 1;
      },
      10s))
      << "the gateway did not take the early entry";
  sweep();
  sweep();
  ASSERT_FALSE(cache.has_app("dup_node"));
  drain(kTriggerId);
  EXPECT_TRUE(events_[kTriggerId].empty()) << "replayed for the Component with the app's id";

  app_node_ = std::make_shared<rclcpp::Node>("dup_node", "/");
  executor_->add_node(app_node_);
  ASSERT_TRUE(spin_until(
      [&] {
        return cache.has_app("dup_node");
      },
      20s));

  sweep();
  ASSERT_TRUE(spin_until_delivered({kTriggerId}, "early")) << "the early entry never reached the restored trigger";
  // A sweep after a live entry replays nothing again: an OnChange trigger would fire on a
  // repeated "early", which differs from "later". A last entry closes the check.
  publish("dup_node", "later");
  ASSERT_TRUE(spin_until_delivered({kTriggerId}, "later"));
  sweep();
  publish("dup_node", "last");
  ASSERT_TRUE(spin_until_delivered({kTriggerId}, "last"));
  EXPECT_EQ(events_[kTriggerId], (std::vector<std::string>{"early", "later", "last"}));
}

TEST_F(LogTriggerRestoreTest, TriggerIsAnOrphanOnceItsAppIsGoneAlthoughAComponentKeepsTheId) {
  const auto & cache = gateway_->get_thread_safe_cache();
  auto * triggers = gateway_->get_trigger_manager();
  ASSERT_NE(triggers, nullptr);
  ASSERT_TRUE(triggers->get(kTriggerId).has_value()) << "the trigger was not restored";

  app_node_ = std::make_shared<rclcpp::Node>("dup_node", "/");
  executor_->add_node(app_node_);
  ASSERT_TRUE(spin_until(
      [&] {
        return cache.has_app("dup_node") && cache.has_component("dup_node");
      },
      20s));
  // A sweep while the App is there: the trigger has seen its entity and stays.
  sweep();
  ASSERT_TRUE(triggers->get(kTriggerId).has_value()) << "swept while its App was there";

  executor_->remove_node(app_node_);
  app_node_.reset();
  ASSERT_TRUE(spin_until(
      [&] {
        return !cache.has_app("dup_node");
      },
      20s));
  ASSERT_TRUE(cache.has_component("dup_node"));

  EXPECT_TRUE(spin_until(
      [&] {
        return !triggers->get(kTriggerId).has_value();
      },
      10s))
      << "the App is gone; the Component with its id must not keep the trigger";
  SqliteTriggerStore store(db_path_.string());
  auto rows = store.load_all();
  ASSERT_TRUE(rows.has_value());
  EXPECT_TRUE(std::none_of(rows->begin(), rows->end(), [](const TriggerInfo & row) {
    return row.id == kTriggerId;
  })) << "the store row outlived the orphan";
}

TEST_F(LogTriggerRestoreMarkerTest, MarkersWrittenByTheReplayFireNoTrigger) {
  const auto & cache = gateway_->get_thread_safe_cache();
  auto * triggers = gateway_->get_trigger_manager();
  ASSERT_NE(triggers, nullptr);
  ASSERT_TRUE(triggers->get(kMarkerTriggerA).has_value()) << "trigger a was not restored";
  ASSERT_TRUE(triggers->get(kMarkerTriggerB).has_value()) << "trigger b was not restored";
  ASSERT_TRUE(spin_until(
      [&] {
        return cache.has_component("dup_node") && rosout_->get_subscription_count() >= 1;
      },
      20s));
  publish("dup_node", "early");
  ASSERT_TRUE(spin_until(
      [&] {
        return listed().size() == 1;
      },
      10s))
      << "the gateway did not take the early entry";
  sweep();
  ASSERT_FALSE(cache.has_app("dup_node"));

  app_node_ = std::make_shared<rclcpp::Node>("dup_node", "/");
  executor_->add_node(app_node_);
  ASSERT_TRUE(spin_until(
      [&] {
        return cache.has_app("dup_node");
      },
      20s));
  // The replay writes one marker per trigger. Each marker reaches the notifier like an
  // entry, before the entries logged after it.
  sweep();
  ASSERT_TRUE(spin_until(
      [&] {
        const auto messages = listed();
        return count_prefix(messages, "mark-a ") >= 1 && count_prefix(messages, "mark-b ") >= 1;
      },
      20s))
      << "the replays wrote no markers";

  // A live entry fires each trigger once and writes one marker each; a last entry is
  // delivered after anything the markers could fire.
  publish("dup_node", "late");
  ASSERT_TRUE(spin_until_delivered({kMarkerTriggerA, kMarkerTriggerB}, "late")) << "the live entry fired no trigger";
  publish("dup_node", "done");
  ASSERT_TRUE(spin_until_delivered({kMarkerTriggerA, kMarkerTriggerB}, "done"));
  ASSERT_TRUE(spin_until(
      [&] {
        const auto messages = listed();
        return count_prefix(messages, "mark-a ") >= 3 && count_prefix(messages, "mark-b ") >= 3;
      },
      20s))
      << "a firing wrote no marker";
  const std::vector<std::string> entries{"early", "late", "done"};
  EXPECT_EQ(events_[kMarkerTriggerA], entries);
  EXPECT_EQ(events_[kMarkerTriggerB], entries);
  const auto messages = listed();
  EXPECT_EQ(count_prefix(messages, "mark-a "), 3u);
  EXPECT_EQ(count_prefix(messages, "mark-b "), 3u);
  EXPECT_EQ(messages.size(), 9u) << "three entries plus one marker per firing";
}

// A real entry logged before discovery with the text of trigger a's marker,
// its trigger_id included, under the marker's logger name. The replay takes
// markers by how they were written, so it delivers this one.
TEST_F(LogTriggerRestoreMarkerTest, ReplayDeliversARealEntryThatLooksLikeAMarker) {
  const std::string lookalike = "mark-a " + nlohmann::json{{"condition_type", "OnChange"},
                                                           {"resource", "/api/v1/apps/dup_node/logs"},
                                                           {"trigger_id", kMarkerTriggerA}}
                                                .dump();
  const auto & cache = gateway_->get_thread_safe_cache();
  auto * triggers = gateway_->get_trigger_manager();
  ASSERT_NE(triggers, nullptr);
  ASSERT_TRUE(triggers->get(kMarkerTriggerA).has_value()) << "trigger a was not restored";
  ASSERT_TRUE(triggers->get(kMarkerTriggerB).has_value()) << "trigger b was not restored";
  ASSERT_TRUE(spin_until(
      [&] {
        return cache.has_component("dup_node") && rosout_->get_subscription_count() >= 1;
      },
      20s));
  publish("dup_node", "early");
  publish("dup_node", lookalike);
  ASSERT_TRUE(spin_until(
      [&] {
        return listed().size() == 2;
      },
      10s))
      << "the gateway did not take both early entries";
  ASSERT_FALSE(cache.has_app("dup_node"));

  app_node_ = std::make_shared<rclcpp::Node>("dup_node", "/");
  executor_->add_node(app_node_);
  ASSERT_TRUE(spin_until(
      [&] {
        return cache.has_app("dup_node");
      },
      20s));

  sweep();
  ASSERT_TRUE(spin_until(
      [&] {
        return count_prefix(listed(), "mark-b ") >= 2;
      },
      20s))
      << "the replays wrote no markers";
  // A last entry is delivered after anything the replay's markers could fire.
  publish("dup_node", "done");
  ASSERT_TRUE(spin_until_delivered({kMarkerTriggerA, kMarkerTriggerB}, "done"));
  ASSERT_TRUE(spin_until(
      [&] {
        return count_prefix(listed(), "mark-b ") >= 3;
      },
      20s))
      << "a firing wrote no marker";
  const std::vector<std::string> entries{"early", lookalike, "done"};
  EXPECT_EQ(events_[kMarkerTriggerA], entries);
  EXPECT_EQ(events_[kMarkerTriggerB], entries);
  const auto messages = listed();
  EXPECT_EQ(count_prefix(messages, "mark-b "), 3u);
  EXPECT_EQ(count_prefix(messages, "mark-a "), 4u) << "the look-alike plus one marker per firing of a";
  EXPECT_EQ(messages.size(), 9u);
}
