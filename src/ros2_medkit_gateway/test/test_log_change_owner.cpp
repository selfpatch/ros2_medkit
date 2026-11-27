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

// "logs" changes as a plugin sees them through PluginContext: a real
// GatewayNode in hybrid discovery, real bound nodes and real /rosout entries.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <rcl_interfaces/msg/log.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/managers/trigger_manager.hpp"
#include "ros2_medkit_gateway/core/resource_change_notifier.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/plugins/ros_plugin_context.hpp"

using namespace std::chrono_literals;
using ros2_medkit_gateway::NotifierFilter;
using ros2_medkit_gateway::NotifierSubscriptionId;
using ros2_medkit_gateway::ResourceChange;
using ros2_medkit_gateway::ResourceChangeNotifier;
using ros2_medkit_gateway::TriggerCreateRequest;

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

// engine-temp-app is bound to a root node, rpm-app to a namespaced one. The
// app ids differ from the node names, so an id taken from a logger name never
// equals them. gauge is a root node whose app id is its node name.
constexpr const char * kManifest = R"(
manifest_version: "1.0"
metadata:
  name: log-change-owner
  version: "0.0.1"
config:
  unmanifested_nodes: ignore
areas:
  - id: vehicle
    name: Vehicle
components:
  - id: ecu
    name: ECU
    area: vehicle
apps:
  - id: engine-temp-app
    name: Engine temperature
    is_located_on: ecu
    ros_binding:
      node_name: engine_temp
      namespace: /
  - id: rpm-app
    name: Engine RPM
    is_located_on: ecu
    ros_binding:
      node_name: rpm
      namespace: /powertrain/engine
  - id: gauge
    name: Gauge
    is_located_on: ecu
    ros_binding:
      node_name: gauge
      namespace: /
)";

constexpr auto kDeliveryTimeout = 20s;

/// Records the "logs" changes one notifier subscription receives.
class Recorder {
 public:
  Recorder(ResourceChangeNotifier & notifier, NotifierFilter filter) : notifier_(notifier) {
    id_ = notifier_.subscribe(std::move(filter), [this](const ResourceChange & change) {
      std::lock_guard<std::mutex> lock(mutex_);
      changes_.push_back(change);
    });
  }
  ~Recorder() {
    notifier_.unsubscribe(id_);
  }
  Recorder(const Recorder &) = delete;
  Recorder & operator=(const Recorder &) = delete;
  Recorder(Recorder &&) = delete;
  Recorder & operator=(Recorder &&) = delete;

  /// The change whose entry message is `message`, if one arrived.
  std::optional<ResourceChange> find(const std::string & message) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & change : changes_) {
      if (change.value.value("message", std::string{}) == message) {
        return change;
      }
    }
    return std::nullopt;
  }

  /// The changes whose entry message starts with `prefix`.
  std::vector<ResourceChange> with_prefix(const std::string & prefix) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ResourceChange> out;
    for (const auto & change : changes_) {
      if (change.value.value("message", std::string{}).rfind(prefix, 0) == 0) {
        out.push_back(change);
      }
    }
    return out;
  }

 private:
  ResourceChangeNotifier & notifier_;
  NotifierSubscriptionId id_{0};
  mutable std::mutex mutex_;
  std::vector<ResourceChange> changes_;
};

class LogChangeOwnerTest : public ::testing::Test {
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
    work_dir_ =
        std::filesystem::temp_directory_path() / ("medkit-log-change-owner-" + std::to_string(::getpid()) + "-" + name);
    std::error_code ec;
    std::filesystem::remove_all(work_dir_, ec);
    std::filesystem::create_directories(work_dir_);
    const auto manifest_path = work_dir_ / "manifest.yaml";
    std::ofstream(manifest_path) << kManifest;

    const int port = reserve_local_port();
    ASSERT_GT(port, 0);
    rclcpp::NodeOptions opts;
    opts.parameter_overrides({
        {"discovery.mode", "hybrid"},
        {"discovery.manifest_path", manifest_path.string()},
        {"discovery.manifest_strict_validation", false},
        {"discovery.merge_pipeline.gap_fill.allow_heuristic_apps", false},
        {"refresh_interval_ms", 1000},
        {"server.host", std::string("127.0.0.1")},
        {"server.port", port},
    });
    gateway_ = std::make_shared<ros2_medkit_gateway::GatewayNode>(opts);
    root_node_ = std::make_shared<rclcpp::Node>("engine_temp", "/");
    ns_node_ = std::make_shared<rclcpp::Node>("rpm", "/powertrain/engine");
    gauge_node_ = std::make_shared<rclcpp::Node>("gauge", "/");
    probe_ = std::make_shared<rclcpp::Node>("log_owner_probe", "/probe");
    rosout_ = probe_->create_publisher<rcl_interfaces::msg::Log>("/rosout", rclcpp::QoS(1000).reliable());

    ctx_ = ros2_medkit_gateway::make_gateway_plugin_context(gateway_.get(), gateway_->get_fault_manager());
    notifier_ = ctx_->get_resource_change_notifier();
    ASSERT_NE(notifier_, nullptr);

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(gateway_);
    executor_->add_node(root_node_);
    executor_->add_node(ns_node_);
    executor_->add_node(gauge_node_);
    executor_->add_node(probe_);
  }

  void TearDown() override {
    recorders_.clear();
    if (executor_) {
      executor_->cancel();
    }
    executor_.reset();
    ctx_.reset();
    rosout_.reset();
    probe_.reset();
    gauge_node_.reset();
    ns_node_.reset();
    root_node_.reset();
    gateway_.reset();
    std::error_code ec;
    std::filesystem::remove_all(work_dir_, ec);
  }

  Recorder & record(NotifierFilter filter) {
    recorders_.push_back(std::make_unique<Recorder>(*notifier_, std::move(filter)));
    return *recorders_.back();
  }

  void publish(const std::string & logger, const std::string & message) {
    rcl_interfaces::msg::Log msg;
    msg.stamp = probe_->now();
    msg.level = rcl_interfaces::msg::Log::INFO;
    msg.name = logger;
    msg.msg = message;
    msg.file = "test_log_change_owner.cpp";
    msg.function = "publish";
    msg.line = 1;
    rosout_->publish(msg);
  }

  /// A multishot logs trigger on an app, with log_settings.
  std::string create_logs_trigger(const std::string & app_id, const std::string & marker) {
    TriggerCreateRequest req;
    req.entity_id = app_id;
    req.entity_type = "apps";
    req.resource_uri = "/api/v1/apps/" + app_id + "/logs";
    req.collection = "logs";
    req.condition_type = "OnChange";
    req.condition_params = nlohmann::json::object();
    req.multishot = true;
    req.lifetime_sec = 120;
    req.log_settings = nlohmann::json{{"severity", "warning"}, {"marker", marker}};
    auto created = gateway_->get_trigger_manager()->create(req);
    EXPECT_TRUE(created.has_value()) << (created ? "" : created.error().message);
    return created ? created->id : std::string{};
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

  /// Publish numbered entries on `logger` until `recorder` gets one; discovery
  /// and linking take a few refresh cycles. Returns the change received.
  std::optional<ResourceChange> publish_until_received(Recorder & recorder, const std::string & logger,
                                                       const std::string & tag) {
    std::optional<ResourceChange> got;
    int attempt = 0;
    auto next_publish = std::chrono::steady_clock::now();
    spin_until(
        [&]() {
          for (int i = 1; i <= attempt && !got; ++i) {
            got = recorder.find(tag + "-" + std::to_string(i));
          }
          if (got) {
            return true;
          }
          if (std::chrono::steady_clock::now() >= next_publish) {
            publish(logger, tag + "-" + std::to_string(++attempt));
            next_publish = std::chrono::steady_clock::now() + 500ms;
          }
          return false;
        },
        kDeliveryTimeout);
    return got;
  }

  std::filesystem::path work_dir_;
  std::shared_ptr<ros2_medkit_gateway::GatewayNode> gateway_;
  std::shared_ptr<rclcpp::Node> root_node_;
  std::shared_ptr<rclcpp::Node> ns_node_;
  std::shared_ptr<rclcpp::Node> gauge_node_;
  std::shared_ptr<rclcpp::Node> probe_;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr rosout_;
  std::unique_ptr<ros2_medkit_gateway::RosPluginContext> ctx_;
  ResourceChangeNotifier * notifier_{nullptr};
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::vector<std::unique_ptr<Recorder>> recorders_;
};

}  // namespace

TEST_F(LogChangeOwnerTest, RootNodeEntriesReachTheBoundAppId) {
  auto & app = record(NotifierFilter{"logs", "engine-temp-app", ""});

  auto got = publish_until_received(app, "engine_temp", "root");

  ASSERT_TRUE(got.has_value()) << "a plugin filtering on engine-temp-app got no entry of logger engine_temp";
  EXPECT_EQ(got->entity_id, "engine-temp-app");
  EXPECT_EQ(got->value["context"]["node"], "engine_temp");
}

TEST_F(LogChangeOwnerTest, SlashFormEntriesReachTheBoundAppId) {
  auto & app = record(NotifierFilter{"logs", "rpm-app", ""});

  auto got = publish_until_received(app, "powertrain/engine/rpm", "slash");

  ASSERT_TRUE(got.has_value()) << "a plugin filtering on rpm-app got no entry of logger powertrain/engine/rpm";
  EXPECT_EQ(got->entity_id, "rpm-app");
  EXPECT_EQ(got->value["context"]["node"], "powertrain/engine/rpm");
}

TEST_F(LogChangeOwnerTest, DotFormEntriesReachTheBoundAppId) {
  auto & app = record(NotifierFilter{"logs", "rpm-app", ""});

  auto got = publish_until_received(app, "powertrain.engine.rpm", "dot");

  ASSERT_TRUE(got.has_value()) << "a plugin filtering on rpm-app got no entry of logger powertrain.engine.rpm";
  EXPECT_EQ(got->entity_id, "rpm-app");
  EXPECT_EQ(got->value["context"]["node"], "powertrain.engine.rpm");
}

TEST_F(LogChangeOwnerTest, LoggerNoAppOwnsCarriesNoEntityId) {
  auto & app = record(NotifierFilter{"logs", "rpm-app", ""});
  auto & all = record(NotifierFilter{"logs", "", ""});
  // Owned by rpm-app; its arrival proves the app-filtered stream is live.
  ASSERT_TRUE(publish_until_received(app, "powertrain.engine.rpm", "live").has_value());

  // A child logger and a logger of no node: GET /apps/rpm-app/logs lists neither.
  publish("powertrain.engine.rpm.worker", "child");
  publish("stranger", "stranger");
  publish("powertrain.engine.rpm", "sentinel");
  ASSERT_TRUE(spin_until(
      [&]() {
        return app.find("sentinel").has_value() && all.find("child").has_value() && all.find("stranger").has_value();
      },
      kDeliveryTimeout));

  EXPECT_FALSE(app.find("child").has_value());
  EXPECT_FALSE(app.find("stranger").has_value());
  EXPECT_EQ(all.find("child")->entity_id, "");
  EXPECT_EQ(all.find("child")->value["context"]["node"], "powertrain.engine.rpm.worker");
  EXPECT_EQ(all.find("stranger")->entity_id, "");
  EXPECT_EQ(all.find("stranger")->value["context"]["node"], "stranger");
}

// A marker a trigger's log_settings writes reaches plugins as one "logs"
// CREATED change: entity_id of the app that owns its logger name (the
// trigger's entity id), or "".
TEST_F(LogChangeOwnerTest, TriggerMarkerReachesPluginsAsOneLogsChange) {
  auto & all = record(NotifierFilter{"logs", "", ""});
  auto & app = record(NotifierFilter{"logs", "engine-temp-app", ""});
  ASSERT_TRUE(publish_until_received(app, "engine_temp", "live").has_value());

  const auto trigger_id = create_logs_trigger("engine-temp-app", "pin-marker");
  ASSERT_FALSE(trigger_id.empty());
  publish("engine_temp", "pin-real");
  ASSERT_TRUE(spin_until(
      [&]() {
        return !all.with_prefix("pin-marker ").empty();
      },
      kDeliveryTimeout))
      << "no marker change reached the plugin";
  publish("engine_temp", "pin-sentinel");
  ASSERT_TRUE(spin_until(
      [&]() {
        return all.find("pin-sentinel").has_value();
      },
      kDeliveryTimeout));
  spin_until(
      [&]() {
        return all.with_prefix("pin-marker ").size() >= 2;
      },
      2s);

  const auto markers = all.with_prefix("pin-marker ");
  ASSERT_EQ(markers.size(), 2u) << "one change per firing: pin-real and pin-sentinel";
  const auto & change = markers.front();
  EXPECT_EQ(change.collection, "logs");
  EXPECT_EQ(change.entity_id, "");
  EXPECT_EQ(change.resource_path, "");
  EXPECT_EQ(change.change_type, ros2_medkit_gateway::ChangeType::CREATED);
  EXPECT_EQ(change.value["severity"], "warning");
  EXPECT_EQ(change.value["context"], nlohmann::json({{"node", "engine-temp-app"}}));
  const nlohmann::json metadata{
      {"trigger_id", trigger_id}, {"condition_type", "OnChange"}, {"resource", "/api/v1/apps/engine-temp-app/logs"}};
  EXPECT_EQ(change.value["message"], "pin-marker " + metadata.dump());
  EXPECT_EQ(change.value["id"].get<std::string>().rfind("log_", 0), 0u);
  EXPECT_TRUE(app.with_prefix("pin-marker ").empty()) << "no app owns the logger engine-temp-app";
  gateway_->get_trigger_manager()->remove(trigger_id);
}

TEST_F(LogChangeOwnerTest, TriggerMarkerUnderAnAppLoggerReachesThatApp) {
  auto & gauge = record(NotifierFilter{"logs", "gauge", ""});
  ASSERT_TRUE(publish_until_received(gauge, "gauge", "live").has_value());

  const auto trigger_id = create_logs_trigger("gauge", "gauge-marker");
  ASSERT_FALSE(trigger_id.empty());
  publish("gauge", "gauge-real");
  ASSERT_TRUE(spin_until(
      [&]() {
        return !gauge.with_prefix("gauge-marker ").empty();
      },
      kDeliveryTimeout))
      << "a plugin filtering on gauge got no marker of the gauge trigger";
  gateway_->get_trigger_manager()->remove(trigger_id);

  const auto change = gauge.with_prefix("gauge-marker ").front();
  EXPECT_EQ(change.entity_id, "gauge");
  EXPECT_EQ(change.change_type, ros2_medkit_gateway::ChangeType::CREATED);
  EXPECT_EQ(change.value["context"], nlohmann::json({{"node", "gauge"}}));
  EXPECT_EQ(change.value["severity"], "warning");
}
