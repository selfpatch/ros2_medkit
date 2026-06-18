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

#include <httplib.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <regex>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/discovery_handlers.hpp"
#include "ros2_medkit_gateway/core/http/handlers/lifecycle_handlers.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/dto/json_writer.hpp"
#include "ros2_medkit_gateway/dto/lifecycle.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::dto::JsonWriter;
using ros2_medkit_gateway::handlers::DiscoveryHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::LifecycleHandlers;
namespace dto = ros2_medkit_gateway::dto;
namespace http = ros2_medkit_gateway::http;

namespace {

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

}  // namespace

// =============================================================================
// Fixture-based tests against a live GatewayNode.
// =============================================================================

class LifecycleHandlersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    std::vector<std::string> args = {"test_lifecycle_handlers", "--ros-args", "-p", "refresh_interval_ms:=60000"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    gateway_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(gateway_node_, nullptr);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<LifecycleHandlers>(*ctx_, nullptr);  // no plugin manager

    // Give the executor a moment to start before seeding cache.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    seed_cache();
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    handlers_.reset();
    ctx_.reset();
    executor_.reset();
    gateway_node_.reset();
  }

  void seed_cache() {
    // Online app
    App online_app;
    online_app.id = "online_sensor";
    online_app.name = "OnlineSensor";
    online_app.is_online = true;

    // Offline app
    App offline_app;
    offline_app.id = "offline_sensor";
    offline_app.name = "OfflineSensor";
    offline_app.is_online = false;

    // Component with host_metadata (host component -> "ready")
    Component host_comp;
    host_comp.id = "host_controller";
    host_comp.name = "HostController";
    host_comp.host_metadata = json{{"hostname", "robot-main"}};

    // Component without host_metadata (non-host -> "notReady")
    Component non_host_comp;
    non_host_comp.id = "bridge_component";
    non_host_comp.name = "BridgeComponent";
    // host_metadata left as nullopt

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, {host_comp, non_host_comp}, {online_app, offline_app}, {});
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<LifecycleHandlers> handlers_;
};

// GET /apps/{online_sensor}/status -> 200, status == "ready", no restart URI
TEST_F(LifecycleHandlersTest, GetStatusOnlineAppReturnsReady) {
  auto raw = make_request_with_match("/api/v1/apps/online_sensor/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
  EXPECT_FALSE(result->restart.has_value());
}

// GET /apps/{offline_sensor}/status -> 200, status == "notReady"
TEST_F(LifecycleHandlersTest, GetStatusOfflineAppReturnsNotReady) {
  auto raw = make_request_with_match("/api/v1/apps/offline_sensor/status", R"(/api/v1/apps/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "notReady");
}

// GET /components/{host_controller}/status -> 200, status == "ready" (has host_metadata)
TEST_F(LifecycleHandlersTest, GetStatusHostComponentReturnsReady) {
  auto raw =
      make_request_with_match("/api/v1/components/host_controller/status", R"(/api/v1/components/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "ready");
}

// GET /components/{bridge_component}/status -> 200, status == "notReady" (no host_metadata, no provider)
TEST_F(LifecycleHandlersTest, GetStatusNonHostComponentReturnsNotReady) {
  auto raw =
      make_request_with_match("/api/v1/components/bridge_component/status", R"(/api/v1/components/([^/]+)/status)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_get_status(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "notReady");
}

// PUT /apps/{online_sensor}/status/restart -> 501 (no provider registered)
TEST_F(LifecycleHandlersTest, TransitionRestartWithNoProviderReturns501) {
  // Route regex: the action is captured as param "1" = entity_id from /apps/{id}/status/restart
  // We register a route with {app_id} as capture group 1. In the handler, path_param("1") gives entity_id.
  auto raw =
      make_request_with_match("/api/v1/apps/online_sensor/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// PUT /apps/{online_sensor}/status/force-shutdown -> 501 (no provider registered)
TEST_F(LifecycleHandlersTest, TransitionForceShutdownWithNoProviderReturns501) {
  auto raw = make_request_with_match("/api/v1/apps/online_sensor/status/force-shutdown",
                                     R"(/api/v1/apps/([^/]+)/status/force-shutdown)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "force-shutdown");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 501);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// PUT /apps/{unknown}/status/restart -> 404 (validate_entity_for_route before provider)
TEST_F(LifecycleHandlersTest, TransitionUnknownEntityReturns404) {
  auto raw =
      make_request_with_match("/api/v1/apps/does_not_exist/status/restart", R"(/api/v1/apps/([^/]+)/status/restart)");
  http::TypedRequest req(raw);

  auto result = handlers_->handle_transition(req, "restart");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 404);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_ENTITY_NOT_FOUND);
}

// =============================================================================
// Entity detail status link advertisement tests.
// Verify GET /apps/{id} and GET /components/{id} include "status" URI field.
// =============================================================================

namespace {

template <class T>
json detail_body_json(const ros2_medkit_gateway::http::Result<T> & result) {
  if (!result.has_value()) {
    ADD_FAILURE() << "Expected result to have value but it was empty";
    return json{};
  }
  return JsonWriter<T>::write(result.value());
}

}  // namespace

class EntityDetailStatusLinkTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      std::vector<std::string> args = {"test_lifecycle_handlers", "--ros-args", "-p", "refresh_interval_ms:=60000"};
      std::vector<char *> argv;
      argv.reserve(args.size());
      for (auto & arg : args) {
        argv.push_back(arg.data());
      }
      rclcpp::init(static_cast<int>(argv.size()), argv.data());
    }
  }

  static void TearDownTestSuite() {
  }

  void SetUp() override {
    gateway_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(gateway_node_, nullptr);

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    App app;
    app.id = "online_sensor";
    app.name = "OnlineSensor";
    app.is_online = true;

    Component comp;
    comp.id = "host_controller";
    comp.name = "HostController";

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, {comp}, {app}, {});

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    discovery_ = std::make_unique<DiscoveryHandlers>(*ctx_);
  }

  void TearDown() override {
    if (executor_) {
      executor_->cancel();
    }
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    discovery_.reset();
    ctx_.reset();
    executor_.reset();
    gateway_node_.reset();
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<DiscoveryHandlers> discovery_;
};

// GET /apps/{id} must include "status" -> "/api/v1/apps/{id}/status"
TEST_F(EntityDetailStatusLinkTest, AppDetailAdvertisesStatusLink) {
  httplib::Request req;
  req.path = "/api/v1/apps/online_sensor";
  std::regex re(R"(/api/v1/apps/([^/]+))");
  std::regex_match(req.path, req.matches, re);
  http::TypedRequest typed_req(req);

  auto result = discovery_->get_app(typed_req);
  auto body = detail_body_json(result);
  EXPECT_EQ(body["status"], "/api/v1/apps/online_sensor/status");
}

// GET /components/{id} must include "status" -> "/api/v1/components/{id}/status"
TEST_F(EntityDetailStatusLinkTest, ComponentDetailAdvertisesStatusLink) {
  httplib::Request req;
  req.path = "/api/v1/components/host_controller";
  std::regex re(R"(/api/v1/components/([^/]+))");
  std::regex_match(req.path, req.matches, re);
  http::TypedRequest typed_req(req);

  auto result = discovery_->get_component(typed_req);
  auto body = detail_body_json(result);
  EXPECT_EQ(body["status"], "/api/v1/components/host_controller/status");
}
