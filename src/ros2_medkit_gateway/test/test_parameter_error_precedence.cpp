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
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <regex>
#include <string>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/config_handlers.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/typed_router.hpp"

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::ERR_INTERNAL_ERROR;
using ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND;
using ros2_medkit_gateway::ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE;
using ros2_medkit_gateway::ErrorInfo;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::ConfigHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
namespace http = ros2_medkit_gateway::http;
namespace dto = ros2_medkit_gateway::dto;

namespace {

constexpr const char * kMissingParam = "definitely_missing_param";
constexpr const char * kGetConfigPattern = R"(/api/v1/components/([^/]+)/configurations/([^/]+))";
constexpr const char * kListConfigPattern = R"(/api/v1/components/([^/]+)/configurations)";
constexpr const char * kGhostFqn = "/param_precedence_ghost";

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

}  // namespace

// =============================================================================
// Handler-level regression tests for the multi-node config endpoints, exercised
// end-to-end through ConfigHandlers against a live GatewayNode with real
// per-node failures:
//   - "ghost" node FQN with no ROS node behind it -> SERVICE_UNAVAILABLE
//   - unspun node (parameter services exist, never answered) -> TIMEOUT
//   - the gateway's own node missing the parameter -> NOT_FOUND
//
// GET /{entity}/configurations/{param}: when no backing node succeeds, the
// surfaced error must be the highest-severity per-node failure regardless of
// node iteration order (a NOT_FOUND from one node must never mask an
// unavailable/timeout from another).
//
// GET /{entity}/configurations (list, issue #541): when some backing nodes are
// up and others are down, the 200 response must flag itself partial and name
// the unavailable nodes rather than silently dropping them - matching the
// single-parameter GET's 503 honesty for the same outage.
// =============================================================================

class ParameterErrorPrecedenceTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        // Fail fast on the ghost node's wait_for_service and keep the test
        // runtime bounded.
        rclcpp::Parameter("parameter_service_timeout_sec", 1.0),
        // Disable the negative cache so every probe classifies fresh
        // (TIMEOUT stays TIMEOUT instead of a cached SERVICE_UNAVAILABLE).
        rclcpp::Parameter("parameter_service_negative_cache_sec", 0.0),
    });
    gateway_node_ = std::make_shared<GatewayNode>(options);
    ASSERT_NE(gateway_node_, nullptr);
    // The cache is injected below; a graph-driven refresh would wipe it.
    gateway_node_->stop_discovery_refresh_for_testing();

    // A node whose parameter services exist in the DDS graph but are never
    // spun: parameter requests to it time out instead of failing fast.
    unspun_node_ = std::make_shared<rclcpp::Node>("param_precedence_unspun");

    executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(gateway_node_);
    spin_thread_ = std::thread([this]() {
      executor_->spin();
    });

    ctx_ = std::make_unique<HandlerContext>(gateway_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<ConfigHandlers>(*ctx_);

    wait_for_unspun_node_discovery();
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
    unspun_node_.reset();
    gateway_node_.reset();
  }

  /// Wait until the gateway sees the unspun node's parameter services in the
  /// graph, so its probes deterministically reach the TIMEOUT path (service
  /// discovered, request never answered) instead of SERVICE_UNAVAILABLE.
  void wait_for_unspun_node_discovery() {
    const std::string service = unspun_fqn() + "/get_parameters";
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (std::chrono::steady_clock::now() < deadline) {
      auto services = gateway_node_->get_service_names_and_types();
      if (services.count(service) > 0) {
        return;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    FAIL() << "unspun node parameter service not discovered: " << service;
  }

  std::string self_fqn() const {
    return gateway_node_->get_fully_qualified_name();
  }

  std::string unspun_fqn() const {
    return unspun_node_->get_fully_qualified_name();
  }

  static App make_app(const std::string & id, const std::string & component_id, const std::string & fqn) {
    App app;
    app.id = id;
    app.name = id;
    app.component_id = component_id;
    app.bound_fqn = fqn;
    app.is_online = true;
    return app;
  }

  /// Seed aggregated components whose backing-node order is fixed by app
  /// insertion order (get_component_configurations preserves it), covering
  /// both iteration orders for every scenario.
  void seed_cache() {
    const std::string ghost_fqn = kGhostFqn;

    std::vector<Component> components;
    std::vector<App> apps;

    auto add_component = [&](const std::string & id, const std::vector<std::string> & fqns) {
      Component comp;
      comp.id = id;
      comp.name = id;
      components.push_back(comp);
      for (size_t i = 0; i < fqns.size(); ++i) {
        apps.push_back(make_app(id + "_app" + std::to_string(i), id, fqns[i]));
      }
    };

    // Unavailable node first / last, plus a node that lacks the parameter.
    add_component("stack_unavail_first", {ghost_fqn, self_fqn()});
    add_component("stack_unavail_last", {self_fqn(), ghost_fqn});

    // Three distinct verdicts (TIMEOUT, SERVICE_UNAVAILABLE, NOT_FOUND) in
    // both orders.
    add_component("stack_mixed_timeout_first", {unspun_fqn(), ghost_fqn, self_fqn()});
    add_component("stack_mixed_timeout_last", {self_fqn(), ghost_fqn, unspun_fqn()});

    // Every node reachable but none has the parameter.
    add_component("stack_all_missing", {self_fqn(), self_fqn()});

    auto & cache = const_cast<ThreadSafeEntityCache &>(gateway_node_->get_thread_safe_cache());
    cache.update_all({}, components, apps, {});
  }

  ErrorInfo get_missing_param_error(const std::string & component_id) {
    const std::string path = "/api/v1/components/" + component_id + "/configurations/" + kMissingParam;
    auto raw = make_request_with_match(path, kGetConfigPattern);
    http::TypedRequest req(raw);

    auto result = handlers_->get_configuration(req);
    EXPECT_FALSE(result.has_value()) << "expected an error for " << path;
    return result.has_value() ? ErrorInfo{} : result.error();
  }

  using ConfigListCollection = dto::Collection<dto::ConfigurationMetaData, dto::ConfigListXMedkit>;

  /// Run GET /{entity}/configurations and return the full typed collection.
  /// The call must succeed (list is a 200 even when a backing node is down).
  ConfigListCollection list_collection(const std::string & component_id) {
    const std::string path = "/api/v1/components/" + component_id + "/configurations";
    auto raw = make_request_with_match(path, kListConfigPattern);
    http::TypedRequest req(raw);

    auto result = handlers_->list_configurations(req);
    EXPECT_TRUE(result.has_value()) << "list_configurations should return 200 for " << path;
    return result.has_value() ? result.value() : ConfigListCollection{};
  }

  /// Convenience: just the parsed x-medkit block of the list response.
  dto::ConfigListXMedkit list_xmedkit(const std::string & component_id) {
    auto collection = list_collection(component_id);
    return collection.x_medkit.has_value() ? *collection.x_medkit : dto::ConfigListXMedkit{};
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::shared_ptr<GatewayNode> gateway_node_;
  std::shared_ptr<rclcpp::Node> unspun_node_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<ConfigHandlers> handlers_;
};

// Regression order for the pre-fix overwrite bug: the unavailable node is
// probed first, then the reachable node reports NOT_FOUND last. The old code
// classified only the LAST failure, so this surfaced 404 instead of 503.
TEST_F(ParameterErrorPrecedenceTest, UnavailableNodeFirstNotFoundLastReturns503) {
  auto err = get_missing_param_error("stack_unavail_first");

  EXPECT_EQ(err.http_status, 503);
  EXPECT_EQ(err.code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
}

// Opposite iteration order must surface the same verdict.
TEST_F(ParameterErrorPrecedenceTest, NotFoundFirstUnavailableNodeLastReturns503) {
  auto err = get_missing_param_error("stack_unavail_last");

  EXPECT_EQ(err.http_status, 503);
  EXPECT_EQ(err.code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
}

// Three distinct per-node verdicts (TIMEOUT, SERVICE_UNAVAILABLE, NOT_FOUND):
// a 503-class failure must win over NOT_FOUND, and within the 503 rank the
// first failure probed is the one surfaced in details.
TEST_F(ParameterErrorPrecedenceTest, MixedThreeNodeFailuresTimeoutFirstReturns503) {
  auto err = get_missing_param_error("stack_mixed_timeout_first");

  EXPECT_EQ(err.http_status, 503);
  EXPECT_EQ(err.code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
  // First 503-rank failure probed: the unspun node's TIMEOUT.
  ASSERT_TRUE(err.params.contains("details"));
  EXPECT_NE(err.params["details"].get<std::string>().find("did not respond"), std::string::npos) << err.params.dump();
}

TEST_F(ParameterErrorPrecedenceTest, MixedThreeNodeFailuresTimeoutLastReturns503) {
  auto err = get_missing_param_error("stack_mixed_timeout_last");

  EXPECT_EQ(err.http_status, 503);
  EXPECT_EQ(err.code, ERR_X_MEDKIT_ROS2_NODE_UNAVAILABLE);
  // First 503-rank failure probed: the ghost node's SERVICE_UNAVAILABLE.
  ASSERT_TRUE(err.params.contains("details"));
  EXPECT_NE(err.params["details"].get<std::string>().find("not available"), std::string::npos) << err.params.dump();
}

// When every backing node is reachable and reports NOT_FOUND, the aggregate
// verdict stays a plain 404.
TEST_F(ParameterErrorPrecedenceTest, AllNodesMissingParameterReturns404) {
  auto err = get_missing_param_error("stack_all_missing");

  EXPECT_EQ(err.http_status, 404);
  EXPECT_EQ(err.code, ERR_RESOURCE_NOT_FOUND);
}

// Issue #541: one backing node down and one up. The list endpoint must stay a
// 200 but flag itself partial and name the unavailable node, instead of
// silently returning a shrunken list. Both node orders are covered.
TEST_F(ParameterErrorPrecedenceTest, ListWithUnavailableNodeFirstIsPartial) {
  auto collection = list_collection("stack_unavail_first");
  const auto & xm = collection.x_medkit.value();

  ASSERT_TRUE(xm.partial.has_value());
  EXPECT_TRUE(*xm.partial);
  ASSERT_TRUE(xm.unavailable_nodes.has_value());
  EXPECT_NE(std::find(xm.unavailable_nodes->begin(), xm.unavailable_nodes->end(), kGhostFqn),
            xm.unavailable_nodes->end())
      << "unavailable node list must name the down node";
  // The reachable node's parameters are still merged into the returned list -
  // a partial result must not throw away the nodes that answered.
  EXPECT_FALSE(collection.items.empty()) << "answering node's parameters must survive the partial";
}

TEST_F(ParameterErrorPrecedenceTest, ListWithUnavailableNodeLastIsPartial) {
  auto collection = list_collection("stack_unavail_last");
  const auto & xm = collection.x_medkit.value();

  ASSERT_TRUE(xm.partial.has_value());
  EXPECT_TRUE(*xm.partial);
  ASSERT_TRUE(xm.unavailable_nodes.has_value());
  EXPECT_NE(std::find(xm.unavailable_nodes->begin(), xm.unavailable_nodes->end(), kGhostFqn),
            xm.unavailable_nodes->end());
  // Reachable node comes first in this order, so it is the one that stresses
  // accumulation: assert its parameters survive here too.
  EXPECT_FALSE(collection.items.empty()) << "answering node's parameters must survive the partial";
}

// Control: when every backing node is reachable, the list is not flagged
// partial and no unavailable nodes are reported.
TEST_F(ParameterErrorPrecedenceTest, ListWithAllNodesReachableIsNotPartial) {
  auto xm = list_xmedkit("stack_all_missing");

  EXPECT_FALSE(xm.partial.has_value() && *xm.partial);
  EXPECT_FALSE(xm.unavailable_nodes.has_value());
}

// When NO backing node answers, the list fails with the worst per-node
// verdict instead of returning an empty partial 200. Here the manager is shut
// down so every node returns SHUT_DOWN and the surfaced status is 500
// internal-error, mirroring GET, rather than mislabelling the nodes
// "unavailable" behind a partial 200.
//
// The complementary case - one node failing while another answers - now keeps
// the answering node's parameters and stays a 200 partial (asserted by the
// ListWith*NodeIsPartial cases above via collection.items), so the old
// 200->500 loud-drop no longer exists. That mixed case cannot be pinned more
// tightly here because the live-node harness has no per-node non-503 fault
// hook (shutdown is manager-wide; the ghost/unspun nodes only yield 503).
TEST_F(ParameterErrorPrecedenceTest, ListWithHardErrorFailsRequestNotPartial) {
  gateway_node_->get_configuration_manager()->shutdown();

  const std::string path = "/api/v1/components/stack_all_missing/configurations";
  auto raw = make_request_with_match(path, kListConfigPattern);
  http::TypedRequest req(raw);

  auto result = handlers_->list_configurations(req);
  ASSERT_FALSE(result.has_value()) << "hard per-node error must fail the list, not return a partial 200";
  EXPECT_EQ(result.error().http_status, 500);
  EXPECT_EQ(result.error().code, ERR_INTERNAL_ERROR);
}
