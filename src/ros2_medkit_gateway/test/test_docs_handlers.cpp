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

#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/config.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/docs_handlers.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"

#include "../src/openapi/route_registry.hpp"

using namespace ros2_medkit_gateway;
using namespace std::chrono_literals;

namespace {

void populate_docs_test_routes(openapi::RouteRegistry & reg) {
  auto noop = [](const httplib::Request &, httplib::Response &) {};
  reg.get("/health", noop).tag("Server").summary("Health check");
  reg.get("/", noop).tag("Server").summary("API overview");
  reg.get("/version-info", noop).tag("Server").summary("SOVD version information");
  for (const auto * et : {"areas", "components", "apps", "functions"}) {
    std::string base = std::string("/") + et;
    reg.get(base, noop).tag("Discovery").summary(std::string("List ") + et);
  }
}

}  // namespace

// =============================================================================
// Test fixture - creates a full GatewayNode for DocsHandlers tests
// =============================================================================

class DocsHandlersTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    node_ = std::make_shared<GatewayNode>();
    std::this_thread::sleep_for(100ms);

    CorsConfig cors_config;
    AuthConfig auth_config;
    TlsConfig tls_config;

    ctx_ = std::make_unique<handlers::HandlerContext>(node_.get(), cors_config, auth_config, tls_config, nullptr);

    route_registry_ = std::make_unique<openapi::RouteRegistry>();
    populate_docs_test_routes(*route_registry_);
  }

  void TearDown() override {
    route_registry_.reset();
    ctx_.reset();
    node_.reset();
  }

  std::shared_ptr<GatewayNode> node_;
  std::unique_ptr<handlers::HandlerContext> ctx_;
  std::unique_ptr<openapi::RouteRegistry> route_registry_;
};

// =============================================================================
// Docs disabled returns 501
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(DocsHandlersTest, DocsDisabledReturns501) {
  // Override docs.enabled to false (GatewayNode already declares it as true by default)
  node_->set_parameter(rclcpp::Parameter("docs.enabled", false));

  handlers::DocsHandlers docs_handlers(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get());

  httplib::Request req;
  httplib::Response res;

  docs_handlers.handle_docs_root(req, res);

  EXPECT_EQ(res.status, 501);

  auto body = nlohmann::json::parse(res.body);
  EXPECT_TRUE(body.contains("error_code"));
}

// =============================================================================
// Docs root returns valid JSON
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(DocsHandlersTest, DocsRootReturnsValidJson) {
  handlers::DocsHandlers docs_handlers(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get());

  httplib::Request req;
  httplib::Response res;

  docs_handlers.handle_docs_root(req, res);

  // send_json does not set res.status (httplib server framework does that),
  // so verify the response body contains a valid OpenAPI spec
  ASSERT_FALSE(res.body.empty());
  auto body = nlohmann::json::parse(res.body);
  EXPECT_EQ(body["openapi"], "3.1.0");
  EXPECT_TRUE(body.contains("info"));
  EXPECT_TRUE(body.contains("paths"));
  EXPECT_TRUE(body.contains("servers"));

  // Root spec should have non-empty paths from the RouteRegistry
  EXPECT_FALSE(body["paths"].empty()) << "Root spec should contain paths from RouteRegistry";
  EXPECT_TRUE(body["paths"].contains("/health"));
  EXPECT_TRUE(body["paths"].contains("/apps"));
}

// =============================================================================
// Entity collection path returns 200 (happy path)
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(DocsHandlersTest, DocsAnyPathReturns200ForEntityCollection) {
  handlers::DocsHandlers docs_handlers(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get());

  httplib::Request req;
  httplib::Response res;

  // Simulate cpp-httplib regex match for /apps collection path.
  // httplib::Match is std::smatch, populated via std::regex_match.
  req.path = "/api/v1/apps/docs";
  std::regex pattern(R"(/api/v1/(.*)/docs)");
  std::smatch match;
  std::regex_match(req.path, match, pattern);
  req.matches = match;

  docs_handlers.handle_docs_any_path(req, res);

  // Entity collection path should generate a valid OpenAPI spec (not 404)
  ASSERT_FALSE(res.body.empty());
  auto body = nlohmann::json::parse(res.body);

  // Should not be an error response
  EXPECT_FALSE(body.contains("error_code")) << "Unexpected error: " << res.body;

  // Should be a valid OpenAPI spec
  EXPECT_EQ(body["openapi"], "3.1.0");
  EXPECT_TRUE(body.contains("paths"));
  EXPECT_TRUE(body["paths"].contains("/apps"));
}

// =============================================================================
// Invalid path returns 404
// =============================================================================

// @verifies REQ_INTEROP_002
TEST_F(DocsHandlersTest, DocsAnyPathReturns404ForInvalidPath) {
  handlers::DocsHandlers docs_handlers(*ctx_, *node_, node_->get_plugin_manager(), route_registry_.get());

  httplib::Request req;
  httplib::Response res;

  // Simulate cpp-httplib regex match: matches[1] = the captured base path.
  // httplib::Match is std::smatch, populated via std::regex_match.
  req.path = "/api/v1/totally_nonexistent_path/docs";
  std::regex pattern(R"(/api/v1/(.*)/docs)");
  std::smatch match;
  std::regex_match(req.path, match, pattern);
  req.matches = match;

  docs_handlers.handle_docs_any_path(req, res);

  EXPECT_EQ(res.status, 404);

  auto body = nlohmann::json::parse(res.body);
  EXPECT_TRUE(body.contains("error_code"));
}
