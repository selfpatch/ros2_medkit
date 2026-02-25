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
#include <string>

#include "ros2_medkit_gateway/http/handlers/health_handlers.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::HealthHandlers;

// HealthHandlers has no dependency on GatewayNode or AuthManager:
// - handle_health only calls HandlerContext::send_json() (static)
// - handle_version_info only calls HandlerContext::send_json() (static)
// - handle_root reads ctx_.auth_config() and ctx_.tls_config() (both disabled by default)
// All tests use a null GatewayNode and null AuthManager which is safe for these handlers.

class HealthHandlersTest : public ::testing::Test {
 protected:
  CorsConfig cors_config_{};
  AuthConfig auth_config_{};  // enabled = false by default
  TlsConfig tls_config_{};    // enabled = false by default
  HandlerContext ctx_{nullptr, cors_config_, auth_config_, tls_config_, nullptr};
  HealthHandlers handlers_{ctx_};

  httplib::Request req_;
  httplib::Response res_;

  HandlerContext make_context(const AuthConfig & auth, const TlsConfig & tls) {
    return HandlerContext(nullptr, cors_config_, auth, tls, nullptr);
  }
};

// --- handle_health ---

TEST_F(HealthHandlersTest, HandleHealthResponseContainsStatusHealthy) {
  handlers_.handle_health(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_EQ(body["status"], "healthy");
}

TEST_F(HealthHandlersTest, HandleHealthResponseContainsTimestamp) {
  handlers_.handle_health(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_TRUE(body.contains("timestamp"));
  EXPECT_TRUE(body["timestamp"].is_number());
}

TEST_F(HealthHandlersTest, HandleHealthResponseIsValidJson) {
  handlers_.handle_health(req_, res_);
  EXPECT_NO_THROW(json::parse(res_.body));
}

// --- handle_version_info ---

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoContainsSovdInfoArray) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  ASSERT_TRUE(body.contains("sovd_info"));
  ASSERT_TRUE(body["sovd_info"].is_array());
  EXPECT_FALSE(body["sovd_info"].empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoSovdEntryHasVersionField) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  auto & entry = body["sovd_info"][0];
  EXPECT_TRUE(entry.contains("version"));
  EXPECT_TRUE(entry["version"].is_string());
  EXPECT_FALSE(entry["version"].get<std::string>().empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoSovdEntryHasBaseUri) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  auto & entry = body["sovd_info"][0];
  EXPECT_TRUE(entry.contains("base_uri"));
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoSovdEntryHasVendorInfo) {
  handlers_.handle_version_info(req_, res_);
  auto body = json::parse(res_.body);
  auto & entry = body["sovd_info"][0];
  EXPECT_TRUE(entry.contains("vendor_info"));
  EXPECT_TRUE(entry["vendor_info"].contains("name"));
  EXPECT_EQ(entry["vendor_info"]["name"], "ros2_medkit");
}

// --- handle_root ---

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootResponseContainsRequiredTopLevelFields) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_TRUE(body.contains("name"));
  EXPECT_FALSE(body["name"].get<std::string>().empty());
  EXPECT_TRUE(body.contains("version"));
  EXPECT_FALSE(body["version"].get<std::string>().empty());
  EXPECT_TRUE(body.contains("api_base"));
  EXPECT_FALSE(body["api_base"].get<std::string>().empty());
  EXPECT_TRUE(body.contains("endpoints"));
  EXPECT_TRUE(body.contains("capabilities"));
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootEndpointsIsNonEmptyArray) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  ASSERT_TRUE(body["endpoints"].is_array());
  EXPECT_FALSE(body["endpoints"].empty());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesContainsDiscovery) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  auto & caps = body["capabilities"];
  EXPECT_TRUE(caps.contains("discovery"));
  EXPECT_TRUE(caps["discovery"].get<bool>());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthDisabledNoAuthEndpoints) {
  // With auth disabled (default), auth endpoints must not appear in the list
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  for (const auto & ep : body["endpoints"]) {
    EXPECT_EQ(ep.get<std::string>().find("/auth/"), std::string::npos)
        << "Unexpected auth endpoint when auth is disabled: " << ep;
  }
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesAuthDisabled) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_FALSE(body["capabilities"]["authentication"].get<bool>());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootCapabilitiesTlsDisabled) {
  handlers_.handle_root(req_, res_);
  auto body = json::parse(res_.body);
  EXPECT_FALSE(body["capabilities"]["tls"].get<bool>());
  EXPECT_FALSE(body.contains("tls"));
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledAddsAuthEndpoints) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auto ctx_auth = make_context(auth_enabled, tls_config_);
  HealthHandlers handlers_auth(ctx_auth);

  handlers_auth.handle_root(req_, res_);
  auto body = json::parse(res_.body);

  bool has_auth_endpoint = false;
  for (const auto & ep : body["endpoints"]) {
    if (ep.get<std::string>().find("/auth/") != std::string::npos) {
      has_auth_endpoint = true;
      break;
    }
  }
  EXPECT_TRUE(has_auth_endpoint);
  EXPECT_TRUE(body["capabilities"]["authentication"].get<bool>());
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootAuthEnabledIncludesAuthMetadataBlock) {
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  auth_enabled.require_auth_for = ros2_medkit_gateway::AuthRequirement::ALL;
  auth_enabled.jwt_algorithm = ros2_medkit_gateway::JwtAlgorithm::HS256;
  auto ctx_auth = make_context(auth_enabled, tls_config_);
  HealthHandlers handlers_auth(ctx_auth);

  handlers_auth.handle_root(req_, res_);
  auto body = json::parse(res_.body);

  ASSERT_TRUE(body.contains("auth"));
  EXPECT_TRUE(body["auth"]["enabled"].get<bool>());
  EXPECT_EQ(body["auth"]["algorithm"], "HS256");
  EXPECT_EQ(body["auth"]["require_auth_for"], "all");
}

// @verifies REQ_INTEROP_010
TEST_F(HealthHandlersTest, HandleRootTlsEnabledIncludesTlsMetadataBlock) {
  TlsConfig tls_enabled{};
  tls_enabled.enabled = true;
  tls_enabled.min_version = "1.3";
  auto ctx_tls = make_context(auth_config_, tls_enabled);
  HealthHandlers handlers_tls(ctx_tls);

  handlers_tls.handle_root(req_, res_);
  auto body = json::parse(res_.body);

  ASSERT_TRUE(body.contains("tls"));
  EXPECT_TRUE(body["tls"]["enabled"].get<bool>());
  EXPECT_EQ(body["tls"]["min_version"], "1.3");
  EXPECT_TRUE(body["capabilities"]["tls"].get<bool>());
}
