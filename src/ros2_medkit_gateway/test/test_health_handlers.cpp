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

class HealthHandlersTest : public ::testing::Test
{
protected:
  CorsConfig cors_config_{};
  AuthConfig auth_config_{};  // enabled = false by default
  TlsConfig tls_config_{};    // enabled = false by default
  HandlerContext ctx_{nullptr, cors_config_, auth_config_, tls_config_, nullptr};
  HealthHandlers handlers_{ctx_};
};

// --- handle_health ---

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleHealthResponseContainsStatusHealthy)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_health(req, res);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["status"], "healthy");
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleHealthResponseContainsTimestamp)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_health(req, res);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("timestamp"));
  EXPECT_TRUE(body["timestamp"].is_number());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleHealthResponseIsValidJson)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_health(req, res);
  EXPECT_NO_THROW(json::parse(res.body));
}

// --- handle_version_info ---

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoContainsSovdInfoArray)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_version_info(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("sovd_info"));
  ASSERT_TRUE(body["sovd_info"].is_array());
  EXPECT_FALSE(body["sovd_info"].empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoSovdEntryHasVersionField)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_version_info(req, res);
  auto body = json::parse(res.body);
  auto & entry = body["sovd_info"][0];
  EXPECT_TRUE(entry.contains("version"));
  EXPECT_TRUE(entry["version"].is_string());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoSovdEntryHasBaseUri)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_version_info(req, res);
  auto body = json::parse(res.body);
  auto & entry = body["sovd_info"][0];
  EXPECT_TRUE(entry.contains("base_uri"));
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleVersionInfoSovdEntryHasVendorInfo)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_version_info(req, res);
  auto body = json::parse(res.body);
  auto & entry = body["sovd_info"][0];
  EXPECT_TRUE(entry.contains("vendor_info"));
  EXPECT_TRUE(entry["vendor_info"].contains("name"));
  EXPECT_EQ(entry["vendor_info"]["name"], "ros2_medkit");
}

// --- handle_root ---

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleRootResponseContainsRequiredTopLevelFields)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_root(req, res);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("name"));
  EXPECT_TRUE(body.contains("version"));
  EXPECT_TRUE(body.contains("api_base"));
  EXPECT_TRUE(body.contains("endpoints"));
  EXPECT_TRUE(body.contains("capabilities"));
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleRootEndpointsIsNonEmptyArray)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_root(req, res);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body["endpoints"].is_array());
  EXPECT_FALSE(body["endpoints"].empty());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleRootCapabilitiesContainsDiscovery)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_root(req, res);
  auto body = json::parse(res.body);
  auto & caps = body["capabilities"];
  EXPECT_TRUE(caps.contains("discovery"));
  EXPECT_TRUE(caps["discovery"].get<bool>());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleRootAuthDisabledNoAuthEndpoints)
{
  // With auth disabled (default), auth endpoints must not appear in the list
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_root(req, res);
  auto body = json::parse(res.body);
  for (const auto & ep : body["endpoints"]) {
    EXPECT_EQ(ep.get<std::string>().find("/auth/"), std::string::npos)
        << "Unexpected auth endpoint when auth is disabled: " << ep;
  }
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleRootCapabilitiesAuthDisabled)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_root(req, res);
  auto body = json::parse(res.body);
  EXPECT_FALSE(body["capabilities"]["authentication"].get<bool>());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleRootCapabilitiesTlsDisabled)
{
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_root(req, res);
  auto body = json::parse(res.body);
  EXPECT_FALSE(body["capabilities"]["tls"].get<bool>());
}

// @verifies REQ_INTEROP_001
TEST_F(HealthHandlersTest, HandleRootAuthEnabledAddsAuthEndpoints)
{
  AuthConfig auth_enabled{};
  auth_enabled.enabled = true;
  TlsConfig tls{};
  CorsConfig cors{};
  HandlerContext ctx_auth(nullptr, cors, auth_enabled, tls, nullptr);
  HealthHandlers handlers_auth(ctx_auth);

  httplib::Request req;
  httplib::Response res;
  handlers_auth.handle_root(req, res);
  auto body = json::parse(res.body);

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
