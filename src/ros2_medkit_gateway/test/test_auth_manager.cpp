// Copyright 2025 bburda
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

#include <chrono>
#include <thread>

#include "ros2_medkit_gateway/auth_config.hpp"
#include "ros2_medkit_gateway/auth_manager.hpp"
#include "ros2_medkit_gateway/auth_models.hpp"

using namespace ros2_medkit_gateway;

// Test fixture for AuthManager tests
// @verifies REQ_INTEROP_086, REQ_INTEROP_087
class AuthManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a test configuration with auth enabled
    config_ = AuthConfigBuilder()
                  .with_enabled(true)
                  .with_jwt_secret("test_secret_key_for_jwt_signing_12345")
                  .with_algorithm(JwtAlgorithm::HS256)
                  .with_token_expiry(3600)
                  .with_refresh_token_expiry(86400)
                  .with_require_auth_for(AuthRequirement::WRITE)
                  .with_issuer("test_issuer")
                  .add_client("admin_user", "admin_password", UserRole::ADMIN)
                  .add_client("operator_user", "operator_password", UserRole::OPERATOR)
                  .add_client("viewer_user", "viewer_password", UserRole::VIEWER)
                  .add_client("configurator_user", "configurator_password", UserRole::CONFIGURATOR)
                  .build();

    auth_manager_ = std::make_unique<AuthManager>(config_);
  }

  AuthConfig config_;
  std::unique_ptr<AuthManager> auth_manager_;
};

// Test configuration builder
TEST(AuthConfigBuilderTest, BuildValidConfig) {
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("test_secret")
                          .with_algorithm(JwtAlgorithm::HS256)
                          .with_token_expiry(3600)
                          .with_refresh_token_expiry(86400)
                          .with_require_auth_for(AuthRequirement::WRITE)
                          .with_issuer("test_issuer")
                          .add_client("test_client", "test_secret", UserRole::ADMIN)
                          .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.jwt_secret, "test_secret");
  EXPECT_EQ(config.jwt_algorithm, JwtAlgorithm::HS256);
  EXPECT_EQ(config.token_expiry_seconds, 3600);
  EXPECT_EQ(config.refresh_token_expiry_seconds, 86400);
  EXPECT_EQ(config.require_auth_for, AuthRequirement::WRITE);
  EXPECT_EQ(config.issuer, "test_issuer");
  EXPECT_EQ(config.clients.size(), 1);
  EXPECT_EQ(config.clients[0].client_id, "test_client");
}

TEST(AuthConfigBuilderTest, BuildWithoutSecretThrows) {
  EXPECT_THROW(
      { AuthConfigBuilder().with_enabled(true).with_token_expiry(3600).with_refresh_token_expiry(86400).build(); },
      std::invalid_argument);
}

TEST(AuthConfigBuilderTest, RefreshExpiryLessThanTokenExpiryThrows) {
  EXPECT_THROW(
      {
        AuthConfigBuilder()
            .with_enabled(true)
            .with_jwt_secret("secret")
            .with_token_expiry(3600)
            .with_refresh_token_expiry(1800)  // Less than token expiry
            .build();
      },
      std::invalid_argument);
}

TEST(AuthConfigBuilderTest, DisabledConfigDoesNotRequireSecret) {
  AuthConfig config = AuthConfigBuilder().with_enabled(false).build();

  EXPECT_FALSE(config.enabled);
}

// Test role/algorithm conversions
TEST(AuthConfigTest, RoleToString) {
  EXPECT_EQ(role_to_string(UserRole::VIEWER), "viewer");
  EXPECT_EQ(role_to_string(UserRole::OPERATOR), "operator");
  EXPECT_EQ(role_to_string(UserRole::CONFIGURATOR), "configurator");
  EXPECT_EQ(role_to_string(UserRole::ADMIN), "admin");
}

TEST(AuthConfigTest, StringToRole) {
  EXPECT_EQ(string_to_role("viewer"), UserRole::VIEWER);
  EXPECT_EQ(string_to_role("operator"), UserRole::OPERATOR);
  EXPECT_EQ(string_to_role("configurator"), UserRole::CONFIGURATOR);
  EXPECT_EQ(string_to_role("admin"), UserRole::ADMIN);
  // Case insensitive
  EXPECT_EQ(string_to_role("ADMIN"), UserRole::ADMIN);
  EXPECT_EQ(string_to_role("Viewer"), UserRole::VIEWER);
}

TEST(AuthConfigTest, StringToRoleInvalid) {
  EXPECT_THROW(string_to_role("invalid_role"), std::invalid_argument);
}

TEST(AuthConfigTest, AlgorithmToString) {
  EXPECT_EQ(algorithm_to_string(JwtAlgorithm::HS256), "HS256");
  EXPECT_EQ(algorithm_to_string(JwtAlgorithm::RS256), "RS256");
}

TEST(AuthConfigTest, StringToAlgorithm) {
  EXPECT_EQ(string_to_algorithm("HS256"), JwtAlgorithm::HS256);
  EXPECT_EQ(string_to_algorithm("RS256"), JwtAlgorithm::RS256);
  EXPECT_EQ(string_to_algorithm("hs256"), JwtAlgorithm::HS256);
}

TEST(AuthConfigTest, StringToAlgorithmInvalid) {
  EXPECT_THROW(string_to_algorithm("invalid"), std::invalid_argument);
}

TEST(AuthConfigTest, StringToAuthRequirement) {
  EXPECT_EQ(string_to_auth_requirement("none"), AuthRequirement::NONE);
  EXPECT_EQ(string_to_auth_requirement("write"), AuthRequirement::WRITE);
  EXPECT_EQ(string_to_auth_requirement("all"), AuthRequirement::ALL);
  EXPECT_EQ(string_to_auth_requirement("WRITE"), AuthRequirement::WRITE);
}

TEST(AuthConfigTest, StringToAuthRequirementInvalid) {
  EXPECT_THROW(string_to_auth_requirement("invalid"), std::invalid_argument);
}

// Test AuthManager authentication
// @verifies REQ_INTEROP_086
TEST_F(AuthManagerTest, AuthenticateValidCredentials) {
  auto result = auth_manager_->authenticate("admin_user", "admin_password");

  ASSERT_TRUE(result.has_value());
  EXPECT_FALSE(result->access_token.empty());
  EXPECT_FALSE(result->refresh_token.value_or("").empty());
  EXPECT_EQ(result->token_type, "Bearer");
  EXPECT_EQ(result->expires_in, 3600);
  EXPECT_EQ(result->scope, "admin");
}

TEST_F(AuthManagerTest, AuthenticateInvalidClientId) {
  auto result = auth_manager_->authenticate("nonexistent_client", "password");

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().error, "invalid_client");
}

TEST_F(AuthManagerTest, AuthenticateInvalidPassword) {
  auto result = auth_manager_->authenticate("admin_user", "wrong_password");

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().error, "invalid_client");
}

// Test token validation
// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, ValidateValidToken) {
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());

  auto validation = auth_manager_->validate_token(auth_result->access_token);

  EXPECT_TRUE(validation.valid);
  EXPECT_TRUE(validation.claims.has_value());
  EXPECT_EQ(validation.claims->sub, "admin_user");
  EXPECT_EQ(validation.claims->role, UserRole::ADMIN);
  EXPECT_EQ(validation.claims->iss, "test_issuer");
}

TEST_F(AuthManagerTest, ValidateInvalidToken) {
  auto validation = auth_manager_->validate_token("invalid.token.here");

  EXPECT_FALSE(validation.valid);
  EXPECT_FALSE(validation.error.empty());
}

TEST_F(AuthManagerTest, ValidateTamperedToken) {
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());

  // Tamper with the token
  std::string tampered = auth_result->access_token;
  if (!tampered.empty()) {
    tampered[tampered.length() / 2] = 'X';
  }

  auto validation = auth_manager_->validate_token(tampered);
  EXPECT_FALSE(validation.valid);
}

// Test token refresh
// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, RefreshAccessToken) {
  auto auth_result = auth_manager_->authenticate("operator_user", "operator_password");
  ASSERT_TRUE(auth_result.has_value());
  ASSERT_TRUE(auth_result->refresh_token.has_value());

  auto refresh_result = auth_manager_->refresh_access_token(auth_result->refresh_token.value());

  ASSERT_TRUE(refresh_result.has_value());
  EXPECT_FALSE(refresh_result->access_token.empty());
  EXPECT_EQ(refresh_result->scope, "operator");
  // Refresh should return a new access token
  EXPECT_NE(refresh_result->access_token, auth_result->access_token);
}

TEST_F(AuthManagerTest, RefreshWithInvalidToken) {
  auto result = auth_manager_->refresh_access_token("invalid_refresh_token");

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().error, "invalid_grant");
}

// Test token revocation
TEST_F(AuthManagerTest, RevokeRefreshToken) {
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());
  ASSERT_TRUE(auth_result->refresh_token.has_value());

  // Revoke the refresh token
  bool revoked = auth_manager_->revoke_refresh_token(auth_result->refresh_token.value());
  EXPECT_TRUE(revoked);

  // Try to use the revoked refresh token
  auto refresh_result = auth_manager_->refresh_access_token(auth_result->refresh_token.value());
  ASSERT_FALSE(refresh_result.has_value());
  EXPECT_EQ(refresh_result.error().error, "invalid_grant");

  // Access token should still be valid (until expiry)
  auto validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_FALSE(validation.valid);  // Access token tied to revoked refresh token
}

// Test RBAC authorization
// @verifies REQ_INTEROP_086
TEST_F(AuthManagerTest, AuthorizeViewerCanRead) {
  auto result = auth_manager_->check_authorization(UserRole::VIEWER, "GET", "/api/v1/components");
  EXPECT_TRUE(result.authorized);

  result = auth_manager_->check_authorization(UserRole::VIEWER, "GET", "/api/v1/components/engine/data");
  EXPECT_TRUE(result.authorized);

  result = auth_manager_->check_authorization(UserRole::VIEWER, "GET", "/api/v1/areas");
  EXPECT_TRUE(result.authorized);
}

TEST_F(AuthManagerTest, AuthorizeViewerCannotWrite) {
  auto result =
      auth_manager_->check_authorization(UserRole::VIEWER, "POST", "/api/v1/components/engine/operations/calibrate");
  EXPECT_FALSE(result.authorized);

  result =
      auth_manager_->check_authorization(UserRole::VIEWER, "PUT", "/api/v1/components/engine/configurations/threshold");
  EXPECT_FALSE(result.authorized);

  result = auth_manager_->check_authorization(UserRole::VIEWER, "DELETE", "/api/v1/components/engine/faults/F001");
  EXPECT_FALSE(result.authorized);
}

TEST_F(AuthManagerTest, AuthorizeOperatorCanTriggerOperations) {
  auto result =
      auth_manager_->check_authorization(UserRole::OPERATOR, "POST", "/api/v1/components/engine/operations/calibrate");
  EXPECT_TRUE(result.authorized);

  result = auth_manager_->check_authorization(UserRole::OPERATOR, "DELETE", "/api/v1/components/engine/faults/F001");
  EXPECT_TRUE(result.authorized);

  result = auth_manager_->check_authorization(UserRole::OPERATOR, "PUT", "/api/v1/components/engine/data/temperature");
  EXPECT_TRUE(result.authorized);
}

TEST_F(AuthManagerTest, AuthorizeOperatorCannotModifyConfigurations) {
  auto result = auth_manager_->check_authorization(UserRole::OPERATOR, "PUT",
                                                   "/api/v1/components/engine/configurations/threshold");
  EXPECT_FALSE(result.authorized);
}

TEST_F(AuthManagerTest, AuthorizeConfiguratorCanModifyConfigurations) {
  auto result = auth_manager_->check_authorization(UserRole::CONFIGURATOR, "PUT",
                                                   "/api/v1/components/engine/configurations/threshold");
  EXPECT_TRUE(result.authorized);

  result = auth_manager_->check_authorization(UserRole::CONFIGURATOR, "DELETE",
                                              "/api/v1/components/engine/configurations/threshold");
  EXPECT_TRUE(result.authorized);
}

TEST_F(AuthManagerTest, AuthorizeAdminHasFullAccess) {
  auto result = auth_manager_->check_authorization(UserRole::ADMIN, "GET", "/api/v1/components");
  EXPECT_TRUE(result.authorized);

  result =
      auth_manager_->check_authorization(UserRole::ADMIN, "POST", "/api/v1/components/engine/operations/calibrate");
  EXPECT_TRUE(result.authorized);

  result =
      auth_manager_->check_authorization(UserRole::ADMIN, "PUT", "/api/v1/components/engine/configurations/threshold");
  EXPECT_TRUE(result.authorized);

  result = auth_manager_->check_authorization(UserRole::ADMIN, "DELETE", "/api/v1/anything/goes");
  EXPECT_TRUE(result.authorized);
}

// Test auth requirement checking
TEST_F(AuthManagerTest, RequiresAuthForWriteOnly) {
  EXPECT_FALSE(auth_manager_->requires_authentication("GET", "/api/v1/components"));
  EXPECT_TRUE(auth_manager_->requires_authentication("POST", "/api/v1/components/engine/operations/calibrate"));
  EXPECT_TRUE(auth_manager_->requires_authentication("PUT", "/api/v1/components/engine/configurations/threshold"));
  EXPECT_TRUE(auth_manager_->requires_authentication("DELETE", "/api/v1/components/engine/faults/F001"));
}

TEST_F(AuthManagerTest, AuthEndpointsNeverRequireAuth) {
  EXPECT_FALSE(auth_manager_->requires_authentication("POST", "/api/v1/auth/authorize"));
  EXPECT_FALSE(auth_manager_->requires_authentication("POST", "/api/v1/auth/token"));
  EXPECT_FALSE(auth_manager_->requires_authentication("POST", "/api/v1/auth/revoke"));
}

// Test all auth requirement mode
TEST(AuthManagerRequirementTest, RequireAuthForAll) {
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("test_secret")
                          .with_token_expiry(3600)
                          .with_refresh_token_expiry(86400)
                          .with_require_auth_for(AuthRequirement::ALL)
                          .build();

  AuthManager manager(config);

  EXPECT_TRUE(manager.requires_authentication("GET", "/api/v1/components"));
  EXPECT_TRUE(manager.requires_authentication("POST", "/api/v1/components/engine/operations/calibrate"));
  // Auth endpoints still don't require auth
  EXPECT_FALSE(manager.requires_authentication("POST", "/api/v1/auth/authorize"));
}

// Test none auth requirement mode
TEST(AuthManagerRequirementTest, RequireAuthForNone) {
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("test_secret")
                          .with_token_expiry(3600)
                          .with_refresh_token_expiry(86400)
                          .with_require_auth_for(AuthRequirement::NONE)
                          .build();

  AuthManager manager(config);

  EXPECT_FALSE(manager.requires_authentication("GET", "/api/v1/components"));
  EXPECT_FALSE(manager.requires_authentication("POST", "/api/v1/components/engine/operations/calibrate"));
}

// Test disabled auth
TEST(AuthManagerDisabledTest, DisabledAuthManagerNeverRequiresAuth) {
  AuthConfig config = AuthConfigBuilder().with_enabled(false).build();

  AuthManager manager(config);

  EXPECT_FALSE(manager.is_enabled());
  EXPECT_FALSE(manager.requires_authentication("GET", "/api/v1/components"));
  EXPECT_FALSE(manager.requires_authentication("POST", "/api/v1/components/engine/operations/calibrate"));
}

// Test client registration
TEST_F(AuthManagerTest, RegisterNewClient) {
  bool registered = auth_manager_->register_client("new_client", "new_secret", UserRole::VIEWER);
  EXPECT_TRUE(registered);

  auto client = auth_manager_->get_client("new_client");
  ASSERT_TRUE(client.has_value());
  EXPECT_EQ(client->client_id, "new_client");
  EXPECT_EQ(client->role, UserRole::VIEWER);

  // Can authenticate with new client
  auto result = auth_manager_->authenticate("new_client", "new_secret");
  EXPECT_TRUE(result.has_value());
}

TEST_F(AuthManagerTest, RegisterDuplicateClientFails) {
  bool registered = auth_manager_->register_client("admin_user", "different_secret", UserRole::VIEWER);
  EXPECT_FALSE(registered);
}

// Test cleanup of expired tokens
TEST_F(AuthManagerTest, CleanupExpiredTokens) {
  // Create config with very short expiry for testing
  AuthConfig short_expiry_config = AuthConfigBuilder()
                                       .with_enabled(true)
                                       .with_jwt_secret("test_secret")
                                       .with_token_expiry(1)  // 1 second
                                       .with_refresh_token_expiry(1)
                                       .add_client("test", "test", UserRole::VIEWER)
                                       .build();

  AuthManager manager(short_expiry_config);

  // Authenticate to create tokens
  auto result = manager.authenticate("test", "test");
  ASSERT_TRUE(result.has_value());

  // Wait for tokens to expire
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // Cleanup should remove expired tokens
  size_t cleaned = manager.cleanup_expired_tokens();
  EXPECT_GE(cleaned, 1);
}

// Test JwtClaims
TEST(JwtClaimsTest, ToJson) {
  JwtClaims claims;
  claims.iss = "test_issuer";
  claims.sub = "test_subject";
  claims.exp = 1234567890;
  claims.iat = 1234567800;
  claims.jti = "test_jti";
  claims.role = UserRole::ADMIN;
  claims.permissions = {"read", "write"};
  claims.refresh_token_id = "refresh_123";

  auto j = claims.to_json();

  EXPECT_EQ(j["iss"], "test_issuer");
  EXPECT_EQ(j["sub"], "test_subject");
  EXPECT_EQ(j["exp"], 1234567890);
  EXPECT_EQ(j["iat"], 1234567800);
  EXPECT_EQ(j["jti"], "test_jti");
  EXPECT_EQ(j["role"], "admin");
  EXPECT_EQ(j["permissions"].size(), 2);
  EXPECT_EQ(j["refresh_token_id"], "refresh_123");
}

TEST(JwtClaimsTest, FromJson) {
  nlohmann::json j = {
      {"iss", "test_issuer"}, {"sub", "test_subject"}, {"exp", 1234567890},       {"iat", 1234567800},
      {"jti", "test_jti"},    {"role", "operator"},    {"permissions", {"read"}}, {"refresh_token_id", "refresh_456"}};

  auto claims = JwtClaims::from_json(j);

  EXPECT_EQ(claims.iss, "test_issuer");
  EXPECT_EQ(claims.sub, "test_subject");
  EXPECT_EQ(claims.exp, 1234567890);
  EXPECT_EQ(claims.iat, 1234567800);
  EXPECT_EQ(claims.jti, "test_jti");
  EXPECT_EQ(claims.role, UserRole::OPERATOR);
  EXPECT_EQ(claims.permissions.size(), 1);
  EXPECT_TRUE(claims.refresh_token_id.has_value());
  EXPECT_EQ(claims.refresh_token_id.value(), "refresh_456");
}

TEST(JwtClaimsTest, IsExpired) {
  JwtClaims claims;

  // Expired token
  claims.exp = 1000;
  EXPECT_TRUE(claims.is_expired());

  // Future token
  auto future = std::chrono::system_clock::now() + std::chrono::hours(1);
  claims.exp = std::chrono::duration_cast<std::chrono::seconds>(future.time_since_epoch()).count();
  EXPECT_FALSE(claims.is_expired());
}

// Test TokenResponse
TEST(TokenResponseTest, ToJson) {
  TokenResponse response;
  response.access_token = "access_123";
  response.token_type = "Bearer";
  response.expires_in = 3600;
  response.refresh_token = "refresh_456";
  response.scope = "admin";

  auto j = response.to_json();

  EXPECT_EQ(j["access_token"], "access_123");
  EXPECT_EQ(j["token_type"], "Bearer");
  EXPECT_EQ(j["expires_in"], 3600);
  EXPECT_EQ(j["refresh_token"], "refresh_456");
  EXPECT_EQ(j["scope"], "admin");
}

TEST(TokenResponseTest, ToJsonWithoutRefreshToken) {
  TokenResponse response;
  response.access_token = "access_123";
  response.token_type = "Bearer";
  response.expires_in = 3600;
  response.scope = "viewer";

  auto j = response.to_json();

  EXPECT_EQ(j["access_token"], "access_123");
  EXPECT_FALSE(j.contains("refresh_token"));
}

// Test AuthErrorResponse
TEST(AuthErrorResponseTest, StandardErrors) {
  auto invalid_request = AuthErrorResponse::invalid_request("Missing parameter");
  EXPECT_EQ(invalid_request.error, "invalid_request");
  EXPECT_EQ(invalid_request.error_description, "Missing parameter");

  auto invalid_client = AuthErrorResponse::invalid_client("Unknown client");
  EXPECT_EQ(invalid_client.error, "invalid_client");

  auto invalid_grant = AuthErrorResponse::invalid_grant("Token expired");
  EXPECT_EQ(invalid_grant.error, "invalid_grant");

  auto unsupported_grant = AuthErrorResponse::unsupported_grant_type("Not supported");
  EXPECT_EQ(unsupported_grant.error, "unsupported_grant_type");

  auto invalid_token = AuthErrorResponse::invalid_token("Malformed");
  EXPECT_EQ(invalid_token.error, "invalid_token");

  auto insufficient_scope = AuthErrorResponse::insufficient_scope("Need admin");
  EXPECT_EQ(insufficient_scope.error, "insufficient_scope");
}

TEST(AuthErrorResponseTest, ToJson) {
  auto error = AuthErrorResponse::invalid_request("Test description");
  auto j = error.to_json();

  EXPECT_EQ(j["error"], "invalid_request");
  EXPECT_EQ(j["error_description"], "Test description");
}

// Test AuthorizeRequest form parsing
TEST(AuthorizeRequestTest, FromFormData) {
  std::string form_data = "grant_type=client_credentials&client_id=test_client&client_secret=test_secret&scope=admin";

  auto req = AuthorizeRequest::from_form_data(form_data);

  EXPECT_EQ(req.grant_type, "client_credentials");
  EXPECT_TRUE(req.client_id.has_value());
  EXPECT_EQ(req.client_id.value(), "test_client");
  EXPECT_TRUE(req.client_secret.has_value());
  EXPECT_EQ(req.client_secret.value(), "test_secret");
  EXPECT_TRUE(req.scope.has_value());
  EXPECT_EQ(req.scope.value(), "admin");
}

TEST(AuthorizeRequestTest, FromFormDataWithUrlEncoding) {
  std::string form_data = "grant_type=refresh_token&refresh_token=abc%2Bdef%3D123";

  auto req = AuthorizeRequest::from_form_data(form_data);

  EXPECT_EQ(req.grant_type, "refresh_token");
  EXPECT_TRUE(req.refresh_token.has_value());
  EXPECT_EQ(req.refresh_token.value(), "abc+def=123");
}

TEST(AuthorizeRequestTest, FromJson) {
  nlohmann::json j = {{"grant_type", "client_credentials"},
                      {"client_id", "test_client"},
                      {"client_secret", "test_secret"},
                      {"scope", "operator"}};

  auto req = AuthorizeRequest::from_json(j);

  EXPECT_EQ(req.grant_type, "client_credentials");
  EXPECT_TRUE(req.client_id.has_value());
  EXPECT_EQ(req.client_id.value(), "test_client");
  EXPECT_TRUE(req.client_secret.has_value());
  EXPECT_EQ(req.client_secret.value(), "test_secret");
  EXPECT_TRUE(req.scope.has_value());
  EXPECT_EQ(req.scope.value(), "operator");
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
