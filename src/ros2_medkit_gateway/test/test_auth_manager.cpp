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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include <jwt-cpp/jwt.h>

#include "ros2_medkit_gateway/core/auth/auth.hpp"

using namespace ros2_medkit_gateway;

namespace {

/// A stand-in for the shipped permission table, and honestly a stand-in.
///
/// The real table is derived from the route registrations and installed by
/// `RESTServer::setup_routes()`; an `AuthManager` built on its own has none and
/// fails closed. What the authorization tests below exercise is the *matcher* -
/// exact hit, single-segment `*`, multi-segment `**`, and the no-match refusal
/// - so they need a table shaped like the derived one rather than the derived
/// one itself: roles expanded upward, because `AuthConfig` has no inheritance.
///
/// It says nothing about whether the shipped table grants the right thing on
/// the right path. No hand-written table can answer that, since it would be the
/// same author restating the same belief twice. The gateway's real table is
/// checked against the gateway's real enforcement in
/// `test_rbac_contract.test.py`.
RoutePermissions matcher_fixture_permissions() {
  const std::unordered_set<std::string> viewer = {
      "GET:/api/v1/components",
      "GET:/api/v1/components/*/data",
      "GET:/api/v1/areas",
  };
  const std::unordered_set<std::string> operator_only = {
      "POST:/api/v1/components/*/operations/*/executions",
      "DELETE:/api/v1/components/*/faults/*",
      "PUT:/api/v1/components/*/data/*",
  };
  const std::unordered_set<std::string> configurator_only = {
      "PUT:/api/v1/components/*/configurations/*",
      "DELETE:/api/v1/components/*/configurations/*",
  };

  RoutePermissions permissions;
  permissions[UserRole::VIEWER] = viewer;
  permissions[UserRole::OPERATOR] = viewer;
  permissions[UserRole::OPERATOR].insert(operator_only.begin(), operator_only.end());
  permissions[UserRole::CONFIGURATOR] = permissions[UserRole::OPERATOR];
  permissions[UserRole::CONFIGURATOR].insert(configurator_only.begin(), configurator_only.end());
  return permissions;
}

}  // namespace

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
    // ADMIN's entries come from the residual list, which is where they live on
    // a running gateway too - `**` per method, covering the routes the registry
    // never sees.
    auth_manager_->add_route_permissions(matcher_fixture_permissions());
    auth_manager_->add_route_permissions(AuthConfig::residual_route_permissions());
  }

  AuthConfig config_;
  std::unique_ptr<AuthManager> auth_manager_;
};

// Test configuration builder
TEST(AuthConfigBuilderTest, BuildValidConfig) {
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("test_secret_key_min_32_chars_12345")
                          .with_algorithm(JwtAlgorithm::HS256)
                          .with_token_expiry(3600)
                          .with_refresh_token_expiry(86400)
                          .with_require_auth_for(AuthRequirement::WRITE)
                          .with_issuer("test_issuer")
                          .add_client("test_client", "test_secret", UserRole::ADMIN)
                          .build();

  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.jwt_secret, "test_secret_key_min_32_chars_12345");
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
            .with_jwt_secret("secret_key_with_at_least_32_chars_xyz")
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

// Test TokenType
// @verifies REQ_INTEROP_087
TEST(TokenTypeTest, TokenTypeToString) {
  EXPECT_EQ(token_type_to_string(TokenType::ACCESS), "access");
  EXPECT_EQ(token_type_to_string(TokenType::REFRESH), "refresh");
}

TEST(TokenTypeTest, StringToTokenType) {
  EXPECT_EQ(string_to_token_type("access"), TokenType::ACCESS);
  EXPECT_EQ(string_to_token_type("refresh"), TokenType::REFRESH);
}

TEST(TokenTypeTest, StringToTokenTypeInvalid) {
  EXPECT_THROW(string_to_token_type("invalid"), std::invalid_argument);
  EXPECT_THROW(string_to_token_type("ACCESS"), std::invalid_argument);  // Case-sensitive
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

// Test token type enforcement
// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, ValidateTokenWithCorrectType) {
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());

  // Access token should validate as ACCESS type
  auto access_validation = auth_manager_->validate_token(auth_result->access_token, TokenType::ACCESS);
  EXPECT_TRUE(access_validation.valid);
  EXPECT_EQ(access_validation.claims->typ, TokenType::ACCESS);

  // Refresh token should validate as REFRESH type
  ASSERT_TRUE(auth_result->refresh_token.has_value());
  auto refresh_validation = auth_manager_->validate_token(auth_result->refresh_token.value(), TokenType::REFRESH);
  EXPECT_TRUE(refresh_validation.valid);
  EXPECT_EQ(refresh_validation.claims->typ, TokenType::REFRESH);
}

// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, ValidateTokenWithWrongTypeRejectsToken) {
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());
  ASSERT_TRUE(auth_result->refresh_token.has_value());

  // Access token should NOT validate as REFRESH type
  auto access_as_refresh = auth_manager_->validate_token(auth_result->access_token, TokenType::REFRESH);
  EXPECT_FALSE(access_as_refresh.valid);
  EXPECT_TRUE(access_as_refresh.error.find("Invalid token type") != std::string::npos);

  // Refresh token should NOT validate as ACCESS type
  auto refresh_as_access = auth_manager_->validate_token(auth_result->refresh_token.value(), TokenType::ACCESS);
  EXPECT_FALSE(refresh_as_access.valid);
  EXPECT_TRUE(refresh_as_access.error.find("Invalid token type") != std::string::npos);
}

// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, RefreshWithAccessTokenFails) {
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());

  // Try to use access token as refresh token (should fail due to type check)
  auto refresh_result = auth_manager_->refresh_access_token(auth_result->access_token, "admin_user");
  ASSERT_FALSE(refresh_result.has_value());
  EXPECT_EQ(refresh_result.error().error, "invalid_grant");
  EXPECT_TRUE(refresh_result.error().error_description.find("not a refresh token") != std::string::npos);
}

// Test token refresh
// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, RefreshAccessToken) {
  auto auth_result = auth_manager_->authenticate("operator_user", "operator_password");
  ASSERT_TRUE(auth_result.has_value());
  ASSERT_TRUE(auth_result->refresh_token.has_value());

  auto refresh_result = auth_manager_->refresh_access_token(auth_result->refresh_token.value(), "operator_user");

  ASSERT_TRUE(refresh_result.has_value());
  EXPECT_FALSE(refresh_result->access_token.empty());
  EXPECT_EQ(refresh_result->scope, "operator");
  // Refresh should return a new access token
  EXPECT_NE(refresh_result->access_token, auth_result->access_token);
}

TEST_F(AuthManagerTest, RefreshWithInvalidToken) {
  auto result = auth_manager_->refresh_access_token("invalid_refresh_token", "admin_user");

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().error, "invalid_grant");
}

// Test token revocation
TEST_F(AuthManagerTest, RevokeRefreshToken) {
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());
  ASSERT_TRUE(auth_result->refresh_token.has_value());

  // Revoke the refresh token
  bool revoked = auth_manager_->revoke_refresh_token(auth_result->refresh_token.value(), "admin_user");
  EXPECT_TRUE(revoked);

  // Try to use the revoked refresh token
  auto refresh_result = auth_manager_->refresh_access_token(auth_result->refresh_token.value(), "admin_user");
  ASSERT_FALSE(refresh_result.has_value());
  EXPECT_EQ(refresh_result.error().error, "invalid_grant");

  // Access token should also be invalid (propagated revocation)
  auto validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_FALSE(validation.valid);
  EXPECT_TRUE(validation.error.find("revoked") != std::string::npos);
}

// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, RefreshRevocationPropagatestoAccessToken) {
  // Get tokens
  auto auth_result = auth_manager_->authenticate("operator_user", "operator_password");
  ASSERT_TRUE(auth_result.has_value());

  // Access token is valid initially
  auto validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_TRUE(validation.valid);

  // Revoke refresh token
  auth_manager_->revoke_refresh_token(auth_result->refresh_token.value(), "operator_user");

  // Access token should now be invalid because its refresh token was revoked
  validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_FALSE(validation.valid);
  EXPECT_TRUE(validation.error.find("refresh token has been revoked") != std::string::npos);
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
  // Executions endpoint
  auto result = auth_manager_->check_authorization(UserRole::OPERATOR, "POST",
                                                   "/api/v1/components/engine/operations/calibrate/executions");
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

// @verifies REQ_INTEROP_086
TEST(AuthManagerPermissionTableTest, AManagerWithNoTableAuthorizesNothing) {
  // The fail-closed property the whole derivation rests on. `RESTServer` feeds
  // the table before the server listens; a manager that never got one must
  // refuse rather than fall back to some built-in default, because a default
  // would be a second source for every grant.
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("test_secret_key_min_32_chars_empty")
                          .with_token_expiry(3600)
                          .with_refresh_token_expiry(86400)
                          .build();
  AuthManager manager(config);

  for (UserRole role : {UserRole::VIEWER, UserRole::OPERATOR, UserRole::CONFIGURATOR, UserRole::ADMIN}) {
    auto result = manager.check_authorization(role, "GET", "/api/v1/health");
    EXPECT_FALSE(result.authorized) << "role " << static_cast<int>(role);
  }
}

// @verifies REQ_INTEROP_086
TEST(AuthManagerPermissionTableTest, AddedPermissionsMergeRatherThanReplace) {
  // `RESTServer` calls this twice - once with the registry's derivation, once
  // with the residual list - so the second call must not erase the first.
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("test_secret_key_min_32_chars_merge")
                          .with_token_expiry(3600)
                          .with_refresh_token_expiry(86400)
                          .build();
  AuthManager manager(config);

  RoutePermissions first;
  first[UserRole::VIEWER] = {"GET:/api/v1/health"};
  RoutePermissions second;
  second[UserRole::VIEWER] = {"GET:/api/v1/version-info"};
  manager.add_route_permissions(first);
  manager.add_route_permissions(second);

  EXPECT_TRUE(manager.check_authorization(UserRole::VIEWER, "GET", "/api/v1/health").authorized);
  EXPECT_TRUE(manager.check_authorization(UserRole::VIEWER, "GET", "/api/v1/version-info").authorized);
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
                          .with_jwt_secret("test_secret_key_min_32_chars_all__")
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
                          .with_jwt_secret("test_secret_key_min_32_chars_none")
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

// Test RS256 fail fast at startup
// @verifies REQ_INTEROP_087
TEST(AuthManagerRS256Test, RS256WithMissingPrivateKeyThrows) {
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_algorithm(JwtAlgorithm::RS256)
                          .with_jwt_secret("/nonexistent/private.pem")
                          .with_jwt_public_key("/nonexistent/public.pem")
                          .add_client("test", "test", UserRole::VIEWER)
                          .build();

  EXPECT_THROW(AuthManager manager(config), std::runtime_error);
}

TEST(AuthManagerRS256Test, RS256DisabledDoesNotValidateKeys) {
  // When auth is disabled, RS256 keys should not be validated
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(false)
                          .with_algorithm(JwtAlgorithm::RS256)
                          .with_jwt_secret("/nonexistent/private.pem")
                          .with_jwt_public_key("/nonexistent/public.pem")
                          .build();

  EXPECT_NO_THROW(AuthManager manager(config));
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

// Test client state validation on every request
// @verifies REQ_INTEROP_087
TEST_F(AuthManagerTest, DisabledClientTokenBecomesInvalid) {
  // Authenticate first
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());

  // Token should be valid initially
  auto validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_TRUE(validation.valid);

  // Disable the client
  bool disabled = auth_manager_->disable_client("admin_user");
  EXPECT_TRUE(disabled);

  // Token should now be invalid
  validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_FALSE(validation.valid);
  EXPECT_TRUE(validation.error.find("disabled") != std::string::npos);
}

TEST_F(AuthManagerTest, ReenabledClientTokenBecomesValid) {
  // Authenticate first
  auto auth_result = auth_manager_->authenticate("admin_user", "admin_password");
  ASSERT_TRUE(auth_result.has_value());

  // Disable the client
  auth_manager_->disable_client("admin_user");
  auto validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_FALSE(validation.valid);

  // Re-enable the client
  bool enabled = auth_manager_->enable_client("admin_user");
  EXPECT_TRUE(enabled);

  // Token should be valid again
  validation = auth_manager_->validate_token(auth_result->access_token);
  EXPECT_TRUE(validation.valid);
}

TEST_F(AuthManagerTest, DisableNonexistentClientFails) {
  bool disabled = auth_manager_->disable_client("nonexistent_client");
  EXPECT_FALSE(disabled);
}

TEST_F(AuthManagerTest, EnableNonexistentClientFails) {
  bool enabled = auth_manager_->enable_client("nonexistent_client");
  EXPECT_FALSE(enabled);
}

// Test cleanup of expired tokens
TEST_F(AuthManagerTest, CleanupExpiredTokens) {
  // Create config with very short expiry for testing
  AuthConfig short_expiry_config = AuthConfigBuilder()
                                       .with_enabled(true)
                                       .with_jwt_secret("test_secret_key_min_32_chars_exp__")
                                       .with_token_expiry(1)  // 1 second
                                       .with_refresh_token_expiry(1)
                                       .add_client("test", "test", UserRole::VIEWER)
                                       .build();

  AuthManager manager(short_expiry_config);

  // Authenticate to create tokens
  auto result = manager.authenticate("test", "test");
  ASSERT_TRUE(result.has_value());

  // Wait for tokens to expire (3s margin for loaded systems)
  std::this_thread::sleep_for(std::chrono::seconds(3));

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
  claims.typ = TokenType::ACCESS;
  claims.role = UserRole::ADMIN;
  claims.permissions = {"read", "write"};
  claims.refresh_token_id = "refresh_123";

  auto j = claims.to_json();

  EXPECT_EQ(j["iss"], "test_issuer");
  EXPECT_EQ(j["sub"], "test_subject");
  EXPECT_EQ(j["exp"], 1234567890);
  EXPECT_EQ(j["iat"], 1234567800);
  EXPECT_EQ(j["jti"], "test_jti");
  EXPECT_EQ(j["typ"], "access");
  EXPECT_EQ(j["role"], "admin");
  EXPECT_EQ(j["permissions"].size(), 2);
  EXPECT_EQ(j["refresh_token_id"], "refresh_123");
}

TEST(JwtClaimsTest, ToJsonRefreshToken) {
  JwtClaims claims;
  claims.iss = "test_issuer";
  claims.sub = "test_subject";
  claims.exp = 1234567890;
  claims.iat = 1234567800;
  claims.jti = "test_jti";
  claims.typ = TokenType::REFRESH;
  claims.role = UserRole::OPERATOR;

  auto j = claims.to_json();

  EXPECT_EQ(j["typ"], "refresh");
  EXPECT_EQ(j["role"], "operator");
}

TEST(JwtClaimsTest, FromJson) {
  nlohmann::json j = {{"iss", "test_issuer"}, {"sub", "test_subject"},   {"exp", 1234567890},
                      {"iat", 1234567800},    {"jti", "test_jti"},       {"typ", "access"},
                      {"role", "operator"},   {"permissions", {"read"}}, {"refresh_token_id", "refresh_456"}};

  auto claims = JwtClaims::from_json(j);

  EXPECT_EQ(claims.iss, "test_issuer");
  EXPECT_EQ(claims.sub, "test_subject");
  EXPECT_EQ(claims.exp, 1234567890);
  EXPECT_EQ(claims.iat, 1234567800);
  EXPECT_EQ(claims.jti, "test_jti");
  EXPECT_EQ(claims.typ, TokenType::ACCESS);
  EXPECT_EQ(claims.role, UserRole::OPERATOR);
  EXPECT_EQ(claims.permissions.size(), 1);
  EXPECT_TRUE(claims.refresh_token_id.has_value());
  EXPECT_EQ(claims.refresh_token_id.value(), "refresh_456");
}

TEST(JwtClaimsTest, FromJsonRefreshToken) {
  nlohmann::json j = {{"iss", "test_issuer"}, {"sub", "test_subject"}, {"exp", 1234567890}, {"iat", 1234567800},
                      {"jti", "test_jti"},    {"typ", "refresh"},      {"role", "admin"}};

  auto claims = JwtClaims::from_json(j);

  EXPECT_EQ(claims.typ, TokenType::REFRESH);
  EXPECT_EQ(claims.role, UserRole::ADMIN);
}

TEST(JwtClaimsTest, FromJsonWithInvalidTypDefaultsToAccess) {
  nlohmann::json j = {{"iss", "test_issuer"}, {"sub", "test_subject"}, {"exp", 1234567890}, {"iat", 1234567800},
                      {"jti", "test_jti"},    {"typ", "unknown_type"}, {"role", "admin"}};

  auto claims = JwtClaims::from_json(j);

  // Should default to ACCESS for backward compatibility
  EXPECT_EQ(claims.typ, TokenType::ACCESS);
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

  ASSERT_TRUE(req.has_value());
  EXPECT_EQ(req->grant_type, "client_credentials");
  EXPECT_TRUE(req->client_id.has_value());
  EXPECT_EQ(req->client_id.value(), "test_client");
  EXPECT_TRUE(req->client_secret.has_value());
  EXPECT_EQ(req->client_secret.value(), "test_secret");
  EXPECT_TRUE(req->scope.has_value());
  EXPECT_EQ(req->scope.value(), "admin");
}

TEST(AuthorizeRequestTest, FromFormDataWithUrlEncoding) {
  std::string form_data = "grant_type=refresh_token&refresh_token=abc%2Bdef%3D123";

  auto req = AuthorizeRequest::from_form_data(form_data);

  ASSERT_TRUE(req.has_value());
  EXPECT_EQ(req->grant_type, "refresh_token");
  EXPECT_TRUE(req->refresh_token.has_value());
  EXPECT_EQ(req->refresh_token.value(), "abc+def=123");
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

// @verifies REQ_INTEROP_086
TEST(AuthorizeRequestTest, ParseRequestJson) {
  std::string content_type = "application/json";
  std::string body = R"({"grant_type": "client_credentials", "client_id": "test", "client_secret": "secret"})";

  auto result = AuthorizeRequest::parse_request(content_type, body);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->grant_type, "client_credentials");
  EXPECT_TRUE(result->client_id.has_value());
  EXPECT_EQ(result->client_id.value(), "test");
}

// @verifies REQ_INTEROP_086
TEST(AuthorizeRequestTest, ParseRequestFormUrlEncoded) {
  std::string content_type = "application/x-www-form-urlencoded";
  std::string body = "grant_type=client_credentials&client_id=test&client_secret=secret";

  auto result = AuthorizeRequest::parse_request(content_type, body);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->grant_type, "client_credentials");
  EXPECT_TRUE(result->client_id.has_value());
  EXPECT_EQ(result->client_id.value(), "test");
}

// @verifies REQ_INTEROP_086
TEST(AuthorizeRequestTest, ParseRequestInvalidContentType) {
  std::string content_type = "text/plain";
  std::string body = "some text";

  auto result = AuthorizeRequest::parse_request(content_type, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().error, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST(AuthorizeRequestTest, ParseRequestInvalidJson) {
  std::string content_type = "application/json";
  std::string body = "{ invalid json }";

  auto result = AuthorizeRequest::parse_request(content_type, body);
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().error, "invalid_request");
}

// @verifies REQ_INTEROP_086
TEST(AuthorizeRequestTest, ParseRequestJsonWithCharset) {
  // Content-Type may include charset
  std::string content_type = "application/json; charset=utf-8";
  std::string body = R"({"grant_type": "client_credentials"})";

  auto result = AuthorizeRequest::parse_request(content_type, body);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->grant_type, "client_credentials");
}

// ============================================================================
// AuthMiddleware tests
// ============================================================================

class AuthMiddlewareTest : public ::testing::Test {
 protected:
  void SetUp() override {
    config_ = AuthConfigBuilder()
                  .with_enabled(true)
                  .with_jwt_secret("test_secret_key_for_middleware_test_12345")
                  .with_algorithm(JwtAlgorithm::HS256)
                  .with_token_expiry(3600)
                  .with_refresh_token_expiry(86400)
                  .with_require_auth_for(AuthRequirement::WRITE)
                  .add_client("test_admin", "admin_secret", UserRole::ADMIN)
                  .add_client("test_viewer", "viewer_secret", UserRole::VIEWER)
                  .build();

    auth_manager_ = std::make_unique<AuthManager>(config_);
    // The middleware delegates the RBAC decision to the manager, and a manager
    // with no table refuses everything - see matcher_fixture_permissions().
    // ADMIN's grant comes from the residual list, which is where the token
    // these tests use gets its reach on a running gateway too.
    auth_manager_->add_route_permissions(matcher_fixture_permissions());
    auth_manager_->add_route_permissions(AuthConfig::residual_route_permissions());
    middleware_ = std::make_unique<AuthMiddleware>(config_, auth_manager_.get());
  }

  AuthConfig config_;
  std::unique_ptr<AuthManager> auth_manager_;
  std::unique_ptr<AuthMiddleware> middleware_;
};

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ExtractBearerToken_ValidToken) {
  auto token = AuthMiddleware::extract_bearer_token("Bearer abc123xyz");
  ASSERT_TRUE(token.has_value());
  EXPECT_EQ(token.value(), "abc123xyz");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ExtractBearerToken_CaseInsensitive) {
  auto token = AuthMiddleware::extract_bearer_token("bearer abc123xyz");
  ASSERT_TRUE(token.has_value());
  EXPECT_EQ(token.value(), "abc123xyz");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ExtractBearerToken_EmptyHeader) {
  auto token = AuthMiddleware::extract_bearer_token("");
  EXPECT_FALSE(token.has_value());
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ExtractBearerToken_InvalidPrefix) {
  auto token = AuthMiddleware::extract_bearer_token("Basic abc123xyz");
  EXPECT_FALSE(token.has_value());
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ExtractBearerToken_EmptyToken) {
  auto token = AuthMiddleware::extract_bearer_token("Bearer ");
  EXPECT_FALSE(token.has_value());
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ProcessGetRequestWithoutAuth) {
  // GET requests don't require auth when require_auth_for=WRITE
  AuthRequest req;
  req.method = "GET";
  req.path = "/api/v1/components";

  auto result = middleware_->process(req);
  EXPECT_TRUE(result.allowed);
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ProcessWriteRequestWithoutAuth) {
  // POST requests require auth when require_auth_for=WRITE
  AuthRequest req;
  req.method = "POST";
  req.path = "/api/v1/components/engine/operations/calibrate";

  auto result = middleware_->process(req);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.status_code, 401);
}

// @verifies REQ_INTEROP_086, REQ_INTEROP_087
TEST_F(AuthMiddlewareTest, ProcessWriteRequestWithValidToken) {
  // Authenticate first to get a valid token
  auto auth_result = auth_manager_->authenticate("test_admin", "admin_secret");
  ASSERT_TRUE(auth_result.has_value());

  // POST request with valid admin token
  AuthRequest req;
  req.method = "POST";
  req.path = "/api/v1/components/engine/operations/calibrate";
  req.authorization_header = "Bearer " + auth_result->access_token;

  auto result = middleware_->process(req);
  EXPECT_TRUE(result.allowed);
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ProcessWriteRequestWithInvalidToken) {
  AuthRequest req;
  req.method = "POST";
  req.path = "/api/v1/components/engine/operations/calibrate";
  req.authorization_header = "Bearer invalid_token_12345";

  auto result = middleware_->process(req);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.status_code, 401);
  EXPECT_FALSE(result.www_authenticate_header.empty());
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, ProcessWriteRequestWithInsufficientPermissions) {
  // Authenticate as viewer
  auto auth_result = auth_manager_->authenticate("test_viewer", "viewer_secret");
  ASSERT_TRUE(auth_result.has_value());

  // Viewer trying to POST (not allowed)
  AuthRequest req;
  req.method = "POST";
  req.path = "/api/v1/components/engine/operations/calibrate";
  req.authorization_header = "Bearer " + auth_result->access_token;

  auto result = middleware_->process(req);
  EXPECT_FALSE(result.allowed);
  EXPECT_EQ(result.status_code, 403);  // Forbidden, not Unauthorized
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, AuthEndpointsNeverRequireAuth) {
  AuthRequest req;
  req.method = "POST";
  req.path = "/api/v1/auth/authorize";
  // No authorization header

  auto result = middleware_->process(req);
  EXPECT_TRUE(result.allowed);  // Auth endpoints are always accessible
}

// @verifies REQ_INTEROP_086
TEST_F(AuthMiddlewareTest, DisabledMiddlewareAllowsAll) {
  AuthConfig disabled_config;
  disabled_config.enabled = false;

  AuthMiddleware disabled_middleware(disabled_config, nullptr);

  AuthRequest req;
  req.method = "POST";
  req.path = "/api/v1/components/engine/operations/calibrate";

  auto result = disabled_middleware.process(req);
  EXPECT_TRUE(result.allowed);
}

// ============================================================================
// AuthRequirementPolicy Tests
// ============================================================================

class AuthRequirementPolicyTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Set up auth requirements for configurable policy testing
    auth_requirements_ = {
        {"/api/v1/version", AuthRequirement::NONE},
        {"/api/v1/health", AuthRequirement::NONE},
        {"/api/v1/components", AuthRequirement::ALL},
        {"/api/v1/components/*", AuthRequirement::ALL},
        {"/api/v1/components/*/data/*", AuthRequirement::WRITE},
        {"/api/v1/admin/*", AuthRequirement::ALL},
    };
  }

  std::unordered_map<std::string, AuthRequirement> auth_requirements_;
};

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, NoAuthPolicyNeverRequiresAuth) {
  NoAuthRequirementPolicy policy;

  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/version"));
  EXPECT_FALSE(policy.requires_authentication("POST", "/api/v1/components/engine/operations"));
  EXPECT_FALSE(policy.requires_authentication("DELETE", "/api/v1/admin/users"));
  EXPECT_FALSE(policy.requires_authentication("PUT", "/anything"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, AllAuthPolicyAlwaysRequiresAuth) {
  AllAuthRequirementPolicy policy;

  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/version"));
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/components"));
  EXPECT_TRUE(policy.requires_authentication("POST", "/api/v1/components/engine/operations"));
  EXPECT_TRUE(policy.requires_authentication("DELETE", "/api/v1/admin/users"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, WriteOnlyPolicyForGetRequests) {
  WriteOnlyAuthRequirementPolicy policy;

  // GET requests don't require auth
  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/version"));
  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/components"));
  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/admin/users"));
  EXPECT_FALSE(policy.requires_authentication("HEAD", "/api/v1/health"));
  EXPECT_FALSE(policy.requires_authentication("OPTIONS", "/api/v1/anything"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, WriteOnlyPolicyForWriteRequests) {
  WriteOnlyAuthRequirementPolicy policy;

  // Write requests require auth
  EXPECT_TRUE(policy.requires_authentication("POST", "/api/v1/components/engine/operations"));
  EXPECT_TRUE(policy.requires_authentication("PUT", "/api/v1/components/engine/config"));
  EXPECT_TRUE(policy.requires_authentication("DELETE", "/api/v1/admin/users"));
  EXPECT_TRUE(policy.requires_authentication("PATCH", "/api/v1/components/engine"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, ConfigurablePolicyExactMatch) {
  ConfigurableAuthRequirementPolicy policy(auth_requirements_);

  // Public endpoints (NONE)
  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/version"));
  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/health"));
  EXPECT_FALSE(policy.requires_authentication("POST", "/api/v1/version"));  // NONE for any method
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, ConfigurablePolicyWildcardMatch) {
  ConfigurableAuthRequirementPolicy policy(auth_requirements_);

  // Wildcard match for /api/v1/components/*
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/components/engine"));
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/components/sensor"));

  // Admin wildcard
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/admin/users"));
  EXPECT_TRUE(policy.requires_authentication("POST", "/api/v1/admin/settings"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, ConfigurablePolicyMultipleWildcards) {
  ConfigurableAuthRequirementPolicy policy(auth_requirements_);

  // /api/v1/components/*/data/* should match
  EXPECT_TRUE(policy.requires_authentication("POST", "/api/v1/components/engine/data/temperature"));
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/components/sensor/data/pressure"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, ConfigurablePolicyUnknownPathsRequireAuth) {
  ConfigurableAuthRequirementPolicy policy(auth_requirements_);

  // Unknown paths default to requiring authentication
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/unknown/path"));
  EXPECT_TRUE(policy.requires_authentication("POST", "/api/v1/secret"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, ConfigurablePolicyLongestMatchWins) {
  // Create a policy where /api/v1/public is public, but /api/v1/public/secret/* requires auth
  std::unordered_map<std::string, AuthRequirement> requirements = {
      {"/api/v1/public", AuthRequirement::NONE},
      {"/api/v1/public/*", AuthRequirement::NONE},
      {"/api/v1/public/secret/*", AuthRequirement::ALL},
  };
  ConfigurableAuthRequirementPolicy policy(requirements);

  // /api/v1/public/anything should be public
  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/public/info"));

  // /api/v1/public/secret/data should require auth (longest match wins)
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/public/secret/data"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, FactoryCreatesNoAuthPolicy) {
  AuthConfig config;
  config.enabled = false;

  auto policy = AuthRequirementPolicyFactory::create(config);
  ASSERT_NE(policy, nullptr);

  // Should be NoAuthRequirementPolicy
  EXPECT_FALSE(policy->requires_authentication("POST", "/api/v1/admin/users"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, FactoryCreatesWriteOnlyFromConfig) {
  AuthConfig config;
  config.enabled = true;
  config.require_auth_for = AuthRequirement::WRITE;

  auto policy = AuthRequirementPolicyFactory::create(config);
  ASSERT_NE(policy, nullptr);

  // Should be WriteOnlyAuthRequirementPolicy
  EXPECT_FALSE(policy->requires_authentication("GET", "/api/v1/version"));
  EXPECT_TRUE(policy->requires_authentication("POST", "/api/v1/admin/users"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, FactoryCreatesAllAuthFromConfig) {
  AuthConfig config;
  config.enabled = true;
  config.require_auth_for = AuthRequirement::ALL;

  auto policy = AuthRequirementPolicyFactory::create(config);
  ASSERT_NE(policy, nullptr);

  // Should be AllAuthRequirementPolicy
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/version"));
  EXPECT_TRUE(policy->requires_authentication("POST", "/api/v1/admin/users"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, PolicyDescriptions) {
  NoAuthRequirementPolicy no_auth;
  AllAuthRequirementPolicy all_auth;
  WriteOnlyAuthRequirementPolicy write_only;
  ConfigurableAuthRequirementPolicy configurable(auth_requirements_);

  EXPECT_FALSE(no_auth.description().empty());
  EXPECT_FALSE(all_auth.description().empty());
  EXPECT_FALSE(write_only.description().empty());
  EXPECT_FALSE(configurable.description().empty());

  // Descriptions should be unique
  EXPECT_NE(no_auth.description(), all_auth.description());
  EXPECT_NE(write_only.description(), configurable.description());
}

// ---------------------------------------------------------------------------
// Refresh record lifetime
// ---------------------------------------------------------------------------

namespace {

/// One admin client, with both expiries set by the test.
std::unique_ptr<AuthManager> make_manager(int access_expiry, int refresh_expiry,
                                          AuthManager::WallClock clock = std::chrono::system_clock::now) {
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                    .with_require_auth_for(AuthRequirement::ALL)
                    .with_token_expiry(access_expiry)
                    .with_refresh_token_expiry(refresh_expiry)
                    .add_client("svc", "svc_secret", UserRole::ADMIN)
                    .build();
  return std::make_unique<AuthManager>(config, std::move(clock));
}

/// A clock that reads `seconds`, which the test moves.
AuthManager::WallClock clock_at(const std::shared_ptr<std::atomic<int64_t>> & seconds) {
  return [seconds] {
    return std::chrono::system_clock::time_point(std::chrono::seconds(seconds->load()));
  };
}

/// A clock that returns `value` and then moves it on by `step` seconds. `last` holds what it returned last.
struct SteppingClock {
  std::shared_ptr<std::atomic<int64_t>> value = std::make_shared<std::atomic<int64_t>>(0);
  std::shared_ptr<std::atomic<int64_t>> step = std::make_shared<std::atomic<int64_t>>(1);
  std::shared_ptr<std::atomic<int64_t>> last = std::make_shared<std::atomic<int64_t>>(0);

  AuthManager::WallClock clock() const {
    return [value = value, step = step, last = last] {
      const int64_t now = value->fetch_add(step->load());
      last->store(now);
      return std::chrono::system_clock::time_point(std::chrono::seconds(now));
    };
  }
};

/// Wall-clock seconds, the unit of every token and record time.
int64_t wall_seconds() {
  return std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

/// The `exp` claim of a token, in wall-clock seconds.
int64_t exp_seconds(const std::string & token) {
  return std::chrono::duration_cast<std::chrono::seconds>(jwt::decode(token).get_expires_at().time_since_epoch())
      .count();
}

/// Polls the wall clock until it reads `target` seconds. False if 30 s of steady time pass first.
bool wait_for_wall_second(int64_t target) {
  const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (wall_seconds() < target) {
    if (std::chrono::steady_clock::now() > give_up) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return true;
}

}  // namespace

// The login path sweeps, so the count after ordinary use is the subject, not a sweep the test calls.
// @verifies REQ_INTEROP_086
TEST(AuthManagerTokenLifetimeTest, RepeatedLoginsDoNotGrowTheStoreWithoutBound) {
  // 1 s is the lowest legal pair: refresh must be >= access.
  auto manager = make_manager(1, 1);

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(manager->authenticate("svc", "svc_secret").has_value());
  }
  const int64_t last_issue = wall_seconds();
  EXPECT_EQ(manager->refresh_token_count(), 5U) << "records should accumulate while they are live";

  // A record expires at issue + 1 and is held one access lifetime more; the sweep drops it at issue + 3.
  ASSERT_TRUE(wait_for_wall_second(last_issue + 3));
  ASSERT_TRUE(manager->authenticate("svc", "svc_secret").has_value());

  EXPECT_EQ(manager->refresh_token_count(), 1U)
      << "the five expired records survived a later authorisation - the sweep is not running";
}

// A refresh-only client reaches no other path that clears the store.
// @verifies REQ_INTEROP_086
TEST(AuthManagerTokenLifetimeTest, ARefreshOnlyWorkloadBoundsTheStore) {
  auto manager = make_manager(1, 1);

  auto first = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(first->refresh_token.has_value());
  auto second = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(manager->refresh_token_count(), 2U);

  // Refresh is the only call. It fails once the token expires, and the sweep must still run.
  const auto give_up = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (manager->refresh_token_count() != 0U && std::chrono::steady_clock::now() < give_up) {
    (void)manager->refresh_access_token(first->refresh_token.value(), "svc");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  EXPECT_EQ(manager->refresh_token_count(), 0U)
      << "a workload that only ever refreshes left expired records in the store";
}

// A revoked record stays until the last access token minted from it has expired.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ARevokedRecordOutlivesTheTokensItWithdraws) {
  // An accepted withdrawn token fails at once. The record checks need the late token live by the wall clock, which
  // jwt-cpp reads; an attempt where it was seen expired, after a forward clock step, shows nothing and runs again.
  for (int attempt = 0; attempt < 3; ++attempt) {
    // 3 s each, the tightest legal pair that leaves a checkable second between the two expiries.
    auto manager = make_manager(3, 3);

    // Issue at the start of a second, so the record's expiry is known to the second.
    ASSERT_TRUE(wait_for_wall_second(wall_seconds() + 1));
    const int64_t issued_at = wall_seconds();
    auto issued = manager->authenticate("svc", "svc_secret");
    ASSERT_TRUE(issued.has_value());
    ASSERT_TRUE(issued->refresh_token.has_value());
    if (wall_seconds() != issued_at) {
      continue;
    }

    // Mint a late access token (expires at issued_at + 5 or later), then revoke its record (expires at issued_at + 3).
    ASSERT_TRUE(wait_for_wall_second(issued_at + 2));
    auto late = manager->refresh_access_token(issued->refresh_token.value(), "svc");
    // jwt-cpp reads sub-second time: the token is expired from the start of its `exp` second.
    if (!late.has_value() && wall_seconds() >= issued_at + 3) {
      continue;
    }
    ASSERT_TRUE(late.has_value()) << "the refresh failed while the refresh token was live: "
                                  << late.error().error_description;
    ASSERT_TRUE(manager->revoke_refresh_token(issued->refresh_token.value(), "svc"));

    // Past the record's own expiry, inside the late token's.
    ASSERT_TRUE(wait_for_wall_second(issued_at + 4));
    const size_t dropped = manager->cleanup_expired_tokens();
    const size_t held = manager->refresh_token_count();
    const auto validation = manager->validate_token(late->access_token);
    ASSERT_FALSE(validation.valid) << "the revocation stopped holding while the token it withdrew was still live";
    const bool seen_expired = validation.error.find("expired") != std::string::npos;
    if (seen_expired && wall_seconds() >= exp_seconds(late->access_token)) {
      continue;
    }

    EXPECT_NE(validation.error.find("revoked"), std::string::npos) << validation.error;
    EXPECT_EQ(dropped, 0U) << "the sweep dropped a revoked record past its own expiry";
    EXPECT_EQ(held, 1U);
    return;
  }
  FAIL() << "the wall clock stepped past the scenario's seconds in every attempt";
}

// A foreign record expires no later than this gateway's refresh lifetime after the revocation.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AForeignRecordIsHeldUntilItsTokenExpiresPlusTheAccessLifetime) {
  // The issuer's refresh lifetime (2 s) is longer than the peer's (1 s). The peer's clock is the test's, so a
  // wall-clock step cannot move the sweep to another second. Revocation reads no token times.
  auto issuer = make_manager(1, 2);
  const auto now = std::make_shared<std::atomic<int64_t>>(wall_seconds());
  auto peer = make_manager(1, 1, clock_at(now));
  auto issued = issuer->authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());
  const int64_t exp = exp_seconds(issued->refresh_token.value());

  ASSERT_TRUE(peer->revoke_refresh_token(issued->refresh_token.value(), "svc"));
  EXPECT_EQ(peer->refresh_token_count(), 1U);

  // Kept to exp + 1, past the peer's own refresh lifetime: no sweep drops it while the clock reads exp + 1.
  now->store(exp + 1);
  EXPECT_EQ(peer->cleanup_expired_tokens(), 0U) << "a foreign record was cut at this gateway's refresh lifetime";
  EXPECT_EQ(peer->refresh_token_count(), 1U);

  // Dropped once exp + 1 has passed.
  now->store(exp + 2);
  EXPECT_EQ(peer->cleanup_expired_tokens(), 1U);
  EXPECT_EQ(peer->refresh_token_count(), 0U);
}

// A foreign revocation is recorded only for a subject in this gateway's client table.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AForeignRevocationOfAnUnknownSubjectWritesNothing) {
  // Same secret and issuer as make_manager, another client id.
  auto stranger_issuer =
      std::make_unique<AuthManager>(AuthConfigBuilder()
                                        .with_enabled(true)
                                        .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                                        .with_require_auth_for(AuthRequirement::ALL)
                                        .with_token_expiry(3600)
                                        .with_refresh_token_expiry(86400)
                                        .add_client("stranger", "stranger_secret", UserRole::ADMIN)
                                        .build());
  auto known_issuer = make_manager(3600, 86400);
  auto peer = make_manager(3600, 86400);

  auto stranger = stranger_issuer->authenticate("stranger", "stranger_secret");
  ASSERT_TRUE(stranger.has_value());
  ASSERT_TRUE(stranger->refresh_token.has_value());
  (void)peer->revoke_refresh_token(stranger->refresh_token.value(), "stranger");
  EXPECT_EQ(peer->refresh_token_count(), 0U) << "a revocation for a subject this gateway does not list wrote a record";

  // The same revocation for a listed subject writes one record.
  auto known = known_issuer->authenticate("svc", "svc_secret");
  ASSERT_TRUE(known.has_value());
  ASSERT_TRUE(known->refresh_token.has_value());
  EXPECT_TRUE(peer->revoke_refresh_token(known->refresh_token.value(), "svc"));
  EXPECT_EQ(peer->refresh_token_count(), 1U);
  EXPECT_FALSE(peer->validate_token(known->access_token).valid);
}

// A revoked record minted here is kept only until the access tokens minted from it have expired.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ALocalRevokedRecordIsHeldOnlyForTheAccessLifetime) {
  auto manager = make_manager(1, 3600);
  auto issued = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());

  ASSERT_TRUE(manager->revoke_refresh_token(issued->refresh_token.value(), "svc"));
  const int64_t revoked_at = wall_seconds();
  EXPECT_EQ(manager->cleanup_expired_tokens(), 0U) << "dropped while an access token minted from it can be live";
  EXPECT_EQ(manager->refresh_token_count(), 1U);

  // Kept to revoked_at + 1 at the latest, so a login sweeps it once the clock reads revoked_at + 2.
  ASSERT_TRUE(wait_for_wall_second(revoked_at + 2));
  ASSERT_TRUE(manager->authenticate("svc", "svc_secret").has_value());
  EXPECT_EQ(manager->refresh_token_count(), 1U) << "the revoked record outlived the access lifetime";
  EXPECT_FALSE(manager->refresh_access_token(issued->refresh_token.value(), "svc").has_value())
      << "the revoked refresh token refreshed once its record was gone";
}

// A jti names one record; a revocation carrying it for another subject leaves the record alone.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ARevocationWithAnotherSubjectLeavesTheRecord) {
  auto manager = make_manager(3600, 86400);
  // Listed, so only the subject check can refuse its revocation.
  ASSERT_TRUE(manager->register_client("another_client", "another_secret", UserRole::VIEWER));
  auto issued = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());

  // Signed with the same key and naming the same jti, for another subject.
  const auto now = std::chrono::system_clock::now();
  const std::string forged = jwt::create()
                                 .set_type("refresh")
                                 .set_issuer("ros2_medkit_gateway")
                                 .set_subject("another_client")
                                 .set_id(jwt::decode(issued->refresh_token.value()).get_id())
                                 .set_issued_at(now)
                                 .set_expires_at(now + std::chrono::hours(1))
                                 .sign(jwt::algorithm::hs256{"expiry_sweep_secret_key_at_least_32_chars_long"});

  EXPECT_FALSE(manager->revoke_refresh_token(forged, "another_client"));
  EXPECT_TRUE(manager->validate_token(issued->access_token).valid) << "another subject revoked this client's record";
  EXPECT_TRUE(manager->refresh_access_token(issued->refresh_token.value(), "svc").has_value());

  // The token's own subject does revoke it.
  EXPECT_TRUE(manager->revoke_refresh_token(issued->refresh_token.value(), "svc"));
  EXPECT_FALSE(manager->validate_token(issued->access_token).valid);
}

// An access token names no refresh record: revoking one is refused and records nothing.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, RevokingAnAccessTokenRecordsNothing) {
  auto manager = make_manager(3600, 86400);
  auto issued = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  const size_t held = manager->refresh_token_count();

  const auto outcome = manager->try_revoke_refresh_token(issued->access_token, "svc");
  EXPECT_EQ(manager->refresh_token_count(), held);
  ASSERT_FALSE(outcome.has_value());
  EXPECT_EQ(outcome.error().error, "unsupported_token_type");
}

// A refresh that reads the record before a revocation mints nothing that outlives the revoked record.
// In-process: the interleaving cannot be forced over HTTP.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ARefreshRacingARevokeMintsNothingThatOutlivesTheRecord) {
  // The highest cap, so the revoked records of every iteration fit.
  AuthManager manager(AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                          .with_require_auth_for(AuthRequirement::ALL)
                          .with_token_expiry(3600)
                          .with_refresh_token_expiry(86400)
                          .with_max_refresh_tokens_per_client(100000)
                          .add_client("svc", "svc_secret", UserRole::ADMIN)
                          .build());
  int minted = 0;
  for (int i = 0; i < 200; ++i) {
    auto issued = manager.authenticate("svc", "svc_secret");
    ASSERT_TRUE(issued.has_value());
    const std::string refresh = issued->refresh_token.value();
    const std::string jti = jwt::decode(refresh).get_id();

    std::atomic<bool> go{false};
    tl::expected<TokenResponse, AuthErrorResponse> refreshed = tl::unexpected(AuthErrorResponse::invalid_grant(""));
    std::thread refresher([&] {
      while (!go.load()) {
      }
      refreshed = manager.refresh_access_token(refresh, "svc");
    });
    std::thread revoker([&] {
      while (!go.load()) {
      }
      (void)manager.revoke_refresh_token(refresh, "svc");
    });
    go.store(true);
    refresher.join();
    revoker.join();

    const auto kept_until = manager.refresh_record_kept_until(jti);
    ASSERT_TRUE(kept_until.has_value()) << "iteration " << i << ": the revoked record is gone";
    if (!refreshed.has_value()) {
      continue;
    }
    ++minted;
    EXPECT_LE(exp_seconds(refreshed->access_token), *kept_until)
        << "iteration " << i << ": a minted access token outlives the revoked record";
  }
  std::cout << "refreshes that minted: " << minted << " of 200" << '\n';
  EXPECT_GT(minted, 0) << "no refresh minted, so no minted token was checked against its record";
}

// A revoked record outlives every access token minted from it, also when the clock stepped back before the revoke.
// In-process: a real process's wall clock cannot be moved without root. validate_token checks expiry against the
// system clock, and the test clock stays within the access lifetime of it.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AClockSteppedBackBeforeARevokeKeepsTheRecordPastEveryMintedToken) {
  const auto now = std::make_shared<std::atomic<int64_t>>(wall_seconds());
  auto manager = make_manager(3600, 86400, clock_at(now));
  const auto jti_of = [](const std::string & token) {
    return jwt::decode(token).get_id();
  };

  // A refreshed access token, then the clock steps back 10 s before the revoke.
  auto issued = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  const std::string refresh = issued->refresh_token.value();
  auto refreshed = manager->refresh_access_token(refresh, "svc");
  ASSERT_TRUE(refreshed.has_value());
  const int64_t refreshed_exp = exp_seconds(refreshed->access_token);
  now->fetch_sub(10);
  ASSERT_TRUE(manager->revoke_refresh_token(refresh, "svc"));
  const auto kept = manager->refresh_record_kept_until(jti_of(refresh));
  ASSERT_TRUE(kept.has_value());
  EXPECT_GE(*kept, refreshed_exp) << "the record goes before the refreshed access token expires";

  // Past the horizon a revoke-time clock reading gives (refreshed_exp - 10), before the token expires.
  now->store(refreshed_exp - 5);
  (void)manager->cleanup_expired_tokens();
  EXPECT_FALSE(manager->validate_token(refreshed->access_token).valid)
      << "the refreshed access token came back after the sweep";

  // The access token minted at login, with no refresh.
  now->store(wall_seconds());
  auto login = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(login.has_value());
  const int64_t login_exp = exp_seconds(login->access_token);
  now->fetch_sub(10);
  ASSERT_TRUE(manager->revoke_refresh_token(login->refresh_token.value(), "svc"));
  const auto login_kept = manager->refresh_record_kept_until(jti_of(login->refresh_token.value()));
  ASSERT_TRUE(login_kept.has_value());
  EXPECT_GE(*login_kept, login_exp) << "the record goes before the login access token expires";
  now->store(login_exp - 5);
  (void)manager->cleanup_expired_tokens();
  EXPECT_FALSE(manager->validate_token(login->access_token).valid) << "the login access token came back";

  // A steady clock: kept exactly until the last minted exp, and dropped once it has passed.
  now->store(wall_seconds());
  auto steady = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(steady.has_value());
  const std::string steady_refresh = steady->refresh_token.value();
  auto steady_refreshed = manager->refresh_access_token(steady_refresh, "svc");
  ASSERT_TRUE(steady_refreshed.has_value());
  const int64_t last_exp = std::max(exp_seconds(steady->access_token), exp_seconds(steady_refreshed->access_token));
  ASSERT_TRUE(manager->revoke_refresh_token(steady_refresh, "svc"));
  EXPECT_EQ(manager->refresh_record_kept_until(jti_of(steady_refresh)), std::optional<int64_t>(last_exp));
  now->store(last_exp);
  (void)manager->cleanup_expired_tokens();
  EXPECT_TRUE(manager->refresh_record_kept_until(jti_of(steady_refresh)).has_value()) << "dropped at its last exp";
  now->store(last_exp + 1);
  (void)manager->cleanup_expired_tokens();
  EXPECT_FALSE(manager->refresh_record_kept_until(jti_of(steady_refresh)).has_value())
      << "kept after the last minted access token expired";
}

// A revocation holds through a clock that jumps forward past the access token's exp and comes back.
// In-process: a real process's wall clock cannot be moved without root.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AClockJumpingForwardAndBackKeepsTheRevocation) {
  const auto now = std::make_shared<std::atomic<int64_t>>(wall_seconds());
  const int64_t start = now->load();
  auto manager = make_manager(3600, 86400, clock_at(now));
  auto login = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(login.has_value());
  const std::string refresh = login->refresh_token.value();
  const int64_t access_exp = exp_seconds(login->access_token);
  auto kept_login = manager->authenticate("svc", "svc_secret");
  ASSERT_TRUE(kept_login.has_value());

  // Forward past the access token's exp, revoke and sweep there, then back inside its lifetime.
  now->store(start + 3600 + 1);
  ASSERT_TRUE(manager->revoke_refresh_token(refresh, "svc"));
  (void)manager->cleanup_expired_tokens();
  now->store(start + 3600 / 2);
  (void)manager->cleanup_expired_tokens();

  const auto kept = manager->refresh_record_kept_until(jwt::decode(refresh).get_id());
  ASSERT_TRUE(kept.has_value()) << "the sweep dropped the revoked record while the clock was ahead";
  EXPECT_GE(*kept, access_exp);
  const auto validation = manager->validate_token(login->access_token);
  EXPECT_FALSE(validation.valid) << "the login access token came back when the clock returned";
  EXPECT_NE(validation.error.find("revoked"), std::string::npos) << validation.error;
  EXPECT_TRUE(manager->validate_token(kept_login->access_token).valid) << "an unrevoked access token was refused";
}

// Retry-After is read from the clock reading the sweep used, so retrying then gets through.
// In-process: the test clock moves between reads.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ARetryAfterMatchesTheClockTheSweepRead) {
  SteppingClock clock;
  clock.value->store(wall_seconds());
  AuthManager manager(AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                          .with_require_auth_for(AuthRequirement::ALL)
                          .with_token_expiry(60)
                          .with_refresh_token_expiry(600)
                          .with_max_refresh_tokens_per_client(1)
                          .add_client("svc", "svc_secret", UserRole::ADMIN)
                          .build(),
                      clock.clock());

  // 429: the client's one login slot holds a revoked record.
  auto first = manager.authenticate("svc", "svc_secret");
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(manager.revoke_refresh_token(first->refresh_token.value(), "svc"));
  const auto slot_until = manager.refresh_record_kept_until(jwt::decode(first->refresh_token.value()).get_id());
  ASSERT_TRUE(slot_until.has_value());
  auto refused = manager.authenticate("svc", "svc_secret");
  ASSERT_FALSE(refused.has_value());
  ASSERT_TRUE(refused.error().retry_after_seconds.has_value());
  const int64_t login_now = clock.last->load();
  EXPECT_EQ(*refused.error().retry_after_seconds, *slot_until + 1 - login_now)
      << "Retry-After at login was read from another clock reading than the sweep's";

  clock.step->store(0);
  const int64_t login_free_at = login_now + *refused.error().retry_after_seconds;
  clock.value->store(login_free_at - 1);
  EXPECT_FALSE(manager.authenticate("svc", "svc_secret").has_value()) << "a login got through before Retry-After";
  clock.value->store(login_free_at);
  EXPECT_TRUE(manager.authenticate("svc", "svc_secret").has_value()) << "a login at Retry-After was refused";

  // 503: the client's foreign pool of one is full.
  auto issuer = make_manager(60, 600);
  auto foreign_first = issuer->authenticate("svc", "svc_secret");
  auto foreign_second = issuer->authenticate("svc", "svc_secret");
  ASSERT_TRUE(foreign_first.has_value());
  ASSERT_TRUE(foreign_second.has_value());
  clock.step->store(1);
  const auto recorded = manager.try_revoke_refresh_token(foreign_first->refresh_token.value(), "svc");
  ASSERT_TRUE(recorded.has_value() && *recorded);
  const auto pool_until = manager.refresh_record_kept_until(jwt::decode(foreign_first->refresh_token.value()).get_id());
  ASSERT_TRUE(pool_until.has_value());
  const auto full = manager.try_revoke_refresh_token(foreign_second->refresh_token.value(), "svc");
  ASSERT_FALSE(full.has_value());
  ASSERT_TRUE(full.error().retry_after_seconds.has_value());
  const int64_t revoke_now = clock.last->load();
  EXPECT_EQ(*full.error().retry_after_seconds, *pool_until + 1 - revoke_now)
      << "Retry-After at revoke was read from another clock reading than the sweep's";

  clock.step->store(0);
  const int64_t pool_free_at = revoke_now + *full.error().retry_after_seconds;
  clock.value->store(pool_free_at - 1);
  EXPECT_FALSE(manager.try_revoke_refresh_token(foreign_second->refresh_token.value(), "svc").has_value())
      << "a revocation got through before Retry-After";
  clock.value->store(pool_free_at);
  const auto through = manager.try_revoke_refresh_token(foreign_second->refresh_token.value(), "svc");
  ASSERT_TRUE(through.has_value()) << "a revocation at Retry-After was refused";
  EXPECT_TRUE(*through);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
