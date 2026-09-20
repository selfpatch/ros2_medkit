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
#include <memory>
#include <thread>
#include <vector>

#include "ros2_medkit_gateway/core/auth/auth.hpp"
#include "ros2_medkit_gateway/core/http/rate_limiter.hpp"

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
  auto refresh_result = auth_manager_->refresh_access_token(auth_result->access_token);
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
  auth_manager_->revoke_refresh_token(auth_result->refresh_token.value());

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

// An access token keeps its promised lifetime after its refresh token expires.
//
// validate_token() refuses an access token whose refresh record is gone, which
// is what makes a revocation survive. The cost is that the sweep decides how
// long an access token really lives: sweeping on the refresh token's own
// expiry would cut short the last access token minted from it, which was
// promised a full token_expiry_seconds a moment earlier. This is the test that
// fails if the grace period is removed.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRequirementTest, AccessTokenOutlivesItsExpiredRefreshRecord) {
  // The two expiries are equal, which is the tightest the builder allows
  // (refresh must be >= access). That is also the worst case: refresh_access_token
  // reuses the refresh token's jti and does not rotate it, so the access token
  // it mints is promised token_expiry_seconds from NOW while the record still
  // dies at its original expiry. Every refresh therefore produces an access
  // token that outlives its own record.
  // Three seconds, not one. Two constraints set these numbers.
  //
  // Expiries are whole seconds, so the sweep's comparison only moves at second
  // boundaries: with a one-second expiry and a 1.3 s wait, `expires_at < now`
  // is still false through integer truncation, and the test would pass with or
  // without the grace period - measuring nothing.
  //
  // And the waits are wall-clock on a machine running the rest of the suite, so
  // each one has to sit well clear of the boundary it is about, and never just
  // past it. The record expires at t+3 and is swept after t+6; the checks are
  // at t+4 and t+9, leaving 2 s and 3 s of slack for a late wake-up.
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("test_secret_key_min_32_chars_life_")
                          .with_token_expiry(3)
                          .with_refresh_token_expiry(3)
                          .with_require_auth_for(AuthRequirement::ALL)
                          .build();
  config.clients.push_back({"c", "s", UserRole::ADMIN, true});

  AuthManager manager(config);
  ASSERT_TRUE(manager.authenticate("c", "s").has_value());
  ASSERT_EQ(manager.refresh_token_count(), 1u);

  // t+4s: past the record's own expiry (t+3), inside the one access-token
  // lifetime it is held for (to t+6). Sweeping here is what cut an access token
  // short.
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  manager.cleanup_expired_tokens();
  EXPECT_EQ(manager.refresh_token_count(), 1u)
      << "the record was dropped at its own expiry, so any access token minted "
         "from it in its last moments is refused with most of its life left";

  // t+9s: past the grace too. It does not live forever, or a revocation would
  // be honoured out of a map that only ever grows.
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  manager.cleanup_expired_tokens();
  EXPECT_EQ(manager.refresh_token_count(), 0u) << "the record outlived even its grace period";
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

// ---------------------------------------------------------------------------
// Refresh-record growth, constant-time secret comparison, and revocation.
// ---------------------------------------------------------------------------

namespace {

/// A manager with a single admin client, parameterised on the two expiry
/// values, so a test can put them at their endpoints, away from one
/// comfortable middle value.
AuthManager make_manager(int access_expiry, int refresh_expiry) {
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                    .with_require_auth_for(AuthRequirement::ALL)
                    .with_token_expiry(access_expiry)
                    .with_refresh_token_expiry(refresh_expiry)
                    .add_client("svc", "svc_secret", UserRole::ADMIN)
                    .build();
  return AuthManager(config);
}

}  // namespace

// The store is bounded by the sweep the authorisation path runs, so what this
// asserts is the COUNT after ordinary use. A sweep called directly by a test
// returns the right answer whether or not production ever reaches it, which is
// why the count and not the return value is the subject.
// @verifies REQ_INTEROP_086
TEST(AuthManagerTokenLifetimeTest, RepeatedLoginsDoNotGrowTheStoreWithoutBound) {
  // Refresh expiry at its minimum legal value: validate() requires
  // refresh >= access, so this is the endpoint, not a convenient number.
  auto manager = make_manager(1, 1);

  for (int i = 0; i < 5; ++i) {
    ASSERT_TRUE(manager.authenticate("svc", "svc_secret").has_value());
  }
  EXPECT_EQ(manager.refresh_token_count(), 5U) << "records should accumulate while they are live";

  // Past the refresh expiry AND the access-token lifetime a record is held for
  // beyond it, the next authorisation must clear them out. Records expire at
  // t+1 and are swept after t+2, so this waits to t+4: far enough clear of the
  // boundary that a late wake-up on a loaded machine cannot land short of it.
  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  ASSERT_TRUE(manager.authenticate("svc", "svc_secret").has_value());

  EXPECT_EQ(manager.refresh_token_count(), 1U)
      << "the five expired records survived a later authorisation - the sweep is not running";
}

// The other endpoint. A long-lived refresh token must NOT be swept: an
// over-eager sweep would log clients out mid-session, which is the opposite
// failure and just as real.
// @verifies REQ_INTEROP_086
TEST(AuthManagerTokenLifetimeTest, LongLivedRecordsAreNotSweptEarly) {
  auto manager = make_manager(1, 86400);

  for (int i = 0; i < 4; ++i) {
    ASSERT_TRUE(manager.authenticate("svc", "svc_secret").has_value());
  }
  std::this_thread::sleep_for(std::chrono::seconds(2));
  ASSERT_TRUE(manager.authenticate("svc", "svc_secret").has_value());

  EXPECT_EQ(manager.refresh_token_count(), 5U) << "records well inside their expiry were discarded";
}

// Degenerate case: access and refresh expiry equal and both large.
// @verifies REQ_INTEROP_086
TEST(AuthManagerTokenLifetimeTest, EqualAccessAndRefreshExpiryKeepsRecords) {
  auto manager = make_manager(3600, 3600);
  ASSERT_TRUE(manager.authenticate("svc", "svc_secret").has_value());
  ASSERT_TRUE(manager.authenticate("svc", "svc_secret").has_value());
  EXPECT_EQ(manager.refresh_token_count(), 2U);
}

// A wrong secret must be refused whatever its shape. The interesting inputs
// are the ones a short-circuiting comparison treats differently from a
// constant-time one: a correct prefix, and a value that extends the real one.
// @verifies REQ_INTEROP_086
TEST(AuthManagerSecretComparisonTest, OnlyTheExactSecretAuthenticates) {
  auto manager = make_manager(3600, 3600);

  EXPECT_TRUE(manager.authenticate("svc", "svc_secret").has_value()) << "the real secret must work";

  // The last two are the ones that matter. Everything before them differs in
  // length, or in the first byte, so a comparison that checked the length and
  // then only a prefix would satisfy the whole list. "svc_secreT" differs only
  // in the FINAL byte: shorten the comparison loop by one and it is accepted
  // while every other case here still fails correctly.
  for (const auto & wrong :
       {"", "s", "svc_secre", "svc_secret_", "svc_secretX", "SVC_SECRET", "xxxxxxxxxx", "svc_secreT", "Svc_secret"}) {
    EXPECT_FALSE(manager.authenticate("svc", wrong).has_value()) << "secret \"" << wrong << "\" was accepted";
  }
}

// The denylist half: a record held and marked revoked refuses the access
// tokens minted from it. This is the whole of what the record store decides,
// so it is the half that must not regress.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ARevokedRecordRefusesTheAccessTokenMintedFromIt) {
  auto manager = make_manager(3600, 3600);
  auto issued = manager.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());

  EXPECT_TRUE(manager.validate_token(issued->access_token).valid)
      << "the token must be valid while its record is present and not revoked";

  ASSERT_TRUE(issued->refresh_token.has_value());
  ASSERT_TRUE(manager.revoke_refresh_token(issued->refresh_token.value()));
  EXPECT_FALSE(manager.validate_token(issued->access_token).valid)
      << "an access token whose refresh record was revoked was still accepted";
}

// A second manager standing in for the same gateway after a restart: same
// secret and issuer, so the signature still verifies, and no records, because
// they lived in the memory of the process that is gone.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ARestartKeepsAcceptingATokenItCanStillVerify) {
  auto before = make_manager(3600, 3600);
  auto issued = before.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(before.validate_token(issued->access_token).valid);

  auto after_restart = make_manager(3600, 3600);
  EXPECT_TRUE(after_restart.validate_token(issued->access_token).valid)
      << "a token that verifies under the configured secret and is inside its expiry was refused "
         "because this process holds no record of issuing it";
}

// The cross-instance case the same rule has to serve: two gateways sharing a
// JWT configuration, which is the deployment `aggregation.forward_auth`
// describes. The peer never issued this token and never will hold a record for
// it, so an allowlist would refuse every forwarded request.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, APeerSharingTheJwtConfigurationAcceptsTheOthersToken) {
  auto aggregator = make_manager(3600, 3600);
  auto peer = make_manager(3600, 3600);

  auto issued = aggregator.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());

  EXPECT_TRUE(peer.validate_token(issued->access_token).valid)
      << "a peer sharing the secret, the issuer and the client refused a token the aggregator "
         "minted - forward_auth cannot work against such a peer";
}

// The control for the two above: nothing here is accepting tokens blindly. A
// manager configured with a different secret refuses the same token, so the
// acceptances are the signature verifying and not a check that stopped running.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AGatewayWithAnotherSecretRefusesTheToken) {
  auto issuer = make_manager(3600, 3600);
  auto issued = issuer.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());

  auto stranger_config = AuthConfigBuilder()
                             .with_enabled(true)
                             .with_jwt_secret("a_completely_different_secret_key_at_least_32_chars")
                             .with_require_auth_for(AuthRequirement::ALL)
                             .with_token_expiry(3600)
                             .with_refresh_token_expiry(3600)
                             .add_client("svc", "svc_secret", UserRole::ADMIN)
                             .build();
  AuthManager stranger(stranger_config);

  EXPECT_FALSE(stranger.validate_token(issued->access_token).valid)
      << "a gateway that shares no secret with the issuer accepted its token";
}

// The revocation has to outlast the tokens it withdraws.
//
// The race the grace period closes: an access token minted just before the
// refresh token expires is promised a full access lifetime, so it is still
// live after the record's own expiry has passed. Sweeping on expires_at alone
// would drop the record there and start honouring a withdrawn token again.
//
// Both expiries are three seconds - config validation requires refresh >=
// access, so this is the tightest legal pair - and the check lands at about
// t+4: past the refresh token's expiry, inside the access token's.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, ARevokedRecordOutlivesTheTokensItWithdraws) {
  auto manager = make_manager(3, 3);
  auto issued = manager.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());

  // Mint a late access token, then withdraw the record it came from.
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  auto late = manager.refresh_access_token(issued->refresh_token.value());
  ASSERT_TRUE(late.has_value()) << "the refresh token expired before the late access token was minted";
  ASSERT_TRUE(manager.revoke_refresh_token(issued->refresh_token.value()));

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  EXPECT_EQ(manager.cleanup_expired_tokens(), 0U) << "the sweep dropped a revoked record past its own expiry";
  EXPECT_EQ(manager.refresh_token_count(), 1U);
  EXPECT_FALSE(manager.validate_token(late->access_token).valid)
      << "the revocation stopped holding while the token it withdrew was still live";
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

// Health is NOT special to the ALL policy. It is closed like everything else
// until an operator names it in auth.public_routes, and this is the test that
// fails if somebody hardcodes the exemption back in.
// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, AllAuthPolicyDoesNotExemptHealth) {
  AllAuthRequirementPolicy policy;

  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/health"));
  EXPECT_TRUE(policy.requires_authentication("HEAD", "/api/v1/health"));
}

// An entry of auth.public_routes opens the route it names and nothing beside
// it. Widening the comparison to a prefix, or dropping the method, is the
// natural next edit and would open a hole, so the boundary is pinned here.
// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, PublicRouteExemptionOpensOnlyWhatItNames) {
  auto policy = AuthRequirementPolicyFactory::create(AuthRequirement::ALL, {{"GET", "/api/v1/health"}});

  // The route the operator named.
  EXPECT_FALSE(policy->requires_authentication("GET", "/api/v1/health"));

  // Only GET. A write to the health path is not a liveness probe, and
  // cpp-httplib dispatches HEAD into the GET handler table, so dropping the
  // method check would hand the status document to an anonymous HEAD.
  EXPECT_TRUE(policy->requires_authentication("POST", "/api/v1/health"));
  EXPECT_TRUE(policy->requires_authentication("PUT", "/api/v1/health"));
  EXPECT_TRUE(policy->requires_authentication("DELETE", "/api/v1/health"));
  EXPECT_TRUE(policy->requires_authentication("PATCH", "/api/v1/health"));
  EXPECT_TRUE(policy->requires_authentication("HEAD", "/api/v1/health"));

  // Only that exact path. A prefix or suffix match would hand an attacker a
  // trivial bypass: append or prepend the magic word and walk in.
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/health/detail"));
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/healthz"));
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/components/health"));
  EXPECT_TRUE(policy->requires_authentication("GET", "/health"));
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v2/health"));

  // And the rest of the surface is untouched by the entry.
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/areas"));
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/"));
}

// The layer only ever removes a requirement. Wrapping must not make a gateway
// stricter than the policy underneath, or an operator who adds a probe route
// would silently close the reads that `write` leaves open.
// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, PublicRouteExemptionNeverAddsARequirement) {
  auto policy = AuthRequirementPolicyFactory::create(AuthRequirement::WRITE, {{"POST", "/api/v1/health"}});

  EXPECT_FALSE(policy->requires_authentication("GET", "/api/v1/areas"));
  EXPECT_FALSE(policy->requires_authentication("POST", "/api/v1/health"));
  EXPECT_TRUE(policy->requires_authentication("POST", "/api/v1/areas"));
}

// An empty list must leave the policy exactly as it was, or "closed by
// default" would depend on the wrapper behaving itself.
// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, EmptyPublicRoutesChangesNothing) {
  auto policy = AuthRequirementPolicyFactory::create(AuthRequirement::ALL, {});

  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/health"));
  EXPECT_TRUE(policy->requires_authentication("GET", "/api/v1/areas"));
  EXPECT_FALSE(policy->requires_authentication("POST", "/api/v1/auth/authorize"));
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, PublicRouteEntryParsing) {
  auto ok = parse_public_route("GET /api/v1/health");
  ASSERT_TRUE(ok.has_value());
  EXPECT_EQ(ok->method, "GET");
  EXPECT_EQ(ok->path, "/api/v1/health");

  // Case and surrounding whitespace are the operator's typing, not a decision.
  auto lower = parse_public_route("  get /api/v1/health  ");
  ASSERT_TRUE(lower.has_value());
  EXPECT_EQ(lower->method, "GET");
  EXPECT_EQ(lower->path, "/api/v1/health");

  // Everything below must be refused, and never half-understood. A wildcard
  // accepted and then matched literally would read as "this opens the subtree"
  // and open nothing, which is the worst of both.
  EXPECT_FALSE(parse_public_route("/api/v1/health").has_value());
  EXPECT_FALSE(parse_public_route("GET").has_value());
  EXPECT_FALSE(parse_public_route("").has_value());
  EXPECT_FALSE(parse_public_route("   ").has_value());
  EXPECT_FALSE(parse_public_route("FETCH /api/v1/health").has_value());
  EXPECT_FALSE(parse_public_route("GET api/v1/health").has_value());
  EXPECT_FALSE(parse_public_route("GET /api/v1/*").has_value());
  EXPECT_FALSE(parse_public_route("GET /api/v1/health extra").has_value());
}

// A malformed entry must not quietly open something. Dropping it keeps the
// route protected; GatewayNode refuses to start so the typo is not silent.
// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, MalformedPublicRoutesAreDropped) {
  auto routes = parse_public_routes({"GET /api/v1/health", "nonsense", "GET /api/v1/*"});

  ASSERT_EQ(routes.size(), 1u);
  EXPECT_EQ(routes[0].path, "/api/v1/health");
}

// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, AllAuthPolicyExemptsAuthEndpoints) {
  AllAuthRequirementPolicy policy;

  // Authentication cannot bootstrap through a door that already demands the
  // credential it exists to hand out.
  EXPECT_FALSE(policy.requires_authentication("POST", "/api/v1/auth/authorize"));
  EXPECT_FALSE(policy.requires_authentication("POST", "/api/v1/auth/token"));
  EXPECT_FALSE(policy.requires_authentication("POST", "/api/v1/auth/revoke"));

  // The prefix must be anchored: a path that merely mentions auth later is
  // not an auth endpoint.
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/components/auth/data"));
  EXPECT_TRUE(policy.requires_authentication("GET", "/api/v1/authorization"));
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

// `is_public` answers a narrower question than `!requires_authentication`, and
// a handler that withholds part of its body has to ask the narrow one. Under
// "write" every GET is answered anonymously and NONE of them is public in this
// sense, because nobody named one.
// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, IsPublicNamesOnlyTheListedRoutes) {
  PublicRouteExemptionPolicy policy(std::make_unique<WriteOnlyAuthRequirementPolicy>(),
                                    std::vector<PublicRoute>{PublicRoute{"GET", "/api/v1/health"}});

  EXPECT_TRUE(policy.is_public("GET", "/api/v1/health"));

  // Answered without a credential by the requirement level, and not listed.
  EXPECT_FALSE(policy.requires_authentication("GET", "/api/v1/areas"));
  EXPECT_FALSE(policy.is_public("GET", "/api/v1/areas"))
      << "a route open only because reads are open was reported as named by an operator";

  // Exact matching, same as requires_authentication.
  EXPECT_FALSE(policy.is_public("HEAD", "/api/v1/health"));
  EXPECT_FALSE(policy.is_public("GET", "/api/v1/healthz"));
  EXPECT_FALSE(policy.is_public("GET", "/api/v1/health/"));
}

// Every other policy carries no list, so nothing is public in this sense - not
// even /auth/*, which is open because authentication cannot bootstrap through
// a closed door, not because somebody listed it.
// @verifies REQ_INTEROP_086
TEST_F(AuthRequirementPolicyTest, APolicyWithNoListHasNoPublicRoutes) {
  AllAuthRequirementPolicy all_auth;
  WriteOnlyAuthRequirementPolicy write_only;
  NoAuthRequirementPolicy no_auth;

  for (const IAuthRequirementPolicy * policy : {static_cast<const IAuthRequirementPolicy *>(&all_auth),
                                                static_cast<const IAuthRequirementPolicy *>(&write_only),
                                                static_cast<const IAuthRequirementPolicy *>(&no_auth)}) {
    EXPECT_FALSE(policy->is_public("GET", "/api/v1/health")) << policy->description();
    EXPECT_FALSE(policy->is_public("POST", "/api/v1/auth/authorize")) << policy->description();
  }
}

// The same question through the manager, which is how a handler reaches it.
// @verifies REQ_INTEROP_086
TEST(AuthManagerPublicRouteTest, TheManagerReportsTheOperatorsList) {
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("public_route_probe_secret_key_at_least_32_chars")
                    .with_require_auth_for(AuthRequirement::WRITE)
                    .with_public_routes({"GET /api/v1/health"})
                    .add_client("svc", "svc_secret", UserRole::ADMIN)
                    .build();
  AuthManager manager(config);

  EXPECT_TRUE(manager.is_public_route("GET", "/api/v1/health"));
  EXPECT_FALSE(manager.is_public_route("GET", "/api/v1/areas"));
  EXPECT_FALSE(manager.requires_authentication("GET", "/api/v1/areas"))
      << "the fixture is not in \"write\" mode, so the case it was built for is not being exercised";
}

// With no list at all, which is both shipped profiles.
// @verifies REQ_INTEROP_086
TEST(AuthManagerPublicRouteTest, AnEmptyListMakesNothingPublic) {
  auto config = AuthConfigBuilder()
                    .with_enabled(true)
                    .with_jwt_secret("public_route_probe_secret_key_at_least_32_chars")
                    .with_require_auth_for(AuthRequirement::WRITE)
                    .add_client("svc", "svc_secret", UserRole::ADMIN)
                    .build();
  AuthManager manager(config);

  EXPECT_FALSE(manager.is_public_route("GET", "/api/v1/health"));
  EXPECT_FALSE(manager.is_public_route("GET", "/api/v1/areas"));
}

// The ordering rule the pre-routing handler applies, swept over its whole
// input space - three booleans, eight cases. Only one of them may skip the
// verifier, and the "no header" row is the one that must not: an anonymous
// caller keeps the 401 every other anonymous caller gets.
// @verifies REQ_INTEROP_086
TEST(RateLimitOrderingTest, OnlyAnExhaustedCallerWithACredentialSkipsTheVerifier) {
  struct Case {
    bool rate_limited;
    bool has_header;
    bool route_protected;
    bool expected;
  };
  const Case cases[] = {
      {false, false, false, false}, {false, false, true, false}, {false, true, false, false},
      {false, true, true, false},   {true, false, false, false}, {true, false, true, false},
      {true, true, false, false},   {true, true, true, true},
  };

  for (const auto & c : cases) {
    EXPECT_EQ(AuthMiddleware::rate_limit_precedes_validation(c.rate_limited, c.has_header, c.route_protected),
              c.expected)
        << "rate_limited=" << c.rate_limited << " has_header=" << c.has_header
        << " route_protected=" << c.route_protected;
  }
}

// The instrument for the claim, checked against itself: the counter has to
// move when a token IS verified, or a test that sees it stay put proves
// nothing.
// @verifies REQ_INTEROP_086
TEST(RateLimitOrderingTest, TheValidationCounterMovesWhenTheVerifierRuns) {
  auto manager = make_manager(3600, 3600);
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                          .with_require_auth_for(AuthRequirement::ALL)
                          .add_client("svc", "svc_secret", UserRole::ADMIN)
                          .build();
  AuthMiddleware middleware(config, &manager);

  const size_t before = manager.token_validation_count();

  AuthRequest anonymous;
  anonymous.method = "GET";
  anonymous.path = "/api/v1/areas";
  EXPECT_FALSE(middleware.process(anonymous).allowed);
  EXPECT_EQ(manager.token_validation_count(), before) << "a request with no Authorization header reached the verifier";

  AuthRequest garbage = anonymous;
  garbage.authorization_header = "Bearer not-a-token";
  EXPECT_FALSE(middleware.process(garbage).allowed);
  EXPECT_EQ(manager.token_validation_count(), before + 1)
      << "a bearer that reached process() was not counted, so the counter cannot witness a skipped verify";

  // And the route question the ordering rule asks, answered by the same object
  // the middleware uses.
  EXPECT_TRUE(middleware.requires_authentication(anonymous));
  AuthRequest auth_route = anonymous;
  auth_route.path = "/api/v1/auth/authorize";
  EXPECT_FALSE(middleware.requires_authentication(auth_route));
}

// Revocation has to work on a gateway that never issued the token, or it does
// not work at all under a shared JWT configuration: a peer holds no record of
// anything the aggregator minted, and with the records read as a denylist
// "no record" would make revoke a no-op on exactly the gateway being locked
// down.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, RevokingAForeignTokenRefusesItHere) {
  auto issuer = make_manager(3600, 3600);
  auto peer = make_manager(3600, 3600);

  auto issued = issuer.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());

  ASSERT_TRUE(peer.validate_token(issued->access_token).valid) << "the peer must accept it before it is revoked";

  EXPECT_TRUE(peer.revoke_refresh_token(issued->refresh_token.value()))
      << "the peer declined to revoke a token minted elsewhere";
  EXPECT_FALSE(peer.validate_token(issued->access_token).valid) << "the peer went on accepting a token revoked on it";

  // Only here. Revocation is per gateway, and the issuer was never told.
  EXPECT_TRUE(issuer.validate_token(issued->access_token).valid)
      << "revoking on the peer reached across to the issuer, which shares no state with it";
}

// The record written for a foreign token expires with the token, so this
// cannot grow past the tokens in flight.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AForeignRevocationRecordIsSweptWithItsToken) {
  auto issuer = make_manager(1, 1);
  auto peer = make_manager(1, 1);

  auto issued = issuer.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());
  ASSERT_TRUE(peer.revoke_refresh_token(issued->refresh_token.value()));
  EXPECT_EQ(peer.refresh_token_count(), 1U);

  std::this_thread::sleep_for(std::chrono::milliseconds(4000));
  EXPECT_EQ(peer.cleanup_expired_tokens(), 1U);
  EXPECT_EQ(peer.refresh_token_count(), 0U);
}

// The role a token carries is what the ISSUING gateway granted. Under a shared
// JWT configuration that is a different gateway, and letting the claim decide
// would export one deployment's grants into another: a client this gateway
// lists as viewer would write here because the issuer listed it as admin.
// @verifies REQ_INTEROP_086, REQ_INTEROP_087
TEST(AuthManagerRoleTest, TheRoleComesFromThisGatewaysClientTable) {
  auto admin_side = AuthConfigBuilder()
                        .with_enabled(true)
                        .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                        .with_require_auth_for(AuthRequirement::ALL)
                        .with_token_expiry(3600)
                        .with_refresh_token_expiry(3600)
                        .add_client("svc", "svc_secret", UserRole::ADMIN)
                        .build();
  auto viewer_side = AuthConfigBuilder()
                         .with_enabled(true)
                         .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                         .with_require_auth_for(AuthRequirement::ALL)
                         .with_token_expiry(3600)
                         .with_refresh_token_expiry(3600)
                         .add_client("svc", "svc_secret", UserRole::VIEWER)
                         .build();

  AuthManager issuer(admin_side);
  AuthManager peer(viewer_side);

  auto issued = issuer.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  EXPECT_EQ(issued->scope, "admin");

  auto on_issuer = issuer.validate_token(issued->access_token);
  ASSERT_TRUE(on_issuer.valid);
  EXPECT_EQ(on_issuer.claims->role, UserRole::ADMIN);

  auto on_peer = peer.validate_token(issued->access_token);
  ASSERT_TRUE(on_peer.valid) << "the peer must still accept the token; only the role it grants differs";
  EXPECT_EQ(on_peer.claims->role, UserRole::VIEWER)
      << "the peer granted the role the token claimed; its own table is what decides here";
}

// A refresh-only workload reaches no other code that would clear the store, so
// the refresh path sweeps too. Without it the map grows for the life of a
// process whose clients authorise once and refresh forever.
// @verifies REQ_INTEROP_086
TEST(AuthManagerTokenLifetimeTest, ARefreshOnlyWorkloadBoundsTheStore) {
  // Both expiries at their minimum, so the sweep point is issue + 2 s and the
  // wait below clears it by three. Timestamps are whole seconds, so two
  // authorisations either side of a second boundary carry expiries a second
  // apart; a margin of one would make the later record's fate depend on where
  // in a second the test happened to start.
  auto manager = make_manager(1, 1);

  auto first = manager.authenticate("svc", "svc_secret");
  ASSERT_TRUE(first.has_value());
  ASSERT_TRUE(first->refresh_token.has_value());

  // A second login, so there is an older record for the sweep to find.
  auto second = manager.authenticate("svc", "svc_secret");
  ASSERT_TRUE(second.has_value());
  ASSERT_TRUE(second->refresh_token.has_value());
  EXPECT_EQ(manager.refresh_token_count(), 2U);

  // Past both refresh expiries and the access lifetime held beyond them.
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // Refreshing is the ONLY call made here. It fails, because the refresh token
  // expired too - and the sweep still has to have run, which is the point.
  (void)manager.refresh_access_token(first->refresh_token.value());

  EXPECT_EQ(manager.refresh_token_count(), 0U)
      << "a workload that only ever refreshes left expired records in the store";
}

// What the predicate and the middleware do together, on a lambda that repeats
// the server's decision order.
//
// This pins two things: that `rate_limit_precedes_validation` selects the rows
// it claims to, and that `process` leaves the verifier untouched on each of
// them. It is a COPY of the order rest_server applies, so it cannot catch that
// file being reordered - the instrument for the server's own order is the
// integration case test_04_an_exhausted_caller_with_a_header_gets_a_bare_429,
// which drives a running gateway.
// @verifies REQ_INTEROP_086
TEST(RateLimitOrderingTest, TheVerifierIsNotReachedOnTheExhaustedHeaderPath) {
  auto manager = make_manager(3600, 3600);
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                          .with_require_auth_for(AuthRequirement::ALL)
                          .add_client("svc", "svc_secret", UserRole::ADMIN)
                          .build();
  AuthMiddleware middleware(config, &manager);

  RateLimitConfig rl_config;
  rl_config.enabled = true;
  rl_config.global_requests_per_minute = 3;
  rl_config.client_requests_per_minute = 3;
  RateLimiter limiter(rl_config);

  // The pre-routing decision, in the order rest_server makes it: meter, then
  // let the limiter answer where it may, then authenticate.
  const auto serve = [&](const std::string & path, std::optional<std::string> header) {
    AuthRequest request;
    request.method = "GET";
    request.path = path;
    request.authorization_header = std::move(header);

    auto rl = limiter.check("10.0.0.7", path);
    const bool rate_limited = !rl.allowed;
    if (rate_limited && request.authorization_header.has_value() &&
        AuthMiddleware::rate_limit_precedes_validation(rate_limited, true,
                                                       middleware.requires_authentication(request))) {
      return 429;
    }
    auto result = middleware.process(request);
    if (!result.allowed) {
      return result.status_code;
    }
    return rate_limited ? 429 : 200;
  };

  const std::string protected_path = "/api/v1/areas";
  const std::string public_path = "/api/v1/auth/authorize";

  // Spend the allowance with requests carrying no credential, so nothing here
  // has verified anything yet.
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ(serve(protected_path, std::nullopt), 401);
  }
  const size_t before = manager.token_validation_count();
  ASSERT_EQ(before, 0U) << "an uncredentialed request reached the verifier";

  // Exhausted, with a header: 429, and the verifier untouched.
  EXPECT_EQ(serve(protected_path, std::string("Bearer not-a-token")), 429);
  EXPECT_EQ(manager.token_validation_count(), before)
      << "the gateway verified a token belonging to a caller it had already refused";

  // Exhausted, no header: the anonymous 401, still no verification.
  EXPECT_EQ(serve(protected_path, std::nullopt), 401);
  EXPECT_EQ(manager.token_validation_count(), before);

  // Exhausted, header, PUBLIC route. The limiter has no special word here, so
  // process() runs - and on a route needing no credential it returns before it
  // looks at the header, so the verifier is still not reached.
  EXPECT_EQ(serve(public_path, std::string("Bearer not-a-token")), 429);
  EXPECT_EQ(manager.token_validation_count(), before) << "a public route put an unverified header through the verifier";
}

// The mirror of the test above: the harness it uses does reach the verifier
// when the allowance is there, so a counter that never moved would not be
// evidence of anything.
// @verifies REQ_INTEROP_086
TEST(RateLimitOrderingTest, TheSameHarnessReachesTheVerifierWithAllowanceLeft) {
  auto manager = make_manager(3600, 3600);
  AuthConfig config = AuthConfigBuilder()
                          .with_enabled(true)
                          .with_jwt_secret("expiry_sweep_secret_key_at_least_32_chars_long")
                          .with_require_auth_for(AuthRequirement::ALL)
                          .add_client("svc", "svc_secret", UserRole::ADMIN)
                          .build();
  AuthMiddleware middleware(config, &manager);

  RateLimitConfig rl_config;
  rl_config.enabled = true;
  rl_config.global_requests_per_minute = 100;
  rl_config.client_requests_per_minute = 100;
  RateLimiter limiter(rl_config);

  AuthRequest request;
  request.method = "GET";
  request.path = "/api/v1/areas";
  request.authorization_header = "Bearer not-a-token";

  auto rl = limiter.check("10.0.0.8", request.path);
  ASSERT_TRUE(rl.allowed);
  EXPECT_FALSE(middleware.process(request).allowed);
  EXPECT_EQ(manager.token_validation_count(), 1U) << "the harness never reaches the verifier at all";
}

// `[""]` is how a ROS 2 YAML file writes an empty string sequence, and both
// shipped profiles use that idiom for auth.clients. A list written that way
// carries one blank entry, means "no routes", and must neither open anything
// nor stop the gateway.
// @verifies REQ_INTEROP_086
TEST(PublicRouteParsingTest, ABlankEntryIsNeitherARouteNorATypo) {
  EXPECT_TRUE(is_blank_public_route_entry(""));
  EXPECT_TRUE(is_blank_public_route_entry("   "));
  EXPECT_TRUE(is_blank_public_route_entry("\t"));
  EXPECT_FALSE(is_blank_public_route_entry("GET /api/v1/health"));
  EXPECT_FALSE(is_blank_public_route_entry("nonsense"));

  EXPECT_TRUE(parse_public_routes({""}).empty());
  EXPECT_TRUE(parse_public_routes({"", "  "}).empty());

  // A blank entry beside a real one leaves the real one standing.
  auto mixed = parse_public_routes({"", "GET /api/v1/health"});
  ASSERT_EQ(mixed.size(), 1U);
  EXPECT_EQ(mixed[0].method, "GET");
  EXPECT_EQ(mixed[0].path, "/api/v1/health");
}

// /auth/revoke takes refresh tokens. An access token's own jti is not a key
// anything reads - validate_token looks up the `refresh_token_id` claim - so
// writing a record under it would store a revocation nothing consults while
// reporting success to the caller who asked for one.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AnAccessTokenIsNotRevocable) {
  auto manager = make_manager(3600, 3600);
  auto issued = manager.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());

  const size_t before = manager.refresh_token_count();
  EXPECT_FALSE(manager.revoke_refresh_token(issued->access_token)) << "an access token was accepted for revocation";
  EXPECT_EQ(manager.refresh_token_count(), before) << "a record was written for an access token";
  EXPECT_TRUE(manager.validate_token(issued->access_token).valid)
      << "the token was reported revoked and went on working, which is the state this refuses to create";

  // The refresh token it came with is still revocable, so the refusal above is
  // about the token TYPE and not about revocation having stopped working.
  ASSERT_TRUE(issued->refresh_token.has_value());
  EXPECT_TRUE(manager.revoke_refresh_token(issued->refresh_token.value()));
  EXPECT_FALSE(manager.validate_token(issued->access_token).valid);
}

// A foreign revocation record is held until the token's refresh expiry plus
// the LOCAL access lifetime, so whether it outlives every token the issuer can
// mint from it depends on how the two gateways' access expiries compare. Both
// directions are pinned: where the peer's is no shorter the revocation holds
// for every token; where it is shorter, a token minted late on the issuer
// outlives the record, which is what the documented rule exists to prevent.
//
// Timestamps are whole seconds, so each sleep below leaves at least half a
// second of margin against where in a second the block happened to start.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AForeignRevocationHoldsOnlyWhereTheExpiriesAgree) {
  // Equal configurations: the record outlives the token.
  {
    auto issuer = make_manager(2, 2);
    auto peer = make_manager(2, 2);
    auto issued = issuer.authenticate("svc", "svc_secret");
    ASSERT_TRUE(issued.has_value());
    ASSERT_TRUE(issued->refresh_token.has_value());
    ASSERT_TRUE(peer.revoke_refresh_token(issued->refresh_token.value()));
    EXPECT_FALSE(peer.validate_token(issued->access_token).valid);
  }

  // The peer's access expiry is LONGER than the issuer's: the record stands
  // past the last moment any token minted from that refresh token can verify.
  // The issuer's refresh token expires one second after issue, so nothing it
  // mints can be valid two and a half seconds later - and the record is held
  // a further three past the refresh expiry.
  {
    auto issuer = make_manager(1, 1);
    auto peer = make_manager(3, 3);
    auto issued = issuer.authenticate("svc", "svc_secret");
    ASSERT_TRUE(issued.has_value());
    ASSERT_TRUE(issued->refresh_token.has_value());
    ASSERT_TRUE(peer.revoke_refresh_token(issued->refresh_token.value()));
    EXPECT_FALSE(peer.validate_token(issued->access_token).valid);

    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    EXPECT_EQ(peer.cleanup_expired_tokens(), 0U)
        << "the peer swept the record while a token the issuer minted could still have been live";
  }

  // The peer's access expiry is SHORTER than the issuer's: the lapse.
  //
  // The issuer's tokens live five seconds and the peer's one. The refresh
  // token is exchanged three and a half seconds after issue, so the access
  // token it mints has five seconds from there; the peer's record, sized by
  // the refresh expiry (five) plus the peer's own access lifetime (one), is
  // gone a second earlier. Between the two the token verifies on the peer
  // with the revocation forgotten.
  {
    auto issuer = make_manager(5, 5);
    auto peer = make_manager(1, 1);
    auto issued = issuer.authenticate("svc", "svc_secret");
    ASSERT_TRUE(issued.has_value());
    ASSERT_TRUE(issued->refresh_token.has_value());
    ASSERT_TRUE(peer.revoke_refresh_token(issued->refresh_token.value()));

    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    auto late = issuer.refresh_access_token(issued->refresh_token.value());
    ASSERT_TRUE(late.has_value()) << "the issuer refused a refresh inside the refresh token's life: "
                                  << late.error().error_description;
    EXPECT_FALSE(peer.validate_token(late->access_token).valid) << "the revocation must hold while the record is up";

    std::this_thread::sleep_for(std::chrono::milliseconds(3500));
    EXPECT_EQ(peer.cleanup_expired_tokens(), 1U) << "the record was sized by something other than the refresh expiry "
                                                    "plus the peer's own access lifetime";
    EXPECT_TRUE(peer.validate_token(late->access_token).valid)
        << "the late token was refused after the record was gone, so the lapse the documented rule guards "
           "against does not exist and the docs overstate it";
  }
}

// A revocation reaches a refresh token past its own expiry.
//
// The last access token minted from a refresh token can outlive it by a whole
// access lifetime, and a revoke that refused the expired refresh token would
// leave that access token unrevocable for exactly that long. On the revoke
// path the signature and the issuer are verified and the expiry is not; the
// record is written either way, and the sweep still bounds it.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AnExpiredRefreshTokenStillRevokesItsAccessTokens) {
  auto issuer = make_manager(4, 4);
  auto peer = make_manager(4, 4);
  auto issued = issuer.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());
  const std::string refresh = issued->refresh_token.value();

  // Exchanged inside the refresh token's life, so the access token it mints
  // has four seconds from here, which is past the refresh expiry.
  std::this_thread::sleep_for(std::chrono::milliseconds(2500));
  auto late = issuer.refresh_access_token(refresh);
  ASSERT_TRUE(late.has_value()) << late.error().error_description;

  // Now the refresh token has expired and the access token has not; both are
  // asserted, so the revocation below is measured on exactly that state.
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  ASSERT_FALSE(issuer.refresh_access_token(refresh).has_value())
      << "the refresh token is still live, so this pins nothing";
  ASSERT_TRUE(issuer.validate_token(late->access_token).valid)
      << "the access token expired first, so this pins nothing";

  EXPECT_TRUE(issuer.revoke_refresh_token(refresh)) << "an expired refresh token was refused for revocation";
  auto on_issuer = issuer.validate_token(late->access_token);
  EXPECT_FALSE(on_issuer.valid);
  EXPECT_NE(on_issuer.error.find("revoked"), std::string::npos) << on_issuer.error;

  // The foreign variant: a peer that never saw the token writes the record.
  EXPECT_TRUE(peer.revoke_refresh_token(refresh)) << "an expired foreign refresh token was refused for revocation";
  auto on_peer = peer.validate_token(late->access_token);
  EXPECT_FALSE(on_peer.valid);
  EXPECT_NE(on_peer.error.find("revoked"), std::string::npos) << on_peer.error;
}

// A foreign record is held for at most this gateway's own refresh lifetime.
//
// The record is sized by the issuer's refresh expiry, which this gateway does
// not control; an issuer with a refresh lifetime of years would otherwise leave
// records here for years. Under the shared-configuration rule the clamp
// changes nothing, which is what makes the rule safe to state.
// @verifies REQ_INTEROP_086
TEST(AuthManagerRevocationTest, AForeignRecordIsHeldNoLongerThanThisGatewaysRefreshLifetime) {
  auto issuer = make_manager(1, 315360000);
  auto peer = make_manager(1, 1);
  auto issued = issuer.authenticate("svc", "svc_secret");
  ASSERT_TRUE(issued.has_value());
  ASSERT_TRUE(issued->refresh_token.has_value());
  ASSERT_TRUE(peer.revoke_refresh_token(issued->refresh_token.value()));
  EXPECT_FALSE(peer.validate_token(issued->access_token).valid);

  // Past the peer's refresh lifetime and its access lifetime on top.
  std::this_thread::sleep_for(std::chrono::milliseconds(3000));
  EXPECT_EQ(peer.cleanup_expired_tokens(), 1U) << "a foreign record outlived this gateway's own refresh lifetime";
}

// A blank entry is any whitespace, the same rule parse_public_route trims by:
// a list written `["\n"]` is as blank as one written `[" "]`.
// @verifies REQ_INTEROP_086
TEST(PublicRouteParsingTest, ABlankEntryIsAnyWhitespace) {
  for (const char * blank : {"", " ", "\t", "\n", "\r\n", "\v", "\f", " \n\t "}) {
    EXPECT_TRUE(is_blank_public_route_entry(blank)) << "entry " << testing::PrintToString(std::string(blank));
    EXPECT_TRUE(parse_public_routes({blank}).empty()) << "entry " << testing::PrintToString(std::string(blank));
  }
  EXPECT_FALSE(is_blank_public_route_entry(" x "));
  EXPECT_FALSE(is_blank_public_route_entry("\nGET /api/v1/health\n"));
}

// A role name carrying a byte at or above 0x80 is refused like any other
// unknown role, and lower-casing it is defined behaviour.
// @verifies REQ_INTEROP_086
TEST(AuthConfigRoleTest, ANonAsciiRoleNameIsRefusedAndDoesNotCrash) {
  for (const auto & name : {"\xC3\xA4"
                            "dmin",
                            "admin\xFF", "\x80", "\xFF\xFE"}) {
    EXPECT_THROW((void)string_to_role(name), std::invalid_argument) << "role \"" << name << "\" was accepted";
  }
  // The ASCII path still works, so the loop above is not passing because
  // everything throws.
  EXPECT_EQ(string_to_role("ADMIN"), UserRole::ADMIN);
}

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
