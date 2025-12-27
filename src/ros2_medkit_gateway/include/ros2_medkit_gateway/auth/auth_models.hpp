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

#pragma once

#include <chrono>
#include <expected>
#include <nlohmann/json.hpp>
#include <optional>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/auth/auth_config.hpp"

namespace ros2_medkit_gateway {

using json = nlohmann::json;

// Forward declarations
struct AuthErrorResponse;

/**
 * @brief JWT token type for distinguishing access vs refresh tokens
 */
enum class TokenType {
  ACCESS,  ///< Short-lived access token for API authorization
  REFRESH  ///< Long-lived refresh token for obtaining new access tokens
};

/**
 * @brief Convert TokenType to string (for JWT typ claim)
 */
inline std::string token_type_to_string(TokenType type) {
  switch (type) {
    case TokenType::ACCESS:
      return "access";
    case TokenType::REFRESH:
      return "refresh";
    default:
      return "unknown";
  }
}

/**
 * @brief Convert string to TokenType
 * @throws std::invalid_argument if string is not a valid token type
 */
inline TokenType string_to_token_type(const std::string & type_str) {
  if (type_str == "access") {
    return TokenType::ACCESS;
  }
  if (type_str == "refresh") {
    return TokenType::REFRESH;
  }
  throw std::invalid_argument("Invalid token type: " + type_str);
}

/**
 * @brief JWT token claims
 * @verifies REQ_INTEROP_087
 */
struct JwtClaims {
  std::string iss;                              ///< Issuer
  std::string sub;                              ///< Subject (client_id)
  int64_t exp{0};                               ///< Expiration time (Unix timestamp)
  int64_t iat{0};                               ///< Issued at time (Unix timestamp)
  std::string jti;                              ///< JWT ID (unique identifier)
  TokenType typ{TokenType::ACCESS};             ///< Token type (access or refresh)
  UserRole role{UserRole::VIEWER};              ///< User role for RBAC
  std::vector<std::string> permissions;         ///< Explicit permissions (optional)
  std::optional<std::string> refresh_token_id;  ///< Associated refresh token ID (for access tokens)

  json to_json() const {
    json j = {{"iss", iss},
              {"sub", sub},
              {"exp", exp},
              {"iat", iat},
              {"jti", jti},
              {"typ", token_type_to_string(typ)},
              {"role", role_to_string(role)}};

    if (!permissions.empty()) {
      j["permissions"] = permissions;
    }

    if (refresh_token_id.has_value()) {
      j["refresh_token_id"] = refresh_token_id.value();
    }

    return j;
  }

  static JwtClaims from_json(const json & j) {
    JwtClaims claims;
    claims.iss = j.value("iss", "");
    claims.sub = j.value("sub", "");
    claims.exp = j.value("exp", int64_t{0});
    claims.iat = j.value("iat", int64_t{0});
    claims.jti = j.value("jti", "");

    if (j.contains("typ")) {
      try {
        claims.typ = string_to_token_type(j["typ"].get<std::string>());
      } catch (const std::invalid_argument &) {
        claims.typ = TokenType::ACCESS;  // Default to access for backward compatibility
      }
    }

    if (j.contains("role")) {
      claims.role = string_to_role(j["role"].get<std::string>());
    }

    if (j.contains("permissions")) {
      claims.permissions = j["permissions"].get<std::vector<std::string>>();
    }

    if (j.contains("refresh_token_id")) {
      claims.refresh_token_id = j["refresh_token_id"].get<std::string>();
    }

    return claims;
  }

  bool is_expired() const {
    auto now = std::chrono::system_clock::now();
    auto exp_time = std::chrono::system_clock::from_time_t(exp);
    return now > exp_time;
  }
};

/**
 * @brief Token response following OAuth2 format
 * @verifies REQ_INTEROP_087
 */
struct TokenResponse {
  std::string access_token;
  std::string token_type{"Bearer"};
  int expires_in{0};  ///< Seconds until expiration
  std::optional<std::string> refresh_token;
  std::string scope;  ///< Role-based scope

  json to_json() const {
    json j = {{"access_token", access_token}, {"token_type", token_type}, {"expires_in", expires_in}, {"scope", scope}};

    if (refresh_token.has_value()) {
      j["refresh_token"] = refresh_token.value();
    }

    return j;
  }
};

/**
 * @brief Authorization request for /auth/authorize endpoint
 * @verifies REQ_INTEROP_086
 */
struct AuthorizeRequest {
  std::string grant_type;  ///< "client_credentials" or "refresh_token"
  std::optional<std::string> client_id;
  std::optional<std::string> client_secret;
  std::optional<std::string> refresh_token;
  std::optional<std::string> scope;  ///< Requested scope/role

  static AuthorizeRequest from_json(const json & j) {
    AuthorizeRequest req;
    req.grant_type = j.value("grant_type", "");

    if (j.contains("client_id")) {
      req.client_id = j["client_id"].get<std::string>();
    }

    if (j.contains("client_secret")) {
      req.client_secret = j["client_secret"].get<std::string>();
    }

    if (j.contains("refresh_token")) {
      req.refresh_token = j["refresh_token"].get<std::string>();
    }

    if (j.contains("scope")) {
      req.scope = j["scope"].get<std::string>();
    }

    return req;
  }

  // Parse from URL-encoded form data (application/x-www-form-urlencoded)
  static AuthorizeRequest from_form_data(const std::string & body);

  /**
   * @brief Parse AuthorizeRequest from HTTP request body with content-type detection
   * @param content_type Content-Type header value
   * @param body Request body
   * @return AuthorizeRequest on success, AuthErrorResponse on failure
   */
  static std::expected<AuthorizeRequest, AuthErrorResponse> parse_request(const std::string & content_type,
                                                                          const std::string & body);
};

/**
 * @brief Result of token validation
 */
struct TokenValidationResult {
  bool valid{false};
  std::string error;
  std::optional<JwtClaims> claims;
};

/**
 * @brief Result of authorization check
 */
struct AuthorizationResult {
  bool authorized{false};
  std::string error;
  std::optional<std::string> required_permission;
};

/**
 * @brief Refresh token storage record
 */
struct RefreshTokenRecord {
  std::string token_id;   ///< Unique ID (jti)
  std::string client_id;  ///< Associated client
  UserRole role;          ///< Role at time of issuance
  int64_t issued_at{0};   ///< Unix timestamp
  int64_t expires_at{0};  ///< Unix timestamp
  bool revoked{false};    ///< Whether token has been revoked
};

/**
 * @brief Error response for OAuth2 errors
 */
struct AuthErrorResponse {
  std::string error;              ///< Error code (e.g., "invalid_grant")
  std::string error_description;  ///< Human-readable description

  json to_json() const {
    return {{"error", error}, {"error_description", error_description}};
  }

  // Standard OAuth2 error codes
  static AuthErrorResponse invalid_request(const std::string & description) {
    return {"invalid_request", description};
  }

  static AuthErrorResponse invalid_client(const std::string & description) {
    return {"invalid_client", description};
  }

  static AuthErrorResponse invalid_grant(const std::string & description) {
    return {"invalid_grant", description};
  }

  static AuthErrorResponse unauthorized_client(const std::string & description) {
    return {"unauthorized_client", description};
  }

  static AuthErrorResponse unsupported_grant_type(const std::string & description) {
    return {"unsupported_grant_type", description};
  }

  static AuthErrorResponse invalid_scope(const std::string & description) {
    return {"invalid_scope", description};
  }

  static AuthErrorResponse access_denied(const std::string & description) {
    return {"access_denied", description};
  }

  static AuthErrorResponse invalid_token(const std::string & description) {
    return {"invalid_token", description};
  }

  static AuthErrorResponse insufficient_scope(const std::string & description) {
    return {"insufficient_scope", description};
  }
};

}  // namespace ros2_medkit_gateway
