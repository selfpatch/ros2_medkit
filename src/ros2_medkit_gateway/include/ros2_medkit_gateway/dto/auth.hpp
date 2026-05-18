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

#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <tuple>

#include "ros2_medkit_gateway/dto/contract.hpp"

namespace ros2_medkit_gateway {
namespace dto {

// =============================================================================
// AuthCredentials - request body for POST /auth/authorize and POST /auth/token.
//
// Wire shape (from AuthorizeRequest::from_json + AuthorizeRequest::from_form_data):
//   grant_type    - OAuth2 grant type (required)
//                   "client_credentials" for /auth/authorize,
//                   "refresh_token" for /auth/token
//   client_id     - client identifier (optional; required by handler for
//                   client_credentials flow)
//   client_secret - client secret (optional; required by handler for
//                   client_credentials flow)
//   refresh_token - refresh token string (optional; required by handler for
//                   refresh_token flow)
//   scope         - requested scope / role (optional)
//
// grant_type is NOT field_enum: the handler performs bespoke grant_type
// validation with an OAuth2 "unsupported_grant_type" error response that
// is distinct from the SOVD GenericError format used by parse_body.
// =============================================================================
struct AuthCredentials {
  std::string grant_type;
  std::optional<std::string> client_id;
  std::optional<std::string> client_secret;
  std::optional<std::string> refresh_token;
  std::optional<std::string> scope;
};

template <>
inline constexpr auto dto_fields<AuthCredentials> =
    std::make_tuple(field("grant_type", &AuthCredentials::grant_type), field("client_id", &AuthCredentials::client_id),
                    field("client_secret", &AuthCredentials::client_secret),
                    field("refresh_token", &AuthCredentials::refresh_token), field("scope", &AuthCredentials::scope));

template <>
inline constexpr std::string_view dto_name<AuthCredentials> = "AuthCredentials";

// =============================================================================
// AuthTokenResponse - success response for POST /auth/authorize and
// POST /auth/token.
//
// Wire shape (from TokenResponse::to_json in auth_models.hpp):
//   access_token  - JWT access token string (required)
//   token_type    - always "Bearer" (required)
//   expires_in    - seconds until access token expires (required, integer)
//   scope         - role-based scope string (required)
//   refresh_token - JWT refresh token string (optional; absent on token refresh
//                   if the existing refresh token is reused)
// =============================================================================
struct AuthTokenResponse {
  std::string access_token;
  std::string token_type;
  int expires_in{0};
  std::string scope;
  std::optional<std::string> refresh_token;
};

template <>
inline constexpr auto dto_fields<AuthTokenResponse> =
    std::make_tuple(field("access_token", &AuthTokenResponse::access_token),
                    field("token_type", &AuthTokenResponse::token_type),
                    field("expires_in", &AuthTokenResponse::expires_in), field("scope", &AuthTokenResponse::scope),
                    field("refresh_token", &AuthTokenResponse::refresh_token));

template <>
inline constexpr std::string_view dto_name<AuthTokenResponse> = "AuthTokenResponse";

}  // namespace dto
}  // namespace ros2_medkit_gateway
