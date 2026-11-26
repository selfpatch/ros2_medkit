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

#include "ros2_medkit_gateway/core/auth/auth_manager.hpp"

#include <jwt-cpp/jwt.h>
#include <openssl/crypto.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <utility>
#include <vector>

namespace ros2_medkit_gateway {

namespace {

// Constant time over the secret's bytes: the time does not depend on where the first difference is.
// A length mismatch returns early, which leaks the length only.
bool secret_matches(const std::string & expected, const std::string & presented) {
  return expected.size() == presented.size() && CRYPTO_memcmp(expected.data(), presented.data(), expected.size()) == 0;
}

// Replaces the verifier's default `exp`, `iat` and `nbf` checks with ones that accept any time.
template <typename Verifier>
void ignore_time_claims(Verifier & verifier) {
  for (const char * claim : {"exp", "iat", "nbf"}) {
    verifier.with_claim(claim, [](const auto &, std::error_code &) {});
  }
}

// The 6-bit value of a base64url character, or -1 for any other character.
int base64url_value(char c) {
  if (c >= 'A' && c <= 'Z') {
    return c - 'A';
  }
  if (c >= 'a' && c <= 'z') {
    return c - 'a' + 26;
  }
  if (c >= '0' && c <= '9') {
    return c - '0' + 52;
  }
  if (c == '-') {
    return 62;
  }
  return c == '_' ? 63 : -1;
}

// Unpadded base64url (RFC 4648 3.5): non-empty, never 1 mod 4 long, and the
// bits of the last character past the final byte are zero.
bool is_base64url_part(std::size_t length, int last) {
  switch (length % 4) {
    case 0:
      return length != 0;
    case 2:
      return (last & 0x0F) == 0;
    case 3:
      return (last & 0x03) == 0;
    default:
      return false;
  }
}

// JWS compact serialization (RFC 7515 7.1): three unpadded base64url parts.
bool is_jws_compact(const std::string & token) {
  std::size_t parts = 1;
  std::size_t length = 0;
  int last = 0;
  for (const char c : token) {
    if (c == '.') {
      if (!is_base64url_part(length, last)) {
        return false;
      }
      ++parts;
      length = 0;
      continue;
    }
    last = base64url_value(c);
    if (last < 0) {
      return false;
    }
    ++length;
  }
  return parts == 3 && is_base64url_part(length, last);
}

}  // namespace

// Helper to read file contents
static std::string read_file_contents(const std::string & path) {
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + path);
  }
  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}

// Helper to validate RSA key file exists and is readable
static void validate_key_file(const std::string & path, const std::string & key_type) {
  if (path.empty()) {
    throw std::runtime_error(key_type + " path is empty");
  }
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Failed to open " + key_type + " file: " + path);
  }
  // Check file is not empty
  file.seekg(0, std::ios::end);
  if (file.tellg() == 0) {
    throw std::runtime_error(key_type + " file is empty: " + path);
  }
}

AuthManager::AuthManager(const AuthConfig & config, WallClock wall_clock)
  : config_(config), wall_clock_(std::move(wall_clock)) {
  // Validate RS256 key files at startup (fail fast), then read them once. A rotated key takes effect at the next start.
  if (config_.enabled && config_.jwt_algorithm == JwtAlgorithm::RS256) {
    validate_key_file(config_.jwt_secret, "RS256 private key");
    validate_key_file(config_.jwt_public_key, "RS256 public key");
    rs256_private_key_ = read_file_contents(config_.jwt_secret);
    rs256_public_key_ = read_file_contents(config_.jwt_public_key);
  }

  // Initialize clients from config
  for (const auto & client : config_.clients) {
    clients_[client.client_id] = client;
  }

  // Create auth requirement policy from config
  auth_policy_ = AuthRequirementPolicyFactory::create(config_.require_auth_for);
}

tl::expected<const ClientCredentials *, AuthErrorResponse>
AuthManager::check_client_locked(const std::string & client_id, const std::string & client_secret) const {
  // One answer for an unknown client, a disabled one and a wrong secret.
  auto it = clients_.find(client_id);
  if (it == clients_.end() || !it->second.enabled || !secret_matches(it->second.client_secret, client_secret)) {
    return tl::unexpected(AuthErrorResponse::client_authentication_failed());
  }
  return &it->second;
}

tl::expected<void, AuthErrorResponse> AuthManager::authenticate_client(const std::string & client_id,
                                                                       const std::string & client_secret) const {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  const auto checked = check_client_locked(client_id, client_secret);
  if (!checked) {
    return tl::unexpected(checked.error());
  }
  return {};
}

tl::expected<TokenResponse, AuthErrorResponse> AuthManager::authenticate(const std::string & client_id,
                                                                         const std::string & client_secret) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  const auto checked = check_client_locked(client_id, client_secret);
  if (!checked) {
    return tl::unexpected(checked.error());
  }
  const auto & client = **checked;

  // The record is stored first: its one clock reading, taken under the record lock, decides the sweep and the cap
  // and stamps both tokens.
  std::string refresh_token_id = generate_token_id();
  const auto stored_at = store_refresh_token(refresh_token_id, client_id, client.role);
  if (!stored_at) {
    return tl::unexpected(AuthErrorResponse::temporarily_unavailable(
        "Every slot of auth.max_refresh_tokens_per_client holds a revoked refresh token of this client; retry after "
        "the oldest one expires",
        stored_at.error()));
  }
  const int64_t now_ts = *stored_at;

  // Generate refresh token
  JwtClaims refresh_claims;
  refresh_claims.iss = config_.issuer;
  refresh_claims.sub = client_id;
  refresh_claims.iat = now_ts;
  refresh_claims.exp = now_ts + config_.refresh_token_expiry_seconds;
  refresh_claims.jti = refresh_token_id;
  refresh_claims.typ = TokenType::REFRESH;  // Mark as refresh token
  refresh_claims.role = client.role;

  std::string refresh_token = generate_jwt(refresh_claims);

  // Generate access token, the one the record reserved
  JwtClaims access_claims;
  access_claims.iss = config_.issuer;
  access_claims.sub = client_id;
  access_claims.iat = now_ts;
  access_claims.exp = now_ts + config_.token_expiry_seconds;
  access_claims.jti = generate_token_id();
  access_claims.typ = TokenType::ACCESS;  // Mark as access token
  access_claims.role = client.role;
  access_claims.refresh_token_id = refresh_token_id;

  std::string access_token = generate_jwt(access_claims);

  // Build response
  TokenResponse response;
  response.access_token = access_token;
  response.token_type = "Bearer";
  response.expires_in = config_.token_expiry_seconds;
  response.refresh_token = refresh_token;
  response.scope = role_to_string(client.role);

  return response;
}

tl::expected<TokenResponse, AuthErrorResponse> AuthManager::refresh_access_token(const std::string & refresh_token,
                                                                                 const std::string & client_id) {
  // Decode and validate refresh token
  auto decode_result = decode_jwt(refresh_token);
  if (!decode_result) {
    std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
    // Sweep on the failing paths too: a client that only refreshes reaches no other sweep.
    cleanup_expired_locked(now_seconds());
    return tl::unexpected(AuthErrorResponse::invalid_grant(decode_result.error()));
  }

  const auto & claims = decode_result.value();

  // Verify this is actually a refresh token, not an access token
  if (claims.typ != TokenType::REFRESH) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Token is not a refresh token"));
  }

  // RFC 6749 §6: the refresh token is bound to the client it was issued to.
  if (claims.sub != client_id) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token was issued to another client"));
  }

  // Read before the record lock: authenticate() holds the client lock while it stores a record.
  const auto client = get_client(claims.sub);

  int64_t now_ts = 0;
  UserRole role = UserRole::VIEWER;
  {
    std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
    // One clock reading for the sweep, the expiry check and the reservation.
    now_ts = now_seconds();
    cleanup_expired_locked(now_ts);

    const auto it = refresh_tokens_.find(claims.jti);
    if (it == refresh_tokens_.end()) {
      return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token not found"));
    }
    const RefreshTokenRecord & record = it->second.record;

    // A jti names one record, and a token for another subject did not mint it.
    if (record.client_id != client_id) {
      return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token was issued to another client"));
    }

    if (record.revoked) {
      return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token has been revoked"));
    }

    if (record.expires_at < now_ts) {
      return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token has expired"));
    }

    if (!client.has_value()) {
      return tl::unexpected(AuthErrorResponse::invalid_grant("Client no longer exists"));
    }

    if (!client->enabled) {
      return tl::unexpected(AuthErrorResponse::invalid_grant("Client is disabled"));
    }

    // Reserved before the lock is released: a revocation after this keeps the record until this exp. This refresh
    // still returns its token, and validate_token refuses it once the record is revoked.
    it->second.minted_until = std::max(it->second.minted_until, now_ts + config_.token_expiry_seconds);
    role = record.role;
  }

  // Signed outside the record lock, so signing does not hold up validate_token.
  JwtClaims access_claims;
  access_claims.iss = config_.issuer;
  access_claims.sub = claims.sub;
  access_claims.iat = now_ts;
  access_claims.exp = now_ts + config_.token_expiry_seconds;
  access_claims.jti = generate_token_id();
  access_claims.typ = TokenType::ACCESS;  // Mark as access token
  access_claims.role = role;              // Use role from refresh token record
  access_claims.refresh_token_id = claims.jti;

  std::string access_token = generate_jwt(access_claims);

  // Build response (no new refresh token on refresh)
  TokenResponse response;
  response.access_token = access_token;
  response.token_type = "Bearer";
  response.expires_in = config_.token_expiry_seconds;
  response.scope = role_to_string(role);
  // Return the existing refresh token so clients can continue to use it
  response.refresh_token = refresh_token;

  return response;
}

TokenValidationResult AuthManager::validate_token(const std::string & token, TokenType expected_type) const {
  TokenValidationResult result;

  auto decode_result = decode_jwt(token);
  if (!decode_result) {
    result.valid = false;
    result.error = decode_result.error();
    return result;
  }

  const auto & claims = decode_result.value();

  // Check token type matches expected
  if (claims.typ != expected_type) {
    result.valid = false;
    result.error = "Invalid token type: expected " + token_type_to_string(expected_type) + ", got " +
                   token_type_to_string(claims.typ);
    return result;
  }

  // Check expiration
  if (claims.is_expired()) {
    result.valid = false;
    result.error = "Token has expired";
    return result;
  }

  // Check if client is still enabled (security: validate on every request)
  auto client = get_client(claims.sub);
  if (!client.has_value()) {
    result.valid = false;
    result.error = "Client no longer exists";
    return result;
  }
  if (!client->enabled) {
    result.valid = false;
    result.error = "Client has been disabled";
    return result;
  }

  // Check if associated refresh token is revoked (for access tokens)
  if (claims.refresh_token_id.has_value()) {
    if (refresh_token_revoked(claims.refresh_token_id.value())) {
      result.valid = false;
      result.error = "Associated refresh token has been revoked";
      return result;
    }
  }

  result.valid = true;
  result.claims = claims;
  // The role is this gateway's grant. The signed claim is the issuer's, and a peer sharing the secret may differ.
  result.claims->role = client->role;
  return result;
}

void AuthManager::add_route_permissions(const RoutePermissions & permissions) {
  for (const auto & [role, entries] : permissions) {
    permissions_[role].insert(entries.begin(), entries.end());
  }
}

AuthorizationResult AuthManager::check_authorization(UserRole role, const std::string & method,
                                                     const std::string & path) const {
  AuthorizationResult result;

  auto it = permissions_.find(role);
  if (it == permissions_.end()) {
    result.authorized = false;
    // Not "unknown role" any more: every enumerator is a role the gateway
    // knows, and reaching here means the table was never given entries for it
    // (see add_route_permissions). Saying "unknown role" would send a reader
    // hunting for a typo in the token instead of a gap in the table.
    result.error = "No permissions are configured for this role";
    return result;
  }

  const auto & permissions = it->second;
  std::string permission_key = method + ":" + path;

  // Check exact match first
  if (permissions.count(permission_key) > 0) {
    result.authorized = true;
    return result;
  }

  // Check wildcard patterns
  for (const auto & pattern : permissions) {
    // Extract method and path pattern
    size_t colon_pos = pattern.find(':');
    if (colon_pos == std::string::npos) {
      continue;
    }

    std::string pattern_method = pattern.substr(0, colon_pos);
    std::string pattern_path = pattern.substr(colon_pos + 1);

    // Method must match exactly
    if (pattern_method != method) {
      continue;
    }

    // Check path pattern
    if (matches_path(pattern_path, path)) {
      result.authorized = true;
      return result;
    }
  }

  result.authorized = false;
  result.error = "Insufficient permissions";
  result.required_permission = permission_key;
  return result;
}

bool AuthManager::requires_authentication(const std::string & method, const std::string & path) const {
  if (!config_.enabled) {
    return false;
  }

  // Delegate to policy
  return auth_policy_->requires_authentication(method, path);
}

bool AuthManager::revoke_refresh_token(const std::string & refresh_token, const std::string & client_id) {
  return try_revoke_refresh_token(refresh_token, client_id).value_or(false);
}

tl::expected<bool, AuthErrorResponse> AuthManager::try_revoke_refresh_token(const std::string & refresh_token,
                                                                            const std::string & client_id) {
  // Times ignored: a refresh token past its expiry can have live access tokens, and an issuer's clock may run ahead.
  auto decode_result = decode_jwt(refresh_token, TimeChecks::IGNORE);
  if (!decode_result) {
    return false;
  }

  const auto & claims = decode_result.value();

  // Records are keyed by refresh jti. A record for an access token would be read by nothing.
  if (claims.typ != TokenType::REFRESH) {
    return tl::unexpected(AuthErrorResponse::unsupported_token_type("Only refresh tokens are revocable here"));
  }

  // RFC 7009 §2.1: the token must have been issued to the requesting client.
  if (claims.sub != client_id) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token was issued to another client"));
  }

  // Read before the record lock: authenticate() holds the client lock while it stores a record.
  const auto client = get_client(claims.sub);
  const int64_t access_lifetime = config_.token_expiry_seconds;

  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  // One clock reading for the sweep, the horizon and Retry-After.
  const auto now_ts = now_seconds();
  cleanup_expired_locked(now_ts);
  auto it = refresh_tokens_.find(claims.jti);
  if (it != refresh_tokens_.end()) {
    // A jti names one record. A token for another subject did not mint it.
    if (it->second.record.client_id != claims.sub) {
      return false;
    }
    // After this no refresh reserves an access token from the record. Kept to the later of two horizons: the last
    // reserved exp covers a clock that stepped back between a mint and now, and min(expires_at, now) + access covers
    // a clock that jumped forward before now and comes back.
    it->second.record.revoked = true;
    it->second.keep_until = std::min(
        it->second.keep_until,
        std::max(it->second.minted_until, std::min<int64_t>(it->second.record.expires_at, now_ts) + access_lifetime));
    return true;
  }

  // No record here: minted elsewhere, or dropped by the cap. validate_token already refuses a subject outside the
  // table, so only listed ones get a record.
  if (!client.has_value()) {
    return false;
  }
  if (foreign_revocations_.count(claims.jti) > 0) {
    return true;
  }

  // A pool per client, apart from its login records, so these never block a login. Nothing is dropped to make room:
  // a dropped record would let its access tokens through again.
  const auto cap = static_cast<size_t>(std::max(1, config_.max_refresh_tokens_per_client));
  size_t held = 0;
  int64_t oldest = std::numeric_limits<int64_t>::max();
  for (const auto & [jti, foreign] : foreign_revocations_) {
    if (foreign.client_id == claims.sub) {
      ++held;
      oldest = std::min(oldest, foreign.keep_until);
    }
  }
  if (held >= cap) {
    return tl::unexpected(AuthErrorResponse::temporarily_unavailable(
        "This client holds auth.max_refresh_tokens_per_client revocations of refresh tokens this gateway holds no "
        "record for; retry after the oldest one expires",
        std::max<int64_t>(1, oldest + 1 - now_ts)));
  }

  // validate_token reads records as a denylist. The issuer may mint from the token until its exp, so the record is
  // kept one access lifetime past it.
  // Precondition: a Medkit issuer wrote exp, so it is at most iat + auth.refresh_token_expiry_seconds (int seconds).
  foreign_revocations_[claims.jti] = ForeignRevocation{claims.sub, claims.exp + access_lifetime};
  return true;
}

size_t AuthManager::cleanup_expired_locked(int64_t now_ts) {
  size_t count = 0;
  for (auto it = refresh_tokens_.begin(); it != refresh_tokens_.end();) {
    if (it->second.keep_until < now_ts) {
      it = refresh_tokens_.erase(it);
      ++count;
    } else {
      ++it;
    }
  }
  for (auto it = foreign_revocations_.begin(); it != foreign_revocations_.end();) {
    if (it->second.keep_until < now_ts) {
      it = foreign_revocations_.erase(it);
      ++count;
    } else {
      ++it;
    }
  }
  return count;
}

size_t AuthManager::cleanup_expired_tokens() {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  return cleanup_expired_locked(now_seconds());
}

size_t AuthManager::refresh_token_count() const {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  return refresh_tokens_.size() + foreign_revocations_.size();
}

std::optional<int64_t> AuthManager::refresh_record_kept_until(const std::string & jti) const {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  if (const auto it = refresh_tokens_.find(jti); it != refresh_tokens_.end()) {
    return it->second.keep_until;
  }
  if (const auto it = foreign_revocations_.find(jti); it != foreign_revocations_.end()) {
    return it->second.keep_until;
  }
  return std::nullopt;
}

bool AuthManager::register_client(const std::string & client_id, const std::string & client_secret, UserRole role) {
  std::lock_guard<std::mutex> lock(clients_mutex_);

  if (clients_.count(client_id) > 0) {
    return false;
  }

  ClientCredentials creds;
  creds.client_id = client_id;
  creds.client_secret = client_secret;
  creds.role = role;
  creds.enabled = true;

  clients_[client_id] = creds;
  return true;
}

std::optional<ClientCredentials> AuthManager::get_client(const std::string & client_id) const {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return std::nullopt;
  }
  return it->second;
}

bool AuthManager::disable_client(const std::string & client_id) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return false;
  }
  it->second.enabled = false;
  return true;
}

bool AuthManager::enable_client(const std::string & client_id) {
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return false;
  }
  it->second.enabled = true;
  return true;
}

std::string AuthManager::generate_jwt(const JwtClaims & claims) const {
  auto builder = jwt::create()
                     .set_type(token_type_to_string(claims.typ))  // Set typ in JWT header
                     .set_issuer(claims.iss)
                     .set_subject(claims.sub)
                     .set_issued_at(std::chrono::system_clock::from_time_t(claims.iat))
                     .set_expires_at(std::chrono::system_clock::from_time_t(claims.exp))
                     .set_id(claims.jti)
                     .set_payload_claim("role", jwt::claim(role_to_string(claims.role)));

  if (!claims.permissions.empty()) {
    // Convert vector to set for jwt-cpp
    std::set<std::string> perms_set(claims.permissions.begin(), claims.permissions.end());
    builder.set_payload_claim("permissions", jwt::claim(perms_set));
  }

  if (claims.refresh_token_id.has_value()) {
    builder.set_payload_claim("refresh_token_id", jwt::claim(claims.refresh_token_id.value()));
  }

  // Sign based on algorithm
  switch (config_.jwt_algorithm) {
    case JwtAlgorithm::HS256:
      return builder.sign(jwt::algorithm::hs256{config_.jwt_secret});

    case JwtAlgorithm::RS256:
      return builder.sign(jwt::algorithm::rs256("", rs256_private_key_, "", ""));

    default:
      throw std::runtime_error("Unsupported JWT algorithm");
  }
}

tl::expected<JwtClaims, std::string> AuthManager::decode_jwt(const std::string & token, TimeChecks time_checks) const {
  // jwt-cpp copies the rest of a part once per trailing fill (`%3d`), so other forms never reach it.
  if (!is_jws_compact(token)) {
    return tl::unexpected("JWT decode error: invalid token supplied");
  }
  try {
    // Decode token first
    auto decoded = jwt::decode(token);

    // Verify signature
    try {
      switch (config_.jwt_algorithm) {
        case JwtAlgorithm::HS256: {
          auto verifier =
              jwt::verify().allow_algorithm(jwt::algorithm::hs256{config_.jwt_secret}).with_issuer(config_.issuer);
          if (time_checks == TimeChecks::IGNORE) {
            ignore_time_claims(verifier);
          }
          verifier.verify(decoded);
          break;
        }

        case JwtAlgorithm::RS256: {
          auto verifier = jwt::verify()
                              .allow_algorithm(jwt::algorithm::rs256(rs256_public_key_, "", "", ""))
                              .with_issuer(config_.issuer);
          if (time_checks == TimeChecks::IGNORE) {
            ignore_time_claims(verifier);
          }
          verifier.verify(decoded);
          break;
        }

        default:
          return tl::unexpected("Unsupported JWT algorithm");
      }
    } catch (const jwt::error::token_verification_exception & e) {
      return tl::unexpected("Token verification failed: " + std::string(e.what()));
    }

    // Extract claims
    JwtClaims claims;
    claims.iss = decoded.get_issuer();
    claims.sub = decoded.get_subject();
    claims.jti = decoded.get_id();

    // Extract typ from header
    if (decoded.has_type()) {
      try {
        claims.typ = string_to_token_type(decoded.get_type());
      } catch (const std::invalid_argument &) {
        claims.typ = TokenType::ACCESS;  // Default for backward compatibility
      }
    }

    auto exp_claim = decoded.get_expires_at();
    claims.exp = std::chrono::duration_cast<std::chrono::seconds>(exp_claim.time_since_epoch()).count();

    auto iat_claim = decoded.get_issued_at();
    claims.iat = std::chrono::duration_cast<std::chrono::seconds>(iat_claim.time_since_epoch()).count();

    if (decoded.has_payload_claim("permissions")) {
      auto perms = decoded.get_payload_claim("permissions").as_array();
      for (const auto & p : perms) {
        claims.permissions.push_back(p.get<std::string>());
      }
    }

    if (decoded.has_payload_claim("refresh_token_id")) {
      claims.refresh_token_id = decoded.get_payload_claim("refresh_token_id").as_string();
    }

    return claims;
  } catch (const std::exception & e) {
    return tl::unexpected("JWT decode error: " + std::string(e.what()));
  }
}

std::string AuthManager::generate_token_id() {
  // Generate UUID-like string using a per-call RNG seeded from std::random_device
  // to avoid reuse of time-based seeds and thread-local initialization issues.
  std::random_device rd;
  std::mt19937_64 gen(rd());
  std::uniform_int_distribution<uint64_t> dis;

  std::stringstream ss;
  ss << std::hex << std::setfill('0');

  uint64_t part1 = dis(gen);
  uint64_t part2 = dis(gen);

  ss << std::setw(8) << (part1 >> 32) << "-";
  ss << std::setw(4) << ((part1 >> 16) & 0xFFFF) << "-";
  ss << std::setw(4) << (part1 & 0xFFFF) << "-";
  ss << std::setw(4) << (part2 >> 48) << "-";
  ss << std::setw(12) << (part2 & 0xFFFFFFFFFFFF);

  return ss.str();
}

bool AuthManager::matches_path(const std::string & pattern, const std::string & path) {
  // Simple wildcard matching
  // * matches any single path segment
  // ** matches any number of path segments (including none)
  // Pattern: /api/v1/components/*/data
  // Path: /api/v1/components/engine/data
  // Pattern: /api/v1/**
  // Path: /api/v1/components/engine/data/temperature (matches)

  if (pattern == path) {
    return true;
  }

  // Convert pattern to regex
  std::string regex_pattern;
  regex_pattern.reserve(pattern.size() * 2);

  for (size_t i = 0; i < pattern.size(); ++i) {
    char c = pattern[i];
    if (c == '*') {
      // Check for ** (multi-segment wildcard)
      if (i + 1 < pattern.size() && pattern[i + 1] == '*') {
        regex_pattern += ".*";  // Match anything including slashes
        ++i;                    // Skip the second *
      } else {
        regex_pattern += "[^/]+";  // Match any non-slash characters (single segment)
      }
    } else {
      switch (c) {
        case '.':
        case '[':
        case ']':
        case '(':
        case ')':
        case '{':
        case '}':
        case '\\':
        case '^':
        case '$':
        case '|':
        case '?':
        case '+':
          regex_pattern += '\\';
          regex_pattern += c;
          break;
        default:
          regex_pattern += c;
      }
    }
  }

  // Anchor the pattern
  regex_pattern = "^" + regex_pattern + "$";

  try {
    std::regex compiled(regex_pattern);
    return std::regex_match(path, compiled);
  } catch (const std::regex_error &) {
    // If regex compilation fails, treat as non-match
    return false;
  }
}

tl::expected<int64_t, int64_t> AuthManager::store_refresh_token(const std::string & token_id,
                                                                const std::string & client_id, UserRole role) {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  // One clock reading for the sweep, the cap's Retry-After and the record's times.
  const auto now_ts = now_seconds();
  // Sweep on every login, so the store stays bounded without the node's timer.
  cleanup_expired_locked(now_ts);
  if (const auto retry_after = make_room_for_client_locked(client_id, now_ts)) {
    return tl::unexpected(*retry_after);
  }
  RefreshTokenRecord record;
  record.token_id = token_id;
  record.client_id = client_id;
  record.role = role;
  record.issued_at = now_ts;
  record.expires_at = now_ts + config_.refresh_token_expiry_seconds;
  record.revoked = false;
  // Kept one access lifetime past expiry: no access token is minted from it after expires_at. The login access token
  // is reserved here.
  refresh_tokens_[token_id] =
      StoredRefreshToken{record, next_refresh_sequence_++, record.expires_at + config_.token_expiry_seconds,
                         now_ts + config_.token_expiry_seconds};
  return now_ts;
}

std::optional<int64_t> AuthManager::make_room_for_client_locked(const std::string & client_id, int64_t now_ts) {
  using Entry = std::unordered_map<std::string, StoredRefreshToken>::iterator;
  std::vector<Entry> unrevoked;
  std::vector<int64_t> revoked_keep_until;
  for (auto it = refresh_tokens_.begin(); it != refresh_tokens_.end(); ++it) {
    if (it->second.record.client_id != client_id) {
      continue;
    }
    if (it->second.record.revoked) {
      revoked_keep_until.push_back(it->second.keep_until);
    } else {
      unrevoked.push_back(it);
    }
  }
  // The builder refuses a cap below 1; 1 is the floor for a config built another way.
  const auto cap = static_cast<size_t>(std::max(1, config_.max_refresh_tokens_per_client));
  const size_t held = unrevoked.size() + revoked_keep_until.size();
  if (held < cap) {
    return std::nullopt;
  }
  // A revoked record is what refuses its access tokens, so the cap never drops one. When they fill it, a slot frees
  // when the sweep drops the record at this index in keep_until order.
  if (revoked_keep_until.size() >= cap) {
    const auto freeing = revoked_keep_until.begin() + static_cast<std::ptrdiff_t>(revoked_keep_until.size() - cap);
    std::nth_element(revoked_keep_until.begin(), freeing, revoked_keep_until.end());
    return std::max<int64_t>(1, *freeing + 1 - now_ts);
  }
  const size_t excess = held - cap + 1;
  std::partial_sort(unrevoked.begin(), unrevoked.begin() + static_cast<std::ptrdiff_t>(excess), unrevoked.end(),
                    [](const Entry & a, const Entry & b) {
                      return a->second.sequence < b->second.sequence;
                    });
  for (size_t i = 0; i < excess; ++i) {
    refresh_tokens_.erase(unrevoked[i]);
  }
  return std::nullopt;
}

int64_t AuthManager::now_seconds() const {
  return std::chrono::duration_cast<std::chrono::seconds>(wall_clock_().time_since_epoch()).count();
}

bool AuthManager::refresh_token_revoked(const std::string & token_id) const {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  if (const auto it = refresh_tokens_.find(token_id); it != refresh_tokens_.end()) {
    return it->second.record.revoked;
  }
  return foreign_revocations_.count(token_id) > 0;
}

}  // namespace ros2_medkit_gateway
