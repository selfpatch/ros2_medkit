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
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <tl/expected.hpp>
#include <unordered_map>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"
#include "ros2_medkit_gateway/core/auth/auth_models.hpp"
#include "ros2_medkit_gateway/core/auth/auth_requirement_policy.hpp"

namespace ros2_medkit_gateway {

/**
 * @brief Manages authentication and authorization for the gateway
 *
 * Implements JWT-based authentication with RBAC (Role-Based Access Control).
 * Supports both symmetric (HS256) and asymmetric (RS256) JWT signing.
 *
 * @verifies REQ_INTEROP_086, REQ_INTEROP_087
 */
class AuthManager {
 public:
  /// The wall clock that token times and record horizons are read from.
  using WallClock = std::function<std::chrono::system_clock::time_point()>;

  /**
   * @brief Construct AuthManager with configuration
   * @param config Authentication configuration
   * @param wall_clock Clock for token times and record horizons. A test replaces it to move time; jwt-cpp's
   *        verification keeps the system clock.
   */
  explicit AuthManager(const AuthConfig & config, WallClock wall_clock = std::chrono::system_clock::now);

  ~AuthManager() = default;

  // Disable copy and move (contains non-movable mutex members)
  AuthManager(const AuthManager &) = delete;
  AuthManager & operator=(const AuthManager &) = delete;
  AuthManager(AuthManager &&) = delete;
  AuthManager & operator=(AuthManager &&) = delete;

  /**
   * @brief Check if authentication is enabled
   * @return true if auth is enabled
   */
  bool is_enabled() const {
    return config_.enabled;
  }

  /**
   * @brief Get the auth requirement level
   * @return AuthRequirement level
   */
  AuthRequirement get_requirement() const {
    return config_.require_auth_for;
  }

  /**
   * @brief Authenticate client credentials and generate tokens
   * @param client_id Client identifier
   * @param client_secret Client secret
   * @return TokenResponse on success, AuthErrorResponse on failure
   */
  tl::expected<TokenResponse, AuthErrorResponse> authenticate(const std::string & client_id,
                                                              const std::string & client_secret);

  /**
   * @brief Check a client's credentials (RFC 6749 §2.3.1) without issuing tokens
   * @param client_id Client identifier
   * @param client_secret Client secret
   * @return `invalid_client` for an unknown or disabled client or a wrong secret
   */
  tl::expected<void, AuthErrorResponse> authenticate_client(const std::string & client_id,
                                                            const std::string & client_secret) const;

  /**
   * @brief Refresh an access token using a refresh token
   * @param refresh_token The refresh token
   * @param client_id The authenticated client. A token issued to another client is `invalid_grant`.
   * @return TokenResponse on success, AuthErrorResponse on failure
   */
  tl::expected<TokenResponse, AuthErrorResponse> refresh_access_token(const std::string & refresh_token,
                                                                      const std::string & client_id);

  /**
   * @brief Validate a JWT access token
   *
   * The returned role is the one this manager's client table gives `sub`, not the token's `role` claim.
   *
   * @param token The JWT token string
   * @param expected_type Expected token type (defaults to ACCESS)
   * @return TokenValidationResult with claims if valid
   */
  TokenValidationResult validate_token(const std::string & token, TokenType expected_type = TokenType::ACCESS) const;

  /**
   * @brief Check if a role is authorized for a specific HTTP method and path
   * @param role User role
   * @param method HTTP method (GET, POST, PUT, DELETE)
   * @param path Request path (e.g., /api/v1/components/engine/data)
   * @return AuthorizationResult indicating if authorized
   */
  AuthorizationResult check_authorization(UserRole role, const std::string & method, const std::string & path) const;

  /**
   * @brief Merge permission entries into the table check_authorization reads.
   *
   * The table starts empty and the check fails closed, so a manager nobody
   * feeds authorizes nothing. That is deliberate: the entries belong to the
   * route set actually mounted, and only the caller that mounts the routes
   * knows what that is. `RESTServer::setup_routes()` merges two sources - the
   * registry's derivation for the gateway's own routes and
   * `AuthConfig::residual_route_permissions()` for the ones mounted outside it
   * - before the HTTP server starts listening.
   *
   * Merging, not replacing, so those two calls compose. Not thread-safe, and
   * not made so: every call happens on the constructing thread before
   * `RESTServer::start()`, which is what publishes the table to the request
   * threads that read it.
   *
   * @param permissions Entries to add, keyed by role
   */
  void add_route_permissions(const RoutePermissions & permissions);

  /**
   * @brief Check if authentication is required for a request
   * @param method HTTP method
   * @param path Request path
   * @return true if authentication is required
   */
  bool requires_authentication(const std::string & method, const std::string & path) const;

  /**
   * @brief Whether `auth.public_routes` names this exact route
   * @param method HTTP method
   * @param path Request path
   */
  bool is_public_route(const std::string & method, const std::string & path) const;

  /**
   * @brief The policy half of `requires_authentication`
   *
   * Borrowed, never null, owned by this manager. Exposed so a reader that has
   * to answer the same question about a route it is not currently serving -
   * `RouteRegistry`, deciding whether an operation may publish a token
   * requirement - asks the same object rather than a second copy of the rule.
   * Callers must apply `AuthConfig::enabled` themselves; this accessor
   * deliberately does not, because it hands out the policy, not the verdict.
   *
   * @return The configured requirement policy
   */
  const IAuthRequirementPolicy * auth_policy() const {
    return auth_policy_.get();
  }

  /**
   * @brief Revoke a refresh token, also one past its expiry or minted by another gateway
   *
   * `exp`, `iat` and `nbf` are not checked. A login record is revoked only by a token with its client as subject, and
   * kept until the later of the last access token `exp` reserved from it and `min(expires_at, now)` plus the access
   * token lifetime. A token with no login record here (minted
   * elsewhere, or dropped by the cap) gets a revoked record only when its subject is in the client table. Those
   * records form a pool per client of `max_refresh_tokens_per_client`, apart from the login records, and are kept to
   * the token's `exp` plus the access token lifetime.
   *
   * @param refresh_token The refresh token to revoke
   * @param client_id The authenticated client (RFC 7009 §2.1)
   * @return true if a record is revoked, false if nothing is recorded: the token does not verify, or its record names
   *         another subject. Errors record nothing: `unsupported_token_type` for an access token, `invalid_grant` for
   *         a token issued to another client, and `temporarily_unavailable` with `retry_after_seconds` (seconds until
   *         the oldest record expires) when the client's pool is full.
   */
  tl::expected<bool, AuthErrorResponse> try_revoke_refresh_token(const std::string & refresh_token,
                                                                 const std::string & client_id);

  /// try_revoke_refresh_token(), with every error reported as false.
  bool revoke_refresh_token(const std::string & refresh_token, const std::string & client_id);

  /**
   * @brief Drop refresh records whose horizon has passed (see try_revoke_refresh_token)
   * @return Number of records dropped
   */
  size_t cleanup_expired_tokens();

  /**
   * @brief Register a new client (for dynamic client registration)
   * @param client_id Client identifier
   * @param client_secret Client secret
   * @param role Role to assign
   * @return true if registered, false if client_id already exists
   */
  bool register_client(const std::string & client_id, const std::string & client_secret, UserRole role);

  /**
   * @brief Get client credentials by ID
   * @param client_id Client identifier
   * @return ClientCredentials if found
   */
  std::optional<ClientCredentials> get_client(const std::string & client_id) const;

  /**
   * @brief Disable a client (all tokens become invalid immediately)
   * @param client_id Client identifier
   * @return true if disabled, false if client not found
   */
  bool disable_client(const std::string & client_id);

  /**
   * @brief Enable a previously disabled client
   * @param client_id Client identifier
   * @return true if enabled, false if client not found
   */
  bool enable_client(const std::string & client_id);

  /// Number of refresh records held, login and foreign. Public so a test can observe the sweep bound.
  size_t refresh_token_count() const;

  /// Last wall-clock second the record for refresh `jti` is kept; empty when none is held.
  /// Public so a test can observe the horizon.
  std::optional<int64_t> refresh_record_kept_until(const std::string & jti) const;

 private:
  /**
   * @brief Generate a JWT token
   * @param claims Token claims
   * @return JWT token string
   */
  std::string generate_jwt(const JwtClaims & claims) const;

  /// Whether decode_jwt checks `exp`, `iat` and `nbf` against this clock. Signature and issuer are verified either way.
  enum class TimeChecks { CHECK, IGNORE };

  /**
   * @brief Decode and verify a JWT token
   *
   * The `role` claim is not read: the role comes from the client table or the refresh record.
   *
   * @param token JWT token string
   * @param time_checks TimeChecks::IGNORE accepts a token outside its validity window
   * @return JwtClaims if valid
   */
  tl::expected<JwtClaims, std::string> decode_jwt(const std::string & token,
                                                  TimeChecks time_checks = TimeChecks::CHECK) const;

  /**
   * @brief Generate a unique token ID
   * @return UUID string
   */
  static std::string generate_token_id();

  /**
   * @brief Check if a permission pattern matches a path
   * @param pattern Permission pattern (may contain *)
   * @param path Actual request path
   * @return true if matches
   */
  static bool matches_path(const std::string & pattern, const std::string & path);

  /**
   * @brief Store a login refresh record, making room under the client's cap
   * @param token_id The refresh token's jti
   * @param client_id Client the record belongs to
   * @param role Role the record grants on refresh
   * @return The clock second the record was stored at, which stamps the login tokens. When revoked records fill the
   *         client's cap, nothing is stored and the error is the seconds until a sweep frees a slot.
   */
  tl::expected<int64_t, int64_t> store_refresh_token(const std::string & token_id, const std::string & client_id,
                                                     UserRole role);

  /// Whether refresh `token_id` is revoked here, in a login record or in the foreign pool.
  bool refresh_token_revoked(const std::string & token_id) const;

  /// cleanup_expired_tokens() without the lock, at the caller's one clock reading `now_ts`.
  /// The caller holds refresh_tokens_mutex_.
  size_t cleanup_expired_locked(int64_t now_ts);

  /// Drops the client's oldest unrevoked records until one more fits under the cap. Revoked records count toward the
  /// cap and are never dropped; when they fill it, returns the seconds until a sweep frees a slot.
  /// The caller holds refresh_tokens_mutex_.
  std::optional<int64_t> make_room_for_client_locked(const std::string & client_id, int64_t now_ts);

  /// Whole seconds of wall_clock_, the unit of every token and record time.
  int64_t now_seconds() const;

  /// The client entry when the id is listed, enabled and the secret matches. The caller holds clients_mutex_.
  tl::expected<const ClientCredentials *, AuthErrorResponse>
  check_client_locked(const std::string & client_id, const std::string & client_secret) const;

  AuthConfig config_;
  WallClock wall_clock_;

  // RS256 key material, read once in the constructor. Empty under HS256. Not written after construction.
  std::string rs256_private_key_;
  std::string rs256_public_key_;

  // RBAC entries check_authorization matches against, populated via
  // add_route_permissions() before the server starts listening. Empty until
  // then, and an empty set authorizes nothing - see add_route_permissions().
  RoutePermissions permissions_;

  // Auth requirement policy (created from config)
  std::unique_ptr<IAuthRequirementPolicy> auth_policy_;

  // Client credentials storage (thread-safe)
  mutable std::mutex clients_mutex_;
  std::unordered_map<std::string, ClientCredentials> clients_;

  // A refresh record, its store order (ranks a client's records for the cap), the last second it is kept, and the
  // latest exp of an access token minted from it.
  struct StoredRefreshToken {
    RefreshTokenRecord record;
    uint64_t sequence{0};
    int64_t keep_until{0};
    int64_t minted_until{0};
  };

  // A revocation of a refresh token with no login record here: its client and the last second it is kept.
  struct ForeignRevocation {
    std::string client_id;
    int64_t keep_until{0};
  };

  // Refresh token storage (thread-safe). Login records, and revocations of tokens without one, by jti.
  mutable std::mutex refresh_tokens_mutex_;
  std::unordered_map<std::string, StoredRefreshToken> refresh_tokens_;
  std::unordered_map<std::string, ForeignRevocation> foreign_revocations_;
  uint64_t next_refresh_sequence_{0};
};

}  // namespace ros2_medkit_gateway
