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

#include <chrono>
#include <fstream>
#include <iomanip>
#include <random>
#include <regex>
#include <set>
#include <sstream>

namespace ros2_medkit_gateway {

namespace {

/// Compare two secrets without returning early on the first differing byte.
///
/// The lengths are compared too, and a length mismatch is reported. That does
/// leak the length, which is acceptable: secrets here are operator-chosen and
/// their length is not the secret. What must not leak is WHICH bytes matched,
/// and the loop below always visits every byte of the expected value.
bool constant_time_equals(const std::string & expected, const std::string & presented) {
  // Fold the length difference into the result and keep going, so both
  // branches cost the same.
  unsigned char diff = static_cast<unsigned char>(expected.size() != presented.size());
  const std::size_t n = expected.size();
  for (std::size_t i = 0; i < n; ++i) {
    // Index the presented value modulo its own size so a shorter input cannot
    // read out of bounds; the length check above already forced a mismatch.
    const unsigned char p = presented.empty() ? 0U : static_cast<unsigned char>(presented[i % presented.size()]);
    diff |= static_cast<unsigned char>(static_cast<unsigned char>(expected[i]) ^ p);
  }
  return diff == 0;
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

// Checks that an RS256 key file exists and carries something.
//
// The message names the PARAMETER and never the value. Under RS256 the private
// key path is read from `auth.jwt_secret`, and that parameter carries a secret
// under HS256; a message that echoed the value would put a secret into the
// startup log of any deployment that had the algorithm wrong. The parameter
// name is what the operator has to go and fix, so it is also the useful half.
static void validate_key_file(const std::string & path, const std::string & parameter_name) {
  if (path.empty()) {
    throw std::runtime_error(parameter_name + " is empty and RS256 needs a key file path there");
  }
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("the file named by " + parameter_name + " could not be opened");
  }
  // Check file is not empty
  file.seekg(0, std::ios::end);
  if (file.tellg() == 0) {
    throw std::runtime_error("the file named by " + parameter_name + " is empty");
  }
}

AuthManager::AuthManager(const AuthConfig & config) : config_(config) {
  // RS256 keys are read here, once, and held for the life of the manager.
  //
  // The public key is the one that matters for cost: it verifies a signature,
  // so a read inside decode_jwt charges every request that carries a token one
  // file open, and a flood is made of exactly those requests. The validation
  // just above has already required both files to exist and carry something,
  // so the read here cannot fail in a way that check would have missed.
  //
  // The consequence to know about: a key rotated on disk takes effect at the
  // next restart, which is also when the files are checked at all.
  if (config_.enabled && config_.jwt_algorithm == JwtAlgorithm::RS256) {
    validate_key_file(config_.jwt_secret, "auth.jwt_secret");
    validate_key_file(config_.jwt_public_key, "auth.jwt_public_key");
    rs256_private_key_ = read_file_contents(config_.jwt_secret);
    rs256_public_key_ = read_file_contents(config_.jwt_public_key);
  }

  // Initialize clients from config
  for (const auto & client : config_.clients) {
    clients_[client.client_id] = client;
  }

  // Create auth requirement policy from config. `require_auth_for` decides the
  // baseline; `public_routes` then lifts the credential requirement from the
  // routes an operator named, and from nothing else.
  auth_policy_ =
      AuthRequirementPolicyFactory::create(config_.require_auth_for, parse_public_routes(config_.public_routes));
}

tl::expected<TokenResponse, AuthErrorResponse> AuthManager::authenticate(const std::string & client_id,
                                                                         const std::string & client_secret) {
  // Find client
  std::lock_guard<std::mutex> lock(clients_mutex_);
  auto it = clients_.find(client_id);
  if (it == clients_.end()) {
    return tl::unexpected(AuthErrorResponse::invalid_client("Unknown client_id"));
  }

  const auto & client = it->second;

  // Check if client is enabled
  if (!client.enabled) {
    return tl::unexpected(AuthErrorResponse::invalid_client("Client is disabled"));
  }

  // Verify secret in constant time. A plain std::string comparison returns as
  // soon as two bytes differ, so the time it takes to refuse leaks how many
  // leading bytes were right, and a caller who can measure it can recover the
  // secret one byte at a time. Every deployment that turns authentication on
  // authenticates a client here, so this path carries all of them.
  //
  // Secrets are still stored in plaintext in the configuration; making this
  // comparison constant-time does not change that and is not meant to.
  if (!constant_time_equals(client.client_secret, client_secret)) {
    return tl::unexpected(AuthErrorResponse::invalid_client("Invalid client_secret"));
  }

  // Generate tokens
  auto now = std::chrono::system_clock::now();
  auto now_ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

  // Generate refresh token first
  std::string refresh_token_id = generate_token_id();
  JwtClaims refresh_claims;
  refresh_claims.iss = config_.issuer;
  refresh_claims.sub = client_id;
  refresh_claims.iat = now_ts;
  refresh_claims.exp = now_ts + config_.refresh_token_expiry_seconds;
  refresh_claims.jti = refresh_token_id;
  refresh_claims.typ = TokenType::REFRESH;  // Mark as refresh token
  refresh_claims.role = client.role;

  std::string refresh_token = generate_jwt(refresh_claims);

  // Store refresh token record
  RefreshTokenRecord refresh_record;
  refresh_record.token_id = refresh_token_id;
  refresh_record.client_id = client_id;
  refresh_record.role = client.role;
  refresh_record.issued_at = now_ts;
  refresh_record.expires_at = refresh_claims.exp;
  refresh_record.revoked = false;
  store_refresh_token(refresh_record);

  // Generate access token
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

tl::expected<TokenResponse, AuthErrorResponse> AuthManager::refresh_access_token(const std::string & refresh_token) {
  // Sweep first, on every call, whatever this one goes on to answer.
  //
  // A client that authorises once and refreshes for a month touches this path
  // and no other, so without a sweep here the store is cleared only by the
  // node's timer - and in any process without that timer, which is every unit
  // test of this class, never. First, and never on the way out, because the
  // paths that fail (an expired refresh token, a record already gone) are
  // exactly the ones a long-lived client produces, and a sweep placed past
  // them does not run when it is needed.
  {
    std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
    cleanup_expired_locked();
  }

  // Decode and validate refresh token
  auto decode_result = decode_jwt(refresh_token);
  if (!decode_result) {
    return tl::unexpected(AuthErrorResponse::invalid_grant(decode_result.error()));
  }

  const auto & claims = decode_result.value();

  // Verify this is actually a refresh token, not an access token
  if (claims.typ != TokenType::REFRESH) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Token is not a refresh token"));
  }

  // Check if refresh token exists and is not revoked
  auto record = get_refresh_token(claims.jti);
  if (!record.has_value()) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token not found"));
  }

  if (record->revoked) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token has been revoked"));
  }

  // Check expiration
  if (claims.is_expired()) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Refresh token has expired"));
  }

  // Get client to check if still enabled
  auto client = get_client(claims.sub);
  if (!client.has_value()) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Client no longer exists"));
  }

  if (!client->enabled) {
    return tl::unexpected(AuthErrorResponse::invalid_grant("Client is disabled"));
  }

  // Generate new access token
  auto now = std::chrono::system_clock::now();
  auto now_ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

  JwtClaims access_claims;
  access_claims.iss = config_.issuer;
  access_claims.sub = claims.sub;
  access_claims.iat = now_ts;
  access_claims.exp = now_ts + config_.token_expiry_seconds;
  access_claims.jti = generate_token_id();
  access_claims.typ = TokenType::ACCESS;  // Mark as access token
  access_claims.role = record->role;      // Use role from refresh token record
  access_claims.refresh_token_id = claims.jti;

  std::string access_token = generate_jwt(access_claims);

  // Build response (no new refresh token on refresh)
  TokenResponse response;
  response.access_token = access_token;
  response.token_type = "Bearer";
  response.expires_in = config_.token_expiry_seconds;
  response.scope = role_to_string(record->role);
  // Return the existing refresh token so clients can continue to use it
  response.refresh_token = refresh_token;

  return response;
}

TokenValidationResult AuthManager::validate_token(const std::string & token, TokenType expected_type) const {
  TokenValidationResult result;

  token_validations_.fetch_add(1, std::memory_order_relaxed);

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

  // The refresh records are a DENYLIST: a record held and marked revoked
  // refuses the access tokens minted from it, and a record this gateway does
  // not hold carries no information either way.
  //
  // Every access token names the refresh record it came from, and the records
  // live in this process's memory. Reading an absent record as "invalid"
  // therefore refuses two whole classes of token that are sound: every token
  // issued before a restart, and every token minted by another gateway sharing
  // this JWT configuration, which is what `aggregation.forward_auth` puts on a
  // forwarded request (docs/config/aggregation.rst). Such a token verifies
  // under the shared secret, names a client this gateway knows, and sits
  // inside its expiry; the missing record says only that this process was not
  // the one that issued it.
  //
  // The trade, stated plainly: a token revoked on this gateway is honoured
  // again across a restart, for at most `token_expiry_seconds` - the longest a
  // live access token can outlast the moment its record was lost. While the
  // process runs a revocation holds for the whole life of every token this
  // gateway issued, because cleanup_expired_locked keeps a record until
  // nothing minted from it can still be valid; for a token minted elsewhere
  // that is true where the issuer's access expiry is at most this gateway's
  // (see revoke_refresh_token).
  if (claims.refresh_token_id.has_value()) {
    auto record = get_refresh_token(claims.refresh_token_id.value());
    if (record.has_value() && record->revoked) {
      result.valid = false;
      result.error = "Associated refresh token has been revoked";
      return result;
    }
  }

  result.valid = true;
  result.claims = claims;

  // The role comes from THIS gateway's client table, and the role claim in the
  // token is not consulted.
  //
  // The claim is signed, so it cannot be edited in flight, but it says what the
  // ISSUING gateway granted. Under a shared JWT configuration a peer receives
  // tokens another gateway minted, and letting the claim decide would export
  // the issuer's grants: a client the peer lists as `viewer` would write on the
  // peer because the aggregator lists it as `admin`. Each gateway grants what
  // its own configuration says, so an operator can read one file and know what
  // a client may do here.
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

bool AuthManager::is_public_route(const std::string & method, const std::string & path) const {
  // No `config_.enabled` short-circuit here, unlike requires_authentication
  // above. With authentication off nothing is anonymous and nothing is
  // withheld, and the callers say so themselves; this accessor reports what
  // the operator listed, which is a property of the configuration and says
  // nothing about the request.
  return auth_policy_ != nullptr && auth_policy_->is_public(method, path);
}

bool AuthManager::revoke_refresh_token(const std::string & refresh_token) {
  // Signature and issuer verified, the expiry not. The last access token
  // minted from a refresh token can outlive it by a whole access lifetime, so
  // a revocation has to reach a refresh token past its own expiry; the record
  // it writes is bounded by the sweep like any other.
  auto decode_result = decode_jwt(refresh_token, Expiry::IGNORE);
  if (!decode_result) {
    return false;
  }

  const auto & claims = decode_result.value();

  // Refresh tokens only, checked the same way refresh_access_token checks it.
  //
  // The records are keyed by a REFRESH token's jti, and validate_token looks
  // one up by the `refresh_token_id` an access token carries. Writing an
  // access token's own jti into that map therefore stores a record nothing
  // ever reads, and the token it was meant to withdraw goes on working while
  // the call reports success. Refusing here keeps "revoked" meaning one thing.
  //
  // AuthHandlers::post_revoke (auth_handlers.cpp) answers 200 either way, per
  // RFC 7009 §2.2: what a caller may learn from /auth/revoke is nothing about
  // the token they sent.
  if (claims.typ != TokenType::REFRESH) {
    return false;
  }

  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  auto it = refresh_tokens_.find(claims.jti);
  if (it != refresh_tokens_.end()) {
    it->second.revoked = true;
    return true;
  }

  // A token this gateway did not issue, revoked here anyway.
  //
  // Under a shared JWT configuration - which is what aggregation.forward_auth
  // describes - a peer is handed tokens another gateway minted and holds no
  // record of any of them. Since validate_token reads the records as a
  // denylist, "no record" would make revocation a no-op on exactly the gateway
  // an operator is trying to lock down. Writing the record is what gives them
  // a way to refuse a token here.
  //
  // The record carries the token's own refresh expiry, clamped to THIS
  // gateway's refresh lifetime, and the sweep holds it for this gateway's
  // access lifetime past that - the only lifetimes this process knows. The
  // clamp is what bounds the store: the issuer's refresh expiry is carried in
  // the token and this gateway does not control it, so an issuer with a
  // refresh lifetime of years would otherwise leave records here for years.
  // Under the rule in docs/config/aggregation.rst - gateways sharing a signing
  // configuration share both expiries - the clamp changes nothing and the
  // record covers every access token the issuer can mint from it; a peer with
  // the shorter access expiry drops the record while a late-minted token of
  // the issuer's is still live, and one with the shorter refresh expiry drops
  // it while the issuer can still refresh.
  const auto now_ts =
      std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  const auto own_refresh_horizon = now_ts + config_.refresh_token_expiry_seconds;
  RefreshTokenRecord foreign;
  foreign.token_id = claims.jti;
  foreign.client_id = claims.sub;
  foreign.role = claims.role;
  foreign.issued_at = claims.iat;
  foreign.expires_at = claims.exp < own_refresh_horizon ? claims.exp : own_refresh_horizon;
  foreign.revoked = true;
  refresh_tokens_[claims.jti] = foreign;
  return true;
}

size_t AuthManager::cleanup_expired_locked() {
  auto now = std::chrono::system_clock::now();
  auto now_ts = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();

  // A record outlives its own expiry by one access-token lifetime.
  //
  // A revoked record is what refuses the access tokens minted from it, so
  // dropping it the instant the refresh token expires ends the revocation
  // while tokens it withdrew are still live. The last access token can be
  // minted a second before the refresh token expires and is then promised a
  // full token_expiry_seconds; sweeping on expires_at alone would start
  // honouring it again about a minute later, with most of its life left.
  // Holding every record - revoked ones included, they take the same branch -
  // until nothing minted from it can still be valid costs one extra lifetime
  // of memory per client and removes the whole race.
  const int64_t grace = static_cast<int64_t>(config_.token_expiry_seconds);

  size_t count = 0;
  for (auto it = refresh_tokens_.begin(); it != refresh_tokens_.end();) {
    if (it->second.expires_at + grace < now_ts) {
      it = refresh_tokens_.erase(it);
      ++count;
    } else {
      ++it;
    }
  }
  return count;
}

size_t AuthManager::refresh_token_count() const {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  return refresh_tokens_.size();
}

size_t AuthManager::cleanup_expired_tokens() {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  return cleanup_expired_locked();
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

tl::expected<JwtClaims, std::string> AuthManager::decode_jwt(const std::string & token, Expiry expiry) const {
  try {
    // Decode token first
    auto decoded = jwt::decode(token);

    // Verify signature
    try {
      switch (config_.jwt_algorithm) {
        case JwtAlgorithm::HS256: {
          auto verifier =
              jwt::verify().allow_algorithm(jwt::algorithm::hs256{config_.jwt_secret}).with_issuer(config_.issuer);
          if (expiry == Expiry::IGNORE) {
            verifier.with_claim("exp", [](const auto &, std::error_code &) {});
          }
          verifier.verify(decoded);
          break;
        }

        case JwtAlgorithm::RS256: {
          auto verifier = jwt::verify()
                              .allow_algorithm(jwt::algorithm::rs256(rs256_public_key_, "", "", ""))
                              .with_issuer(config_.issuer);
          if (expiry == Expiry::IGNORE) {
            verifier.with_claim("exp", [](const auto &, std::error_code &) {});
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

    if (decoded.has_payload_claim("role")) {
      claims.role = string_to_role(decoded.get_payload_claim("role").as_string());
    }

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

void AuthManager::store_refresh_token(const RefreshTokenRecord & record) {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);

  // Sweep before inserting, which bounds the map on the authorisation path.
  //
  // One of three sweeps, and they cover different traffic. This one runs per
  // authorisation; refresh_access_token() runs one per refresh, which is the
  // only path a client that logs in once and refreshes forever ever touches;
  // and the node drives a timer for a process doing neither. Keeping it here
  // as well makes the bound a property of the data structure, observable in a
  // test with no timer running and no wall clock to wait on.
  //
  // The cost is a scan per authorisation. The map only ever holds unexpired
  // records, so it is sized by how many tokens are live at once, not by how
  // many have ever been issued.
  cleanup_expired_locked();

  refresh_tokens_[record.token_id] = record;
}

std::optional<RefreshTokenRecord> AuthManager::get_refresh_token(const std::string & token_id) const {
  std::lock_guard<std::mutex> lock(refresh_tokens_mutex_);
  auto it = refresh_tokens_.find(token_id);
  if (it == refresh_tokens_.end()) {
    return std::nullopt;
  }
  return it->second;
}

}  // namespace ros2_medkit_gateway
