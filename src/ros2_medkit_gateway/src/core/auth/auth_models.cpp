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

#include "ros2_medkit_gateway/core/auth/auth_models.hpp"

#include <jwt-cpp/base.h>

#include <cctype>
#include <initializer_list>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace ros2_medkit_gateway {

namespace {

constexpr const char * kJson = "application/json";
constexpr const char * kForm = "application/x-www-form-urlencoded";

// The media type of a Content-Type value: before any parameter, trimmed, lower case (RFC 9110 §8.3.1).
std::string media_type(const std::string & content_type) {
  std::string type = content_type.substr(0, content_type.find(';'));
  const size_t first = type.find_first_not_of(" \t");
  if (first == std::string::npos) {
    return {};
  }
  type = type.substr(first, type.find_last_not_of(" \t") - first + 1);
  for (auto & c : type) {
    c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
  }
  return type;
}

int hex_value(char c) {
  if (c >= '0' && c <= '9') {
    return c - '0';
  }
  const int lower = std::tolower(static_cast<unsigned char>(c));
  if (lower >= 'a' && lower <= 'f') {
    return lower - 'a' + 10;
  }
  return -1;
}

// application/x-www-form-urlencoded decoding that refuses a `%` not followed by two hex digits.
std::optional<std::string> form_decode_strict(const std::string & encoded) {
  std::string decoded;
  decoded.reserve(encoded.size());
  for (size_t i = 0; i < encoded.size(); ++i) {
    if (encoded[i] == '+') {
      decoded += ' ';
    } else if (encoded[i] != '%') {
      decoded += encoded[i];
    } else {
      if (i + 2 >= encoded.size()) {
        return std::nullopt;
      }
      const int high = hex_value(encoded[i + 1]);
      const int low = hex_value(encoded[i + 2]);
      if (high < 0 || low < 0) {
        return std::nullopt;
      }
      decoded += static_cast<char>(high * 16 + low);
      i += 2;
    }
  }
  return decoded;
}

// One field of a request body and the member it fills.
using FieldTargets = std::initializer_list<std::pair<const char *, std::optional<std::string> *>>;

// Fills `targets` from a form body. RFC 6749 §3.1 and §3.2: a field without a value is omitted, a field the caller
// reads must not repeat, and a field it does not read is ignored. Every pair must decode.
tl::expected<void, AuthErrorResponse> read_form(const std::string & body, FieldTargets targets) {
  std::set<std::string> seen;
  std::istringstream stream(body);
  std::string pair;
  while (std::getline(stream, pair, '&')) {
    if (pair.empty()) {
      continue;
    }
    const size_t eq_pos = pair.find('=');
    const auto key = form_decode_strict(pair.substr(0, eq_pos));
    const auto value = form_decode_strict(eq_pos == std::string::npos ? std::string{} : pair.substr(eq_pos + 1));
    if (!key || !value) {
      return tl::unexpected(
          AuthErrorResponse::invalid_request("The body is not valid application/x-www-form-urlencoded"));
    }
    for (const auto & [name, target] : targets) {
      if (*key != name) {
        continue;
      }
      if (!seen.insert(*key).second) {
        return tl::unexpected(AuthErrorResponse::invalid_request(*key + " appears more than once"));
      }
      if (!value->empty()) {
        *target = *value;
      }
    }
  }
  return {};
}

// Fills `targets` from a JSON object body. A field present and not a string is an error.
tl::expected<void, AuthErrorResponse> read_json(const std::string & body, FieldTargets targets) {
  nlohmann::json json_body;
  try {
    json_body = nlohmann::json::parse(body);
  } catch (const nlohmann::json::parse_error & e) {
    return tl::unexpected(AuthErrorResponse::invalid_request("Invalid JSON: " + std::string(e.what())));
  }
  if (!json_body.is_object()) {
    return tl::unexpected(AuthErrorResponse::invalid_request("The body must be a JSON object"));
  }
  for (const auto & [name, target] : targets) {
    const auto it = json_body.find(name);
    if (it == json_body.end()) {
      continue;
    }
    if (!it->is_string()) {
      return tl::unexpected(AuthErrorResponse::invalid_request(std::string(name) + " must be a string"));
    }
    *target = it->get<std::string>();
  }
  return {};
}

// Reads `targets` from a JSON or form body by its media type.
tl::expected<void, AuthErrorResponse> read_body(const std::string & content_type, const std::string & body,
                                                FieldTargets targets) {
  const std::string type = media_type(content_type);
  if (type == kJson) {
    return read_json(body, targets);
  }
  if (type == kForm) {
    return read_form(body, targets);
  }
  return tl::unexpected(
      AuthErrorResponse::invalid_request(std::string("Content-Type must be ") + kForm + " or " + kJson));
}

}  // namespace

tl::expected<AuthorizeRequest, AuthErrorResponse> AuthorizeRequest::from_form_data(const std::string & body) {
  return parse_request(kForm, body);
}

tl::expected<AuthorizeRequest, AuthErrorResponse> AuthorizeRequest::parse_request(const std::string & content_type,
                                                                                  const std::string & body) {
  AuthorizeRequest req;
  std::optional<std::string> grant_type;
  const auto read = read_body(content_type, body,
                              {{"grant_type", &grant_type},
                               {"client_id", &req.client_id},
                               {"client_secret", &req.client_secret},
                               {"refresh_token", &req.refresh_token},
                               {"scope", &req.scope}});
  if (!read) {
    return tl::unexpected(read.error());
  }
  req.grant_type = grant_type.value_or("");
  return req;
}

tl::expected<RevokeRequest, AuthErrorResponse> RevokeRequest::parse_request(const std::string & content_type,
                                                                            const std::string & body) {
  RevokeRequest req;
  std::optional<std::string> token;
  const auto read = read_body(content_type, body,
                              {{"token", &token},
                               {"token_type_hint", &req.token_type_hint},
                               {"client_id", &req.client_id},
                               {"client_secret", &req.client_secret}});
  if (!read) {
    return tl::unexpected(read.error());
  }
  if (!token.has_value() || token->empty()) {
    return tl::unexpected(AuthErrorResponse::invalid_request("token is required"));
  }
  req.token = std::move(*token);
  return req;
}

tl::expected<std::optional<BasicCredentials>, AuthErrorResponse>
parse_basic_authorization(const std::string & authorization) {
  // RFC 7235: the scheme is case-insensitive and followed by one or more spaces.
  constexpr size_t kSchemeLength = 5;
  if (authorization.size() < kSchemeLength) {
    return std::optional<BasicCredentials>{};
  }
  for (size_t i = 0; i < kSchemeLength; ++i) {
    if (std::tolower(static_cast<unsigned char>(authorization[i])) != "basic"[i]) {
      return std::optional<BasicCredentials>{};
    }
  }
  if (authorization.size() > kSchemeLength && authorization[kSchemeLength] != ' ') {
    return std::optional<BasicCredentials>{};
  }
  const size_t start = authorization.find_first_not_of(' ', kSchemeLength);
  const std::string encoded = start == std::string::npos ? std::string{} : authorization.substr(start);
  // The decoder copies the rest of the value once per trailing '='; base64 ends in at most two.
  const size_t last_data = encoded.find_last_not_of('=');
  const size_t padding = last_data == std::string::npos ? encoded.size() : encoded.size() - last_data - 1;
  if (padding > 2) {
    return tl::unexpected(AuthErrorResponse::invalid_request("Basic credentials are not valid base64"));
  }

  std::string decoded;
  try {
    decoded = jwt::base::decode<jwt::alphabet::base64>(encoded);
  } catch (const std::runtime_error &) {
    return tl::unexpected(AuthErrorResponse::invalid_request("Basic credentials are not valid base64"));
  }
  const size_t colon = decoded.find(':');
  if (colon == std::string::npos) {
    return tl::unexpected(AuthErrorResponse::invalid_request("Basic credentials have no ':' after the client_id"));
  }
  auto client_id = form_decode_strict(decoded.substr(0, colon));
  auto client_secret = form_decode_strict(decoded.substr(colon + 1));
  if (!client_id || !client_secret) {
    return tl::unexpected(
        AuthErrorResponse::invalid_request("Basic credentials are not application/x-www-form-urlencoded"));
  }
  return std::optional<BasicCredentials>{BasicCredentials{std::move(*client_id), std::move(*client_secret)}};
}

}  // namespace ros2_medkit_gateway
