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

#include "ros2_medkit_gateway/core/auth/auth_environment.hpp"

#include <algorithm>
#include <cstdlib>
#include <set>
#include <stdexcept>

#include "ros2_medkit_gateway/core/auth/auth_config.hpp"

namespace ros2_medkit_gateway {

namespace {

std::optional<std::string> read_env(const char * name) {
  const char * value = std::getenv(name);
  if (value == nullptr) {
    return std::nullopt;
  }
  return std::string(value);
}

/// Drops leading and trailing spaces and tabs from a whole entry.
std::string trim_surrounding_space(const std::string & value) {
  const auto is_space = [](unsigned char c) {
    return c == ' ' || c == '\t';
  };
  std::size_t begin = 0;
  while (begin < value.size() && is_space(static_cast<unsigned char>(value[begin]))) {
    ++begin;
  }
  std::size_t end = value.size();
  while (end > begin && is_space(static_cast<unsigned char>(value[end - 1]))) {
    --end;
  }
  return value.substr(begin, end - begin);
}

/// Splits MEDKIT_CLIENTS and checks each entry, appending a notice per refusal.
/// Notices carry the 1-based position as written, empties included, and never
/// the entry text: any field of a malformed entry may hold a secret.
std::vector<std::string> parse_client_entries(const std::string & value, std::vector<std::string> & notices,
                                              std::size_t & entries_seen) {
  std::vector<std::string> accepted;
  std::set<std::string> ids_taken;

  std::size_t position = 0;
  std::size_t start = 0;
  while (start <= value.size()) {
    const std::size_t comma = value.find(',', start);
    const std::string raw = value.substr(start, comma == std::string::npos ? std::string::npos : comma - start);
    start = comma == std::string::npos ? value.size() + 1 : comma + 1;
    ++position;

    const std::string entry = trim_surrounding_space(raw);
    if (entry.empty()) {
      continue;
    }
    ++entries_seen;

    std::string notice = "MEDKIT_CLIENTS entry ";
    notice += std::to_string(position);

    const std::size_t first_colon = entry.find(':');
    const std::size_t last_colon = entry.rfind(':');
    if (first_colon == std::string::npos || first_colon == last_colon) {
      notice += " is not <id>:<secret>:<role> and was dropped";
      notices.push_back(std::move(notice));
      continue;
    }

    const std::string id = entry.substr(0, first_colon);
    const std::string secret = entry.substr(first_colon + 1, last_colon - first_colon - 1);
    const std::string role = entry.substr(last_colon + 1);
    if (id.empty() || secret.empty() || role.empty()) {
      notice += " has an empty id, secret or role and was dropped";
      notices.push_back(std::move(notice));
      continue;
    }

    try {
      (void)string_to_role(role);
    } catch (const std::invalid_argument &) {
      // An entry written `id:role:secret` has its secret in the role field.
      notice += " for id \"";
      notice += id;
      notice += "\" names an unknown role (viewer, operator, configurator or admin) and was dropped";
      notices.push_back(std::move(notice));
      continue;
    }

    if (!ids_taken.insert(id).second) {
      notice += " repeats client id \"";
      notice += id;
      notice += "\", which an earlier entry already claimed; the earlier one stands";
      notices.push_back(std::move(notice));
      continue;
    }

    accepted.push_back(entry);
  }

  return accepted;
}

}  // namespace

AuthEnvironment resolve_auth_environment(const std::optional<std::string> & auth_disabled,
                                         const std::optional<std::string> & jwt_secret,
                                         const std::optional<std::string> & clients) {
  AuthEnvironment env;

  // Exactly "1": "true" or "0" is not the documented opt-out.
  if (auth_disabled.has_value() && auth_disabled.value() == "1") {
    env.applies = true;
    env.auth_disabled = true;
    env.enabled = false;
    env.notices.push_back(
        "MEDKIT_AUTH_DISABLED=1: authentication is OFF and every route is readable by anyone who can reach this "
        "port. This overrides auth.enabled, MEDKIT_JWT_SECRET and every other source.");
    return env;
  }

  if (jwt_secret.has_value() && jwt_secret.value().empty()) {
    env.jwt_secret_empty = true;
    return env;
  }

  if (!jwt_secret.has_value()) {
    if (clients.has_value() && !clients.value().empty()) {
      env.notices.push_back(
          "MEDKIT_CLIENTS is set and MEDKIT_JWT_SECRET is unset, so MEDKIT_CLIENTS is ignored and auth.clients from "
          "the parameters stands. Set MEDKIT_JWT_SECRET as well to use the credentials in MEDKIT_CLIENTS.");
    }
    return env;
  }

  env.applies = true;
  env.enabled = true;
  env.require_auth_for_all = true;
  env.jwt_secret = jwt_secret.value();
  env.notices.push_back(
      "MEDKIT_JWT_SECRET is set: authentication is ON, auth.require_auth_for is \"all\" and the secret comes from "
      "the environment. This overrides auth.enabled, auth.require_auth_for and auth.jwt_secret.");

  if (clients.has_value()) {
    // Set means replace, the empty string included.
    env.clients_given = true;
    env.clients = parse_client_entries(clients.value(), env.notices, env.client_entries_seen);

    std::string notice = "MEDKIT_CLIENTS supplied ";
    notice += std::to_string(env.clients.size());
    notice += " client credential(s), overriding auth.clients.";
    env.notices.push_back(std::move(notice));

    if (env.client_entries_seen == 0) {
      env.notices.push_back(
          "MEDKIT_CLIENTS is empty, so auth.clients is replaced by nothing and no client can obtain a token. Give "
          "it <id>:<secret>:<role> entries, or unset it to keep auth.clients.");
    }
  }

  return env;
}

std::vector<std::string> redact_client_entries(const std::vector<std::string> & entries, const std::string & sentinel) {
  std::vector<std::string> shown;
  shown.reserve(entries.size());
  for (const auto & entry : entries) {
    const std::size_t first_colon = entry.find(':');
    const std::size_t last_colon = entry.rfind(':');
    if (first_colon == std::string::npos || first_colon == last_colon) {
      continue;
    }
    UserRole role;
    try {
      role = string_to_role(entry.substr(last_colon + 1));
    } catch (const std::invalid_argument &) {
      continue;
    }
    shown.push_back(entry.substr(0, first_colon) + ":" + sentinel + ":" + role_to_string(role));
  }
  return shown;
}

AuthEnvironment resolve_auth_environment_from_process() {
  return resolve_auth_environment(read_env("MEDKIT_AUTH_DISABLED"), read_env("MEDKIT_JWT_SECRET"),
                                  read_env("MEDKIT_CLIENTS"));
}

}  // namespace ros2_medkit_gateway
