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

/// Drops leading and trailing spaces and tabs.
///
/// Applied to a whole entry, never inside one. `a:b:admin, c:d:viewer` is how
/// a list gets written by hand, and the space after the comma belongs to the
/// separator; a space inside a secret belongs to the secret.
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
///
/// The check is on the SHAPE and on the role name, both of which are things a
/// caller can get wrong in a way that leaves the gateway closed to its own
/// operator. A refused entry is dropped and the entries around it still
/// register: one typo must not take the other credentials with it.
///
/// `entries_seen` counts the entries that carried anything, so the caller can
/// tell "no credentials were asked for" from "every credential asked for was
/// refused" - two states that look the same in `accepted` and are not the same
/// misconfiguration.
///
/// Positions count every comma-separated field as written, empties included,
/// so the number in a notice matches what an operator can point at in the
/// variable.
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

    // A trailing or doubled comma is a separator artefact, not an entry
    // somebody wrote wrong. Saying nothing about it keeps the warnings that do
    // appear worth reading.
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
      // The role field is not quoted: an entry written `id:role:secret` puts
      // the secret there, and every notice goes to a log.
      notice += " for id \"";
      notice += id;
      notice += "\" names an unknown role (viewer, operator, configurator or admin) and was dropped";
      notices.push_back(std::move(notice));
      continue;
    }

    // First entry for an id wins. Taking the last would make the credential in
    // force depend on a position nobody thinks about, and the id is what a
    // client authenticates as.
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

  // Exactly "1". A variable set to "true", "yes" or "0" is not the documented
  // opt-out, and reading any of them as one would turn authentication off for
  // somebody who meant the opposite.
  if (auth_disabled.has_value() && auth_disabled.value() == "1") {
    env.applies = true;
    env.enabled = false;
    env.notices.push_back(
        "MEDKIT_AUTH_DISABLED=1: authentication is OFF and every route is readable by anyone who can reach this "
        "port. This overrides auth.enabled, MEDKIT_JWT_SECRET and every other source.");
    return env;
  }

  if (!jwt_secret.has_value() || jwt_secret.value().empty()) {
    // A set-but-empty secret closes nothing, and says so. The variable is
    // documented as non-empty, and an operator who exported it empty is
    // holding half a configuration; silence would read as "closed".
    if (jwt_secret.has_value()) {
      env.notices.push_back(
          "MEDKIT_JWT_SECRET is set but empty, so it closes nothing; auth.enabled and auth.jwt_secret from the "
          "parameters stand. Set it to a secret of at least 32 characters to close this gateway.");
    }
    // MEDKIT_CLIENTS on its own does nothing, and says so.
    //
    // Credentials are read only where this gateway is the one closing, because
    // replacing auth.clients under a secret the operator did not set would
    // change who can log in to a gateway they did not ask to change. Setting
    // the variable alone is a reasonable mistake - it looks like half a
    // configuration and behaves like none - so it gets a line of its own.
    if (clients.has_value() && !clients.value().empty()) {
      env.notices.push_back(
          "MEDKIT_CLIENTS is set and MEDKIT_JWT_SECRET is unset or empty, so the environment is closing nothing "
          "and MEDKIT_CLIENTS is ignored. auth.clients from the parameters stands. Set MEDKIT_JWT_SECRET as well "
          "to close this gateway with the credentials in MEDKIT_CLIENTS.");
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
    // Set means replace, the empty string included. A variable an operator set
    // to nothing is a statement about the credentials this gateway offers, and
    // falling back to the file's clients under a secret they were never issued
    // against is the outcome that statement is meant to prevent.
    env.clients_given = true;
    env.clients = parse_client_entries(clients.value(), env.notices, env.client_entries_seen);

    std::string notice = "MEDKIT_CLIENTS supplied ";
    notice += std::to_string(env.clients.size());
    notice += " client credential(s), overriding auth.clients.";
    env.notices.push_back(std::move(notice));

    if (env.client_entries_seen == 0) {
      env.notices.push_back(
          "MEDKIT_CLIENTS is empty, so auth.clients is replaced by nothing and no client can obtain a token. Every "
          "route will refuse every caller. Give it <id>:<secret>:<role> entries, or unset it to keep auth.clients.");
    }
  }

  return env;
}

std::vector<std::string> redact_client_entries(const std::vector<std::string> & entries, const std::string & sentinel) {
  // The same reading GatewayNode registers by: two distinct colons, the id
  // before the first, the role after the last, and a role the gateway knows.
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
