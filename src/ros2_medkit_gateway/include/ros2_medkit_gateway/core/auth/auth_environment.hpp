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

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace ros2_medkit_gateway {

/// What `MEDKIT_AUTH_DISABLED`, `MEDKIT_JWT_SECRET` and `MEDKIT_CLIENTS` say
/// about authentication. When `applies` is false the `auth.*` parameters keep
/// the values their sources gave them.
struct AuthEnvironment {
  /// The environment made a statement; the fields below replace parameters.
  bool applies{false};

  /// `MEDKIT_AUTH_DISABLED` was exactly `1`.
  bool auth_disabled{false};

  /// `MEDKIT_JWT_SECRET` was set to the empty string. The node refuses to start.
  bool jwt_secret_empty{false};

  /// The value `auth.enabled` takes.
  bool enabled{false};

  /// `auth.require_auth_for` is forced to "all". Only set together with `enabled`.
  bool require_auth_for_all{false};

  /// The value `auth.jwt_secret` takes. Empty unless `enabled`.
  std::string jwt_secret;

  /// `MEDKIT_CLIENTS` was set, so `clients` replaces `auth.clients` even when empty.
  bool clients_given{false};

  /// Comma-separated entries that carried anything. Zero means an empty value;
  /// non-zero with empty `clients` means every entry was refused.
  std::size_t client_entries_seen{0};

  /// The value `auth.clients` takes, one `id:secret:role` per element.
  std::vector<std::string> clients;

  /// Lines to log at WARN, in order.
  std::vector<std::string> notices;
};

/// Applies the environment rule to three already-read values.
///
/// - `MEDKIT_AUTH_DISABLED=1` turns authentication off and wins over everything.
/// - A non-empty `MEDKIT_JWT_SECRET` turns authentication on, sets
///   `require_auth_for` to "all" and supplies the secret.
/// - An empty `MEDKIT_JWT_SECRET` sets `jwt_secret_empty`.
/// - `MEDKIT_CLIENTS` is read only with a non-empty secret; entries are
///   `id:secret:role`, split on commas. The id ends at the first colon and the
///   role starts after the last, so a secret may contain colons. Surrounding
///   spaces and tabs are trimmed. A malformed or repeated-id entry is refused
///   and named by position; an empty entry is skipped silently.
AuthEnvironment resolve_auth_environment(const std::optional<std::string> & auth_disabled,
                                         const std::optional<std::string> & jwt_secret,
                                         const std::optional<std::string> & clients);

/// Reads the three variables from the process environment and applies the rule.
AuthEnvironment resolve_auth_environment_from_process();

/// The `auth.clients` list as the parameter services may show it: one
/// `<id>:<sentinel>:<role>` per entry the gateway registers. Entries that do
/// not parse or name an unknown role are omitted, since they may hold a secret
/// in any field.
std::vector<std::string> redact_client_entries(const std::vector<std::string> & entries, const std::string & sentinel);

}  // namespace ros2_medkit_gateway
