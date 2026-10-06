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
#include <vector>

namespace ros2_medkit_gateway {

/// What three environment variables say about this gateway's authentication.
///
/// `applies` false means the environment said nothing and every `auth.*`
/// parameter keeps the value the params file or the command line gave it.
struct AuthEnvironment {
  /// The environment made a statement, so the fields below replace parameters.
  bool applies{false};

  /// The value `auth.enabled` takes.
  bool enabled{false};

  /// `auth.require_auth_for` is forced to "all". Never set without `enabled`:
  /// closing a gateway and leaving every read open is the one outcome the
  /// variable must not produce.
  bool require_auth_for_all{false};

  /// The value `auth.jwt_secret` takes. Empty unless `enabled`.
  std::string jwt_secret;

  /// `MEDKIT_CLIENTS` was set, so `clients` replaces `auth.clients` even when
  /// it is empty or every entry in it was refused. Leaving the file's
  /// credentials standing under a secret they were not issued against would be
  /// the surprise.
  bool clients_given{false};

  /// How many comma-separated entries carried anything at all.
  ///
  /// Separates "no credentials were asked for" (`MEDKIT_CLIENTS=`) from "every
  /// credential asked for was refused" (`MEDKIT_CLIENTS=typo`). Both leave
  /// `clients` empty; only the second is a list the operator wrote and the
  /// gateway could not use.
  std::size_t client_entries_seen{0};

  /// The value `auth.clients` takes, one "id:secret:role" entry per element.
  std::vector<std::string> clients;

  /// Lines to log at WARN, in order: what the environment overrode, then every
  /// client entry it refused. The operator is changing the gateway's posture
  /// from outside its configuration, so the log has to say what happened.
  std::vector<std::string> notices;
};

/// Applies the environment rule to three already-read values.
///
/// THE RULE, in one place:
///   - `MEDKIT_AUTH_DISABLED=1` turns authentication off and wins over
///     everything, the parameters and the other two variables included.
///   - otherwise a non-empty `MEDKIT_JWT_SECRET` turns authentication on, sets
///     `require_auth_for` to "all", and supplies the secret. "write" would
///     leave every read open, and the reads are the disclosure.
///   - otherwise the environment says nothing.
///
/// `MEDKIT_CLIENTS` is read only in the second case, where a credential can be
/// exchanged for a token; set on its own it is ignored, with a notice saying
/// so. Entries are separated by commas and each is written
/// `id:secret:role`: the id is everything before the first colon, the role
/// everything after the last, and the secret is what lies between - so a
/// secret may contain colons, while an id and a role may not, and no field may
/// contain a comma. Roles are `viewer`, `operator`, `configurator`, `admin`.
/// Surrounding spaces and tabs are dropped from each entry, so `a:b:admin,
/// c:d:viewer` works; space inside a field is part of that field. An entry
/// that does not parse is refused and named by its position, and so is one
/// repeating an id an earlier entry claimed; an empty entry is a separator
/// artefact and is skipped in silence.
///
/// Values are passed in so the rule can be exercised without touching the
/// process environment.
///
/// @param auth_disabled Value of MEDKIT_AUTH_DISABLED, or nullopt if unset
/// @param jwt_secret    Value of MEDKIT_JWT_SECRET, or nullopt if unset
/// @param clients       Value of MEDKIT_CLIENTS, or nullopt if unset
/// @return What the environment decided, and what to log about it
AuthEnvironment resolve_auth_environment(const std::optional<std::string> & auth_disabled,
                                         const std::optional<std::string> & jwt_secret,
                                         const std::optional<std::string> & clients);

/// Reads the three variables from the process environment and applies the rule.
AuthEnvironment resolve_auth_environment_from_process();

/// The `auth.clients` list as the parameter services may show it: one
/// `<id>:<sentinel>:<role>` per entry the gateway registers, in order. An
/// entry that does not parse or names an unknown role is omitted; it is
/// registered by nothing, and showing it would print whatever was put where
/// the secret goes.
std::vector<std::string> redact_client_entries(const std::vector<std::string> & entries, const std::string & sentinel);

}  // namespace ros2_medkit_gateway
