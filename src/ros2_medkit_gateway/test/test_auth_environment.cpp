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

/// The environment rule the gateway node applies to its auth parameters.
///
/// One rule, one place, and this sweeps its whole input space: the three
/// variables are independent, so the interesting cases are the combinations,
/// not a single happy path. The precedence between MEDKIT_AUTH_DISABLED and
/// MEDKIT_JWT_SECRET is the half that decides whether a container its operator
/// believes closed is actually closed.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <vector>

#include "ros2_medkit_gateway/core/auth/auth_environment.hpp"

using namespace ros2_medkit_gateway;

namespace {

constexpr const char * kSecret = "an_environment_supplied_secret_of_at_least_32";

std::optional<std::string> unset() {
  return std::nullopt;
}

/// True when some notice mentions `needle`. The notices are what an operator
/// reads to learn the environment overrode their file, so their presence is
/// part of the contract, not decoration.
bool mentions(const AuthEnvironment & env, const std::string & needle) {
  return std::any_of(env.notices.begin(), env.notices.end(), [&needle](const std::string & n) {
    return n.find(needle) != std::string::npos;
  });
}

}  // namespace

// Nothing set: the environment says nothing and every parameter stands.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AnEmptyEnvironmentDecidesNothing) {
  auto env = resolve_auth_environment(unset(), unset(), unset());
  EXPECT_FALSE(env.applies);
  EXPECT_TRUE(env.notices.empty());
  EXPECT_FALSE(env.clients_given);
}

// The closing statement, and the whole of it: on, "all", the secret, and the
// clients. Leaving require_auth_for at the open profile's "write" was the
// failure this rule exists to prevent - every read stays open and the operator
// is told the gateway is closed.
// @verifies REQ_INTEROP_086, REQ_INTEROP_087
TEST(AuthEnvironmentTest, ASecretClosesTheGatewayForEveryRoute) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret), std::string("svc:svc_secret:admin"));

  EXPECT_TRUE(env.applies);
  EXPECT_TRUE(env.enabled);
  EXPECT_TRUE(env.require_auth_for_all);
  EXPECT_EQ(env.jwt_secret, kSecret);
  ASSERT_EQ(env.clients.size(), 1U);
  EXPECT_EQ(env.clients[0], "svc:svc_secret:admin");
  EXPECT_TRUE(env.clients_given);
  EXPECT_TRUE(mentions(env, "MEDKIT_JWT_SECRET"));
}

// MEDKIT_AUTH_DISABLED=1 wins, including over a secret that would otherwise
// close the gateway.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, DisabledWinsOverASecret) {
  auto env = resolve_auth_environment(std::string("1"), std::string(kSecret), std::string("svc:svc_secret:admin"));

  EXPECT_TRUE(env.applies);
  EXPECT_FALSE(env.enabled);
  EXPECT_FALSE(env.require_auth_for_all);
  EXPECT_TRUE(env.jwt_secret.empty());
  EXPECT_TRUE(env.clients.empty());
  EXPECT_TRUE(mentions(env, "MEDKIT_AUTH_DISABLED=1"));
}

// Only the exact string "1". A variable set to something that looks
// affirmative must not turn authentication off for somebody who meant the
// opposite, and "0" must not turn it off either.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, OnlyTheLiteralOneDisables) {
  for (const auto & value : {"", "0", "true", "TRUE", "yes", "01", "1 ", " 1", "2"}) {
    auto env = resolve_auth_environment(std::string(value), std::string(kSecret), std::string("svc:s:admin"));
    EXPECT_TRUE(env.enabled) << "MEDKIT_AUTH_DISABLED=\"" << value << "\" was read as the opt-out";
  }
}

// Disabled alone, with no secret anywhere: still a statement, and still the
// one that must reach the node - a params file with auth on has to lose.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, DisabledAloneStillOverridesTheParameters) {
  auto env = resolve_auth_environment(std::string("1"), unset(), unset());
  EXPECT_TRUE(env.applies);
  EXPECT_FALSE(env.enabled);
}

// An empty secret is not a secret. Docker sets a variable to the empty string
// when `-e MEDKIT_JWT_SECRET` is passed with no value, and reading that as
// "close the gateway" would produce a gateway with authentication on and an
// empty signing key, which refuses to start.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AnEmptySecretIsNotAStatement) {
  auto env = resolve_auth_environment(unset(), std::string(""), std::string("svc:s:admin"));
  EXPECT_FALSE(env.applies);
  EXPECT_FALSE(env.clients_given);
}

// Clients are read only where a token can be obtained. With authentication
// off, replacing auth.clients would be a change nobody asked for.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, ClientsAreIgnoredWhereNoTokenCanBeIssued) {
  auto disabled = resolve_auth_environment(std::string("1"), unset(), std::string("svc:s:admin"));
  EXPECT_FALSE(disabled.clients_given);

  auto silent = resolve_auth_environment(unset(), unset(), std::string("svc:s:admin"));
  EXPECT_FALSE(silent.clients_given);
}

// MEDKIT_CLIENTS on its own closes nothing, and the operator is told.
//
// Credentials are read only where the environment is what closes this gateway.
// Setting the variable alone looks like half a configuration and behaves like
// none, so silence here reads as "the credential was accepted" to whoever set
// it and then cannot log in.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, ClientsWithoutASecretAreIgnoredOutLoud) {
  auto env = resolve_auth_environment(unset(), unset(), std::string("svc:s:admin"));

  EXPECT_FALSE(env.applies);
  EXPECT_FALSE(env.clients_given) << "auth.clients was replaced with no secret to close the gateway";
  EXPECT_TRUE(mentions(env, "MEDKIT_CLIENTS is set and MEDKIT_JWT_SECRET is unset or empty"))
      << "MEDKIT_CLIENTS was ignored in silence";
  EXPECT_TRUE(mentions(env, "auth.clients from the parameters stands"));
}

// A set-but-empty MEDKIT_JWT_SECRET closes nothing, and says so. The variable
// is documented as non-empty, and an operator who exported it empty is holding
// half a configuration; silence here reads as "the gateway is closed".
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AnEmptySecretClosesNothingAndSaysSo) {
  auto env = resolve_auth_environment(unset(), std::string(""), unset());
  EXPECT_FALSE(env.applies);
  EXPECT_TRUE(mentions(env, "MEDKIT_JWT_SECRET is set but empty"))
      << "an empty secret was folded into unset in silence";
}

// With clients beside it, the notice says the secret is empty, not that it is
// unset: the operator did set it.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, ClientsWithAnEmptySecretAreToldTheSecretIsEmpty) {
  auto env = resolve_auth_environment(unset(), std::string(""), std::string("svc:s:admin"));
  EXPECT_FALSE(env.applies);
  EXPECT_FALSE(env.clients_given);
  EXPECT_TRUE(mentions(env, "MEDKIT_CLIENTS is set and MEDKIT_JWT_SECRET is unset or empty"));
}

// A swapped entry `id:role:secret` puts the secret where the role goes; the
// notice names the position and the id and never the role field.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, ABadRoleFieldIsNotEchoed) {
  auto env = resolve_auth_environment(unset(), std::string("a_secret_of_at_least_32_characters_long_x"),
                                      std::string("svc:admin:s3cret_swapped_zz"));
  EXPECT_TRUE(env.clients.empty());
  EXPECT_TRUE(mentions(env, "names an unknown role"));
  for (const auto & notice : env.notices) {
    EXPECT_EQ(notice.find("s3cret_swapped_zz"), std::string::npos) << notice;
  }
}

// The redacted client list mirrors what the gateway registers: one
// `<id>:<sentinel>:<role>` per entry with two distinct colons and a role the
// gateway knows, in order, the role in its canonical spelling. Anything else is
// omitted, because a malformed entry may carry its secret in any field.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, RedactedClientEntriesShowIdAndRoleOnly) {
  const std::vector<std::string> entries = {"a:s3cret:admin",  "b:s3cret:Viewer",    "nocolons", "c:s3cret:wizard",
                                            "twofield:s3cret", "d:s3:cret:operator", ""};
  const auto shown = redact_client_entries(entries, "<x>");
  const std::vector<std::string> expected = {"a:<x>:admin", "b:<x>:viewer", "d:<x>:operator"};
  EXPECT_EQ(shown, expected);
  for (const auto & line : shown) {
    EXPECT_EQ(line.find("s3"), std::string::npos) << line;
  }
  EXPECT_TRUE(redact_client_entries({}, "<x>").empty());
  EXPECT_TRUE(redact_client_entries({""}, "<x>").empty()) << "the empty-sequence idiom is not an entry";
}

// An empty MEDKIT_CLIENTS with no secret is not worth a line: nothing was
// asked for and nothing happened.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AnEmptyClientsVariableWithNoSecretSaysNothing) {
  auto env = resolve_auth_environment(unset(), unset(), std::string(""));
  EXPECT_FALSE(env.applies);
  EXPECT_TRUE(env.notices.empty());
}

// Set means replace, and the empty string is set. Falling back to the file's
// credentials here would hand out tokens against a secret those credentials
// were never issued under.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AnEmptyClientListReplacesTheFilesClientsWithNone) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret), std::string(""));

  EXPECT_TRUE(env.clients_given) << "an empty MEDKIT_CLIENTS left auth.clients standing";
  EXPECT_TRUE(env.clients.empty());
  EXPECT_EQ(env.client_entries_seen, 0U) << "an empty value must not look like a list that failed to parse";
  EXPECT_TRUE(mentions(env, "no client can obtain a token"))
      << "the consequence of an empty client list was not stated";
}

// The separator is a comma, and a list written by hand puts a space after it.
// Space INSIDE a field belongs to that field: a secret may legitimately carry
// one, and trimming it would authenticate a credential the operator did not
// configure.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, SurroundingSpaceIsDroppedAndInnerSpaceIsKept) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret),
                                      std::string(" one:a:admin ,\ttwo:b:viewer\t, three:c c:operator "));

  ASSERT_EQ(env.clients.size(), 3U);
  EXPECT_EQ(env.clients[0], "one:a:admin");
  EXPECT_EQ(env.clients[1], "two:b:viewer");
  EXPECT_EQ(env.clients[2], "three:c c:operator") << "a space inside the secret was trimmed away";
}

// A whitespace-only entry is a separator artefact like an empty one.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AWhitespaceOnlyEntryIsSkippedInSilence) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret), std::string("one:a:admin,   ,two:b:viewer"));

  EXPECT_EQ(env.clients.size(), 2U);
  EXPECT_FALSE(mentions(env, "entry 2")) << "a whitespace-only entry was reported as malformed";
}

// Positions name what an operator can point at in the variable, so they count
// every comma-separated field as written, empties included.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, PositionsCountEveryFieldAsWritten) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret), std::string("one:a:admin,,,bad,two:b:viewer"));

  EXPECT_EQ(env.clients.size(), 2U);
  EXPECT_TRUE(mentions(env, "entry 4")) << "the malformed entry is the fourth field and was numbered otherwise";
  EXPECT_FALSE(mentions(env, "entry 2"));
  EXPECT_FALSE(mentions(env, "entry 3"));
}

// An id names the credential a client authenticates as. Taking the last entry
// would make the secret in force depend on a position nobody thinks about.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, TheFirstEntryForAnIdWins) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret),
                                      std::string("svc:first:admin,svc:second:viewer,other:x:viewer"));

  ASSERT_EQ(env.clients.size(), 2U);
  EXPECT_EQ(env.clients[0], "svc:first:admin");
  EXPECT_EQ(env.clients[1], "other:x:viewer");
  EXPECT_TRUE(mentions(env, "repeats client id")) << "the duplicate was dropped without saying so";
  EXPECT_TRUE(mentions(env, "entry 2"));
}

// The state the caller has to be able to distinguish: a list that was written
// and refused entirely, which is a misconfiguration, against an empty value,
// which is a decision. Both leave `clients` empty.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, EntriesSeenSeparatesAnEmptyListFromARefusedOne) {
  auto refused = resolve_auth_environment(unset(), std::string(kSecret), std::string("nocolons,alsobad"));
  EXPECT_TRUE(refused.clients.empty());
  EXPECT_EQ(refused.client_entries_seen, 2U);

  auto empty = resolve_auth_environment(unset(), std::string(kSecret), std::string(",,"));
  EXPECT_TRUE(empty.clients.empty());
  EXPECT_EQ(empty.client_entries_seen, 0U);
}

// The separator contract: commas between entries, three colon-separated fields
// in each, and a secret that may itself contain colons because the id stops at
// the FIRST colon and the role starts after the LAST.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, ClientsSplitOnCommasAndKeepColonsInTheSecret) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret),
                                      std::string("one:a:admin,two:b1:b2:b3:viewer,three:c:operator"));

  ASSERT_EQ(env.clients.size(), 3U);
  EXPECT_EQ(env.clients[0], "one:a:admin");
  EXPECT_EQ(env.clients[1], "two:b1:b2:b3:viewer") << "a secret containing colons was mangled";
  EXPECT_EQ(env.clients[2], "three:c:operator");
}

// Every role name the gateway knows, and nothing else.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, EveryDocumentedRoleIsAccepted) {
  auto env =
      resolve_auth_environment(unset(), std::string(kSecret),
                               std::string("a:s:viewer,b:s:operator,c:s:configurator,d:s:admin,e:s:ADMIN,f:s:Viewer"));
  EXPECT_EQ(env.clients.size(), 6U) << "a documented role was refused, or case-folding stopped working";
}

// A malformed entry is dropped, named by its position, and takes nothing else
// with it. Dropping the whole list would leave a closed container nobody can
// open, which is worse than the typo.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AMalformedEntryIsDroppedAndNamedByPosition) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret),
                                      std::string("good:s:admin,nocolons,bad:s:wizard,:s:admin,last:s:viewer"));

  ASSERT_EQ(env.clients.size(), 2U);
  EXPECT_EQ(env.clients[0], "good:s:admin");
  EXPECT_EQ(env.clients[1], "last:s:viewer");

  EXPECT_TRUE(mentions(env, "entry 2")) << "the entry with no colons was not named";
  EXPECT_TRUE(mentions(env, "entry 3")) << "the entry with an unknown role was not named";
  EXPECT_TRUE(mentions(env, "entry 4")) << "the entry with an empty id was not named";
  EXPECT_TRUE(mentions(env, "names an unknown role")) << "the entry with an unknown role was not explained";
  EXPECT_FALSE(mentions(env, "wizard")) << "the role field was quoted back, and a swapped entry puts the secret there";
}

// Degenerate separators. A trailing or doubled comma is a habit, not a typo,
// and warning about it would drown the warnings that matter.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, EmptyEntriesAreSkippedWithoutComplaint) {
  auto env = resolve_auth_environment(unset(), std::string(kSecret), std::string(",,one:s:admin,,two:s:viewer,,"));

  ASSERT_EQ(env.clients.size(), 2U);
  EXPECT_FALSE(mentions(env, "entry 1")) << "an empty entry was reported as malformed";
  EXPECT_FALSE(mentions(env, "and was dropped"));
}

// Every field has to be there. An entry with an empty secret would register a
// client authenticating on the empty string.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, AnEmptyFieldRefusesTheEntry) {
  for (const auto & entry : {":secret:admin", "id::admin", "id:secret:", "::", "::admin"}) {
    auto env = resolve_auth_environment(unset(), std::string(kSecret), std::string(entry));
    EXPECT_TRUE(env.clients.empty()) << "entry \"" << entry << "\" was accepted";
    EXPECT_TRUE(env.clients_given) << "entry \"" << entry << "\" left auth.clients standing";
  }
}

// Two fields is the shape people reach for first, and "id:role" is the one
// that gets through a check counting colons loosely: with only the presence of
// a colon required, the field after it is read as the role, it parses, and a
// client is registered whose secret is its own role name. Every entry here has
// one colon; the last three end in a legal role and are the discriminating
// cases.
// @verifies REQ_INTEROP_086
TEST(AuthEnvironmentTest, TwoFieldsIsNotAClient) {
  for (const auto & entry : {"svc:secret", "svc:", ":svc", "svc:admin", "svc:viewer", "svc:operator"}) {
    auto env = resolve_auth_environment(unset(), std::string(kSecret), std::string(entry));
    EXPECT_TRUE(env.clients.empty()) << "\"" << entry << "\" was registered as a client";
    EXPECT_TRUE(mentions(env, "entry 1")) << "\"" << entry << "\" was dropped without naming its position";
    EXPECT_TRUE(mentions(env, "is not <id>:<secret>:<role>"))
        << "\"" << entry
        << "\" was refused for some other reason than its shape, so the shape check is not what "
           "caught it";
  }
}
