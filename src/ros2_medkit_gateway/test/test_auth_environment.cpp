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

// MEDKIT_CLIENTS parsing: one test per class of input. The precedence rules
// between the three variables are covered by the integration tests.

#include <gtest/gtest.h>

#include <algorithm>
#include <string>

#include "ros2_medkit_gateway/core/auth/auth_environment.hpp"

using namespace ros2_medkit_gateway;

namespace {

AuthEnvironment parse(const std::string & clients) {
  return resolve_auth_environment(std::nullopt, std::string("a_secret_of_at_least_32_characters_xx"), clients);
}

bool mentions(const AuthEnvironment & env, const std::string & needle) {
  return std::any_of(env.notices.begin(), env.notices.end(), [&needle](const std::string & n) {
    return n.find(needle) != std::string::npos;
  });
}

}  // namespace

TEST(AuthEnvironmentClientsTest, EntryWithoutColonIsRefusedByPosition) {
  const auto env = parse("good:s:admin,nocolon");
  ASSERT_EQ(env.clients.size(), 1U);
  EXPECT_TRUE(mentions(env, "entry 2 is not <id>:<secret>:<role>"));
  EXPECT_FALSE(mentions(env, "nocolon"));
}

TEST(AuthEnvironmentClientsTest, TwoFieldsIsNotAClient) {
  // "svc:admin" would be a client whose secret is its role name under a loose colon check.
  for (const auto & entry : {"svc:secret", "svc:admin"}) {
    const auto env = parse(entry);
    EXPECT_TRUE(env.clients.empty()) << entry;
    EXPECT_TRUE(mentions(env, "entry 1 is not <id>:<secret>:<role>")) << entry;
  }
}

TEST(AuthEnvironmentClientsTest, UnknownRoleIsRefusedWithoutEchoingTheField) {
  const auto env = parse("svc:admin:s3cret_in_role_field");
  EXPECT_TRUE(env.clients.empty());
  EXPECT_TRUE(mentions(env, "names an unknown role"));
  EXPECT_FALSE(mentions(env, "s3cret_in_role_field"));
}

TEST(AuthEnvironmentClientsTest, EmptyIdIsRefused) {
  const auto env = parse(":secret:admin");
  EXPECT_TRUE(env.clients.empty());
  EXPECT_TRUE(mentions(env, "empty id, secret or role"));
}

TEST(AuthEnvironmentClientsTest, EmptySecretIsRefused) {
  const auto env = parse("id::admin");
  EXPECT_TRUE(env.clients.empty());
  EXPECT_TRUE(mentions(env, "empty id, secret or role"));
}

TEST(AuthEnvironmentClientsTest, DuplicateIdKeepsTheFirstEntry) {
  const auto env = parse("svc:first:admin,svc:second:viewer");
  ASSERT_EQ(env.clients.size(), 1U);
  EXPECT_EQ(env.clients[0], "svc:first:admin");
  EXPECT_TRUE(mentions(env, "repeats the client id of entry 1"));
}

TEST(AuthEnvironmentClientsTest, SurroundingWhitespaceIsTrimmedAndInnerKept) {
  const auto env = parse(" one:a:admin ,\ttwo:b b:viewer\t");
  ASSERT_EQ(env.clients.size(), 2U);
  EXPECT_EQ(env.clients[0], "one:a:admin");
  EXPECT_EQ(env.clients[1], "two:b b:viewer");
}

TEST(AuthEnvironmentClientsTest, EmptyValueReplacesClientsWithNone) {
  const auto env = parse("");
  EXPECT_TRUE(env.clients_given);
  EXPECT_TRUE(env.clients.empty());
  EXPECT_EQ(env.client_entries_seen, 0U);
  EXPECT_TRUE(mentions(env, "MEDKIT_CLIENTS is empty"));
}
