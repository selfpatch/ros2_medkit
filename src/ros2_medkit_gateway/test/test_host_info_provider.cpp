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

/**
 * @file test_host_info_provider.cpp
 * @brief Unit tests for HostInfoProvider - default Component from host system info
 */

#include <gtest/gtest.h>

#include <cctype>
#include <string>

#include "ros2_medkit_gateway/discovery/host_info_provider.hpp"

using ros2_medkit_gateway::HostInfoProvider;
using ros2_medkit_gateway::json;

// =============================================================================
// HostInfoProvider Tests
// =============================================================================

class HostInfoProviderTest : public ::testing::Test {
 protected:
  HostInfoProvider provider_;
};

// @verifies REQ_INTEROP_003
TEST_F(HostInfoProviderTest, creates_default_component) {
  const auto & comp = provider_.get_default_component();

  // id, name, source, type must be populated
  EXPECT_FALSE(comp.id.empty());
  EXPECT_FALSE(comp.name.empty());
  EXPECT_EQ(comp.type, "Component");
  EXPECT_EQ(comp.source, "runtime");
}

TEST_F(HostInfoProviderTest, component_has_os_metadata) {
  const auto & comp = provider_.get_default_component();
  json j = comp.to_json();

  // Must have x-medkit.host with hostname, os, arch
  ASSERT_TRUE(j.contains("x-medkit"));
  ASSERT_TRUE(j["x-medkit"].contains("host"));

  const auto & host = j["x-medkit"]["host"];
  EXPECT_TRUE(host.contains("hostname"));
  EXPECT_TRUE(host.contains("os"));
  EXPECT_TRUE(host.contains("arch"));

  // Values should match provider accessors
  EXPECT_EQ(host["hostname"].get<std::string>(), provider_.hostname());
  EXPECT_EQ(host["os"].get<std::string>(), provider_.os());
  EXPECT_EQ(host["arch"].get<std::string>(), provider_.arch());
}

// @verifies REQ_INTEROP_003
TEST_F(HostInfoProviderTest, sanitizes_hostname_to_valid_entity_id) {
  const auto & comp = provider_.get_default_component();
  const std::string & id = comp.id;

  // All chars must be alphanumeric, underscore, or hyphen
  for (char c : id) {
    EXPECT_TRUE(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')
        << "Invalid character '" << c << "' in entity ID: " << id;
  }

  // ID must be lowercase
  for (char c : id) {
    if (std::isalpha(static_cast<unsigned char>(c))) {
      EXPECT_TRUE(std::islower(static_cast<unsigned char>(c)))
          << "Uppercase character '" << c << "' in entity ID: " << id;
    }
  }

  // ID must not exceed 256 characters
  EXPECT_LE(id.size(), 256u);
}

// =============================================================================
// sanitize_entity_id() Tests
// =============================================================================

TEST(SanitizeEntityIdTest, converts_dots_to_underscores) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id("my.host.name"), "my_host_name");
}

TEST(SanitizeEntityIdTest, converts_spaces_to_underscores) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id("my host"), "my_host");
}

TEST(SanitizeEntityIdTest, converts_to_lowercase) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id("MyHost"), "myhost");
}

TEST(SanitizeEntityIdTest, preserves_hyphens) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id("my-host"), "my-host");
}

TEST(SanitizeEntityIdTest, strips_invalid_characters) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id("my@host!name#1"), "myhostname1");
}

TEST(SanitizeEntityIdTest, handles_mixed_input) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id("Dev.Machine 01-A!"), "dev_machine_01-a");
}

TEST(SanitizeEntityIdTest, truncates_to_256_chars) {
  std::string long_input(300, 'a');
  std::string result = HostInfoProvider::sanitize_entity_id(long_input);
  EXPECT_EQ(result.size(), 256u);
}

TEST(SanitizeEntityIdTest, handles_empty_input) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id(""), "");
}

TEST(SanitizeEntityIdTest, handles_all_invalid_chars) {
  EXPECT_EQ(HostInfoProvider::sanitize_entity_id("@#$%^&*()"), "");
}

// =============================================================================
// Host info accessor tests
// =============================================================================

TEST_F(HostInfoProviderTest, hostname_is_not_empty) {
  EXPECT_FALSE(provider_.hostname().empty());
}

TEST_F(HostInfoProviderTest, os_is_not_empty) {
  EXPECT_FALSE(provider_.os().empty());
}

TEST_F(HostInfoProviderTest, arch_is_not_empty) {
  EXPECT_FALSE(provider_.arch().empty());
}

TEST_F(HostInfoProviderTest, description_contains_os_and_arch) {
  const auto & comp = provider_.get_default_component();
  // Description should be "OS on arch"
  EXPECT_NE(comp.description.find(provider_.os()), std::string::npos);
  EXPECT_NE(comp.description.find(provider_.arch()), std::string::npos);
  EXPECT_NE(comp.description.find(" on "), std::string::npos);
}

// =============================================================================
// Main
// =============================================================================

int main(int argc, char ** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
