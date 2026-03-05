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

#include <gtest/gtest.h>

#include <thread>
#include <vector>

#include "ros2_medkit_gateway/resource_sampler.hpp"

using namespace ros2_medkit_gateway;

TEST(ResourceSamplerRegistryTest, RegisterAndLookupBuiltinSampler) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"test", true}};
      },
      true);

  EXPECT_TRUE(registry.has_sampler("data"));
  auto sampler = registry.get_sampler("data");
  ASSERT_TRUE(sampler.has_value());
  auto result = (*sampler)("entity1", "/topic");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["test"], true);
}

TEST(ResourceSamplerRegistryTest, RegisterAndLookupVendorExtensionSampler) {
  ResourceSamplerRegistry registry;
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{{"cpu", 42}};
                            });

  EXPECT_TRUE(registry.has_sampler("x-medkit-metrics"));
}

TEST(ResourceSamplerRegistryTest, HasSamplerReturnsFalseForUnregistered) {
  ResourceSamplerRegistry registry;
  EXPECT_FALSE(registry.has_sampler("nonexistent"));
  EXPECT_FALSE(registry.get_sampler("nonexistent").has_value());
}

TEST(ResourceSamplerRegistryTest, BuiltinOverwritesExisting) {
  ResourceSamplerRegistry registry;
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"version", 1}};
      },
      true);
  registry.register_sampler(
      "data",
      [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
        return nlohmann::json{{"version", 2}};
      },
      true);

  auto sampler = registry.get_sampler("data");
  ASSERT_TRUE(sampler.has_value());
  auto result = (*sampler)("e", "");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["version"], 2);
}

TEST(ResourceSamplerRegistryTest, RejectDuplicatePluginRegistration) {
  ResourceSamplerRegistry registry;
  registry.register_sampler("x-medkit-metrics",
                            [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                              return nlohmann::json{};
                            });

  EXPECT_THROW(registry.register_sampler(
                   "x-medkit-metrics",
                   [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                     return nlohmann::json{};
                   }),
               std::runtime_error);
}

TEST(ResourceSamplerRegistryTest, RejectPluginRegistrationWithoutXPrefix) {
  ResourceSamplerRegistry registry;
  EXPECT_THROW(registry.register_sampler(
                   "data",
                   [](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
                     return nlohmann::json{};
                   }),
               std::runtime_error);
}

TEST(ResourceSamplerRegistryTest, ConcurrentRegisterAndLookup) {
  ResourceSamplerRegistry registry;
  std::vector<std::thread> threads;

  for (int i = 0; i < 10; ++i) {
    threads.emplace_back([&registry, i]() {
      registry.register_sampler(
          "collection_" + std::to_string(i),
          [i](const std::string &, const std::string &) -> tl::expected<nlohmann::json, std::string> {
            return nlohmann::json{{"i", i}};
          },
          true);
    });
  }
  for (auto & t : threads) {
    t.join();
  }

  for (int i = 0; i < 10; ++i) {
    EXPECT_TRUE(registry.has_sampler("collection_" + std::to_string(i)));
  }
}
