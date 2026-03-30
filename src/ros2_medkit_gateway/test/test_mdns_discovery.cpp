// Copyright 2026 Selfpatch GmbH
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

#include <string>

#include "ros2_medkit_gateway/aggregation/mdns_discovery.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// Config / construction tests (no network sockets opened)
// =============================================================================

TEST(MdnsDiscovery, default_config_values) {
  MdnsDiscovery::Config config;

  EXPECT_TRUE(config.announce);
  EXPECT_TRUE(config.discover);
  EXPECT_EQ(config.service, "_medkit._tcp.local");
  EXPECT_EQ(config.port, 8080);
  EXPECT_TRUE(config.name.empty());
}

TEST(MdnsDiscovery, respects_announce_flag) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;  // Don't try to open sockets

  MdnsDiscovery discovery(config);

  // Start with both flags off - no threads should launch
  bool found_called = false;
  bool removed_called = false;

  discovery.start(
      [&](const std::string & /*url*/, const std::string & /*name*/) {
        found_called = true;
      },
      [&](const std::string & /*name*/) {
        removed_called = true;
      });

  // Neither thread should be running since both flags are off
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());

  discovery.stop();

  EXPECT_FALSE(found_called);
  EXPECT_FALSE(removed_called);
}

TEST(MdnsDiscovery, respects_discover_flag) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});

  EXPECT_FALSE(discovery.is_discovering());
  EXPECT_FALSE(discovery.is_announcing());

  discovery.stop();
}

TEST(MdnsDiscovery, stop_is_idempotent) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  // Start then stop multiple times - should not crash or hang
  discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});

  discovery.stop();
  discovery.stop();  // Second stop should be a no-op
  discovery.stop();  // Third stop should also be safe

  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());
}

TEST(MdnsDiscovery, destructor_stops_cleanly) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  {
    MdnsDiscovery discovery(config);
    discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});
    // Destructor should call stop() without hanging
  }

  // If we get here without hanging, the test passes
  SUCCEED();
}

TEST(MdnsDiscovery, custom_config_values) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;
  config.service = "_custom._tcp.local";
  config.port = 9090;
  config.name = "test-gateway";

  MdnsDiscovery discovery(config);

  // Construction with custom values should succeed
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());
}

TEST(MdnsDiscovery, start_without_callbacks_is_safe) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  // Passing nullptr-equivalent callbacks should not crash
  discovery.start(nullptr, nullptr);
  discovery.stop();
}

TEST(MdnsDiscovery, not_running_before_start) {
  MdnsDiscovery::Config config;
  config.announce = true;
  config.discover = true;

  MdnsDiscovery discovery(config);

  // Before start(), nothing should be running
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());
}
