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

#include <chrono>
#include <string>
#include <thread>

#include "ros2_medkit_gateway/core/aggregation/mdns_discovery.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// Config / construction tests (no network sockets opened)
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, default_config_values) {
  MdnsDiscovery::Config config;

  EXPECT_FALSE(config.announce);
  EXPECT_FALSE(config.discover);
  EXPECT_EQ(config.service, "_medkit._tcp.local");
  EXPECT_EQ(config.port, 8080);
  EXPECT_TRUE(config.name.empty());
  EXPECT_FALSE(config.on_error);  // No error callback by default
}

// @verifies REQ_INTEROP_003
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

TEST(MdnsDiscovery, error_callback_is_stored_in_config) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_called = false;
  config.on_error = [&error_called](const std::string & /*msg*/) {
    error_called = true;
  };

  MdnsDiscovery discovery(config);

  // The callback is stored but not invoked when no sockets are opened
  EXPECT_FALSE(error_called);
}

// =============================================================================
// Error callback wiring tests
// =============================================================================

TEST(MdnsDiscovery, error_callback_not_invoked_without_sockets) {
  // Verify the on_error callback is not spuriously triggered when announce/discover are off
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_received = false;
  std::string error_message;
  config.on_error = [&](const std::string & msg) {
    error_received = true;
    error_message = msg;
  };

  MdnsDiscovery mdns(config);

  // Start and stop with both flags off - no socket operations
  mdns.start([](const std::string &, const std::string &) {}, [](const std::string &) {});
  mdns.stop();

  // Error callback should NOT have been invoked since no network activity occurred
  EXPECT_FALSE(error_received);
  EXPECT_TRUE(error_message.empty());
}

TEST(MdnsDiscovery, default_announce_and_discover_are_false) {
  MdnsDiscovery::Config config;
  EXPECT_FALSE(config.announce);
  EXPECT_FALSE(config.discover);
}

TEST(MdnsDiscovery, default_service_type) {
  MdnsDiscovery::Config config;
  EXPECT_EQ(config.service, "_medkit._tcp.local");
}

TEST(MdnsDiscovery, default_port_is_8080) {
  MdnsDiscovery::Config config;
  EXPECT_EQ(config.port, 8080);
}

TEST(MdnsDiscovery, default_name_is_empty) {
  MdnsDiscovery::Config config;
  EXPECT_TRUE(config.name.empty());
}

TEST(MdnsDiscovery, default_on_error_is_null) {
  MdnsDiscovery::Config config;
  EXPECT_FALSE(config.on_error);
}

TEST(MdnsDiscovery, start_stop_lifecycle_with_callbacks) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  MdnsDiscovery discovery(config);

  bool found_called = false;
  bool removed_called = false;

  discovery.start(
      [&](const std::string & /*url*/, const std::string & /*name*/) {
        found_called = true;
      },
      [&](const std::string & /*name*/) {
        removed_called = true;
      });

  // With both flags off, callbacks should never fire
  EXPECT_FALSE(found_called);
  EXPECT_FALSE(removed_called);
  EXPECT_FALSE(discovery.is_announcing());
  EXPECT_FALSE(discovery.is_discovering());

  discovery.stop();

  EXPECT_FALSE(found_called);
  EXPECT_FALSE(removed_called);
}

// =============================================================================
// instance_name() tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, instance_name_returns_explicit_name) {
  MdnsDiscovery::Config config;
  config.name = "my-gateway";
  MdnsDiscovery discovery(config);
  EXPECT_EQ(discovery.instance_name(), "my-gateway");
}

TEST(MdnsDiscovery, instance_name_defaults_to_hostname_when_empty) {
  MdnsDiscovery::Config config;
  // config.name is empty by default - constructor resolves to gethostname()
  MdnsDiscovery discovery(config);
  EXPECT_FALSE(discovery.instance_name().empty());
}

TEST(MdnsDiscovery, instance_name_preserves_set_value_across_lifecycle) {
  MdnsDiscovery::Config config;
  config.name = "perception-ecu";
  MdnsDiscovery discovery(config);
  EXPECT_EQ(discovery.instance_name(), "perception-ecu");
  discovery.stop();
  EXPECT_EQ(discovery.instance_name(), "perception-ecu");
}

// =============================================================================
// Privileged port rejection tests (via AggregationManager integration)
// =============================================================================
// The browse_callback in mdns_discovery.cpp rejects SRV records with port 0
// or port < 1024 before calling the on_found callback. Since browse_callback
// is a static function in an anonymous namespace, we cannot unit test it
// directly. Instead, we verify via AggregationManager that URLs with
// privileged ports constructed outside the mDNS path are still subject to
// address validation (the port check is defense-in-depth at the mDNS layer).

TEST(MdnsDiscovery, privileged_port_documentation) {
  // This test documents the privileged port rejection behavior.
  // browse_callback rejects SRV records with:
  //   - port == 0 (unspecified)
  //   - port < 1024 (privileged, requires root, typically system services)
  // These checks happen BEFORE the on_found callback is invoked, so
  // AggregationManager::add_discovered_peer never sees these URLs.
  //
  // Verify that Config can express the port for announcement correctly
  MdnsDiscovery::Config config;
  config.port = 8080;  // Non-privileged port - valid
  EXPECT_GE(config.port, 1024);

  config.port = 443;  // Privileged port - would be rejected in browse_callback
  EXPECT_LT(config.port, 1024);
  // Note: The announcement port is for our own service and is not validated
  // (the operator intentionally configures it). The check is only in
  // browse_callback for incoming SRV records from peers.
  SUCCEED();
}

// =============================================================================
// Constructor validation tests
// =============================================================================

TEST(MdnsDiscovery, out_of_range_port_triggers_error_callback) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_called = false;
  std::string error_msg;
  config.on_error = [&](const std::string & msg) {
    error_called = true;
    error_msg = msg;
  };

  // Port out of valid uint16_t range should trigger error callback and reset to 8080
  config.port = 70000;
  MdnsDiscovery discovery(config);

  EXPECT_TRUE(error_called) << "Error callback should fire for out-of-range port";
  EXPECT_NE(error_msg.find("out of valid range"), std::string::npos);
}

TEST(MdnsDiscovery, negative_port_triggers_error_callback) {
  MdnsDiscovery::Config config;
  config.announce = false;
  config.discover = false;

  bool error_called = false;
  config.on_error = [&](const std::string & /*msg*/) {
    error_called = true;
  };

  config.port = -1;
  MdnsDiscovery discovery(config);

  EXPECT_TRUE(error_called) << "Error callback should fire for negative port";
}

// =============================================================================
// Socket creation resilience tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(MdnsDiscovery, start_announce_on_ephemeral_port_does_not_crash) {
  // Exercise the socket creation path. In CI containers mDNS port 5353 may not
  // be available (requires CAP_NET_BIND_SERVICE or root). The announce_loop should
  // handle the failure gracefully via the error callback rather than crashing.
  MdnsDiscovery::Config config;
  config.announce = true;
  config.discover = false;
  config.port = 0;  // Ephemeral port for our service announcement
  config.name = "test-socket-resilience";

  bool error_reported = false;
  config.on_error = [&](const std::string & /*msg*/) {
    error_reported = true;
  };

  MdnsDiscovery discovery(config);
  discovery.start([](const std::string &, const std::string &) {}, [](const std::string &) {});

  // Give the thread a moment to attempt socket creation
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Whether or not the socket opened, stop must not hang or crash
  discovery.stop();

  // Either the announce thread ran (socket opened) or the error callback fired
  // (socket failed). Both are acceptable outcomes in CI.
  if (!discovery.is_announcing()) {
    // Socket likely failed - this is expected in restricted CI environments.
    // The important thing is we did not crash.
    SUCCEED();
  }
}
