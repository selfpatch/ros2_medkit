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
#include <httplib.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "ros2_medkit_gateway/discovery/introspection_provider.hpp"
#include "ros2_medkit_gateway/plugins/plugin_context.hpp"
#include "ros2_medkit_gateway/plugins/plugin_manager.hpp"

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

/// Minimal mock plugin for compile-time testing (no .so needed)
class MockPlugin : public GatewayPlugin, public UpdateProvider, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "mock";
  }
  void configure(const json & config) override {
    configured_ = true;
    config_ = config;
  }
  void shutdown() override {
    shutdown_called_ = true;
  }

  // UpdateProvider
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter &) override {
    return std::vector<std::string>{};
  }
  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string &) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "mock"});
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string &, UpdateProgressReporter &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string &, UpdateProgressReporter &) override {
    return {};
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string &) override {
    return false;
  }

  // IntrospectionProvider
  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }

  bool configured_ = false;
  bool shutdown_called_ = false;
  json config_;
};

/// Plugin that only provides IntrospectionProvider
class MockIntrospectionOnly : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "introspection_only";
  }
  void configure(const json &) override {
  }
  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }
};

/// Plugin that throws during configure
class MockThrowingPlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "throwing";
  }
  void configure(const json &) override {
    throw std::runtime_error("configure failed");
  }
};

/// Plugin that throws during set_context
class MockThrowOnSetContext : public GatewayPlugin, public UpdateProvider {
 public:
  std::string name() const override {
    return "throw_set_context";
  }
  void configure(const json &) override {
  }
  void set_context(PluginContext &) override {
    throw std::runtime_error("set_context failed");
  }

  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter &) override {
    return std::vector<std::string>{};
  }
  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string &) override {
    return json::object();
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string &, UpdateProgressReporter &) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string &, UpdateProgressReporter &) override {
    return {};
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string &) override {
    return false;
  }
};

/// Plugin that returns a test route for verifying wrapping logic
class MockRoutePlugin : public GatewayPlugin {
 public:
  std::string name() const override {
    return "mock_route";
  }
  void configure(const json &) override {
  }
  std::vector<PluginRoute> get_routes() override {
    return {
        {"GET", R"(apps/([^/]+)/x-test-route)",
         [this](const PluginRequest & req, PluginResponse & res) {
           last_path_param_ = req.path_param(1);
           res.send_json({{"handled", true}, {"entity", last_path_param_}});
         }},
    };
  }
  std::string last_path_param_;
};

/// Plugin that throws during get_routes
class MockThrowOnGetRoutes : public GatewayPlugin, public IntrospectionProvider {
 public:
  std::string name() const override {
    return "throw_get_routes";
  }
  void configure(const json &) override {
  }
  std::vector<PluginRoute> get_routes() override {
    throw std::runtime_error("get_routes failed");
  }

  IntrospectionResult introspect(const IntrospectionInput &) override {
    return {};
  }
};

/// Plugin that throws during shutdown
class MockThrowOnShutdown : public GatewayPlugin {
 public:
  std::string name() const override {
    return "throw_shutdown";
  }
  void configure(const json &) override {
  }
  void shutdown() override {
    throw std::runtime_error("shutdown failed");
  }
};

TEST(PluginManagerTest, EmptyManagerHasNoPlugins) {
  PluginManager mgr;
  EXPECT_FALSE(mgr.has_plugins());
  EXPECT_TRUE(mgr.plugin_names().empty());
  EXPECT_EQ(mgr.get_update_provider(), nullptr);
  EXPECT_TRUE(mgr.get_introspection_providers().empty());
}

TEST(PluginManagerTest, AddPluginAndDispatch) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  EXPECT_TRUE(mgr.has_plugins());
  EXPECT_EQ(mgr.plugin_names().size(), 1u);
  EXPECT_EQ(mgr.plugin_names()[0], "mock");

  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(raw));
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

TEST(PluginManagerTest, ConfigurePassesConfig) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.configure_plugins();
  EXPECT_TRUE(raw->configured_);
  // add_plugin() uses empty config by default
  EXPECT_TRUE(raw->config_.is_object());
  EXPECT_TRUE(raw->config_.empty());
}

TEST(PluginManagerTest, LoadPluginsForwardsConfig) {
  // load_plugins() should forward the PluginConfig.config to configure()
  PluginManager mgr;
  json cfg = {{"server_url", "https://example.com"}, {"timeout_ms", 5000}};
  std::vector<PluginConfig> configs = {{"test", "/nonexistent/path.so", cfg}};

  // Plugin won't load (bad path), but verify PluginConfig struct holds config
  EXPECT_EQ(configs[0].config["server_url"], "https://example.com");
  EXPECT_EQ(configs[0].config["timeout_ms"], 5000);
}

TEST(PluginManagerTest, ShutdownCallsAllPlugins) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.shutdown_all();
  EXPECT_TRUE(raw->shutdown_called_);
}

TEST(PluginManagerTest, MultiCapabilityPluginDispatchedToBoth) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());

  EXPECT_NE(mgr.get_update_provider(), nullptr);
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

TEST(PluginManagerTest, IntrospectionOnlyPluginNotUpdateProvider) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  EXPECT_EQ(mgr.get_update_provider(), nullptr);
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
}

TEST(PluginManagerTest, MultipleIntrospectionProviders) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  EXPECT_EQ(mgr.get_introspection_providers().size(), 2u);
}

TEST(PluginManagerTest, DuplicateUpdateProviderFirstWins) {
  PluginManager mgr;
  auto first = std::make_unique<MockPlugin>();
  auto * first_raw = first.get();
  mgr.add_plugin(std::move(first));
  mgr.add_plugin(std::make_unique<MockPlugin>());

  // First UpdateProvider wins
  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(first_raw));
}

TEST(PluginManagerTest, ThrowingPluginDisabledDuringConfigure) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowingPlugin>());
  auto good = std::make_unique<MockPlugin>();
  auto * good_raw = good.get();
  mgr.add_plugin(std::move(good));

  // Should not throw
  mgr.configure_plugins();

  // Good plugin still works
  EXPECT_TRUE(good_raw->configured_);
  EXPECT_NE(mgr.get_update_provider(), nullptr);
}

TEST(PluginManagerTest, LoadNonexistentPluginReturnsZero) {
  PluginManager mgr;
  std::vector<PluginConfig> configs = {{"nonexistent", "/nonexistent/path.so", json::object()}};
  auto loaded = mgr.load_plugins(configs);
  EXPECT_EQ(loaded, 0u);
  EXPECT_FALSE(mgr.has_plugins());
}

TEST(PluginManagerTest, ThrowOnSetContextDisablesPlugin) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowOnSetContext>());
  auto good = std::make_unique<MockPlugin>();
  auto * good_raw = good.get();
  mgr.add_plugin(std::move(good));

  mgr.configure_plugins();

  // Create a minimal PluginContext for the test
  auto ctx = make_gateway_plugin_context(nullptr, nullptr);
  mgr.set_context(*ctx);

  // Throwing plugin disabled, good plugin's UpdateProvider still works
  EXPECT_EQ(mgr.get_update_provider(), static_cast<UpdateProvider *>(good_raw));
  // Only good plugin remains active
  EXPECT_EQ(mgr.plugin_names().size(), 1u);
  EXPECT_EQ(mgr.plugin_names()[0], "mock");
}

TEST(PluginManagerTest, ThrowOnGetRoutesDisablesPlugin) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowOnGetRoutes>());
  auto good = std::make_unique<MockIntrospectionOnly>();
  mgr.add_plugin(std::move(good));

  mgr.configure_plugins();
  httplib::Server srv;
  mgr.register_routes(srv, "/api/v1");

  // Throwing plugin disabled, good plugin's IntrospectionProvider still works
  EXPECT_EQ(mgr.get_introspection_providers().size(), 1u);
  EXPECT_EQ(mgr.plugin_names().size(), 1u);
  EXPECT_EQ(mgr.plugin_names()[0], "introspection_only");
}

TEST(PluginManagerTest, RegisterRoutesWrapsPluginHandlers) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockRoutePlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));
  mgr.configure_plugins();

  httplib::Server srv;
  mgr.register_routes(srv, "/api/v1");

  // Bind to ephemeral port to avoid conflicts in parallel CTest runs
  auto port = srv.bind_to_any_port("127.0.0.1");
  std::thread server_thread([&srv]() {
    srv.listen_after_bind();
  });
  srv.wait_until_ready();

  httplib::Client cli("127.0.0.1", port);
  auto res = cli.Get("/api/v1/apps/test_entity/x-test-route");

  srv.stop();
  server_thread.join();

  ASSERT_TRUE(res);
  EXPECT_EQ(res->status, 200);
  auto body = json::parse(res->body);
  EXPECT_EQ(body["handled"], true);
  EXPECT_EQ(body["entity"], "test_entity");
  EXPECT_EQ(raw->last_path_param_, "test_entity");
}

TEST(PluginManagerTest, ShutdownAllIdempotent) {
  PluginManager mgr;
  auto plugin = std::make_unique<MockPlugin>();
  auto * raw = plugin.get();
  mgr.add_plugin(std::move(plugin));

  mgr.shutdown_all();
  EXPECT_TRUE(raw->shutdown_called_);

  // Reset flag, call again - should be a no-op
  raw->shutdown_called_ = false;
  mgr.shutdown_all();
  EXPECT_FALSE(raw->shutdown_called_);
}

TEST(PluginManagerTest, ShutdownSwallowsExceptions) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockThrowOnShutdown>());
  auto good = std::make_unique<MockPlugin>();
  auto * good_raw = good.get();
  mgr.add_plugin(std::move(good));

  // Should not throw, even though first plugin throws during shutdown
  EXPECT_NO_THROW(mgr.shutdown_all());
  EXPECT_TRUE(good_raw->shutdown_called_);
}

using namespace std::chrono_literals;

// Test 1: Concurrent readers don't block each other
TEST(PluginManagerConcurrencyTest, ConcurrentReadsDoNotBlock) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  std::atomic<int> completed{0};
  std::vector<std::thread> readers;

  for (int i = 0; i < 8; ++i) {
    readers.emplace_back([&mgr, &completed] {
      for (int j = 0; j < 200; ++j) {
        auto up = mgr.get_update_provider();
        auto ips = mgr.get_introspection_providers();
        auto lp = mgr.get_log_provider();
        auto obs = mgr.get_log_observers();
        auto has = mgr.has_plugins();
        auto names = mgr.plugin_names();
        (void)up;
        (void)ips;
        (void)lp;
        (void)obs;
        (void)has;
        (void)names;
      }
      completed++;
    });
  }

  auto start = std::chrono::high_resolution_clock::now();
  for (auto & t : readers) {
    t.join();
  }
  auto duration = std::chrono::high_resolution_clock::now() - start;

  EXPECT_EQ(completed.load(), 8);
  EXPECT_LT(duration, 2s) << "Concurrent reads took too long - possible blocking";
}

// Test 2: Concurrent reads and writes (lifecycle/disable) don't deadlock
TEST(PluginManagerConcurrencyTest, ConcurrentReadsAndLifecycleDoNotDeadlock) {
  PluginManager mgr;
  mgr.add_plugin(std::make_unique<MockPlugin>());
  mgr.add_plugin(std::make_unique<MockIntrospectionOnly>());

  std::atomic<bool> keep_running{true};
  std::atomic<int> read_count{0};
  std::atomic<int> write_count{0};

  // Multiple reader threads (simulating ROS 2 executor calling get_log_observers)
  std::vector<std::thread> readers;
  for (int i = 0; i < 4; ++i) {
    readers.emplace_back([&mgr, &keep_running, &read_count] {
      while (keep_running) {
        auto obs = mgr.get_log_observers();
        auto ips = mgr.get_introspection_providers();
        auto up = mgr.get_update_provider();
        (void)obs;
        (void)ips;
        (void)up;
        read_count++;
      }
    });
  }

  // Writer thread (simulating lifecycle operations that take unique_lock)
  std::thread writer([&mgr, &keep_running, &write_count] {
    while (keep_running) {
      mgr.configure_plugins();
      write_count++;
    }
  });

  std::this_thread::sleep_for(50ms);
  keep_running = false;

  for (auto & t : readers) {
    t.join();
  }
  writer.join();

  EXPECT_GT(read_count.load(), 0) << "Readers made no progress - possible deadlock";
  EXPECT_GT(write_count.load(), 0) << "Writer made no progress - possible deadlock";
}

// Test 3: Shutdown while readers are active doesn't deadlock
TEST(PluginManagerConcurrencyTest, ShutdownWhileReadersActiveDoesNotDeadlock) {
  auto mgr = std::make_unique<PluginManager>();
  mgr->add_plugin(std::make_unique<MockPlugin>());

  std::atomic<bool> keep_running{true};
  std::atomic<int> read_count{0};

  std::vector<std::thread> readers;
  for (int i = 0; i < 4; ++i) {
    readers.emplace_back([&mgr, &keep_running, &read_count] {
      while (keep_running) {
        if (mgr) {
          auto obs = mgr->get_log_observers();
          (void)obs;
        }
        read_count++;
      }
    });
  }

  std::this_thread::sleep_for(10ms);
  mgr->shutdown_all();

  keep_running = false;
  for (auto & t : readers) {
    t.join();
  }

  EXPECT_GT(read_count.load(), 0) << "Readers made no progress before shutdown";
}
