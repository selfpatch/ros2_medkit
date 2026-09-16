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

// The plugin clear path, driven through FaultHandlers with a real GatewayNode.
//
// The other fault-handler suites test pure helpers. These need the handler
// itself, because what they pin is which value the handler hands the provider:
// a record is (fault_code, owner), the owner is what the gateway resolved in the
// entity's fault scope, and for a COMPONENT that owner is the hosted APP, not
// the component in the URL. Handing the entity id down instead addresses a
// record nobody owns, the fault manager declines it, and the route still
// answers 2xx with the record untouched.
//
// The fault manager is a stub service on a second node, the same shape
// test_fault_manager.cpp uses; the plugin is a mock FaultProvider added to the
// node's own PluginManager.

#include <gtest/gtest.h>

#include <httplib.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/list_faults.hpp"

using json = nlohmann::json;
using namespace std::chrono_literals;
using ros2_medkit_gateway::App;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::FaultProvider;
using ros2_medkit_gateway::FaultProviderErrorInfo;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::GatewayPlugin;
using ros2_medkit_gateway::PluginManager;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::dto::FaultClearResult;
using ros2_medkit_gateway::dto::FaultDetailResult;
using ros2_medkit_gateway::dto::FaultListResult;
using ros2_medkit_gateway::handlers::FaultHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_msgs::srv::ListFaults;

namespace {

constexpr const char * kCode = "SHARED_CODE";
constexpr const char * kComponent = "device_hub";
constexpr const char * kHostedApp = "tank";
constexpr const char * kOtherApp = "pump";

/// Records the (entity_id, fault_code, owner) triple of every clear it is asked
/// for, which is the whole point of the fixture.
class RecordingFaultPlugin : public GatewayPlugin, public FaultProvider {
 public:
  struct ClearCall {
    std::string entity_id;
    std::string fault_code;
    std::string owner;
  };

  std::string name() const override {
    return "recording_fault_plugin";
  }
  void configure(const json & /*config*/) override {
  }
  void shutdown() override {
  }

  tl::expected<FaultListResult, FaultProviderErrorInfo> list_faults(const std::string & /*entity_id*/) override {
    return FaultListResult{json{{"items", listed_items_}}};
  }
  tl::expected<FaultDetailResult, FaultProviderErrorInfo> get_fault(const std::string & /*entity_id*/,
                                                                    const std::string & code) override {
    return FaultDetailResult{json{{"code", code}}};
  }
  tl::expected<FaultClearResult, FaultProviderErrorInfo>
  clear_fault(const std::string & entity_id, const std::string & code, const std::string & owner) override {
    clears.push_back(ClearCall{entity_id, code, owner});
    return FaultClearResult{json{{"code", code}, {"cleared", true}}};
  }

  std::vector<ClearCall> clears;
  json listed_items_ = json::array();
};

/// A TypedRequest carrying the two positional captures the fault routes read.
/// The smatch holds iterators into `path`, so both outlive the request.
class RoutedRequest {
 public:
  RoutedRequest(std::string path, const std::string & pattern) : path_(std::move(path)) {
    std::regex re(pattern);
    matched_ = std::regex_match(path_, raw_.matches, re);
    raw_.path = path_;
    raw_.method = "DELETE";
  }

  bool matched() const {
    return matched_;
  }
  const httplib::Request & raw() const {
    return raw_;
  }

 private:
  std::string path_;
  httplib::Request raw_;
  bool matched_ = false;
};

}  // namespace

class PluginClearOwnerTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    const int test_id = test_counter_++;
    ns_ = "pco" + std::to_string(test_id);

    auto options = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(false).parameter_overrides({
        {"server.port", 0},
        {"fault_manager.namespace", ns_},
        {"fault_manager.service_timeout_sec", 5.0},
    });
    node_ = std::make_shared<GatewayNode>(options);
    // The injected cache is the single source of truth for these tests; the
    // graph-event refresh would reconcile it back to the live ROS graph.
    node_->stop_discovery_refresh_for_testing();

    store_ = std::make_shared<rclcpp::Node>("pco_store_" + std::to_string(test_id));
    list_srv_ = store_->create_service<ListFaults>(
        "/" + ns_ + "/fault_manager/list_faults",
        [this](const std::shared_ptr<ListFaults::Request> & req, const std::shared_ptr<ListFaults::Response> & res) {
          last_include_muted_.store(req->include_muted);
          res->faults = stored_faults_;
          // A muted record is one the correlation engine hid from the default
          // listing, so the store only answers with it when it is asked for.
          if (req->include_muted) {
            for (const auto & muted : muted_faults_) {
              res->faults.push_back(muted);
            }
          }
        });
    clear_srv_ = store_->create_service<ros2_medkit_msgs::srv::ClearFault>(
        "/" + ns_ + "/fault_manager/clear_fault",
        [this](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & req,
               const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & res) {
          std::lock_guard<std::mutex> lock(cleared_mutex_);
          cleared_.push_back({req->fault_code, req->source_id});
          res->success = true;
          res->message = "cleared";
        });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(store_);
    spin_ = std::thread([this] {
      executor_->spin();
    });

    ctx_ = std::make_unique<HandlerContext>(node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<FaultHandlers>(*ctx_);
  }

  void TearDown() override {
    if (spin_.joinable()) {
      executor_->cancel();
      spin_.join();
    }
    executor_.reset();
    store_.reset();
    handlers_.reset();
    ctx_.reset();
    node_.reset();
  }

  /// One stored record, owned by `owner`.
  void store_record(const std::string & code, const std::string & owner) {
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = code;
    fault.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
    fault.reporting_sources = {owner};
    stored_faults_.push_back(fault);
  }

  /// A component hosting two external apps, all three owned by the plugin, so
  /// the fault routes take the plugin branch on every one of them.
  RecordingFaultPlugin * seed_topology() {
    App a;
    a.id = kHostedApp;
    a.component_id = kComponent;
    a.external = true;
    App b;
    b.id = kOtherApp;
    b.component_id = kComponent;
    b.external = true;
    Component c;
    c.id = kComponent;
    c.external = true;
    auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
    cache.update_all({}, {c}, {a, b}, {});

    auto plugin = std::make_unique<RecordingFaultPlugin>();
    auto * raw = plugin.get();
    auto * pmgr = node_->get_plugin_manager();
    pmgr->add_plugin(std::move(plugin));
    pmgr->register_entity_ownership("recording_fault_plugin", {kComponent, kHostedApp, kOtherApp});
    return raw;
  }

  bool wait_for_store(std::chrono::milliseconds timeout = 5s) {
    auto probe = store_->create_client<ListFaults>("/" + ns_ + "/fault_manager/list_faults");
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (probe->service_is_ready()) {
        return true;
      }
      std::this_thread::sleep_for(20ms);
    }
    return false;
  }

  static inline int test_counter_ = 0;
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::string ns_;
  std::shared_ptr<GatewayNode> node_;
  std::shared_ptr<rclcpp::Node> store_;
  rclcpp::Service<ListFaults>::SharedPtr list_srv_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_;
  /// One muted record, owned by `owner`. Served only to a listing that asks.
  void store_muted_record(const std::string & code, const std::string & owner) {
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = code;
    fault.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
    fault.reporting_sources = {owner};
    muted_faults_.push_back(fault);
  }

  /// The same component and apps, owned by NOBODY, so the fault routes take the
  /// native fault-manager path and resolve through resolve_scoped_fault.
  void seed_native_topology() {
    App a;
    a.id = kHostedApp;
    a.component_id = kComponent;
    a.external = true;
    Component c;
    c.id = kComponent;
    c.external = true;
    auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
    cache.update_all({}, {c}, {a}, {});
  }

  std::vector<std::pair<std::string, std::string>> cleared_snapshot() {
    std::lock_guard<std::mutex> lock(cleared_mutex_);
    return cleared_;
  }

  std::vector<ros2_medkit_msgs::msg::Fault> stored_faults_;
  std::vector<ros2_medkit_msgs::msg::Fault> muted_faults_;
  std::atomic<bool> last_include_muted_{false};
  rclcpp::Service<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_srv_;
  std::mutex cleared_mutex_;
  std::vector<std::pair<std::string, std::string>> cleared_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<FaultHandlers> handlers_;
};

// A component owns the records its hosted apps reported. The clear the plugin
// receives has to name that app, because that is the record the gateway
// resolved. Handing it the component id addresses a record no source owns.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, ComponentClearHandsTheHostedAppsOwnerToThePlugin) {
  auto * plugin = seed_topology();
  store_record(kCode, kHostedApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/components/") + kComponent + "/faults/" + kCode,
                    R"(/api/v1/components/([^/]+)/faults/([^/]+))");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  ASSERT_EQ(plugin->clears.size(), 1u);
  EXPECT_EQ(plugin->clears[0].entity_id, kComponent);
  EXPECT_EQ(plugin->clears[0].fault_code, kCode);
  EXPECT_EQ(plugin->clears[0].owner, kHostedApp) << "the clear must name the record's owner, not the entity in the URL";
}

// The app route resolves to its own record, so owner and entity coincide there.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, AppClearHandsItsOwnOwnerToThePlugin) {
  auto * plugin = seed_topology();
  store_record(kCode, kHostedApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/apps/") + kHostedApp + "/faults/" + kCode,
                    R"(/api/v1/apps/([^/]+)/faults/([^/]+))");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  ASSERT_EQ(plugin->clears.size(), 1u);
  EXPECT_EQ(plugin->clears[0].owner, kHostedApp);
}

// Two hosted apps report the code, so the component's URL names neither record
// and the plugin must not be asked to clear anything.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, AmbiguousComponentClearReachesNoProvider) {
  auto * plugin = seed_topology();
  store_record(kCode, kHostedApp);
  store_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/components/") + kComponent + "/faults/" + kCode,
                    R"(/api/v1/components/([^/]+)/faults/([^/]+))");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 409);
  EXPECT_TRUE(plugin->clears.empty()) << "an ambiguous address must not reach the provider";
}

// The bulk clear walks the plugin's own listing, and each item names its own
// record. Sending the entity id for all of them clears one owner's record at
// most and silently misses the rest.
// @verifies REQ_INTEROP_014
TEST_F(PluginClearOwnerTest, ComponentBulkClearSendsEachRecordsOwnOwner) {
  auto * plugin = seed_topology();
  plugin->listed_items_ = json::array({
      json{{"code", kCode}, {"source_id", kHostedApp}},
      json{{"code", kCode}, {"source_id", kOtherApp}},
  });
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/components/") + kComponent + "/faults",
                    R"(/api/v1/components/([^/]+)/faults)");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_all_faults(typed);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  ASSERT_EQ(plugin->clears.size(), 2u);
  EXPECT_EQ(plugin->clears[0].owner, kHostedApp);
  EXPECT_EQ(plugin->clears[1].owner, kOtherApp) << "each item's own source_id must travel with its clear";
}

// The correlation engine mutes a symptom to keep it out of the default listing.
// That is a display decision, not an unaddressing: the record still exists and
// its entity could read and clear it before. resolve_scoped_fault resolving out
// of a listing that excludes muted records 404s it instead.
//
// The topology here is deliberately NOT plugin-owned, so the route takes the
// native fault-manager path and resolve_scoped_fault is the function under test.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, ResolutionKeepsAMutedRecordAddressable) {
  seed_native_topology();
  store_muted_record(kCode, kHostedApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/apps/") + kHostedApp + "/faults/" + kCode,
                    R"(/api/v1/apps/([^/]+)/faults/([^/]+))");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << "a muted record must stay addressable: " << result.error().message;
  const auto cleared = cleared_snapshot();
  ASSERT_EQ(cleared.size(), 1u) << "the muted record was never cleared";
  EXPECT_EQ(cleared[0].first, kCode);
  EXPECT_EQ(cleared[0].second, kHostedApp) << "the clear must carry the muted record's owner";
}

// The same route on an unmuted record, so the muted case above is not simply
// passing because the harness serves everything regardless of the flag.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, ResolutionClearsAnUnmutedRecordWithItsOwner) {
  seed_native_topology();
  store_record(kCode, kHostedApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/apps/") + kHostedApp + "/faults/" + kCode,
                    R"(/api/v1/apps/([^/]+)/faults/([^/]+))");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  const auto cleared = cleared_snapshot();
  ASSERT_EQ(cleared.size(), 1u);
  EXPECT_EQ(cleared[0].second, kHostedApp);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
