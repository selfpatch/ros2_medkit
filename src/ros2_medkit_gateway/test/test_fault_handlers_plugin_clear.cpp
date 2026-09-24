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

// The per-record routes, driven through the handlers with a real GatewayNode:
// the fault detail and clear (native and plugin provider) and the recording
// download by fault code.
//
// The other fault-handler suites test pure helpers. These need the handler
// itself, because what they pin is which record the handler resolves and which
// owner it hands on: a record is (fault_code, owner), the owner is what the
// gateway resolved in the entity's fault scope, and for a COMPONENT that owner
// is the hosted APP, not the component in the URL. Handing the entity id down
// instead addresses a record nobody owns, the fault manager declines it, and
// the route still answers 2xx with the record untouched.
//
// The fault manager is a stub service on a second node, the same shape
// test_fault_manager.cpp uses. It answers ListFaults the way the real one
// does: a muted record is left out unless the listing asks for muted records,
// and then it is also named in `muted_faults`. The plugin is a mock
// FaultProvider added to the node's own PluginManager.

#include <gtest/gtest.h>

#include <httplib.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <regex>
#include <string>
#include <thread>
#include <utility>
#include <variant>
#include <vector>

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/bulkdata_handlers.hpp"
#include "ros2_medkit_gateway/core/models/thread_safe_entity_cache.hpp"
#include "ros2_medkit_gateway/core/plugins/plugin_manager.hpp"
#include "ros2_medkit_gateway/core/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"
#include "ros2_medkit_gateway/http/handlers/fault_handlers.hpp"
#include "ros2_medkit_gateway/http/handlers/handler_context.hpp"
#include "ros2_medkit_msgs/msg/muted_fault_info.hpp"
#include "ros2_medkit_msgs/srv/clear_fault.hpp"
#include "ros2_medkit_msgs/srv/get_fault.hpp"
#include "ros2_medkit_msgs/srv/get_rosbag.hpp"
#include "ros2_medkit_msgs/srv/list_faults.hpp"
#include "ros2_medkit_msgs/srv/list_rosbags.hpp"

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
using ros2_medkit_gateway::handlers::BulkDataHandlers;
using ros2_medkit_gateway::handlers::FaultHandlers;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_msgs::srv::GetFault;
using ros2_medkit_msgs::srv::GetRosbag;
using ros2_medkit_msgs::srv::ListFaults;
using ros2_medkit_msgs::srv::ListRosbags;

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
  // The gateway clears through clear_fault_record, so a call landing here means
  // it went back to the code-only clear and dropped the owner it resolved.
  tl::expected<FaultClearResult, FaultProviderErrorInfo> clear_fault(const std::string & /*entity_id*/,
                                                                     const std::string & code) override {
    ADD_FAILURE() << "the gateway cleared '" << code << "' by code alone instead of calling clear_fault_record";
    return FaultClearResult{json{{"code", code}, {"cleared", true}}};
  }
  tl::expected<FaultClearResult, FaultProviderErrorInfo>
  clear_fault_record(const std::string & entity_id, const std::string & code, const std::string & owner) override {
    if (fail_if_called) {
      ADD_FAILURE() << "the provider was asked to clear '" << code << "' with owner '" << owner
                    << "', which the gateway never resolved";
    }
    clears.push_back(ClearCall{entity_id, code, owner});
    return FaultClearResult{json{{"code", code}, {"cleared", true}}};
  }

  std::vector<ClearCall> clears;
  json listed_items_ = json::array();
  /// Set by a test in which any clear reaching the provider is itself the defect.
  bool fail_if_called = false;
};

/// A provider written against the two-argument clear contract: it overrides
/// clear_fault(entity_id, fault_code) and nothing else of the clear. It has to
/// keep compiling against the current headers and keep receiving the clear.
class TwoArgumentClearPlugin : public GatewayPlugin, public FaultProvider {
 public:
  struct ClearCall {
    std::string entity_id;
    std::string fault_code;
  };

  std::string name() const override {
    return "two_argument_clear_plugin";
  }
  void configure(const json & /*config*/) override {
  }
  void shutdown() override {
  }

  tl::expected<FaultListResult, FaultProviderErrorInfo> list_faults(const std::string & /*entity_id*/) override {
    return FaultListResult{json{{"items", json::array()}}};
  }
  tl::expected<FaultDetailResult, FaultProviderErrorInfo> get_fault(const std::string & /*entity_id*/,
                                                                    const std::string & code) override {
    return FaultDetailResult{json{{"code", code}}};
  }
  tl::expected<FaultClearResult, FaultProviderErrorInfo> clear_fault(const std::string & entity_id,
                                                                     const std::string & code) override {
    clears.push_back(ClearCall{entity_id, code});
    return FaultClearResult{json{{"code", code}, {"cleared", true}}};
  }

  std::vector<ClearCall> clears;
};

/// A TypedRequest carrying the two positional captures the fault routes read.
/// The smatch holds iterators into `path`, so both outlive the request.
class RoutedRequest {
 public:
  RoutedRequest(std::string path, const std::string & pattern, const std::string & method = "DELETE")
    : path_(std::move(path)) {
    std::regex re(pattern);
    matched_ = std::regex_match(path_, raw_.matches, re);
    raw_.path = path_;
    raw_.method = method;
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
  /// A recording the stub fault manager holds for one record.
  struct Recording {
    std::string id;
    std::string fault_code;
    std::string owner;
    std::string path;
  };

  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    const int test_id = test_counter_++;
    ns_ = "pco" + std::to_string(test_id);
    bag_dir_ = std::filesystem::temp_directory_path() / ("pco_bags_" + std::to_string(::getpid()) + "_" + ns_);
    std::filesystem::create_directories(bag_dir_);

    auto options = rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(false).parameter_overrides({
        {"server.port", 0},
        {"fault_manager.namespace", ns_},
        {"fault_manager.service_timeout_sec", 5.0},
    });
    node_ = std::make_shared<GatewayNode>(options);
    // The injected cache is the single source of truth for these tests. The
    // graph-event refresh would reconcile it back to the live ROS graph.
    node_->stop_discovery_refresh_for_testing();

    store_ = std::make_shared<rclcpp::Node>("pco_store_" + std::to_string(test_id));
    const std::string base = "/" + ns_ + "/fault_manager/";
    list_srv_ = store_->create_service<ListFaults>(
        base + "list_faults",
        [this](const std::shared_ptr<ListFaults::Request> & req, const std::shared_ptr<ListFaults::Response> & res) {
          std::lock_guard<std::mutex> lock(store_mutex_);
          last_include_muted_.store(req->include_muted);
          res->faults = stored_faults_;
          res->muted_count = static_cast<uint32_t>(muted_faults_.size());
          // The real fault manager's contract: a muted record is one the
          // correlation engine hid from the default listing, so it is served
          // only to a listing that asks, and then named in muted_faults by its
          // code and owner.
          if (req->include_muted) {
            for (const auto & muted : muted_faults_) {
              res->faults.push_back(muted);
              ros2_medkit_msgs::msg::MutedFaultInfo info;
              info.fault_code = muted.fault_code;
              info.source_id = muted.reporting_sources.front();
              info.root_cause_code = "ROOT_CAUSE";
              info.rule_id = "rule";
              res->muted_faults.push_back(info);
            }
          }
        });
    get_srv_ =
        store_->create_service<GetFault>(base + "get_fault", [this](const std::shared_ptr<GetFault::Request> & req,
                                                                    const std::shared_ptr<GetFault::Response> & res) {
          std::lock_guard<std::mutex> lock(store_mutex_);
          get_requests_.emplace_back(req->fault_code, req->source_id);
          for (const auto * list : {&stored_faults_, &muted_faults_}) {
            for (const auto & fault : *list) {
              if (fault.fault_code == req->fault_code && fault.reporting_sources.front() == req->source_id) {
                res->success = true;
                res->fault = fault;
                return;
              }
            }
          }
          res->success = false;
          res->error_message = "Fault not found: " + req->fault_code;
        });
    clear_srv_ = store_->create_service<ros2_medkit_msgs::srv::ClearFault>(
        base + "clear_fault", [this](const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Request> & req,
                                     const std::shared_ptr<ros2_medkit_msgs::srv::ClearFault::Response> & res) {
          {
            std::lock_guard<std::mutex> lock(cleared_mutex_);
            cleared_.push_back({req->fault_code, req->source_id});
          }
          // The record the request names moves to CLEARED, so a test can read
          // the outcome off the store rather than off the request log alone.
          std::lock_guard<std::mutex> lock(store_mutex_);
          for (auto * list : {&stored_faults_, &muted_faults_}) {
            for (auto & fault : *list) {
              if (fault.fault_code == req->fault_code && fault.reporting_sources.front() == req->source_id) {
                fault.status = ros2_medkit_msgs::msg::Fault::STATUS_CLEARED;
              }
            }
          }
          res->success = true;
          res->message = "cleared";
        });
    // The fault manager's lookup order: the recording id first, then the fault
    // code scoped to the owner the gateway sent. An unscoped code with more than
    // one owner holding recordings answers nothing.
    rosbag_srv_ = store_->create_service<GetRosbag>(
        base + "get_rosbag",
        [this](const std::shared_ptr<GetRosbag::Request> & req, const std::shared_ptr<GetRosbag::Response> & res) {
          std::lock_guard<std::mutex> lock(store_mutex_);
          rosbag_requests_.push_back(req->source_id);
          const Recording * found = nullptr;
          for (const auto & rec : recordings_) {
            if (rec.id == req->recording_id) {
              found = &rec;
            }
          }
          if (found == nullptr) {
            std::vector<const Recording *> by_code;
            for (const auto & rec : recordings_) {
              if (rec.fault_code == req->fault_code && (req->source_id.empty() || rec.owner == req->source_id)) {
                by_code.push_back(&rec);
              }
            }
            if (by_code.size() == 1) {
              found = by_code.front();
            }
          }
          if (found == nullptr) {
            res->success = false;
            res->error_message = "No recording for " + req->recording_id;
            return;
          }
          res->success = true;
          res->file_path = found->path;
          res->recording_id = found->id;
          res->fault_codes = {found->fault_code};
          res->format = "mcap";
          res->size_bytes = std::filesystem::file_size(found->path);
        });
    list_rosbags_srv_ = store_->create_service<ListRosbags>(
        base + "list_rosbags",
        [this](const std::shared_ptr<ListRosbags::Request> & req, const std::shared_ptr<ListRosbags::Response> & res) {
          std::lock_guard<std::mutex> lock(store_mutex_);
          res->success = true;
          for (const auto & rec : recordings_) {
            if (rec.owner != req->entity_fqn) {
              continue;
            }
            res->fault_codes.push_back(rec.fault_code);
            res->recording_ids.push_back(rec.id);
            res->file_paths.push_back(rec.path);
            res->formats.push_back("mcap");
            res->durations_sec.push_back(1.0);
            res->sizes_bytes.push_back(std::filesystem::file_size(rec.path));
            res->created_at_ns.push_back(1'700'000'000'000'000'000);
          }
        });
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(store_);
    spin_ = std::thread([this] {
      executor_->spin();
    });

    ctx_ = std::make_unique<HandlerContext>(node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<FaultHandlers>(*ctx_);
    bulk_handlers_ = std::make_unique<BulkDataHandlers>(*ctx_);
  }

  void TearDown() override {
    if (spin_.joinable()) {
      executor_->cancel();
      spin_.join();
    }
    executor_.reset();
    store_.reset();
    bulk_handlers_.reset();
    handlers_.reset();
    ctx_.reset();
    node_.reset();
    std::error_code ec;
    std::filesystem::remove_all(bag_dir_, ec);
  }

  static ros2_medkit_msgs::msg::Fault make_record(const std::string & code, const std::string & owner) {
    ros2_medkit_msgs::msg::Fault fault;
    fault.fault_code = code;
    fault.status = ros2_medkit_msgs::msg::Fault::STATUS_CONFIRMED;
    fault.severity = ros2_medkit_msgs::msg::Fault::SEVERITY_ERROR;
    fault.reporting_sources = {owner};
    return fault;
  }

  /// One stored record, owned by `owner`.
  void store_record(const std::string & code, const std::string & owner) {
    std::lock_guard<std::mutex> lock(store_mutex_);
    stored_faults_.push_back(make_record(code, owner));
  }

  /// One muted record, owned by `owner`. Served only to a listing that asks.
  void store_muted_record(const std::string & code, const std::string & owner) {
    std::lock_guard<std::mutex> lock(store_mutex_);
    muted_faults_.push_back(make_record(code, owner));
  }

  /// A recording of the record (code, owner), with real bytes behind it.
  void store_recording(const std::string & id, const std::string & code, const std::string & owner) {
    const auto path = bag_dir_ / (id + ".mcap");
    {
      std::ofstream out(path, std::ios::binary);
      out << "bag of " << owner;
    }
    std::lock_guard<std::mutex> lock(store_mutex_);
    recordings_.push_back(Recording{id, code, owner, path.string()});
  }

  /// A component hosting two external apps, all three owned by the plugin, so
  /// the fault routes take the plugin branch on every one of them.
  RecordingFaultPlugin * seed_topology() {
    return seed_plugin_topology<RecordingFaultPlugin>();
  }

  /// The same topology owned by a plugin of type P.
  template <typename P>
  P * seed_plugin_topology() {
    seed_entities();
    auto plugin = std::make_unique<P>();
    auto * raw = plugin.get();
    const std::string plugin_name = raw->name();
    auto * pmgr = node_->get_plugin_manager();
    pmgr->add_plugin(std::move(plugin));
    pmgr->register_entity_ownership(plugin_name, {kComponent, kHostedApp, kOtherApp});
    return raw;
  }

  /// The same component and apps, owned by NOBODY, so the fault routes take the
  /// native fault-manager path and resolve through resolve_scoped_fault.
  void seed_native_topology() {
    seed_entities();
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

  std::vector<std::pair<std::string, std::string>> cleared_snapshot() {
    std::lock_guard<std::mutex> lock(cleared_mutex_);
    return cleared_;
  }

  /// The stored status of the record (code, owner), muted or not.
  std::string status_of(const std::string & code, const std::string & owner) {
    std::lock_guard<std::mutex> lock(store_mutex_);
    for (const auto * list : {&stored_faults_, &muted_faults_}) {
      for (const auto & fault : *list) {
        if (fault.fault_code == code && fault.reporting_sources.front() == owner) {
          return fault.status;
        }
      }
    }
    return "";
  }

  std::vector<std::pair<std::string, std::string>> get_requests_snapshot() {
    std::lock_guard<std::mutex> lock(store_mutex_);
    return get_requests_;
  }

  std::vector<std::string> rosbag_requests_snapshot() {
    std::lock_guard<std::mutex> lock(store_mutex_);
    return rosbag_requests_;
  }

  static std::string component_fault_path() {
    return std::string("/api/v1/components/") + kComponent + "/faults/" + kCode;
  }
  static constexpr const char * kComponentFaultPattern = R"(/api/v1/components/([^/]+)/faults/([^/]+))";
  static constexpr const char * kComponentBagPattern = R"(/api/v1/components/([^/]+)/bulk-data/([^/]+)/([^/]+))";

  static inline int test_counter_ = 0;
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::string ns_;
  std::filesystem::path bag_dir_;
  std::shared_ptr<GatewayNode> node_;
  std::shared_ptr<rclcpp::Node> store_;
  rclcpp::Service<ListFaults>::SharedPtr list_srv_;
  rclcpp::Service<GetFault>::SharedPtr get_srv_;
  rclcpp::Service<ros2_medkit_msgs::srv::ClearFault>::SharedPtr clear_srv_;
  rclcpp::Service<GetRosbag>::SharedPtr rosbag_srv_;
  rclcpp::Service<ListRosbags>::SharedPtr list_rosbags_srv_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_;

  std::mutex store_mutex_;
  std::vector<ros2_medkit_msgs::msg::Fault> stored_faults_;
  std::vector<ros2_medkit_msgs::msg::Fault> muted_faults_;
  std::vector<Recording> recordings_;
  std::vector<std::pair<std::string, std::string>> get_requests_;
  std::vector<std::string> rosbag_requests_;
  std::atomic<bool> last_include_muted_{false};

  std::mutex cleared_mutex_;
  std::vector<std::pair<std::string, std::string>> cleared_;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<FaultHandlers> handlers_;
  std::unique_ptr<BulkDataHandlers> bulk_handlers_;

 private:
  void seed_entities() {
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
  }
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

// The plugin clear reads the fault manager to learn which record the code
// names. When that read fails the route cannot tell a record the fault manager
// holds from a plugin-internal one, so it must not guess: calling the provider
// with an empty owner answered 2xx while the record stayed untouched.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, PluginClearAnswers503WhenTheFaultManagerCannotBeRead) {
  auto * plugin = seed_topology();
  plugin->fail_if_called = true;
  store_record(kCode, kHostedApp);
  ASSERT_TRUE(wait_for_store());
  // Take ListFaults away and wait until the graph agrees it is gone, so the
  // call below fails on the read and not on a race with the service teardown.
  list_srv_.reset();
  auto probe = store_->create_client<ListFaults>("/" + ns_ + "/fault_manager/list_faults");
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (probe->service_is_ready() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(20ms);
  }
  ASSERT_FALSE(probe->service_is_ready()) << "ListFaults is still discoverable";

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_FALSE(result.has_value()) << "a clear the gateway could not resolve must not report success";
  EXPECT_EQ(result.error().http_status, 503);
  EXPECT_EQ(result.error().code, ros2_medkit_gateway::ERR_SERVICE_UNAVAILABLE);
  EXPECT_TRUE(plugin->clears.empty()) << "the provider was called without a resolved owner";
}

// The positive control for the case above, on the same fixture and the same
// stub: with ListFaults answering, the same request reaches the provider with
// the owner the gateway resolved. So the silence above is the route refusing,
// not a harness that never reaches the provider.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, PluginClearReachesTheProviderWhenTheFaultManagerAnswers) {
  auto * plugin = seed_topology();
  store_record(kCode, kHostedApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  ASSERT_EQ(plugin->clears.size(), 1u);
  EXPECT_EQ(plugin->clears[0].owner, kHostedApp);
}

// A provider that predates the owner-aware clear overrides only the
// two-argument clear_fault. It must still compile against these headers and
// still receive the clear the gateway resolved, through the default
// clear_fault_record, which hands it the entity and the code as before.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, ATwoArgumentClearProviderStillReceivesTheClear) {
  auto * plugin = seed_plugin_topology<TwoArgumentClearPlugin>();
  store_record(kCode, kHostedApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  ASSERT_TRUE(std::holds_alternative<FaultClearResult>(*result)) << "the plugin's acknowledgement is the answer";
  ASSERT_EQ(plugin->clears.size(), 1u) << "the two-argument clear_fault never received the clear";
  EXPECT_EQ(plugin->clears[0].entity_id, kComponent);
  EXPECT_EQ(plugin->clears[0].fault_code, kCode);
}

// ---------------------------------------------------------------------------
// Which record a code in the URL names when muted records are in scope.
//
// Every per-entity fault list leaves muted records out, so the records it shows
// are the ones a client can read a code off. A code resolves over those first.
// Only when the list shows no record of the code does a muted one answer, so a
// record hidden as a symptom stays reachable without turning a shown record
// into an ambiguous address. Each route that resolves a code gets its own case,
// because each reads the fault manager on its own.
// ---------------------------------------------------------------------------

// The component's list shows tank's record only, because pump's record of the
// same code is muted. Resolving over both answered 409 naming pump as well, and
// pointed the client at a list that could never show it pump's record.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, NativeComponentClearTakesTheShownRecordOverAMutedOne) {
  seed_native_topology();
  store_record(kCode, kHostedApp);
  store_muted_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << "the shown record is the one the code names: " << result.error().message;
  EXPECT_TRUE(std::holds_alternative<ros2_medkit_gateway::http::NoContent>(*result));
  const auto cleared = cleared_snapshot();
  ASSERT_EQ(cleared.size(), 1u) << "exactly the shown record is cleared, and pump's muted record is left alone";
  EXPECT_EQ(cleared[0].first, kCode);
  EXPECT_EQ(cleared[0].second, kHostedApp);
}

// The same scenario on a plugin-owned component: the plugin clear resolves the
// owner on its own read of the fault manager, so it needs its own case.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, PluginComponentClearTakesTheShownRecordOverAMutedOne) {
  auto * plugin = seed_topology();
  store_record(kCode, kHostedApp);
  store_muted_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << "the shown record is the one the code names: " << result.error().message;
  ASSERT_EQ(plugin->clears.size(), 1u);
  EXPECT_EQ(plugin->clears[0].owner, kHostedApp) << "the plugin must be handed the shown record's owner";
}

// A muted record with no shown record of its code is still the record the code
// names, and the detail route serves it with its owner.
// @verifies REQ_INTEROP_013
TEST_F(PluginClearOwnerTest, AMutedRecordAloneIsServedOnTheDetailRoute) {
  seed_native_topology();
  store_muted_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern, "GET");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->get_fault(typed);

  ASSERT_TRUE(result.has_value()) << "a muted record must stay addressable: " << result.error().message;
  EXPECT_EQ(result->content["x-medkit"]["owner"], kOtherApp);
  const auto reads = get_requests_snapshot();
  ASSERT_EQ(reads.size(), 1u);
  EXPECT_EQ(reads[0].second, kOtherApp) << "the enriched read must be addressed to the muted record's owner";
}

// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, AMutedRecordAloneIsClearedOnTheNativeRoute) {
  seed_native_topology();
  store_muted_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << "a muted record must stay addressable: " << result.error().message;
  const auto cleared = cleared_snapshot();
  ASSERT_EQ(cleared.size(), 1u);
  EXPECT_EQ(cleared[0].second, kOtherApp);
}

// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, AMutedRecordAloneReachesThePluginWithItsOwner) {
  auto * plugin = seed_topology();
  store_muted_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_TRUE(result.has_value()) << result.error().message;
  ASSERT_EQ(plugin->clears.size(), 1u);
  EXPECT_EQ(plugin->clears[0].owner, kOtherApp) << "the plugin must be handed the muted record's owner";
}

// A recording URL carrying a fault code resolves that code to a record first.
// A muted record keeps its recordings, so its code still serves its bytes.
// @verifies REQ_INTEROP_072
TEST_F(PluginClearOwnerTest, AMutedRecordAloneServesItsRecordingByCode) {
  seed_native_topology();
  store_muted_record(kCode, kOtherApp);
  store_recording("rec_pump", kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/components/") + kComponent + "/bulk-data/rosbags/" + kCode,
                    kComponentBagPattern, "GET");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = bulk_handlers_->download(typed);

  ASSERT_TRUE(result.has_value()) << "a muted record's recording must stay reachable: " << result.error().message;
  EXPECT_EQ(result->filename.value_or(""), "rec_pump.mcap");
  const auto asked = rosbag_requests_snapshot();
  ASSERT_EQ(asked.size(), 1u);
  EXPECT_EQ(asked[0], kOtherApp) << "the recording must be asked for under the muted record's owner";
}

// The recording route resolves a code the way the fault routes do: tank's
// record is the one the component's list shows, so the code names tank's
// recording and pump's muted record does not make it ambiguous.
// @verifies REQ_INTEROP_072
TEST_F(PluginClearOwnerTest, ARecordingCodeTakesTheShownRecordOverAMutedOne) {
  seed_native_topology();
  store_record(kCode, kHostedApp);
  store_muted_record(kCode, kOtherApp);
  store_recording("rec_tank", kCode, kHostedApp);
  store_recording("rec_pump", kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/components/") + kComponent + "/bulk-data/rosbags/" + kCode,
                    kComponentBagPattern, "GET");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = bulk_handlers_->download(typed);

  ASSERT_TRUE(result.has_value()) << "the shown record is the one the code names: " << result.error().message;
  EXPECT_EQ(result->filename.value_or(""), "rec_tank.mcap");
  const auto asked = rosbag_requests_snapshot();
  ASSERT_EQ(asked.size(), 1u);
  EXPECT_EQ(asked[0], kHostedApp);
}

// Two muted records and no shown one: the code names neither, the route says so
// and names both owners, and it points at where each can be addressed.
// @verifies REQ_INTEROP_015
TEST_F(PluginClearOwnerTest, TwoMutedRecordsAloneAreAmbiguous) {
  seed_native_topology();
  store_muted_record(kCode, kHostedApp);
  store_muted_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(component_fault_path(), kComponentFaultPattern);
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = handlers_->clear_fault(typed);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 409);
  EXPECT_EQ(result.error().params["owners"], json::array({kOtherApp, kHostedApp}));
  const std::string details = result.error().params.value("details", "");
  EXPECT_NE(details.find("parameters.owners"), std::string::npos) << details;
  EXPECT_NE(details.find("/apps/{app_id}/faults/{fault_code}"), std::string::npos) << details;
  EXPECT_TRUE(cleared_snapshot().empty()) << "an ambiguous address must clear nothing";
}

// @verifies REQ_INTEROP_072
TEST_F(PluginClearOwnerTest, TwoMutedRecordsAloneMakeTheRecordingCodeAmbiguous) {
  seed_native_topology();
  store_muted_record(kCode, kHostedApp);
  store_muted_record(kCode, kOtherApp);
  store_recording("rec_tank", kCode, kHostedApp);
  store_recording("rec_pump", kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest req(std::string("/api/v1/components/") + kComponent + "/bulk-data/rosbags/" + kCode,
                    kComponentBagPattern, "GET");
  ASSERT_TRUE(req.matched());
  ros2_medkit_gateway::http::TypedRequest typed(req.raw());

  auto result = bulk_handlers_->download(typed);

  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().http_status, 409);
  EXPECT_EQ(result.error().params["owners"], json::array({kOtherApp, kHostedApp}));
  EXPECT_TRUE(rosbag_requests_snapshot().empty()) << "no recording is looked up for an ambiguous code";
}

// The bulk clear walks the entity's fault list, which leaves muted records out,
// and it always has. It clears what the list shows and leaves a muted record
// CONFIRMED. That record is not stranded: its own per-code DELETE clears it.
// @verifies REQ_INTEROP_014
TEST_F(PluginClearOwnerTest, BulkClearLeavesAMutedRecordToItsPerCodeClear) {
  seed_native_topology();
  store_record(kCode, kHostedApp);
  store_muted_record(kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());

  RoutedRequest bulk(std::string("/api/v1/components/") + kComponent + "/faults",
                     R"(/api/v1/components/([^/]+)/faults)");
  ASSERT_TRUE(bulk.matched());
  auto bulk_result = handlers_->clear_all_faults(ros2_medkit_gateway::http::TypedRequest(bulk.raw()));

  ASSERT_TRUE(bulk_result.has_value()) << bulk_result.error().message;
  EXPECT_EQ(status_of(kCode, kHostedApp), "CLEARED") << "the bulk clear must clear the record the list shows";
  EXPECT_EQ(status_of(kCode, kOtherApp), "CONFIRMED") << "the bulk clear must leave the muted record alone";

  RoutedRequest per_code(std::string("/api/v1/apps/") + kOtherApp + "/faults/" + kCode,
                         R"(/api/v1/apps/([^/]+)/faults/([^/]+))");
  ASSERT_TRUE(per_code.matched());
  auto per_code_result = handlers_->clear_fault(ros2_medkit_gateway::http::TypedRequest(per_code.raw()));

  ASSERT_TRUE(per_code_result.has_value()) << per_code_result.error().message;
  EXPECT_EQ(status_of(kCode, kOtherApp), "CLEARED") << "the muted record's own per-code DELETE must clear it";
}

// The recording route's 409 has to send the client somewhere that works. It
// names the rosbags listing and the fault detail's bulk_data_uri, and following
// it does work: the listing carries one descriptor per recording, and each
// descriptor id downloads that owner's bytes.
// @verifies REQ_INTEROP_072
TEST_F(PluginClearOwnerTest, TheAmbiguousRecordingCodeNamesWhereEachRecordingIsListed) {
  seed_native_topology();
  store_record(kCode, kHostedApp);
  store_record(kCode, kOtherApp);
  store_recording("rec_tank", kCode, kHostedApp);
  store_recording("rec_pump", kCode, kOtherApp);
  ASSERT_TRUE(wait_for_store());
  const std::string rosbags = std::string("/api/v1/components/") + kComponent + "/bulk-data/rosbags";

  RoutedRequest by_code(rosbags + "/" + kCode, kComponentBagPattern, "GET");
  ASSERT_TRUE(by_code.matched());
  auto refused = bulk_handlers_->download(ros2_medkit_gateway::http::TypedRequest(by_code.raw()));
  ASSERT_FALSE(refused.has_value());
  ASSERT_EQ(refused.error().http_status, 409);
  const std::string details = refused.error().params.value("details", "");
  EXPECT_NE(details.find("GET .../bulk-data/rosbags"), std::string::npos) << details;
  EXPECT_NE(details.find("descriptor's id"), std::string::npos) << details;
  EXPECT_NE(details.find("environment_data.snapshots[].bulk_data_uri"), std::string::npos) << details;
  EXPECT_EQ(details.find("x-medkit.recording_id"), std::string::npos)
      << "the fault listing carries no recording id, so the message must not send the client there: " << details;

  // Follow the message: list the category, then download each descriptor id.
  RoutedRequest listing(rosbags, R"(/api/v1/components/([^/]+)/bulk-data/([^/]+))", "GET");
  ASSERT_TRUE(listing.matched());
  auto listed = bulk_handlers_->list_descriptors(ros2_medkit_gateway::http::TypedRequest(listing.raw()));
  ASSERT_TRUE(listed.has_value()) << listed.error().message;
  std::vector<std::string> ids;
  for (const auto & descriptor : listed->items) {
    ids.push_back(descriptor.id);
  }
  std::sort(ids.begin(), ids.end());
  ASSERT_EQ(ids, (std::vector<std::string>{"rec_pump", "rec_tank"}));

  for (const auto & id : ids) {
    RoutedRequest by_id(rosbags + "/" + id, kComponentBagPattern, "GET");
    ASSERT_TRUE(by_id.matched());
    auto served = bulk_handlers_->download(ros2_medkit_gateway::http::TypedRequest(by_id.raw()));
    ASSERT_TRUE(served.has_value()) << id << ": " << served.error().message;
    EXPECT_EQ(served->filename.value_or(""), id + ".mcap");
  }
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
