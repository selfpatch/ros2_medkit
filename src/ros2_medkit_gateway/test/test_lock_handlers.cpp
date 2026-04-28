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

#include <arpa/inet.h>
#include <httplib.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include <cstring>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/lock_handlers.hpp"
#include "ros2_medkit_gateway/core/http/http_utils.hpp"
#include "ros2_medkit_gateway/core/managers/lock_manager.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::LockConfig;
using ros2_medkit_gateway::LockManager;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::LockHandlers;

namespace {

// Manifest with components and apps for entity validation
const char * kManifestYaml = R"(
manifest_version: "1.0"
metadata:
  name: "lock-handlers-test"
  version: "1.0.0"
components:
  - id: "ecu1"
    name: "ECU 1"
    namespace: "/ecu1"
apps:
  - id: "planner"
    name: "Planner"
    is_located_on: "ecu1"
)";

int reserve_local_port() {
  int sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0) {
    return 0;
  }
  int opt = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  if (bind(sock, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    close(sock);
    return 0;
  }
  socklen_t addr_len = sizeof(addr);
  if (getsockname(sock, reinterpret_cast<sockaddr *>(&addr), &addr_len) != 0) {
    close(sock);
    return 0;
  }
  int port = ntohs(addr.sin_port);
  close(sock);
  return port;
}

httplib::Request make_request_with_match(const std::string & path, const std::string & pattern) {
  httplib::Request req;
  req.path = path;
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

std::string write_temp_manifest(const std::string & contents) {
  char path_template[] = "/tmp/ros2_medkit_lock_handlers_XXXXXX.yaml";
  int fd = mkstemps(path_template, 5);
  if (fd < 0) {
    return {};
  }
  close(fd);
  std::ofstream out(path_template);
  if (!out) {
    std::remove(path_template);
    return {};
  }
  out << contents;
  out.close();
  return path_template;
}

}  // namespace

// ============================================================================
// Static tests (no GatewayNode required)
// ============================================================================

TEST(LockHandlersStaticTest, LockingDisabledReturns501) {
  CorsConfig cors;
  AuthConfig auth;
  TlsConfig tls;
  HandlerContext ctx(nullptr, cors, auth, tls, nullptr);
  LockHandlers handlers(ctx, nullptr);

  httplib::Request req;
  httplib::Response res;
  handlers.handle_acquire_lock(req, res);
  EXPECT_EQ(res.status, 501);

  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "not-implemented");
}

TEST(LockHandlersStaticTest, LockToJsonWithMatchingClientShowsOwned) {
  ros2_medkit_gateway::LockInfo lock;
  lock.lock_id = "lock_1";
  lock.entity_id = "comp1";
  lock.client_id = "client_a";
  lock.scopes = {"configurations"};
  lock.expires_at = std::chrono::steady_clock::now() + std::chrono::seconds(300);

  auto j = LockHandlers::lock_to_json(lock, "client_a");
  EXPECT_EQ(j["id"], "lock_1");
  EXPECT_TRUE(j["owned"].get<bool>());
  ASSERT_TRUE(j.contains("scopes"));
  EXPECT_EQ(j["scopes"].size(), 1);
  EXPECT_EQ(j["scopes"][0], "configurations");
  EXPECT_TRUE(j.contains("lock_expiration"));
  auto expiration = j["lock_expiration"].get<std::string>();
  EXPECT_TRUE(expiration.find("T") != std::string::npos);
  EXPECT_TRUE(expiration.find("Z") != std::string::npos);
}

TEST(LockHandlersStaticTest, LockToJsonWithDifferentClientShowsNotOwned) {
  ros2_medkit_gateway::LockInfo lock;
  lock.lock_id = "lock_2";
  lock.entity_id = "comp1";
  lock.client_id = "client_a";
  lock.scopes = {};
  lock.expires_at = std::chrono::steady_clock::now() + std::chrono::seconds(300);

  auto j = LockHandlers::lock_to_json(lock, "client_b");
  EXPECT_FALSE(j["owned"].get<bool>());
  // Empty scopes should not produce "scopes" field
  EXPECT_FALSE(j.contains("scopes"));
}

TEST(LockHandlersStaticTest, LockToJsonWithEmptyClientShowsNotOwned) {
  ros2_medkit_gateway::LockInfo lock;
  lock.lock_id = "lock_3";
  lock.entity_id = "comp1";
  lock.client_id = "client_a";
  lock.expires_at = std::chrono::steady_clock::now() + std::chrono::seconds(300);

  auto j = LockHandlers::lock_to_json(lock, "");
  EXPECT_FALSE(j["owned"].get<bool>());
}

// ============================================================================
// Full handler tests (with GatewayNode for entity validation)
// ============================================================================

class LockHandlersTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> suite_node_;
  static inline int suite_server_port_ = 0;

  static void SetUpTestSuite() {
    suite_server_port_ = reserve_local_port();
    ASSERT_NE(suite_server_port_, 0);

    std::vector<std::string> args = {"test_lock_handlers",
                                     "--ros-args",
                                     "-p",
                                     "server.port:=" + std::to_string(suite_server_port_),
                                     "-p",
                                     "refresh_interval_ms:=60000",
                                     "-p",
                                     "locking.enabled:=true",
                                     "-p",
                                     "locking.default_max_expiration:=3600"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }

    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    suite_node_ = std::make_shared<GatewayNode>();
    ASSERT_NE(suite_node_, nullptr);
  }

  static void TearDownTestSuite() {
    suite_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    ASSERT_NE(suite_node_, nullptr);

    // Write manifest and initialize discovery
    manifest_path_ = write_temp_manifest(kManifestYaml);
    ASSERT_FALSE(manifest_path_.empty());

    ros2_medkit_gateway::DiscoveryConfig config;
    config.mode = ros2_medkit_gateway::DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = manifest_path_;
    config.manifest_strict_validation = false;

    ASSERT_TRUE(suite_node_->get_discovery_manager()->initialize(config));

    auto areas = suite_node_->get_discovery_manager()->discover_areas();
    auto components = suite_node_->get_discovery_manager()->discover_components();
    auto apps = suite_node_->get_discovery_manager()->discover_apps();
    auto functions = suite_node_->get_discovery_manager()->discover_functions();

    // Set up apps with FQNs
    for (auto & app : apps) {
      if (app.id == "planner") {
        app.bound_fqn = "/ecu1/planner";
      }
    }

    auto & cache = const_cast<ThreadSafeEntityCache &>(suite_node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);

    // Get the LockManager from the node
    lock_manager_ = suite_node_->get_lock_manager();
    ASSERT_NE(lock_manager_, nullptr) << "LockManager should be created when locking.enabled=true";

    ctx_ = std::make_unique<HandlerContext>(suite_node_.get(), cors_, auth_, tls_, nullptr);
    handlers_ = std::make_unique<LockHandlers>(*ctx_, lock_manager_);
  }

  void TearDown() override {
    handlers_.reset();
    ctx_.reset();

    // Clean up any locks from previous test
    if (lock_manager_) {
      // Release locks by acquiring and releasing for known entities
      for (const auto & eid : {"ecu1", "planner"}) {
        auto lock = lock_manager_->get_lock(eid);
        if (lock) {
          // Force-acquire to take ownership, then release
          (void)lock_manager_->acquire(eid, "__cleanup__", {}, 1, true);
          (void)lock_manager_->release(eid, "__cleanup__");
        }
      }
    }

    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  // Helper: build a POST /api/v1/components/{id}/locks request
  httplib::Request make_component_locks_request(const std::string & component_id) {
    return make_request_with_match("/api/v1/components/" + component_id + "/locks",
                                   R"(/api/v1/components/([^/]+)/locks)");
  }

  // Helper: build a POST /api/v1/apps/{id}/locks request
  httplib::Request make_app_locks_request(const std::string & app_id) {
    return make_request_with_match("/api/v1/apps/" + app_id + "/locks", R"(/api/v1/apps/([^/]+)/locks)");
  }

  // Helper: build a GET/PUT/DELETE /api/v1/components/{id}/locks/{lock_id} request
  httplib::Request make_component_lock_item_request(const std::string & component_id, const std::string & lock_id) {
    return make_request_with_match("/api/v1/components/" + component_id + "/locks/" + lock_id,
                                   R"(/api/v1/components/([^/]+)/locks/([^/]+))");
  }

  // Helper: build a GET/PUT/DELETE /api/v1/apps/{id}/locks/{lock_id} request
  httplib::Request make_app_lock_item_request(const std::string & app_id, const std::string & lock_id) {
    return make_request_with_match("/api/v1/apps/" + app_id + "/locks/" + lock_id,
                                   R"(/api/v1/apps/([^/]+)/locks/([^/]+))");
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  LockManager * lock_manager_ = nullptr;
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<LockHandlers> handlers_;
  std::string manifest_path_;
};

// ============================================================================
// POST acquire lock
// ============================================================================

TEST_F(LockHandlersTest, AcquireLockOnComponentReturns201) {
  auto req = make_component_locks_request("ecu1");
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"lock_expiration": 300})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 201);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("id"));
  EXPECT_TRUE(body["owned"].get<bool>());
  EXPECT_TRUE(body.contains("lock_expiration"));
  auto expiration = body["lock_expiration"].get<std::string>();
  EXPECT_TRUE(expiration.find("T") != std::string::npos);
  EXPECT_TRUE(expiration.find("Z") != std::string::npos);
  // No scopes specified - field should be absent
  EXPECT_FALSE(body.contains("scopes"));
}

TEST_F(LockHandlersTest, AcquireLockOnAppReturns201) {
  auto req = make_app_locks_request("planner");
  req.set_header("X-Client-Id", "client_b");
  req.body = R"({"lock_expiration": 600, "scopes": ["configurations"]})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 201);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body.contains("id"));
  EXPECT_TRUE(body["owned"].get<bool>());
  ASSERT_TRUE(body.contains("scopes"));
  EXPECT_EQ(body["scopes"].size(), 1);
  EXPECT_EQ(body["scopes"][0], "configurations");
  EXPECT_TRUE(body.contains("lock_expiration"));
}

TEST_F(LockHandlersTest, AcquireLockWithoutClientIdReturns400) {
  auto req = make_component_locks_request("ecu1");
  // No X-Client-Id header
  req.body = R"({"lock_expiration": 300})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "invalid-parameter");
}

TEST_F(LockHandlersTest, AcquireLockWithInvalidScopeReturns400) {
  auto req = make_component_locks_request("ecu1");
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"lock_expiration": 300, "scopes": ["invalid_scope"]})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "invalid-parameter");
}

TEST_F(LockHandlersTest, AcquireLockWithMissingExpirationReturns400) {
  auto req = make_component_locks_request("ecu1");
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"scopes": ["configurations"]})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "invalid-parameter");
}

TEST_F(LockHandlersTest, AcquireLockWithZeroExpirationReturns400) {
  auto req = make_component_locks_request("ecu1");
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"lock_expiration": 0})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "invalid-parameter");
}

TEST_F(LockHandlersTest, AcquireLockAlreadyLockedReturns409) {
  // First acquire succeeds
  auto req1 = make_component_locks_request("ecu1");
  req1.set_header("X-Client-Id", "client_a");
  req1.body = R"({"lock_expiration": 300})";
  httplib::Response res1;
  handlers_->handle_acquire_lock(req1, res1);
  ASSERT_EQ(res1.status, 201);

  // Second acquire by different client should fail
  auto req2 = make_component_locks_request("ecu1");
  req2.set_header("X-Client-Id", "client_b");
  req2.body = R"({"lock_expiration": 300})";
  httplib::Response res2;
  handlers_->handle_acquire_lock(req2, res2);

  EXPECT_EQ(res2.status, 409);
  auto body = json::parse(res2.body);
  EXPECT_EQ(body["error_code"], "invalid-request");
}

TEST_F(LockHandlersTest, AcquireLockOnNonexistentEntityReturns404) {
  auto req = make_component_locks_request("nonexistent_component");
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"lock_expiration": 300})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 404);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "entity-not-found");
}

TEST_F(LockHandlersTest, AcquireLockWithInvalidJsonReturns400) {
  auto req = make_component_locks_request("ecu1");
  req.set_header("X-Client-Id", "client_a");
  req.body = "not json";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 400);
}

// ============================================================================
// GET list locks
// ============================================================================

TEST_F(LockHandlersTest, ListLocksReturnsItemsArray) {
  // Acquire a lock first
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300, "scopes": ["configurations"]})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  // List locks
  auto req = make_component_locks_request("ecu1");
  req.set_header("X-Client-Id", "client_a");
  httplib::Response res;
  handlers_->handle_list_locks(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body.contains("items"));
  ASSERT_TRUE(body["items"].is_array());
  ASSERT_EQ(body["items"].size(), 1);

  auto & lock_item = body["items"][0];
  EXPECT_TRUE(lock_item.contains("id"));
  EXPECT_TRUE(lock_item["owned"].get<bool>());
  ASSERT_TRUE(lock_item.contains("scopes"));
  EXPECT_EQ(lock_item["scopes"][0], "configurations");
  // Verify lock_expiration is ISO 8601
  auto expiration = lock_item["lock_expiration"].get<std::string>();
  EXPECT_TRUE(expiration.find("T") != std::string::npos);
  EXPECT_TRUE(expiration.find("Z") != std::string::npos);
}

TEST_F(LockHandlersTest, ListLocksOwnedFieldReflectsClient) {
  // Acquire a lock
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  // List as same client - should be owned
  auto req1 = make_component_locks_request("ecu1");
  req1.set_header("X-Client-Id", "client_a");
  httplib::Response res1;
  handlers_->handle_list_locks(req1, res1);
  auto body1 = json::parse(res1.body);
  EXPECT_TRUE(body1["items"][0]["owned"].get<bool>());

  // List as different client - should not be owned
  auto req2 = make_component_locks_request("ecu1");
  req2.set_header("X-Client-Id", "client_b");
  httplib::Response res2;
  handlers_->handle_list_locks(req2, res2);
  auto body2 = json::parse(res2.body);
  EXPECT_FALSE(body2["items"][0]["owned"].get<bool>());

  // List without client id - should not be owned
  auto req3 = make_component_locks_request("ecu1");
  httplib::Response res3;
  handlers_->handle_list_locks(req3, res3);
  auto body3 = json::parse(res3.body);
  EXPECT_FALSE(body3["items"][0]["owned"].get<bool>());
}

TEST_F(LockHandlersTest, ListLocksEmptyWhenNoLocks) {
  auto req = make_component_locks_request("ecu1");
  httplib::Response res;
  handlers_->handle_list_locks(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = json::parse(res.body);
  ASSERT_TRUE(body["items"].is_array());
  EXPECT_TRUE(body["items"].empty());
}

// ============================================================================
// GET single lock
// ============================================================================

TEST_F(LockHandlersTest, GetLockReturns200) {
  // Acquire a lock
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Get the lock
  auto req = make_component_lock_item_request("ecu1", lock_id);
  req.set_header("X-Client-Id", "client_a");
  httplib::Response res;
  handlers_->handle_get_lock(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["id"], lock_id);
  EXPECT_TRUE(body["owned"].get<bool>());
  EXPECT_TRUE(body.contains("lock_expiration"));
}

TEST_F(LockHandlersTest, GetLockNonExistentReturns404) {
  auto req = make_component_lock_item_request("ecu1", "lock_nonexistent");
  httplib::Response res;
  handlers_->handle_get_lock(req, res);

  EXPECT_EQ(res.status, 404);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "resource-not-found");
}

// ============================================================================
// PUT extend lock
// ============================================================================

TEST_F(LockHandlersTest, ExtendLockReturns204) {
  // Acquire a lock
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Extend
  auto req = make_component_lock_item_request("ecu1", lock_id);
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"lock_expiration": 600})";
  httplib::Response res;
  handlers_->handle_extend_lock(req, res);

  EXPECT_EQ(res.status, 204);
}

TEST_F(LockHandlersTest, ExtendLockNotOwnerReturns403) {
  // Acquire a lock as client_a
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Try to extend as client_b - should fail with 403
  auto req = make_component_lock_item_request("ecu1", lock_id);
  req.set_header("X-Client-Id", "client_b");
  req.body = R"({"lock_expiration": 600})";
  httplib::Response res;
  handlers_->handle_extend_lock(req, res);

  EXPECT_EQ(res.status, 403);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "forbidden");
}

TEST_F(LockHandlersTest, ExtendLockWithoutClientIdReturns400) {
  // Acquire a lock
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // No X-Client-Id header
  auto req = make_component_lock_item_request("ecu1", lock_id);
  req.body = R"({"lock_expiration": 600})";
  httplib::Response res;
  handlers_->handle_extend_lock(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "invalid-parameter");
}

// ============================================================================
// DELETE release lock
// ============================================================================

TEST_F(LockHandlersTest, ReleaseLockReturns204) {
  // Acquire a lock
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Release
  auto req = make_component_lock_item_request("ecu1", lock_id);
  req.set_header("X-Client-Id", "client_a");
  httplib::Response res;
  handlers_->handle_release_lock(req, res);

  EXPECT_EQ(res.status, 204);

  // Verify lock is gone
  auto check = lock_manager_->get_lock("ecu1");
  EXPECT_FALSE(check.has_value());
}

TEST_F(LockHandlersTest, ReleaseLockNotOwnerReturns403) {
  // Acquire a lock as client_a
  auto acquire_req = make_component_locks_request("ecu1");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Try to release as client_b - should fail with 403
  auto req = make_component_lock_item_request("ecu1", lock_id);
  req.set_header("X-Client-Id", "client_b");
  httplib::Response res;
  handlers_->handle_release_lock(req, res);

  EXPECT_EQ(res.status, 403);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "forbidden");
}

TEST_F(LockHandlersTest, ReleaseLockNonexistentReturns404) {
  auto req = make_component_lock_item_request("ecu1", "lock_nonexistent");
  req.set_header("X-Client-Id", "client_a");
  httplib::Response res;
  handlers_->handle_release_lock(req, res);

  EXPECT_EQ(res.status, 404);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "resource-not-found");
}

TEST_F(LockHandlersTest, ReleaseLockWithoutClientIdReturns400) {
  auto req = make_component_lock_item_request("ecu1", "some_lock");
  // No X-Client-Id
  httplib::Response res;
  handlers_->handle_release_lock(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "invalid-parameter");
}

// ============================================================================
// App path tests (dual-path validation)
// ============================================================================

TEST_F(LockHandlersTest, AcquireLockOnAppPathReturns201) {
  auto req = make_app_locks_request("planner");
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"lock_expiration": 300, "scopes": ["operations", "configurations"]})";
  httplib::Response res;

  handlers_->handle_acquire_lock(req, res);

  EXPECT_EQ(res.status, 201);
  auto body = json::parse(res.body);
  EXPECT_TRUE(body["owned"].get<bool>());
  ASSERT_TRUE(body.contains("scopes"));
  EXPECT_EQ(body["scopes"].size(), 2);
}

TEST_F(LockHandlersTest, ListLocksOnAppPathReturns200) {
  // Acquire first
  auto acquire_req = make_app_locks_request("planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  // List
  auto req = make_app_locks_request("planner");
  httplib::Response res;
  handlers_->handle_list_locks(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = json::parse(res.body);
  ASSERT_EQ(body["items"].size(), 1);
}

TEST_F(LockHandlersTest, GetLockOnAppPathReturns200) {
  // Acquire
  auto acquire_req = make_app_locks_request("planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Get
  auto req = make_app_lock_item_request("planner", lock_id);
  httplib::Response res;
  handlers_->handle_get_lock(req, res);

  EXPECT_EQ(res.status, 200);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["id"], lock_id);
}

TEST_F(LockHandlersTest, ExtendLockOnAppPathReturns204) {
  // Acquire
  auto acquire_req = make_app_locks_request("planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Extend
  auto req = make_app_lock_item_request("planner", lock_id);
  req.set_header("X-Client-Id", "client_a");
  req.body = R"({"lock_expiration": 600})";
  httplib::Response res;
  handlers_->handle_extend_lock(req, res);

  EXPECT_EQ(res.status, 204);
}

TEST_F(LockHandlersTest, ReleaseLockOnAppPathReturns204) {
  // Acquire
  auto acquire_req = make_app_locks_request("planner");
  acquire_req.set_header("X-Client-Id", "client_a");
  acquire_req.body = R"({"lock_expiration": 300})";
  httplib::Response acquire_res;
  handlers_->handle_acquire_lock(acquire_req, acquire_res);
  ASSERT_EQ(acquire_res.status, 201);

  auto acquire_body = json::parse(acquire_res.body);
  auto lock_id = acquire_body["id"].get<std::string>();

  // Release
  auto req = make_app_lock_item_request("planner", lock_id);
  req.set_header("X-Client-Id", "client_a");
  httplib::Response res;
  handlers_->handle_release_lock(req, res);

  EXPECT_EQ(res.status, 204);
}

// ============================================================================
// Break lock test
// ============================================================================

TEST_F(LockHandlersTest, AcquireLockWithBreakReplacesExisting) {
  // First acquire
  auto req1 = make_component_locks_request("ecu1");
  req1.set_header("X-Client-Id", "client_a");
  req1.body = R"({"lock_expiration": 300})";
  httplib::Response res1;
  handlers_->handle_acquire_lock(req1, res1);
  ASSERT_EQ(res1.status, 201);

  // Break and acquire by different client
  auto req2 = make_component_locks_request("ecu1");
  req2.set_header("X-Client-Id", "client_b");
  req2.body = R"({"lock_expiration": 600, "break_lock": true})";
  httplib::Response res2;
  handlers_->handle_acquire_lock(req2, res2);

  EXPECT_EQ(res2.status, 201);
  auto body = json::parse(res2.body);
  EXPECT_TRUE(body["owned"].get<bool>());
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
