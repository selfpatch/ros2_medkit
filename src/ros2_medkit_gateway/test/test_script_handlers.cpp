// Copyright 2026 Bartlomiej Burda
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

#include <cstring>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <string>

#include "ros2_medkit_gateway/core/http/error_codes.hpp"
#include "ros2_medkit_gateway/core/http/handlers/script_handlers.hpp"
#include "ros2_medkit_gateway/gateway_node.hpp"

using json = nlohmann::json;
using ros2_medkit_gateway::AuthConfig;
using ros2_medkit_gateway::CorsConfig;
using ros2_medkit_gateway::EntityType;
using ros2_medkit_gateway::ExecutionInfo;
using ros2_medkit_gateway::ExecutionRequest;
using ros2_medkit_gateway::GatewayNode;
using ros2_medkit_gateway::ScriptBackendError;
using ros2_medkit_gateway::ScriptBackendErrorInfo;
using ros2_medkit_gateway::ScriptInfo;
using ros2_medkit_gateway::ScriptManager;
using ros2_medkit_gateway::ScriptProvider;
using ros2_medkit_gateway::ScriptUploadResult;
using ros2_medkit_gateway::ThreadSafeEntityCache;
using ros2_medkit_gateway::TlsConfig;
using ros2_medkit_gateway::handlers::HandlerContext;
using ros2_medkit_gateway::handlers::ScriptHandlers;

namespace {

// =============================================================================
// Mock script provider that returns configurable errors
// =============================================================================

class ErrorMockScriptProvider : public ScriptProvider {
 public:
  // The error to return from all operations (set per-test)
  ScriptBackendError error_code{ScriptBackendError::Internal};
  std::string error_message{"mock error"};

  // When true, operations succeed instead of returning errors
  bool succeed{false};

  tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo>
  list_scripts(const std::string & /*entity_id*/) override {
    if (succeed) {
      return std::vector<ScriptInfo>{};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & /*entity_id*/,
                                                              const std::string & /*script_id*/) override {
    if (succeed) {
      return ScriptInfo{"s1", "Script", "desc", false, false, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & /*entity_id*/, const std::string & /*filename*/, const std::string & /*content*/,
                const std::optional<json> & /*metadata*/) override {
    if (succeed) {
      return ScriptUploadResult{"uploaded_001", "Uploaded Script"};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & /*entity_id*/,
                                                           const std::string & /*script_id*/) override {
    if (succeed) {
      return {};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> start_execution(const std::string & /*entity_id*/,
                                                                      const std::string & /*script_id*/,
                                                                      const ExecutionRequest & /*request*/) override {
    if (succeed) {
      return ExecutionInfo{"exec_001",   "running",    std::nullopt, "2026-03-18T10:00:00Z",
                           std::nullopt, std::nullopt, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> get_execution(const std::string & /*entity_id*/,
                                                                    const std::string & /*script_id*/,
                                                                    const std::string & /*execution_id*/) override {
    if (succeed) {
      return ExecutionInfo{"exec_001", "running", 50, "2026-03-18T10:00:00Z", std::nullopt, std::nullopt, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & /*entity_id*/,
                                                                        const std::string & /*script_id*/,
                                                                        const std::string & /*execution_id*/,
                                                                        const std::string & /*action*/) override {
    if (succeed) {
      return ExecutionInfo{"exec_001",   "terminated", std::nullopt, "2026-03-18T10:00:00Z", "2026-03-18T10:01:00Z",
                           std::nullopt, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }

  tl::expected<void, ScriptBackendErrorInfo> delete_execution(const std::string & /*entity_id*/,
                                                              const std::string & /*script_id*/,
                                                              const std::string & /*execution_id*/) override {
    if (succeed) {
      return {};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{error_code, error_message});
  }
};

// =============================================================================
// Helpers
// =============================================================================

// Minimal manifest that provides a component and app with SCRIPTS support
const char * kScriptTestManifest = R"(
manifest_version: "1.0"
metadata:
  name: "script-handlers-test"
  version: "1.0.0"
components:
  - id: "ecu"
    name: "ECU"
    namespace: "/vehicle"
apps:
  - id: "planner"
    name: "Planner"
    is_located_on: "ecu"
)";

std::string write_temp_manifest(const std::string & contents) {
  char path_template[] = "/tmp/ros2_medkit_script_handlers_XXXXXX.yaml";
  int fd = mkstemps(path_template, 5);
  if (fd < 0) {
    ADD_FAILURE() << "Failed to create temp manifest: " << std::strerror(errno);
    return {};
  }
  close(fd);

  std::ofstream out(path_template);
  if (!out) {
    ADD_FAILURE() << "Failed to open temp manifest for writing";
    std::remove(path_template);
    return {};
  }
  out << contents;
  out.close();
  return path_template;
}

httplib::Request make_list_request(const std::string & entity_type, const std::string & entity_id) {
  httplib::Request req;
  req.path = "/api/v1/" + entity_type + "/" + entity_id + "/scripts";
  std::string pattern = "/api/v1/" + entity_type + "/([^/]+)/scripts";
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

httplib::Request make_script_request(const std::string & entity_type, const std::string & entity_id,
                                     const std::string & script_id) {
  httplib::Request req;
  req.path = "/api/v1/" + entity_type + "/" + entity_id + "/scripts/" + script_id;
  std::string pattern = "/api/v1/" + entity_type + "/([^/]+)/scripts/([^/]+)";
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

httplib::Request make_execution_request(const std::string & entity_type, const std::string & entity_id,
                                        const std::string & script_id, const std::string & execution_id) {
  httplib::Request req;
  req.path = "/api/v1/" + entity_type + "/" + entity_id + "/scripts/" + script_id + "/executions/" + execution_id;
  std::string pattern = "/api/v1/" + entity_type + "/([^/]+)/scripts/([^/]+)/executions/([^/]+)";
  std::regex re(pattern);
  std::regex_match(req.path, req.matches, re);
  return req;
}

}  // namespace

// =============================================================================
// Test fixture - no backend (tests 501 responses with null HandlerContext)
// =============================================================================

class ScriptHandlersNoBackendTest : public ::testing::Test {
 protected:
  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  HandlerContext ctx_{nullptr, cors_, auth_, tls_, nullptr};
  // Null script manager -> check_backend returns 501
  ScriptHandlers handlers_{ctx_, nullptr};
};

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersNoBackendTest, UploadReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_upload_script(req, res);
  EXPECT_EQ(res.status, 501);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_NOT_IMPLEMENTED);
}

// @verifies REQ_INTEROP_041
TEST_F(ScriptHandlersNoBackendTest, ListReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_list_scripts(req, res);
  EXPECT_EQ(res.status, 501);
}

// @verifies REQ_INTEROP_042
TEST_F(ScriptHandlersNoBackendTest, GetReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_script(req, res);
  EXPECT_EQ(res.status, 501);
}

// @verifies REQ_INTEROP_043
TEST_F(ScriptHandlersNoBackendTest, DeleteReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_delete_script(req, res);
  EXPECT_EQ(res.status, 501);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersNoBackendTest, StartExecutionReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_start_execution(req, res);
  EXPECT_EQ(res.status, 501);
}

// @verifies REQ_INTEROP_046
TEST_F(ScriptHandlersNoBackendTest, GetExecutionReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_get_execution(req, res);
  EXPECT_EQ(res.status, 501);
}

// @verifies REQ_INTEROP_047
TEST_F(ScriptHandlersNoBackendTest, ControlExecutionReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_control_execution(req, res);
  EXPECT_EQ(res.status, 501);
}

TEST_F(ScriptHandlersNoBackendTest, DeleteExecutionReturns501WhenNoBackend) {
  httplib::Request req;
  httplib::Response res;
  handlers_.handle_delete_execution(req, res);
  EXPECT_EQ(res.status, 501);
}

// Also test with a ScriptManager that has no backend set
TEST_F(ScriptHandlersNoBackendTest, ScriptManagerWithoutBackendReturns501) {
  ScriptManager manager;  // has_backend() returns false
  ScriptHandlers handlers(ctx_, &manager);

  httplib::Request req;
  httplib::Response res;
  handlers.handle_list_scripts(req, res);
  EXPECT_EQ(res.status, 501);
}

// =============================================================================
// Test fixture - full setup with GatewayNode, manifest entities, mock provider
// =============================================================================

class ScriptHandlersErrorMappingTest : public ::testing::Test {
 protected:
  static inline std::shared_ptr<GatewayNode> node_;

  static void SetUpTestSuite() {
    std::vector<std::string> args = {"test_script_handlers", "--ros-args", "-p",
                                     "server.port:=0",       "-p",         "refresh_interval_ms:=60000"};
    std::vector<char *> argv;
    argv.reserve(args.size());
    for (auto & arg : args) {
      argv.push_back(arg.data());
    }
    rclcpp::init(static_cast<int>(argv.size()), argv.data());
    node_ = std::make_shared<GatewayNode>();
  }

  static void TearDownTestSuite() {
    node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override {
    manifest_path_ = write_temp_manifest(kScriptTestManifest);
    ASSERT_FALSE(manifest_path_.empty());

    ros2_medkit_gateway::DiscoveryConfig config;
    config.mode = ros2_medkit_gateway::DiscoveryMode::MANIFEST_ONLY;
    config.manifest_path = manifest_path_;
    config.manifest_strict_validation = false;

    ASSERT_TRUE(node_->get_discovery_manager()->initialize(config));

    auto areas = node_->get_discovery_manager()->discover_areas();
    auto components = node_->get_discovery_manager()->discover_components();
    auto apps = node_->get_discovery_manager()->discover_apps();
    auto functions = node_->get_discovery_manager()->discover_functions();

    auto & cache = const_cast<ThreadSafeEntityCache &>(node_->get_thread_safe_cache());
    cache.update_all(areas, components, apps, functions);

    ctx_ = std::make_unique<HandlerContext>(node_.get(), cors_, auth_, tls_, nullptr);

    mock_provider_ = std::make_unique<ErrorMockScriptProvider>();
    script_manager_ = std::make_unique<ScriptManager>();
    script_manager_->set_backend(mock_provider_.get());

    handlers_ = std::make_unique<ScriptHandlers>(*ctx_, script_manager_.get());
  }

  void TearDown() override {
    handlers_.reset();
    script_manager_.reset();
    mock_provider_.reset();
    ctx_.reset();
    if (!manifest_path_.empty()) {
      std::remove(manifest_path_.c_str());
    }
  }

  /// Helper: call handle_get_script with entity "ecu" and trigger the mock error
  void call_get_script_with_error(ScriptBackendError err, httplib::Response & res) {
    mock_provider_->succeed = false;
    mock_provider_->error_code = err;
    mock_provider_->error_message = "test error";

    auto req = make_script_request("components", "ecu", "test_script");
    handlers_->handle_get_script(req, res);
  }

  /// Helper: call handle_delete_script with entity "ecu" and trigger the mock error
  void call_delete_script_with_error(ScriptBackendError err, httplib::Response & res) {
    mock_provider_->succeed = false;
    mock_provider_->error_code = err;
    mock_provider_->error_message = "test error";

    auto req = make_script_request("components", "ecu", "test_script");
    handlers_->handle_delete_script(req, res);
  }

  /// Helper: call handle_start_execution with entity "ecu" and trigger the mock error
  void call_start_execution_with_error(ScriptBackendError err, httplib::Response & res) {
    mock_provider_->succeed = false;
    mock_provider_->error_code = err;
    mock_provider_->error_message = "test error";

    auto req = make_script_request("components", "ecu", "test_script");
    req.body = R"({"execution_type": "now"})";
    handlers_->handle_start_execution(req, res);
  }

  CorsConfig cors_{};
  AuthConfig auth_{};
  TlsConfig tls_{};
  std::unique_ptr<HandlerContext> ctx_;
  std::unique_ptr<ErrorMockScriptProvider> mock_provider_;
  std::unique_ptr<ScriptManager> script_manager_;
  std::unique_ptr<ScriptHandlers> handlers_;
  std::string manifest_path_;
};

// =============================================================================
// Error mapping: ScriptBackendError -> HTTP status code
// =============================================================================

// @verifies REQ_INTEROP_042
TEST_F(ScriptHandlersErrorMappingTest, NotFoundMapsTo404) {
  httplib::Response res;
  call_get_script_with_error(ScriptBackendError::NotFound, res);
  EXPECT_EQ(res.status, 404);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_RESOURCE_NOT_FOUND);
}

// @verifies REQ_INTEROP_043
TEST_F(ScriptHandlersErrorMappingTest, ManagedScriptMapsTo409) {
  httplib::Response res;
  call_delete_script_with_error(ScriptBackendError::ManagedScript, res);
  EXPECT_EQ(res.status, 409);
  auto body = json::parse(res.body);
  // Vendor-specific error codes are wrapped
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], ros2_medkit_gateway::ERR_SCRIPT_MANAGED);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersErrorMappingTest, AlreadyRunningMapsTo409) {
  httplib::Response res;
  call_delete_script_with_error(ScriptBackendError::AlreadyRunning, res);
  EXPECT_EQ(res.status, 409);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], ros2_medkit_gateway::ERR_SCRIPT_RUNNING);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersErrorMappingTest, ConcurrencyLimitMapsTo429) {
  httplib::Response res;
  call_start_execution_with_error(ScriptBackendError::ConcurrencyLimit, res);
  EXPECT_EQ(res.status, 429);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], ros2_medkit_gateway::ERR_SCRIPT_CONCURRENCY_LIMIT);
}

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersErrorMappingTest, FileTooLargeMapsTo413) {
  // Upload handler needs multipart file field - use get_script with FileTooLarge instead
  // since send_script_error is shared across all handlers
  httplib::Response res;
  call_get_script_with_error(ScriptBackendError::FileTooLarge, res);
  EXPECT_EQ(res.status, 413);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], ros2_medkit_gateway::ERR_SCRIPT_FILE_TOO_LARGE);
}

TEST_F(ScriptHandlersErrorMappingTest, InternalErrorMapsTo500) {
  httplib::Response res;
  call_get_script_with_error(ScriptBackendError::Internal, res);
  EXPECT_EQ(res.status, 500);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INTERNAL_ERROR);
}

// =============================================================================
// Success paths: upload -> 201, start -> 202, delete -> 204
// =============================================================================

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersErrorMappingTest, UploadReturns201WithLocation) {
  mock_provider_->succeed = true;

  auto req = make_list_request("components", "ecu");
  req.set_header("Content-Type", "multipart/form-data; boundary=----WebKitFormBoundary");
  // Upload needs multipart file data
  httplib::MultipartFormData file_part;
  file_part.name = "file";
  file_part.filename = "diag.py";
  file_part.content = "#!/usr/bin/env python3\nprint('hello')";
  file_part.content_type = "application/octet-stream";
  req.files.emplace("file", file_part);

  httplib::Response res;
  handlers_->handle_upload_script(req, res);

  EXPECT_EQ(res.status, 201);
  EXPECT_FALSE(res.get_header_value("Location").empty());
  EXPECT_NE(res.get_header_value("Location").find("/scripts/uploaded_001"), std::string::npos);

  auto body = json::parse(res.body);
  EXPECT_EQ(body["id"], "uploaded_001");
  EXPECT_EQ(body["name"], "Uploaded Script");
}

// @verifies REQ_INTEROP_040
TEST_F(ScriptHandlersErrorMappingTest, UploadRejectsWrongContentType) {
  mock_provider_->succeed = true;

  auto req = make_list_request("components", "ecu");
  // No Content-Type header set - should be rejected
  httplib::MultipartFormData file_part;
  file_part.name = "file";
  file_part.filename = "diag.py";
  file_part.content = "#!/usr/bin/env python3\nprint('hello')";
  file_part.content_type = "application/octet-stream";
  req.files.emplace("file", file_part);

  httplib::Response res;
  handlers_->handle_upload_script(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

// @verifies REQ_INTEROP_044
TEST_F(ScriptHandlersErrorMappingTest, StartExecutionReturns202WithLocation) {
  mock_provider_->succeed = true;

  auto req = make_script_request("components", "ecu", "test_script");
  req.body = R"({"execution_type": "now"})";

  httplib::Response res;
  handlers_->handle_start_execution(req, res);

  EXPECT_EQ(res.status, 202);
  EXPECT_FALSE(res.get_header_value("Location").empty());
  EXPECT_NE(res.get_header_value("Location").find("/executions/exec_001"), std::string::npos);

  auto body = json::parse(res.body);
  EXPECT_EQ(body["id"], "exec_001");
  EXPECT_EQ(body["status"], "running");
}

// @verifies REQ_INTEROP_043
TEST_F(ScriptHandlersErrorMappingTest, DeleteReturns204) {
  mock_provider_->succeed = true;

  auto req = make_script_request("components", "ecu", "test_script");

  httplib::Response res;
  handlers_->handle_delete_script(req, res);

  EXPECT_EQ(res.status, 204);
}

// =============================================================================
// Adversarial input validation for is_valid_resource_id
// =============================================================================

TEST_F(ScriptHandlersErrorMappingTest, PathTraversalScriptIdRejected) {
  // GET /apps/{entity}/scripts/../../../etc/passwd -> 400
  auto req = make_script_request("components", "ecu", "../../../etc/passwd");
  httplib::Response res;
  handlers_->handle_get_script(req, res);
  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

TEST_F(ScriptHandlersErrorMappingTest, ShellMetacharsRejected) {
  auto req = make_script_request("components", "ecu", "test;rm");
  httplib::Response res;
  handlers_->handle_get_script(req, res);
  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

TEST_F(ScriptHandlersErrorMappingTest, OverlongScriptIdRejected) {
  auto req = make_script_request("components", "ecu", std::string(257, 'a'));
  httplib::Response res;
  handlers_->handle_get_script(req, res);
  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_PARAMETER);
}

TEST_F(ScriptHandlersErrorMappingTest, ValidScriptIdWithSpecialCharsAccepted) {
  // Valid ID with underscore and hyphen passes validation (gets 404 from mock provider)
  mock_provider_->succeed = false;
  mock_provider_->error_code = ScriptBackendError::NotFound;
  mock_provider_->error_message = "not found";

  auto req = make_script_request("components", "ecu", "my-script_01");
  httplib::Response res;
  handlers_->handle_get_script(req, res);
  // Should NOT be 400 - it passes validation and reaches the provider (which returns 404)
  EXPECT_EQ(res.status, 404);
}

// =============================================================================
// Control execution validation
// =============================================================================

TEST_F(ScriptHandlersErrorMappingTest, ControlExecutionMissingActionField) {
  mock_provider_->succeed = true;

  auto req = make_execution_request("components", "ecu", "test_script", "exec_001");
  req.body = R"({})";

  httplib::Response res;
  handlers_->handle_control_execution(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
  EXPECT_NE(body["message"].get<std::string>().find("action"), std::string::npos);
}

TEST_F(ScriptHandlersErrorMappingTest, ControlExecutionBogusAction) {
  // The handler validates JSON structure but delegates action validation to the provider.
  // The mock provider succeeds, so this tests that a real provider would reject unknown actions.
  // Use the error mock to simulate InvalidInput from the provider.
  mock_provider_->succeed = false;
  mock_provider_->error_code = ScriptBackendError::InvalidInput;
  mock_provider_->error_message = "Unknown action: bogus";

  auto req = make_execution_request("components", "ecu", "test_script", "exec_001");
  req.body = R"({"action": "bogus"})";

  httplib::Response res;
  handlers_->handle_control_execution(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

TEST_F(ScriptHandlersErrorMappingTest, ControlExecutionInvalidJson) {
  auto req = make_execution_request("components", "ecu", "test_script", "exec_001");
  req.body = "not json";

  httplib::Response res;
  handlers_->handle_control_execution(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

TEST_F(ScriptHandlersErrorMappingTest, ControlExecutionEmptyAction) {
  auto req = make_execution_request("components", "ecu", "test_script", "exec_001");
  req.body = R"({"action": ""})";

  httplib::Response res;
  handlers_->handle_control_execution(req, res);

  EXPECT_EQ(res.status, 400);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], ros2_medkit_gateway::ERR_INVALID_REQUEST);
}

TEST_F(ScriptHandlersErrorMappingTest, NotRunningMapsTo409) {
  httplib::Response res;
  call_get_script_with_error(ScriptBackendError::NotRunning, res);
  EXPECT_EQ(res.status, 409);
  auto body = json::parse(res.body);
  EXPECT_EQ(body["error_code"], "vendor-error");
  EXPECT_EQ(body["vendor_code"], ros2_medkit_gateway::ERR_SCRIPT_NOT_RUNNING);
}
