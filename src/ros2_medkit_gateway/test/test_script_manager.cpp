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

#include "ros2_medkit_gateway/providers/script_provider.hpp"
#include "ros2_medkit_gateway/script_manager.hpp"

using namespace ros2_medkit_gateway;

class MockScriptProvider : public ScriptProvider {
 public:
  tl::expected<std::vector<ScriptInfo>, ScriptBackendErrorInfo>
  list_scripts(const std::string & /*entity_id*/) override {
    return std::vector<ScriptInfo>{{"script_001", "Test Script", "A test script", false, false, std::nullopt}};
  }

  tl::expected<ScriptInfo, ScriptBackendErrorInfo> get_script(const std::string & /*entity_id*/,
                                                              const std::string & script_id) override {
    if (script_id == "script_001") {
      return ScriptInfo{"script_001", "Test Script", "A test script", false, false, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Script not found"});
  }

  tl::expected<ScriptUploadResult, ScriptBackendErrorInfo>
  upload_script(const std::string & /*entity_id*/, const std::string & /*filename*/, const std::string & /*content*/,
                const std::optional<nlohmann::json> & /*metadata*/) override {
    return ScriptUploadResult{"script_002", "Uploaded Script"};
  }

  tl::expected<void, ScriptBackendErrorInfo> delete_script(const std::string & /*entity_id*/,
                                                           const std::string & script_id) override {
    if (script_id == "manifest_script") {
      return tl::make_unexpected(
          ScriptBackendErrorInfo{ScriptBackendError::ManagedScript, "Cannot delete manifest script"});
    }
    return {};
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> start_execution(const std::string & /*entity_id*/,
                                                                      const std::string & /*script_id*/,
                                                                      const ExecutionRequest & /*request*/) override {
    return ExecutionInfo{"exec_001", "prepared", std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt};
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> get_execution(const std::string & /*entity_id*/,
                                                                    const std::string & /*script_id*/,
                                                                    const std::string & execution_id) override {
    if (execution_id == "exec_001") {
      return ExecutionInfo{"exec_001", "running", 45, "2026-03-16T10:00:00Z", std::nullopt, std::nullopt, std::nullopt};
    }
    return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Execution not found"});
  }

  tl::expected<ExecutionInfo, ScriptBackendErrorInfo> control_execution(const std::string & /*entity_id*/,
                                                                        const std::string & /*script_id*/,
                                                                        const std::string & execution_id,
                                                                        const std::string & action) override {
    if (execution_id != "exec_001") {
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::NotFound, "Execution not found"});
    }
    if (action != "stop" && action != "forced_termination") {
      return tl::make_unexpected(ScriptBackendErrorInfo{ScriptBackendError::InvalidInput, "Unknown action"});
    }
    return ExecutionInfo{"exec_001",   "terminated", std::nullopt, "2026-03-16T10:00:00Z", "2026-03-16T10:01:00Z",
                         std::nullopt, std::nullopt};
  }

  tl::expected<void, ScriptBackendErrorInfo> delete_execution(const std::string & /*entity_id*/,
                                                              const std::string & /*script_id*/,
                                                              const std::string & execution_id) override {
    if (execution_id == "running_exec") {
      return tl::make_unexpected(
          ScriptBackendErrorInfo{ScriptBackendError::AlreadyRunning, "Cannot delete running execution"});
    }
    return {};
  }
};

class ScriptManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    backend_ = std::make_unique<MockScriptProvider>();
    backend_ptr_ = backend_.get();
    manager_ = std::make_unique<ScriptManager>();
    manager_->set_backend(backend_ptr_);
  }

  std::unique_ptr<MockScriptProvider> backend_;
  MockScriptProvider * backend_ptr_ = nullptr;
  std::unique_ptr<ScriptManager> manager_;
};

// @verifies REQ_INTEROP_090
TEST_F(ScriptManagerTest, HasBackend) {
  EXPECT_TRUE(manager_->has_backend());
}

TEST_F(ScriptManagerTest, NoBackend) {
  ScriptManager no_backend;
  EXPECT_FALSE(no_backend.has_backend());
}

TEST_F(ScriptManagerTest, ListScripts) {
  auto result = manager_->list_scripts("comp1");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->size(), 1u);
  EXPECT_EQ((*result)[0].id, "script_001");
}

TEST_F(ScriptManagerTest, GetScript) {
  auto result = manager_->get_script("comp1", "script_001");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "script_001");
}

TEST_F(ScriptManagerTest, GetScriptNotFound) {
  auto result = manager_->get_script("comp1", "nonexistent");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::NotFound);
}

TEST_F(ScriptManagerTest, UploadScript) {
  auto result = manager_->upload_script("comp1", "test.py", "#!/usr/bin/env python3", std::nullopt);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "script_002");
}

TEST_F(ScriptManagerTest, DeleteScript) {
  auto result = manager_->delete_script("comp1", "script_001");
  EXPECT_TRUE(result.has_value());
}

TEST_F(ScriptManagerTest, DeleteManifestScript) {
  auto result = manager_->delete_script("comp1", "manifest_script");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::ManagedScript);
}

TEST_F(ScriptManagerTest, StartExecution) {
  ExecutionRequest req{"now", std::nullopt, std::nullopt};
  auto result = manager_->start_execution("comp1", "script_001", req);
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->id, "exec_001");
  EXPECT_EQ(result->status, "prepared");
}

TEST_F(ScriptManagerTest, GetExecution) {
  auto result = manager_->get_execution("comp1", "script_001", "exec_001");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "running");
  EXPECT_EQ(result->progress, 45);
}

TEST_F(ScriptManagerTest, ControlExecution) {
  auto result = manager_->control_execution("comp1", "script_001", "exec_001", "stop");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ(result->status, "terminated");
}

TEST_F(ScriptManagerTest, DeleteExecution) {
  auto result = manager_->delete_execution("comp1", "script_001", "exec_001");
  EXPECT_TRUE(result.has_value());
}

TEST_F(ScriptManagerTest, DeleteRunningExecution) {
  auto result = manager_->delete_execution("comp1", "script_001", "running_exec");
  ASSERT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, ScriptBackendError::AlreadyRunning);
}
