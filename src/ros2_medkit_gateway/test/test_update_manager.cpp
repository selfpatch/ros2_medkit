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
#include <thread>

#include "ros2_medkit_gateway/updates/update_manager.hpp"

using namespace ros2_medkit_gateway;
using json = nlohmann::json;

/// Mock backend for unit testing
class MockUpdateBackend : public UpdateBackend {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo> list_updates(const UpdateFilter &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> ids;
    for (const auto & [id, _] : packages_) {
      ids.push_back(id);
    }
    return ids;
  }

  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found"});
    }
    return it->second;
  }

  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & metadata) override {
    auto id = metadata.value("id", std::string{});
    if (id.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing id"});
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (packages_.count(id)) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::AlreadyExists, "already exists"});
    }
    packages_[id] = metadata;
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (packages_.erase(id) == 0) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found"});
    }
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string &, UpdateProgressReporter & reporter) override {
    reporter.set_progress(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reporter.set_progress(100);
    return {};
  }

  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string &, UpdateProgressReporter & reporter) override {
    reporter.set_progress(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return {};
  }

  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::NotFound, "not found"});
    }
    return it->second.value("automated", false);
  }

 private:
  std::mutex mutex_;
  std::unordered_map<std::string, json> packages_;
};

class UpdateManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    auto backend = std::make_unique<MockUpdateBackend>();
    manager_ = std::make_unique<UpdateManager>(std::move(backend));
  }

  std::unique_ptr<UpdateManager> manager_;
};

// @verifies REQ_INTEROP_082
TEST_F(UpdateManagerTest, HasBackend) {
  EXPECT_TRUE(manager_->has_backend());
}

// @verifies REQ_INTEROP_082
TEST_F(UpdateManagerTest, NoBackendMode) {
  UpdateManager no_backend(nullptr);
  EXPECT_FALSE(no_backend.has_backend());
  auto result = no_backend.list_updates({});
  EXPECT_FALSE(result.has_value());
}

// @verifies REQ_INTEROP_082
TEST_F(UpdateManagerTest, RegisterAndList) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  auto reg = manager_->register_update(pkg);
  EXPECT_TRUE(reg.has_value());

  auto list = manager_->list_updates({});
  ASSERT_TRUE(list.has_value());
  EXPECT_EQ(list->size(), 1u);
  EXPECT_EQ((*list)[0], "test-pkg");
}

// @verifies REQ_INTEROP_085
TEST_F(UpdateManagerTest, GetUpdate) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto result = manager_->get_update("test-pkg");
  ASSERT_TRUE(result.has_value());
  EXPECT_EQ((*result)["id"], "test-pkg");
}

// @verifies REQ_INTEROP_085
TEST_F(UpdateManagerTest, GetUpdateNotFound) {
  auto result = manager_->get_update("nonexistent");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, UpdateErrorCode::NotFound);
}

// @verifies REQ_INTEROP_084
TEST_F(UpdateManagerTest, DeleteUpdate) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto del = manager_->delete_update("test-pkg");
  EXPECT_TRUE(del.has_value());

  auto get = manager_->get_update("test-pkg");
  EXPECT_FALSE(get.has_value());
}

// @verifies REQ_INTEROP_091
TEST_F(UpdateManagerTest, PrepareAndPollStatus) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto prep = manager_->start_prepare("test-pkg");
  ASSERT_TRUE(prep.has_value());

  // Poll until completed
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager_->get_status("test-pkg");
    if (s) {
      status = *s;
      if (status.status == UpdateStatus::Completed) {
        found = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for status Completed";
}

// @verifies REQ_INTEROP_092
TEST_F(UpdateManagerTest, ExecuteRequiresPrepare) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto exec = manager_->start_execute("test-pkg");
  EXPECT_FALSE(exec.has_value());
  EXPECT_EQ(exec.error().code, UpdateErrorCode::NotPrepared);
}

// @verifies REQ_INTEROP_092
TEST_F(UpdateManagerTest, ExecuteAfterPrepare) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  (void)manager_->start_prepare("test-pkg");
  // Wait for prepare to complete
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    bool found = false;
    while (std::chrono::steady_clock::now() < deadline) {
      auto s = manager_->get_status("test-pkg");
      if (s && s->status == UpdateStatus::Completed) {
        found = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(found) << "Timed out waiting for prepare to complete";
  }

  auto exec = manager_->start_execute("test-pkg");
  ASSERT_TRUE(exec.has_value());

  // Wait for execute to complete
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    UpdateStatusInfo status;
    bool found = false;
    while (std::chrono::steady_clock::now() < deadline) {
      auto s = manager_->get_status("test-pkg");
      ASSERT_TRUE(s.has_value());
      status = *s;
      if (status.status == UpdateStatus::Completed) {
        found = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(found) << "Timed out waiting for execute to complete";
    EXPECT_EQ(status.status, UpdateStatus::Completed);
  }
}

// @verifies REQ_INTEROP_093
TEST_F(UpdateManagerTest, AutomatedCompletes) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", true}};
  (void)manager_->register_update(pkg);

  auto result = manager_->start_automated("test-pkg");
  ASSERT_TRUE(result.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager_->get_status("test-pkg");
    ASSERT_TRUE(s.has_value());
    status = *s;
    if (status.status == UpdateStatus::Completed) {
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for automated update to complete";
  EXPECT_EQ(status.status, UpdateStatus::Completed);
}

// @verifies REQ_INTEROP_093
TEST_F(UpdateManagerTest, AutomatedRejectsNonAutomated) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto result = manager_->start_automated("test-pkg");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, UpdateErrorCode::NotAutomated);
}

// @verifies REQ_INTEROP_084
TEST_F(UpdateManagerTest, DeleteDuringOperationFails) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  (void)manager_->start_prepare("test-pkg");

  auto del = manager_->delete_update("test-pkg");
  EXPECT_FALSE(del.has_value());
  EXPECT_EQ(del.error().code, UpdateErrorCode::InProgress);
}

// @verifies REQ_INTEROP_094
TEST_F(UpdateManagerTest, StatusNotFoundForUnknown) {
  auto result = manager_->get_status("unknown");
  EXPECT_FALSE(result.has_value());
  EXPECT_EQ(result.error().code, UpdateErrorCode::NotFound);
}

// @verifies REQ_INTEROP_083
TEST_F(UpdateManagerTest, DuplicateRegistration) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  auto first = manager_->register_update(pkg);
  EXPECT_TRUE(first.has_value());

  auto second = manager_->register_update(pkg);
  EXPECT_FALSE(second.has_value());
  EXPECT_EQ(second.error().code, UpdateErrorCode::AlreadyExists);
}

// @verifies REQ_INTEROP_091
TEST_F(UpdateManagerTest, ConcurrentPrepareOnSamePackageRejected) {
  json pkg = {{"id", "test-pkg"}};
  (void)manager_->register_update(pkg);

  auto result1 = manager_->start_prepare("test-pkg");
  EXPECT_TRUE(result1.has_value());

  auto result2 = manager_->start_prepare("test-pkg");
  EXPECT_FALSE(result2.has_value());
  EXPECT_EQ(result2.error().code, UpdateErrorCode::InProgress);
}

/// Mock backend that returns errors from prepare/execute
class MockFailingBackend : public UpdateBackend {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "backend error"});
  }
  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return json{{"id", "pkg"}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & metadata) override {
    auto id = metadata.value("id", std::string{});
    if (id.empty()) {
      return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::InvalidInput, "missing id"});
    }
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "download failed"});
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "install failed"});
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

/// Mock backend that throws exceptions from prepare/execute
class MockThrowingBackend : public UpdateBackend {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return json{{"id", "pkg"}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    throw std::runtime_error("plugin crashed");
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    throw std::runtime_error("plugin crashed");
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

// @verifies REQ_INTEROP_091
TEST(UpdateManagerFailureTest, PrepareFailureSetsFailedStatus) {
  auto backend = std::make_unique<MockFailingBackend>();
  auto manager = std::make_unique<UpdateManager>(std::move(backend));
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  auto result = manager->start_prepare("test-pkg");
  ASSERT_TRUE(result.has_value());

  // Poll until status is no longer InProgress
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("download failed"), std::string::npos);
}

/// Mock backend with working prepare but failing execute
class MockExecuteFailingBackend : public UpdateBackend {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return json{{"id", "pkg"}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & reporter) override {
    reporter.set_progress(100);
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    return tl::make_unexpected(UpdateBackendErrorInfo{UpdateBackendError::Internal, "install failed"});
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

/// Mock backend with working prepare but throwing execute
class MockExecuteThrowingBackend : public UpdateBackend {
 public:
  tl::expected<std::vector<std::string>, UpdateBackendErrorInfo>
  list_updates(const UpdateFilter & /*filter*/) override {
    return std::vector<std::string>{};
  }
  tl::expected<json, UpdateBackendErrorInfo> get_update(const std::string & /*id*/) override {
    return json{{"id", "pkg"}};
  }
  tl::expected<void, UpdateBackendErrorInfo> register_update(const json & /*metadata*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> delete_update(const std::string & /*id*/) override {
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> prepare(const std::string & /*id*/,
                                                     UpdateProgressReporter & reporter) override {
    reporter.set_progress(100);
    return {};
  }
  tl::expected<void, UpdateBackendErrorInfo> execute(const std::string & /*id*/,
                                                     UpdateProgressReporter & /*reporter*/) override {
    throw std::runtime_error("plugin crashed during install");
  }
  tl::expected<bool, UpdateBackendErrorInfo> supports_automated(const std::string & /*id*/) override {
    return true;
  }
};

// @verifies REQ_INTEROP_091
TEST(UpdateManagerFailureTest, PrepareExceptionSetsFailedStatus) {
  auto backend = std::make_unique<MockThrowingBackend>();
  auto manager = std::make_unique<UpdateManager>(std::move(backend));
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  auto result = manager->start_prepare("test-pkg");
  ASSERT_TRUE(result.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("Exception"), std::string::npos);
}

// Helper: prepare a package and wait for completion
static bool prepare_and_wait(UpdateManager & manager, const std::string & id) {
  auto prep = manager.start_prepare(id);
  if (!prep) {
    return false;
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager.get_status(id);
    if (s && s->status == UpdateStatus::Completed) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

// @verifies REQ_INTEROP_092
TEST(UpdateManagerFailureTest, ExecuteFailureSetsFailedStatus) {
  auto backend = std::make_unique<MockExecuteFailingBackend>();
  auto manager = std::make_unique<UpdateManager>(std::move(backend));
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  ASSERT_TRUE(prepare_and_wait(*manager, "test-pkg")) << "Timed out waiting for prepare to complete";

  auto exec = manager->start_execute("test-pkg");
  ASSERT_TRUE(exec.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for execute to fail";
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("install failed"), std::string::npos);
}

// @verifies REQ_INTEROP_092
TEST(UpdateManagerFailureTest, ExecuteExceptionSetsFailedStatus) {
  auto backend = std::make_unique<MockExecuteThrowingBackend>();
  auto manager = std::make_unique<UpdateManager>(std::move(backend));
  json pkg = {{"id", "test-pkg"}};
  (void)manager->register_update(pkg);

  ASSERT_TRUE(prepare_and_wait(*manager, "test-pkg")) << "Timed out waiting for prepare to complete";

  auto exec = manager->start_execute("test-pkg");
  ASSERT_TRUE(exec.has_value());

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  UpdateStatusInfo status;
  bool found = false;
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = manager->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Failed) {
      status = *s;
      found = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_TRUE(found) << "Timed out waiting for execute exception to be caught";
  EXPECT_EQ(status.status, UpdateStatus::Failed);
  ASSERT_TRUE(status.error_message.has_value());
  EXPECT_NE(status.error_message->find("Exception"), std::string::npos);
}
