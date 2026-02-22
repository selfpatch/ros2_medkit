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
  tl::expected<std::vector<std::string>, std::string> list_updates(const UpdateFilter &) override {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> ids;
    for (const auto & [id, _] : packages_) {
      ids.push_back(id);
    }
    return ids;
  }

  tl::expected<json, std::string> get_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected("not found");
    }
    return it->second;
  }

  tl::expected<void, std::string> register_update(const json & metadata) override {
    auto id = metadata.value("id", std::string{});
    if (id.empty()) {
      return tl::make_unexpected("missing id");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (packages_.count(id)) {
      return tl::make_unexpected("already exists");
    }
    packages_[id] = metadata;
    return {};
  }

  tl::expected<void, std::string> delete_update(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    if (packages_.erase(id) == 0) {
      return tl::make_unexpected("not found");
    }
    return {};
  }

  tl::expected<void, std::string> prepare(const std::string &, UpdateProgressReporter & reporter) override {
    reporter.set_progress(50);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    reporter.set_progress(100);
    return {};
  }

  tl::expected<void, std::string> execute(const std::string &, UpdateProgressReporter & reporter) override {
    reporter.set_progress(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return {};
  }

  tl::expected<bool, std::string> supports_automated(const std::string & id) override {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = packages_.find(id);
    if (it == packages_.end()) {
      return tl::make_unexpected("not found");
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
  UpdateStatusInfo status;
  for (int i = 0; i < 100; ++i) {
    auto s = manager_->get_status("test-pkg");
    ASSERT_TRUE(s.has_value());
    status = *s;
    if (status.status == UpdateStatus::Completed) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(status.status, UpdateStatus::Completed);
}

// @verifies REQ_INTEROP_092
TEST_F(UpdateManagerTest, ExecuteRequiresPrepare) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto exec = manager_->start_execute("test-pkg");
  EXPECT_FALSE(exec.has_value());
  EXPECT_NE(exec.error().find("must be prepared"), std::string::npos);
}

// @verifies REQ_INTEROP_092
TEST_F(UpdateManagerTest, ExecuteAfterPrepare) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  (void)manager_->start_prepare("test-pkg");
  // Wait for prepare to complete
  for (int i = 0; i < 100; ++i) {
    auto s = manager_->get_status("test-pkg");
    if (s && s->status == UpdateStatus::Completed) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  auto exec = manager_->start_execute("test-pkg");
  ASSERT_TRUE(exec.has_value());

  // Wait for execute to complete
  UpdateStatusInfo status;
  for (int i = 0; i < 100; ++i) {
    auto s = manager_->get_status("test-pkg");
    ASSERT_TRUE(s.has_value());
    status = *s;
    if (status.status == UpdateStatus::Completed) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(status.status, UpdateStatus::Completed);
}

// @verifies REQ_INTEROP_093
TEST_F(UpdateManagerTest, AutomatedCompletes) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", true}};
  (void)manager_->register_update(pkg);

  auto result = manager_->start_automated("test-pkg");
  ASSERT_TRUE(result.has_value());

  UpdateStatusInfo status;
  for (int i = 0; i < 200; ++i) {
    auto s = manager_->get_status("test-pkg");
    ASSERT_TRUE(s.has_value());
    status = *s;
    if (status.status == UpdateStatus::Completed) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_EQ(status.status, UpdateStatus::Completed);
}

// @verifies REQ_INTEROP_093
TEST_F(UpdateManagerTest, AutomatedRejectsNonAutomated) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  auto result = manager_->start_automated("test-pkg");
  EXPECT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("does not support"), std::string::npos);
}

// @verifies REQ_INTEROP_084
TEST_F(UpdateManagerTest, DeleteDuringOperationFails) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  (void)manager_->register_update(pkg);

  (void)manager_->start_prepare("test-pkg");

  auto del = manager_->delete_update("test-pkg");
  EXPECT_FALSE(del.has_value());
  EXPECT_NE(del.error().find("in progress"), std::string::npos);
}

// @verifies REQ_INTEROP_094
TEST_F(UpdateManagerTest, StatusNotFoundForUnknown) {
  auto result = manager_->get_status("unknown");
  EXPECT_FALSE(result.has_value());
}

// @verifies REQ_INTEROP_083
TEST_F(UpdateManagerTest, DuplicateRegistration) {
  json pkg = {{"id", "test-pkg"}, {"update_name", "Test"}, {"automated", false}};
  auto first = manager_->register_update(pkg);
  EXPECT_TRUE(first.has_value());

  auto second = manager_->register_update(pkg);
  EXPECT_FALSE(second.has_value());
}
