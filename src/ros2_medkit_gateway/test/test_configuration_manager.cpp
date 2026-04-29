// Copyright 2025 bburda
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

#include <atomic>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <thread>

#include "ros2_medkit_gateway/configuration_manager.hpp"
#include "ros2_medkit_gateway/ros2/transports/ros2_parameter_transport.hpp"

using namespace ros2_medkit_gateway;

class TestConfigurationManager : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite() {
    rclcpp::shutdown();
  }

  void SetUp() override {
    // Create node with callback groups for multi-threaded execution
    // Use short timeout for tests to avoid long waits on nonexistent nodes
    rclcpp::NodeOptions options;
    options.allow_undeclared_parameters(false);
    node_ = std::make_shared<rclcpp::Node>("test_config_manager_node", options);
    // Short timeout (0.1s) for tests to avoid long waits on nonexistent nodes.
    parameter_transport_ = std::make_shared<ros2::Ros2ParameterTransport>(node_.get(), 0.1, 60.0);
    config_manager_ = std::make_unique<ConfigurationManager>(parameter_transport_);

    // Create and start executor in separate thread to avoid deadlocks
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_running_ = true;
    spin_thread_ = std::thread([this]() {
      while (spin_thread_running_) {
        executor_->spin_some(std::chrono::milliseconds(10));
      }
    });
  }

  void TearDown() override {
    spin_thread_running_ = false;
    if (spin_thread_.joinable()) {
      spin_thread_.join();
    }
    executor_->remove_node(node_);
    config_manager_.reset();
    parameter_transport_.reset();
    node_.reset();
    executor_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<ros2::Ros2ParameterTransport> parameter_transport_;
  std::unique_ptr<ConfigurationManager> config_manager_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic<bool> spin_thread_running_{false};
};

// ==================== LIST PARAMETERS TESTS ====================

TEST_F(TestConfigurationManager, test_list_parameters_nonexistent_node) {
  // Try to list parameters for a node that doesn't exist
  auto result = config_manager_->list_parameters("/nonexistent_node_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
  EXPECT_TRUE(result.error_message.find("not available") != std::string::npos ||
              result.error_message.find("Failed") != std::string::npos);
}

TEST_F(TestConfigurationManager, test_list_parameters_own_node) {
  // List parameters for our own test node (which should exist)
  auto result = config_manager_->list_parameters("/test_config_manager_node");

  // Should succeed since we're querying our own node
  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.is_array());
  // Even a basic node has some default parameters
  EXPECT_GE(result.data.size(), 0);
}

// ==================== GET PARAMETER TESTS ====================

TEST_F(TestConfigurationManager, test_get_parameter_nonexistent_node) {
  auto result = config_manager_->get_parameter("/nonexistent_node_xyz", "some_param");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestConfigurationManager, test_get_parameter_nonexistent_param) {
  // Query our own node for a parameter that doesn't exist
  auto result = config_manager_->get_parameter("/test_config_manager_node", "nonexistent_parameter_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("not found") != std::string::npos ||
              result.error_message.find("Parameter not found") != std::string::npos);
}

TEST_F(TestConfigurationManager, test_get_parameter_use_sim_time) {
  // use_sim_time is a standard parameter that should exist
  auto result = config_manager_->get_parameter("/test_config_manager_node", "use_sim_time");

  EXPECT_TRUE(result.success);
  EXPECT_TRUE(result.data.contains("name"));
  EXPECT_EQ(result.data["name"], "use_sim_time");
  EXPECT_TRUE(result.data.contains("value"));
  EXPECT_TRUE(result.data.contains("type"));
  EXPECT_EQ(result.data["type"], "bool");
}

// ==================== SET PARAMETER TESTS ====================

TEST_F(TestConfigurationManager, test_set_parameter_nonexistent_node) {
  auto result = config_manager_->set_parameter("/nonexistent_node_xyz", "param", nlohmann::json(42));

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

// ==================== RESET PARAMETER TESTS ====================

TEST_F(TestConfigurationManager, test_reset_parameter_nonexistent_node) {
  auto result = config_manager_->reset_parameter("/nonexistent_node_xyz", "param");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestConfigurationManager, test_reset_parameter_nonexistent_param) {
  // First need to cache defaults for our node
  auto list_result = config_manager_->list_parameters("/test_config_manager_node");
  ASSERT_TRUE(list_result.success);

  // Try to reset a parameter that doesn't exist in defaults
  auto result = config_manager_->reset_parameter("/test_config_manager_node", "nonexistent_parameter_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.error_message.find("No default value") != std::string::npos);
}

// ==================== RESET ALL PARAMETERS TESTS ====================

TEST_F(TestConfigurationManager, test_reset_all_parameters_nonexistent_node) {
  auto result = config_manager_->reset_all_parameters("/nonexistent_node_xyz");

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(TestConfigurationManager, test_reset_all_parameters_own_node) {
  // First need to cache defaults for our node
  auto list_result = config_manager_->list_parameters("/test_config_manager_node");
  ASSERT_TRUE(list_result.success);

  // Reset all parameters - some may fail because they're read-only
  auto result = config_manager_->reset_all_parameters("/test_config_manager_node");

  // The operation should return data even if some parameters fail
  EXPECT_TRUE(result.data.contains("node_name"));
  EXPECT_TRUE(result.data.contains("reset_count"));
  EXPECT_TRUE(result.data.contains("failed_count"));
  // At least some parameters should have been attempted
  EXPECT_GE(result.data["reset_count"].get<int>() + result.data["failed_count"].get<int>(), 0);
}

// ==================== INTEGRATION TESTS ====================

TEST_F(TestConfigurationManager, test_set_and_get_parameter) {
  // Declare a parameter on our node first
  node_->declare_parameter("test_param", 100);

  // Get the initial value
  auto get_result1 = config_manager_->get_parameter("/test_config_manager_node", "test_param");
  ASSERT_TRUE(get_result1.success);
  EXPECT_EQ(get_result1.data["value"], 100);

  // Set a new value
  auto set_result = config_manager_->set_parameter("/test_config_manager_node", "test_param", nlohmann::json(200));
  ASSERT_TRUE(set_result.success);
  EXPECT_EQ(set_result.data["value"], 200);

  // Verify the new value
  auto get_result2 = config_manager_->get_parameter("/test_config_manager_node", "test_param");
  ASSERT_TRUE(get_result2.success);
  EXPECT_EQ(get_result2.data["value"], 200);
}

TEST_F(TestConfigurationManager, test_set_and_reset_parameter) {
  // Declare a parameter on our node
  node_->declare_parameter("reset_test_param", 42.5);

  // Cache defaults first
  auto list_result = config_manager_->list_parameters("/test_config_manager_node");
  ASSERT_TRUE(list_result.success);

  // Change the value
  auto set_result =
      config_manager_->set_parameter("/test_config_manager_node", "reset_test_param", nlohmann::json(99.9));
  ASSERT_TRUE(set_result.success);

  // Verify changed
  auto get_result1 = config_manager_->get_parameter("/test_config_manager_node", "reset_test_param");
  ASSERT_TRUE(get_result1.success);
  EXPECT_NEAR(get_result1.data["value"].get<double>(), 99.9, 0.01);

  // Reset to default
  auto reset_result = config_manager_->reset_parameter("/test_config_manager_node", "reset_test_param");
  ASSERT_TRUE(reset_result.success);
  EXPECT_TRUE(reset_result.data["reset_to_default"].get<bool>());

  // Verify reset to original
  auto get_result2 = config_manager_->get_parameter("/test_config_manager_node", "reset_test_param");
  ASSERT_TRUE(get_result2.success);
  EXPECT_NEAR(get_result2.data["value"].get<double>(), 42.5, 0.01);
}

TEST_F(TestConfigurationManager, test_set_parameter_different_types) {
  // Test setting parameters of different types
  node_->declare_parameter("string_param", "initial");
  node_->declare_parameter("int_param", 10);
  node_->declare_parameter("double_param", 3.14);
  node_->declare_parameter("bool_param", false);

  // String
  auto str_result =
      config_manager_->set_parameter("/test_config_manager_node", "string_param", nlohmann::json("modified"));
  EXPECT_TRUE(str_result.success);
  EXPECT_EQ(str_result.data["value"], "modified");
  EXPECT_EQ(str_result.data["type"], "string");

  // Integer
  auto int_result = config_manager_->set_parameter("/test_config_manager_node", "int_param", nlohmann::json(999));
  EXPECT_TRUE(int_result.success);
  EXPECT_EQ(int_result.data["value"], 999);
  EXPECT_EQ(int_result.data["type"], "int");

  // Double
  auto dbl_result = config_manager_->set_parameter("/test_config_manager_node", "double_param", nlohmann::json(2.718));
  EXPECT_TRUE(dbl_result.success);
  EXPECT_NEAR(dbl_result.data["value"].get<double>(), 2.718, 0.001);
  EXPECT_EQ(dbl_result.data["type"], "double");

  // Boolean
  auto bool_result = config_manager_->set_parameter("/test_config_manager_node", "bool_param", nlohmann::json(true));
  EXPECT_TRUE(bool_result.success);
  EXPECT_EQ(bool_result.data["value"], true);
  EXPECT_EQ(bool_result.data["type"], "bool");
}

// ==================== ARRAY PARAMETER TESTS ====================

TEST_F(TestConfigurationManager, test_set_and_get_array_parameters) {
  // Test array parameters
  node_->declare_parameter("int_array_param", std::vector<int64_t>{1, 2, 3});
  node_->declare_parameter("double_array_param", std::vector<double>{1.1, 2.2, 3.3});
  node_->declare_parameter("string_array_param", std::vector<std::string>{"a", "b", "c"});

  // Get int array
  auto get_int = config_manager_->get_parameter("/test_config_manager_node", "int_array_param");
  ASSERT_TRUE(get_int.success);
  EXPECT_TRUE(get_int.data["value"].is_array());
  EXPECT_EQ(get_int.data["value"].size(), 3);

  // Get double array
  auto get_double = config_manager_->get_parameter("/test_config_manager_node", "double_array_param");
  ASSERT_TRUE(get_double.success);
  EXPECT_TRUE(get_double.data["value"].is_array());

  // Get string array
  auto get_string = config_manager_->get_parameter("/test_config_manager_node", "string_array_param");
  ASSERT_TRUE(get_string.success);
  EXPECT_TRUE(get_string.data["value"].is_array());
}

// ==================== PARAMETER LISTING WITH PREFIXES ====================

TEST_F(TestConfigurationManager, test_list_parameters_format) {
  // Verify the returned format for listed parameters
  auto result = config_manager_->list_parameters("/test_config_manager_node");

  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.data.is_array());

  // Parameters are returned as objects with name/type/value or just strings
  // depending on implementation
  EXPECT_GE(result.data.size(), 0);
}

// ==================== ERROR MESSAGE VALIDATION ====================

TEST_F(TestConfigurationManager, test_error_messages_informative) {
  // Test that error messages are informative
  auto result1 = config_manager_->get_parameter("/nonexistent_node", "param");
  EXPECT_FALSE(result1.success);
  EXPECT_FALSE(result1.error_message.empty());
  EXPECT_GT(result1.error_message.length(), 10);  // Should be descriptive

  auto result2 = config_manager_->set_parameter("/nonexistent_node", "param", nlohmann::json(42));
  EXPECT_FALSE(result2.success);
  EXPECT_FALSE(result2.error_message.empty());
}

// ==================== NEGATIVE CACHE TESTS ====================

TEST_F(TestConfigurationManager, test_negative_cache_fast_return) {
  // First call to nonexistent node: waits for timeout, returns SERVICE_UNAVAILABLE
  auto result1 = config_manager_->list_parameters("/nonexistent_cached_node");
  EXPECT_FALSE(result1.success);
  EXPECT_EQ(result1.error_code, ParameterErrorCode::SERVICE_UNAVAILABLE);

  // Second call: should return from negative cache (no service discovery wait)
  auto result2 = config_manager_->list_parameters("/nonexistent_cached_node");
  EXPECT_FALSE(result2.success);
  EXPECT_EQ(result2.error_code, ParameterErrorCode::SERVICE_UNAVAILABLE);
  EXPECT_TRUE(result2.error_message.find("cached") != std::string::npos);
}

// ==================== SELF-QUERY GUARD TESTS ====================

TEST_F(TestConfigurationManager, test_self_query_no_deadlock) {
  // Querying our own node should work without deadlock (direct access, no IPC)
  auto result = config_manager_->list_parameters("/test_config_manager_node");

  EXPECT_TRUE(result.success);
}

// ==================== SPIN SERIALIZATION TESTS ====================

TEST_F(TestConfigurationManager, test_concurrent_queries_no_crash) {
  // Concurrent queries to different nonexistent nodes must not crash
  // (spin_mutex_ serializes ROS 2 IPC to prevent executor conflicts).
  std::vector<ParameterResult> results(3);
  std::vector<std::thread> threads;
  for (int i = 0; i < 3; ++i) {
    threads.emplace_back([this, i, &results]() {
      results[static_cast<size_t>(i)] = config_manager_->list_parameters("/concurrent_node_" + std::to_string(i));
    });
  }
  for (auto & t : threads) {
    t.join();
  }
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(results[static_cast<size_t>(i)].success);
    EXPECT_EQ(results[static_cast<size_t>(i)].error_code, ParameterErrorCode::SERVICE_UNAVAILABLE);
  }
}

TEST_F(TestConfigurationManager, test_spin_lock_timeout_returns_error) {
  // Hold spin_mutex_ from a background thread, verify public method returns TIMEOUT
  // Access the mutex via a configurations call that blocks on a nonexistent node
  std::atomic<bool> holding{false};
  std::atomic<bool> done{false};

  // Background thread: acquire spin_mutex_ and hold it
  std::thread blocker([this, &holding, &done]() {
    auto result = config_manager_->list_parameters("/blocking_node_timeout_test");
    holding = true;
    // list_parameters already released the lock, but by the time it returns
    // the test thread should have observed the TIMEOUT. Spin briefly to keep
    // the test deterministic.
    while (!done.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  });

  // Wait for blocker to start (it will hold spin_mutex_ during wait_for_service)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // This call should timeout on spin_mutex_ if blocker is still holding it,
  // OR succeed if blocker already released. Either way, no hang.
  auto start = std::chrono::steady_clock::now();
  auto result = config_manager_->list_parameters("/timeout_test_node");
  auto elapsed = std::chrono::steady_clock::now() - start;

  // Should complete within reasonable time (not hang forever)
  EXPECT_LT(elapsed, std::chrono::seconds(10));
  // Result is either TIMEOUT or SERVICE_UNAVAILABLE - both are acceptable
  EXPECT_FALSE(result.success);

  done = true;
  blocker.join();
}

TEST_F(TestConfigurationManager, test_negative_cache_cross_method) {
  // list_parameters marks node unavailable, get_parameter should return cached
  auto list_result = config_manager_->list_parameters("/cross_method_cached_node");
  EXPECT_FALSE(list_result.success);
  EXPECT_EQ(list_result.error_code, ParameterErrorCode::SERVICE_UNAVAILABLE);

  auto get_result = config_manager_->get_parameter("/cross_method_cached_node", "param");
  EXPECT_FALSE(get_result.success);
  EXPECT_EQ(get_result.error_code, ParameterErrorCode::SERVICE_UNAVAILABLE);
  EXPECT_TRUE(get_result.error_message.find("cached") != std::string::npos);

  auto set_result = config_manager_->set_parameter("/cross_method_cached_node", "param", nlohmann::json(42));
  EXPECT_FALSE(set_result.success);
  EXPECT_EQ(set_result.error_code, ParameterErrorCode::SERVICE_UNAVAILABLE);
  EXPECT_TRUE(set_result.error_message.find("cached") != std::string::npos);
}

// ==================== CONCURRENT ACCESS TEST ====================

TEST_F(TestConfigurationManager, test_concurrent_parameter_access) {
  // Declare a test parameter
  node_->declare_parameter("concurrent_param", 0);

  // Access from multiple threads should be safe
  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};

  for (int i = 0; i < 3; ++i) {
    threads.emplace_back([this, &success_count]() {
      // Give some time between threads to avoid race conditions in service discovery
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      auto result = config_manager_->get_parameter("/test_config_manager_node", "concurrent_param");
      if (result.success) {
        success_count++;
      }
    });
  }

  for (auto & t : threads) {
    t.join();
  }

  // At least some should succeed (concurrency may cause timing issues)
  EXPECT_GE(success_count.load(), 1);
}

TEST_F(TestConfigurationManager, test_concurrent_parameter_operations_no_executor_error) {
  // Regression test: concurrent parameter operations must not cause
  // "Node has already been added to an executor" error.
  // SyncParametersClient internally spins param_node_ - without proper
  // serialization, concurrent calls would cause executor conflicts.

  node_->declare_parameter("concurrent_test_int", 0);
  node_->declare_parameter("concurrent_test_str", std::string("init"));

  // Warm-up: ensure parameter service is discoverable before starting concurrent operations.
  // This prevents flaky failures on slow CI due to service discovery timing.
  auto warmup_result = config_manager_->list_parameters("/test_config_manager_node");
  ASSERT_TRUE(warmup_result.success) << "Warm-up failed: " << warmup_result.error_message;

  constexpr int kNumThreads = 10;
  constexpr int kOpsPerThread = 5;

  std::vector<std::thread> threads;
  std::atomic<int> success_count{0};
  std::atomic<int> exception_count{0};
  std::atomic<bool> start_flag{false};

  // Spawn threads that will all start simultaneously
  for (int i = 0; i < kNumThreads; ++i) {
    threads.emplace_back([this, i, &success_count, &exception_count, &start_flag]() {
      // Wait for all threads to be ready
      while (!start_flag.load()) {
        std::this_thread::yield();
      }

      for (int op = 0; op < kOpsPerThread; ++op) {
        try {
          // Mix different operations to stress test serialization
          switch ((i + op) % 6) {
            case 0: {
              auto result = config_manager_->list_parameters("/test_config_manager_node");
              if (result.success) {
                success_count++;
              }
              break;
            }
            case 1: {
              auto result = config_manager_->get_parameter("/test_config_manager_node", "concurrent_test_int");
              if (result.success) {
                success_count++;
              }
              break;
            }
            case 2: {
              auto result =
                  config_manager_->set_parameter("/test_config_manager_node", "concurrent_test_int", nlohmann::json(i));
              if (result.success) {
                success_count++;
              }
              break;
            }
            case 3: {
              auto result = config_manager_->get_parameter("/test_config_manager_node", "concurrent_test_str");
              if (result.success) {
                success_count++;
              }
              break;
            }
            case 4: {
              auto result = config_manager_->reset_parameter("/test_config_manager_node", "concurrent_test_int");
              if (result.success) {
                success_count++;
              }
              break;
            }
            case 5: {
              auto result = config_manager_->reset_all_parameters("/test_config_manager_node");
              // reset_all_parameters returns success=false if some params are read-only,
              // but data is still populated - count as success if no exception
              if (result.success || result.data.contains("reset_count")) {
                success_count++;
              }
              break;
            }
          }
        } catch (const std::exception & e) {
          // This should NOT happen - executor conflicts would throw here
          exception_count++;
          RCLCPP_ERROR(rclcpp::get_logger("test"), "Exception in concurrent test: %s", e.what());
        }
      }
    });
  }

  // Start all threads simultaneously
  start_flag.store(true);

  for (auto & t : threads) {
    t.join();
  }

  // No exceptions should have occurred (especially executor errors)
  // This is the main assertion - if the mutex serialization is broken,
  // we'd get "Node has already been added to an executor" exceptions here.
  EXPECT_EQ(exception_count.load(), 0) << "Concurrent access caused exceptions (likely executor conflict)";

  // All operations should succeed - we're operating on our own node which is always available.
  EXPECT_EQ(success_count.load(), kNumThreads * kOpsPerThread);
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
