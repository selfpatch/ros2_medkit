// Copyright 2025 mfaferek93
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

#include "ros2_medkit_fault_reporter/local_filter.hpp"
#include "ros2_medkit_msgs/msg/fault.hpp"

using ros2_medkit_fault_reporter::FilterConfig;
using ros2_medkit_fault_reporter::LocalFilter;
using ros2_medkit_msgs::msg::Fault;

class LocalFilterTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Default config: threshold=3, window=10s, bypass_severity=ERROR
    FilterConfig config;
    config.enabled = true;
    config.default_threshold = 3;
    config.default_window_sec = 10.0;
    config.bypass_severity = Fault::SEVERITY_ERROR;
    filter_ = std::make_unique<LocalFilter>(config);
  }

  std::unique_ptr<LocalFilter> filter_;
};

TEST_F(LocalFilterTest, FilterDisabledAlwaysForwards) {
  FilterConfig config;
  config.enabled = false;
  filter_->set_config(config);

  // Should forward on first call when filtering is disabled
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, ThresholdNotMetFilters) {
  // First two calls should not forward (threshold is 3)
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));

  // Third call meets threshold, should forward
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, DifferentFaultCodesTrackedSeparately) {
  // Each fault code has its own counter
  EXPECT_FALSE(filter_->should_forward("FAULT_A", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("FAULT_B", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("FAULT_A", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("FAULT_B", Fault::SEVERITY_INFO));

  // Third call for each should forward
  EXPECT_TRUE(filter_->should_forward("FAULT_A", Fault::SEVERITY_INFO));
  EXPECT_TRUE(filter_->should_forward("FAULT_B", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, HighSeverityBypassesFilter) {
  // ERROR and CRITICAL should bypass filtering (bypass_severity = ERROR = 2)
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_ERROR));
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_CRITICAL));
}

TEST_F(LocalFilterTest, TimeWindowExpiry) {
  // Use a short window for testing
  FilterConfig config;
  config.enabled = true;
  config.default_threshold = 3;
  config.default_window_sec = 0.1;  // 100ms window
  config.bypass_severity = Fault::SEVERITY_CRITICAL;
  filter_->set_config(config);

  // Two reports
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));

  // Wait for window to expire
  std::this_thread::sleep_for(std::chrono::milliseconds(150));

  // Previous reports expired, start counting again
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, ResetClearsTracking) {
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));

  filter_->reset("TEST_FAULT");

  // Should start counting from zero again
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, ResetAllClearsAllTracking) {
  EXPECT_FALSE(filter_->should_forward("FAULT_A", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("FAULT_B", Fault::SEVERITY_INFO));

  filter_->reset_all();

  // Both should start from zero
  EXPECT_FALSE(filter_->should_forward("FAULT_A", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("FAULT_B", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, ConfigReturnsCurrentConfig) {
  const auto & config = filter_->config();
  EXPECT_TRUE(config.enabled);
  EXPECT_EQ(config.default_threshold, 3);
  EXPECT_DOUBLE_EQ(config.default_window_sec, 10.0);
  EXPECT_EQ(config.bypass_severity, Fault::SEVERITY_ERROR);
}

TEST_F(LocalFilterTest, SetConfigClearsState) {
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));

  // Change config (this clears state)
  FilterConfig new_config;
  new_config.enabled = true;
  new_config.default_threshold = 2;
  new_config.default_window_sec = 10.0;
  new_config.bypass_severity = Fault::SEVERITY_CRITICAL;
  filter_->set_config(new_config);

  // State was cleared, need 2 reports now (new threshold)
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, ThresholdOneForwardsImmediately) {
  FilterConfig config;
  config.enabled = true;
  config.default_threshold = 1;
  config.default_window_sec = 10.0;
  config.bypass_severity = Fault::SEVERITY_CRITICAL;
  filter_->set_config(config);

  // With threshold=1, first call should forward
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
}

TEST_F(LocalFilterTest, ContinuousReportingAfterThreshold) {
  // Reach threshold
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_FALSE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));

  // After threshold is met, subsequent reports also forward (count >= threshold)
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
  EXPECT_TRUE(filter_->should_forward("TEST_FAULT", Fault::SEVERITY_INFO));
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
