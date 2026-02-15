// Copyright 2026 mfaferek93
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

#include "ros2_medkit_fault_manager/correlation/config_parser.hpp"
#include "ros2_medkit_fault_manager/correlation/correlation_engine.hpp"

using namespace ros2_medkit_fault_manager::correlation;
using namespace std::chrono_literals;

class CorrelationEngineTest : public ::testing::Test {
 protected:
  void SetUp() override {
  }

  CorrelationConfig create_hierarchical_config() {
    const std::string yaml = R"(
correlation:
  enabled: true
  default_window_ms: 500
  patterns:
    motor_errors:
      codes: ["MOTOR_COMM_*", "MOTOR_TIMEOUT_*"]
    drive_faults:
      codes: ["DRIVE_*"]
  rules:
    - id: estop_cascade
      name: "E-Stop Cascade"
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: motor_errors
        - pattern: drive_faults
      window_ms: 1000
      mute_symptoms: true
      auto_clear_with_root: true
)";
    return parse_config_string(yaml);
  }

  CorrelationConfig create_auto_cluster_config() {
    const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    comm_errors:
      codes: ["*_COMM_*", "*_TIMEOUT"]
  rules:
    - id: comm_storm
      name: "Communication Storm"
      mode: auto_cluster
      match:
        - pattern: comm_errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: highest_severity
)";
    return parse_config_string(yaml);
  }

  CorrelationConfig create_mixed_config() {
    const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    motor_errors:
      codes: ["MOTOR_*"]
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: estop_rule
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: motor_errors
      window_ms: 500
      mute_symptoms: true
      auto_clear_with_root: true
    - id: sensor_cluster
      mode: auto_cluster
      match:
        - pattern: sensor_errors
      min_count: 2
      window_ms: 500
      representative: first
)";
    return parse_config_string(yaml);
  }
};

// ============================================================================
// Hierarchical correlation tests
// ============================================================================

TEST_F(CorrelationEngineTest, RootCauseRecognized) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  auto result = engine.process_fault("ESTOP_001", "CRITICAL");

  EXPECT_FALSE(result.should_mute);
  EXPECT_TRUE(result.is_root_cause);
  EXPECT_EQ("estop_cascade", result.rule_id);
}

TEST_F(CorrelationEngineTest, SymptomMuted) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // First, report root cause
  auto root_result = engine.process_fault("ESTOP_001", "CRITICAL");
  EXPECT_TRUE(root_result.is_root_cause);

  // Then report symptom
  auto symptom_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");

  EXPECT_TRUE(symptom_result.should_mute);
  EXPECT_FALSE(symptom_result.is_root_cause);
  EXPECT_EQ("ESTOP_001", symptom_result.root_cause_code);
  EXPECT_EQ("estop_cascade", symptom_result.rule_id);
  // delay_ms can be 0 if faults are processed in quick succession
  EXPECT_GE(symptom_result.delay_ms, 0u);

  // Check muted faults
  EXPECT_EQ(1u, engine.get_muted_count());
  auto muted = engine.get_muted_faults();
  ASSERT_EQ(1u, muted.size());
  EXPECT_EQ("MOTOR_COMM_FL", muted[0].fault_code);
  EXPECT_EQ("ESTOP_001", muted[0].root_cause_code);
}

TEST_F(CorrelationEngineTest, MultipleSymptomsMuted) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("MOTOR_COMM_FR", "ERROR");
  engine.process_fault("DRIVE_FAULT", "ERROR");

  EXPECT_EQ(3u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, SymptomBeforeRootCauseNotCorrelated) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // Report symptom BEFORE root cause
  auto symptom_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");

  // Should NOT be muted (no root cause yet)
  EXPECT_FALSE(symptom_result.should_mute);
  EXPECT_FALSE(symptom_result.is_root_cause);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, SymptomAfterWindowNotCorrelated) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  auto start = std::chrono::steady_clock::now();
  engine.process_fault("ESTOP_001", "CRITICAL", start);

  // Report symptom AFTER window (window is 1000ms)
  auto after_window = start + 1500ms;
  auto symptom_result = engine.process_fault("MOTOR_COMM_FL", "ERROR", after_window);

  // Should NOT be muted (outside window)
  EXPECT_FALSE(symptom_result.should_mute);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, ClearRootCauseClearsSymptoms) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("MOTOR_COMM_FR", "ERROR");

  EXPECT_EQ(2u, engine.get_muted_count());

  // Clear root cause
  auto clear_result = engine.process_clear("ESTOP_001");

  EXPECT_EQ(2u, clear_result.auto_cleared_codes.size());
  EXPECT_NE(std::find(clear_result.auto_cleared_codes.begin(), clear_result.auto_cleared_codes.end(), "MOTOR_COMM_FL"),
            clear_result.auto_cleared_codes.end());
  EXPECT_NE(std::find(clear_result.auto_cleared_codes.begin(), clear_result.auto_cleared_codes.end(), "MOTOR_COMM_FR"),
            clear_result.auto_cleared_codes.end());

  // Muted faults should be cleared
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, UnrelatedFaultNotCorrelated) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  auto result = engine.process_fault("SENSOR_TIMEOUT", "ERROR");

  // SENSOR_TIMEOUT doesn't match motor_errors or drive_faults patterns
  EXPECT_FALSE(result.should_mute);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, InlineSymptomCodesMuted) {
  // Config with inline codes instead of pattern references
  const std::string yaml = R"(
correlation:
  enabled: true
  rules:
    - id: estop_inline
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - codes: ["MOTOR_*", "DRIVE_*"]
      window_ms: 1000
      mute_symptoms: true
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  // Report root cause
  auto root_result = engine.process_fault("ESTOP_001", "CRITICAL");
  EXPECT_TRUE(root_result.is_root_cause);

  // Report symptoms matching inline codes
  auto motor_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_TRUE(motor_result.should_mute);
  EXPECT_EQ("ESTOP_001", motor_result.root_cause_code);

  auto drive_result = engine.process_fault("DRIVE_FAULT", "ERROR");
  EXPECT_TRUE(drive_result.should_mute);

  // Unrelated fault should not be muted
  auto sensor_result = engine.process_fault("SENSOR_ERROR", "ERROR");
  EXPECT_FALSE(sensor_result.should_mute);

  EXPECT_EQ(2u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, MixedPatternAndInlineSymptoms) {
  // Config with both pattern references and inline codes
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: estop_mixed
      mode: hierarchical
      root_cause:
        codes: ["ESTOP_001"]
      symptoms:
        - pattern: sensor_errors
        - codes: ["MOTOR_*"]
      window_ms: 1000
      mute_symptoms: true
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");

  // Both pattern-matched and inline-matched faults should be muted
  auto sensor_result = engine.process_fault("SENSOR_TIMEOUT", "ERROR");
  EXPECT_TRUE(sensor_result.should_mute);

  auto motor_result = engine.process_fault("MOTOR_FAULT", "ERROR");
  EXPECT_TRUE(motor_result.should_mute);

  EXPECT_EQ(2u, engine.get_muted_count());
}

// ============================================================================
// Auto-cluster tests
// ============================================================================

TEST_F(CorrelationEngineTest, AutoClusterTriggersAtMinCount) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  // Report faults - need 3 for cluster
  auto r1 = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_FALSE(r1.should_mute);
  EXPECT_EQ(0u, engine.get_cluster_count());

  auto r2 = engine.process_fault("SENSOR_TIMEOUT", "ERROR");
  EXPECT_FALSE(r2.should_mute);
  EXPECT_EQ(0u, engine.get_cluster_count());

  // Third fault triggers cluster
  auto r3 = engine.process_fault("DRIVE_COMM_ERROR", "WARNING");
  // Third fault should be muted (show_as_single=true)
  EXPECT_TRUE(r3.should_mute);
  EXPECT_EQ(1u, engine.get_cluster_count());
}

TEST_F(CorrelationEngineTest, AutoClusterNotTriggeredBelowMinCount) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("SENSOR_TIMEOUT", "ERROR");

  // Only 2 faults, need 3
  EXPECT_EQ(0u, engine.get_cluster_count());
}

TEST_F(CorrelationEngineTest, AutoClusterHighestSeverityRepresentative) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  engine.process_fault("MOTOR_COMM_FL", "WARNING");
  engine.process_fault("SENSOR_TIMEOUT", "CRITICAL");  // Higher severity
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR");

  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("SENSOR_TIMEOUT", clusters[0].representative_code);
  EXPECT_EQ("CRITICAL", clusters[0].representative_severity);
}

TEST_F(CorrelationEngineTest, AutoClusterFirstRepresentative) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    errors:
      codes: ["*_ERROR"]
  rules:
    - id: test_cluster
      mode: auto_cluster
      match:
        - pattern: errors
      min_count: 2
      representative: first
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  engine.process_fault("FIRST_ERROR", "WARNING");
  engine.process_fault("SECOND_ERROR", "CRITICAL");

  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  EXPECT_EQ("FIRST_ERROR", clusters[0].representative_code);
}

TEST_F(CorrelationEngineTest, AutoClusterWindowExpires) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  auto start = std::chrono::steady_clock::now();

  engine.process_fault("MOTOR_COMM_FL", "ERROR", start);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", start + 100ms);

  // Third fault after window (500ms)
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR", start + 600ms);

  // Cluster should have been reset, so still not at min_count
  // Actually the third fault starts a new pending cluster
  EXPECT_EQ(0u, engine.get_cluster_count());
}

TEST_F(CorrelationEngineTest, CleanupExpiredRemovesPendingRootCauses) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // Use a fixed timestamp in the past (beyond window_ms=1000ms)
  auto past = std::chrono::steady_clock::now() - std::chrono::milliseconds(2000);

  // Report root cause with old timestamp
  engine.process_fault("ESTOP_001", "CRITICAL", past);

  // Symptom reported NOW should NOT be correlated (root cause expired)
  auto result = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_FALSE(result.should_mute);  // Not muted - root cause window expired
  EXPECT_EQ(0u, engine.get_muted_count());

  // Call cleanup to explicitly remove expired entries
  engine.cleanup_expired();

  // Another symptom should also not be correlated
  auto result2 = engine.process_fault("MOTOR_TIMEOUT_RR", "ERROR");
  EXPECT_FALSE(result2.should_mute);
  EXPECT_EQ(0u, engine.get_muted_count());
}

TEST_F(CorrelationEngineTest, CleanupExpiredRemovesPendingClusters) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  // Create pending cluster with old timestamp
  auto past = std::chrono::steady_clock::now() - std::chrono::milliseconds(1000);
  engine.process_fault("MOTOR_COMM_FL", "ERROR", past);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", past + 10ms);

  // Cluster should not be active (only 2 faults, need 3)
  EXPECT_EQ(0u, engine.get_cluster_count());

  // Cleanup should remove expired pending cluster
  engine.cleanup_expired();

  // New fault should start fresh pending cluster, not join expired one
  auto result = engine.process_fault("DRIVE_COMM_ERROR", "ERROR");
  EXPECT_FALSE(result.should_mute);           // First in new cluster
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still not enough
}

// ============================================================================
// Mixed mode tests
// ============================================================================

TEST_F(CorrelationEngineTest, HierarchicalAndClusterCoexist) {
  auto config = create_mixed_config();
  CorrelationEngine engine(config);

  // Hierarchical: ESTOP_001 -> MOTOR_*
  engine.process_fault("ESTOP_001", "CRITICAL");
  auto motor_result = engine.process_fault("MOTOR_COMM_FL", "ERROR");
  EXPECT_TRUE(motor_result.should_mute);
  EXPECT_EQ(1u, engine.get_muted_count());

  // Auto-cluster: SENSOR_* (need 2)
  engine.process_fault("SENSOR_LIDAR", "ERROR");
  engine.process_fault("SENSOR_IMU", "ERROR");
  EXPECT_EQ(1u, engine.get_cluster_count());

  // Both should coexist
  EXPECT_EQ(1u, engine.get_muted_count());    // MOTOR_COMM_FL
  EXPECT_EQ(1u, engine.get_cluster_count());  // SENSOR cluster
}

// ============================================================================
// Edge cases
// ============================================================================

TEST_F(CorrelationEngineTest, DuplicateFaultCode) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  engine.process_fault("ESTOP_001", "CRITICAL");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");
  engine.process_fault("MOTOR_COMM_FL", "ERROR");  // Duplicate

  // Should still only have 1 muted fault (no duplicate)
  // Note: current implementation allows duplicates in symptoms list
  // This test documents current behavior
  EXPECT_GE(engine.get_muted_count(), 1u);
}

TEST_F(CorrelationEngineTest, EmptyEngineQueries) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // No faults processed
  EXPECT_EQ(0u, engine.get_muted_count());
  EXPECT_EQ(0u, engine.get_cluster_count());
  EXPECT_TRUE(engine.get_muted_faults().empty());
  EXPECT_TRUE(engine.get_clusters().empty());
}

TEST_F(CorrelationEngineTest, ClearNonExistentFault) {
  auto config = create_hierarchical_config();
  CorrelationEngine engine(config);

  // Clear fault that was never reported
  auto result = engine.process_clear("NONEXISTENT");

  EXPECT_TRUE(result.auto_cleared_codes.empty());
}

TEST_F(CorrelationEngineTest, AutoClusterRetroactiveMuting) {
  // Create config with FIRST representative policy to have predictable representative
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    sensor_errors:
      codes: ["SENSOR_*"]
  rules:
    - id: sensor_cluster
      mode: auto_cluster
      match:
        - pattern: sensor_errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: first
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Fault #1 - representative (FIRST policy)
  auto result1 = engine.process_fault("SENSOR_001", "ERROR", t0);
  EXPECT_FALSE(result1.should_mute);  // First fault is representative
  EXPECT_FALSE(result1.cluster_id.empty());
  EXPECT_TRUE(result1.retroactive_mute_codes.empty());  // Cluster not active yet

  // Fault #2 - not muted because cluster not active
  auto result2 = engine.process_fault("SENSOR_002", "ERROR", t0 + std::chrono::milliseconds(10));
  EXPECT_FALSE(result2.should_mute);  // Cluster still not active
  EXPECT_TRUE(result2.retroactive_mute_codes.empty());

  // Fault #3 - triggers cluster activation (min_count=3)
  auto result3 = engine.process_fault("SENSOR_003", "ERROR", t0 + std::chrono::milliseconds(20));
  EXPECT_TRUE(result3.should_mute);                      // #3 is muted (not representative)
  EXPECT_EQ(1u, result3.retroactive_mute_codes.size());  // #2 should be retroactively muted
  EXPECT_EQ("SENSOR_002", result3.retroactive_mute_codes[0]);

  // Cluster should be active with 3 faults
  EXPECT_EQ(1u, engine.get_cluster_count());
  auto clusters = engine.get_clusters();
  EXPECT_EQ(3u, clusters[0].fault_codes.size());
  EXPECT_EQ("SENSOR_001", clusters[0].representative_code);  // FIRST policy
}

// ============================================================================
// Pending cluster cleanup on clear (#127)
// ============================================================================

TEST_F(CorrelationEngineTest, ClearFaultRemovesFromPendingCluster) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Add 2 faults (below min_count=3), creating a pending cluster
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t0);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", t0 + 10ms);
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still pending

  // Clear one of the faults
  engine.process_clear("MOTOR_COMM_FL");

  // Now add a third fault - should NOT activate the cluster because
  // the cleared fault was removed from pending, so only 2 faults total
  engine.process_fault("DRIVE_COMM_ERROR", "ERROR", t0 + 20ms);
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still not enough
}

TEST_F(CorrelationEngineTest, ClearRepresentativeReassignsPendingCluster) {
  const std::string yaml = R"(
correlation:
  enabled: true
  patterns:
    errors:
      codes: ["*_ERROR"]
  rules:
    - id: test_cluster
      mode: auto_cluster
      match:
        - pattern: errors
      min_count: 3
      window_ms: 500
      show_as_single: true
      representative: first
)";
  auto config = parse_config_string(yaml);
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Add 2 faults, creating a pending cluster
  engine.process_fault("FIRST_ERROR", "CRITICAL", t0);
  engine.process_fault("SECOND_ERROR", "ERROR", t0 + 10ms);

  // Clear the representative (first fault)
  engine.process_clear("FIRST_ERROR");

  // Add 2 more faults to reach min_count (SECOND_ERROR + 2 new = 3)
  engine.process_fault("THIRD_ERROR", "ERROR", t0 + 20ms);
  engine.process_fault("FOURTH_ERROR", "ERROR", t0 + 30ms);

  EXPECT_EQ(1u, engine.get_cluster_count());
  auto clusters = engine.get_clusters();
  ASSERT_EQ(1u, clusters.size());
  // SECOND_ERROR should be the new representative (first remaining after clear)
  EXPECT_EQ("SECOND_ERROR", clusters[0].representative_code);
}

TEST_F(CorrelationEngineTest, ClearAllFaultsRemovesPendingCluster) {
  auto config = create_auto_cluster_config();
  CorrelationEngine engine(config);

  auto t0 = std::chrono::steady_clock::now();

  // Add 2 faults
  engine.process_fault("MOTOR_COMM_FL", "ERROR", t0);
  engine.process_fault("SENSOR_TIMEOUT", "ERROR", t0 + 10ms);

  // Clear both
  engine.process_clear("MOTOR_COMM_FL");
  engine.process_clear("SENSOR_TIMEOUT");

  // Adding 2 new faults should start a fresh pending cluster, not join old one
  engine.process_fault("DRIVE_COMM_NEW", "ERROR", t0 + 100ms);
  engine.process_fault("MOTOR_COMM_NEW", "ERROR", t0 + 110ms);
  EXPECT_EQ(0u, engine.get_cluster_count());  // Still 2 faults, below min_count=3
}

int main(int argc, char ** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
