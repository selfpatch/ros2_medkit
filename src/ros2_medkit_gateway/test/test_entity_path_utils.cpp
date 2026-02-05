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

#include "ros2_medkit_gateway/http/entity_path_utils.hpp"

using namespace ros2_medkit_gateway;

// ==================== parse_entity_path tests ====================

TEST(EntityPathUtilsTest, ParseTopLevelApp) {
  auto info = parse_entity_path("/api/v1/apps/motor_controller/bulk-data/rosbags");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::APP);
  EXPECT_EQ(info->entity_id, "motor_controller");
  EXPECT_EQ(info->resource_path, "/bulk-data/rosbags");
  EXPECT_EQ(info->entity_path, "/apps/motor_controller");
  EXPECT_TRUE(info->parent_id.empty());
  EXPECT_FALSE(info->is_nested);
}

TEST(EntityPathUtilsTest, ParseTopLevelComponent) {
  auto info = parse_entity_path("/api/v1/components/perception_lidar/faults/ERROR");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::COMPONENT);
  EXPECT_EQ(info->entity_id, "perception_lidar");
  EXPECT_EQ(info->resource_path, "/faults/ERROR");
  EXPECT_EQ(info->entity_path, "/components/perception_lidar");
  EXPECT_TRUE(info->parent_id.empty());
  EXPECT_FALSE(info->is_nested);
}

TEST(EntityPathUtilsTest, ParseTopLevelArea) {
  auto info = parse_entity_path("/api/v1/areas/powertrain/data");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::AREA);
  EXPECT_EQ(info->entity_id, "powertrain");
  EXPECT_EQ(info->resource_path, "/data");
  EXPECT_EQ(info->entity_path, "/areas/powertrain");
  EXPECT_TRUE(info->parent_id.empty());
  EXPECT_FALSE(info->is_nested);
}

TEST(EntityPathUtilsTest, ParseTopLevelFunction) {
  auto info = parse_entity_path("/api/v1/functions/motor_control/operations");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::FUNCTION);
  EXPECT_EQ(info->entity_id, "motor_control");
  EXPECT_EQ(info->resource_path, "/operations");
  EXPECT_EQ(info->entity_path, "/functions/motor_control");
  EXPECT_TRUE(info->parent_id.empty());
  EXPECT_FALSE(info->is_nested);
}

TEST(EntityPathUtilsTest, ParseNestedSubarea) {
  auto info = parse_entity_path("/api/v1/areas/perception/subareas/lidar/faults/LIDAR_ERROR");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::AREA);
  EXPECT_EQ(info->entity_id, "lidar");
  EXPECT_EQ(info->resource_path, "/faults/LIDAR_ERROR");
  EXPECT_EQ(info->entity_path, "/areas/perception/subareas/lidar");
  EXPECT_EQ(info->parent_id, "perception");
  EXPECT_TRUE(info->is_nested);
}

TEST(EntityPathUtilsTest, ParseNestedSubcomponent) {
  auto info = parse_entity_path("/api/v1/components/ecu/subcomponents/sensor_hub/data");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::COMPONENT);
  EXPECT_EQ(info->entity_id, "sensor_hub");
  EXPECT_EQ(info->resource_path, "/data");
  EXPECT_EQ(info->entity_path, "/components/ecu/subcomponents/sensor_hub");
  EXPECT_EQ(info->parent_id, "ecu");
  EXPECT_TRUE(info->is_nested);
}

TEST(EntityPathUtilsTest, ParseEntityWithNoResourcePath) {
  auto info = parse_entity_path("/api/v1/apps/motor_controller");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::APP);
  EXPECT_EQ(info->entity_id, "motor_controller");
  EXPECT_TRUE(info->resource_path.empty());
  EXPECT_EQ(info->entity_path, "/apps/motor_controller");
}

TEST(EntityPathUtilsTest, ParseInvalidPath) {
  auto info = parse_entity_path("/api/v1/invalid/path");
  EXPECT_FALSE(info.has_value());
}

TEST(EntityPathUtilsTest, ParseEmptyPath) {
  auto info = parse_entity_path("");
  EXPECT_FALSE(info.has_value());
}

TEST(EntityPathUtilsTest, ParsePathWithoutApiPrefix) {
  // Should not match without /api/v1 prefix
  auto info = parse_entity_path("/apps/motor_controller/faults");
  EXPECT_FALSE(info.has_value());
}

TEST(EntityPathUtilsTest, ParsePathWithTrailingSlash) {
  auto info = parse_entity_path("/api/v1/apps/motor_controller/");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::APP);
  EXPECT_EQ(info->entity_id, "motor_controller");
  EXPECT_EQ(info->resource_path, "/");
}

// ==================== extract_bulk_data_category tests ====================

TEST(EntityPathUtilsTest, ExtractCategoryFromBulkDataPath) {
  EXPECT_EQ(extract_bulk_data_category("/bulk-data/rosbags/abc-123"), "rosbags");
  EXPECT_EQ(extract_bulk_data_category("/bulk-data/snapshots/xyz"), "snapshots");
}

TEST(EntityPathUtilsTest, ExtractCategoryFromFullPath) {
  EXPECT_EQ(extract_bulk_data_category("/api/v1/apps/motor/bulk-data/rosbags/uuid"), "rosbags");
}

TEST(EntityPathUtilsTest, ExtractCategoryEmpty) {
  EXPECT_EQ(extract_bulk_data_category("/api/v1/apps/motor/faults"), "");
  EXPECT_EQ(extract_bulk_data_category(""), "");
}

// ==================== extract_bulk_data_id tests ====================

TEST(EntityPathUtilsTest, ExtractBulkDataIdFromPath) {
  EXPECT_EQ(extract_bulk_data_id("/bulk-data/rosbags/550e8400-e29b-41d4-a716-446655440000"),
            "550e8400-e29b-41d4-a716-446655440000");
}

TEST(EntityPathUtilsTest, ExtractBulkDataIdFromFullPath) {
  EXPECT_EQ(extract_bulk_data_id("/api/v1/apps/motor/bulk-data/snapshots/my-snapshot-id"), "my-snapshot-id");
}

TEST(EntityPathUtilsTest, ExtractBulkDataIdEmpty) {
  EXPECT_EQ(extract_bulk_data_id("/bulk-data/rosbags"), "");  // No ID after category
  EXPECT_EQ(extract_bulk_data_id("/api/v1/apps/motor"), "");  // No bulk-data segment
  EXPECT_EQ(extract_bulk_data_id(""), "");
}

// ==================== Complex path scenarios ====================

TEST(EntityPathUtilsTest, ParseAppWithMultipleResourceSegments) {
  auto info = parse_entity_path("/api/v1/apps/motor_controller/faults/MOTOR_ERR/snapshots");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::APP);
  EXPECT_EQ(info->entity_id, "motor_controller");
  EXPECT_EQ(info->resource_path, "/faults/MOTOR_ERR/snapshots");
}

TEST(EntityPathUtilsTest, ParseComponentWithBulkData) {
  auto info = parse_entity_path("/api/v1/components/sensor_hub/bulk-data/rosbags/uuid-123");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::COMPONENT);
  EXPECT_EQ(info->entity_id, "sensor_hub");
  EXPECT_EQ(info->resource_path, "/bulk-data/rosbags/uuid-123");

  // Verify bulk-data extraction still works on resource_path
  EXPECT_EQ(extract_bulk_data_category(info->resource_path), "rosbags");
  EXPECT_EQ(extract_bulk_data_id(info->resource_path), "uuid-123");
}

TEST(EntityPathUtilsTest, ParseNestedSubareaWithBulkData) {
  auto info = parse_entity_path("/api/v1/areas/perception/subareas/camera/bulk-data/snapshots/snap-001");
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->type, SovdEntityType::AREA);
  EXPECT_EQ(info->entity_id, "camera");
  EXPECT_EQ(info->parent_id, "perception");
  EXPECT_TRUE(info->is_nested);
  EXPECT_EQ(info->resource_path, "/bulk-data/snapshots/snap-001");
}
