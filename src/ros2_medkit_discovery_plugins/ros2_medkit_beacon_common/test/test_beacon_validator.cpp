// Copyright 2026 selfpatch GmbH
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

#include <string>

#include "ros2_medkit_beacon_common/beacon_validator.hpp"

using ros2_medkit_beacon::BeaconHint;
using ros2_medkit_beacon::validate_beacon_hint;
using ros2_medkit_beacon::ValidationLimits;

class TestBeaconValidator : public ::testing::Test {
 protected:
  ValidationLimits limits_;  // defaults

  BeaconHint make_valid_hint() {
    BeaconHint hint;
    hint.entity_id = "engine_temp_sensor";
    hint.transport_type = "shared_memory";
    hint.process_id = 12847;
    hint.received_at = std::chrono::steady_clock::now();
    return hint;
  }
};

TEST_F(TestBeaconValidator, ValidHintAccepted) {
  auto hint = make_valid_hint();
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_TRUE(result.valid);
  EXPECT_TRUE(result.reason.empty());
}

TEST_F(TestBeaconValidator, EmptyEntityIdRejected) {
  auto hint = make_valid_hint();
  hint.entity_id = "";
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.reason.empty());
}

TEST_F(TestBeaconValidator, InvalidEntityIdCharsRejected) {
  auto hint = make_valid_hint();
  hint.entity_id = "bad/entity id";
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_FALSE(result.valid);
}

TEST_F(TestBeaconValidator, EntityIdWithNullByteRejected) {
  auto hint = make_valid_hint();
  hint.entity_id = std::string("bad\0id", 6);
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_FALSE(result.valid);
}

TEST_F(TestBeaconValidator, OversizedEntityIdRejected) {
  auto hint = make_valid_hint();
  hint.entity_id = std::string(257, 'a');
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_FALSE(result.valid);
}

TEST_F(TestBeaconValidator, InvalidFunctionIdSkipped) {
  auto hint = make_valid_hint();
  hint.function_ids = {"valid-id", "bad/id", "also_valid"};
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_TRUE(result.valid);                // hint itself is valid
  EXPECT_EQ(hint.function_ids.size(), 2u);  // invalid entry removed
  EXPECT_EQ(hint.function_ids[0], "valid-id");
  EXPECT_EQ(hint.function_ids[1], "also_valid");
}

TEST_F(TestBeaconValidator, FunctionIdsTruncatedAtLimit) {
  auto hint = make_valid_hint();
  for (size_t i = 0; i < 105; ++i) {
    hint.function_ids.push_back("func_" + std::to_string(i));
  }
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(hint.function_ids.size(), limits_.max_function_ids);
}

TEST_F(TestBeaconValidator, MetadataKeyTooLongSkipped) {
  auto hint = make_valid_hint();
  hint.metadata[std::string(65, 'k')] = "value";
  hint.metadata["good_key"] = "value";
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(hint.metadata.size(), 1u);
  EXPECT_TRUE(hint.metadata.count("good_key"));
}

TEST_F(TestBeaconValidator, MetadataValueTruncated) {
  auto hint = make_valid_hint();
  hint.metadata["key"] = std::string(2000, 'v');
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_TRUE(result.valid);
  EXPECT_EQ(hint.metadata["key"].size(), limits_.max_metadata_value_length);
}

TEST_F(TestBeaconValidator, ZeroPidTreatedAsNotProvided) {
  auto hint = make_valid_hint();
  hint.process_id = 0;
  auto result = validate_beacon_hint(hint, limits_);
  EXPECT_TRUE(result.valid);
}
