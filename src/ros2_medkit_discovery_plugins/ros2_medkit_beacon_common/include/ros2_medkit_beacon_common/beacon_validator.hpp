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

#pragma once

#include <cstddef>
#include <string>

#include "ros2_medkit_beacon_common/beacon_types.hpp"

namespace ros2_medkit_beacon {

struct ValidationResult {
  bool valid{true};
  std::string reason;
};

struct ValidationLimits {
  size_t max_id_length{256};
  size_t max_string_length{512};
  size_t max_function_ids{100};
  size_t max_depends_on{100};
  size_t max_metadata_entries{50};
  size_t max_metadata_key_length{64};
  size_t max_metadata_value_length{1024};
};

/// Validates and sanitizes a beacon hint in-place.
/// Returns valid=false if entity_id fails validation (hint must be rejected).
/// For other fields, invalid entries are removed/truncated (hint remains valid).
ValidationResult validate_beacon_hint(BeaconHint & hint, const ValidationLimits & limits);

}  // namespace ros2_medkit_beacon
