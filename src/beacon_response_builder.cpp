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

#include "ros2_medkit_beacon_common/beacon_response_builder.hpp"

#include <chrono>

namespace ros2_medkit_beacon {

nlohmann::json build_beacon_response(const std::string & entity_id, const BeaconHintStore::StoredHint & stored) {
  nlohmann::json data;
  data["entity_id"] = entity_id;
  if (stored.status == BeaconHintStore::HintStatus::ACTIVE) {
    data["status"] = "active";
  } else if (stored.status == BeaconHintStore::HintStatus::STALE) {
    data["status"] = "stale";
  } else {
    data["status"] = "expired";
  }
  auto age = std::chrono::duration<double>(std::chrono::steady_clock::now() - stored.last_seen).count();
  data["age_sec"] = age;
  data["stable_id"] = stored.hint.stable_id;
  data["display_name"] = stored.hint.display_name;
  data["transport_type"] = stored.hint.transport_type;
  data["negotiated_format"] = stored.hint.negotiated_format;
  data["process_id"] = stored.hint.process_id;
  data["process_name"] = stored.hint.process_name;
  data["hostname"] = stored.hint.hostname;
  data["component_id"] = stored.hint.component_id;
  data["function_ids"] = stored.hint.function_ids;
  data["depends_on"] = stored.hint.depends_on;
  data["metadata"] = stored.hint.metadata;
  return data;
}

}  // namespace ros2_medkit_beacon
