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

#include <nlohmann/json.hpp>
#include <string>

#include "ros2_medkit_beacon_common/beacon_hint_store.hpp"

namespace ros2_medkit_beacon {

/// Build the JSON response for a beacon metadata endpoint from a stored hint.
nlohmann::json build_beacon_response(const std::string & entity_id, const BeaconHintStore::StoredHint & stored);

}  // namespace ros2_medkit_beacon
