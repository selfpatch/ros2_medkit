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

#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "ros2_medkit_beacon_common/beacon_hint_store.hpp"
#include "ros2_medkit_beacon_common/beacon_types.hpp"
#include "ros2_medkit_gateway/providers/introspection_provider.hpp"

namespace ros2_medkit_beacon {

class BeaconEntityMapper {
 public:
  struct Config {
    bool allow_new_entities{false};
  };

  BeaconEntityMapper();
  explicit BeaconEntityMapper(Config config);

  ros2_medkit_gateway::IntrospectionResult map(const std::vector<BeaconHintStore::StoredHint> & hints,
                                               const ros2_medkit_gateway::IntrospectionInput & current) const;

 private:
  nlohmann::json build_metadata(const BeaconHintStore::StoredHint & hint) const;

  void apply_function_membership(const BeaconHint & hint, ros2_medkit_gateway::IntrospectionResult & result,
                                 const ros2_medkit_gateway::IntrospectionInput & current) const;

  Config config_;
};

}  // namespace ros2_medkit_beacon
