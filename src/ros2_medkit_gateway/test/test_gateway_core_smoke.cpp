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

// Smoke test: this translation unit must compile and link against
// gateway_core alone, without any ament/rclcpp link dependency. Any ROS
// transitive include reaching the core layer would surface here as a
// missing-symbol link error, catching leaks the grep-based purity check
// might miss when an include is reached via a third-party header.

#include "ros2_medkit_gateway/core/discovery/models/app.hpp"
#include "ros2_medkit_gateway/core/discovery/models/area.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"
#include "ros2_medkit_gateway/core/discovery/models/function.hpp"
#include "ros2_medkit_gateway/core/providers/data_provider.hpp"
#include "ros2_medkit_gateway/core/providers/fault_provider.hpp"
#include "ros2_medkit_gateway/core/providers/introspection_provider.hpp"
#include "ros2_medkit_gateway/core/providers/log_provider.hpp"
#include "ros2_medkit_gateway/core/providers/operation_provider.hpp"

#include <gtest/gtest.h>

#include <type_traits>

namespace {

// Reference each entity model and provider interface so the includes
// above are not flagged as unused. The translation unit must still link
// against gateway_core alone, which proves the neutral-layer contract.

using ros2_medkit_gateway::App;
using ros2_medkit_gateway::Area;
using ros2_medkit_gateway::Component;
using ros2_medkit_gateway::DataProvider;
using ros2_medkit_gateway::FaultProvider;
using ros2_medkit_gateway::Function;
using ros2_medkit_gateway::IntrospectionProvider;
using ros2_medkit_gateway::LogProvider;
using ros2_medkit_gateway::OperationProvider;

static_assert(sizeof(Area) > 0);
static_assert(sizeof(Component) > 0);
static_assert(sizeof(App) > 0);
static_assert(sizeof(Function) > 0);
static_assert(std::is_abstract_v<DataProvider>);
static_assert(std::is_abstract_v<OperationProvider>);
static_assert(std::is_abstract_v<FaultProvider>);
static_assert(std::is_abstract_v<LogProvider>);
static_assert(std::is_abstract_v<IntrospectionProvider>);

}  // namespace

TEST(GatewayCoreSmoke, HeadersCompileAndLinkWithoutRos) {
  // The mere fact that this translation unit compiles and links against
  // gateway_core alone — with no ament_target_dependencies and no rclcpp
  // on the link line — proves the neutral layer carries no ROS coupling.
  // The static_asserts above pin the entity model and provider interface
  // contracts so the includes are exercised at compile time.
  SUCCEED();
}
