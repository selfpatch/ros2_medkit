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

// Standalone contract test for ServiceActionResolver.
//
// ServiceActionResolver is the abstract lookup interface that decouples
// OperationManager from DiscoveryManager. Until now it was only exercised
// transitively through OperationManager routing tests. This file pins the
// abstract contract independently so future implementors (e.g. an
// aggregation-aware resolver) have an explicit specification:
//
//   - find_service / find_action MUST return std::nullopt when no match
//     exists (the contract permits absence).
//   - When a match exists, the resolver MUST return a populated
//     ServiceInfo / ActionInfo value (the contract permits success).
//
// The test links only gateway_core - no rclcpp, no ament transitive deps -
// proving the interface is genuinely transport-neutral.

#include "ros2_medkit_gateway/core/discovery/service_action_resolver.hpp"

#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <unordered_map>
#include <utility>

#include "ros2_medkit_gateway/core/discovery/models/common.hpp"

namespace ros2_medkit_gateway {
namespace {

// Minimal in-memory stub. The key (component_ns, operation_name) pair maps to
// a fully populated ServiceInfo / ActionInfo, mirroring how DiscoveryManager
// would expose runtime-discovered services and actions.
class StubServiceActionResolver : public ServiceActionResolver {
 public:
  StubServiceActionResolver() = default;
  ~StubServiceActionResolver() override = default;

  std::optional<ServiceInfo> find_service(const std::string & component_ns,
                                          const std::string & operation_name) const override {
    auto it = services_.find(key(component_ns, operation_name));
    if (it == services_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  std::optional<ActionInfo> find_action(const std::string & component_ns,
                                        const std::string & operation_name) const override {
    auto it = actions_.find(key(component_ns, operation_name));
    if (it == actions_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  void add_service(const std::string & component_ns, const std::string & operation_name, ServiceInfo info) {
    services_.emplace(key(component_ns, operation_name), std::move(info));
  }

  void add_action(const std::string & component_ns, const std::string & operation_name, ActionInfo info) {
    actions_.emplace(key(component_ns, operation_name), std::move(info));
  }

 private:
  static std::string key(const std::string & component_ns, const std::string & operation_name) {
    return component_ns + "::" + operation_name;
  }

  std::unordered_map<std::string, ServiceInfo> services_;
  std::unordered_map<std::string, ActionInfo> actions_;
};

TEST(ServiceActionResolverContract, ReturnsNulloptForUnknownPaths) {
  StubServiceActionResolver resolver;

  EXPECT_FALSE(resolver.find_service("/powertrain/engine", "calibrate").has_value());
  EXPECT_FALSE(resolver.find_action("/navigation", "navigate_to_pose").has_value());
}

TEST(ServiceActionResolverContract, ReturnsPopulatedInfoForKnownPaths) {
  StubServiceActionResolver resolver;

  ServiceInfo svc;
  svc.name = "calibrate";
  svc.full_path = "/powertrain/engine/calibrate";
  svc.type = "std_srvs/srv/Trigger";
  resolver.add_service("/powertrain/engine", "calibrate", svc);

  ActionInfo act;
  act.name = "navigate_to_pose";
  act.full_path = "/navigation/navigate_to_pose";
  act.type = "nav2_msgs/action/NavigateToPose";
  resolver.add_action("/navigation", "navigate_to_pose", act);

  auto resolved_svc = resolver.find_service("/powertrain/engine", "calibrate");
  ASSERT_TRUE(resolved_svc.has_value());
  EXPECT_EQ(resolved_svc->name, "calibrate");
  EXPECT_EQ(resolved_svc->full_path, "/powertrain/engine/calibrate");
  EXPECT_EQ(resolved_svc->type, "std_srvs/srv/Trigger");

  auto resolved_act = resolver.find_action("/navigation", "navigate_to_pose");
  ASSERT_TRUE(resolved_act.has_value());
  EXPECT_EQ(resolved_act->name, "navigate_to_pose");
  EXPECT_EQ(resolved_act->full_path, "/navigation/navigate_to_pose");
  EXPECT_EQ(resolved_act->type, "nav2_msgs/action/NavigateToPose");
}

TEST(ServiceActionResolverContract, ServiceAndActionNamespacesAreIndependent) {
  // The contract treats services and actions as separate namespaces - adding
  // a service named "X" must not satisfy a find_action("X") request, and
  // vice versa. This pins the orthogonality of the two lookup methods so a
  // future implementation cannot accidentally collapse them into one table.
  StubServiceActionResolver resolver;

  ServiceInfo svc;
  svc.name = "shared_name";
  svc.full_path = "/ns/shared_name";
  svc.type = "std_srvs/srv/Trigger";
  resolver.add_service("/ns", "shared_name", svc);

  EXPECT_TRUE(resolver.find_service("/ns", "shared_name").has_value());
  EXPECT_FALSE(resolver.find_action("/ns", "shared_name").has_value());
}

}  // namespace
}  // namespace ros2_medkit_gateway
