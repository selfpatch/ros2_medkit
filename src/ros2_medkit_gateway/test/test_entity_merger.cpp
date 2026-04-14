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

#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/aggregation/entity_merger.hpp"

using namespace ros2_medkit_gateway;

// =============================================================================
// Helper factories - C++17 compatible (no designated initializers)
// =============================================================================

static Area make_area(const std::string & id, const std::string & name = "", const std::string & source = "") {
  Area a;
  a.id = id;
  a.name = name.empty() ? id : name;
  a.source = source;
  return a;
}

static Function make_function(const std::string & id, const std::vector<std::string> & hosts,
                              const std::string & source = "manifest") {
  Function f;
  f.id = id;
  f.name = id;
  f.hosts = hosts;
  f.source = source;
  return f;
}

static Component make_component(const std::string & id, const std::string & area = "",
                                const std::string & source = "node") {
  Component c;
  c.id = id;
  c.name = id;
  c.area = area;
  c.source = source;
  return c;
}

static App make_app(const std::string & id, const std::string & component_id = "",
                    const std::string & source = "manifest") {
  App a;
  a.id = id;
  a.name = id;
  a.component_id = component_id;
  a.source = source;
  return a;
}

// =============================================================================
// Area merge tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(EntityMerger, areas_merge_by_id) {
  EntityMerger merger("peer_a");

  Area local_area = make_area("powertrain", "Powertrain System");
  local_area.tags = {"engine", "transmission"};

  Area remote_area = make_area("powertrain", "Powertrain");
  remote_area.tags = {"engine", "drivetrain"};  // "engine" overlaps

  auto result = merger.merge_areas({local_area}, {remote_area});

  // Same ID -> one entity
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].id, "powertrain");
  // Name stays as local
  EXPECT_EQ(result[0].name, "Powertrain System");

  // Tags merged without duplicates: engine, transmission, drivetrain
  EXPECT_EQ(result[0].tags.size(), 3u);
  auto has_tag = [&](const std::string & tag) {
    return std::find(result[0].tags.begin(), result[0].tags.end(), tag) != result[0].tags.end();
  };
  EXPECT_TRUE(has_tag("engine"));
  EXPECT_TRUE(has_tag("transmission"));
  EXPECT_TRUE(has_tag("drivetrain"));
}

TEST(EntityMerger, areas_no_collision_both_kept) {
  EntityMerger merger("peer_a");

  auto local_area = make_area("powertrain");
  auto remote_area = make_area("chassis");

  auto result = merger.merge_areas({local_area}, {remote_area});

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, "powertrain");
  EXPECT_EQ(result[1].id, "chassis");
  // Remote-only area gets source tagged
  EXPECT_EQ(result[1].source, "peer:peer_a");
}

TEST(EntityMerger, areas_merge_takes_remote_description_when_local_empty) {
  EntityMerger merger("peer_a");

  Area local_area = make_area("powertrain");
  // local has no description

  Area remote_area = make_area("powertrain");
  remote_area.description = "Powertrain system for engine and drivetrain";

  auto result = merger.merge_areas({local_area}, {remote_area});

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].description, "Powertrain system for engine and drivetrain");
}

// =============================================================================
// Function merge tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(EntityMerger, functions_merge_by_id_combining_hosts) {
  EntityMerger merger("peer_b");

  auto local_func = make_function("navigation", {"planner_app", "controller_app"});
  auto remote_func = make_function("navigation", {"controller_app", "localization_app"});

  auto result = merger.merge_functions({local_func}, {remote_func});

  // Same ID -> one function
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].id, "navigation");

  // Hosts combined without duplicates: planner_app, controller_app, localization_app
  EXPECT_EQ(result[0].hosts.size(), 3u);
  auto has_host = [&](const std::string & host) {
    return std::find(result[0].hosts.begin(), result[0].hosts.end(), host) != result[0].hosts.end();
  };
  EXPECT_TRUE(has_host("planner_app"));
  EXPECT_TRUE(has_host("controller_app"));
  EXPECT_TRUE(has_host("localization_app"));
}

TEST(EntityMerger, functions_no_collision_both_kept) {
  EntityMerger merger("peer_b");

  auto local_func = make_function("navigation", {"planner_app"});
  auto remote_func = make_function("perception", {"camera_app"});

  auto result = merger.merge_functions({local_func}, {remote_func});

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, "navigation");
  EXPECT_EQ(result[1].id, "perception");
  EXPECT_EQ(result[1].source, "peer:peer_b");
}

// =============================================================================
// Component merge tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(EntityMerger, components_merge_by_id) {
  EntityMerger merger("subsystem_b");

  auto local_comp = make_component("engine_ctrl", "powertrain");
  local_comp.tags = {"engine", "control"};

  auto remote_comp = make_component("engine_ctrl", "powertrain");
  remote_comp.tags = {"control", "ecu"};

  auto result = merger.merge_components({local_comp}, {remote_comp});

  // Same ID -> one entity (merged)
  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].id, "engine_ctrl");
  EXPECT_EQ(result[0].source, "node");  // local source preserved

  // Tags merged without duplicates: engine, control, ecu
  EXPECT_EQ(result[0].tags.size(), 3u);
  auto has_tag = [&](const std::string & tag) {
    return std::find(result[0].tags.begin(), result[0].tags.end(), tag) != result[0].tags.end();
  };
  EXPECT_TRUE(has_tag("engine"));
  EXPECT_TRUE(has_tag("control"));
  EXPECT_TRUE(has_tag("ecu"));
}

TEST(EntityMerger, components_merge_takes_remote_description_when_local_empty) {
  EntityMerger merger("subsystem_b");

  auto local_comp = make_component("engine_ctrl", "powertrain");
  // local has no description

  auto remote_comp = make_component("engine_ctrl", "powertrain");
  remote_comp.description = "Engine control unit for powertrain management";

  auto result = merger.merge_components({local_comp}, {remote_comp});

  ASSERT_EQ(result.size(), 1u);
  EXPECT_EQ(result[0].description, "Engine control unit for powertrain management");
}

TEST(EntityMerger, components_no_collision_no_prefix) {
  EntityMerger merger("subsystem_b");

  auto local_comp = make_component("engine_ctrl");
  auto remote_comp = make_component("brake_ctrl");

  auto result = merger.merge_components({local_comp}, {remote_comp});

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, "engine_ctrl");
  EXPECT_EQ(result[1].id, "brake_ctrl");  // No prefix needed
  EXPECT_EQ(result[1].source, "peer:subsystem_b");
}

// =============================================================================
// App merge tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(EntityMerger, apps_prefix_on_collision) {
  EntityMerger merger("subsystem_b");

  auto local_app = make_app("camera_driver", "perception_comp");
  auto remote_app = make_app("camera_driver", "perception_comp");

  auto result = merger.merge_apps({local_app}, {remote_app});

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, "camera_driver");
  EXPECT_EQ(result[0].source, "manifest");               // local unchanged
  EXPECT_EQ(result[0].component_id, "perception_comp");  // local keeps original

  EXPECT_EQ(result[1].id, "subsystem_b__camera_driver");
  EXPECT_EQ(result[1].name, "subsystem_b__camera_driver");
  EXPECT_EQ(result[1].source, "peer:subsystem_b");
  // Remote app preserves its original component_id (the remote component
  // is merged or added separately, so the reference remains valid)
  EXPECT_EQ(result[1].component_id, "perception_comp");
}

// @verifies REQ_INTEROP_003
TEST(EntityMerger, apps_collision_sets_original_id) {
  EntityMerger merger("subsystem_b");

  auto local_app = make_app("camera_driver", "perception_comp");
  auto remote_app = make_app("camera_driver", "perception_comp");

  auto result = merger.merge_apps({local_app}, {remote_app});

  ASSERT_EQ(result.size(), 2u);
  // Local app should not have original_id set
  EXPECT_TRUE(result[0].original_id.empty());
  // Collision-renamed remote app should preserve original_id
  EXPECT_EQ(result[1].id, "subsystem_b__camera_driver");
  EXPECT_EQ(result[1].original_id, "camera_driver");
}

TEST(EntityMerger, apps_no_collision_does_not_set_original_id) {
  EntityMerger merger("subsystem_b");

  auto local_app = make_app("planner");
  auto remote_app = make_app("localizer");

  auto result = merger.merge_apps({local_app}, {remote_app});

  ASSERT_EQ(result.size(), 2u);
  // No collision - original_id should be empty for both
  EXPECT_TRUE(result[0].original_id.empty());
  EXPECT_TRUE(result[1].original_id.empty());
}

TEST(EntityMerger, no_collision_no_prefix) {
  EntityMerger merger("subsystem_b");

  auto local_app = make_app("planner");
  auto remote_app = make_app("localizer");

  auto result = merger.merge_apps({local_app}, {remote_app});

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, "planner");
  EXPECT_EQ(result[1].id, "localizer");  // No prefix
  EXPECT_EQ(result[1].source, "peer:subsystem_b");
  // Remote app preserves its original component_id (empty when not set)
  EXPECT_EQ(result[1].component_id, "");
}

// =============================================================================
// Routing table tests
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(EntityMerger, builds_routing_table_for_remote_entities) {
  EntityMerger merger("peer_x");

  // Remote-only app (no collision) should get routing entry
  auto local_apps = std::vector<App>{make_app("local_app")};
  auto remote_apps = std::vector<App>{make_app("remote_app")};

  merger.merge_apps(local_apps, remote_apps);

  const auto & table = merger.get_routing_table();
  ASSERT_EQ(table.count("remote_app"), 1u);
  EXPECT_EQ(table.at("remote_app"), "peer_x");
}

TEST(EntityMerger, routing_table_uses_prefixed_id_on_app_collision) {
  EntityMerger merger("peer_x");

  // Same ID -> collision -> remote gets prefixed (Apps use prefix strategy)
  auto local_apps = std::vector<App>{make_app("shared_app")};
  auto remote_apps = std::vector<App>{make_app("shared_app")};

  merger.merge_apps(local_apps, remote_apps);

  const auto & table = merger.get_routing_table();

  // The routing table should use the prefixed ID
  ASSERT_EQ(table.count("peer_x__shared_app"), 1u);
  EXPECT_EQ(table.at("peer_x__shared_app"), "peer_x");

  // The original ID should NOT be in the routing table (it's local)
  EXPECT_EQ(table.count("shared_app"), 0u);
}

TEST(EntityMerger, merged_components_route_to_peer) {
  EntityMerger merger("peer_x");

  // Same component ID across peers refers to the same physical ECU, but only
  // the peer owns the runtime state (data, logs, hosts, operations). The
  // collision-merged entity must route to the peer so sub-resource requests
  // reach the owning gateway instead of returning empty locally.
  auto local_comps = std::vector<Component>{make_component("robot-alpha")};
  auto remote_comps = std::vector<Component>{make_component("robot-alpha")};

  merger.merge_components(local_comps, remote_comps);

  const auto & table = merger.get_routing_table();
  ASSERT_EQ(table.count("robot-alpha"), 1u);
  EXPECT_EQ(table.at("robot-alpha"), "peer_x");
}

// @verifies REQ_INTEROP_003
TEST(EntityMerger, merged_component_hybrid_synthetic_collision_routes_to_peer) {
  // Scenario: primary in hybrid mode creates a synthetic component from its
  // namespace (source "node"), and a peer announces a real component with the
  // same ID. The peer is the authoritative owner of runtime state - all
  // sub-resource requests (/logs, /hosts, /data, /operations) must forward
  // to the peer instead of being handled locally.
  EntityMerger merger("ecu_peer");

  auto local_synthetic = make_component("engine_ctrl", "powertrain", "node");
  auto remote_real = make_component("engine_ctrl", "powertrain", "manifest");

  merger.merge_components({local_synthetic}, {remote_real});

  const auto & table = merger.get_routing_table();
  ASSERT_EQ(table.count("engine_ctrl"), 1u);
  EXPECT_EQ(table.at("engine_ctrl"), "ecu_peer");
}

TEST(EntityMerger, remote_only_component_gets_routing_entry) {
  EntityMerger merger("peer_x");

  auto local_comps = std::vector<Component>{make_component("engine_ctrl")};
  auto remote_comps = std::vector<Component>{make_component("brake_ctrl")};

  merger.merge_components(local_comps, remote_comps);

  const auto & table = merger.get_routing_table();
  ASSERT_EQ(table.count("brake_ctrl"), 1u);
  EXPECT_EQ(table.at("brake_ctrl"), "peer_x");
}

TEST(EntityMerger, merged_areas_not_in_routing_table) {
  EntityMerger merger("peer_x");

  // Same area ID -> merged, should NOT appear in routing table
  auto local_areas = std::vector<Area>{make_area("powertrain")};
  auto remote_areas = std::vector<Area>{make_area("powertrain")};

  merger.merge_areas(local_areas, remote_areas);

  const auto & table = merger.get_routing_table();
  EXPECT_EQ(table.count("powertrain"), 0u);
}

TEST(EntityMerger, merged_functions_not_in_routing_table) {
  EntityMerger merger("peer_x");

  auto local_funcs = std::vector<Function>{make_function("navigation", {"app_a"})};
  auto remote_funcs = std::vector<Function>{make_function("navigation", {"app_b"})};

  merger.merge_functions(local_funcs, remote_funcs);

  const auto & table = merger.get_routing_table();
  EXPECT_EQ(table.count("navigation"), 0u);
}

TEST(EntityMerger, remote_only_area_gets_routing_entry) {
  EntityMerger merger("peer_x");

  auto local_areas = std::vector<Area>{make_area("powertrain")};
  auto remote_areas = std::vector<Area>{make_area("chassis")};

  merger.merge_areas(local_areas, remote_areas);

  const auto & table = merger.get_routing_table();
  ASSERT_EQ(table.count("chassis"), 1u);
  EXPECT_EQ(table.at("chassis"), "peer_x");
}

// =============================================================================
// Source tagging tests
// =============================================================================

// =============================================================================
// Contributors provenance tests
// =============================================================================

TEST(EntityMerger, components_collision_appends_peer_contributor) {
  EntityMerger merger("peer_b");

  auto local_comp = make_component("robot-alpha");
  local_comp.contributors = {"local"};
  auto remote_comp = make_component("robot-alpha");

  auto result = merger.merge_components({local_comp}, {remote_comp});

  ASSERT_EQ(result.size(), 1u);
  ASSERT_EQ(result[0].contributors.size(), 2u);
  EXPECT_EQ(result[0].contributors[0], "local");
  EXPECT_EQ(result[0].contributors[1], "peer:peer_b");
}

TEST(EntityMerger, components_remote_only_gets_peer_contributor_only) {
  EntityMerger merger("peer_b");

  auto remote_comp = make_component("ecu-c");

  auto result = merger.merge_components({}, {remote_comp});

  ASSERT_EQ(result.size(), 1u);
  ASSERT_EQ(result[0].contributors.size(), 1u);
  EXPECT_EQ(result[0].contributors[0], "peer:peer_b");
}

TEST(EntityMerger, apps_collision_prefixed_gets_peer_contributor_only) {
  EntityMerger merger("peer_b");

  auto local_app = make_app("camera");
  local_app.contributors = {"local"};
  auto remote_app = make_app("camera");

  auto result = merger.merge_apps({local_app}, {remote_app});

  ASSERT_EQ(result.size(), 2u);
  // Local keeps "local"
  ASSERT_EQ(result[0].contributors.size(), 1u);
  EXPECT_EQ(result[0].contributors[0], "local");
  // Prefixed remote gets only peer contributor
  ASSERT_EQ(result[1].contributors.size(), 1u);
  EXPECT_EQ(result[1].contributors[0], "peer:peer_b");
}

TEST(EntityMerger, contributors_no_duplicate_on_repeat_merge) {
  // If the same peer is merged twice (defensive check), contributors must
  // stay unique.
  EntityMerger merger("peer_b");

  auto local_area = make_area("root");
  local_area.contributors = {"local"};
  auto remote_area = make_area("root");

  auto first = merger.merge_areas({local_area}, {remote_area});
  auto second = merger.merge_areas(first, {remote_area});

  ASSERT_EQ(second.size(), 1u);
  ASSERT_EQ(second[0].contributors.size(), 2u);
  EXPECT_EQ(second[0].contributors[0], "local");
  EXPECT_EQ(second[0].contributors[1], "peer:peer_b");
}

// =============================================================================
// Source tagging tests
// =============================================================================

TEST(EntityMerger, remote_source_tagged) {
  EntityMerger merger("robot_arm");

  // Test across all entity types
  auto areas = merger.merge_areas({}, {make_area("workspace")});
  ASSERT_EQ(areas.size(), 1u);
  EXPECT_EQ(areas[0].source, "peer:robot_arm");

  auto funcs = merger.merge_functions({}, {make_function("grasp", {"gripper_app"})});
  ASSERT_EQ(funcs.size(), 1u);
  EXPECT_EQ(funcs[0].source, "peer:robot_arm");

  auto comps = merger.merge_components({}, {make_component("joint_ctrl")});
  ASSERT_EQ(comps.size(), 1u);
  EXPECT_EQ(comps[0].source, "peer:robot_arm");

  auto apps = merger.merge_apps({}, {make_app("trajectory_planner")});
  ASSERT_EQ(apps.size(), 1u);
  EXPECT_EQ(apps[0].source, "peer:robot_arm");
}

// =============================================================================
// Edge cases
// =============================================================================

TEST(EntityMerger, empty_local_returns_remote_only) {
  EntityMerger merger("peer_z");

  auto areas = merger.merge_areas({}, {make_area("remote_area")});
  ASSERT_EQ(areas.size(), 1u);
  EXPECT_EQ(areas[0].id, "remote_area");
  EXPECT_EQ(areas[0].source, "peer:peer_z");
}

TEST(EntityMerger, empty_remote_returns_local_only) {
  EntityMerger merger("peer_z");

  auto areas = merger.merge_areas({make_area("local_area")}, {});
  ASSERT_EQ(areas.size(), 1u);
  EXPECT_EQ(areas[0].id, "local_area");
}

TEST(EntityMerger, both_empty_returns_empty) {
  EntityMerger merger("peer_z");

  auto areas = merger.merge_areas({}, {});
  EXPECT_TRUE(areas.empty());

  auto funcs = merger.merge_functions({}, {});
  EXPECT_TRUE(funcs.empty());

  auto comps = merger.merge_components({}, {});
  EXPECT_TRUE(comps.empty());

  auto apps = merger.merge_apps({}, {});
  EXPECT_TRUE(apps.empty());
}

TEST(EntityMerger, multiple_remote_entities_all_routed) {
  EntityMerger merger("peer_m");

  auto local_apps = std::vector<App>{make_app("local_only")};
  auto remote_apps = std::vector<App>{make_app("remote_a"), make_app("remote_b")};

  merger.merge_apps(local_apps, remote_apps);

  const auto & table = merger.get_routing_table();
  EXPECT_EQ(table.size(), 2u);
  EXPECT_EQ(table.at("remote_a"), "peer_m");
  EXPECT_EQ(table.at("remote_b"), "peer_m");
}

TEST(EntityMerger, separator_constant_is_double_underscore) {
  EXPECT_STREQ(EntityMerger::SEPARATOR, "__");
}

TEST(EntityMerger, routing_table_accumulates_across_merge_calls) {
  EntityMerger merger("peer_acc");

  merger.merge_apps({}, {make_app("app_a")});
  merger.merge_components({}, {make_component("comp_b")});
  merger.merge_areas({}, {make_area("area_c")});
  merger.merge_functions({}, {make_function("func_d", {"app_x"})});

  const auto & table = merger.get_routing_table();
  EXPECT_EQ(table.size(), 4u);
  EXPECT_EQ(table.count("app_a"), 1u);
  EXPECT_EQ(table.count("comp_b"), 1u);
  EXPECT_EQ(table.count("area_c"), 1u);
  EXPECT_EQ(table.count("func_d"), 1u);
}

// =============================================================================
// Multi-peer overlapping entity IDs
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(EntityMerger, multi_peer_overlapping_app_ids_no_data_loss) {
  // Scenario: local gateway has "camera_driver", two different peers also have
  // an app named "camera_driver". After merging both peers sequentially (as
  // AggregationManager does), all three apps must exist with correct routing.
  auto local_app = make_app("camera_driver", "perception");

  // --- Merge peer_a's camera_driver ---
  EntityMerger merger_a("peer_a");
  auto remote_a = make_app("camera_driver", "sensors");
  auto after_a = merger_a.merge_apps({local_app}, {remote_a});

  // Two apps now: local "camera_driver" and prefixed "peer_a__camera_driver"
  ASSERT_EQ(after_a.size(), 2u);
  EXPECT_EQ(after_a[0].id, "camera_driver");
  EXPECT_EQ(after_a[0].component_id, "perception");  // local unchanged
  EXPECT_EQ(after_a[1].id, "peer_a__camera_driver");
  EXPECT_EQ(after_a[1].original_id, "camera_driver");
  EXPECT_EQ(after_a[1].source, "peer:peer_a");

  // Routing table for peer_a: prefixed ID routes to peer_a
  const auto & table_a = merger_a.get_routing_table();
  ASSERT_EQ(table_a.count("peer_a__camera_driver"), 1u);
  EXPECT_EQ(table_a.at("peer_a__camera_driver"), "peer_a");

  // --- Merge peer_b's camera_driver into the accumulated result ---
  EntityMerger merger_b("peer_b");
  auto remote_b = make_app("camera_driver", "vision");
  auto after_b = merger_b.merge_apps(after_a, {remote_b});

  // Three apps now: local, peer_a__camera_driver, and peer_b__camera_driver.
  // peer_b collides with local "camera_driver" and gets prefixed.
  ASSERT_EQ(after_b.size(), 3u);
  EXPECT_EQ(after_b[0].id, "camera_driver");
  EXPECT_EQ(after_b[0].component_id, "perception");  // local still unchanged
  EXPECT_TRUE(after_b[0].original_id.empty());       // local never renamed
  EXPECT_EQ(after_b[1].id, "peer_a__camera_driver");
  EXPECT_EQ(after_b[2].id, "peer_b__camera_driver");
  EXPECT_EQ(after_b[2].original_id, "camera_driver");
  EXPECT_EQ(after_b[2].source, "peer:peer_b");

  // Routing table for peer_b: prefixed ID routes to peer_b
  const auto & table_b = merger_b.get_routing_table();
  ASSERT_EQ(table_b.count("peer_b__camera_driver"), 1u);
  EXPECT_EQ(table_b.at("peer_b__camera_driver"), "peer_b");

  // Verify no data loss: all three apps have distinct IDs
  std::unordered_set<std::string> all_ids;
  for (const auto & app : after_b) {
    all_ids.insert(app.id);
  }
  EXPECT_EQ(all_ids.size(), 3u);
  EXPECT_EQ(all_ids.count("camera_driver"), 1u);
  EXPECT_EQ(all_ids.count("peer_a__camera_driver"), 1u);
  EXPECT_EQ(all_ids.count("peer_b__camera_driver"), 1u);

  // Combined routing table from both mergers covers both peers
  std::unordered_map<std::string, std::string> combined_routing;
  for (const auto & [id, name] : table_a) {
    combined_routing[id] = name;
  }
  for (const auto & [id, name] : table_b) {
    combined_routing[id] = name;
  }
  EXPECT_EQ(combined_routing.size(), 2u);
  EXPECT_EQ(combined_routing.at("peer_a__camera_driver"), "peer_a");
  EXPECT_EQ(combined_routing.at("peer_b__camera_driver"), "peer_b");
}
