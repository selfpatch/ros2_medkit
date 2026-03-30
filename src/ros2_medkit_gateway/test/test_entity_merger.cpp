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

TEST(EntityMerger, components_prefix_on_collision) {
  EntityMerger merger("subsystem_b");

  auto local_comp = make_component("engine_ctrl", "powertrain");
  auto remote_comp = make_component("engine_ctrl", "powertrain");

  auto result = merger.merge_components({local_comp}, {remote_comp});

  // Both kept - local unchanged, remote prefixed
  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, "engine_ctrl");
  EXPECT_EQ(result[0].source, "node");  // local unchanged

  EXPECT_EQ(result[1].id, "subsystem_b__engine_ctrl");
  EXPECT_EQ(result[1].name, "subsystem_b__engine_ctrl");
  EXPECT_EQ(result[1].source, "peer:subsystem_b");
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

TEST(EntityMerger, apps_prefix_on_collision) {
  EntityMerger merger("subsystem_b");

  auto local_app = make_app("camera_driver", "perception_comp");
  auto remote_app = make_app("camera_driver", "perception_comp");

  auto result = merger.merge_apps({local_app}, {remote_app});

  ASSERT_EQ(result.size(), 2u);
  EXPECT_EQ(result[0].id, "camera_driver");
  EXPECT_EQ(result[0].source, "manifest");  // local unchanged

  EXPECT_EQ(result[1].id, "subsystem_b__camera_driver");
  EXPECT_EQ(result[1].name, "subsystem_b__camera_driver");
  EXPECT_EQ(result[1].source, "peer:subsystem_b");
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
}

// =============================================================================
// Routing table tests
// =============================================================================

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

TEST(EntityMerger, routing_table_uses_prefixed_id_on_collision) {
  EntityMerger merger("peer_x");

  // Same ID -> collision -> remote gets prefixed
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
