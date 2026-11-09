// Copyright 2026 mfaferek93
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

// Node map entries addressed by browse_path: parsing, the walk that resolves
// them on a session, and how a resolved entry joins the map.

#include <gtest/gtest.h>

#include <cstdint>
#include <fstream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "ros2_medkit_opcua/address_space_browser.hpp"
#include "ros2_medkit_opcua/node_map.hpp"

namespace ros2_medkit_gateway {
namespace {

const char * kDi = "http://opcfoundation.org/UA/DI/";
const char * kRobotics = "http://opcfoundation.org/UA/Robotics/";

OpcuaClient::BrowseChild child(const std::string & node_id, uint16_t ns, const std::string & name,
                               opcua::NodeClass node_class = opcua::NodeClass::Object) {
  OpcuaClient::BrowseChild c;
  c.node_id = NodeMap::parse_node_id(node_id);
  c.browse_name_ns = ns;
  c.browse_name = name;
  c.display_name = name;
  c.node_class = node_class;
  return c;
}

class TreeSource : public AutoBrowseSource {
 public:
  void add(const std::string & parent, const OpcuaClient::BrowseChild & c) {
    tree_[NodeMap::parse_node_id(parent).toString()].push_back(c);
  }
  std::vector<OpcuaClient::BrowseChild> browse_children(const opcua::NodeId & parent) override {
    ++browse_calls;
    auto it = tree_.find(parent.toString());
    return it == tree_.end() ? std::vector<OpcuaClient::BrowseChild>{} : it->second;
  }
  std::string read_type_name(const opcua::NodeId & /*node_id*/) override {
    return "Boolean";
  }
  std::optional<OpcuaValue> read_initial_value(const opcua::NodeId & /*node_id*/) override {
    return OpcuaValue{true};
  }
  std::optional<bool> read_writable(const opcua::NodeId & /*node_id*/) override {
    return false;
  }
  int namespace_index(const std::string & uri) override {
    auto it = namespaces.find(uri);
    return it == namespaces.end() ? -1 : it->second;
  }

  std::map<std::string, int> namespaces;
  int browse_calls{0};

 private:
  std::unordered_map<std::string, std::vector<OpcuaClient::BrowseChild>> tree_;
};

// Objects/DeviceSet/<system>/MotionDevices/<robot>/... and a SafetyStates
// branch, the way OPC 40010-1 lays a robot out, with the server's own index
// for each namespace (DI=2, Robotics=3, vendor=4).
void add_robot(TreeSource & s, const std::string & robot) {
  s.namespaces = {{kDi, 2}, {kRobotics, 3}};
  s.add("i=85", child("ns=2;i=5001", 2, "DeviceSet"));
  s.add("ns=2;i=5001", child("ns=4;s=Cell", 4, "Cell"));
  s.add("ns=4;s=Cell", child("ns=4;s=Cell.MotionDevices", 3, "MotionDevices"));
  s.add("ns=4;s=Cell", child("ns=4;s=Cell.SafetyStates", 3, "SafetyStates"));
  s.add("ns=4;s=Cell.MotionDevices", child("ns=4;s=" + robot, 4, robot));
  s.add("ns=4;s=" + robot, child("ns=4;s=" + robot + ".SerialNumber", 2, "SerialNumber", opcua::NodeClass::Variable));
  s.add("ns=4;s=Cell.SafetyStates", child("ns=4;s=Safety", 4, "Safety"));
  s.add("ns=4;s=Safety", child("ns=4;s=Safety.ParameterSet", 2, "ParameterSet"));
  s.add("ns=4;s=Safety.ParameterSet",
        child("ns=4;s=Safety.EmergencyStop", 3, "EmergencyStop", opcua::NodeClass::Variable));
}

std::vector<BrowsePathElement> parse(const std::string & path) {
  std::vector<BrowsePathElement> out;
  std::string error;
  const std::map<std::string, std::string> aliases{{"di", kDi}, {"rob", kRobotics}};
  EXPECT_TRUE(NodeMap::parse_browse_path(path, aliases, out, error)) << error;
  return out;
}

}  // namespace

TEST(BrowsePathParse, AliasesIndicesAndWildcards) {
  auto path = parse("/di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop");
  ASSERT_EQ(path.size(), 6u);
  EXPECT_EQ(path[0].namespace_uri, kDi);
  EXPECT_EQ(path[0].name, "DeviceSet");
  EXPECT_TRUE(path[1].any);
  EXPECT_EQ(path[2].namespace_uri, kRobotics);
  EXPECT_EQ(path[5].name, "EmergencyStop");

  path = parse("3:Line\\/1/2:Level");
  ASSERT_EQ(path.size(), 2u);
  ASSERT_TRUE(path[0].namespace_index.has_value());
  EXPECT_EQ(*path[0].namespace_index, 3);
  EXPECT_EQ(path[0].name, "Line/1");
}

TEST(BrowsePathParse, RefusesWhatItCannotResolve) {
  const std::map<std::string, std::string> aliases{{"di", kDi}};
  for (const std::string bad :
       {"", "/", "di:DeviceSet/*", "nope:DeviceSet/di:X", "DeviceSet/di:X", "di:/di:X", "70000:X", "di:X\\"}) {
    std::vector<BrowsePathElement> out;
    std::string error;
    EXPECT_FALSE(NodeMap::parse_browse_path(bad, aliases, out, error)) << bad;
    EXPECT_FALSE(error.empty()) << bad;
    EXPECT_TRUE(out.empty()) << bad;
  }
}

TEST(BrowsePathResolve, WalksWildcardsToTheVariable) {
  TreeSource s;
  add_robot(s, "KR10_R1100");
  auto id = resolve_browse_path(s, parse("di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop"));
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->toString(), NodeMap::parse_node_id("ns=4;s=Safety.EmergencyStop").toString());

  id = resolve_browse_path(s, parse("di:DeviceSet/*/rob:MotionDevices/*/di:SerialNumber"));
  ASSERT_TRUE(id.has_value());
  EXPECT_EQ(id->toString(), NodeMap::parse_node_id("ns=4;s=KR10_R1100.SerialNumber").toString());
}

TEST(BrowsePathResolve, NamespaceIsMatchedByUriNotByName) {
  TreeSource s;
  add_robot(s, "R1");
  // The same server with the namespaces in another order: the old indices no
  // longer match, the URIs still do.
  s.namespaces = {{kDi, 3}, {kRobotics, 2}};
  EXPECT_FALSE(resolve_browse_path(s, parse("di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop")));
  s.namespaces = {{kDi, 2}};
  EXPECT_FALSE(resolve_browse_path(s, parse("di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop")));
}

TEST(BrowsePathResolve, LastStepMustBeAVariable) {
  TreeSource s;
  add_robot(s, "R1");
  EXPECT_FALSE(resolve_browse_path(s, parse("di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet")));
}

TEST(BrowsePathResolve, BacktracksPastADeadBranch) {
  // A first device without SafetyStates must not end the search.
  TreeSource t;
  t.namespaces = {{kDi, 2}, {kRobotics, 3}};
  t.add("i=85", child("ns=2;i=5001", 2, "DeviceSet"));
  t.add("ns=2;i=5001", child("ns=4;s=Other", 4, "Other"));
  t.add("ns=2;i=5001", child("ns=4;s=Cell", 4, "Cell"));
  t.add("ns=4;s=Cell", child("ns=4;s=Cell.SafetyStates", 3, "SafetyStates"));
  t.add("ns=4;s=Cell.SafetyStates", child("ns=4;s=Safety", 4, "Safety"));
  t.add("ns=4;s=Safety", child("ns=4;s=Safety.ParameterSet", 2, "ParameterSet"));
  t.add("ns=4;s=Safety.ParameterSet",
        child("ns=4;s=Safety.EmergencyStop", 3, "EmergencyStop", opcua::NodeClass::Variable));
  EXPECT_TRUE(resolve_browse_path(t, parse("di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop")));
}

TEST(BrowsePathResolve, StopsAtTheNodeBudget) {
  TreeSource s;
  add_robot(s, "R1");
  EXPECT_FALSE(resolve_browse_path(s, parse("di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop"), 3));
}

class BrowsePathNodeMap : public ::testing::Test {
 protected:
  bool load(const std::string & yaml) {
    path_ = ::testing::TempDir() + "browse_path_node_map.yaml";
    std::ofstream(path_) << yaml;
    return map_.load(path_);
  }
  std::string path_;
  NodeMap map_;
};

constexpr const char * kRobotMap = R"(
component_id: robot
namespaces:
  di: "http://opcfoundation.org/UA/DI/"
  rob: "http://opcfoundation.org/UA/Robotics/"
nodes:
  - browse_path: "di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop"
    entity_id: safety
    data_name: emergency_stop
    data_type: bool
    alarm: {fault_code: ROBOT_EMERGENCY_STOP, severity: ERROR, threshold: 0.5, above_threshold: true}
  - node_id: "ns=4;s=Plain"
    entity_id: cell
    data_name: plain
)";

TEST_F(BrowsePathNodeMap, WaitsOutsideTheEntriesUntilResolved) {
  ASSERT_TRUE(load(kRobotMap));
  ASSERT_EQ(map_.entries().size(), 1u);
  ASSERT_EQ(map_.unresolved_browse_entries().size(), 1u);
  EXPECT_EQ(map_.unresolved_browse_entries()[0].browse_path.size(), 6u);

  TreeSource s;
  add_robot(s, "R1");
  EXPECT_EQ(map_.resolve_browse_entries([&](const NodeMapEntry & e) {
                  return resolve_browse_path(s, e.browse_path);
                })
                .joined,
            1u);
  EXPECT_TRUE(map_.unresolved_browse_entries().empty());
  const auto * entry = map_.find_by_data_name("safety", "emergency_stop");
  ASSERT_NE(entry, nullptr);
  EXPECT_EQ(entry->node_id_str, NodeMap::parse_node_id("ns=4;s=Safety.EmergencyStop").toString());
  EXPECT_TRUE(entry->detection.has_value());
  EXPECT_EQ(map_.find_by_node_id(entry->node_id_str), entry);
  bool safety_entity = false;
  for (const auto & def : map_.entity_defs()) {
    safety_entity = safety_entity || (def.id == "safety" && def.has_faults);
  }
  EXPECT_TRUE(safety_entity);
}

TEST_F(BrowsePathNodeMap, AGateOnABrowsePathEntryJoinsWithIt) {
  std::string yaml = kRobotMap;
  const std::string rule = "threshold: 0.5, above_threshold: true}\n";
  yaml.insert(yaml.find(rule) + rule.size(), "    inhibit_when: {node_id: \"ns=4;s=Plain\", values: [2, 3]}\n");
  ASSERT_TRUE(load(yaml));
  ASSERT_EQ(map_.unresolved_browse_entries().size(), 1u);
  ASSERT_TRUE(map_.unresolved_browse_entries()[0].inhibit_when.has_value());

  TreeSource s;
  add_robot(s, "R1");
  map_.resolve_browse_entries([&](const NodeMapEntry & e) {
    return resolve_browse_path(s, e.browse_path);
  });
  const auto * entry = map_.find_by_data_name("safety", "emergency_stop");
  ASSERT_NE(entry, nullptr);
  ASSERT_TRUE(entry->inhibit_when.has_value());
  EXPECT_EQ(entry->inhibit_when->node_id_str, "ns=4;s=Plain");
  EXPECT_EQ(entry->inhibit_when->values, (std::vector<std::int64_t>{2, 3}));
}

TEST_F(BrowsePathNodeMap, RejectsAGateWhosePointIsMissingOrAddressedByBrowsePath) {
  std::string missing = kRobotMap;
  const std::string rule = "threshold: 0.5, above_threshold: true}\n";
  missing.insert(missing.find(rule) + rule.size(), "    only_when: {node_id: \"ns=4;s=Nope\", values: [1]}\n");
  EXPECT_FALSE(load(missing));

  std::string by_path = kRobotMap;
  by_path += R"(    alarm: {fault_code: PLAIN_HIGH, threshold: 1.0}
    only_when: {node_id: "browse_path=di:DeviceSet/*/rob:SafetyStates/*/di:ParameterSet/rob:EmergencyStop", values: [1]}
)";
  EXPECT_FALSE(load(by_path));
}

TEST_F(BrowsePathNodeMap, AnUnresolvedPathKeepsWaiting) {
  ASSERT_TRUE(load(kRobotMap));
  EXPECT_EQ(map_.resolve_browse_entries([](const NodeMapEntry &) {
                  return std::optional<opcua::NodeId>{};
                })
                .joined,
            0u);
  EXPECT_EQ(map_.unresolved_browse_entries().size(), 1u);
  EXPECT_EQ(map_.entries().size(), 1u);
}

TEST_F(BrowsePathNodeMap, TakesThePlaceOfAnAutoBrowsedEntry) {
  ASSERT_TRUE(load(kRobotMap));
  NodeMapEntry browsed;
  browsed.node_id = NodeMap::parse_node_id("ns=4;s=Safety.EmergencyStop");
  browsed.node_id_str = browsed.node_id.toString();
  browsed.entity_id = "cell_safety_parameterset";
  browsed.entity_display_name = "Cell / Safety / ParameterSet";
  browsed.data_name = "emergencystop";
  std::vector<NodeMapEntry> merged;
  merged.push_back(browsed);
  ASSERT_EQ(map_.merge_auto_browsed_entries(std::move(merged)), 1u);

  TreeSource s;
  add_robot(s, "R1");
  map_.resolve_browse_entries([&](const NodeMapEntry & e) {
    return resolve_browse_path(s, e.browse_path);
  });
  EXPECT_EQ(map_.entries().size(), 2u);
  EXPECT_NE(map_.find_by_data_name("safety", "emergency_stop"), nullptr);
  EXPECT_EQ(map_.find_by_data_name("cell_safety_parameterset", "emergencystop"), nullptr);
  EXPECT_TRUE(map_.entries_for_entity("cell_safety_parameterset").empty());
}

TEST_F(BrowsePathNodeMap, FollowsANodeThatMovedOnTheNextSession) {
  ASSERT_TRUE(load(kRobotMap));
  TreeSource s;
  add_robot(s, "R1");
  auto by_walk = [&](const NodeMapEntry & e) {
    return resolve_browse_path(s, e.browse_path);
  };
  EXPECT_EQ(map_.resolve_browse_entries(by_walk).joined, 1u);
  EXPECT_TRUE(map_.resolve_browse_entries(by_walk).moved.empty());

  // The server restarted with one more namespace in front: every instance
  // node now sits at ns=5.
  TreeSource restarted;
  restarted.namespaces = {{kDi, 2}, {kRobotics, 3}};
  restarted.add("i=85", child("ns=2;i=5001", 2, "DeviceSet"));
  restarted.add("ns=2;i=5001", child("ns=5;s=Cell", 5, "Cell"));
  restarted.add("ns=5;s=Cell", child("ns=5;s=Cell.SafetyStates", 3, "SafetyStates"));
  restarted.add("ns=5;s=Cell.SafetyStates", child("ns=5;s=Safety", 5, "Safety"));
  restarted.add("ns=5;s=Safety", child("ns=5;s=Safety.ParameterSet", 2, "ParameterSet"));
  restarted.add("ns=5;s=Safety.ParameterSet",
                child("ns=5;s=Safety.EmergencyStop", 3, "EmergencyStop", opcua::NodeClass::Variable));
  const auto old_id = NodeMap::parse_node_id("ns=4;s=Safety.EmergencyStop").toString();
  const auto new_id = NodeMap::parse_node_id("ns=5;s=Safety.EmergencyStop").toString();
  auto moved = map_.resolve_browse_entries([&](const NodeMapEntry & e) {
    return resolve_browse_path(restarted, e.browse_path);
  });
  ASSERT_EQ(moved.moved.size(), 1u);
  EXPECT_EQ(moved.moved[0].first, old_id);
  EXPECT_EQ(moved.moved[0].second, new_id);
  EXPECT_EQ(map_.find_by_node_id(old_id), nullptr);
  ASSERT_NE(map_.find_by_node_id(new_id), nullptr);
  EXPECT_EQ(map_.find_by_node_id(new_id)->data_name, "emergency_stop");

  // Not found on a later session: the entry keeps the node it has.
  EXPECT_TRUE(map_.resolve_browse_entries([](const NodeMapEntry &) {
                    return std::optional<opcua::NodeId>{};
                  })
                  .moved.empty());
  EXPECT_NE(map_.find_by_node_id(new_id), nullptr);
}

TEST_F(BrowsePathNodeMap, FaultCodesStayUniqueAcrossBothKinds) {
  EXPECT_FALSE(load(R"(
namespaces: {rob: "http://opcfoundation.org/UA/Robotics/"}
nodes:
  - browse_path: "rob:A/rob:B"
    entity_id: a
    data_name: b
    alarm: {fault_code: SAME, threshold: 1}
  - node_id: "ns=2;s=X"
    entity_id: x
    data_name: x
    alarm: {fault_code: SAME, threshold: 1}
)"));
}

TEST_F(BrowsePathNodeMap, OneAddressPerEntryAndKnownAliases) {
  ASSERT_TRUE(load(R"(
namespaces: {rob: "http://opcfoundation.org/UA/Robotics/"}
nodes:
  - node_id: "ns=2;s=X"
    browse_path: "rob:A/rob:B"
    entity_id: both
    data_name: x
  - browse_path: "zz:A/rob:B"
    entity_id: unknown_alias
    data_name: x
  - browse_path: "rob:A/rob:B"
    entity_id: ok
    data_name: x
)"));
  EXPECT_TRUE(map_.entries().empty());
  ASSERT_EQ(map_.unresolved_browse_entries().size(), 1u);
  EXPECT_EQ(map_.unresolved_browse_entries()[0].entity_id, "ok");

  EXPECT_FALSE(load("namespaces: [a, b]\nnodes: []\n"));
  EXPECT_FALSE(load("namespaces: {\"3\": \"urn:x\"}\nnodes: []\n"));
}

}  // namespace ros2_medkit_gateway
