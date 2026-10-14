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

// The node-list reader's decisions. The test_graph_leftover_nodes*.test.py suites in
// ros2_medkit_integration_tests cover a real graph.

#include <gtest/gtest.h>

#include <chrono>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2_medkit_gateway/ros2_common/graph_node_list.hpp"

using ros2_medkit_gateway::ros2_common::graph_node_fqn;
using ros2_medkit_gateway::ros2_common::graph_node_has_endpoints;
using ros2_medkit_gateway::ros2_common::GraphNodeEntry;
using ros2_medkit_gateway::ros2_common::GraphNodeList;
using ros2_medkit_gateway::ros2_common::GraphNodeListReader;
using ros2_medkit_gateway::ros2_common::LeftoverNodeReporter;

namespace {

using NamePair = std::pair<std::string, std::string>;
using Clock = GraphNodeListReader::Clock;
using std::chrono::seconds;

/// Endpoint lookup over a fixed set of FQNs that resolve endpoints, counting every question asked.
struct EndpointTable {
  std::set<std::string> with_endpoints;
  std::map<std::string, int> asked;

  auto probe() {
    return [this](const std::string & name, const std::string & ns) {
      const auto fqn = graph_node_fqn(name, ns);
      ++asked[fqn];
      return with_endpoints.count(fqn) > 0;
    };
  }
};

const Clock::time_point kStart{};

GraphNodeEntry running(const std::string & name, const std::string & ns) {
  return GraphNodeEntry{name, ns, "/"};
}

GraphNodeEntry no_enclave(const std::string & name, const std::string & ns) {
  return GraphNodeEntry{name, ns, ""};
}

}  // namespace

TEST(GraphNodeList, FqnOfRootAndNestedNamespaces) {
  EXPECT_EQ(graph_node_fqn("a", "/"), "/a");
  EXPECT_EQ(graph_node_fqn("a", ""), "/a");
  EXPECT_EQ(graph_node_fqn("a", "/ns/sub"), "/ns/sub/a");
}

TEST(GraphNodeListReader, EntryWithEnclaveIsListedWithoutAnEndpointQuery) {
  GraphNodeListReader reader;
  EndpointTable table;
  auto list = reader.filter({running("quiet", "/ns")}, kStart, table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"quiet", "/ns"}}));
  EXPECT_TRUE(list.leftovers.empty());
  EXPECT_TRUE(table.asked.empty());
  EXPECT_EQ(reader.remembered(), 0u);
}

TEST(GraphNodeListReader, EntryWithoutEnclaveOfANodeNeverSeenRunningIsListedWithoutAQuery) {
  GraphNodeListReader reader;
  EndpointTable table;
  auto list = reader.filter({no_enclave("far", "/router")}, kStart, table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"far", "/router"}}));
  EXPECT_TRUE(list.leftovers.empty());
  EXPECT_TRUE(table.asked.empty());
  EXPECT_EQ(reader.remembered(), 0u);
}

TEST(GraphNodeListReader, LeftoverOfANodeSeenRunningIsLeftOutUnlessItHasEndpoints) {
  GraphNodeListReader reader;
  EndpointTable table;
  reader.filter({running("gone", "/ns"), running("bridged", "/ns")}, kStart, table.probe());

  table.with_endpoints = {"/ns/bridged"};
  auto list =
      reader.filter({no_enclave("gone", "/ns"), no_enclave("bridged", "/ns")}, kStart + seconds(1), table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"bridged", "/ns"}}));
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/ns/gone"}));
  EXPECT_EQ(table.asked, (std::map<std::string, int>{{"/ns/bridged", 1}, {"/ns/gone", 1}}));
}

TEST(GraphNodeListReader, EntryWithoutEnclaveNextToOneWithAnEnclaveIsDroppedWithoutAQuery) {
  GraphNodeListReader reader;
  EndpointTable table;
  auto list = reader.filter({no_enclave("node", "/"), running("node", "/")}, kStart, table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"node", "/"}}));
  EXPECT_TRUE(list.leftovers.empty());
  EXPECT_TRUE(table.asked.empty());
}

TEST(GraphNodeListReader, EndpointsAreAskedOncePerNameAndGraphOrderIsKept) {
  GraphNodeListReader reader;
  EndpointTable table;
  reader.filter({running("x", "/a"), running("x", "/b")}, kStart, table.probe());
  table.with_endpoints = {"/b/x"};
  auto list = reader.filter(
      {no_enclave("x", "/a"), running("y", "/"), no_enclave("x", "/b"), no_enclave("x", "/a"), no_enclave("x", "/b")},
      kStart + seconds(1), table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"y", "/"}, {"x", "/b"}, {"x", "/b"}}));
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/a/x"}));
  EXPECT_EQ(table.asked, (std::map<std::string, int>{{"/a/x", 1}, {"/b/x", 1}}));
}

TEST(GraphNodeListReader, AbsenceIsCountedFromTheFirstReadThatFindsTheNameAbsent) {
  GraphNodeListReader reader(seconds(10), 16);
  EndpointTable table;
  reader.filter({running("n", "/")}, kStart, table.probe());
  // The next read comes long after the last one that listed the node; the absence starts here.
  reader.filter({}, kStart + seconds(60), table.probe());
  reader.filter({}, kStart + seconds(69), table.probe());
  auto list =
      reader.filter({no_enclave("n", "/")}, kStart + seconds(69) + std::chrono::milliseconds(500), table.probe());
  EXPECT_TRUE(list.nodes.empty());
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/n"}));
}

TEST(GraphNodeListReader, NameAbsentLongerThanTheHoldIsForgottenAndItsLeftoverListed) {
  GraphNodeListReader reader(seconds(10), 16);
  EndpointTable table;
  reader.filter({running("n", "/")}, kStart, table.probe());
  reader.filter({}, kStart + seconds(1), table.probe());
  reader.filter({}, kStart + seconds(11), table.probe());
  EXPECT_EQ(reader.remembered(), 1u);
  reader.filter({}, kStart + seconds(12), table.probe());
  EXPECT_EQ(reader.remembered(), 0u);
  auto list = reader.filter({no_enclave("n", "/")}, kStart + seconds(13), table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"n", "/"}}));
  EXPECT_TRUE(list.leftovers.empty());
  EXPECT_TRUE(table.asked.empty());
}

TEST(GraphNodeListReader, ListedLeftoverIsRememberedForAsLongAsItIsListed) {
  GraphNodeListReader reader(seconds(10), 16);
  EndpointTable table;
  reader.filter({running("n", "/")}, kStart, table.probe());
  for (int i = 1; i <= 100; ++i) {
    auto list = reader.filter({no_enclave("n", "/")}, kStart + seconds(i), table.probe());
    ASSERT_EQ(list.leftovers, (std::vector<std::string>{"/n"})) << "read " << i;
  }
}

TEST(GraphNodeListReader, NamesRunningNowDoNotCountAgainstTheCapacity) {
  GraphNodeListReader reader(seconds(100), 2);
  EndpointTable table;
  std::vector<GraphNodeEntry> graph;
  graph.reserve(9);
  for (int i = 0; i < 8; ++i) {
    graph.push_back(running("live_" + std::to_string(i), "/"));
  }
  graph.push_back(running("gone", "/"));
  reader.filter(graph, kStart, table.probe());
  reader.filter(graph, kStart + seconds(1), table.probe());
  EXPECT_EQ(reader.remembered(), 0u);

  graph.back() = no_enclave("gone", "/");
  for (int i = 2; i <= 20; ++i) {
    auto list = reader.filter(graph, kStart + seconds(i), table.probe());
    ASSERT_EQ(list.leftovers, (std::vector<std::string>{"/gone"})) << "read " << i;
    ASSERT_EQ(list.nodes.size(), 8u) << "read " << i;
  }
  EXPECT_EQ(reader.remembered(), 1u);
}

TEST(GraphNodeListReader, ANameThatRunsAgainIsNoLongerDeparted) {
  GraphNodeListReader reader(seconds(100), 16);
  EndpointTable table;
  reader.filter({running("n", "/")}, kStart, table.probe());
  reader.filter({no_enclave("n", "/")}, kStart + seconds(1), table.probe());
  EXPECT_EQ(reader.remembered(), 1u);
  auto list = reader.filter({no_enclave("n", "/"), running("n", "/")}, kStart + seconds(2), table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"n", "/"}}));
  EXPECT_EQ(reader.remembered(), 0u);
  // It departs again: the leftover of the earlier run is left out once more.
  list = reader.filter({no_enclave("n", "/")}, kStart + seconds(3), table.probe());
  EXPECT_TRUE(list.nodes.empty());
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/n"}));
}

TEST(GraphNodeListReader, PastTheCapacityTheNameUnlistedLongestIsForgottenFirst) {
  GraphNodeListReader reader(seconds(100), 3);
  EndpointTable table;
  reader.filter({running("listed", "/"), running("early", "/"), running("late", "/"), running("new", "/")}, kStart,
                table.probe());
  // listed departs first and stays listed through a leftover; early and late depart with no entry left.
  reader.filter({no_enclave("listed", "/"), running("early", "/"), running("late", "/"), running("new", "/")},
                kStart + seconds(1), table.probe());
  reader.filter({no_enclave("listed", "/"), running("late", "/"), running("new", "/")}, kStart + seconds(2),
                table.probe());
  reader.filter({no_enclave("listed", "/"), running("new", "/")}, kStart + seconds(3), table.probe());
  EXPECT_EQ(reader.remembered(), 3u);

  // new departs past the capacity: early has had no entry for longest, so it is forgotten first,
  // although listed departed before it.
  auto list = reader.filter({no_enclave("listed", "/"), no_enclave("new", "/")}, kStart + seconds(4), table.probe());
  EXPECT_EQ(reader.remembered(), 3u);
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/listed", "/new"}));
  list = reader.filter(
      {no_enclave("listed", "/"), no_enclave("early", "/"), no_enclave("late", "/"), no_enclave("new", "/")},
      kStart + seconds(5), table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"early", "/"}}));
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/listed", "/late", "/new"}));
}

TEST(GraphNodeListReader, PastTheCapacityAmongListedNamesTheOneThatDepartedLongestAgoIsForgottenFirst) {
  GraphNodeListReader reader(seconds(100), 2);
  EndpointTable table;
  reader.filter({running("old", "/")}, kStart, table.probe());
  reader.filter({no_enclave("old", "/"), running("mid", "/")}, kStart + seconds(1), table.probe());
  reader.filter({no_enclave("old", "/"), no_enclave("mid", "/"), running("new", "/")}, kStart + seconds(2),
                table.probe());
  EXPECT_EQ(reader.remembered(), 2u);

  auto list = reader.filter({no_enclave("old", "/"), no_enclave("mid", "/"), no_enclave("new", "/")},
                            kStart + seconds(3), table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"old", "/"}}));
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/mid", "/new"}));
}

TEST(GraphNodeListReader, PastTheCapacityANameThatDepartsWithNoEntryLeftIsForgottenBeforeListedLeftovers) {
  GraphNodeListReader reader(seconds(100), 2);
  EndpointTable table;
  reader.filter({running("a", "/"), running("b", "/"), running("c", "/")}, kStart, table.probe());
  reader.filter({no_enclave("a", "/"), no_enclave("b", "/"), running("c", "/")}, kStart + seconds(1), table.probe());
  // c departs and the graph lists no entry of it: of three departed names it is the one no entry lists.
  auto list = reader.filter({no_enclave("a", "/"), no_enclave("b", "/")}, kStart + seconds(2), table.probe());
  EXPECT_EQ(reader.remembered(), 2u);
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/a", "/b"}));
  list = reader.filter({no_enclave("a", "/"), no_enclave("b", "/"), no_enclave("c", "/")}, kStart + seconds(3),
                       table.probe());
  EXPECT_EQ(list.nodes, (std::vector<NamePair>{{"c", "/"}}));
  EXPECT_EQ(list.leftovers, (std::vector<std::string>{"/a", "/b"}));
}

TEST(LeftoverNodeReporter, ReportsOnTheReadWhereANodeStartsBeingLeftOut) {
  LeftoverNodeReporter reporter(rclcpp::get_logger("test_graph_node_list"));
  const GraphNodeList left_out{{}, {"/ns/gone"}};
  const GraphNodeList listed{{{"gone", "/ns"}}, {}};
  const GraphNodeList absent{};

  EXPECT_EQ(reporter.report(left_out), (std::vector<std::string>{"/ns/gone"}));
  EXPECT_TRUE(reporter.report(left_out).empty());
  EXPECT_TRUE(reporter.report(left_out).empty());

  EXPECT_TRUE(reporter.report(listed).empty());
  EXPECT_EQ(reporter.report(left_out), (std::vector<std::string>{"/ns/gone"}));

  EXPECT_TRUE(reporter.report(absent).empty());
  EXPECT_EQ(reporter.report(left_out), (std::vector<std::string>{"/ns/gone"}));
}

// Against a real rcl graph: the one rclcpp error that means "the graph no longer lists this node"
// counts as no endpoints, and the error a shut-down context raises propagates.
TEST(GraphNodeHasEndpoints, NodeTheGraphDoesNotListHasNoneAndAShutDownContextThrows) {
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("_graph_node_list_probe", rclcpp::NodeOptions().context(context));
  const auto graph = node->get_node_graph_interface();

  EXPECT_FALSE(graph_node_has_endpoints(*graph, "no_such_node", "/no_such_namespace"));
  EXPECT_TRUE(graph_node_has_endpoints(*graph, node->get_name(), node->get_namespace()));

  context->shutdown("test");
  EXPECT_THROW(graph_node_has_endpoints(*graph, "no_such_node", "/no_such_namespace"), rclcpp::exceptions::RCLError);
}
