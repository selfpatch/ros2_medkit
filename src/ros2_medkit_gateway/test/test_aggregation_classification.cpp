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

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "ros2_medkit_gateway/core/aggregation/classification.hpp"
#include "ros2_medkit_gateway/core/discovery/models/component.hpp"

using namespace ros2_medkit_gateway;

namespace {

Component make_comp(const std::string & id, const std::string & parent_id = "") {
  Component c;
  c.id = id;
  c.name = id;
  c.parent_component_id = parent_id;
  return c;
}

PeerClaim make_claim(const std::string & peer, std::initializer_list<std::string> ids) {
  PeerClaim pc;
  pc.peer_name = peer;
  for (const auto & id : ids) {
    pc.claimed_entity_ids.insert(id);
  }
  return pc;
}

}  // namespace

// =============================================================================
// Leaf ECU Component: collision routes to peer
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, leaf_collision_keeps_routing_to_peer) {
  // Two peers each with an ECU-level leaf "ecu-x" that collides with primary's
  // local one. No Component references "ecu-x" as its parent -> leaf -> stays
  // in routing table (peer authoritative for runtime state).
  std::vector<Component> merged = {
      make_comp("ecu-x"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"ecu-x"}),
  };

  auto result = classify_component_routing(merged, claims);

  ASSERT_EQ(result.routing_table.count("ecu-x"), 1u);
  EXPECT_EQ(result.routing_table.at("ecu-x"), "peer_b");
  EXPECT_TRUE(result.leaf_warnings.empty());
}

// =============================================================================
// Hierarchical parent Component: collision stays local (merged view)
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, hierarchical_parent_drops_routing_when_local_subcomponents_exist) {
  // Primary declares "robot" as parent of its local "perception-ecu".
  // Peer also declares "robot" (collision). Because "perception-ecu" in the
  // merged set references "robot" as parent, "robot" is a hierarchical
  // parent -> removed from routing table (served locally with merged view).
  std::vector<Component> merged = {
      make_comp("robot"),
      make_comp("perception-ecu", "robot"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"robot"}),
  };

  auto result = classify_component_routing(merged, claims);

  EXPECT_EQ(result.routing_table.count("robot"), 0u);
  EXPECT_TRUE(result.leaf_warnings.empty());
}

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, hierarchical_parent_drops_routing_when_remote_subcomponents_exist) {
  // Primary has only "robot" (no local children). Peer brings both "robot"
  // (collision) and "planning-ecu" with parent_component_id=robot. After
  // merge, "planning-ecu" in the set references "robot" -> still
  // hierarchical parent -> no routing entry.
  std::vector<Component> merged = {
      make_comp("robot"),
      make_comp("planning-ecu", "robot"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"robot", "planning-ecu"}),
  };

  auto result = classify_component_routing(merged, claims);

  EXPECT_EQ(result.routing_table.count("robot"), 0u);
  // The sub-component is peer-owned -> routes to peer_b.
  ASSERT_EQ(result.routing_table.count("planning-ecu"), 1u);
  EXPECT_EQ(result.routing_table.at("planning-ecu"), "peer_b");
  EXPECT_TRUE(result.leaf_warnings.empty());
}

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, hierarchical_parent_drops_routing_across_multiple_peers) {
  // multi_ecu_aggregation demo case: primary + 2 peers all declare "robot"
  // as parent; each peer brings its own ECU-leaf child. "robot" stays local,
  // each ECU-leaf routes to the peer that contributed it.
  std::vector<Component> merged = {
      make_comp("robot"), make_comp("perception-ecu", "robot"),  // local
      make_comp("planning-ecu", "robot"),                        // from peer_b
      make_comp("actuation-ecu", "robot"),                       // from peer_c
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"robot", "planning-ecu"}),
      make_claim("peer_c", {"robot", "actuation-ecu"}),
  };

  auto result = classify_component_routing(merged, claims);

  // "robot" is a hierarchical parent - no routing entry even with 2 peers
  // colliding on it.
  EXPECT_EQ(result.routing_table.count("robot"), 0u);
  // Each ECU-leaf routes to its owning peer.
  ASSERT_EQ(result.routing_table.count("planning-ecu"), 1u);
  EXPECT_EQ(result.routing_table.at("planning-ecu"), "peer_b");
  ASSERT_EQ(result.routing_table.count("actuation-ecu"), 1u);
  EXPECT_EQ(result.routing_table.at("actuation-ecu"), "peer_c");
  // Multi-peer collision on "robot" is EXPECTED (hierarchical) and must NOT
  // produce a warning.
  EXPECT_TRUE(result.leaf_warnings.empty());
}

// =============================================================================
// Sub-components of hierarchical parent still route
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, subcomponents_of_hierarchical_parent_still_route_to_peer) {
  // Confirms that removing routing for a hierarchical parent does NOT cascade
  // to its sub-components - those remain peer-owned leaves.
  std::vector<Component> merged = {
      make_comp("robot"),
      make_comp("planning-ecu", "robot"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"robot", "planning-ecu"}),
  };

  auto result = classify_component_routing(merged, claims);

  EXPECT_EQ(result.routing_table.count("robot"), 0u);
  ASSERT_EQ(result.routing_table.count("planning-ecu"), 1u);
  EXPECT_EQ(result.routing_table.at("planning-ecu"), "peer_b");
}

// =============================================================================
// Multi-peer leaf collision: warning + last-writer-wins
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, leaf_collision_across_multiple_peers_emits_warning) {
  // Two peers both claim the same leaf ECU "ecu-shared" (deployment anomaly -
  // two peers exposing the same physical ECU). Routing uses last-writer; a
  // structured warning lists all colliding peers.
  std::vector<Component> merged = {
      make_comp("ecu-shared"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"ecu-shared"}),
      make_claim("peer_c", {"ecu-shared"}),
  };

  auto result = classify_component_routing(merged, claims);

  // Routing exists - last-writer-wins (peer_c processed last here).
  ASSERT_EQ(result.routing_table.count("ecu-shared"), 1u);
  EXPECT_EQ(result.routing_table.at("ecu-shared"), "peer_c");

  // Warning enumerates every peer that claimed the leaf (order-agnostic).
  ASSERT_EQ(result.leaf_warnings.size(), 1u);
  const auto & w = result.leaf_warnings.front();
  EXPECT_EQ(w.entity_id, "ecu-shared");
  std::unordered_set<std::string> claimants(w.peer_names.begin(), w.peer_names.end());
  EXPECT_EQ(claimants.count("peer_b"), 1u);
  EXPECT_EQ(claimants.count("peer_c"), 1u);
  EXPECT_EQ(claimants.size(), 2u);
}

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, leaf_collision_routing_respects_input_order) {
  // Flipped input: peer_b last -> routing must resolve to peer_b. Guards
  // against regressions that sort peer_claims before classification and thus
  // break the documented last-writer-wins contract.
  std::vector<Component> merged = {
      make_comp("ecu-shared"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_c", {"ecu-shared"}),
      make_claim("peer_b", {"ecu-shared"}),
  };

  auto result = classify_component_routing(merged, claims);

  ASSERT_EQ(result.routing_table.count("ecu-shared"), 1u);
  EXPECT_EQ(result.routing_table.at("ecu-shared"), "peer_b");
  ASSERT_EQ(result.leaf_warnings.size(), 1u);
  EXPECT_EQ(result.leaf_warnings.front().entity_id, "ecu-shared");
}

// =============================================================================
// Malformed parent_component_id: self, nonexistent, cycle
// =============================================================================

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, parent_references_nonexistent_component_falls_back_to_leaf) {
  // Peer declares a Component whose parent_component_id points at an ID not
  // present anywhere in the merged set. Prior behaviour silently excluded the
  // ghost ID from routing while leaving the child as a routed leaf; that
  // caused a dangling parent pointer in the serialised child response and no
  // operator signal. The classifier now ignores the ghost edge and records a
  // diagnostic so the misconfiguration surfaces via RCLCPP_WARN.
  std::vector<Component> merged = {
      make_comp("child", "ghost-parent"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"child"}),
  };

  auto result = classify_component_routing(merged, claims);

  ASSERT_EQ(result.routing_table.count("child"), 1u);
  EXPECT_EQ(result.routing_table.at("child"), "peer_b");
  EXPECT_EQ(result.routing_table.count("ghost-parent"), 0u);
  ASSERT_EQ(result.malformed_parent_warnings.size(), 1u);
  EXPECT_NE(result.malformed_parent_warnings.front().find("ghost-parent"), std::string::npos);
  EXPECT_NE(result.malformed_parent_warnings.front().find("non-existent"), std::string::npos);
}

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, self_parent_is_ignored_and_warned) {
  // A Component declaring itself as parent would otherwise be excluded from
  // the routing table AND never render children (it has none), effectively
  // disappearing into the local merged cache. Treat it as a leaf and warn.
  std::vector<Component> merged = {
      make_comp("orphan", "orphan"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"orphan"}),
  };

  auto result = classify_component_routing(merged, claims);

  ASSERT_EQ(result.routing_table.count("orphan"), 1u);
  EXPECT_EQ(result.routing_table.at("orphan"), "peer_b");
  ASSERT_EQ(result.malformed_parent_warnings.size(), 1u);
  EXPECT_NE(result.malformed_parent_warnings.front().find("orphan"), std::string::npos);
  EXPECT_NE(result.malformed_parent_warnings.front().find("itself"), std::string::npos);
}

// @verifies REQ_INTEROP_003
TEST(AggregationClassification, two_way_parent_cycle_falls_back_to_leaves_with_warning) {
  // A<->B cycle: prior behaviour marked both A and B as hierarchical parents
  // of each other, excluded both from the routing table, then served empty
  // local stubs instead of forwarding to the owning peer. The classifier now
  // detects the cycle, drops both edges, and emits a single diagnostic
  // listing the cycle members.
  std::vector<Component> merged = {
      make_comp("node-a", "node-b"),
      make_comp("node-b", "node-a"),
  };
  std::vector<PeerClaim> claims = {
      make_claim("peer_b", {"node-a", "node-b"}),
  };

  auto result = classify_component_routing(merged, claims);

  ASSERT_EQ(result.routing_table.count("node-a"), 1u);
  EXPECT_EQ(result.routing_table.at("node-a"), "peer_b");
  ASSERT_EQ(result.routing_table.count("node-b"), 1u);
  EXPECT_EQ(result.routing_table.at("node-b"), "peer_b");
  ASSERT_EQ(result.malformed_parent_warnings.size(), 1u);
  const auto & w = result.malformed_parent_warnings.front();
  EXPECT_NE(w.find("cycle"), std::string::npos);
  EXPECT_NE(w.find("node-a"), std::string::npos);
  EXPECT_NE(w.find("node-b"), std::string::npos);
}
