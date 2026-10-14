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

#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <rcl/types.h>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_interfaces/node_graph_interface.hpp>

// Header-only: the discovery plugins and their tests use it without the gateway's libraries.

namespace ros2_medkit_gateway::ros2_common {

/// One entry of `get_node_names_with_enclaves()`: name, namespace, enclave.
using GraphNodeEntry = std::tuple<std::string, std::string, std::string>;

/// The ROS graph's node list as a GraphNodeListReader reads it.
struct GraphNodeList {
  /// (name, namespace) of every entry kept, in graph order.
  std::vector<std::pair<std::string, std::string>> nodes;
  /// FQN of every node left out as a leftover, once each, in graph order.
  std::vector<std::string> leftovers;
};

/// Fully qualified name from the name and namespace the graph reports.
inline std::string graph_node_fqn(const std::string & name, const std::string & ns) {
  if (ns.empty() || ns == "/") {
    return "/" + name;
  }
  return ns + "/" + name;
}

/**
 * @brief Is this FQN one of the gateway's in-process helper nodes?
 *
 * True for `<self>_sub`, `<self>_fault_clients` and `<self>_lifecycle_state_reader`, exact matches
 * only. False for the gateway node itself, which is a diagnosable App. The last two take only the
 * gateway's node name, so they also match as `/<name><suffix>`; two gateways with one name in
 * different namespaces both claim those root-namespace nodes.
 *
 * @param node_fqn Fully qualified node name to test ("/ns/node")
 * @param self_fqn The gateway node's own FQN. An empty value matches nothing
 */
inline bool is_own_gateway_helper_node(const std::string & node_fqn, const std::string & self_fqn) {
  if (self_fqn.empty() || node_fqn.empty()) {
    return false;
  }
  struct HelperNode {
    const char * suffix;
    bool follows_gateway_namespace;
  };
  static constexpr std::array<HelperNode, 3> kHelperNodes{{
      {"_sub", true},
      {"_fault_clients", false},
      {"_lifecycle_state_reader", false},
  }};
  const auto last_slash = self_fqn.rfind('/');
  const std::string bare_name = last_slash == std::string::npos ? self_fqn : self_fqn.substr(last_slash + 1);
  return std::any_of(kHelperNodes.begin(), kHelperNodes.end(), [&](const HelperNode & helper) {
    if (node_fqn == self_fqn + helper.suffix) {
      return true;
    }
    return !helper.follows_gateway_namespace && !bare_name.empty() && node_fqn == "/" + bare_name + helper.suffix;
  });
}

/**
 * @brief Whether the ROS graph resolves any publisher or subscriber for a node.
 *
 * On the DDS RMWs (rmw_dds_common), without demangling these also cover services and clients (`rq/`, `rr/` topics).
 * A node the graph no longer lists has none. Other errors, such as a shut-down context, propagate.
 */
inline bool graph_node_has_endpoints(const rclcpp::node_interfaces::NodeGraphInterface & graph,
                                     const std::string & name, const std::string & ns) {
  try {
    return !graph.get_publisher_names_and_types_by_node(name, ns, true).empty() ||
           !graph.get_subscriber_names_and_types_by_node(name, ns, true).empty();
  } catch (const rclcpp::exceptions::RCLError & error) {
    if (error.ret == RCL_RET_NODE_NAME_NON_EXISTENT) {
      return false;
    }
    throw;
  }
}

/**
 * @brief Reads the ROS graph's node list, leaving out the leftovers of nodes it saw running.
 *
 * A leftover is an entry with an empty enclave and no endpoints that a late `ros_discovery_info`
 * sample puts back after its participant left. Nothing removes it again. An empty enclave alone is
 * not enough: micro-ROS and DDS-router nodes read the same, so only names this reader saw running
 * are hidden. Per FQN, on every read:
 * - an entry with an enclave is listed;
 * - an entry without an enclave is dropped when another entry of the name has one;
 * - an entry without an enclave for a departed name is left out when it has no endpoints;
 * - any other entry without an enclave is listed.
 *
 * A name that ran on the previous read and does not run now has departed. It is forgotten when it
 * runs again, or on the first read that finds no entry of it more than the hold after the first
 * read that found none. A listed entry without an enclave stops that clock. At most `capacity`
 * departed names are kept; the ones unlisted longest go first, then the ones departed longest ago.
 *
 * Thread-safe. A read holds the lock from the list query to the last endpoint query.
 */
class GraphNodeListReader {
 public:
  using Clock = std::chrono::steady_clock;

  /// How long after the first read without an entry a departed name is kept. It covers the listener
  /// thread's delay in taking a late sample, which exceeds 0.5 s on a loaded host.
  static constexpr std::chrono::seconds kDefaultHold{10};
  /// Most departed names kept. Each hidden leftover costs two endpoint queries per read.
  static constexpr std::size_t kDefaultCapacity = 1024;

  GraphNodeListReader() = default;
  GraphNodeListReader(Clock::duration hold, std::size_t capacity) : hold_(hold), capacity_(capacity) {
  }

  /**
   * @brief Read the node list, leaving out leftovers of nodes this reader saw running.
   *
   * Throws what the graph queries throw, for example once the context is shut down.
   */
  GraphNodeList read(const rclcpp::node_interfaces::NodeGraphInterface & graph) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto entries = graph.get_node_names_with_enclaves();
    return filter_locked(entries, Clock::now(), [&graph](const std::string & name, const std::string & ns) {
      return graph_node_has_endpoints(graph, name, ns);
    });
  }

  /// The decision read() makes, for `entries` read at `now`.
  /// @param has_endpoints `bool(const std::string & name, const std::string & ns)`
  template <typename HasEndpoints>
  GraphNodeList filter(const std::vector<GraphNodeEntry> & entries, Clock::time_point now,
                       HasEndpoints && has_endpoints) {
    std::lock_guard<std::mutex> lock(mutex_);
    return filter_locked(entries, now, std::forward<HasEndpoints>(has_endpoints));
  }

  /// How many departed names the reader keeps.
  std::size_t remembered() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return departed_.size();
  }

 private:
  /// A name the reader saw running that no entry with an enclave lists.
  struct Departed {
    /// The read that first found the name not running.
    Clock::time_point departed_at;
    /// First read of the current run that listed no entry of the name; empty while one is listed.
    std::optional<Clock::time_point> absent_since;
  };

  template <typename HasEndpoints>
  GraphNodeList filter_locked(const std::vector<GraphNodeEntry> & entries, Clock::time_point now,
                              HasEndpoints && has_endpoints) {
    std::unordered_set<std::string> running;
    std::unordered_set<std::string> listed;
    for (const auto & [name, ns, enclave] : entries) {
      auto fqn = graph_node_fqn(name, ns);
      if (!enclave.empty()) {
        running.insert(fqn);
      }
      listed.insert(std::move(fqn));
    }
    remember(std::move(running), listed, now);

    GraphNodeList result;
    result.nodes.reserve(entries.size());
    std::unordered_map<std::string, bool> resolved;
    for (const auto & [name, ns, enclave] : entries) {
      if (!enclave.empty()) {
        result.nodes.emplace_back(name, ns);
        continue;
      }
      auto fqn = graph_node_fqn(name, ns);
      if (running_.count(fqn) > 0) {
        continue;
      }
      if (departed_.count(fqn) == 0) {
        result.nodes.emplace_back(name, ns);
        continue;
      }
      auto found = resolved.find(fqn);
      if (found == resolved.end()) {
        const bool has = has_endpoints(name, ns);
        found = resolved.emplace(fqn, has).first;
        if (!has) {
          result.leftovers.push_back(fqn);
        }
      }
      if (found->second) {
        result.nodes.emplace_back(name, ns);
      }
    }
    return result;
  }

  void remember(std::unordered_set<std::string> running, const std::unordered_set<std::string> & listed,
                Clock::time_point now) {
    for (const auto & fqn : running) {
      departed_.erase(fqn);
    }
    for (const auto & fqn : running_) {
      if (running.count(fqn) == 0) {
        departed_.try_emplace(fqn, Departed{now, std::nullopt});
      }
    }
    running_ = std::move(running);
    for (auto it = departed_.begin(); it != departed_.end();) {
      auto & departed = it->second;
      if (listed.count(it->first) > 0) {
        departed.absent_since.reset();
      } else if (!departed.absent_since) {
        departed.absent_since = now;
      } else if (now - *departed.absent_since > hold_) {
        it = departed_.erase(it);
        continue;
      }
      ++it;
    }
    if (departed_.size() <= capacity_) {
      return;
    }
    // Drop the names unlisted longest first, then the ones that departed longest ago.
    std::vector<std::unordered_map<std::string, Departed>::iterator> order;
    order.reserve(departed_.size());
    for (auto it = departed_.begin(); it != departed_.end(); ++it) {
      order.push_back(it);
    }
    const auto excess = static_cast<std::ptrdiff_t>(departed_.size() - capacity_);
    std::nth_element(order.begin(), order.begin() + excess, order.end(), [](const auto & lhs, const auto & rhs) {
      const auto & a = lhs->second;
      const auto & b = rhs->second;
      if (a.absent_since.has_value() != b.absent_since.has_value()) {
        return a.absent_since.has_value();
      }
      if (a.absent_since) {
        return *a.absent_since < *b.absent_since;
      }
      return a.departed_at < b.departed_at;
    });
    for (auto it = order.begin(); it != order.begin() + excess; ++it) {
      departed_.erase(*it);
    }
  }

  Clock::duration hold_{kDefaultHold};
  std::size_t capacity_{kDefaultCapacity};
  mutable std::mutex mutex_;
  /// Names an entry with an enclave listed on the latest read.
  std::unordered_set<std::string> running_;
  /// Names seen running that no entry with an enclave lists since, at most `capacity_`.
  std::unordered_map<std::string, Departed> departed_;
};

/**
 * @brief Logs a WARN for each leftover on the first read of a run of reads that leave it out.
 *
 * Thread-safe: the refresh and HTTP handlers both read the node list.
 */
class LeftoverNodeReporter {
 public:
  explicit LeftoverNodeReporter(rclcpp::Logger logger) : logger_(std::move(logger)) {
  }

  /// Log the leftovers `list` leaves out that the previous read did not. Returns their FQNs.
  std::vector<std::string> report(const GraphNodeList & list) {
    std::vector<std::string> newly_left_out;
    std::set<std::string> current(list.leftovers.begin(), list.leftovers.end());
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & fqn : list.leftovers) {
      if (reported_.count(fqn) == 0) {
        newly_left_out.push_back(fqn);
        RCLCPP_WARN(logger_,
                    "Node '%s' is not exposed: this gateway saw it running, and the ROS graph still lists it "
                    "after its participant left, with no endpoints",
                    fqn.c_str());
      }
    }
    reported_ = std::move(current);
    return newly_left_out;
  }

 private:
  rclcpp::Logger logger_;
  std::mutex mutex_;
  std::set<std::string> reported_;
};

}  // namespace ros2_medkit_gateway::ros2_common
