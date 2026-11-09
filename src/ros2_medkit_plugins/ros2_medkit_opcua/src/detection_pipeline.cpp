// Copyright 2026 ahmedsleem109
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

#include "ros2_medkit_opcua/detection_pipeline.hpp"

#include <algorithm>
#include <optional>
#include <type_traits>
#include <utility>
#include <variant>

namespace ros2_medkit_gateway {

namespace fd = ros2_medkit::fault_detection;

void DetectionPipeline::record_sample(const std::string & node_id) {
  auto & node = nodes_[node_id];
  ++node.received;
  node.seen_since_link = true;
}

std::vector<DetectionPipeline::Change>
DetectionPipeline::evaluate(const std::vector<const NodeMapEntry *> & entries,
                            const std::unordered_map<std::string, OpcuaValue> & values, bool subscribed, Pass pass,
                            Clock::time_point now, std::unordered_map<std::string, bool> & alarms) {
  std::vector<Change> changes;

  for (const auto * entry : entries) {
    const fd::RaiseQualifier qualifier =
        subscribed ? entry->raise_qualifier.for_change_notifications(poll_interval_) : entry->raise_qualifier;
    // A recheck can only end a wait in time, so only those entries need it.
    if (pass == Pass::kRecheck && qualifier.min_duration.count() == 0) {
      continue;
    }
    auto it = values.find(entry->node_id_str);
    if (it == values.end()) {
      continue;
    }

    // A value is new when its node's count moved since the last pass.
    const auto found = nodes_.find(entry->node_id_str);
    NodeSamples * node = found == nodes_.end() ? nullptr : &found->second;
    const std::uint64_t received = node ? node->received : 0;
    const bool arrived = node && node->evaluated != received;
    if (node) {
      node->evaluated = received;
    }

    // Under polling a value that did not arrive is left over from a failed
    // read. Every value in the cache was evaluated when it arrived, so skipping
    // it changes nothing for a point without a qualifier, and keeps a waiting
    // point from counting it. Under a subscription it is the current value and
    // lets a wait end, but it starts none.
    if (!arrived && !subscribed) {
      continue;
    }
    // Under a subscription the cached value only stands for the current one
    // once the node reported after the link came back.
    if (subscribed && !(node && node->seen_since_link)) {
      continue;
    }

    // State gates work on the rule's own verdict, before the qualifier, so a
    // gated fault does not wait toward a raise. An unread gate point gates
    // nothing: the rule then behaves as ungated.
    std::uint64_t gate_stamp = 0;
    auto gate_matches = [&](const std::optional<StateGate> & gate) -> std::optional<bool> {
      if (!gate) {
        return std::nullopt;
      }
      if (const auto n = nodes_.find(gate->node_id_str); n != nodes_.end()) {
        gate_stamp += n->second.received;
      }
      const auto g = values.find(gate->node_id_str);
      std::int64_t v = 0;
      if (g == values.end() || !fd::detail::as_int(to_fd_value(g->second), v)) {
        return std::nullopt;
      }
      return std::find(gate->values.begin(), gate->values.end(), v) != gate->values.end();
    };
    const auto only = gate_matches(entry->only_when);
    const auto inhibit = gate_matches(entry->inhibit_when);
    // A new value of a gate point is an observation of this point too: under a
    // subscription the cached value is the current one, so a wait may start.
    bool gate_arrived = false;
    if (entry->only_when || entry->inhibit_when) {
      auto & seen = gate_stamps_[entry->node_id_str];
      gate_arrived = seen != gate_stamp;
      seen = gate_stamp;
    }
    const bool fresh = arrived || gate_arrived;

    auto raw = fd::evaluate(to_fd_value(it->second), *entry->detection);
    for (auto & s : raw) {
      if (only.has_value() && !*only) {
        s.active = false;
      } else if (inhibit.value_or(false) && s.active) {
        // Not raised while inhibited; a fault raised before keeps its state.
        const auto prev = alarms.find(s.fault_code);
        s.active = prev != alarms.end() && prev->second;
      }
    }

    const auto signals = qualifier_.apply(raw, qualifier, now, fresh);
    for (const auto & s : signals) {
      alarms[s.fault_code] = s.active;
    }
    for (auto & edge : tracker_.apply(signals)) {
      changes.push_back({entry->entity_id, std::move(edge)});
    }
  }

  return changes;
}

void DetectionPipeline::link_lost() {
  qualifier_.reset_pending();
  for (auto & entry : nodes_) {
    entry.second.seen_since_link = false;
  }
}

bool DetectionPipeline::has_pending() const {
  return qualifier_.has_pending();
}

fd::Value DetectionPipeline::to_fd_value(const OpcuaValue & v) {
  return std::visit(
      [](auto && val) -> fd::Value {
        using T = std::decay_t<decltype(val)>;
        if constexpr (std::is_same_v<T, bool>) {
          return fd::Value{val};
        } else if constexpr (std::is_same_v<T, std::string>) {
          return fd::Value{val};
        } else if constexpr (std::is_floating_point_v<T>) {
          return fd::Value{static_cast<double>(val)};
        } else {
          return fd::Value{static_cast<std::int64_t>(val)};
        }
      },
      v);
}

}  // namespace ros2_medkit_gateway
