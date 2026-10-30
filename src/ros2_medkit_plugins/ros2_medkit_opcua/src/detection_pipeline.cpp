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

#include <type_traits>
#include <utility>
#include <variant>

namespace ros2_medkit_gateway {

namespace fd = ros2_medkit::fault_detection;

void DetectionPipeline::record_sample(const std::string & node_id) {
  ++received_[node_id];
}

std::vector<DetectionPipeline::Change>
DetectionPipeline::evaluate(const std::vector<const NodeMapEntry *> & entries,
                            const std::unordered_map<std::string, OpcuaValue> & values, bool subscribed, Pass pass,
                            Clock::time_point now, std::unordered_map<std::string, bool> & alarms) {
  std::vector<Change> changes;
  std::unordered_map<std::string, std::uint64_t> evaluated;
  evaluated.reserve(entries.size());

  for (const auto * entry : entries) {
    auto it = values.find(entry->node_id_str);
    if (it == values.end()) {
      continue;
    }

    // Two entries may map one node. Both see the same value, so both judge it
    // against the count the node had before this pass.
    const auto rec = received_.find(entry->node_id_str);
    const std::uint64_t received = rec == received_.end() ? 0 : rec->second;
    const auto prev = evaluated_.find(entry->node_id_str);
    const bool arrived = prev == evaluated_.end() ? received != 0 : prev->second != received;
    evaluated[entry->node_id_str] = received;

    // Under polling a value that did not arrive is left over from a failed
    // read. Every value in the cache was evaluated when it arrived, so skipping
    // it changes nothing for a point without a qualifier, and keeps a waiting
    // point from counting it.
    const bool fresh = arrived || (subscribed && pass == Pass::kTick);
    if (!fresh && !subscribed) {
      continue;
    }

    auto signals =
        qualifier_.apply(fd::evaluate(to_fd_value(it->second), *entry->detection), entry->raise_qualifier, now, fresh);
    for (const auto & s : signals) {
      alarms[s.fault_code] = s.active;
    }
    for (auto & edge : tracker_.apply(signals)) {
      changes.push_back({entry->entity_id, std::move(edge)});
    }
  }

  evaluated_ = std::move(evaluated);
  return changes;
}

void DetectionPipeline::link_lost() {
  qualifier_.reset_pending();
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
