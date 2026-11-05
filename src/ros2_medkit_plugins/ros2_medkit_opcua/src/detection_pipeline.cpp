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
    // A recheck can only end a raise_after_ms wait, so only those entries
    // need it.
    if (pass == Pass::kRecheck && entry->raise_qualifier.min_duration.count() == 0) {
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

    // Under a subscription a tick stands for one more sample of a value that
    // holds; how often it may count is decided per fault below.
    const bool tick_sample = subscribed && pass == Pass::kTick;

    // Under polling a value that did not arrive is left over from a failed
    // read. Every value in the cache was evaluated when it arrived, so skipping
    // it changes nothing for a point without a qualifier, and keeps a waiting
    // point from counting it.
    const bool fresh = arrived || tick_sample;
    if (!fresh && !subscribed) {
      continue;
    }
    // Under a subscription the cached value only stands for the current one
    // once the node reported after the link came back.
    if (subscribed && !(node && node->seen_since_link)) {
      continue;
    }

    auto evaluated = fd::evaluate(to_fd_value(it->second), *entry->detection);
    std::vector<fd::FaultSignal> signals;
    if (entry->raise_qualifier.immediate() || !subscribed || min_spacing_.count() <= 0) {
      signals = qualifier_.apply(evaluated, entry->raise_qualifier, now, fresh);
    } else {
      // Under a subscription a change that starts a wait counts at once, and
      // while the wait is open a fault counts at most one sample per spacing,
      // so two quick changes, or a change and the tick right after it, are one
      // sample, as under polling. Each fault of a status word keeps its own
      // spacing.
      signals.reserve(evaluated.size());
      for (const auto & s : evaluated) {
        bool counts = fresh;
        if (counts && s.active && qualifier_.pending(s.fault_code)) {
          const auto last = last_counted_.find(s.fault_code);
          counts = last == last_counted_.end() || now - last->second >= min_spacing_;
        }
        auto out = qualifier_.apply({s}, entry->raise_qualifier, now, counts);
        if (counts && s.active) {
          last_counted_[s.fault_code] = now;
        }
        signals.insert(signals.end(), out.begin(), out.end());
      }
    }
    for (const auto & s : signals) {
      alarms[s.fault_code] = s.active;
      if (entry->raise_qualifier.min_duration.count() > 0) {
        duration_codes_.insert(s.fault_code);
      }
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

bool DetectionPipeline::has_pending_duration() const {
  for (const auto & code : duration_codes_) {
    if (qualifier_.pending(code)) {
      return true;
    }
  }
  return false;
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
