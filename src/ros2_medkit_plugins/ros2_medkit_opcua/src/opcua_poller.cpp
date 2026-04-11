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

#include "ros2_medkit_opcua/opcua_poller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

namespace ros2_medkit_gateway {

OpcuaPoller::OpcuaPoller(OpcuaClient & client, const NodeMap & node_map) : client_(client), node_map_(node_map) {
}

OpcuaPoller::~OpcuaPoller() {
  stop();
}

void OpcuaPoller::start(const PollerConfig & config) {
  if (running_.load()) {
    return;
  }

  config_ = config;
  running_ = true;

  // Try subscription mode first
  if (config_.prefer_subscriptions) {
    setup_subscriptions();
  }

  // Start poll/reconnect thread regardless (handles reconnection and poll fallback)
  poll_thread_ = std::thread(&OpcuaPoller::poll_loop, this);
}

void OpcuaPoller::stop() {
  running_ = false;
  if (poll_thread_.joinable()) {
    poll_thread_.join();
  }
  client_.remove_subscriptions();
}

PollSnapshot OpcuaPoller::snapshot() const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  return snapshot_;
}

std::optional<OpcuaValue> OpcuaPoller::get_value(const std::string & node_id_str) const {
  std::lock_guard<std::mutex> lock(snapshot_mutex_);
  auto it = snapshot_.values.find(node_id_str);
  if (it != snapshot_.values.end()) {
    return it->second;
  }
  return std::nullopt;
}

void OpcuaPoller::set_alarm_callback(AlarmChangeCallback callback) {
  std::lock_guard<std::mutex> lock(alarm_mutex_);
  alarm_callback_ = std::move(callback);
}

void OpcuaPoller::set_poll_callback(PollCallback callback) {
  assert(!running_.load() && "poll_callback_ must be set before start()");
  poll_callback_ = std::move(callback);
}

void OpcuaPoller::setup_subscriptions() {
  auto sub_id = client_.create_subscription(config_.subscription_interval_ms,
                                            [this](const std::string & nid, const OpcuaValue & val) {
                                              on_data_change(nid, val);
                                            });

  if (sub_id == 0) {
    using_subscriptions_ = false;
    return;
  }

  bool all_ok = true;
  for (const auto & entry : node_map_.entries()) {
    if (!client_.add_monitored_item(sub_id, entry.node_id)) {
      all_ok = false;
      break;
    }
  }

  if (all_ok) {
    using_subscriptions_ = true;
  } else {
    client_.remove_subscriptions();
    using_subscriptions_ = false;
  }
}

void OpcuaPoller::on_data_change(const std::string & node_id, const OpcuaValue & value) {
  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.values[node_id] = value;
    snapshot_.timestamp = std::chrono::system_clock::now();
    snapshot_.connected = true;
    snapshot_.poll_count++;
  }
  evaluate_alarms();
}

void OpcuaPoller::poll_loop() {
  while (running_.load()) {
    // Handle reconnection
    if (!client_.is_connected()) {
      {
        std::lock_guard<std::mutex> lock(snapshot_mutex_);
        snapshot_.connected = false;
      }

      // Attempt reconnect with original config (preserves timeout, etc.)
      if (client_.connect(client_.current_config())) {
        if (config_.prefer_subscriptions) {
          setup_subscriptions();
        }
      } else {
        // Wait before retry using configured interval
        auto wait_ms = config_.reconnect_interval.count();
        for (int64_t i = 0; i < wait_ms / 100 && running_.load(); ++i) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        continue;
      }
    }

    // Poll if not using subscriptions, or as a health check
    if (!using_subscriptions_.load() || !client_.is_connected()) {
      do_poll();
    }

    // Fire poll callback for value bridging.
    // Called every cycle regardless of transport mode (poll or subscription)
    // so that ROS 2 publishers always receive updates.
    if (poll_callback_) {
      poll_callback_(snapshot());
    }

    // Sleep for poll interval
    auto sleep_ms = config_.poll_interval.count();
    for (int64_t i = 0; i < sleep_ms / 100 && running_.load(); ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

void OpcuaPoller::do_poll() {
  std::vector<opcua::NodeId> node_ids;
  node_ids.reserve(node_map_.entries().size());
  for (const auto & entry : node_map_.entries()) {
    node_ids.push_back(entry.node_id);
  }

  auto results = client_.read_values(node_ids);

  {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);
    snapshot_.timestamp = std::chrono::system_clock::now();
    snapshot_.connected = client_.is_connected();
    snapshot_.poll_count++;

    for (const auto & r : results) {
      if (r.good) {
        snapshot_.values[r.node_id] = r.value;
      } else {
        snapshot_.error_count++;
      }
    }
  }

  evaluate_alarms();
}

void OpcuaPoller::evaluate_alarms() {
  auto alarm_entries = node_map_.alarm_entries();
  if (alarm_entries.empty()) {
    return;
  }

  // Collect state changes while holding snapshot mutex
  struct AlarmChange {
    std::string fault_code;
    AlarmConfig config;
    bool active;
  };
  std::vector<AlarmChange> changes;

  {
    std::lock_guard<std::mutex> snap_lock(snapshot_mutex_);

    for (const auto * entry : alarm_entries) {
      auto it = snapshot_.values.find(entry->node_id_str);
      if (it == snapshot_.values.end()) {
        continue;
      }

      const auto & alarm = *entry->alarm;
      bool active = false;

      std::visit(
          [&active, &alarm](auto && val) {
            using T = std::decay_t<decltype(val)>;
            if constexpr (std::is_same_v<T, bool>) {
              active = val;
            } else if constexpr (std::is_arithmetic_v<T>) {
              double dval = static_cast<double>(val);
              if (alarm.above_threshold) {
                active = dval > alarm.threshold;
              } else {
                active = dval < alarm.threshold;
              }
            }
          },
          it->second);

      auto & prev_state = alarm_states_[alarm.fault_code];
      snapshot_.alarms[alarm.fault_code] = active;

      if (active != prev_state) {
        prev_state = active;
        changes.push_back({alarm.fault_code, alarm, active});
      }
    }
  }  // snapshot_mutex_ released

  // Fire callbacks outside of locks
  if (!changes.empty()) {
    std::lock_guard<std::mutex> alarm_lock(alarm_mutex_);
    if (alarm_callback_) {
      for (const auto & c : changes) {
        alarm_callback_(c.fault_code, c.config, c.active);
      }
    }
  }
}

}  // namespace ros2_medkit_gateway
