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

#pragma once

#include "ros2_medkit_opcua/node_map.hpp"
#include "ros2_medkit_opcua/opcua_client.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>

namespace ros2_medkit_gateway {

/// Snapshot of all polled values at a point in time
struct PollSnapshot {
  std::chrono::system_clock::time_point timestamp;
  std::unordered_map<std::string, OpcuaValue> values;  // node_id_str -> value
  std::unordered_map<std::string, bool> alarms;        // fault_code -> active
  bool connected{false};
  uint64_t poll_count{0};
  uint64_t error_count{0};
};

/// Callback when an alarm state changes (for fault reporting)
using AlarmChangeCallback =
    std::function<void(const std::string & fault_code, const AlarmConfig & config, bool active)>;

/// Callback after each poll cycle (for publishing values to ROS 2 topics)
using PollCallback = std::function<void(const PollSnapshot & snapshot)>;

/// Configuration for the poller
struct PollerConfig {
  bool prefer_subscriptions{false};  // poll mode by default (subscriptions need event loop)
  double subscription_interval_ms{500.0};
  std::chrono::milliseconds poll_interval{1000};
  std::chrono::milliseconds reconnect_interval{5000};
};

/// Manages OPC-UA data collection via subscriptions (preferred) or polling
class OpcuaPoller {
 public:
  OpcuaPoller(OpcuaClient & client, const NodeMap & node_map);
  ~OpcuaPoller();

  OpcuaPoller(const OpcuaPoller &) = delete;
  OpcuaPoller & operator=(const OpcuaPoller &) = delete;

  /// Start the poller thread
  void start(const PollerConfig & config);

  /// Stop the poller thread
  void stop();

  /// Get current snapshot (thread-safe copy)
  PollSnapshot snapshot() const;

  /// Get value for a specific node (thread-safe)
  std::optional<OpcuaValue> get_value(const std::string & node_id_str) const;

  /// Set callback for alarm state changes
  void set_alarm_callback(AlarmChangeCallback callback);

  /// Set callback fired after each poll cycle (for value bridging)
  void set_poll_callback(PollCallback callback);

  /// Check if using subscriptions (vs polling)
  bool using_subscriptions() const {
    return using_subscriptions_.load();
  }

 private:
  void poll_loop();
  void do_poll();
  void setup_subscriptions();
  void evaluate_alarms();
  void on_data_change(const std::string & node_id, const OpcuaValue & value);

  OpcuaClient & client_;
  const NodeMap & node_map_;
  PollerConfig config_;

  std::thread poll_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> using_subscriptions_{false};

  std::mutex stop_mutex_;
  std::condition_variable stop_cv_;

  mutable std::mutex snapshot_mutex_;
  PollSnapshot snapshot_;

  mutable std::mutex alarm_mutex_;
  AlarmChangeCallback alarm_callback_;
  std::unordered_map<std::string, bool> alarm_states_;  // fault_code -> last known state

  // Thread safety: must be set via set_poll_callback() before start().
  // Not modified after start(), so safe to read from the poll thread without a mutex.
  PollCallback poll_callback_;
};

}  // namespace ros2_medkit_gateway
